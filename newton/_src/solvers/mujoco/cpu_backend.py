# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

"""Native MuJoCo worlds that mirror the batched MuJoCo Warp model and data."""

from __future__ import annotations

import copy
from collections.abc import Iterable, Sequence
from types import SimpleNamespace
from typing import Any

import numpy as np
import warp as wp

from ...core.types import vec5
from .constants import MJ_MINVAL

# Derived MjModel constants that mj_setConst recomputes and Newton's kernels read back.
_SET_CONST_FIELDS = (
    "body_subtreemass",
    "body_invweight0",
    "dof_invweight0",
    "tendon_invweight0",
    "tendon_length0",
    "actuator_acc0",
    "eq_data",
)

# MjData buffers that persist between steps and must not outlive a reset.
_RESET_FIELDS = ("qacc_warmstart", "qfrc_applied", "xfrc_applied", "ctrl", "act")


def _numpy_shape(array: wp.array) -> tuple[int, ...]:
    return (*array.shape, *getattr(array.dtype, "_shape_", ()))


def _assign(dst: wp.array, values: np.ndarray) -> None:
    """Write host values to a Warp array on any device."""
    if dst.device.is_cpu:
        view = dst.numpy()
        view[...] = values.reshape(view.shape)
    else:
        dst.assign(values.reshape(_numpy_shape(dst)))


def _quat_to_mat(quat: np.ndarray) -> np.ndarray:
    """Convert unit quaternions ``[..., 4]`` (wxyz) to rotation matrices ``[..., 3, 3]``."""
    w, x, y, z = np.moveaxis(quat, -1, 0)
    mat = np.empty((*quat.shape[:-1], 3, 3))
    mat[..., 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    mat[..., 0, 1] = 2.0 * (x * y - w * z)
    mat[..., 0, 2] = 2.0 * (x * z + w * y)
    mat[..., 1, 0] = 2.0 * (x * y + w * z)
    mat[..., 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    mat[..., 1, 2] = 2.0 * (y * z - w * x)
    mat[..., 2, 0] = 2.0 * (x * z - w * y)
    mat[..., 2, 1] = 2.0 * (y * z + w * x)
    mat[..., 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return mat


class _BodyBVH:
    """Topology of the compiled per-body midphase trees, which is the same in every world."""

    def __init__(self, m: Any):
        bodies = np.flatnonzero(m.body_bvhnum)
        counts = m.body_bvhnum[bodies]
        node_body = np.repeat(bodies, counts)
        # Each body's nodes are contiguous; children are indexed within their body's tree.
        first = np.cumsum(counts) - counts
        local = np.arange(node_body.size) - np.repeat(first, counts)
        self.nodes = np.repeat(m.body_bvhadr[bodies], counts) + local
        nodeid = m.bvh_nodeid[self.nodes]
        leaf = nodeid >= 0
        self.leaves = np.flatnonzero(leaf)
        self.leaf_geom = nodeid[leaf]
        self.leaf_body = node_body[leaf]
        offset = np.repeat(first, counts)
        child = m.bvh_child[self.nodes]
        self.child = np.where(child >= 0, child + offset[:, None], -1)
        depth = m.bvh_depth[self.nodes]
        internal = np.flatnonzero(~leaf)
        # Deepest first, so every node is refit after its children.
        self.levels = [internal[depth[internal] == d] for d in np.unique(depth[internal])[::-1]]

    def boxes(
        self,
        geom_pos: np.ndarray,
        geom_quat: np.ndarray,
        geom_aabb: np.ndarray,
        body_ipos: np.ndarray,
        body_iquat: np.ndarray,
    ) -> np.ndarray:
        """Return the midphase boxes ``[nworld, node, 6]`` of geom bounds and poses in the body inertial frames.

        ``mj_collideTree`` assumes these boxes, which MuJoCo only computes when compiling.
        """
        geom, body = self.leaf_geom, self.leaf_body
        imat = _quat_to_mat(body_iquat[:, body])
        gmat = _quat_to_mat(geom_quat[:, geom])
        aabb = geom_aabb[:, geom]
        center = geom_pos[:, geom] + np.einsum("wnij,wnj->wni", gmat, aabb[..., :3]) - body_ipos[:, body]
        center = np.einsum("wnji,wnj->wni", imat, center)
        half = np.einsum("wnij,wnj->wni", np.abs(np.einsum("wnji,wnjk->wnik", imat, gmat)), aabb[..., 3:])
        lower = np.full((geom_pos.shape[0], self.nodes.size, 3), np.inf)
        upper = np.full((geom_pos.shape[0], self.nodes.size, 3), -np.inf)
        lower[:, self.leaves] = center - half
        upper[:, self.leaves] = center + half
        for nodes in self.levels:
            for child in self.child[nodes].T:
                valid = child >= 0
                lower[:, nodes[valid]] = np.minimum(lower[:, nodes[valid]], lower[:, child[valid]])
                upper[:, nodes[valid]] = np.maximum(upper[:, nodes[valid]], upper[:, child[valid]])
        return np.concatenate((0.5 * (lower + upper), 0.5 * (upper - lower)), axis=-1)


class MuJoCoCPUBackend:
    """Native MuJoCo worlds in a ``mujoco.batch.Batch``, which steps them on a thread pool.

    Newton's conversion kernels write the MuJoCo Warp model and data on both backends.
    This class copies the fields they write to the batch, runs MuJoCo's C functions on
    it, and copies the results back, so that only the simulation itself differs between
    the backends.

    The batch owns the simulations. The solver's ``mj_model`` and ``mj_data`` are copies
    of the first world that are refreshed after every change; writes to them are lost.
    """

    def __init__(
        self, mujoco: Any, mj_model: Any, mj_data: Any, mjw_model: Any, mjw_data: Any, nworld: int, nthread: int
    ):
        from mujoco import batch

        self._mujoco = mujoco
        self._mjw_model = mjw_model
        self._mjw_data = mjw_data
        # One MjData per world, which contact and constraint export reads.
        self.batch = batch.Batch(mj_model, nworld, nthread, persistent=True)
        """The simulations; state fields are bound and model fields expanded per world."""
        state = np.empty(self.batch.nstate)
        mujoco.mj_getState(mj_model, mj_data, state, mujoco.mjtState.mjSTATE_INTEGRATION)
        self.batch.state[:] = state
        # Copies of each world's model and data for MuJoCo functions the batch does not run.
        self._models = [mj_model]
        self._data = [mj_data]
        self._models_stale = False
        self._timestep: float | None = None
        self._bvh = _BodyBVH(mj_model)
        # Newton only defines the size components that a geom's collision shape uses. MuJoCo compiles
        # the others for rendering, such as a plane's extent and grid spacing, or from mesh bounds.
        geom = mujoco.mjtGeom
        size_count = {geom.mjGEOM_SPHERE: 1, geom.mjGEOM_CAPSULE: 2, geom.mjGEOM_CYLINDER: 2}
        size_count |= {geom.mjGEOM_ELLIPSOID: 3, geom.mjGEOM_BOX: 3}
        used = np.array([size_count.get(geom(int(t)), 0) for t in mj_model.geom_type], dtype=int)
        self.model_field_masks = {"geom_size": np.arange(3) < used[:, None]}
        """Entries of a model field that :meth:`push_model` copies, for fields it copies only in part."""

        self.dirty_model_fields: set[str] = set()
        """MuJoCo Warp model fields written since the last :meth:`flush`."""
        self.dirty_data_fields: set[str] = set()
        """MuJoCo Warp data fields written since the last :meth:`flush`."""

    @property
    def nworld(self) -> int:
        return self.batch.nsim

    @property
    def nthread(self) -> int:
        return self.batch.nthread

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    def set_timestep(self, dt: float) -> None:
        if dt != self._timestep:
            self.batch.expand("opt.timestep")[:] = dt
            self._timestep = dt
            self._model_changed()

    def step(self) -> None:
        """Advance every world by one timestep."""
        self.batch.step()
        self._data_changed()

    def forward(self) -> None:
        """Run the forward dynamics of every world; a following :meth:`step` continues from them."""
        self.batch.forward()
        self._data_changed()

    def set_const(self) -> None:
        """Recompute every world's model constants and read the ones Newton's kernels use."""
        self.flush()
        self.batch.set_const()
        self._model_changed()
        self.pull_model(_SET_CONST_FIELDS)

    def reset(self, worlds: Sequence[int] | None = None) -> None:
        """Clear the buffers that persist between steps in the selected worlds."""
        for name in _RESET_FIELDS:
            self.batch.bind(name)[slice(None) if worlds is None else worlds] = 0.0
        self._data_changed()

    def worlds(self) -> list[tuple[Any, Any]]:
        """Return a copy of every world's model and data, which the next change to the batch outdates."""
        for _ in range(len(self._models), self.nworld):
            self._models.append(copy.copy(self._models[0]))
            self._data.append(self._mujoco.MjData(self._models[0]))
            self._models_stale = True
        if self._models_stale:
            for world in range(1, self.nworld):
                self.batch.get_model(world, self._models[world])
            self._models_stale = False
        for world in range(1, self.nworld):
            self.batch.get_data(world, self._data[world])
        return list(zip(self._models, self._data, strict=True))

    def _model_changed(self) -> None:
        self.batch.get_model(0, self._models[0])
        self._models_stale = True

    def _data_changed(self) -> None:
        self.batch.get_data(0, self._data[0])

    # ------------------------------------------------------------------
    # Mirroring
    # ------------------------------------------------------------------

    def push_model(self, names: Iterable[str]) -> None:
        """Copy the entries of MuJoCo Warp model fields that differ from the native models.

        Native entries that already round to the MuJoCo Warp value are left alone, so
        constants that MuJoCo computed in double precision survive unrelated updates.
        """
        for name in names:
            native = self.batch.expand(name)
            if not native.size:
                continue
            # Fields without a world dimension, or with a single entry, are shared by all worlds.
            values = np.broadcast_to(
                getattr(self._mjw_model, name).numpy().reshape(-1, *native.shape[1:]), native.shape
            )
            changed = native.astype(values.dtype) != values
            mask = self.model_field_masks.get(name)
            if mask is not None:
                changed &= mask
            native[changed] = values[changed]
        self._model_changed()

    def pull_model(self, names: Iterable[str]) -> None:
        """Copy native model fields of every world to the MuJoCo Warp model."""
        for name in names:
            native = self.model_field(name)
            if native.size:
                target = getattr(self._mjw_model, name)
                # Fields without a world dimension keep the first world's values.
                shared = int(np.prod(_numpy_shape(target))) == native[0].size
                _assign(target, native[0] if shared else native)

    def push_data(self, names: Iterable[str], worlds: Sequence[int] | None = None) -> None:
        """Copy MuJoCo Warp data fields to the native state of the selected worlds."""
        rows = slice(None) if worlds is None else worlds
        for name in names:
            native = self.batch.bind(name)
            if native.size:
                native[rows] = getattr(self._mjw_data, name).numpy().reshape(native.shape)[rows]
        self._data_changed()

    def pull_data(self, names: Iterable[str]) -> None:
        """Copy native data fields of every world to the MuJoCo Warp data."""
        for name in names:
            native = self.batch.bind(name)
            if native.size:
                _assign(getattr(self._mjw_data, name), native)

    def pull_rne_postconstraint(self) -> None:
        """Compute the body accelerations and interaction forces, which MuJoCo skips without sensors that use them."""
        worlds = self.worlds()
        for model, data in worlds:
            self._mujoco.mj_rnePostConstraint(model, data)
        for name in ("cacc", "cfrc_int"):
            _assign(getattr(self._mjw_data, name), np.stack([getattr(data, name) for _, data in worlds]))

    def set_options(self, options: dict[str, np.ndarray]) -> None:
        """Set ``MjOption`` fields from one value per world, or a single value shared by all worlds."""
        for name, option in options.items():
            native = self.batch.expand(f"opt.{name}")
            values = np.broadcast_to(option, native.shape)
            # A non-positive impratio keeps MuJoCo's value, as on MuJoCo Warp.
            write = values > 0.0 if name == "impratio" else np.ones(native.shape, dtype=bool)
            native[write] = values[write]
        self._model_changed()

    def flush(self) -> None:
        """Copy the MuJoCo Warp fields written since the last call to the native worlds."""
        if self.dirty_model_fields:
            self.push_model(self.dirty_model_fields)
            self.dirty_model_fields.clear()
        if self.dirty_data_fields:
            self.push_data(self.dirty_data_fields)
            self.dirty_data_fields.clear()

    def model_field(self, name: str) -> np.ndarray:
        """Return a native model field of every world, shape ``[nworld, ...]``."""
        native = self.batch.expanded(name)
        if native is None:
            template = getattr(self._models[0], name)
            native = np.broadcast_to(template, (self.nworld, *template.shape))
        return native

    # ------------------------------------------------------------------
    # Native computations without a MuJoCo Warp array to mirror
    # ------------------------------------------------------------------

    def refit_body_bvh(self) -> None:
        """Refit every world's midphase boxes after geom bounds, geom poses, or inertial frames changed."""
        tree = self._bvh
        if tree.nodes.size:
            fields = ("geom_pos", "geom_quat", "geom_aabb", "body_ipos", "body_iquat")
            self.batch.expand("bvh_aabb")[:, tree.nodes] = tree.boxes(*map(self.model_field, fields))
            self._model_changed()

    def resolve_dampratio(self, actuators: Sequence[int], dampratio: np.ndarray) -> None:
        """Resolve ``dampratio`` placeholders into ``actuator_biasprm`` from each world's inertia at ``qpos0``.

        Args:
            actuators: MuJoCo actuators to resolve.
            dampratio: Damping ratio per world and actuator, shape ``[nworld or 1, nu]``; non-positive entries are skipped.
        """
        mujoco = self._mujoco
        dampratio = np.broadcast_to(dampratio, (self.nworld, dampratio.shape[1]))
        if not np.any(dampratio[:, list(actuators)] > 0.0):
            return
        self.flush()
        biasprm = self.batch.expand("actuator_biasprm")
        for world, (m, d) in enumerate(self.worlds()):
            ratio = dampratio[world]
            selected = [a for a in actuators if ratio[a] > 0.0]
            if not selected:
                continue
            d.qpos[:] = m.qpos0
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            mujoco.mj_tendon(m, d)
            mujoco.mj_makeM(m, d)
            mujoco.mj_factorM(m, d)
            mujoco.mj_transmission(m, d)
            moment = np.zeros((1, m.nv))
            inv_mass_moment = np.zeros((1, m.nv))
            for actuator in selected:
                # Average over the actuator's force outputs, like MuJoCo's compiler.
                outputs = range(m.actuator_outadr[actuator], m.actuator_outadr[actuator] + m.actuator_outnum[actuator])
                inv_weight = 0.0
                for row in outputs:
                    adr, nnz = d.moment_rowadr[row], d.moment_rownnz[row]
                    moment[:] = 0.0
                    moment[0, d.moment_colind[adr : adr + nnz]] = d.actuator_moment[adr : adr + nnz]
                    mujoco.mj_solveM(m, d, inv_mass_moment, moment)
                    inv_weight += float(moment[0] @ inv_mass_moment[0]) / len(outputs)
                inertia = 1.0 / inv_weight if inv_weight > MJ_MINVAL else 0.0
                biasprm[world, actuator, 2] = (
                    -ratio[actuator] * 2.0 * np.sqrt(m.actuator_gainprm[actuator, 0] * inertia)
                )
        self._model_changed()
        self._data_changed()
        self.pull_model(("actuator_biasprm",))

    def frictionloss_force(self) -> np.ndarray:
        """Return the DOF friction-loss constraint forces of every world, shape ``[nworld, nv]``."""
        friction_dof = self._mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF
        force = np.zeros((self.nworld, self._models[0].nv), dtype=np.float32)
        for world, (_, data) in enumerate(self.worlds()):
            rows = data.efc_type == friction_dof
            force[world, data.efc_id[rows]] = data.efc_force[rows]
        return force

    def contact_count(self) -> int:
        """Return the number of contacts over all worlds."""
        return int(self.batch.bind("ncon").sum())

    def contact_data(self, count: int, device: wp.Device) -> SimpleNamespace:
        """Return the first ``count`` contacts over all worlds in the layout of MuJoCo Warp's data.

        The result has the ``nacon``, ``naconmax``, ``njmax``, ``contact``, and ``efc.force``
        members that contact export reads from MuJoCo Warp's data.
        """
        worlds = self.worlds()
        nefc = max(data.nefc for _, data in worlds)
        pos = np.empty((count, 3), dtype=np.float32)
        frame = np.empty((count, 3, 3), dtype=np.float32)
        friction = np.empty((count, 5), dtype=np.float32)
        dist = np.empty(count, dtype=np.float32)
        dim = np.empty(count, dtype=np.int32)
        geom = np.empty((count, 2), dtype=np.int32)
        efc_address = np.empty(count, dtype=np.int32)
        worldid = np.empty(count, dtype=np.int32)
        efc_force = np.zeros((self.nworld, max(nefc, 1)), dtype=np.float32)
        start = 0
        for world, (_, data) in enumerate(worlds):
            efc_force[world, : data.nefc] = data.efc_force
            n = min(data.ncon, count - start)
            if n <= 0:
                continue
            contact, rows = data.contact, slice(start, start + n)
            pos[rows] = contact.pos[:n]
            frame[rows] = contact.frame[:n].reshape(n, 3, 3)
            friction[rows] = contact.friction[:n]
            dist[rows] = contact.dist[:n]
            dim[rows] = contact.dim[:n]
            geom[rows] = contact.geom[:n]
            efc_address[rows] = contact.efc_address[:n]
            worldid[rows] = world
            start += n
        # A native contact's constraint rows are contiguous; MuJoCo Warp addresses each row.
        rows_per_contact = self._mjw_data.contact.efc_address.shape[1]
        efc_rows = np.where(efc_address[:, None] >= 0, efc_address[:, None] + np.arange(rows_per_contact), -1)
        return SimpleNamespace(
            nacon=wp.array([count], dtype=wp.int32, device=device),
            naconmax=count,
            njmax=efc_force.shape[1],
            contact=SimpleNamespace(
                pos=wp.array(pos, dtype=wp.vec3, device=device),
                frame=wp.array(frame, dtype=wp.mat33, device=device),
                friction=wp.array(friction, dtype=vec5, device=device),
                dist=wp.array(dist, dtype=float, device=device),
                dim=wp.array(dim, dtype=int, device=device),
                geom=wp.array(geom, dtype=wp.vec2i, device=device),
                efc_address=wp.array(efc_rows.astype(np.int32), dtype=int, device=device),
                worldid=wp.array(worldid, dtype=wp.int32, device=device),
                adhesion=wp.zeros(count, dtype=float, device=device),
            ),
            efc=SimpleNamespace(force=wp.array(efc_force, dtype=float, device=device)),
        )
