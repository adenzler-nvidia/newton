# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

"""Native MuJoCo worlds that mirror the batched MuJoCo Warp model and data."""

from __future__ import annotations

import copy
import os
from collections.abc import Callable, Iterable, Sequence
from concurrent.futures import ThreadPoolExecutor
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


def default_thread_count(world_count: int, nv: int) -> int:
    """Return the number of threads used to step native MuJoCo worlds with ``nv`` degrees of freedom.

    The threads hand over Python's global interpreter lock after every world, which costs
    more than stepping a small world. Stepping 256 worlds on 16 cores was fastest with 4,
    8, and 16 threads for worlds with 14, 27, and 81 degrees of freedom.
    """
    try:
        cpus = len(os.sched_getaffinity(0))
    except AttributeError:
        cpus = os.cpu_count() or 1
    # Half the logical CPUs, since two hardware threads of one core do not step two worlds faster.
    return max(1, min(world_count, cpus // 2, nv // 3))


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

    def refit(self, models: Sequence[Any]) -> None:
        """Rebuild the midphase boxes of every model from its geom bounds and poses in the body inertial frames.

        ``mj_collideTree`` assumes these boxes, which MuJoCo only computes when compiling.
        """
        if not self.nodes.size:
            return
        geom, body = self.leaf_geom, self.leaf_body

        def field(name: str, index: np.ndarray) -> np.ndarray:
            return np.stack([getattr(m, name) for m in models])[:, index]

        imat = _quat_to_mat(field("body_iquat", body))
        gmat = _quat_to_mat(field("geom_quat", geom))
        aabb = field("geom_aabb", geom)
        center = field("geom_pos", geom) + np.einsum("wnij,wnj->wni", gmat, aabb[..., :3]) - field("body_ipos", body)
        center = np.einsum("wnji,wnj->wni", imat, center)
        half = np.einsum("wnij,wnj->wni", np.abs(np.einsum("wnji,wnjk->wnik", imat, gmat)), aabb[..., 3:])
        lower = np.full((len(models), self.nodes.size, 3), np.inf)
        upper = np.full((len(models), self.nodes.size, 3), -np.inf)
        lower[:, self.leaves] = center - half
        upper[:, self.leaves] = center + half
        for nodes in self.levels:
            for child in self.child[nodes].T:
                valid = child >= 0
                lower[:, nodes[valid]] = np.minimum(lower[:, nodes[valid]], lower[:, child[valid]])
                upper[:, nodes[valid]] = np.maximum(upper[:, nodes[valid]], upper[:, child[valid]])
        boxes = np.concatenate((0.5 * (lower + upper), 0.5 * (upper - lower)), axis=-1)
        for m, box in zip(models, boxes, strict=True):
            m.bvh_aabb[self.nodes] = box


class MuJoCoCPUBackend:
    """One native ``MjModel`` and ``MjData`` per world, run on a thread pool.

    Newton's conversion kernels write the MuJoCo Warp model and data on both backends.
    This class copies the fields they write to the native worlds, runs MuJoCo's C
    functions on them, and copies the results back, so that only the simulation itself
    differs between the backends.
    """

    def __init__(
        self, mujoco: Any, mj_model: Any, mj_data: Any, mjw_model: Any, mjw_data: Any, nworld: int, nthread: int
    ):
        self._mujoco = mujoco
        self._mjw_model = mjw_model
        self._mjw_data = mjw_data
        self.nworld = nworld
        self.models = [mj_model] + [copy.copy(mj_model) for _ in range(nworld - 1)]
        """Native model of each world; the first is the solver's ``mj_model``."""
        self.data = [mj_data]
        """Native data of each world; the first is the solver's ``mj_data``."""
        for model in self.models[1:]:
            data = mujoco.MjData(model)
            mujoco.mj_copyData(data, model, mj_data)
            self.data.append(data)
        self._worlds = list(zip(self.models, self.data, strict=True))
        self._options = [model.opt for model in self.models]
        self._model_views: dict[str, list[np.ndarray]] = {}
        self._data_views: dict[str, list[np.ndarray]] = {}
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

        self.nthread = max(1, min(nthread, nworld))
        # The calling thread runs one chunk itself.
        self._pool = (
            ThreadPoolExecutor(self.nthread - 1, thread_name_prefix="newton-mujoco") if self.nthread > 1 else None
        )
        bounds = np.linspace(0, nworld, self.nthread + 1).astype(int)
        self._chunks = [self._worlds[bounds[i] : bounds[i + 1]] for i in range(self.nthread)]

    def close(self) -> None:
        """Stop the worker threads."""
        if self._pool is not None:
            self._pool.shutdown(wait=False)
            self._pool = None

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    @staticmethod
    def _run_chunk(fn: Callable[[Any, Any], None], worlds: Sequence[tuple[Any, Any]]) -> None:
        for model, data in worlds:
            fn(model, data)

    def run(self, fn: Callable[[Any, Any], None]) -> None:
        """Call ``fn(model, data)`` for every world, in parallel across worlds."""
        if self._pool is None:
            self._run_chunk(fn, self._worlds)
            return
        futures = [self._pool.submit(self._run_chunk, fn, chunk) for chunk in self._chunks[1:]]
        try:
            self._run_chunk(fn, self._chunks[0])
        finally:
            # Wait for every world before raising, so no thread outlives the call.
            errors = [future.exception() for future in futures]
        for error in errors:
            if error is not None:
                raise error

    def set_timestep(self, dt: float) -> None:
        if dt != self._timestep:
            for option in self._options:
                option.timestep = dt
            self._timestep = dt

    def step(self) -> None:
        """Advance every world by one timestep."""
        self.run(self._mujoco.mj_step)

    def forward(self, rne_postconstraint: bool) -> None:
        """Run the forward half of ``mj_step``, so results can be read before integrating."""
        mujoco = self._mujoco
        compare_fwd_inv = self.models[0].opt.enableflags & mujoco.mjtEnableBit.mjENBL_FWDINV

        def forward(model, data):
            mujoco.mj_checkPos(model, data)
            mujoco.mj_checkVel(model, data)
            mujoco.mj_forward(model, data)
            mujoco.mj_checkAcc(model, data)
            if compare_fwd_inv:
                mujoco.mj_compareFwdInv(model, data)
            if rne_postconstraint:
                # Populate cacc and cfrc_int independently of MuJoCo sensors.
                mujoco.mj_rnePostConstraint(model, data)

        self.run(forward)

    def integrate(self) -> None:
        """Run the integration half of ``mj_step`` after :meth:`forward`."""
        mujoco = self._mujoco
        integrator = self.models[0].opt.integrator
        if integrator == mujoco.mjtIntegrator.mjINT_EULER:
            self.run(mujoco.mj_Euler)
        elif integrator == mujoco.mjtIntegrator.mjINT_RK4:
            self.run(lambda model, data: mujoco.mj_RungeKutta(model, data, 4))
        else:
            self.run(mujoco.mj_implicit)

    def set_const(self) -> None:
        """Recompute every world's model constants and read the ones Newton's kernels use."""
        self.flush()
        mj_set_const = self._mujoco.mj_setConst

        def set_const(model, data):
            # mj_setConst leaves data at qpos0.
            qpos = data.qpos.copy()
            mj_set_const(model, data)
            data.qpos[:] = qpos

        self.run(set_const)
        self.pull_model(_SET_CONST_FIELDS)

    def reset(self, worlds: Sequence[int] | None = None) -> None:
        """Clear the buffers that persist between steps in the selected worlds."""
        for name in _RESET_FIELDS:
            views = self._views(self._data_views, self.data, name)
            for world in range(self.nworld) if worlds is None else worlds:
                views[world][:] = 0.0

    # ------------------------------------------------------------------
    # Mirroring
    # ------------------------------------------------------------------

    @staticmethod
    def _views(cache: dict[str, list[np.ndarray]], objects: list[Any], name: str) -> list[np.ndarray]:
        views = cache.get(name)
        if views is None:
            views = cache[name] = [getattr(obj, name) for obj in objects]
        return views

    @staticmethod
    def _push(source: wp.array, views: list[np.ndarray], worlds: Sequence[int] | None = None) -> None:
        """Copy a MuJoCo Warp array to the native arrays of the selected worlds."""
        if not views[0].size:
            return
        # Fields without a world dimension, or with a single entry, are shared by all worlds.
        values = source.numpy().reshape(-1, *views[0].shape)
        shared = values.shape[0] == 1
        for world in range(len(views)) if worlds is None else worlds:
            views[world][...] = values[0 if shared else world]

    @staticmethod
    def _push_changed(source: wp.array, views: list[np.ndarray], mask: np.ndarray | None) -> None:
        """Copy the entries of a MuJoCo Warp array that differ from the native arrays.

        Native entries that already round to the source value are left alone, so constants
        that MuJoCo computed in double precision survive unrelated updates.

        Args:
            mask: Entries of one world's array to copy, or ``None`` for all.
        """
        if not views[0].size:
            return
        values = source.numpy().reshape(-1, *views[0].shape)
        changed = np.stack(views).astype(values.dtype) != values
        if mask is not None:
            changed &= mask
        shared = values.shape[0] == 1
        for world in np.flatnonzero(changed.reshape(len(views), -1).any(axis=1)):
            np.copyto(views[world], values[0 if shared else world], where=changed[world])

    @staticmethod
    def _pull(views: list[np.ndarray], target: wp.array) -> None:
        if not views[0].size:
            return
        # Fields without a world dimension keep the first world's values.
        shared = int(np.prod(_numpy_shape(target))) == views[0].size
        _assign(target, views[0] if shared else np.stack(views))

    def push_model(self, names: Iterable[str]) -> None:
        """Copy MuJoCo Warp model fields to every native model."""
        for name in names:
            self._push_changed(
                getattr(self._mjw_model, name),
                self._views(self._model_views, self.models, name),
                self.model_field_masks.get(name),
            )

    def pull_model(self, names: Iterable[str]) -> None:
        """Copy native model fields of every world to the MuJoCo Warp model."""
        for name in names:
            self._pull(self._views(self._model_views, self.models, name), getattr(self._mjw_model, name))

    def push_data(self, names: Iterable[str], worlds: Sequence[int] | None = None) -> None:
        """Copy MuJoCo Warp data fields to the native data of the selected worlds."""
        for name in names:
            self._push(getattr(self._mjw_data, name), self._views(self._data_views, self.data, name), worlds)

    def pull_data(self, names: Iterable[str]) -> None:
        """Copy native data fields of every world to the MuJoCo Warp data."""
        for name in names:
            self._pull(self._views(self._data_views, self.data, name), getattr(self._mjw_data, name))

    def set_options(self, options: dict[str, np.ndarray]) -> None:
        """Set ``MjOption`` fields from one value per world, or a single value shared by all worlds."""
        for name, values in options.items():
            for world, option in enumerate(self._options):
                value = values[world % values.shape[0]]
                if value.ndim:
                    getattr(option, name)[:] = value
                elif name != "impratio" or value > 0.0:
                    setattr(option, name, float(value))

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
        return np.stack(self._views(self._model_views, self.models, name))

    # ------------------------------------------------------------------
    # Native computations without a MuJoCo Warp array to mirror
    # ------------------------------------------------------------------

    def refit_body_bvh(self) -> None:
        """Refit every world's midphase boxes after geom bounds, geom poses, or inertial frames changed."""
        self._bvh.refit(self.models)

    def resolve_dampratio(self, actuators: Sequence[int], dampratio: np.ndarray) -> None:
        """Resolve ``dampratio`` placeholders into ``actuator_biasprm`` from each world's inertia at ``qpos0``.

        Args:
            actuators: MuJoCo actuators to resolve.
            dampratio: Damping ratio per world and actuator, shape ``[nworld or 1, nu]``; non-positive entries are skipped.
        """
        mujoco = self._mujoco
        ratios = dict(
            zip(map(id, self.models), np.broadcast_to(dampratio, (self.nworld, dampratio.shape[1])), strict=True)
        )

        def resolve(m, d):
            ratio = ratios[id(m)]
            selected = [a for a in actuators if ratio[a] > 0.0]
            if not selected:
                return
            qpos = d.qpos.copy()
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
                m.actuator_biasprm[actuator, 2] = (
                    -ratio[actuator] * 2.0 * np.sqrt(m.actuator_gainprm[actuator, 0] * inertia)
                )
            d.qpos[:] = qpos

        self.flush()
        self.run(resolve)
        self.pull_model(("actuator_biasprm",))

    def frictionloss_force(self) -> np.ndarray:
        """Return the DOF friction-loss constraint forces of every world, shape ``[nworld, nv]``."""
        friction_dof = self._mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF
        force = np.zeros((self.nworld, self.models[0].nv), dtype=np.float32)
        for world, data in enumerate(self.data):
            rows = data.efc_type == friction_dof
            force[world, data.efc_id[rows]] = data.efc_force[rows]
        return force

    def contact_count(self) -> int:
        """Return the number of contacts over all worlds."""
        return sum(data.ncon for data in self.data)

    def contact_data(self, count: int, device: wp.Device) -> SimpleNamespace:
        """Return the first ``count`` contacts over all worlds in the layout of MuJoCo Warp's data.

        The result has the ``nacon``, ``naconmax``, ``njmax``, ``contact``, and ``efc.force``
        members that contact export reads from MuJoCo Warp's data.
        """
        nefc = max(data.nefc for data in self.data)
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
        for world, data in enumerate(self.data):
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
