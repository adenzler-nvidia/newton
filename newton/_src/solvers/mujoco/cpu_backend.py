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


class _HostArrays:
    """Host access to Warp arrays, without a copy for arrays on the CPU device."""

    def __init__(self):
        # Views of CPU arrays, which wp.array.numpy() takes several microseconds to create.
        self._views: dict[int, tuple[wp.array, np.ndarray]] = {}

    def read(self, array: wp.array) -> np.ndarray:
        """Return the array's values, which alias the array on the CPU device."""
        if not array.device.is_cpu:
            return array.numpy()
        cached = self._views.get(id(array))
        if cached is None:
            cached = self._views[id(array)] = (array, array.numpy())
        return cached[1]

    def write(self, array: wp.array, values: np.ndarray) -> None:
        """Assign host values to the array."""
        if array.device.is_cpu:
            view = self.read(array)
            view[...] = values.reshape(view.shape)
        else:
            array.assign(values.reshape(*array.shape, *getattr(array.dtype, "_shape_", ())))


class MuJoCoCPUBackend:
    """One native ``MjModel`` per world, run by a ``mujoco.batch.Batch`` on a thread pool.

    Newton's conversion kernels write the MuJoCo Warp model and data on both backends.
    This class copies the fields they write to the native worlds, runs MuJoCo's C
    functions on them, and copies the results back, so that only the simulation itself
    differs between the backends.

    The batch owns every world's state and steps it on that world's model, so writes to
    the models take effect. The solver's ``mj_data`` is a copy of the first world's data.
    """

    def __init__(
        self, mujoco: Any, mj_model: Any, mj_data: Any, mjw_model: Any, mjw_data: Any, nworld: int, nthread: int
    ):
        from mujoco import batch

        self._mujoco = mujoco
        self._mjw_model = mjw_model
        self._mjw_data = mjw_data
        self._host = _HostArrays()
        self.models = [mj_model] + [copy.copy(mj_model) for _ in range(nworld - 1)]
        """Native model of each world; the first is the solver's ``mj_model``."""
        # One MjData per world, which contact and constraint export reads.
        self.batch = batch.Batch(mj_model, nworld, nthread, persistent=True)
        """The simulations, which run on :attr:`models`; state fields are bound per world."""
        self.batch.set_models(self.models)
        state = np.empty(self.batch.nstate)
        mujoco.mj_getState(mj_model, mj_data, state, mujoco.mjtState.mjSTATE_INTEGRATION)
        self.batch.state[:] = state
        # Copies of each world's data for MuJoCo functions the batch does not run; the first is the solver's.
        self._data = [mj_data]
        self._data_stale = False
        self._scratch: Any = None
        self._options = [model.opt for model in self.models]
        self._model_views: dict[str, list[np.ndarray]] = {}
        self._timestep: float | None = None
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
            for option in self._options:
                option.timestep = dt
            self._timestep = dt

    def step(self) -> None:
        """Advance every world by one timestep."""
        # The batch requires its models to be set again after they were edited.
        self.batch.set_models(self.models)
        self.batch.step()
        self._refresh_data()

    def forward(self) -> None:
        """Run the forward dynamics of every world; a following :meth:`step` continues from them."""
        self.batch.set_models(self.models)
        self.batch.forward()
        self._refresh_data()

    def set_const(self) -> None:
        """Recompute every world's model constants and read the ones Newton's kernels use."""
        self.flush()
        if self._scratch is None:
            self._scratch = self._mujoco.MjData(self.models[0])
        for model in self.models:
            self._mujoco.mj_setConst(model, self._scratch)
        self.pull_model(_SET_CONST_FIELDS)

    def reset(self, worlds: Sequence[int] | None = None) -> None:
        """Clear the buffers that persist between steps in the selected worlds."""
        for name in _RESET_FIELDS:
            self.batch.bind(name)[slice(None) if worlds is None else worlds] = 0.0
        self._data_stale = True

    def refresh(self) -> None:
        """Update the solver's ``mj_data`` after writes to the first world's state."""
        if self._data_stale:
            self._refresh_data()

    def _refresh_data(self) -> None:
        self.batch.get_data(0, self._data[0])
        self._data_stale = False

    def worlds(self) -> list[tuple[Any, Any]]:
        """Return every world's model and a copy of its data, which the next step outdates."""
        for _ in range(len(self._data), self.nworld):
            self._data.append(self._mujoco.MjData(self.models[0]))
        for world in range(self.nworld):
            self.batch.get_data(world, self._data[world])
        self._data_stale = False
        return list(zip(self.models, self._data, strict=True))

    # ------------------------------------------------------------------
    # Mirroring
    # ------------------------------------------------------------------

    def _views(self, name: str) -> list[np.ndarray]:
        views = self._model_views.get(name)
        if views is None:
            views = self._model_views[name] = [getattr(model, name) for model in self.models]
        return views

    def push_model(self, names: Iterable[str]) -> None:
        """Copy the entries of MuJoCo Warp model fields that differ from the native models.

        Native entries that already round to the MuJoCo Warp value are left alone, so
        constants that MuJoCo computed in double precision survive unrelated updates.
        """
        for name in names:
            views = self._views(name)
            if not views[0].size:
                continue
            # Fields without a world dimension, or with a single entry, are shared by all worlds.
            values = self._host.read(getattr(self._mjw_model, name)).reshape(-1, *views[0].shape)
            changed = np.stack(views).astype(values.dtype) != values
            mask = self.model_field_masks.get(name)
            if mask is not None:
                changed &= mask
            shared = values.shape[0] == 1
            for world in np.flatnonzero(changed.reshape(self.nworld, -1).any(axis=1)):
                np.copyto(views[world], values[0 if shared else world], where=changed[world])

    def pull_model(self, names: Iterable[str]) -> None:
        """Copy native model fields of every world to the MuJoCo Warp model."""
        for name in names:
            views = self._views(name)
            if views[0].size:
                target = getattr(self._mjw_model, name)
                # Fields without a world dimension keep the first world's values.
                shared = target.size * wp.types.type_size(target.dtype) == views[0].size
                self._host.write(target, views[0] if shared else np.stack(views))

    def push_data(self, names: Iterable[str], worlds: Sequence[int] | None = None) -> None:
        """Copy MuJoCo Warp data fields to the native state of the selected worlds."""
        rows = slice(None) if worlds is None else worlds
        for name in names:
            native = self.batch.bind(name)
            if native.size:
                native[rows] = self._host.read(getattr(self._mjw_data, name)).reshape(native.shape)[rows]
        self._data_stale = True

    def pull_data(self, names: Iterable[str]) -> None:
        """Copy native data fields of every world to the MuJoCo Warp data."""
        for name in names:
            native = self.batch.bind(name)
            if native.size:
                self._host.write(getattr(self._mjw_data, name), native)

    def pull_rne_postconstraint(self) -> None:
        """Compute the body accelerations and interaction forces, which MuJoCo skips without sensors that use them."""
        worlds = self.worlds()
        for model, data in worlds:
            self._mujoco.mj_rnePostConstraint(model, data)
        for name in ("cacc", "cfrc_int"):
            self._host.write(getattr(self._mjw_data, name), np.stack([getattr(data, name) for _, data in worlds]))

    def set_options(self, options: dict[str, np.ndarray]) -> None:
        """Set ``MjOption`` fields from one value per world, or a single value shared by all worlds."""
        for name, values in options.items():
            for world, option in enumerate(self._options):
                value = values[world % values.shape[0]]
                if value.ndim:
                    getattr(option, name)[:] = value
                # A non-positive impratio keeps MuJoCo's value, as on MuJoCo Warp.
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
        return np.stack(self._views(name))

    # ------------------------------------------------------------------
    # Native computations without a MuJoCo Warp array to mirror
    # ------------------------------------------------------------------

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
        if self._scratch is None:
            self._scratch = mujoco.MjData(self.models[0])
        d = self._scratch
        for m, ratio in zip(self.models, dampratio, strict=True):
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
                m.actuator_biasprm[actuator, 2] = (
                    -ratio[actuator] * 2.0 * np.sqrt(m.actuator_gainprm[actuator, 0] * inertia)
                )
        self.pull_model(("actuator_biasprm",))

    def frictionloss_force(self) -> np.ndarray:
        """Return the DOF friction-loss constraint forces of every world, shape ``[nworld, nv]``."""
        friction_dof = self._mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF
        force = np.zeros((self.nworld, self.models[0].nv), dtype=np.float32)
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
