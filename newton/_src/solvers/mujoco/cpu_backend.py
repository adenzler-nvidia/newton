# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

"""Native MuJoCo worlds that mirror the batched MuJoCo Warp model and data."""

from __future__ import annotations

import copy
import ctypes
import weakref
from collections.abc import Callable, Iterable, Sequence
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


@wp.kernel(module="unique")
def _copy_to_native(source: wp.array2d[float], native: wp.array2d[wp.float64]):
    world, i = wp.tid()
    native[world, i] = wp.float64(source[world, i])


@wp.kernel(module="unique")
def _copy_flags_to_native(source: wp.array2d[wp.bool], native: wp.array2d[wp.float64]):
    world, i = wp.tid()
    native[world, i] = wp.where(source[world, i], wp.float64(1.0), wp.float64(0.0))


@wp.kernel(module="unique")
def _copy_from_native(native: wp.array2d[wp.float64], target: wp.array2d[float]):
    world, i = wp.tid()
    target[world, i] = wp.float32(native[world, i])


# Warp defines int(x) and float(x) as cast macros, so the snippet avoids a type followed by a parenthesis.
@wp.func_native("""
typedef void host_function_t(void);
((host_function_t*)function)();
""")
def _call_host_function(function: wp.uint64): ...


@wp.kernel(module="unique")
def _call_host(function: wp.uint64, owner: wp.array[wp.float64]):
    """Call a host function; ``owner`` is unused and makes a graph keep the function alive."""
    _call_host_function(function)


_HOST_FUNCTION = ctypes.CFUNCTYPE(None)


class _Replay:
    """Records a step as kernel launches, so that a Warp CPU graph can replay it without the solver's Python.

    Kernels copy the state between the MuJoCo Warp arrays and the batch, whose memory Warp arrays
    alias, and a kernel calls back into Python to run the batch. Every array given to a kernel
    holds this object, which a graph therefore keeps alive together with the batch.
    """

    def __init__(self, batch: Any, mjw_data: Any):
        self._batch = batch
        self._mjw_data = mjw_data
        self.error: BaseException | None = None
        """Exception of a replayed call, which cannot pass through the kernel that made it."""
        self.ran = False
        """Whether a graph has run the batch since the solver last looked."""
        self._fields: dict[str, tuple[wp.array, wp.array] | None] = {}
        self._owner = self._alias(np.zeros(1), wp.float64)
        self.timestep = self._alias(batch.expand("opt.timestep"), wp.float64)
        replay = weakref.ref(self)
        self._calls = {}
        for name, run in (("step", batch.step), ("forward", batch.forward)):

            def call(run: Callable[[], None] = run) -> None:
                self = replay()
                if self is not None:
                    self._run(run)

            function = _HOST_FUNCTION(call)
            self._calls[name] = (function, wp.uint64(ctypes.cast(function, ctypes.c_void_p).value))

    def _alias(self, values: np.ndarray, dtype: Any, *owners: Any) -> wp.array:
        """Return a Warp array over ``values`` that keeps this object and ``owners`` alive."""
        keep = [self, values, *owners]
        return wp.array(
            ptr=values.__array_interface__["data"][0],
            dtype=dtype,
            shape=values.shape,
            strides=values.strides,
            device="cpu",
            deleter=lambda ptr, size: keep.clear(),
        )

    def _run(self, run: Callable[[], None]) -> None:
        self.ran = True
        if self.error is None:
            try:
                run()
            except BaseException as error:
                self.error = error

    def call(self, name: str) -> None:
        """Record a run of the batch."""
        wp.launch(_call_host, dim=1, inputs=[self._calls[name][1], self._owner], device="cpu")

    def _field(self, name: str) -> tuple[wp.array, wp.array] | None:
        """Return a data field's MuJoCo Warp and native values as arrays of ``[nworld, size]`` scalars."""
        if name not in self._fields:
            native = self._batch.bind(name)
            native = native.reshape(native.shape[0], -1)
            if native.size:
                source = getattr(self._mjw_data, name)
                dtype = wp.bool if source.dtype is wp.bool else wp.float32
                staged = self._alias(source.numpy().reshape(native.shape), dtype, source)
                self._fields[name] = (staged, self._alias(native, wp.float64))
            else:
                self._fields[name] = None
        return self._fields[name]

    def push(self, names: Iterable[str]) -> None:
        """Record copies of MuJoCo Warp data fields to the native state."""
        for name in names:
            arrays = self._field(name)
            if arrays is not None:
                staged, native = arrays
                kernel = _copy_flags_to_native if staged.dtype is wp.bool else _copy_to_native
                wp.launch(kernel, dim=native.shape, inputs=[staged], outputs=[native], device="cpu")

    def pull(self, names: Iterable[str]) -> None:
        """Record copies of native data fields to the MuJoCo Warp data."""
        for name in names:
            arrays = self._field(name)
            if arrays is not None:
                staged, native = arrays
                wp.launch(_copy_from_native, dim=native.shape, inputs=[native], outputs=[staged], device="cpu")


class MuJoCoCPUBackend:
    """Native MuJoCo worlds in a ``mujoco.batch.Batch``, which runs them on a thread pool.

    Newton's conversion kernels write the MuJoCo Warp model and data on both backends.
    This class copies the entries they change to the native worlds, runs MuJoCo's C
    functions on them, and copies the results back, so that only the simulation itself
    differs between the backends.

    The batch holds every model field that Newton sets as one ``[nworld, ...]`` array and
    every world's state as a row of its state array. The solver's ``mj_model`` and
    ``mj_data`` show the first world; writes to them have no effect.
    """

    def __init__(
        self, mujoco: Any, mj_model: Any, mj_data: Any, mjw_model: Any, mjw_data: Any, nworld: int, nthread: int
    ):
        from mujoco import batch

        self._mujoco = mujoco
        self._mjw_model = mjw_model
        self._mjw_data = mjw_data
        self._host = _HostArrays()
        self._device = mjw_data.qpos.device
        self._replay: _Replay | None = None
        self._model = mj_model
        # Persistent: one MjData per world, which contact and constraint export reads, and which fields
        # show as soon as they are bound; otherwise a field bound after a call is zero until the next one.
        self.batch = batch.Batch(mj_model, nworld, nthread, persistent=True)
        """The simulations; state fields are bound and model fields expanded per world."""
        # Copies of each world's model and data for MuJoCo functions the batch does not run; the first are the solver's.
        self._models = [mj_model]
        self._data = [mj_data]
        self._models_stale = False
        state = np.empty(self.batch.nstate)
        mujoco.mj_getState(mj_model, mj_data, state, mujoco.mjtState.mjSTATE_INTEGRATION)
        self.batch.state[:] = state
        self._scratch: Any = None
        self._fields: dict[str, np.ndarray] = {}
        # Worlds whose model changed since its constants were computed, and the fields the constants depend on.
        self._const_stale = np.zeros(nworld, dtype=bool)
        self._const_inputs = frozenset(getattr(mujoco.MjModel, "_setconst_inputs", ()))
        self._timestep: float | None = None
        # Newton only defines the size components that a geom's collision shape uses. MuJoCo compiles
        # the others for rendering, such as a plane's extent and grid spacing, or from mesh bounds.
        geom = mujoco.mjtGeom
        size_count = {geom.mjGEOM_SPHERE: 1, geom.mjGEOM_CAPSULE: 2, geom.mjGEOM_CYLINDER: 2}
        size_count |= {geom.mjGEOM_ELLIPSOID: 3, geom.mjGEOM_BOX: 3}
        used = np.array([size_count.get(geom(int(t)), 0) for t in mj_model.geom_type], dtype=int)
        self.model_field_masks = {"geom_size": np.arange(3) < used[:, None]}
        """Entries of a model field that :meth:`flush` copies, for fields it copies only in part."""

        # MuJoCo Warp fields written since the last flush; model fields map to their values before the write.
        self._model_before: dict[str, np.ndarray] = {}
        self._dirty_data: set[str] = set()

    @property
    def nworld(self) -> int:
        return self.batch.nsim

    @property
    def nthread(self) -> int:
        return self.batch.nthread

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    def _recording(self, recordable: bool = True) -> _Replay | None:
        """Return what records the current operation, if a graph is being captured.

        Args:
            recordable: Whether a graph can replay the operation; if not, capturing it raises.
        """
        if not self._device.is_capturing:
            self._after_replay()
            return None
        if not recordable or not self._device.is_cpu:
            raise RuntimeError(
                "A graph can only replay plain steps of the MuJoCo CPU backend, with the model on the CPU device. "
                "Capture steps without contact export, observables, masked updates, and model changes."
            )
        if self._replay is None:
            self._replay = _Replay(self.batch, self._mjw_data)
        return self._replay

    def _after_replay(self) -> None:
        """Raise the failure of a replayed step and show the first world as the replays left it."""
        replay = self._replay
        if replay is not None and (replay.ran or replay.error is not None):
            replay.ran = False
            error, replay.error = replay.error, None
            # A replay may have used another timestep.
            self._timestep = None
            self._model_changed()
            self.batch.get_data(0, self._data[0])
            if error is not None:
                raise error

    def show_replayed(self) -> None:
        """Update the solver's ``mj_model`` and ``mj_data`` after graph replays, which run without the solver."""
        if not self._device.is_capturing:
            self._after_replay()

    def set_timestep(self, dt: float) -> None:
        replay = self._recording()
        if replay is not None:
            replay.timestep.fill_(dt)
        elif dt != self._timestep:
            self.option("timestep")[:] = dt
            self._timestep = dt
            self._model_changed()

    def step(self) -> None:
        """Advance every world by one timestep, continuing from a :meth:`forward` pass that was just run."""
        replay = self._recording()
        if replay is not None:
            replay.call("step")
            return
        self.batch.step()
        self.batch.get_data(0, self._data[0])

    def forward(self) -> None:
        """Run the forward dynamics of every world."""
        replay = self._recording()
        if replay is not None:
            replay.call("forward")
            return
        self.batch.forward()
        self.batch.get_data(0, self._data[0])

    def _show_first_state(self) -> None:
        """Show the first world's state in the solver's ``mj_data``."""
        integration = self._mujoco.mjtState.mjSTATE_INTEGRATION
        self._mujoco.mj_setState(self._model, self._data[0], self.batch.state[0], integration)

    def set_const(self) -> None:
        """Recompute the model constants of the worlds whose model changed and read the ones Newton's kernels use."""
        self._recording(recordable=False)
        self.flush()
        if self._const_stale.any():
            self.batch.set_const(np.flatnonzero(self._const_stale))
            self._const_stale[:] = False
            self._model_changed()
        self.pull_model(_SET_CONST_FIELDS)

    def _scratch_data(self) -> Any:
        if self._scratch is None:
            self._scratch = self._mujoco.MjData(self._model)
        return self._scratch

    def reset(self, worlds: Sequence[int] | None = None) -> None:
        """Clear the buffers that persist between steps in the selected worlds."""
        self._recording(recordable=False)
        for name in _RESET_FIELDS:
            self.batch.bind(name)[slice(None) if worlds is None else worlds] = 0.0
        self._show_first_state()

    def worlds(self) -> list[tuple[Any, Any]]:
        """Return a copy of every world's model and data, which the next change to the batch outdates."""
        self._recording(recordable=False)
        for _ in range(len(self._models), self.nworld):
            self._models.append(copy.copy(self._model))
            self._data.append(self._mujoco.MjData(self._model))
            self._models_stale = True
        if self._models_stale:
            for world in range(1, self.nworld):
                self.batch.get_model(world, self._models[world])
            self._models_stale = False
        for world in range(1, self.nworld):
            self.batch.get_data(world, self._data[world])
        return list(zip(self._models, self._data, strict=True))

    def _model_changed(self) -> None:
        """Show the first world's model in the solver's ``mj_model``."""
        self.batch.get_model(0, self._model)
        self._models_stale = True

    # ------------------------------------------------------------------
    # Mirroring
    # ------------------------------------------------------------------

    def model_field(self, name: str) -> np.ndarray:
        """Return the native values of a model field of every world, shape ``[nworld, ...]``, which can be written."""
        field = self._fields.get(name)
        if field is None:
            field = self._fields[name] = self.batch.expand(name)
        return field

    def option(self, name: str) -> np.ndarray:
        """Return the native values of an ``MjOption`` field of every world, shape ``[nworld, ...]``."""
        return self.model_field(f"opt.{name}")

    def mark_model(self, names: Iterable[str]) -> None:
        """Note MuJoCo Warp model fields that are about to be written."""
        self._recording(recordable=False)
        for name in names:
            if name not in self._model_before:
                self._model_before[name] = self._host.read(getattr(self._mjw_model, name)).copy()

    def mark_data(self, names: Iterable[str]) -> None:
        """Note MuJoCo Warp data fields that are about to be written."""
        self._dirty_data.update(names)

    def _push_model(self, name: str, before: np.ndarray) -> bool:
        """Copy the entries of a MuJoCo Warp model field that were changed to the native worlds.

        Other native entries are left alone, so constants that MuJoCo computed in double
        precision survive unrelated updates.
        """
        native = self.model_field(name)
        if not native.size:
            return False
        # Fields without a world dimension, or with a single entry, are shared by all worlds.
        values = self._host.read(getattr(self._mjw_model, name)).reshape(-1, *native.shape[1:])
        changed = values != before.reshape(values.shape)
        mask = self.model_field_masks.get(name)
        if mask is not None:
            changed &= mask
        if not changed.any():
            return False
        np.copyto(native, values, where=changed)
        if name in self._const_inputs or not self._const_inputs:
            self._const_stale |= np.broadcast_to(changed, native.shape).reshape(self.nworld, -1).any(axis=1)
        return True

    def pull_model(self, names: Iterable[str]) -> None:
        """Copy native model fields of every world to the MuJoCo Warp model."""
        for name in names:
            native = self.model_field(name)
            if native.size:
                target = getattr(self._mjw_model, name)
                # Fields without a world dimension keep the first world's values.
                shared = target.size * wp.types.type_size(target.dtype) == native[0].size
                self._host.write(target, native[0] if shared else native)

    def push_data(self, names: Iterable[str], worlds: Sequence[int] | None = None) -> None:
        """Copy MuJoCo Warp data fields to the native state of the selected worlds."""
        replay = self._recording(recordable=worlds is None)
        if replay is not None:
            replay.push(names)
            return
        rows = slice(None) if worlds is None else worlds
        for name in names:
            native = self.batch.bind(name)
            if native.size:
                native[rows] = self._host.read(getattr(self._mjw_data, name)).reshape(native.shape)[rows]
        self._show_first_state()

    def pull_data(self, names: Iterable[str]) -> None:
        """Copy native data fields of every world to the MuJoCo Warp data."""
        replay = self._recording()
        if replay is not None:
            replay.pull(names)
            return
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
        self._recording(recordable=False)
        for name, option in options.items():
            current = self.option(name)
            values = np.broadcast_to(option, current.shape)
            changed = values != current
            if name == "impratio":
                # A non-positive impratio keeps MuJoCo's value, as on MuJoCo Warp.
                changed &= values > 0.0
            if not changed.any():
                continue
            self._const_stale |= changed.reshape(self.nworld, -1).any(axis=1)
            np.copyto(current, values, where=changed)
        self._model_changed()

    def flush(self) -> None:
        """Copy the MuJoCo Warp fields written since the last call to the native worlds."""
        if self._model_before:
            changed = [self._push_model(name, before) for name, before in self._model_before.items()]
            self._model_before.clear()
            if any(changed):
                self._model_changed()
        if self._dirty_data:
            self.push_data(self._dirty_data)
            self._dirty_data.clear()

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
        d = self._scratch_data()
        biasprm = self.model_field("actuator_biasprm")
        for world, ((m, _), ratio) in enumerate(zip(self.worlds(), dampratio, strict=True)):
            selected = [a for a in actuators if ratio[a] > 0.0]
            if not selected:
                continue
            # The reference configuration of this world, including its mocap poses.
            mujoco.mj_resetData(m, d)
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
        self.pull_model(("actuator_biasprm",))

    def frictionloss_force(self) -> np.ndarray:
        """Return the DOF friction-loss constraint forces of every world, shape ``[nworld, nv]``."""
        friction_dof = self._mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF
        force = np.zeros((self.nworld, self._model.nv), dtype=np.float32)
        for world, (_, data) in enumerate(self.worlds()):
            rows = data.efc_type == friction_dof
            force[world, data.efc_id[rows]] = data.efc_force[rows]
        return force

    def contact_count(self) -> int:
        """Return the number of contacts over all worlds."""
        self._recording(recordable=False)
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
        adhesion = np.empty(count, dtype=np.float32)
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
            adhesion[rows] = contact.adhesion[:n]
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
                adhesion=wp.array(adhesion, dtype=float, device=device),
            ),
            efc=SimpleNamespace(force=wp.array(efc_force, dtype=float, device=device)),
        )
