"""Resolve every spec name against the compiled model once; everything downstream
indexes through these handles and never by literal name.

`ModuleHandles.drive_sign`: the vendored MPO-700's left/right wheel bodies are mirror
images of each other, so the SAME local drive-joint axis (e.g. `[0, -1, 0]`) points in
OPPOSITE world directions on the left vs. right side once each wheel body's own
(mirrored) orientation is applied -- confirmed by reading `xmat @ jnt_axis` per wheel
[measured: scratch probe, 2026-09-20; fl/bl world axis (0,-1,0), fr/br world axis
(0,+1,0)]. `testbed.control.swerve.SwerveAllocator` is deliberately axis-agnostic
kinematics (its own tests assert the SAME `wheel_radps` sign on every module for a
straight command), so a per-wheel hardware sign correction has to live here, where the
compiled model's actual mirrored geometry is known. `drive_sign` is the sign such that
`ctrl = drive_sign * wheel_radps_logical` (and, reading back, `wheel_radps_logical =
drive_sign * qvel`) makes a positive `wheel_radps_logical` mean "this wheel rolls the
chassis toward +x" for every module, derived from the no-slip rolling relation
`v_contact = v_center + omega x r = 0` with the contact point straight below the wheel
center (`r = (0, 0, -1)`, flat ground, z-up): `v_center_x per unit qvel = -(axis x r)_x`,
sign-only. Without this correction, commanding all four wheels the SwerveAllocator's
uniform way drives left and right sides in opposite senses -- measured: the base crept
~0.04 m in 2 s at a 0.3 m/s command (vs. ~0.6 m/s x 2 s expected) while wheel qvel
tracked the command almost exactly, and ground contact repeatedly dropped to 0-2 of 4
wheels (the opposing torques' reaction kicking the rigid, unsprung chassis under lunar
gravity); with the correction applied, base_vx settles at 0.299 m/s for a 0.3 m/s
command with ncon=4 throughout [measured: scratch probe, 2026-09-20]. See
DECISIONS.md D8."""
from dataclasses import dataclass

import mujoco
import numpy as np

from testbed.sim.compose import Compiled


@dataclass(frozen=True)
class ModuleHandles:
    steer_qpos: int
    steer_dof: int
    drive_qpos: int
    drive_dof: int
    steer_act: int
    drive_act: int
    wheel_body: int
    drive_sign: float          # +1/-1: ctrl/qvel sign that means "roll forward" for THIS wheel body


@dataclass(frozen=True)
class Handles:
    compiled: Compiled
    model: mujoco.MjModel
    data: mujoco.MjData
    base_body: int
    base_qpos: int          # freejoint qpos address (7 entries: xyz + wxyz)
    base_dof: int           # freejoint dof address (6 entries)
    modules: tuple[ModuleHandles, ...]
    arm_qpos: np.ndarray
    arm_dof: np.ndarray
    arm_act: np.ndarray
    flange_site: int
    gripper_body: int
    gripper_act: int | None       # None for the magnet kind
    finger_qpos: np.ndarray       # empty for the magnet kind
    interface_site: int
    ft_force_adr: int
    ft_torque_adr: int


def _id(model, objtype, name: str) -> int:
    i = mujoco.mj_name2id(model, objtype, name)
    if i < 0:
        have = [mujoco.mj_id2name(model, objtype, k) for k in range(_count(model, objtype))]
        raise KeyError(f"{name!r} not in compiled model; candidates: {have}")
    return i


def _count(model, objtype) -> int:
    return {mujoco.mjtObj.mjOBJ_JOINT: model.njnt, mujoco.mjtObj.mjOBJ_ACTUATOR: model.nu,
            mujoco.mjtObj.mjOBJ_BODY: model.nbody, mujoco.mjtObj.mjOBJ_SITE: model.nsite,
            mujoco.mjtObj.mjOBJ_SENSOR: model.nsensor}[objtype]


def _drive_sign(m, data, dof: int) -> float:
    """+1/-1 such that `drive_sign * qvel[dof]` (equivalently `drive_sign * wheel_radps`
    written to the actuator) rolls the chassis toward +x -- see the module docstring.
    `data` must already have `mj_kinematics` run so `xmat` reflects the compiled default
    pose (steer joints at 0)."""
    jid = m.dof_jntid[dof]
    body = m.jnt_bodyid[jid]
    axis_world = data.xmat[body].reshape(3, 3) @ m.jnt_axis[jid]
    r_dir = np.array([0.0, 0.0, -1.0])           # wheel-center -> contact point, flat ground, z-up
    v_dir_per_unit_qvel = -np.cross(axis_world, r_dir)   # no-slip rolling: v_center = -omega x r
    return float(np.sign(v_dir_per_unit_qvel[0])) or 1.0


def resolve(compiled: Compiled) -> Handles:
    m = compiled.model
    r = compiled.robot
    J, A, B, S, SEN = (mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco.mjtObj.mjOBJ_BODY,
                       mujoco.mjtObj.mjOBJ_SITE, mujoco.mjtObj.mjOBJ_SENSOR)
    base_body = _id(m, B, r.base.body)
    base_jnt = m.body_jntadr[base_body]
    if m.jnt_type[base_jnt] != mujoco.mjtJoint.mjJNT_FREE:
        raise KeyError(f"base body {r.base.body!r} carries no freejoint")
    data = mujoco.MjData(m)
    mujoco.mj_kinematics(m, data)      # a fresh MjData's xmat is all-zero until this runs
    mods = []
    for mod in r.base.modules:
        sj, dj = _id(m, J, mod.steer_joint), _id(m, J, mod.drive_joint)
        sa, da = _id(m, A, r.base.steer_actuator_prefix + mod.name), _id(m, A, r.base.drive_actuator_prefix + mod.name)
        for act, jnt in ((sa, sj), (da, dj)):
            if m.actuator_trnid[act][0] != jnt:
                raise KeyError(f"actuator {mujoco.mj_id2name(m, A, act)} drives joint "
                               f"{mujoco.mj_id2name(m, J, m.actuator_trnid[act][0])}, spec says {mujoco.mj_id2name(m, J, jnt)}")
        mods.append(ModuleHandles(m.jnt_qposadr[sj], m.jnt_dofadr[sj], m.jnt_qposadr[dj], m.jnt_dofadr[dj],
                                   sa, da, _id(m, B, mod.wheel_body), _drive_sign(m, data, m.jnt_dofadr[dj])))
    arm_j = [_id(m, J, n) for n in r.arm.joints]
    arm_a = [_id(m, A, n) for n in r.arm.actuators]
    for act, jnt in zip(arm_a, arm_j):
        if m.actuator_trnid[act][0] != jnt:
            raise KeyError(f"arm actuator/joint mismatch at {mujoco.mj_id2name(m, A, act)}")
    fingers = [m.jnt_qposadr[_id(m, J, n)] for n in r.gripper.finger_joints]
    interface = _id(m, S, r.gripper.interface_site)
    return Handles(
        compiled=compiled, model=m, data=data, base_body=base_body,
        base_qpos=m.jnt_qposadr[base_jnt], base_dof=m.jnt_dofadr[base_jnt], modules=tuple(mods),
        arm_qpos=np.array([m.jnt_qposadr[j] for j in arm_j]), arm_dof=np.array([m.jnt_dofadr[j] for j in arm_j]),
        arm_act=np.array(arm_a), flange_site=_id(m, S, r.arm.flange_site),
        gripper_body=int(m.site_bodyid[interface]),
        gripper_act=None if r.gripper.actuator is None else _id(m, A, r.gripper.actuator),
        finger_qpos=np.array(fingers, dtype=int),
        interface_site=interface,
        ft_force_adr=m.sensor_adr[_id(m, SEN, "ft_force")], ft_torque_adr=m.sensor_adr[_id(m, SEN, "ft_torque")])
