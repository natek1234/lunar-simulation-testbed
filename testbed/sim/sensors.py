"""Everything a controller may know, read from the compiled model through the handles.
Base pose comes from the freejoint qpos (mj_step leaves xpos one step stale -- see
docs/design.md's measured MuJoCo pitfalls section)."""
import mujoco
import numpy as np

from testbed.control import quat as Q
from testbed.control.records import GripperCmd, GripperState, ModuleState, Observation, Pose, Twist
from testbed.sim.handles import Handles


def base_pose_W(h: Handles) -> Pose:
    q = h.data.qpos[h.base_qpos:h.base_qpos + 7]
    return Pose(q[:3].copy(), q[3:7].copy())


def base_twist_B(h: Handles) -> Twist:
    v = h.data.qvel[h.base_dof:h.base_dof + 6]
    quat = h.data.qpos[h.base_qpos + 3:h.base_qpos + 7]
    v_B = Q.rotate(Q.conj(quat), v[:3])           # freejoint linear velocity is in W
    return Twist(float(v_B[0]), float(v_B[1]), float(v[5]))   # angular part of a freejoint is already in B


def pose_W_to_B(base: Pose, p: Pose) -> Pose:
    qc = Q.conj(base.quat_wxyz)
    return Pose(Q.rotate(qc, p.pos_m - base.pos_m), Q.mul(qc, p.quat_wxyz))


def pose_B_to_W(base: Pose, p: Pose) -> Pose:
    return Pose(base.pos_m + Q.rotate(base.quat_wxyz, p.pos_m), Q.mul(base.quat_wxyz, p.quat_wxyz))


def site_pose_W(h: Handles, site: int) -> Pose:
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, h.data.site_xmat[site])
    return Pose(h.data.site_xpos[site].copy(), quat)


def ft_flange_B(h: Handles) -> np.ndarray:
    """Force/torque sensors report in the SITE frame; rotate into B."""
    d = h.data
    f = d.sensordata[h.ft_force_adr:h.ft_force_adr + 3]
    t = d.sensordata[h.ft_torque_adr:h.ft_torque_adr + 3]
    site_q = site_pose_W(h, h.flange_site).quat_wxyz
    base_q = d.qpos[h.base_qpos + 3:h.base_qpos + 7]
    to_B = Q.mul(Q.conj(base_q), site_q)
    return np.concatenate([Q.rotate(to_B, f), Q.rotate(to_B, t)])


def gripper_state(h: Handles, cmd: GripperCmd, attached_id: str | None) -> GripperState:
    from testbed.sim.compose import gripper_strategy
    aperture = gripper_strategy(h.compiled.robot).aperture_m(h)
    return GripperState(aperture_m=aperture, attached=attached_id is not None, attached_id=attached_id, cmd=cmd)


def wheel_contacts(h: Handles) -> np.ndarray:
    """(n_modules,) bool: True where that module's wheel BODY appears on either side of any
    live contact this tick (`d.contact[i].geom1`/`geom2` resolved back to a body id, per the
    geom1/geom2-order caution the sim layer already documents elsewhere -- here we only need
    "did this body touch anything," which is symmetric in geom1 vs geom2, so no sign/side
    branch is needed). Used by `testbed.bench.metrics` for the wheel-contact-fraction
    metric (docs/guide/benchmarks.md)."""
    d, m = h.data, h.model
    wheel_bodies = [mh.wheel_body for mh in h.modules]
    out = np.zeros(len(wheel_bodies), dtype=bool)
    for i in range(d.ncon):
        c = d.contact[i]
        b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        for k, wb in enumerate(wheel_bodies):
            if not out[k] and (b1 == wb or b2 == wb):
                out[k] = True
    return out


def observe(h: Handles, t_s: float, tick: int, *, gripper_cmd: GripperCmd, phase: str | None,
            attached_id: str | None = None, oracle=None, tags=None, goal=None, instruction=None,
            envelope_active: bool = False, tool_offset_F: Pose | None = None,
            tool_kind: str | None = None) -> Observation:
    d, m = h.data, h.model
    base = base_pose_W(h)
    ee_W = site_pose_W(h, h.flange_site)
    mods = tuple(ModuleState(float(d.qpos[mh.steer_qpos]), float(d.qvel[mh.steer_dof]),
                              float(mh.drive_sign * d.qvel[mh.drive_dof]))     # see handles.ModuleHandles.drive_sign
                 for mh in h.modules)
    return Observation(
        t_s=t_s, tick=tick, base_pose_W=base, base_twist_B=base_twist_B(h),
        q_rad=d.qpos[h.arm_qpos].copy(), qd_radps=d.qvel[h.arm_dof].copy(),
        tau_nm=d.actuator_force[h.arm_act].copy(), modules=mods,
        gripper=gripper_state(h, gripper_cmd, attached_id), ft_flange_B=ft_flange_B(h),
        ee_pose_B=pose_W_to_B(base, ee_W), ee_pose_W=ee_W, oracle=oracle, tags=tags, goal=goal,
        instruction=instruction, envelope_active=envelope_active, phase=phase,
        tool_offset_F=tool_offset_F, tool_kind=tool_kind, wheel_contact=wheel_contacts(h))
