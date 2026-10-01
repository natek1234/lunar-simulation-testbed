"""Parallel gripper strategy: attach the vendored hand at the flange with the mount
correction, add the interface site, optionally retune the servo gain. Attachment =
both finger bodies in contact with the same RCU while commanded closed.

The interface site is added AFTER `mj.attach()`, on the already-prefixed root body --
`spec.interface_site` is written in the robot YAML as the UNPREFIXED name
(`gripper_interface`) and stays that way: a site added on the child spec BEFORE attach
would itself get prefixed along with everything else `attach()` carries over, so adding
it post-attach on the parent's own (now-prefixed) body is what keeps the YAML name
literal [confirmed: tests/test_compose.py::test_hand_variant_is_prefixed_and_mounted_at_minus_45_deg_yaw
resolves `m.site("gripper_interface")` unprefixed]."""
import mujoco
import numpy as np

from testbed.control.records import GripperCmd
from testbed.sim.scaling import scale_spec
from testbed.spec.robot import GripperSpec

INTERFACE_SITE_SIZE_M = 0.005   # [policy default: small marker sphere radius]


def apply(h, cmd: GripperCmd) -> None:
    g = h.compiled.robot.gripper
    if cmd is GripperCmd.OPEN:
        h.data.ctrl[h.gripper_act] = g.cmd_open
    elif cmd is GripperCmd.CLOSE:
        h.data.ctrl[h.gripper_act] = g.cmd_close


def aperture_m(h) -> float | None:
    return float(h.data.qpos[h.finger_qpos].sum())


def attachment(h, cmd: GripperCmd, attached_id: str | None, candidates: dict[str, int]) -> str | None:
    if cmd is not GripperCmd.CLOSE:
        return None
    m, d = h.model, h.data
    fingers = tuple(int(m.jnt_bodyid[m.joint(j).id]) for j in h.compiled.robot.gripper.finger_joints)
    touching = {fb: set() for fb in fingers}
    for i in range(d.ncon):
        c = d.contact[i]
        b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        for fb in fingers:
            other = b2 if b1 == fb else b1 if b2 == fb else None
            if other is not None:
                touching[fb].add(other)
    common = set.intersection(*touching.values())
    return next((rid for rid, bid in candidates.items() if bid in common), None)


def mount_frame_quat_wxyz(target_wxyz, child_root_wxyz) -> np.ndarray:
    """MuJoCo composes child-in-parent as mount (x) child_own; solve mount = target (x) inv(child_own)."""
    inv = np.zeros(4)
    mujoco.mju_negQuat(inv, np.asarray(child_root_wxyz, dtype=float))
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, np.asarray(target_wxyz, dtype=float), inv)
    return out


def compose(spec: GripperSpec, mj: mujoco.MjSpec, child: mujoco.MjSpec,
            flange_body: str, flange_pos: np.ndarray) -> None:
    scale_spec(child, spec.scale)
    root = child.body(spec.root_body)
    corr = mount_frame_quat_wxyz(spec.mount_quat_wxyz, root.quat)
    frame = mj.body(flange_body).add_frame(name="gripper_mount", pos=list(flange_pos), quat=list(corr))
    mj.attach(child, prefix=spec.attach_prefix, frame=frame)
    mounted_root = mj.body(spec.attach_prefix + spec.root_body)
    site = mounted_root.add_site(name=spec.interface_site, pos=list(spec.interface_offset_m))
    site.size = [INTERFACE_SITE_SIZE_M, 0, 0]
    if spec.gain_override is not None:
        act = mj.actuator(spec.actuator)
        act.gainprm[0] = act.gainprm[0] * spec.gain_override
        act.biasprm[1] = act.biasprm[1] * spec.gain_override
        act.biasprm[2] = act.biasprm[2] * spec.gain_override
        act.forcerange = np.asarray(act.forcerange) * spec.gain_override
