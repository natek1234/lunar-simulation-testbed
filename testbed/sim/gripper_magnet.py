"""Electromagnet strategy: an authored cylinder on the flange, no actuator. Attachment is
a WELD equality per candidate RCU, created inactive at compose time and switched on when
the magnet face is within capture distance and angle of the unit's patch while the
command is CLOSE (energised); released on OPEN or when |F_flange| exceeds hold_force_n.

eq_data layout for a WELD equality, confirmed empirically [measured: scratch probe,
2026-09-18, two free bodies welded at a known relative pose, 200 steps of dynamics under
gravity, realized relpose recovered to float64 precision]: eq_data[0:3] = anchor (unused
here, stays 0), eq_data[3:6] = relpose position of body2 in body1's frame, eq_data[6:10]
= relpose quaternion (wxyz) of body2 in body1's frame, eq_data[10] = torquescale. The
magnet face normal sign and the exact capture geometry were RE-VERIFIED against a real RCU
(`tests/test_worksite.py`'s capture test), and the sign this file shipped with (`-z` of the
interface site) was WRONG -- see `attachment()`'s own comment and DECISIONS.md D10 for the
measurement."""
import mujoco
import numpy as np

from testbed.control import quat as Q
from testbed.control.records import GripperCmd
from testbed.spec.robot import GripperSpec

MAGNET_BODY = "magnet"
WELD_PREFIX = "magnet_weld_"
# The default solref MuJoCo gives an equality constraint, [0.02, 1] (a 0.02 s time constant,
# critical damping), is soft enough that a 22 kg RCU welded at a single top-centre point --
# with real rotational inertia over its 0.43 m length -- visibly SWINGS on it: measured
# (tests/test_single_placement.py, 2026-09-21) a well-centred, fully-settled capture (0.02 deg tilt at the
# moment of attachment) still grew to 10 deg by the end of `acquire`'s own lift and to 28.5 deg
# during `transport`'s base motion, oscillating in step with ordinary IK/base accelerations, not
# with any one bad event. `TIMESTEP_S` (testbed/sim/compose.py) is 0.002 s; MuJoCo's own guidance
# is a solref timeconst >= 2x the physics timestep for the implicit solver to stay stable, so
# 0.004 s is the STIFFEST value in that safe range -- an electromagnet holding a payload is
# meant to behave as close to rigid as the solver allows, not with generic-joint compliance.
WELD_SOLREF = (0.004, 1.0)  # [derived: 2 x TIMESTEP_S (0.002 s), the solver's own minimum-stable timeconst]


def compose(spec: GripperSpec, mj: mujoco.MjSpec, child, flange_body: str, flange_pos: np.ndarray) -> None:
    mg = spec.magnet
    body = mj.body(flange_body).add_body(name=MAGNET_BODY, pos=list(flange_pos), quat=list(spec.mount_quat_wxyz))
    g = body.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[mg.radius_m, mg.height_m / 2, 0],
                       pos=[0, 0, mg.height_m / 2], mass=mg.mass_kg)
    g.rgba = [0.15, 0.15, 0.2, 1]     # [policy default: dark grey, no rendering claim]
    site = body.add_site(name=spec.interface_site, pos=list(spec.interface_offset_m))
    site.size = [0.005, 0, 0]         # [policy default: small marker sphere radius]


def add_weld_candidates(mj: mujoco.MjSpec, rcu_ids: list[str]) -> None:
    """Called by the worksite composer: one inactive weld per RCU body, plus a contact
    EXCLUDE for the same pair.

    Without the exclude, the single-placement scenario's own `acquire` lift measured the magnet cylinder and
    the just-welded RCU's mesh resting in permanent, near-constant contact (dist about
    -0.0005 m -- the two geoms were never NOT touching once the unit was captured, since
    the weld's own captured relpose is the pose at which the magnet face sits flush on the
    patch): the weld (a rigid equality) and the unilateral contact constraint (which only
    ever pushes apart) then fight over the SAME 0.5 mm of interpenetration every tick, one
    holding the bodies together and the other trying to separate them. Measured over the
    3 s acquire lift: the flange F/T sensor oscillated between about 8 N and 111 N tick to
    tick (no smooth trend, a genuine fight, not load noise) while the unit's own
    orientation crept from 0.4 deg at the moment of capture to 7.5 deg by the top of the
    lift -- a real, growing tilt, not a transient. A held object shouldn't ALSO be
    collision-checked against the thing holding it (that is what the weld already models),
    so this excludes exactly the magnet/candidate-RCU pairs that can ever be welded, one
    `add_exclude` alongside each `add_equality` [measured: tests/test_single_placement.py, 2026-09-21].
    """
    for rid in rcu_ids:
        mj.add_equality(name=WELD_PREFIX + rid, type=mujoco.mjtEq.mjEQ_WELD, objtype=mujoco.mjtObj.mjOBJ_BODY,
                         name1=MAGNET_BODY, name2=rid, active=False, solref=WELD_SOLREF)
        mj.add_exclude(name=f"magnet_exclude_{rid}", bodyname1=MAGNET_BODY, bodyname2=rid)


def apply(h, cmd: GripperCmd) -> None:
    return None            # no actuator; energising is the CLOSE command handled in attachment()


def aperture_m(h) -> float | None:
    return None


def attachment(h, cmd: GripperCmd, attached_id: str | None, candidates: dict[str, int]) -> str | None:
    m, d = h.model, h.data
    spec = h.compiled.robot.gripper
    if attached_id is not None:
        f = d.sensordata[h.ft_force_adr:h.ft_force_adr + 3]
        if cmd is not GripperCmd.CLOSE or np.linalg.norm(f) > spec.magnet.hold_force_n:
            d.eq_active[m.equality(WELD_PREFIX + attached_id).id] = 0
            return None
        return attached_id
    if cmd is not GripperCmd.CLOSE:
        return None
    face = d.site_xpos[h.interface_site]
    # Magnet face points along +z of the interface site, not -z: the site sits at the FAR
    # end of the authored cylinder (interface_offset_m == the cylinder's own height, i.e.
    # the face), so the direction OUT of the magnet into free space (where a target sits)
    # is the site's own local +z, continuing the same direction the cylinder was extruded
    # in from the flange. Measured directly against a real seated RCU (`seat_in_gripper`,
    # patch pressed flush against the face, patch normal opposed by construction): with the
    # `-z` sign an earlier draft shipped, the angle came back pi (3.1416 rad, exactly
    # anti-aligned) against a capture_angle_rad of 0.35 -- never captures; with `+z` the
    # SAME pose measures 7.5e-8 rad (aligned) [measured: tests/test_worksite.py::
    # test_energised_magnet_on_the_patch_captures_holds_lifts_and_breaks_away, 2026-09-20].
    # Ledgered as DECISIONS.md D10.
    normal = d.site_xmat[h.interface_site].reshape(3, 3)[:, 2]
    for rid, bid in candidates.items():
        patch = m.site(f"{rid}_patch").id
        dist = np.linalg.norm(d.site_xpos[patch] - face)
        pn = d.site_xmat[patch].reshape(3, 3)[:, 2]
        ang = np.arccos(np.clip(-normal @ pn, -1, 1))
        if dist <= spec.magnet.capture_distance_m and ang <= spec.magnet.capture_angle_rad:
            eq = m.equality(WELD_PREFIX + rid).id
            _set_weld_to_current_relpose(m, d, eq, m.body(MAGNET_BODY).id, bid)
            d.eq_active[eq] = 1
            return rid
    return None


def _set_weld_to_current_relpose(m, d, eq: int, b1: int, b2: int) -> None:
    """eq_data for a weld: anchor(3), relpose pos(3) + quat(4) of body2 in body1, torquescale(1)."""
    q1, q2 = d.xquat[b1], d.xquat[b2]
    rel_q = Q.mul(Q.conj(q1), q2)
    rel_p = Q.rotate(Q.conj(q1), d.xpos[b2] - d.xpos[b1])
    m.eq_data[eq][0:3] = 0.0
    m.eq_data[eq][3:6] = rel_p
    m.eq_data[eq][6:10] = rel_q
