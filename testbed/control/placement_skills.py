"""The placement skills: approach/park (base), acquire/transport/place/seat/release/retract/verify
(arm + gripper). All emit TaskActions with FLANGE targets in W; tool poses are converted
through Observation.tool_offset_F. Grasp geometry follows Observation.tool_kind: a magnet
picks top-down on the unit's patch; a parallel jaw grasps the unit's END (fingers closing
vertically on the 0.10 m face). Approach/retreat is always along the tool's own -z.

Geometry cross-checked against the real single-placement scenario (scenarios/single_placement.yaml):
the RCU is 0.43 x 0.25 x 0.10 m (UNIT_HALF_LEN_M/UNIT_HALF_H_M below), the default
robot's gripper is the top-down electromagnet (Observation.tool_kind == "magnet"), and the
slot sits at the SECOND course of a two-unit foundation (centre z = 0.15 m) -- there is no
pallet in this scenario, so `place`'s hover target is simply "above the slot," not "above a
pallet.\""""
import numpy as np

from testbed.control import quat as Q
from testbed.control.records import GripperCmd, Observation, Pose, TaskAction, Twist
from testbed.control.skills import register

KP_LIN = 0.8            # [policy default: base stance P gain, 1/s]
KP_YAW = 1.2            # [policy default]
STANCE_TOL_M = 0.05     # [policy default]
STANCE_TOL_RAD = 0.05   # [policy default]
EE_TOL_M = 0.015        # [policy default: arm waypoint reached]
PARK_TOL_MPS = 0.005    # [policy default]
PARK_HOLD_S = 0.5       # [policy default]
GRASP_INSERT_M = 0.04   # [authored-geometry: pad centre 4 cm inside the unit's end face]
PREGRASP_BACK_M = 0.30  # [policy default: pre-grasp standoff along the approach axis]
LIFT_M = 0.20           # [policy default: clear the pallet / foundation]
SEAT_RATE_MPS = 0.02    # [policy default: seating descent speed]
SEAT_STILL_S = 0.4      # [policy default: unit z unchanged this long = seated]
SEAT_STILL_M = 0.002    # [policy default]
GRIP_SETTLE_S = 1.0     # [policy default: wait after open/close or energise/de-energise]
UNIT_HALF_LEN_M = 0.215 # [ported: RCU 0.43 m length / 2, scenarios/single_placement.yaml]
UNIT_HALF_H_M = 0.05    # [ported: RCU 0.10 m height / 2, scenarios/single_placement.yaml]
# Grasp-time position tolerance for the "at" waypoint (see _ArmSkill's own docstring for the
# continuous-residency `dwell` semantics this depends on): measured (tests/test_single_placement.py,
# 2026-09-21) that energising the magnet the instant a fast approach FIRST swings through the
# ordinary EE_TOL_M=0.015 m band captures the 22 kg, 0.43 m-long RCU off its centroid (up to
# 1.34 cm along the long axis) and, worse, WHILE STILL MOVING. The unit is still resting on the
# ground (friction [1.0, ...]) at that instant, so friction pins whichever corner is still
# touching as a de facto pivot, and the residual swing plus that off-centre pull TIPS the unit
# about the pinned corner instead of lifting it -- measured growing to a 10 deg steady tilt with
# `acquire` stalled indefinitely (the flange target itself froze at a fixed point 0.0295 m short
# of `at`: this exact commanded pose is near the edge of what a resolved-rate DLS solve can
# converge tighter than, at this reach/orientation -- q stopped changing tick to tick with no
# joint at its own limit, i.e. a manipulability/near-singularity floor, not a servo error).
# GRASP_TOL_M is set just above that measured floor, combined with the continuous-dwell fix
# above so the settle happens near-zero-velocity rather than mid-swing.
GRASP_TOL_M = 0.032     # [measured: tests/test_single_placement.py, 2026-09-21 -- steady residual ~0.0295 m]
# `_ArmSkill` never hands a NEW waypoint's tool pose straight to IK: measured (tests/test_single_placement.py,
# 2026-09-21, with the envelope's force clamp temporarily neutralised to isolate the cause) a
# PEAK flange force of 751 N -- about 21x the RCU's 35.6 N lunar weight -- at the exact tick
# `acquire`'s own target switches from the static `at` hold to `up`, a 0.20 m jump. The
# resolved-rate law computes commanded velocity as kp_pos*error with no rate limit of its own
# (`testbed/control/ik.py`'s `IkConfig.v_max_mps` bounds the STEADY-STATE speed, not how fast the
# command can step there from zero); jumping the position target 0.20 m in one tick asks for an
# instantaneous velocity STEP up to v_max, and slamming a 22 kg suspended payload through that
# step is what rang the weld (see WELD_SOLREF) into the growing tilt this file's tests gate on.
# RAMP_RATE_MPS moves the FED reference toward each waypoint's true tool pose at a bounded rate
# instead, the same rate-limited-reference idea `seat` already uses (SEAT_RATE_MPS) generalised
# to every `_ArmSkill` waypoint transition -- cheap because it only changes what's handed to an
# unmodified IK/envelope, not either of those shared modules.
RAMP_RATE_MPS = 0.10    # [measured: tests/test_single_placement.py, 2026-09-21 -- 5x under v_max_mps, kept the peak flange force under budget]
# The SAME step-input problem as RAMP_RATE_MPS, one layer down: `Approach`/`Transport`'s P-law
# on base stance error saturates near the robot's own max_vx/vy/wz caps as soon as the skill
# starts (the stance error is close to the full standoff distance at that instant), and
# `SwerveAllocator`/`ModuleFirmware` rate-limit only the STEER angle, not the drive wheel speed
# -- so the base's own commanded velocity can step from 0 to ~max speed in one control tick.
# Measured (tests/test_single_placement.py, 2026-09-21): even after ramping the arm's own tool-position target
# (RAMP_RATE_MPS), the carried RCU's tilt still grew from ~3 deg to 37 deg over `transport` --
# the base's start/stop/turn transients swinging the suspended payload like a pendulum, the same
# mechanism as the arm-side jump, one level up the kinematic chain. `Approach.act`/`Park.act` ramp
# their own commanded twist toward the P-law's desired one at these bounded accelerations instead
# of handing it over verbatim.
ACCEL_MAX_MPS2 = 0.3        # [measured: tests/test_single_placement.py, 2026-09-21 -- kept the carried unit's tilt from growing during transport]
YAW_ACCEL_MAX_RADPS2 = 0.4  # [policy default: paired with ACCEL_MAX_MPS2, no yaw-specific measurement pass]


def _rot(q):
    x, y, z = Q.rotate(q, [1, 0, 0]), Q.rotate(q, [0, 1, 0]), Q.rotate(q, [0, 0, 1])
    return np.column_stack([x, y, z])


def target_pose_W(obs: Observation, name: str) -> Pose:
    """An RCU id resolves against the oracle (a live physical pose); a slot id resolves
    against the goal (an authored, fixed target pose) -- the two channels Worksite.observe
    already exposes."""
    if obs.oracle is not None:
        for o in obs.oracle:
            if o.id == name:
                return o.pose_W
    if obs.goal is not None and name in obs.goal.slot_ids:
        return obs.goal.slot_poses_W[obs.goal.slot_ids.index(name)]
    raise KeyError(name)


def grasp_tool_pose_W(unit: Pose, kind: str, back_m: float, lift_m: float) -> Pose:
    """Tool pose that holds `unit`, retreated `back_m` along the tool's -z (the approach
    axis) and raised `lift_m` in world z.
    magnet:   tool z points DOWN, tool x follows the unit's yaw, face on the top-face patch
              (this is also the pose that holds the unit when its CENTRE sits at `unit.pos_m`
              -- Worksite.seat_in_gripper's ground truth for the magnet kind).
    parallel: tool z along the unit's +x (end grasp), fingers (tool y) opening along world z,
              pad centre GRASP_INSERT_M inside the unit's -x end."""
    ax = Q.rotate(unit.quat_wxyz, [1, 0, 0]); ax[2] = 0; ax /= np.linalg.norm(ax)
    if kind == "magnet":
        z_tool = np.array([0.0, 0.0, -1.0]); x_tool = ax; y_tool = np.cross(z_tool, x_tool)
        pos = unit.pos_m + np.array([0, 0, UNIT_HALF_H_M])
    else:
        z_tool = ax; y_tool = np.array([0.0, 0.0, 1.0]); x_tool = np.cross(y_tool, z_tool)
        pos = unit.pos_m - ax * (UNIT_HALF_LEN_M - GRASP_INSERT_M)
    pos = pos - z_tool * back_m + np.array([0, 0, lift_m])
    return Pose(pos, Q.from_matrix(np.column_stack([x_tool, y_tool, z_tool])))


def flange_from_tool(tool_W: Pose, off_F: Pose) -> Pose:
    """Invert tool_W = flange_W (x) off_F (composition order: tool pos = flange pos +
    R(flange_quat) @ off_pos, tool quat = flange_quat * off_quat) to recover the flange
    pose that puts the tool at `tool_W`."""
    q_f = Q.mul(tool_W.quat_wxyz, Q.conj(off_F.quat_wxyz))
    return Pose(tool_W.pos_m - Q.rotate(q_f, off_F.pos_m), q_f)


def _ee_err_m(obs: Observation, flange_W: Pose) -> float:
    return float(np.linalg.norm(obs.ee_pose_W.pos_m - flange_W.pos_m))


def _tool_pos_W(obs: Observation) -> np.ndarray:
    """The tool's own current world position, from the flange pose plus the measured
    flange->tool offset (the inverse direction of `flange_from_tool`)."""
    return obs.ee_pose_W.pos_m + Q.rotate(obs.ee_pose_W.quat_wxyz, obs.tool_offset_F.pos_m)


def _ramp_twist(cur: Twist, desired: Twist, dt: float) -> Twist:
    """Move `cur` toward `desired` at ACCEL_MAX_MPS2 (linear, jointly on vx/vy) and
    YAW_ACCEL_MAX_RADPS2 (yaw) -- see their own comment."""
    v_cur, v_des = np.array([cur.vx_mps, cur.vy_mps]), np.array([desired.vx_mps, desired.vy_mps])
    dv = v_des - v_cur
    dv_mag = float(np.linalg.norm(dv))
    step = ACCEL_MAX_MPS2 * max(dt, 0.0)
    v_new = v_des if dv_mag <= step else v_cur + dv / dv_mag * step
    dw = desired.wz_radps - cur.wz_radps
    wstep = YAW_ACCEL_MAX_RADPS2 * max(dt, 0.0)
    w_new = desired.wz_radps if abs(dw) <= wstep else cur.wz_radps + np.sign(dw) * wstep
    return Twist(float(v_new[0]), float(v_new[1]), float(w_new))


def _wp(tool, grip, dwell, tol=EE_TOL_M):
    """A waypoint: (tool pose W | None, gripper cmd, min dwell s, position tolerance m).
    `tol` defaults to the general EE_TOL_M; grasp-critical waypoints pass GRASP_TOL_M."""
    return (tool, grip, dwell, tol)


class _ArmSkill:
    """Sequence of `_wp(...)` waypoints. A `None` tool pose holds the current commanded
    flange target (used by `release`/`verify`, which only change the gripper command).

    `dwell` means CONTINUOUS residency inside `tol`, not "some time has passed since we
    started aiming here": measured (tests/test_single_placement.py, 2026-09-21) that a first-crossing
    check (start a clock when the waypoint is entered, advance once that clock plus
    `reached` are both true) lets a fast, still-decelerating swing through the tolerance
    band satisfy it -- the arm was still moving (a large, and load-dependent, residual
    Cartesian velocity) at the instant `acquire`'s magnet energised, which is exactly the
    dynamic disturbance that let a 22 kg unit's grip go off-centre and then tip (see
    GRASP_TOL_M). Tracking "time continuously below tol" instead (reset the clock the
    moment the error leaves tol again) only ever fires once the arm has genuinely settled.

    `reached`/`done` always check the WAYPOINT's true tool pose (so completion still means
    "actually arrived"), but the pose handed to IK each tick is a RAMPED one that chases the
    true target at RAMP_RATE_MPS (see its own comment) -- softening the velocity step at
    every waypoint switch without changing what "done" means."""
    def start(self, params, obs):
        self.i, self.t_reached = 0, None
        self.wps = self.waypoints(params, obs)
        self._ramp_pos = _tool_pos_W(obs)
        self._t_last = obs.t_s

    def waypoints(self, params, obs):
        raise NotImplementedError

    def _ramped_tool(self, obs, tool):
        if tool is None:
            return None
        step = RAMP_RATE_MPS * max(obs.t_s - self._t_last, 0.0)
        delta = tool.pos_m - self._ramp_pos
        dist = np.linalg.norm(delta)
        self._ramp_pos = tool.pos_m.copy() if dist <= step else self._ramp_pos + delta / dist * step
        return Pose(self._ramp_pos.copy(), tool.quat_wxyz)

    def _reached_and_settled(self, obs):
        tool, grip, dwell, tol = self.wps[self.i]
        fl = None if tool is None else flange_from_tool(tool, obs.tool_offset_F)
        reached = fl is None or _ee_err_m(obs, fl) < tol
        self.t_reached = (self.t_reached or obs.t_s) if reached else None
        settled = reached and self.t_reached is not None and obs.t_s - self.t_reached >= dwell
        return fl, grip, settled

    def act(self, obs):
        tool, grip, dwell, tol = self.wps[self.i]
        _, _, settled = self._reached_and_settled(obs)
        ramped = self._ramped_tool(obs, tool)
        self._t_last = obs.t_s
        if settled and self.i < len(self.wps) - 1:
            self.i += 1; self.t_reached = None
        fl = None if ramped is None else flange_from_tool(ramped, obs.tool_offset_F)
        return TaskAction(ee_target=fl, ee_frame="W", gripper=grip)

    def done(self, obs):
        if self.i != len(self.wps) - 1:
            return False
        _, _, settled = self._reached_and_settled(obs)
        return settled


@register("approach")
class Approach:
    """Drive to a stance `standoff_m` behind the target along the target's -x axis, facing +x."""
    def start(self, params, obs):
        self.name, self.standoff = params["target"], float(params["standoff_m"])
        self._cmd, self._t_last = Twist(), obs.t_s

    def _stance(self, obs):
        t = target_pose_W(obs, self.name)
        ax = Q.rotate(t.quat_wxyz, [1, 0, 0]); ax[2] = 0; ax /= np.linalg.norm(ax)
        return t.pos_m[:2] - ax[:2] * self.standoff, float(np.arctan2(ax[1], ax[0]))

    def act(self, obs):
        goal_xy, goal_yaw = self._stance(obs)
        err_W = goal_xy - obs.base_pose_W.pos_m[:2]
        yaw = Q.yaw_rad(obs.base_pose_W.quat_wxyz)
        c, s = np.cos(yaw), np.sin(yaw)
        ex, ey = c * err_W[0] + s * err_W[1], -s * err_W[0] + c * err_W[1]
        eyaw = float((goal_yaw - yaw + np.pi) % (2 * np.pi) - np.pi)
        desired = Twist(KP_LIN * ex, KP_LIN * ey, KP_YAW * eyaw)
        self._cmd = _ramp_twist(self._cmd, desired, obs.t_s - self._t_last)
        self._t_last = obs.t_s
        return TaskAction(base_twist_B=self._cmd)

    def done(self, obs):
        goal_xy, goal_yaw = self._stance(obs)
        eyaw = (goal_yaw - Q.yaw_rad(obs.base_pose_W.quat_wxyz) + np.pi) % (2 * np.pi) - np.pi
        return np.linalg.norm(goal_xy - obs.base_pose_W.pos_m[:2]) < STANCE_TOL_M and abs(eyaw) < STANCE_TOL_RAD


@register("transport")
class Transport(Approach):
    pass


@register("park")
class Park:
    """Bring the base to rest and hold, so the arm sequence that follows isn't fighting
    residual chassis motion -- `done()` needs PARK_HOLD_S of continuous stillness, not just
    one quiet tick."""
    def start(self, params, obs):
        self.still_since = None
        self._cmd, self._t_last = Twist(obs.base_twist_B.vx_mps, obs.base_twist_B.vy_mps, obs.base_twist_B.wz_radps), obs.t_s

    def act(self, obs):
        v = np.hypot(obs.base_twist_B.vx_mps, obs.base_twist_B.vy_mps) + abs(obs.base_twist_B.wz_radps)
        self.still_since = obs.t_s if (v >= PARK_TOL_MPS or self.still_since is None) else self.still_since
        self._cmd = _ramp_twist(self._cmd, Twist(), obs.t_s - self._t_last)
        self._t_last = obs.t_s
        return TaskAction(base_twist_B=self._cmd)

    def done(self, obs):
        return self.still_since is not None and obs.t_s - self.still_since >= PARK_HOLD_S


@register("acquire")
class Acquire(_ArmSkill):
    """Pre-grasp standoff -> down onto the unit (gripper still open) -> energise/close and
    dwell (GRIP_SETTLE_S, long enough for the magnet weld / parallel-finger contact to
    latch) -> lift clear."""
    def waypoints(self, params, obs):
        u, k = target_pose_W(obs, params["target"]), obs.tool_kind
        pre = grasp_tool_pose_W(u, k, PREGRASP_BACK_M, 0.0)
        at = grasp_tool_pose_W(u, k, 0.0, 0.0)
        up = grasp_tool_pose_W(u, k, 0.0, LIFT_M)
        # The two `at` waypoints use GRASP_TOL_M, not EE_TOL_M -- see GRASP_TOL_M's own
        # comment: energising off-centre by even ~1.3 cm let ground friction pin a corner
        # and tip the unit as `up` tried to lift it, growing to a stalled 10 deg tilt.
        return [_wp(pre, GripperCmd.OPEN, 0.0), _wp(at, GripperCmd.OPEN, 0.3, GRASP_TOL_M),
                _wp(at, GripperCmd.CLOSE, GRIP_SETTLE_S, GRASP_TOL_M), _wp(up, GripperCmd.CLOSE, 0.3)]


@register("place")
class Place(_ArmSkill):
    """Hover the carried unit `hover_m` above the target slot (no pallet in this scenario --
    the hover point is simply above the slot's own seated pose); `seat` does the descent."""
    def waypoints(self, params, obs):
        slot = target_pose_W(obs, params["target"])
        return [_wp(grasp_tool_pose_W(slot, obs.tool_kind, 0.0, float(params["hover_m"])), GripperCmd.CLOSE, 0.3)]


@register("seat")
class Seat:
    """Lower at SEAT_RATE_MPS until the carried unit stops descending (the envelope's force
    freeze only bites above the robot's `force_budget_n` -- 210 N for the magnet robot, so the
    press-in is NOT held to the contact-force budget (docs/reference/requirements.md, 58.6 N),
    DECISIONS.md D15); if nothing is attached (a failed `acquire`), there is
    nothing to seat -- report done immediately rather than lower an empty gripper forever.

    "Stopped descending" is checked over a whole SEAT_STILL_S WINDOW, not tick to tick: a
    per-control-tick (0.02 s) comparison against SEAT_STILL_M=0.002 m is a comparison against
    the WRONG scale -- a genuinely still-descending unit moves SEAT_RATE_MPS*0.02s = 0.0004 m
    per control tick, already below that threshold, so a tick-to-tick check reports "landed"
    on its very first tick regardless of whether the unit ever touched anything. Measured
    (tests/test_single_placement.py, 2026-09-21): with the tick-to-tick version, `seat` reported done ~0.4 s
    after entering, while the unit was still 0.28 m up (its `place` hover height) -- `release`
    then dropped it from there, and toppling on landing is exactly what produced this file's
    gate-failing final tilt/position error. Comparing the TOTAL movement across one whole
    SEAT_STILL_S window (0.02 m/s x 0.4 s = 0.008 m expected while genuinely descending, well
    above SEAT_STILL_M) against SEAT_STILL_M fixes the scale mismatch without changing either
    constant's own value."""
    def start(self, params, obs):
        self.slot = target_pose_W(obs, params["target"])
        self.unit_id = obs.gripper.attached_id
        self.tool = grasp_tool_pose_W(self.slot, obs.tool_kind, 0.0, 0.0)
        self.z = obs.ee_pose_W.pos_m[2] + Q.rotate(obs.ee_pose_W.quat_wxyz, obs.tool_offset_F.pos_m)[2]
        self.t_last = obs.t_s
        self.window_z0, self.t_window0 = None, obs.t_s

    def act(self, obs):
        self.z -= SEAT_RATE_MPS * (obs.t_s - self.t_last); self.t_last = obs.t_s
        tool = Pose(np.array([self.tool.pos_m[0], self.tool.pos_m[1], max(self.z, self.tool.pos_m[2] - 0.05)]), self.tool.quat_wxyz)
        return TaskAction(ee_target=flange_from_tool(tool, obs.tool_offset_F), ee_frame="W", gripper=GripperCmd.CLOSE)

    def done(self, obs):
        if self.unit_id is None:
            return True
        z = target_pose_W(obs, self.unit_id).pos_m[2]
        if self.window_z0 is None:
            self.window_z0, self.t_window0 = z, obs.t_s
            return False
        if obs.t_s - self.t_window0 < SEAT_STILL_S:
            return False
        landed = abs(z - self.window_z0) < SEAT_STILL_M
        self.window_z0, self.t_window0 = z, obs.t_s
        return landed


@register("release")
class Release(_ArmSkill):
    def waypoints(self, params, obs):
        return [_wp(None, GripperCmd.OPEN, GRIP_SETTLE_S)]


@register("retract")
class Retract(_ArmSkill):
    """Back off along the tool's current -z, then lift clear -- computed from wherever the
    flange actually is (not from a remembered target), so retract works even if the arm
    ended up somewhere `seat`'s own targets didn't exactly predict."""
    def waypoints(self, params, obs):
        tool_now = Pose(obs.ee_pose_W.pos_m + Q.rotate(obs.ee_pose_W.quat_wxyz, obs.tool_offset_F.pos_m),
                        Q.mul(obs.ee_pose_W.quat_wxyz, obs.tool_offset_F.quat_wxyz))
        back = Pose(tool_now.pos_m - Q.rotate(tool_now.quat_wxyz, [0, 0, PREGRASP_BACK_M]), tool_now.quat_wxyz)
        up = Pose(back.pos_m + np.array([0, 0, float(params["height_m"])]), back.quat_wxyz)
        return [_wp(back, GripperCmd.OPEN, 0.0), _wp(up, GripperCmd.OPEN, 0.2)]


@register("verify")
class Verify(_ArmSkill):
    """A no-op dwell: the runtime itself marks the target slot filled when this skill
    reports done (Runtime._run_skill), so `verify` just needs to hold position long enough
    for that to be a meaningful "we looked" moment, not an instant rubber stamp."""
    def waypoints(self, params, obs):
        return [_wp(None, GripperCmd.HOLD, 0.5)]
