"""The runtime shield outside every controller (envelope enforcement,
docs/reference/requirements.md): reach + workspace clamp on the EE target, twist caps,
joint limits, and a flange-load limit that freezes the target and backs it off along the
sensed force. Every intervention is counted by kind.

`testbed.control` never imports mujoco (tests/test_architecture.py), so this module knows
nothing about the compiled model -- it only ever sees the `Kinematics` protocol (fk/
jacobian/limits) and the seam's own records."""
from dataclasses import dataclass

import numpy as np

from testbed.control.kinematics import Kinematics
from testbed.control.records import Observation, Pose, Twist

REACH_MARGIN = 0.95          # [policy default: keep targets inside 95 % of measured reach]
FLOOR_Z_B_M = -0.08          # [policy default: EE target may not go below the wheel-contact plane + margin, in B]
FORCE_BACKOFF_M = 0.005      # [policy default: per-tick back-off when over budget]
FORCE_RELEASE_FRAC = 0.7     # [policy default: hysteresis, release the freeze below 70 % of budget]
KINDS = ("twist", "reach", "floor", "force", "joint")   # the closed vocabulary of intervention kinds `_hit` may record; the episode's `interventions_by_kind` column follows this order


@dataclass(frozen=True)
class EnvelopeReport:
    active: bool
    kinds: tuple[str, ...]


def _arm_base(kin: Kinematics) -> np.ndarray:
    return np.asarray(getattr(kin, "arm_base_B_m", np.zeros(3)), dtype=float)


def measure_reach_m(kin: Kinematics, n: int = 2000, seed: int = 0) -> float:
    """Max |flange - arm base| over uniformly sampled joint configurations
    [measured: testbed.control.envelope.measure_reach_m against the x1.5 FR3]."""
    rng = np.random.default_rng(seed)
    lo, hi = kin.limits()
    base = _arm_base(kin)
    best = 0.0
    for _ in range(n):
        q = lo + rng.random(kin.n) * (hi - lo)
        best = max(best, float(np.linalg.norm(kin.fk(q).pos_m - base)))
    return best


class Envelope:
    """Clamps a controller's requested task (`clamp_task`) and joint (`clamp_joint`)
    commands to the robot's physical envelope. Stateful across ticks: the force limit's
    freeze-and-back-off remembers the last commanded (backed-off) pose in `self._frozen`,
    and `self.counts` accumulates intervention counts by kind for the runtime's own
    cumulative `interventions` total."""

    def __init__(self, robot, kin: Kinematics, force_backoff_sign: float = -1.0):
        # force_backoff_sign default is -1.0, NOT the naive +1.0 -- measured
        # [DECISIONS.md D12]: with +1.0 (back off ALONG the raw sensor vector `f`), a
        # sustained push into the ground diverged (|F| climbed past 200 N, backing the
        # target further in, not out); with -1.0 the same scenario settles (|F| -> 26.5 N
        # after 6 s, deterministic). `testbed/sim/handles.py::ft_flange_B` reports the
        # sensor's own (site-frame, then rotated to B) reading, whose sign convention here
        # is such that backing off OPPOSITE the raw vector is the one that reduces contact
        # force -- do not flip this back to +1.0 without re-measuring.
        self.robot, self.kin = robot, kin
        # `self.mount_pos_B_m` and `self.reach_m` MUST share one reference point --
        # `_arm_base(kin)`, the compiled model's true arm-root position -- not
        # `robot.arm.mount_pos_B_m` (the YAML spec value). On this robot the two are
        # numerically identical (the FR3's root body carries no local offset of its own),
        # which is exactly why an earlier version of this file used the spec value here
        # while `measure_reach_m` (below) already used `_arm_base(kin)`: nothing enforced
        # they'd stay equal, and a future arm whose root body DOES carry an offset would
        # silently get a reach clamp centered on the wrong point while the reach BOUND
        # itself stayed correct. `test_mount_reference_matches_the_true_arm_base` pins this.
        self.mount_pos_B_m = _arm_base(kin)
        self.reach_m = measure_reach_m(kin)
        self.sign = force_backoff_sign
        self.counts: dict[str, int] = {}
        self._frozen: Pose | None = None

    def reset(self) -> None:
        """Clear per-episode state (the force freeze and the intervention counts) --
        called by Runtime.reset() so a fresh episode starts with a clean envelope, not one
        still carrying a previous episode's freeze or counts."""
        self.counts = {}
        self._frozen = None

    def _hit(self, kinds: list[str], k: str) -> None:
        kinds.append(k)
        self.counts[k] = self.counts.get(k, 0) + 1

    def _clamp_reach_floor(self, p: np.ndarray, kinds: list[str]) -> np.ndarray:
        """Reach-then-floor clamp on a raw position, in place against `kinds`. Shared by
        the ordinary (non-frozen) path AND the force freeze's own backed-off pose -- see
        `clamp_task`'s docstring note on why the frozen pose must be re-clamped here too,
        not just computed once and trusted forever."""
        p = p.copy()
        r = np.linalg.norm(p - self.mount_pos_B_m)
        if r > self.reach_m * REACH_MARGIN:
            p = self.mount_pos_B_m + (p - self.mount_pos_B_m) * (self.reach_m * REACH_MARGIN / r)
            self._hit(kinds, "reach")
        if p[2] < FLOOR_Z_B_M:
            p[2] = FLOOR_Z_B_M
            self._hit(kinds, "floor")
        return p

    def clamp_task(self, ee_target_B: Pose | None, twist: Twist, obs: Observation):
        """`self._frozen` (the force freeze's own backed-off pose) is re-clamped through
        `_clamp_reach_floor` EVERY tick, not just derived once from an already-clamped
        base -- the backoff step (`back`, below) is a raw vector added on top of the
        previous frozen pose with no bound of its own, so without this the frozen pose
        could walk arbitrarily far past reach or below the floor over many ticks even
        though the ordinary (non-frozen) path enforces both. Measured benign in the
        shipped `Push`-into-ground scenario (the frozen pose stayed near the floor even
        before this fix), but nothing enforced that in general -- see DECISIONS.md D12's
        follow-up and the review that caught it."""
        kinds: list[str] = []
        b = self.robot.base
        tw = Twist(float(np.clip(twist.vx_mps, -b.max_vx_mps, b.max_vx_mps)),
                   float(np.clip(twist.vy_mps, -b.max_vy_mps, b.max_vy_mps)),
                   float(np.clip(twist.wz_radps, -b.max_wz_radps, b.max_wz_radps)))
        if (tw.vx_mps, tw.vy_mps, tw.wz_radps) != (twist.vx_mps, twist.vy_mps, twist.wz_radps):
            self._hit(kinds, "twist")
        f = obs.ft_flange_B[:3]
        fn = float(np.linalg.norm(f))
        budget = self.robot.force_budget_n
        if fn > budget or (self._frozen is not None and fn > FORCE_RELEASE_FRAC * budget):
            back = self.sign * f / max(fn, 1e-9) * FORCE_BACKOFF_M
            base = self._frozen.pos_m if self._frozen is not None else obs.ee_pose_B.pos_m
            frozen_p = self._clamp_reach_floor(base + back, kinds)
            self._frozen = Pose(frozen_p, obs.ee_pose_B.quat_wxyz.copy())
            tgt = self._frozen
            self._hit(kinds, "force")
        else:
            self._frozen = None
            tgt = ee_target_B
            if tgt is not None:
                tgt = Pose(self._clamp_reach_floor(tgt.pos_m, kinds), tgt.quat_wxyz)
        return tgt, tw, EnvelopeReport(bool(kinds), tuple(kinds))

    def clamp_joint(self, q: np.ndarray, obs: Observation):
        lo, hi = self.kin.limits()
        qc = np.clip(q, lo, hi)
        kinds: list[str] = []
        if not np.array_equal(qc, q):
            self._hit(kinds, "joint")
        return qc, EnvelopeReport(bool(kinds), tuple(kinds))
