"""Resolved-rate IK: damped least squares with a null-space posture term.
Rate saturation is ONE scalar on the whole qd vector (per-joint clipping rotates the
realized direction); joint stops are an ACTIVE SET (a joint that would be driven past
its stop is frozen and the rest re-solved), never a clip after the fact -- both are
lessons carried from an earlier MuJoCo build of this same platform (see docs/design.md's
measured MuJoCo pitfalls section: a clip applied after solving is not the same thing as a
constraint the solve respects; measured there as motion perpendicular to, or reversed
from, the command)."""
from dataclasses import dataclass

import numpy as np

from testbed.control import quat as Q
from testbed.control.kinematics import Kinematics
from testbed.control.records import Pose


@dataclass(frozen=True)
class IkConfig:
    kp_pos: float = 5.0            # [policy default: 1/s proportional gain on position error]
    kp_rot: float = 3.0            # [policy default]
    v_max_mps: float = 0.5         # [policy default: task-space speed cap]
    w_max_radps: float = 1.0       # [policy default]
    damping: float = 0.05          # [policy default: DLS lambda, m-equivalent]
    max_qd_radps: float = 1.5      # [policy default: joint rate cap, below FR3's 2.6 rad/s]
    posture_gain: float = 0.5      # [policy default: null-space pull toward home, 1/s]
    # Re-verified 2026-09-20 against the active-set test scenario actually committed:
    # sweeping posture_gain from 0.1 to 2.0 gives a BIT-IDENTICAL result for 0.1-0.7
    # (same active set, same realized displacement to 15 significant digits), because
    # the null-space posture term for the still-free joints is
    # posture_gain * (home[free] - q[free]), which is exactly zero on that scenario --
    # it only perturbs q[3] off of home, and joint 3 itself is the one that gets frozen.
    # Only posture_gain >= ~1.0 changes anything, and what it changes is unsurprising
    # (the posture pull becomes large enough to cancel the predicted stop violation, so
    # the active set comes out empty) -- not a direction reversal. An earlier draft of
    # this file lowered the default to 0.2 and tagged it as measured, claiming 0.5
    # "reversed the realized flange direction" on this test; that claim did not
    # reproduce and has been retracted -- nothing here forces a value other than 0.5.
    stop_margin_rad: float = 0.01  # [policy default: how close to a stop counts as ON it]


@dataclass(frozen=True)
class IkStep:
    q_next: np.ndarray
    err_pos_m: float
    err_rot_rad: float
    active: tuple[int, ...]
    saturation: float              # 1.0 = unsaturated; >1 = the scalar the vector was divided by


class ResolvedRateIk:
    def __init__(self, kin: Kinematics, home_q: np.ndarray, cfg: IkConfig):
        self.kin, self.home, self.cfg = kin, np.asarray(home_q, dtype=float), cfg

    def step(self, q: np.ndarray, target: Pose, dt_s: float) -> IkStep:
        q = np.asarray(q, dtype=float)
        cur = self.kin.fk(q)
        e_pos = target.pos_m - cur.pos_m
        e_rot = Q.to_rotvec(Q.mul(target.quat_wxyz, Q.conj(cur.quat_wxyz)))
        v = _cap(self.cfg.kp_pos * e_pos, self.cfg.v_max_mps)
        w = _cap(self.cfg.kp_rot * e_rot, self.cfg.w_max_radps)
        twist = np.concatenate([v, w])
        J = self.kin.jacobian(q)
        lo, hi = self.kin.limits()
        n = self.kin.n
        active: list[int] = []
        qd = np.zeros(n)
        for _ in range(n + 1):
            free = [k for k in range(n) if k not in active]
            qd = np.zeros(n)
            if free:
                Jf = J[:, free]
                JJt = Jf @ Jf.T + (self.cfg.damping ** 2) * np.eye(6)
                qd_task = Jf.T @ np.linalg.solve(JJt, twist)
                N = np.eye(len(free)) - Jf.T @ np.linalg.solve(JJt, Jf)
                qd_post = N @ (self.cfg.posture_gain * (self.home[free] - q[free]))
                qd[free] = qd_task + qd_post
            q_try = q + qd * dt_s
            newly = [k for k in free if (q_try[k] < lo[k] + self.cfg.stop_margin_rad and qd[k] < 0)
                     or (q_try[k] > hi[k] - self.cfg.stop_margin_rad and qd[k] > 0)]
            if not newly:
                break
            active.extend(newly)
        s = max(1.0, float(np.max(np.abs(qd)) / self.cfg.max_qd_radps)) if n else 1.0
        qd = qd / s
        q_next = np.clip(q + qd * dt_s, lo, hi)
        return IkStep(q_next=q_next, err_pos_m=float(np.linalg.norm(e_pos)),
                      err_rot_rad=float(np.linalg.norm(e_rot)), active=tuple(sorted(active)), saturation=s)


def _cap(v: np.ndarray, vmax: float) -> np.ndarray:
    n = np.linalg.norm(v)
    return v if n <= vmax else v * (vmax / n)
