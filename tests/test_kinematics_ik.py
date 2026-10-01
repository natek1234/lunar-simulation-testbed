import numpy as np
import pytest

from testbed.control import ik as IK, quat as Q
from testbed.control.records import Pose
from testbed.sim import compose as C, handles as H
from testbed.sim.kinematics_mujoco import MujocoKinematics
from testbed.spec import robot as R, surface as S


@pytest.fixture(scope="module")
def h():
    return H.resolve(C.compose(R.load_robot(R.ROBOTS_DIR / "fr3_mpo700.yaml"), S.load_surface(S.SURFACES_DIR / "flat.yaml")))


@pytest.fixture(scope="module")
def kin(h):
    return MujocoKinematics(h)


def test_fk_matches_finite_difference_jacobian(kin):
    rng = np.random.default_rng(0)
    lo, hi = kin.limits()
    q = lo + rng.random(kin.n) * (hi - lo)
    J = kin.jacobian(q)
    eps = 1e-6
    for k in range(kin.n):
        dq = np.zeros(kin.n); dq[k] = eps
        dp = (kin.fk(q + dq).pos_m - kin.fk(q - dq).pos_m) / (2 * eps)
        assert np.allclose(J[:3, k], dp, atol=1e-4), k


def test_ik_reaches_a_target_10cm_from_home(h, kin):
    home = np.array(h.compiled.robot.arm.home_q_rad)
    solver = IK.ResolvedRateIk(kin, home, IK.IkConfig())
    start = kin.fk(home)
    target = Pose(start.pos_m + np.array([0.0, 0.10, -0.05]), start.quat_wxyz)
    q = home.copy()
    for _ in range(400):
        q = solver.step(q, target, 0.02).q_next
    assert np.linalg.norm(kin.fk(q).pos_m - target.pos_m) < 0.005
    assert np.linalg.norm(Q.to_rotvec(Q.mul(target.quat_wxyz, Q.conj(kin.fk(q).quat_wxyz)))) < 0.02


def test_saturation_keeps_the_commanded_direction(h, kin):
    home = np.array(h.compiled.robot.arm.home_q_rad)
    fast = IK.ResolvedRateIk(kin, home, IK.IkConfig(max_qd_radps=100.0))
    slow = IK.ResolvedRateIk(kin, home, IK.IkConfig(max_qd_radps=0.2))
    start = kin.fk(home)
    target = Pose(start.pos_m + np.array([0.3, 0.3, 0.0]), start.quat_wxyz)
    d_fast = kin.fk(fast.step(home, target, 0.02).q_next).pos_m - start.pos_m
    d_slow = kin.fk(slow.step(home, target, 0.02).q_next).pos_m - start.pos_m
    cos = d_fast @ d_slow / (np.linalg.norm(d_fast) * np.linalg.norm(d_slow))
    assert cos > 0.99 and np.linalg.norm(d_slow) < np.linalg.norm(d_fast)


def _naive_clip_after_solve(kin, home, cfg, q, target, dt_s):
    """The thing ResolvedRateIk.step deliberately does NOT do: solve once with every
    joint assumed free, then clip q_next into range afterward. Used only as a comparison
    baseline in test_joint_on_its_stop_joins_the_active_set, to confirm the tightened
    direction assertion there actually discriminates real active-set handling from this."""
    q = np.asarray(q, dtype=float)
    cur = kin.fk(q)
    e_pos = target.pos_m - cur.pos_m
    e_rot = Q.to_rotvec(Q.mul(target.quat_wxyz, Q.conj(cur.quat_wxyz)))
    v = IK._cap(cfg.kp_pos * e_pos, cfg.v_max_mps)
    w = IK._cap(cfg.kp_rot * e_rot, cfg.w_max_radps)
    twist = np.concatenate([v, w])
    J = kin.jacobian(q)
    n = kin.n
    JJt = J @ J.T + (cfg.damping ** 2) * np.eye(6)
    qd_task = J.T @ np.linalg.solve(JJt, twist)
    N = np.eye(n) - J.T @ np.linalg.solve(JJt, J)
    qd_post = N @ (cfg.posture_gain * (home - q))
    qd = qd_task + qd_post
    s = max(1.0, float(np.max(np.abs(qd)) / cfg.max_qd_radps))
    qd = qd / s
    lo, hi = kin.limits()
    return np.clip(q + qd * dt_s, lo, hi)


def test_joint_on_its_stop_joins_the_active_set(h, kin):
    # A first-draft scenario (J4 at its LOWER stop, target +x) assumed a generic FR3;
    # on THIS composed model (swerve-base mount + 1.5x arm scale) it doesn't hold -- at
    # that pinned pose the Jacobian column's raw partial d(flange_x)/d(q4) is +0.71
    # [measured: scratch probe, 2026-09-18], i.e. reaching +x wants q4 to move AWAY from
    # its lower stop, not past it, so nothing should (and nothing does) join the active
    # set there. A sweep over all 7 joints x both stops x the 6 axis directions, checking
    # the sign of each Jacobian column's projection onto each candidate target direction,
    # found J4's UPPER stop with a -x target as the equivalent, empirically-verified
    # case: that projection (Jacobian column [3] dotted with the target direction [-1,0,0])
    # is +0.694 there -- equivalently, the raw partial d(flange_x)/d(q4) is -0.694 (same
    # magnitude, opposite sign, since the target direction itself is -x) -- so a -x target
    # wants q4 to increase past its UPPER stop.
    #
    # offset=0.05 (not pinned right at the stop) and dt=2.0 (not a real control-loop tick)
    # are deliberately coarse: at the tick-realistic offset=1e-4/dt=0.02 the correct
    # active-set solve and a naive clip-after-solve are numerically indistinguishable
    # here (both land at cos ~ 0.997-0.998 against the target direction, a gap of ~0.001
    # -- measured 2026-09-20) because the predicted overshoot past the
    # stop is only ~1e-4 rad either way, so a bare sign check passes for BOTH the real
    # implementation and a naive one -- it wasn't discriminating anything. Backing q4 off
    # the stop and taking one large step makes the overshoot -- and therefore the gap
    # between "freeze and re-solve" and "solve assuming free, then clip" -- large enough
    # to actually test something: swept dt in [1.8, 2.2] x offset in [0.03, 0.07], the
    # real solve stays at cos > 0.998 and naive clip-after-solve stays at cos < 0.951
    # throughout, so this is not a knife-edge choice of dt/offset. At this displacement
    # J6 (index 5) also joins the active set alongside J4 -- the test only requires J4's
    # membership, not exclusivity.
    lo, hi = kin.limits()
    home = np.array(h.compiled.robot.arm.home_q_rad)
    q = home.copy(); q[3] = hi[3] - 0.05           # J4 pinned 0.05 rad off its upper stop
    cfg = IK.IkConfig()
    solver = IK.ResolvedRateIk(kin, home, cfg)
    start = kin.fk(q)
    d = np.array([-1.0, 0.0, 0.0])
    target = Pose(start.pos_m + 0.3 * d, start.quat_wxyz)   # reach out: wants J4 past its stop
    dt_s = 2.0

    step = solver.step(q, target, dt_s)
    assert 3 in step.active
    realized = kin.fk(step.q_next).pos_m - start.pos_m
    cos_real = realized @ d / np.linalg.norm(realized)
    assert cos_real > 0.97     # the others still move the flange the right way

    # Confirm the 0.97 threshold actually discriminates: a naive clip-after-solve
    # implementation, given the exact same inputs, must FAIL it.
    q_naive_next = _naive_clip_after_solve(kin, home, cfg, q, target, dt_s)
    realized_naive = kin.fk(q_naive_next).pos_m - start.pos_m
    cos_naive = realized_naive @ d / np.linalg.norm(realized_naive)
    assert cos_naive < 0.97
