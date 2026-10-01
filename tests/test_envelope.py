import dataclasses

import numpy as np
import pytest

from testbed.control import envelope as E
from testbed.control.records import JointAction, Pose, TaskAction, Twist
from tests.test_runtime import make


def test_mount_reference_matches_the_true_arm_base():
    """Review finding 1: `Envelope.mount_pos_B_m` (the reach clamp's centre) and
    `Envelope.reach_m` (the reach bound) must come from the SAME reference point --
    `kin.arm_base_B_m`, the compiled model's true arm-root position -- not the YAML spec's
    `mount_pos_B_m`. The two are numerically equal on this robot (the FR3's root body
    carries no local offset), which is exactly why nothing but this assertion would catch
    a future regression back to sourcing them from two different places."""
    rt = make()
    assert np.array_equal(rt.envelope.mount_pos_B_m, rt.kin.arm_base_B_m)


def test_reach_is_measured_from_the_kinematics():
    rt = make()
    r = E.measure_reach_m(rt.kin)
    # Uniform sampling over the FULL joint-limit box (not just a "typical fully extended"
    # pose) measures a max reach well above the vendor's quoted nominal reach x scale
    # (0.855 x 1.5 = 1.28 m) -- FR3's J5/J6 ranges (+-2.8065 rad / 0.5445..4.5169 rad) admit
    # wrist configurations that extend the flange further from the base than a straight-arm
    # pose does [measured: this test, deterministic at n=2000/seed=0, ~1.77 m;
    # DECISIONS.md D12]. The bound below brackets that measurement, not the nominal figure.
    assert 1.5 < r < 2.0


def test_target_beyond_reach_is_clamped_onto_the_sphere():
    rt = make()
    env = rt.envelope
    far = Pose(np.array(rt.robot.arm.mount_pos_B_m) + np.array([3.0, 0, 0.5]), rt.obs.ee_pose_B.quat_wxyz)
    tgt, tw, rep = env.clamp_task(far, Twist(), rt.obs)
    assert rep.active and "reach" in rep.kinds
    assert np.linalg.norm(tgt.pos_m - rt.robot.arm.mount_pos_B_m) <= env.reach_m * E.REACH_MARGIN + 1e-9


def test_twist_is_capped():
    rt = make()
    _, tw, rep = rt.envelope.clamp_task(None, Twist(vx_mps=5.0), rt.obs)
    assert tw.vx_mps == rt.robot.base.max_vx_mps and "twist" in rep.kinds


def test_clamp_joint_clamps_to_limits_and_reports():
    """Review finding 2: `clamp_joint` had zero direct test coverage."""
    rt = make()
    lo, hi = rt.kin.limits()
    q = hi + 1.0     # every joint commanded 1 rad past its own upper limit
    qc, rep = rt.envelope.clamp_joint(q, rt.obs)
    assert np.allclose(qc, hi)
    assert rep.active and "joint" in rep.kinds
    assert rt.envelope.counts["joint"] == 1
    # a q already within limits reports no intervention
    q_ok = (lo + hi) / 2.0
    qc2, rep2 = rt.envelope.clamp_joint(q_ok, rt.obs)
    assert np.allclose(qc2, q_ok)
    assert not rep2.active and rep2.kinds == ()
    assert rt.envelope.counts["joint"] == 1     # unchanged


def test_joint_action_past_limit_is_clamped_by_the_runtime_and_counted():
    """Review finding 2: a Runtime-level check that a JointAction past a limit actually
    gets clamped end to end (not just that clamp_joint's own logic is correct in
    isolation) and that envelope_active/interventions reflect it."""
    rt = make()
    _, hi = rt.kin.limits()

    class PushJoint:
        name = "push_joint"

        def reset(self, r, s):
            pass

        def act(self, obs):
            return JointAction(q_target_rad=hi + 1.0)

        def done(self, obs):
            return False

    rt.controller = PushJoint()
    rt.run(0.1)
    assert np.allclose(rt.cmd.q_target_rad, hi, atol=1e-9)
    assert rt.obs.envelope_active
    assert rt.interventions > 0


def test_the_force_freeze_never_escapes_reach_or_floor():
    """The frozen force-backoff target bypassed BOTH the reach and floor clamps (an earlier
    fix only caught the floor one). Drive the same push-into-
    ground scenario, recording EVERY frozen pose `clamp_task` ever produces (not just the
    final one -- the freeze can release again by the end of a long run, which would make a
    final-state-only check pass vacuously), and confirm every one sits on or inside both
    bounds."""
    rt = make()
    seen_frozen: list[np.ndarray] = []
    original = rt.envelope.clamp_task

    def _recording_clamp_task(ee_target_B, twist, obs):
        result = original(ee_target_B, twist, obs)
        if rt.envelope._frozen is not None:
            seen_frozen.append(rt.envelope._frozen.pos_m.copy())
        return result

    rt.envelope.clamp_task = _recording_clamp_task

    class Push:
        name = "push"

        def reset(self, r, s):
            pass

        def act(self, obs):
            return TaskAction(ee_target=Pose(obs.ee_pose_B.pos_m * [1, 1, 0] + [0, 0, -0.5], obs.ee_pose_B.quat_wxyz))

        def done(self, obs):
            return False

    rt.controller = Push()
    rt.run(6.0)
    assert rt.envelope.counts.get("force", 0) > 0
    assert len(seen_frozen) > 0
    for p in seen_frozen:
        assert p[2] >= E.FLOOR_Z_B_M - 1e-9
        assert np.linalg.norm(p - rt.envelope.mount_pos_B_m) <= rt.envelope.reach_m * E.REACH_MARGIN + 1e-9


def test_pressing_into_the_ground_trips_the_force_limit_and_backs_off():
    rt = make()

    class Push:
        name = "push"

        def reset(self, r, s):
            pass

        def act(self, obs):
            return TaskAction(ee_target=Pose(obs.ee_pose_B.pos_m * [1, 1, 0] + [0, 0, -0.5], obs.ee_pose_B.quat_wxyz))

        def done(self, obs):
            return False

    rt.controller = Push()
    rt.run(6.0)
    assert rt.envelope.counts.get("force", 0) > 0
    assert np.linalg.norm(rt.obs.ft_flange_B[:3]) < rt.robot.force_budget_n * 1.5


def _hazard_obs(rt, force_n):
    """`rt.obs` with a synthetic flange force of `force_n` along +z (the envelope only reads
    ee_pose_B and ft_flange_B off the observation, so this is all a hazard needs)."""
    return dataclasses.replace(rt.obs, ft_flange_B=np.array([0.0, 0.0, force_n, 0.0, 0.0, 0.0]))


def test_the_envelope_filters_the_command_and_never_rewrites_the_standing_target():
    """Regression for DECISIONS.md D15 (final review): `Runtime._resolve_task` used to store
    `clamp_task`'s OUTPUT as the standing EE target, so a force freeze's per-tick back-off
    (and any reach/floor clamp) outlived the hazard -- a one-way ratchet that dragged the
    carried unit along the ground in the single-placement run. The controller's intended target is the standing
    command; the envelope's output goes to IK for that tick only."""
    rt = make()
    intended = Pose(rt.obs.ee_pose_B.pos_m + np.array([0.05, 0.0, 0.0]), rt.obs.ee_pose_B.quat_wxyz)
    action = TaskAction(ee_target=intended, ee_frame="B")

    for _ in range(20):                                 # a sustained over-budget reading
        cmd = rt._resolve_task(action, _hazard_obs(rt, 10.0 * rt.robot.force_budget_n))
        assert rt.envelope.counts.get("force", 0) > 0
        assert np.array_equal(rt.ee_target_B.pos_m, intended.pos_m)          # the standing target is untouched...
        assert not np.allclose(cmd.ee_target_B.pos_m, intended.pos_m)        # ...while the frozen back-off is what IK gets
    backed_off = cmd.ee_target_B.pos_m.copy()
    assert np.linalg.norm(backed_off - intended.pos_m) > 0.05               # the back-off really accumulated (20 x 5 mm)

    cmd = rt._resolve_task(action, _hazard_obs(rt, 0.0))                     # hazard gone
    assert np.array_equal(cmd.ee_target_B.pos_m, intended.pos_m)             # command resumes from the INTENDED target, no ratchet
    assert rt.envelope._frozen is None


def test_a_reach_clamped_target_is_clamped_for_the_tick_not_forever():
    rt = make()
    far = Pose(np.array(rt.robot.arm.mount_pos_B_m) + np.array([3.0, 0, 0.5]), rt.obs.ee_pose_B.quat_wxyz)
    cmd = rt._resolve_task(TaskAction(ee_target=far, ee_frame="B"), rt.obs)
    assert np.linalg.norm(cmd.ee_target_B.pos_m - rt.envelope.mount_pos_B_m) <= rt.envelope.reach_m * E.REACH_MARGIN + 1e-9
    assert np.array_equal(rt.ee_target_B.pos_m, far.pos_m)                   # the standing target is still the far one
    near = Pose(rt.obs.ee_pose_B.pos_m.copy(), rt.obs.ee_pose_B.quat_wxyz)
    cmd = rt._resolve_task(TaskAction(ee_target=near, ee_frame="B"), rt.obs)
    assert np.array_equal(cmd.ee_target_B.pos_m, near.pos_m)                  # a later in-reach target passes through untouched


def test_a_command_inside_the_envelope_passes_through_and_counts_no_intervention():
    """`envelope_active` / `interventions` mean "the envelope CHANGED the output": an in-reach, above-floor
    target with an in-cap twist and no force must come back unchanged, report nothing and count nothing."""
    rt = make()
    target = Pose(rt.obs.ee_pose_B.pos_m.copy(), rt.obs.ee_pose_B.quat_wxyz)
    counts_before = dict(rt.envelope.counts)
    tgt, tw, rep = rt.envelope.clamp_task(target, Twist(vx_mps=0.1), rt.obs)
    assert not rep.active and rep.kinds == ()
    assert np.array_equal(tgt.pos_m, target.pos_m) and tw == Twist(vx_mps=0.1)
    assert rt.envelope.counts == counts_before
    cmd = rt._resolve_task(TaskAction(ee_target=target, ee_frame="B", base_twist_B=Twist(vx_mps=0.1)), rt.obs)
    assert rt._env_active is False and rt.interventions == sum(counts_before.values()) and cmd.twist_B == Twist(vx_mps=0.1)
