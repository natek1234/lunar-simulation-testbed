import math

import mujoco
import numpy as np
import pytest

from testbed.sim import compose as C
from testbed.spec import robot as R
from testbed.spec import surface as S


@pytest.fixture(scope="module")
def compiled():
    return C.compose(R.load_robot(R.ROBOTS_DIR / "fr3_mpo700.yaml"),
                      S.load_surface(S.SURFACES_DIR / "flat.yaml"))


@pytest.fixture(scope="module")
def compiled_hand():
    return C.compose(R.load_robot(R.ROBOTS_DIR / "fr3_mpo700_hand.yaml"),
                      S.load_surface(S.SURFACES_DIR / "flat.yaml"))


def test_singletons_are_what_we_wrote_not_the_vendored_ones(compiled):
    m = compiled.model
    assert m.opt.timestep == C.TIMESTEP_S          # vendored mpo_700.xml says 0.005
    assert np.allclose(m.opt.gravity, [0, 0, -1.62])
    assert m.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST


def test_names_survive_attach(compiled):
    m = compiled.model
    for k in range(1, 8):
        assert m.joint(f"fr3_joint{k}").id >= 0
    assert m.site("gripper_interface").id >= 0


def test_magnet_is_an_authored_body_on_the_flange_with_no_actuator(compiled):
    m = compiled.model
    mag = m.body("magnet")
    assert abs(m.body_mass[mag.id] - 1.0) < 1e-6
    assert m.nu == 8 + 7                                    # 4 steer + 4 drive + 7 arm; the magnet adds none
    assert m.body_parentid[mag.id] == m.body("fr3_link7").id


def test_hand_variant_is_prefixed_and_mounted_at_minus_45_deg_yaw(compiled_hand):
    m = compiled_hand.model
    assert m.actuator("hand_actuator8").id >= 0 and m.site("gripper_interface").id >= 0
    assert np.allclose(m.body("hand_hand").quat, [0.9238795, 0, 0, -0.3826834], atol=1e-6)
    assert m.nu == 8 + 7 + 1


def test_rover_scale_applied(compiled):
    m = compiled.model
    fl = m.body("mpo_700_caster_front_left_link")
    assert np.allclose(fl.pos[:2], [0.6, 0.45])
    wheel = m.body("mpo_700_wheel_front_left_link")
    geom_ids = [g for g in range(m.ngeom) if m.geom_bodyid[g] == wheel.id and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE]
    assert len(geom_ids) == 1 and math.isclose(m.geom_size[geom_ids[0]][0], 0.135)


def test_base_actuators_wired_to_the_right_joints(compiled):
    m = compiled.model
    for mod, stem in (("fl", "front_left"), ("br", "back_right")):
        assert m.actuator_trnid[m.actuator(f"steer_{mod}").id][0] == m.joint(f"mpo_700_caster_{stem}_joint").id
        assert m.actuator_trnid[m.actuator(f"drive_{mod}").id][0] == m.joint(f"mpo_700_wheel_{stem}_joint").id


def test_arm_reach_scaled(compiled):
    m = compiled.model
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    link0 = d.body("fr3_link0").xpos
    flange = d.site("attachment_site").xpos
    # At q=0 the vendored (unscaled) FR3 flange sits 0.9260 m above link0 -- measured via
    # mj_forward on the standalone fr3.xml at q=0 [measured: scratch probe, 2026-09-18].
    # The naive DH-length sum 0.333+0.316+0.384+0.107=1.14 m is the arm's max STRETCHED
    # reach, only realized in specific configurations, not at q=0 -- confirmed by comparing
    # against the unscaled chain directly, so this is the scaled q=0 baseline, not a
    # tolerance widened to cover a bug.
    assert math.isclose(flange[2] - link0[2], 0.9260 * 1.5, rel_tol=0.02)


def test_robot_settles_on_the_flat_surface(compiled):
    m = compiled.model
    d = mujoco.MjData(m)
    d.qpos[2] = compiled.robot.base.spawn_z_W_m
    for _ in range(1500):
        mujoco.mj_step(m, d)
    assert 0.0 < d.qpos[2] < 0.12          # resting on its wheels, not fallen through
    assert abs(d.qvel[:6]).max() < 0.01    # at rest
