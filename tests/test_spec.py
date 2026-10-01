import math

import pytest

from testbed.spec import robot as R
from testbed.spec import surface as S


def test_fr3_mpo700_loads_and_is_frozen():
    spec = R.load_robot(R.ROBOTS_DIR / "fr3_mpo700.yaml")
    assert spec.name == "fr3_mpo700"
    assert spec.base.kind == "swerve"
    assert [m.name for m in spec.base.modules] == ["fl", "fr", "bl", "br"]
    assert len(spec.arm.joints) == 7 and len(spec.arm.home_q_rad) == 7
    assert spec.gripper.kind == "magnet" and spec.gripper.magnet.mass_kg == 1.0
    with pytest.raises(Exception):
        spec.name = "x"  # frozen


def test_hand_variant_differs_only_in_the_gripper_block():
    a = R.load_robot(R.ROBOTS_DIR / "fr3_mpo700.yaml")
    b = R.load_robot(R.ROBOTS_DIR / "fr3_mpo700_hand.yaml")
    assert b.gripper.kind == "parallel" and b.gripper.finger_joints == ("hand_finger_joint1", "hand_finger_joint2")
    assert (a.base, a.arm, a.sensors) == (b.base, b.arm, b.sensors)


def test_module_geometry_is_rover_scale():
    spec = R.load_robot(R.ROBOTS_DIR / "fr3_mpo700.yaml")
    fl = spec.base.modules[0]
    assert fl.xy_B_m == (0.6, 0.45)
    assert math.isclose(fl.wheel_radius_m, 0.135)


def test_validation_rejects_mismatched_arm_lengths(tmp_path):
    src = (R.ROBOTS_DIR / "fr3_mpo700.yaml").read_text(encoding="utf-8")
    bad = tmp_path / "bad.yaml"
    bad.write_text(src.replace("home_q_rad: [0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853]",
                               "home_q_rad: [0.0, 0.0]"), encoding="utf-8")
    with pytest.raises(ValueError, match="home_q_rad"):
        R.load_robot(bad)


def test_flat_surface_loads():
    s = S.load_surface(S.SURFACES_DIR / "flat.yaml")
    assert s.kind == "flat" and math.isclose(s.gravity_mps2, 1.62)
