import numpy as np
import pytest

from testbed.sim import compose as C, handles as H
from testbed.spec import robot as R, surface as S


@pytest.fixture(scope="module")
def h():
    return H.resolve(C.compose(R.load_robot(R.ROBOTS_DIR / "fr3_mpo700.yaml"),
                                S.load_surface(S.SURFACES_DIR / "flat.yaml")))


def test_handles_resolve_every_name(h):
    assert len(h.modules) == 4 and h.arm_qpos.shape == (7,) and h.arm_act.shape == (7,)
    assert h.gripper_act is None and h.finger_qpos.shape == (0,) and h.ft_force_adr >= 0 and h.interface_site >= 0
    assert h.model.body(h.gripper_body).name == "magnet"


def test_hand_variant_resolves_its_actuator_and_fingers():
    hh = H.resolve(C.compose(R.load_robot(R.ROBOTS_DIR / "fr3_mpo700_hand.yaml"), S.load_surface(S.SURFACES_DIR / "flat.yaml")))
    assert hh.gripper_act is not None and hh.finger_qpos.shape == (2,)


def test_missing_name_fails_loudly(h):
    bad = R.load_robot(R.ROBOTS_DIR / "fr3_mpo700.yaml")
    bad = R.RobotSpec(**{**bad.__dict__, "arm": R.ArmSpec(**{**bad.arm.__dict__, "flange_site": "nope"})})
    with pytest.raises(KeyError, match="nope"):
        H.resolve(C.Compiled(h.model, bad, h.compiled.surface))


def test_module_order_follows_the_spec(h):
    for mh, mod in zip(h.modules, h.compiled.robot.base.modules):
        assert h.model.joint(h.model.dof_jntid[mh.steer_dof]).name == mod.steer_joint
