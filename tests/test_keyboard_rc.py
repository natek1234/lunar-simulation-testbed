import numpy as np

from testbed.control.keyboard_rc import KeyState, KeyboardRc
from testbed.control.records import GripperCmd, JointAction, TaskAction
from testbed.spec import robot as R
from tests.test_runtime import make


def keys(*held, pressed=(), focused=True):
    return KeyState(frozenset(held), frozenset(pressed), focused)


def test_w_drives_forward_at_the_cap_and_release_brakes():
    rt = make()
    rc = KeyboardRc(rt.robot)
    rc.update(keys("W"))
    a = rc.act(rt.obs)
    assert isinstance(a, TaskAction) and a.base_twist_B.vx_mps == rt.robot.base.max_vx_mps
    rc.update(keys())
    assert rc.act(rt.obs).base_twist_B.vx_mps == 0.0


def test_focus_loss_is_a_deadman():
    rt = make()
    rc = KeyboardRc(rt.robot)
    rc.update(keys("W", focused=False))
    assert rc.act(rt.obs).base_twist_B.vx_mps == 0.0


def test_ee_jog_moves_the_target_in_B():
    rt = make()
    rc = KeyboardRc(rt.robot)
    rc.update(keys("U"))
    a = rc.act(rt.obs)
    assert a.ee_target is not None and a.ee_frame == "B"
    assert a.ee_target.pos_m[2] > rt.obs.ee_pose_B.pos_m[2]


def test_m_toggles_joint_mode_and_digits_select_a_joint():
    rt = make()
    rc = KeyboardRc(rt.robot)
    rc.update(keys(pressed=("M",)))
    rc.act(rt.obs)
    assert rc.mode == "joint"
    rc.update(keys("=", pressed=("3",)))
    a = rc.act(rt.obs)
    assert isinstance(a, JointAction) and rc.selected_joint == 2
    assert a.q_target_rad[2] > rt.obs.q_rad[2]


def test_space_toggles_gripper_and_x_stops():
    rt = make()
    rc = KeyboardRc(rt.robot)
    rc.update(keys(pressed=("SPACE",)))
    assert rc.act(rt.obs).gripper is GripperCmd.CLOSE
    rc.update(keys(pressed=("X",)))
    assert rc.act(rt.obs).stop
