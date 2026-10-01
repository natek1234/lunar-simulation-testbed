import numpy as np

from testbed.control import placement_skills as PS, quat as Q
from testbed.control.records import Pose


def test_flange_from_tool_inverts_the_offset():
    tool = Pose(np.array([1.0, 2.0, 0.5]), Q.from_rotvec(np.array([0, 0, 0.3])))
    off = Pose(np.array([0.0, 0.0, 0.16]), Q.from_rotvec(np.array([0, 0, -np.pi / 4])))
    fl = PS.flange_from_tool(tool, off)
    back = Pose(fl.pos_m + Q.rotate(fl.quat_wxyz, off.pos_m), Q.mul(fl.quat_wxyz, off.quat_wxyz))
    assert np.allclose(back.pos_m, tool.pos_m) and np.allclose(back.quat_wxyz, tool.quat_wxyz, atol=1e-9)


def test_magnet_grasp_pose_is_top_down_on_the_patch_yaw_aligned():
    unit = Pose(np.array([2.5, 0.0, 0.05]), Q.from_rotvec(np.array([0, 0, 0.3])))
    tool = PS.grasp_tool_pose_W(unit, "magnet", back_m=0.0, lift_m=0.0)
    R = PS._rot(tool.quat_wxyz)
    assert np.allclose(R[:, 2], [0, 0, -1])                        # tool z points down onto the top face
    assert np.allclose(R[:, 0], Q.rotate(unit.quat_wxyz, [1, 0, 0]))  # tool x follows the unit's yaw
    assert np.allclose(tool.pos_m, [2.5, 0.0, 0.05 + 0.05])        # face on the patch (top face centre)
    above = PS.grasp_tool_pose_W(unit, "magnet", back_m=0.3, lift_m=0.0)
    assert np.allclose(above.pos_m, [2.5, 0.0, 0.40])              # `back` retreats along -tool z = up


def test_parallel_grasp_pose_is_an_end_grasp_opening_vertically():
    unit = Pose(np.array([2.5, 0.0, 0.05]), np.array([1.0, 0, 0, 0]))
    tool = PS.grasp_tool_pose_W(unit, "parallel", back_m=0.0, lift_m=0.0)
    R = PS._rot(tool.quat_wxyz)
    assert np.allclose(R[:, 2], [1, 0, 0])          # approach along the unit's +x
    assert np.allclose(np.abs(R[:, 1]), [0, 0, 1])  # fingers open along world z
    assert np.allclose(tool.pos_m, [2.5 - 0.215 + 0.04, 0.0, 0.05])
