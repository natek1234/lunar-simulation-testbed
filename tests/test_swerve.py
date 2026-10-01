import math

import numpy as np

from testbed.control.records import Twist
from testbed.control.swerve import ModuleFirmware, SwerveAllocator

XY = ((0.6, 0.45), (0.6, -0.45), (-0.6, 0.45), (-0.6, -0.45))
R = 0.135
LIM = math.radians(95)


def alloc():
    return SwerveAllocator(XY, R, LIM)


def test_straight_drive_all_wheels_forward():
    cmds = alloc().allocate(Twist(vx_mps=0.5), np.zeros(4))
    assert all(math.isclose(c.steer_rad, 0.0, abs_tol=1e-9) and math.isclose(c.wheel_radps, 0.5 / R) for c in cmds)


def test_strafe_steers_90_deg():
    cmds = alloc().allocate(Twist(vy_mps=0.3), np.zeros(4))
    assert all(math.isclose(c.steer_rad, math.pi / 2) and math.isclose(c.wheel_radps, 0.3 / R) for c in cmds)


def test_spin_in_place_is_tangential():
    cmds = alloc().allocate(Twist(wz_radps=0.5), np.zeros(4))
    for (x, y), c in zip(XY, cmds):
        vx, vy = -0.5 * y, 0.5 * x
        assert math.isclose(math.atan2(vy, vx), c.steer_rad, abs_tol=1e-9) or \
               math.isclose(math.atan2(-vy, -vx), c.steer_rad, abs_tol=1e-9)
        assert math.isclose(abs(c.wheel_radps), math.hypot(vx, vy) / R)


def test_beyond_steer_limit_flips_the_wheel():
    cmds = alloc().allocate(Twist(vx_mps=-0.3, vy_mps=0.3), np.zeros(4))   # 135 deg > 95 deg limit
    for c in cmds:
        assert math.isclose(c.steer_rad, math.radians(-45)) and c.wheel_radps < 0


def test_deadband_holds_steer_and_zeroes_wheels():
    cmds = alloc().allocate(Twist(vx_mps=0.005), np.full(4, 0.7))
    assert all(math.isclose(c.steer_rad, 0.7) and c.wheel_radps == 0.0 for c in cmds)


def test_firmware_slews_steer_at_the_rate_limit():
    fw = ModuleFirmware(steer_rate_max_radps=6.5, dt_s=0.002)
    out = fw.step((SwerveAllocator.ModuleCommand(1.0, 3.0),), np.array([0.0]))
    assert math.isclose(out[0].steer_rad, 0.013) and out[0].wheel_radps < 3.0   # wheel scaled by cos(error)
