"""Swerve kinematics for a holonomic base of N steered/driven modules.
allocate(): body twist -> per-module (steer angle, wheel rate) with steer-limit flipping.
ModuleFirmware: slews the steer command at the actuator's rate limit and scales the wheel
rate by cos(steer error) so a module never drives while pointing the wrong way."""
from dataclasses import dataclass

import numpy as np

from testbed.control.records import Twist


@dataclass(frozen=True)
class ModuleCommand:
    steer_rad: float
    wheel_radps: float


class SwerveAllocator:
    ModuleCommand = ModuleCommand

    def __init__(self, module_xy_B_m, wheel_radius_m: float, steer_limit_rad: float,
                 hold_speed_mps: float = 0.02, hold_yaw_radps: float = 0.05):
        # hold_* [policy default: command deadband below which modules hold and wheels stop]
        self.xy = np.asarray(module_xy_B_m, dtype=float)
        self.r = wheel_radius_m
        self.limit = steer_limit_rad
        self.hold_v, self.hold_w = hold_speed_mps, hold_yaw_radps

    def allocate(self, twist: Twist, current_steer_rad: np.ndarray) -> tuple[ModuleCommand, ...]:
        if np.hypot(twist.vx_mps, twist.vy_mps) < self.hold_v and abs(twist.wz_radps) < self.hold_w:
            return tuple(ModuleCommand(float(s), 0.0) for s in current_steer_rad)
        out = []
        for (x, y), cur in zip(self.xy, current_steer_rad):
            vx = twist.vx_mps - twist.wz_radps * y
            vy = twist.vy_mps + twist.wz_radps * x
            speed = float(np.hypot(vx, vy))
            ang = float(np.arctan2(vy, vx))
            # candidate: as computed, or flipped 180 deg with the wheel reversed; keep the one
            # inside the steer limit, preferring the one nearest the current angle
            cands = [(ang, speed), (_wrap(ang + np.pi), -speed)]
            legal = [c for c in cands if abs(c[0]) <= self.limit] or [min(cands, key=lambda c: abs(c[0]))]
            a, s = min(legal, key=lambda c: abs(_wrap(c[0] - cur)))
            out.append(ModuleCommand(a, s / self.r))
        return tuple(out)


class ModuleFirmware:
    def __init__(self, steer_rate_max_radps: float, dt_s: float):
        self.rate, self.dt = steer_rate_max_radps, dt_s

    def step(self, targets, current_steer_rad: np.ndarray) -> tuple[ModuleCommand, ...]:
        out = []
        for t, cur in zip(targets, current_steer_rad):
            err = _wrap(t.steer_rad - cur)
            move = float(np.clip(err, -self.rate * self.dt, self.rate * self.dt))
            out.append(ModuleCommand(float(cur + move), t.wheel_radps * max(0.0, float(np.cos(err)))))
        return tuple(out)


def _wrap(a: float) -> float:
    return float((a + np.pi) % (2 * np.pi) - np.pi)
