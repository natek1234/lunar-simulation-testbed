"""Skill executors: small state machines that turn a SkillAction into TaskActions over
time. Registered by name so a learned executor can replace any one of them later."""
from typing import Protocol

import numpy as np

from testbed.control.records import Observation, Pose, TaskAction, Twist

REGISTRY: dict[str, type] = {}


def register(name: str):
    def deco(cls):
        REGISTRY[name] = cls
        return cls
    return deco


class SkillExecutor(Protocol):
    def start(self, params: dict, obs: Observation) -> None: ...
    def act(self, obs: Observation) -> TaskAction: ...
    def done(self, obs: Observation) -> bool: ...


class _Timed:
    def start(self, params, obs):
        self.t0 = obs.t_s
        self.dur = float(params.get("duration_s", 1.0))

    def done(self, obs):
        return obs.t_s - self.t0 >= self.dur


@register("hold")
class Hold(_Timed):
    def act(self, obs):
        return TaskAction()


@register("drive")
class Drive(_Timed):
    def start(self, params, obs):
        super().start(params, obs)
        self.twist = Twist(float(params.get("vx_mps", 0)), float(params.get("vy_mps", 0)), float(params.get("wz_radps", 0)))

    def act(self, obs):
        return TaskAction(base_twist_B=self.twist)


@register("jog")
class Jog(_Timed):
    """Move the flange by dpos_B_m (in B) and hold there for duration_s."""
    def start(self, params, obs):
        super().start(params, obs)
        self.target = Pose(obs.ee_pose_B.pos_m + np.asarray(params["dpos_B_m"], dtype=float), obs.ee_pose_B.quat_wxyz.copy())

    def act(self, obs):
        return TaskAction(ee_target=self.target, ee_frame="B")


# Import guard, placed after REGISTRY/register are defined: placement_skills.py itself
# does `from testbed.control.skills import register`, so importing it any earlier in
# this file (before `register` exists) would be circular.
from testbed.control import placement_skills as _ps  # noqa: E402,F401  (registers the placement skills)
