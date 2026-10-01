"""Keyboard remote control: KeyState (polled physical keys) -> Action. Pure; the GLFW
device lives in sim/keyboard.py. Held keys drive; releasing (or losing focus) brakes."""
from dataclasses import dataclass

import numpy as np

from testbed.control import quat as Q
from testbed.control.records import GripperCmd, JointAction, Observation, Pose, TaskAction, Twist

EE_JOG_MPS = 0.15          # [policy default: end-effector jog speed]
EE_YAW_RADPS = 0.5         # [policy default]
JOINT_JOG_RADPS = 0.4      # [policy default]
KEY_DT_S = 0.02            # [derived: one control tick at 50 Hz; the jog integrates per act()]


@dataclass(frozen=True)
class KeyState:
    held: frozenset
    pressed: frozenset
    focused: bool


class KeyboardRc:
    name = "keyboard"

    def __init__(self, robot):
        self.robot = robot
        self.keys = KeyState(frozenset(), frozenset(), True)
        self.mode = "task"
        self.selected_joint = 0
        self.gripper = GripperCmd.OPEN
        self.target_B: Pose | None = None
        self.wants_rearm = False
        self.wants_close = False

    def reset(self, robot, seed):
        self.__init__(robot)

    def done(self, obs):
        return self.wants_close

    def update(self, keys: KeyState) -> None:
        self.keys = keys

    def act(self, obs: Observation):
        k = self.keys
        held = k.held if k.focused else frozenset()
        p = k.pressed
        self.wants_rearm = "R" in p
        self.wants_close = "ESC" in p
        if "M" in p:
            self.mode = "joint" if self.mode == "task" else "task"
        for d in "1234567":
            if d in p and int(d) <= len(self.robot.arm.joints):
                self.selected_joint = int(d) - 1
        if "SPACE" in p:
            self.gripper = GripperCmd.CLOSE if self.gripper is GripperCmd.OPEN else GripperCmd.OPEN
        stop, help_ = "X" in p, "H" in p
        b = self.robot.base
        tw = Twist(b.max_vx_mps * (("W" in held) - ("S" in held)), b.max_vy_mps * (("A" in held) - ("D" in held)),
                   b.max_wz_radps * (("Q" in held) - ("E" in held)))
        if self.mode == "joint":
            q = obs.q_rad.copy()
            q[self.selected_joint] += JOINT_JOG_RADPS * KEY_DT_S * (("=" in held) - ("-" in held))
            mods = None
            return JointAction(q_target_rad=q, module_targets=mods, gripper=self.gripper, stop=stop, help=help_)
        if self.target_B is None:
            self.target_B = Pose(obs.ee_pose_B.pos_m.copy(), obs.ee_pose_B.quat_wxyz.copy())
        dp = np.array([("I" in held) - ("K" in held), ("J" in held) - ("L" in held), ("U" in held) - ("O" in held)], dtype=float)
        dyaw = (("]" in held) - ("[" in held)) * EE_YAW_RADPS * KEY_DT_S
        pos = self.target_B.pos_m + dp * EE_JOG_MPS * KEY_DT_S
        quat = Q.mul(Q.from_rotvec(np.array([0, 0, dyaw])), self.target_B.quat_wxyz) if dyaw else self.target_B.quat_wxyz
        self.target_B = Pose(pos, quat)
        return TaskAction(base_twist_B=tw, ee_target=self.target_B, ee_frame="B", gripper=self.gripper, stop=stop, help=help_)
