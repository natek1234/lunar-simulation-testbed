"""The seam's records. Frozen dataclasses; arrays are numpy. Units SI; frames W (world)
and B (base body, x forward, z up). Variable-length fields follow the RobotSpec layout:
Observation.q_rad/tau_nm/qd_radps and modules follow arm.joints and base.modules order
from the robot YAML (testbed.spec.robot).

Holds every seam record: GripperCmd (OPEN/CLOSE/HOLD), enough for the gripper strategies
(testbed/sim/gripper_magnet.py, gripper_parallel.py) to name a commanded gripper state
without importing mujoco; Pose, Twist, Observation, the Action tagged-union, HudSnapshot."""
from dataclasses import dataclass, field
from enum import Enum
from typing import Union

import numpy as np


@dataclass(frozen=True)
class Pose:
    """A rigid pose. pos_m (3,) in metres; quat_wxyz (4,) a unit quaternion, wxyz."""
    pos_m: np.ndarray
    quat_wxyz: np.ndarray


@dataclass(frozen=True)
class Twist:
    """A planar base twist in frame B: forward/lateral velocity plus yaw rate."""
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    wz_radps: float = 0.0


class GripperCmd(Enum):
    """The gripper command level of the control seam. Both gripper strategies switch on
    `cmd is GripperCmd.X` rather than enumerating the whole set, so HOLD (no change) is
    a safe, backward-compatible addition -- it never touches either strategy module."""
    OPEN = "open"
    CLOSE = "close"
    HOLD = "hold"


@dataclass(frozen=True)
class ModuleState:
    """One swerve module's measured state: steer angle/rate (rad, rad/s) and wheel
    spin rate (rad/s)."""
    steer_rad: float
    steer_radps: float
    wheel_radps: float


@dataclass(frozen=True)
class GripperState:
    """The gripper's measured state. aperture_m is the parallel kind's finger gap (None
    for the magnet kind); attached/attached_id name a held object (magnet kind only,
    None otherwise); cmd is the last commanded GripperCmd."""
    aperture_m: float | None
    attached: bool
    attached_id: str | None
    cmd: GripperCmd


@dataclass(frozen=True)
class OracleObject:
    """A privileged (non-perception) view of one scene object, for scripted/oracle
    controllers and evaluation -- never fed to a learned policy's observation."""
    id: str
    pose_W: Pose
    mass_kg: float


@dataclass(frozen=True)
class Goal:
    """The task's placement goal: candidate slot poses (world frame), which are already
    filled, and which one (if any) is the current target."""
    slot_poses_W: tuple[Pose, ...]
    filled: tuple[bool, ...]
    target_slot: int | None
    slot_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class Observation:
    """One tick's full observation. q_rad/qd_radps/tau_nm and modules follow
    arm.joints / base.modules order from the robot YAML (testbed.spec.robot). oracle/
    tags/goal are None when the corresponding subsystem isn't wired for a given
    scenario; images is None on ticks that don't render (rendering is not implemented yet)."""
    t_s: float
    tick: int
    base_pose_W: Pose
    base_twist_B: Twist
    q_rad: np.ndarray
    qd_radps: np.ndarray
    tau_nm: np.ndarray
    modules: tuple[ModuleState, ...]
    gripper: GripperState
    ft_flange_B: np.ndarray       # (6,) force xyz then torque xyz, at the flange site
    ee_pose_B: Pose
    ee_pose_W: Pose
    oracle: tuple[OracleObject, ...] | None
    tags: tuple[tuple[str, float], ...] | None     # (id, fill mass) within read range
    goal: Goal | None
    instruction: str | None
    envelope_active: bool
    phase: str | None
    images: dict | None = None    # name -> array; None on non-render ticks (rendering is not implemented yet)
    tool_offset_F: Pose | None = None   # interface (tool) site pose relative to the flange site, frame F = flange
    tool_kind: str | None = None        # gripper.kind ("magnet" | "parallel"); lets a skill pick its grasp geometry
    wheel_contact: np.ndarray | None = None   # (n_modules,) bool, True where that wheel body has >=1 contact this tick


@dataclass(frozen=True)
class SkillAction:
    """A named, parameterized skill invocation (the highest-level action kind)."""
    name: str
    params: dict = field(default_factory=dict)
    stop: bool = False
    help: bool = False


@dataclass(frozen=True)
class TaskAction:
    """Base twist plus an end-effector target pose, task-space level. ee_target=None
    holds the current commanded target; ee_frame selects "B" or "W" for that target."""
    base_twist_B: Twist = Twist()
    ee_target: Pose | None = None      # None = hold current commanded target
    ee_frame: str = "B"                # "B" or "W"
    gripper: GripperCmd = GripperCmd.HOLD
    stop: bool = False
    help: bool = False


@dataclass(frozen=True)
class JointAction:
    """Joint-space level: a direct arm joint target and/or per-module (steer, wheel)
    targets."""
    q_target_rad: np.ndarray | None = None
    module_targets: tuple[tuple[float, float], ...] | None = None   # (steer_rad, wheel_radps) per module
    gripper: GripperCmd = GripperCmd.HOLD
    stop: bool = False
    help: bool = False


@dataclass(frozen=True)
class ActionChunk:
    """A sequence of actions (SkillAction | TaskAction | JointAction) to run in order,
    e.g. from a policy that predicts several ticks at once."""
    actions: tuple


Action = Union[SkillAction, TaskAction, JointAction, ActionChunk]


@dataclass(frozen=True)
class HudSnapshot:
    """Everything the operator console needs to render one frame; scene-blind (holds no
    reference to the compiled model), so the renderer never needs a mujoco import."""
    mode: str                          # "task" | "joint"
    controller: str
    phase: str | None
    twist_cmd_B: Twist
    twist_meas_B: Twist
    modules: tuple[ModuleState, ...]
    q_rad: np.ndarray
    ee_pos_B_m: np.ndarray
    ee_pos_W_m: np.ndarray
    ee_target_B_m: np.ndarray | None
    gripper: GripperState
    force_n: float
    force_budget_n: float
    interventions: int
    rtf: float
    stopped: bool
    selected_joint: int
