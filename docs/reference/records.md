# Records

The control seam's shared data, defined in `testbed/control/records.py`. All are frozen
dataclasses; array fields are numpy. Units are SI throughout; frames are W (world) and B
(the base body, x-forward z-up) unless noted. `Observation.q_rad`/`qd_radps`/`tau_nm` and
every `modules` tuple follow the order of `arm.joints` and `base.modules` in the robot
YAML (`testbed/spec/robot.py`).

## `Pose`

A rigid pose.

| field | type | unit/frame | meaning |
|---|---|---|---|
| `pos_m` | `np.ndarray (3,)` | m | position |
| `quat_wxyz` | `np.ndarray (4,)` | -- | unit quaternion, wxyz |

## `Twist`

A planar base twist, frame B.

| field | type | unit/frame | meaning |
|---|---|---|---|
| `vx_mps` | `float` | m/s, B | forward velocity |
| `vy_mps` | `float` | m/s, B | lateral velocity |
| `wz_radps` | `float` | rad/s, B | yaw rate |

## `GripperCmd`

Enum: `OPEN`, `CLOSE`, `HOLD` (no change). The gripper command level of the control seam.

## `ModuleState`

One swerve module's measured state.

| field | type | unit/frame | meaning |
|---|---|---|---|
| `steer_rad` | `float` | rad | steer joint angle |
| `steer_radps` | `float` | rad/s | steer joint rate |
| `wheel_radps` | `float` | rad/s | wheel spin rate |

## `GripperState`

The gripper's measured state.

| field | type | unit/frame | meaning |
|---|---|---|---|
| `aperture_m` | `float \| None` | m | finger gap (parallel kind); `None` for the magnet kind |
| `attached` | `bool` | -- | holding an object (magnet kind) |
| `attached_id` | `str \| None` | -- | held object's id, if any |
| `cmd` | `GripperCmd` | -- | last commanded gripper level |

## `OracleObject`

A privileged (non-perception) view of one scene object, for scripted/oracle controllers
and evaluation -- never fed to a learned policy's observation.

| field | type | unit/frame | meaning |
|---|---|---|---|
| `id` | `str` | -- | object id |
| `pose_W` | `Pose` | W | object pose |
| `mass_kg` | `float` | kg | object mass |

## `Goal`

The task's placement goal.

| field | type | unit/frame | meaning |
|---|---|---|---|
| `slot_poses_W` | `tuple[Pose, ...]` | W | candidate slot poses |
| `filled` | `tuple[bool, ...]` | -- | which slots are already filled |
| `target_slot` | `int \| None` | -- | index of the current target slot, if any |
| `slot_ids` | `tuple[str, ...]` | -- | slot ids (default empty) |

## `Observation`

One tick's full observation.

| field | type | unit/frame | meaning |
|---|---|---|---|
| `t_s` | `float` | s | simulation time |
| `tick` | `int` | -- | tick index |
| `base_pose_W` | `Pose` | W | base pose |
| `base_twist_B` | `Twist` | B | base twist |
| `q_rad` | `np.ndarray` | rad | arm joint angles, `arm.joints` order |
| `qd_radps` | `np.ndarray` | rad/s | arm joint rates, `arm.joints` order |
| `tau_nm` | `np.ndarray` | N*m | arm joint torques, `arm.joints` order |
| `modules` | `tuple[ModuleState, ...]` | -- | per-module state, `base.modules` order |
| `gripper` | `GripperState` | -- | gripper state |
| `ft_flange_B` | `np.ndarray (6,)` | N then N*m, B | force xyz then torque xyz at the flange site |
| `ee_pose_B` | `Pose` | B | end-effector pose |
| `ee_pose_W` | `Pose` | W | end-effector pose |
| `oracle` | `tuple[OracleObject, ...] \| None` | -- | privileged object list, or `None` if unwired |
| `tags` | `tuple[tuple[str, float], ...] \| None` | -- | `(id, fill mass)` within RFID read range, or `None` |
| `goal` | `Goal \| None` | -- | placement goal, or `None` if unwired |
| `instruction` | `str \| None` | -- | natural-language instruction, if any |
| `envelope_active` | `bool` | -- | whether the safety envelope changed the command on the last control tick (any kind: twist cap, reach/floor clamp, joint limit, force freeze) |
| `phase` | `str \| None` | -- | scripted/skill phase name, if any |
| `images` | `dict \| None` | -- | name -> array; `None` on non-render ticks (rendering is not implemented yet) |
| `tool_offset_F` | `Pose \| None` | F (flange frame) | the gripper interface (tool) site pose relative to the flange site, measured once off the settled compiled model at `Runtime.reset()`; `flange pose (x) tool_offset_F` = tool pose. `None` until a `Runtime` supplies it |
| `tool_kind` | `str \| None` | -- | `gripper.kind` of the robot spec, `"magnet"` or `"parallel"`; lets a skill pick its grasp geometry |
| `wheel_contact` | `np.ndarray (n_modules,) bool \| None` | -- | per module (`base.modules` order): True where that wheel's body has at least one live contact this tick; `None` if not computed |

## `SkillAction`

A named, parameterized skill invocation -- the highest-level action kind.

| field | type | unit/frame | meaning |
|---|---|---|---|
| `name` | `str` | -- | skill name |
| `params` | `dict` | -- | skill parameters (default empty) |
| `stop` | `bool` | -- | request stop |
| `help` | `bool` | -- | request help |

## `TaskAction`

Task-space level: base twist plus an end-effector target pose.

| field | type | unit/frame | meaning |
|---|---|---|---|
| `base_twist_B` | `Twist` | B | commanded base twist (default zero) |
| `ee_target` | `Pose \| None` | B or W (see `ee_frame`) | end-effector target; `None` holds the current commanded target |
| `ee_frame` | `str` | -- | `"B"` or `"W"`, which frame `ee_target` is expressed in |
| `gripper` | `GripperCmd` | -- | commanded gripper level (default `HOLD`) |
| `stop` | `bool` | -- | request stop |
| `help` | `bool` | -- | request help |

## `JointAction`

Joint-space level: a direct arm joint target and/or per-module targets.

| field | type | unit/frame | meaning |
|---|---|---|---|
| `q_target_rad` | `np.ndarray \| None` | rad | target arm joint angles, `arm.joints` order |
| `module_targets` | `tuple[tuple[float, float], ...] \| None` | rad, rad/s | per-module `(steer_rad, wheel_radps)`, `base.modules` order |
| `gripper` | `GripperCmd` | -- | commanded gripper level (default `HOLD`) |
| `stop` | `bool` | -- | request stop |
| `help` | `bool` | -- | request help |

## `ActionChunk`

A sequence of actions to run in order, e.g. from a policy that predicts several ticks at
once.

| field | type | unit/frame | meaning |
|---|---|---|---|
| `actions` | `tuple` | -- | of `SkillAction \| TaskAction \| JointAction` |

## `Action`

`Union[SkillAction, TaskAction, JointAction, ActionChunk]` -- the control seam's tagged
action union; a controller dispatches on `type(action)`.

## `HudSnapshot`

Everything the operator console needs to render one frame; scene-blind (holds no
reference to the compiled model), so the renderer never needs a `mujoco` import.

| field | type | unit/frame | meaning |
|---|---|---|---|
| `mode` | `str` | -- | `"task"` or `"joint"` |
| `controller` | `str` | -- | controller name |
| `phase` | `str \| None` | -- | scripted/skill phase name, if any |
| `twist_cmd_B` | `Twist` | B | commanded base twist |
| `twist_meas_B` | `Twist` | B | measured base twist |
| `modules` | `tuple[ModuleState, ...]` | -- | per-module state, `base.modules` order |
| `q_rad` | `np.ndarray` | rad | arm joint angles |
| `ee_pos_B_m` | `np.ndarray` | m, B | end-effector position |
| `ee_pos_W_m` | `np.ndarray` | m, W | end-effector position |
| `ee_target_B_m` | `np.ndarray \| None` | m, B | the controller's standing (intended) end-effector target position, if any -- what the envelope is handed; on a tick where it intervenes, IK is fed a different (clamped/frozen) target that is never stored back |
| `gripper` | `GripperState` | -- | gripper state |
| `force_n` | `float` | N | current flange force magnitude (contact force PLUS any carried payload's weight and inertial load -- the F/T sensor cannot tell them apart) |
| `force_budget_n` | `float` | N | the envelope's force limit (`RobotSpec.force_budget_n`) |
| `interventions` | `int` | -- | count of envelope interventions this run |
| `rtf` | `float` | -- | real-time factor |
| `stopped` | `bool` | -- | whether the run is stopped |
| `selected_joint` | `int` | -- | index of the joint selected in the console |
