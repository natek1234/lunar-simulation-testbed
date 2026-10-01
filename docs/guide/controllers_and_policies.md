# Controllers and policies

A controller implements three methods (`testbed/control/controller.py`):

```python
class Controller(Protocol):
    name: str
    def reset(self, robot, seed): ...
    def act(self, obs: Observation) -> Action: ...
    def done(self, obs: Observation) -> bool: ...
```

`Observation` and the `Action` union are in `testbed/control/records.py` and documented
field-by-field in `docs/reference/records.md`.

## Action levels
| level | record | resolved by the runtime as |
|---|---|---|
| 1 skill | `SkillAction(name, params)` | the registered executor's `TaskAction`s until it reports done |
| 2 task | `TaskAction(base_twist_B, ee_target, ee_frame, gripper)` | IK for the arm, swerve allocation for the base |
| 3 joint | `JointAction(q_target_rad, module_targets, gripper)` | actuator commands directly |

Every action has `stop` (brake, hold, latch until `Runtime.rearm()`) and `help` (count one
intervention per help EVENT -- a rising edge of the flag, not every tick it stays raised -- and hand
control to the RC device if one is attached; once handed over the controller is no longer
consulted, and how control ever returns to it is not designed yet).

**Policy controllers do not exist yet.** `app.py run --controller policy` is an advertised choice
but `PolicyAdapter` is not implemented yet: the command prints one line saying so and exits with code 2.

`Controller.done()` matters when `app.py run --headless` is given no `--duration-s`: the
runtime then drives ticks until `done()` returns true (`testbed/sim/runtime.py::Runtime.run`)
instead of a fixed physical-time bound.

## Skills
`testbed/control/skills.py` holds a registry. Shipped: `hold`, `drive`, `jog`, plus the nine
placement skills below (`testbed/control/placement_skills.py`, registered automatically
on import of `testbed/control/skills.py`). Register your own:

```python
from testbed.control.skills import register
@register("wave")
class Wave:
    def start(self, params, obs): ...
    def act(self, obs): return TaskAction(...)
    def done(self, obs): return ...
```

## Scripted sequences
`ScriptedSequencer([("drive", {"vx_mps": 0.3, "duration_s": 3}), ...])` runs skills in order.

## Placement skills
`testbed/control/placement_skills.py`: the base drives (`approach`/`park`/`transport`), the
arm/gripper does the rest (`acquire`/`place`/`seat`/`release`/`retract`/`verify`). Grasp
geometry follows `Observation.tool_kind` (`grasp_tool_pose_W`): a magnet picks top-down on
the unit's patch, a parallel jaw grasps the unit's END (fingers closing vertically). Tool
poses convert to flange targets through `Observation.tool_offset_F` (`flange_from_tool`);
`target_pose_W` resolves an RCU id against the oracle or a slot id against the goal.

| skill | params | what it does | done when |
|---|---|---|---|
| `approach` | `target`, `standoff_m` | Drive to a stance `standoff_m` behind `target`, facing it (P-control on stance error, rate-limited to `ACCEL_MAX_MPS2`/`YAW_ACCEL_MAX_RADPS2`). | Base within `STANCE_TOL_M`/`STANCE_TOL_RAD` of the stance pose. |
| `park` | — | Ramp the base twist to zero and hold. | Base speed under `PARK_TOL_MPS` for `PARK_HOLD_S` continuously. |
| `acquire` | `target` | Pre-grasp standoff → down onto the unit (open) → energise/close and dwell (`GRIP_SETTLE_S`) → lift `LIFT_M` clear. The two "at the unit" waypoints use the tighter `GRASP_TOL_M`, not the general `EE_TOL_M` (see `placement_skills.GRASP_TOL_M`'s own comment on why). | Lift waypoint reached and settled (continuous residency in tolerance for the waypoint's dwell). |
| `transport` | `target`, `standoff_m` | Same as `approach`, to a new target (typically a slot). | Same as `approach`. |
| `place` | `target`, `hover_m` | Hold the carried unit `hover_m` above the target slot's seated pose. | Hover waypoint reached and settled. |
| `seat` | `target` | Lower the carried unit at `SEAT_RATE_MPS` until it stops descending (checked over a whole `SEAT_STILL_S` window of total movement vs `SEAT_STILL_M`, not tick to tick — see the class docstring); nothing else limits the press-in below the envelope's force budget (210 N for the magnet robot, so the seating contact is NOT held to the contact-force budget (docs/reference/requirements.md, 58.6 N) -- DECISIONS.md D15). Reports done immediately if nothing is attached. | Unit's own z stops changing (windowed) or nothing attached. |
| `release` | — | Open the gripper and dwell `GRIP_SETTLE_S`. | Dwell elapsed (gripper cmd `None` tool always "reached"). |
| `retract` | `height_m` | Back off `PREGRASP_BACK_M` along the flange's current -z, then lift `height_m` further, computed from wherever the flange actually is. | Both waypoints reached and settled. |
| `verify` | `target` | No-op dwell (0.5 s); `Runtime._run_skill` marks the target slot filled (`Worksite.mark_filled`) when this skill reports done. | Dwell elapsed. |

Every arm waypoint's tool-position target is fed to IK through a rate-limited reference
(`RAMP_RATE_MPS`), not handed over verbatim — see `RAMP_RATE_MPS`'s own comment for the
measured 751 N transient a step-input target produced on a 22 kg suspended payload.

## The envelope (runs outside every controller)
`testbed/control/envelope.py` clamps what a controller asks for: end-effector targets to
95 % of the measured reach and above the floor, base twist to the robot's caps, joint
targets to their limits, and — when the flange force exceeds `force_budget_n` — freezes
the target and backs it off along the sensed force until it drops below 70 % of budget.
Each intervention is counted by kind (`reach`, `floor`, `twist`, `joint`, `force`); the HUD
shows the total and the episode records `envelope_active` per tick.

`force_budget_n` is per-robot (`RobotSpec`), not universal: it has to sit above whatever
force a legitimate, correctly-executed grasp of that robot's own payload produces, or the
freeze-and-back-off locks onto a normal grip and fights it instead of a real hazard (see
`DECISIONS.md` D12 for the measured incident on the parallel-hand variant).
`measure_reach_m` also measures empirically (uniform sampling over the full joint-limit
box against the compiled model's own arm-base position) rather than trusting a vendor's
quoted "nominal reach" figure, which the full joint range can exceed (D12).
