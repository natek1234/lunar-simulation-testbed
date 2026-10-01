# Surfaces and scenarios

## Surfaces (`surfaces/*.yaml`)
`kind: flat` -- a textured plane. Fields: `gravity_mps2`, `friction` (sliding, torsional,
rolling), `texture`, `extent_m`. Heightfield kinds (slope, macro relief, fine roughness,
spawn pad) are not implemented yet, on the same file.

## Scenarios (`scenarios/*.yaml`)
A scenario is a worksite (RCU geometry, staged units, slots, robot spawn) plus a scripted
baseline and a timeout -- the `ScenarioSpec` dataclass in `testbed/spec/scenario.py`,
loaded by its `load_scenario()`. `scenarios/single_placement.yaml` is the single-placement
baseline:

```yaml
name: single_placement
timeout_s: 285.0
worksite:
  rcu: {extents_m: [0.43, 0.25, 0.10], mass_kg: 22.0, friction: [1.0, 0.005, 0.0001], corner_p: 6.0}
  staged:
    - {id: rcu_0, xyz_W_m: [3.0, 0.0, 0.05], yaw_rad: 0.0}
    - {id: found_0, xyz_W_m: [2.285, 3.0, 0.05], yaw_rad: 0.0}
    - {id: found_1, xyz_W_m: [2.715, 3.0, 0.05], yaw_rad: 0.0}
  pallets: []
  slots:  [{id: slot_0, xyz_W_m: [2.5, 3.0, 0.15], yaw_rad: 0.0}]
  robot_spawn_xy_yaw: [0.0, 0.0, 0.0]
  to_place: [rcu_0]
script_steps: [{skill: approach, params: {target: rcu_0, standoff_m: 1.1}}, ...]   # 1.1, not 0.9: see below
```
`staged` RCUs spawn resting on the surface (`xyz_W_m` is the body CENTRE -- z = half the
RCU height for a ground course, or half height above a foundation course for a stacked
one); `slots` are target centres of a seated RCU and appear as green site markers.
`to_place` names which staged units the script is expected to move (the rest, like
`found_0`/`found_1` here, are pre-placed foundation units the scene needs but the script
never touches). `script_steps` is the scripted baseline `--controller scripted` runs; any
controller sees the same worksite through `Observation.oracle`, `.tags` and `.goal`.

`standoff_m` is a MEASURED value, not a free choice: the original draft's 0.9 m makes the arm's
own forearm (`fr3_link5`/`fr3_link6`) collide with `base_link` while reaching down to the unit,
so the shipped scenario uses **1.1 m** (a sweep found 1.0-1.2 clean; DECISIONS.md D13). The
script also drives a **via stance** (`approach` to `slot_0` at 1.5 m) before `transport`'s final
1.1 m stance: driving straight there sweeps the base's front wheel through the foundation
(DECISIONS.md D16) -- the skills do no obstacle-aware path planning, so a scenario with a
worksite the chassis can hit must author the route.

`rcu_0`'s `x=3.0` is a MEASURED distance, not an authored one: at spawn the x1.5 FR3's own
`home` keyframe already puts the gripper interface site ~1.38 m forward of `base_link`
(the keyframe folds joints 4/6 to +-1.57 rad, which is a stretched-forward posture, not a
tucked one) -- at an earlier draft's `x=2.5` the interface site sat 1.70 m from the RCU's
patch, INSIDE the default `tag_read_range_m` (2.0 m), so the unit would already read as
tagged before the robot ever moved. At `x=3.0`, the distance the `tags` gate actually
computes -- interface site to the RCU's BODY ORIGIN, `Worksite.observe()`'s
`OracleObject.pose_W`, which is the freejoint `qpos`, not the patch site -- measures
**2.0632 m** [measured: `d.site_xpos[interface] - ws.rcu_pose_W(h, "rcu_0").pos_m` after
`rt.reset()`, 2026-09-21]; the interface site to the top-face `<id>_patch` site instead
(a different point, 0.05 m higher) measures 2.0325 m. Both clear the 2.0 m gate; see
DECISIONS.md D10's correction for why this doc originally cited a rounded "2.06 m"
without saying which of the two it meant.

## What a controller sees
- `oracle`: id, pose, mass of every RCU (ground truth; sensing shims are not implemented yet)
- `tags`: (id, fill mass) of RCUs within `sensors.tag_read_range_m` of the gripper
- `goal`: slot poses, filled flags, the current target slot

## Gripper attachment
Both gripper strategies (`testbed/sim/gripper_magnet.py`, `gripper_parallel.py`) decide
attachment against the SAME `Worksite.rcu_bodies` candidate map, and both are exercised in
`tests/test_worksite.py` against a real `RcuSpec`-built body:
- **magnet** (`fr3_mpo700.yaml`, the default): a `mjEQ_WELD` equality per candidate RCU,
  switched active when the magnet face is within `capture_distance_m` and
  `capture_angle_rad` of the unit's `<rcu_id>_patch` site while commanded CLOSE, released
  on OPEN or when `|F_flange|` exceeds `hold_force_n`. The magnet face's outward direction
  is the interface site's own `+z` (the site sits at the far end of the authored cylinder,
  i.e. the face, so `+z` continues the same direction the cylinder extrudes away from the
  flange) -- an earlier draft used `-z` and never captured (measured angle pi rad against
  every real seated RCU); see DECISIONS.md D10.
- **parallel** (`fr3_mpo700_hand.yaml`, alternate): attachment is contact-based (both
  finger bodies touching the same RCU body while commanded CLOSE); holding it under a lunar
  22 kg load needed the vendored hand servo retuned well past an earlier draft's
  originally-guessed 2/4/8 range -- `gain_override: 64.0`, the smallest value on the measured slip-vs-gain
  plateau; see DECISIONS.md D11.
