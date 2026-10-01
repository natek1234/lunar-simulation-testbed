# Design

Architecture and rationale for someone extending this testbed. If you are looking for
*what value a requirement takes and why*, see `docs/reference/requirements.md` instead —
this document is about how the tool is built, not what it is scored against.

## 1. Purpose

A physics-based (MuJoCo) simulation platform for the autonomous placement of Regolith
Containment Units (RCUs, regolith-filled bags) by a mobile manipulator, built so that

1. the **robot** is swappable by writing a descriptor file (a new base or gripper *kind*
   is the only thing that needs code);
2. the **controller** is swappable across three action levels — a human at a keyboard, a
   scripted skill sequencer, or (not implemented yet) a learned policy — behind one seam;
3. the **surface and worksite** are configuration;
4. metrics for evaluating autonomous placement are computable from recorded episodes.

## 2. Architecture

### 2.1 Packages and boundaries (mechanically enforced)

```
testbed/
  spec/      pure data: RobotSpec, BaseSpec, ArmSpec, GripperSpec, SensorSpec,
             SurfaceSpec, ScenarioSpec; YAML loaders; validation.
             stdlib + numpy + pyyaml only.
  control/   MuJoCo-free control: records (Observation, Action, HudSnapshot), the
             Kinematics protocol, IK, the swerve allocator + module firmware, the
             safety envelope, controllers (KeyboardRc, ScriptedSequencer), skills
             and the skill registry.
  sim/       the ONLY mujoco importer: composer, base/gripper strategies, handle
             resolution, surface generator, RCU/worksite authoring, sensors,
             MuJoCo-backed Kinematics, runtime loop, viewer, keyboard device.
  bench/     scenario running, metrics, episode log, benchmark reports, launcher
             glue (`app.py run`/`bench`). Uses `sim` through its public API only.
  viz/       HUD renderers from HudSnapshot (text overlay today).
robots/      fr3_mpo700.yaml, fr3_mpo700_hand.yaml
surfaces/    flat.yaml
scenarios/   single_placement.yaml
assets/      vendored models/textures + LICENSES + PROVENANCE.md
app.py       composition root / launcher
tests/  docs/{design.md,guide,reference,benchmarks}  DECISIONS.md  LICENSE  .github/workflows/
```

Rules, enforced by an AST import scan in `tests/test_architecture.py`:

- `spec` imports no other `testbed` package.
- `control` and `viz` never import `mujoco`, `glfw`, or `testbed/sim`.
- `sim` is the only importer of `mujoco`; it may import `spec` and `control`.
- `bench` never touches `MjModel`/`MjData` directly; it imports `spec`, `control`, and
  `sim`'s public runtime/sensor API.
- Nothing under `testbed/` imports `app.py`.

**Why a boundary at all.** `control` staying MuJoCo-free means IK, the swerve allocator
and the safety envelope can be unit-tested against a synthetic `Kinematics` in
milliseconds, with no physics engine, no compiled model, and no GPU — and it means a
future non-MuJoCo backend could reuse every one of those modules unchanged. `sim` being
the *only* importer of `mujoco` means every MuJoCo-specific fact (how a contact is
signed, which array is stale after a step, how a spec composes) has exactly one place it
can leak from. `bench` sitting above `sim` (never the reverse) keeps the composition/
metrics/reporting layer from becoming something `sim` itself depends on, so `sim` stays
usable standalone. The test that enforces this is not a style preference: three of the
measured MuJoCo pitfalls below (§9) were caught specifically because a boundary violation
would have been a red build rather than a code-review comment.

### 2.2 The robot descriptor

A robot is data — one YAML file loaded into frozen dataclasses (`testbed/spec/robot.py`).

**BaseSpec** — `kind` (only `swerve` is implemented; `skid`/`floating` are accepted by
validation but have no registered strategy yet, so a robot naming one fails at
composition, not at load); `body` (chassis body name); `chassis_mjcf` (a vendored MJCF
path); `modules[]` (name, `xy_B`, steer joint/actuator, drive joint/actuator, wheel body,
wheel radius, lateral offset); `suspension` (optional, currently unused by any shipped
robot); `limits` (max twist, steer rate, steer angle); `spawn_z_B`. Module order is the
list order in the file — nothing in code assumes four modules or their names.

**ArmSpec** — a vendored MJCF (path, optional uniform scale, root body), `joints[]`,
`actuators[]`, `home_q`, `flange_site`, `mount` (position and quaternion on the base
body). Joint limits, link inertias and reach are **read back from the compiled model**,
never retyped.

**GripperSpec** — `kind ∈ {magnet, parallel}`; `mount_quat` correction relative to the
flange; `interface_site` (the tool point). `magnet`: an authored electromagnet cylinder,
no actuator; `capture_distance_m`, `capture_angle_rad`, `hold_force_n`; attach = a weld
equality activated when the magnet face is within capture distance/angle of an RCU's
top-face patch while the command is CLOSE (energised); released on OPEN or when the
flange force exceeds `hold_force_n` (breakaway). `parallel`: a vendored MJCF (the Franka
hand), `actuator`, `cmd_open`, `cmd_close`, `finger_joints`, optional `gain_override`;
attach = both fingers in contact with the same unit while closed. The end effector is
swapped by choosing the robot file (`fr3_mpo700` = magnet, `fr3_mpo700_hand` = parallel);
nothing else changes.

**SensorSpec** — `ft_site` (flange force/torque), `cameras[]` (declared but not rendered
by anything yet — see §5), `tag_read_range_m`.

Robots that ship today: `fr3_mpo700` (rover-scaled swerve base — 1.2 m wheelbase, 0.9 m
track, 0.135 m wheels, ×1.5 FR3 + electromagnet), `fr3_mpo700_hand` (the same with the
Franka hand — the parallel kind).

### 2.3 Composition (`testbed/sim/compose.py`)

Builds one live `mujoco.MjSpec` from `RobotSpec + SurfaceSpec (+ WorksiteSpec)` and
compiles it **in memory** (never `to_xml()` and reload — §9). Order: option/visual
singletons written explicitly on the live spec after every attach (timestep, gravity,
headlight); surface; chassis; base strategy adds modules; arm attached at the mount frame
with a per-attach `prefix` decision; gripper attached at the flange with its correction
quaternion via a new `<frame>`; sensors; worksite bodies.

Base and gripper kinds are registered strategies (`testbed/sim/base_swerve.py`,
`testbed/sim/gripper_parallel.py`, `testbed/sim/gripper_magnet.py`) implementing
`compose(spec, mjspec)`, `resolve(model) -> handles`, `apply(handles, command)`,
`read(handles) -> state`. Adding a kind adds a module; the composer does not change.

`testbed/sim/handles.py` resolves every name in the spec against the compiled model,
failing loudly on any miss, and cross-checks actuator→joint transmission targets so a
swapped wiring is a red build. All downstream code reads through handles.

### 2.4 Records (`testbed/control/records.py`)

**Observation** (per tick, frozen): `t_s`, `tick`; `base_pose_W` (pos, quat wxyz),
`base_twist_B`; `q`, `qd`, `tau` (arm, length = spec); `modules` (steer angle, steer rate,
wheel rate per module); `gripper` (aperture or magnet state, `attached`, `attached_id`);
`ft_flange_B` (6); `ee_pose_B`, `ee_pose_W`; `oracle` (RCU ids + poses, slot poses, or
`None` when unwired); `tags` (ids + fill mass within read range, or `None`); `images`
(not populated by anything yet — see §5); `goal` (blueprint: ordered slot poses, filled
flags, current target slot); `instruction: str | None`; `envelope_active`; `phase` (the
running skill's phase name, if any); `tool_offset_F`, `tool_kind` (which grasp geometry a
skill should use); `wheel_contact`. Layout of the variable-length fields is defined by the
RobotSpec so one record type serves every robot.

**Action** — tagged union, each variant carrying `stop: bool` and `help: bool`:

- `SkillAction(name, params)` — level 1.
- `TaskAction(base_twist_B, ee_target: Pose, ee_frame ∈ {B, W}, gripper_cmd)` — level 2.
- `JointAction(q_target | qd_target, module_targets, gripper_cmd)` — level 3.
- `ActionChunk(actions: list[Action])` — declared for a controller that predicts several
  ticks at once; the runtime currently consumes only the first entry per tick.

`stop` → controlled brake, arm hold, runtime state `STOPPED` until re-armed. `help` →
logged intervention (one per rising edge of the flag, `docs/reference/requirements.md`'s
escalation requirement); the runtime can hand the seam to an attached RC device, but no
shipped path attaches one, so today `help` only counts.

**Why three levels, one seam.** A human at a keyboard naturally commands at task level
(drive the base, jog the end effector); a scripted sequence naturally commands a mix of
skill and task level; a policy that has learned joint torques directly needs joint level.
Rather than three separate runtimes, one seam accepts whichever level a controller
chooses to speak, and the runtime resolves top-down through whichever levels are below
it — so a human, a script, and (eventually) a learned policy are the same kind of thing to
everything downstream of the seam, and swapping one controller for another never touches
the runtime, the envelope, or the IK/allocator.

### 2.5 Controllers and the runtime

`Controller` protocol: `reset(robot_spec, seed)`, `act(obs) -> Action`, `done(obs) ->
bool` (optional). Shipped: `KeyboardRc` (task-space, from a polled key-state device — a
held-key deadman) and `ScriptedSequencer` (a list of skills executed as a state machine —
the single-placement baseline). A policy controller and a hierarchical planner-over-executors
controller are not implemented yet (`app.py run --controller policy` prints one line and
exits 2 rather than pretending to work).

`testbed/sim/runtime.py` resolves each control tick top-down:

```
Skill  ─▶ executor (state machine) ─▶ TaskAction
Task   ─▶ Envelope.clamp_task ─▶ IK (arm) + allocator + firmware (base) ─▶ JointAction
Joint  ─▶ Envelope.clamp_joint ─▶ actuator ctrl
```

Physics `dt` = 2 ms; controller at `control_hz` = 50 with zero-order hold. Every physics
tick appends observation, action, resolved joint command and envelope report to the
episode when one is attached. The loop runs in sim time; a slow policy blocks the loop, so
evaluation stays deterministic. The viewer paces the same loop to wall time.

### 2.6 Kinematics and IK

`testbed/control/kinematics.py` protocol: `fk(q) -> Pose`, `jacobian(q) -> (6, n)`,
`limits()`. `testbed/sim/kinematics_mujoco.py` implements it on a scratch `MjData` of the
compiled model (`mj_kinematics` + `mj_jac` at the flange site), so one IK serves any arm
and no per-robot kinematics table exists. `testbed/control/ik.py`: damped least squares
with a null-space posture term; rate saturation by **one scalar** for the whole
joint-velocity vector (never per-joint clipping — it rotates the realized direction);
joint stops handled as an **active set** (freeze joints that would be driven past a stop,
re-solve the rest); EE targets clamped to reach before solving.

### 2.7 Safety envelope (`testbed/control/envelope.py`)

The runtime shield outside every controller (envelope enforcement,
`docs/reference/requirements.md`): EE target reach + workspace box clamp; joint position
limits from the compiled model; base twist limits; a flange-load limit — when
`|F_flange|` exceeds the robot's own budget the EE target is frozen and backed off along
the force direction; every intervention is counted by kind. Self-collision checking on the
commanded configuration is not implemented.

The flange-load limit is a robot-specific value, not the published contact-force budget
directly — see `docs/reference/requirements.md`'s contact-force budget entry for why the
flange sensor cannot distinguish contact force from a carried payload's own weight and
inertial load, and what each shipped robot's budget is actually sized against.

### 2.8 Surface and worksite

`SurfaceSpec` (`surfaces/*.yaml`): gravity, friction triple, texture, extent. Only
`kind: flat` is implemented today; heightfield generation (slope, macro relief, fine
roughness, a spawn pad) is designed for but not built (`testbed/spec/surface.py` raises on
any other `kind`).

`WorksiteSpec` (inside the scenario, `testbed/sim/worksite.py`): staged RCU spawn poses
(rested on the surface), blueprint slot poses, robot spawn pose. The **RCU** is an
authored superellipsoid body, 0.43 × 0.25 × 0.10 m, 22 kg nominal
(`docs/reference/requirements.md`'s RCU geometry and mass), friction, a tag record (id,
fill mass), an interface patch site on the top face.

### 2.9 Sensing (`testbed/sim/sensors.py`)

Proprio from `qpos`/`qvel`/`actuator_force`; base pose from the freejoint `qpos` (not the
one-step-stale `xpos` — see §9); wrist F/T from MuJoCo `force`/`torque` sensors at the
flange site; oracle and tags read directly from model state (no sensing-noise shims
implemented — every value is ground truth today). RGB-D cameras are declared in
`SensorSpec` but nothing renders or populates `Observation.images` yet.

### 2.10 Scenario and metrics

`scenarios/single_placement.yaml`: robot on the pad; one RCU staged; one blueprint slot
on an existing two-unit foundation. Ends on `verify` or a timeout (a 1.5× hang guard over
the cycle-time budget, `docs/reference/requirements.md`). The scripted sequence:
`approach → park → acquire (descend, grasp/attach, lift) → approach (via stance) →
transport → park → place (coarse approach above slot) → seat (force-regulated descent) →
release → retract → verify`.

`testbed/bench/metrics.py`, from the **recorded episode only** (never live `Runtime`
state): placement position/orientation error vs the slot; cycle and per-phase time; peak
and seating flange force; gripper attach/release confirmation; displacement of
already-placed units; envelope interventions by kind; stop/help counts; wheel contact
fraction. Reports append to `docs/benchmarks/*.md` with provenance (commit, seed,
surface, date); record-then-gate (`docs/guide/benchmarks.md`, CONTRIBUTING.md).

### 2.11 Episode log

`testbed/bench/episode.py`: `episode.npz` + `meta.json` (spec hashes not yet included;
seed, robot/surface/scenario ids, git commit, schema version). **One row per PHYSICS
tick** (2 ms, 500 Hz), not per control tick — `Runtime.tick()` appends a freshly computed
`Observation` after every `mujoco.mj_step()`, independent of the coarser `decim`-gated
refresh the controller sees. This matters concretely: a peak-force measurement taken from
the 50 Hz controller-facing observation under-resolves fast transients (124.2 N measured
vs. a true 411.4 N at the same event, `docs/benchmarks/single_placement.md`), while the
same quantity read from the episode is correct at physics-tick resolution. Replay/verify
against the logged joint commands is not implemented yet.

### 2.12 Operator interface and launcher

`testbed/sim/viewer.py` — MuJoCo GLFW viewer, render decoupled at ~60 fps;
`testbed/sim/keyboard.py` — polled physical key state. Keys and HUD fields:
`docs/guide/teleop.md`. `testbed/viz/hud_text.py` draws a `HudSnapshot` as a text overlay;
an ImGui console is not implemented.

`app.py`: `run --robot R --surface S [--scenario B] --controller keyboard|scripted|policy
[--seed N] [--headless] [--duration-s S]`, `bench --scenario B --seeds a b c`, `physics
--robot R [--surface S]`, `test [pytest args]`. Re-execs into `.venv`; pathlib only;
Windows-first.

## 3. Testing and house rules

- Default tier: headless, CPU-only, `-m "not viewer"`; a `viewer` marker for GLFW tests.
  CI on `ubuntu-latest` + `windows-latest`.
- Enforced from the first commit: import boundaries; provenance tags on every numeric
  module constant, closed vocabulary (`CONTRIBUTING.md`); every path cited in
  code/docs exists; vendored asset sha256 ledger; determinism (same seed ⇒ bit-identical
  episode, in-process and across processes); settle at nominal ride height; F/T reads
  m·g for a held RCU; the scripted placement episode succeeds on `flat` (gated).
- `DECISIONS.md` numbered from D1; `docs/benchmarks/` append-only; numbers in docs carry
  their measurement provenance.

## 4. What is implemented, and what is not

Implemented and gated: the swerve base kind; the magnet and parallel gripper kinds; the
three action levels with `KeyboardRc`/`ScriptedSequencer`; damped-least-squares IK with an
active-set joint-stop solve; the safety envelope's reach/floor/twist/joint/flange-load
clamps; the flat surface; RCU/worksite authoring and both gripper attachment mechanisms;
proprioceptive, base-pose, F/T, oracle and tag sensing; the full scripted placement
scenario with its metrics and benchmark report; the GLFW viewer with a text HUD; the
`app.py run`/`bench`/`physics`/`test` launcher; determinism and cross-process
bit-identical replay of the same construction.

Not implemented, named rather than silently absent:

- **`skid`/`floating` base kinds** — accepted by `RobotSpec` validation, no registered
  composition strategy.
- **A second arm/base descriptor** to exercise the swap beyond the gripper kind.
- **A learned-policy controller and a hierarchical planner-over-executors controller** —
  `app.py run --controller policy` fails cleanly with an explanatory message.
- **Condition sampling** — `--seed` is accepted and threaded through, but nothing today
  draws from it; every seed produces a bit-identical episode.
- **Sensing shims** (proprio/F/T noise, oracle pose noise, tag misread probability, camera
  dropout, observation latency) — every sensed value is ground truth today.
- **RGB-D rendering** — `SensorSpec.cameras` and `Observation.images` exist as fields;
  nothing populates them.
- **Terrain relief and suspension** — only a flat, rigid-chassis surface exists.
- **Self-collision checking** in the safety envelope.
- **Joint velocity limits** in the safety envelope — `Envelope.clamp_joint` clamps
  position only; the only source of an unbounded `JointAction` would be a policy
  controller, which is not implemented yet either.
- **Replay/verify** of a logged episode's commands against fresh physics.
- **A gym-style environment wrapper.**
- **An ImGui operator console** — the HUD is text-only.

## 5. Measured MuJoCo pitfalls (empirically established — do not rediscover)

- **`<include>`/`attach` merge singletons (`<compiler>`, `<option>`, `<visual>`) last-wins;
  `<visual>` is dropped SILENTLY on attach.** Write every singleton explicitly on the live
  spec after composing and assert it from the compiled model — there is no warning to
  notice for `<visual>`, unlike `<option>`, which does warn (lazily, at compile).
- **`MjSpec.attach(prefix="")` keeps child names verbatim; the default `"/"` renames
  them.** Two vendored packages can collide on material names (`white`, `black`) even when
  their body/joint names don't — decide the prefix per attach, on purpose (empty when a
  name is a real contract, e.g. `fr3_joint1..7`; non-empty otherwise).
- **A vendored model's standalone root orientation is not necessarily its mounted
  orientation.** The Franka hand's own file carries a baked identity-ish quaternion
  correct only for loading it standalone; menagerie's own upstream compositions mount it
  at a different rotation. Compensating for a baked child orientation is a quaternion
  composition problem (`mount_frame_quat = target_quat (x) inverse(child_own_quat)`,
  Hamilton product, wxyz) — verify by reading the COMPILED body's `body_quat` back, never
  trust the derivation on paper alone. Attach the correction via a new `<frame>` on the
  parent body, never by mutating an existing vendored named site in place (that site's
  identity orientation is its own vendored meaning).
- **Never round-trip a composed `MjSpec` through `to_xml()` and reload it.** Measured
  losses going through `to_xml()` and back, none caught by a body/joint/dof census: (1)
  each attached sub-model's own per-asset `meshdir` is dropped in favor of one global
  `<compiler meshdir>`, breaking mesh resolution for whichever vendor didn't win; (2)
  `to_xml()` emits an extra, empty, unnamed `<default>` wrapping an attached model's own
  named default class — valid to write, not valid to re-parse; (3) an explicit `mass="0"
  density="0"` override on a default geom class is silently dropped, letting a link's
  collision mesh double its auto-computed mass. Compile the live `MjSpec` object directly
  and never serialize it — there is no text form to lose information in.
- **A body can carry MULTIPLE `<joint>` elements; they compose as a serial chain in
  LISTED order**, each joint's axis interpreted in the frame the PRECEDING joint on that
  body establishes. A hinge-then-slide stack rotates the slide's effective travel axis by
  the hinge's own rotation; slide-then-hinge keeps the slide axis fixed in the parent
  frame regardless of spin — the physically correct order for a suspension slide added
  onto an existing hinged wheel body with no room for an intermediate body.
- **A joint's spring preload is `springref`, not something `stiffness`/`damping` alone
  imply.** MuJoCo's joint spring force is `stiffness * (springref - qpos)`, pulling
  `qpos` toward `springref`, not toward 0. Setting `stiffness`/`damping` without also
  setting `springref` leaves the spring's rest point at `qpos = 0`, correct only if
  whatever external load balances it (e.g. gravity) is already accounted for.
- **A straight-line drive test with symmetric commanded motion can leave an
  actuator↔joint transmission swap undetected**, if the swapped pair happens to be
  symmetric about the vehicle centerline and the test's own commanded motion isn't
  sensitive to which physical joint each actuator drives. Gate actuator↔joint wiring
  directly against the compiled model's own transmission target
  (`model.actuator_trnid[aid][0]` resolved back to a joint name), never indirectly
  through a behavioral test whose particular motion might not be sensitive to that swap.
- **`spec.attach()` carries over EVERYTHING in the child model** — actuators, tendons,
  equality constraints, keyframes, all of it, not just the bodies/joints being reached
  for. After attaching a foreign vendored model, enumerate what it actually added
  (`model.nu`, `model.ntendon`, `model.neq`, ...) rather than reasoning from what was
  intentionally written.
- **`data.contact[i].geom1`/`geom2` are assigned by ASCENDING MuJoCo geom TYPE ENUM**, not
  by which body is static/dynamic or which geom was declared first — the identical body
  can appear as `geom1` against one geom type and `geom2` against another. A function
  reading "the force on body X" from `mj_contactForce`/`data.contact[i].frame` must branch
  on which side body X is on, never assume a fixed side; `mj_contactForce`'s normal
  magnitude is always non-negative, so `abs()`-ing it guards nothing.
- **`data.xpos`/`data.xquat` (and other FK-derived arrays) are ONE STEP STALE immediately
  after `mj_step()`** — they still reflect the kinematics computed before that step's
  integration. A MuJoCo **freejoint**'s `qpos` IS already the body's current world pose
  directly (no FK needed) and is not stale this way — prefer it for "the world pose right
  after stepping"; a caller that must read `xpos`/`xquat` post-step should call
  `mj_forward` first if the state must be current.
- **A rolling contact against a triangulated surface can drop to ZERO contacts for a
  single physics tick, and it can be a genuine microscopic loss of contact, not a solver
  bookkeeping artifact.** Investigate geometrically before dismissing either way: check the
  penetration trend on neighboring ticks, test state-determinism with a fresh
  `mj_forward` (rules out solver/warmstart bookkeeping), and corroborate against a
  flat/analytic-geometry control (rules out a general dynamics bug). A hard "zero airborne
  ticks" gate should debounce (no N-consecutive-tick zero-force run) rather than demand an
  exact 0.00 fraction, once the mechanism is confirmed benign.
- **Contact detection uses the SUM of the two geoms' margins**, so a per-pair threshold
  needs half on each geom; **`mj_geomDistance` can return exactly 0.0 spuriously at a
  large `distmax`** — for a yes/no "within margin" check, use `mj_collision` on a scratch
  model with the masks/margins meant, and if distances are truly needed, keep `distmax`
  just above the range of interest and cross-check against `mj_collision`.
- **`mjv_moveCamera` takes no scene argument, and `+reldy` zooms IN** (distance shrinks) —
  port the convention (wheel-up zooms in), never a literal sign copied from a
  differently-signatured reference implementation, and pin the direction with a
  closed-loop assert on `cam.distance` so an upstream sign change fails the suite, not the
  operator.
- **A per-joint clip applied AFTER solving IK rotates the realized direction** — measured
  as motion nearly perpendicular to, or even reversed from, the commanded direction when a
  joint sits pinned on a stop while the solver commands velocity into it (the other,
  still-free joints execute velocities computed on the false assumption that the pinned
  ones were also moving). **The correct handling of a coordinate on its bound is an ACTIVE
  SET** (freeze it, re-solve the rest), not a clip after the fact — the same failure mode
  bit twice in this lineage, once at a rate limit and once at a position stop, and both
  looked like a stall or a perpendicular/reversed motion, invisible to a test that only
  checks limits were honoured.
- **An IK target beyond reach parks the arm at its singular max-reach configuration and
  can then STALL on the way back** — holding the target ON the boundary (any reach clamp,
  even with zero margin) instead avoids over-driving the arm into that pose. Never let a
  target write bypass the reach clamp.
- **`imgui`-family text/draw APIs drift their argument order across versions in ways that
  only fail at call time, behind whatever input branch happens to exercise them** — sweep
  every draw branch with a forcing/synthetic input in the default test tier, not just a
  live-drive smoke test, since an untested branch hides exactly this kind of drift until
  it ships live.

Everything above was carried forward from an earlier MuJoCo build of this same platform's
architecture and its own measured findings, re-verified rather than assumed where this
testbed's own geometry, scale, or MuJoCo version differs (see the courtesy acknowledgement
in `README.md`).
