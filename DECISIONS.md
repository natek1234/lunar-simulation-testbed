# Decision ledger

Numbered, append-only engineering rationale: why this tool is shaped the way it is, backed
by the measurement that justified each ruling. Each entry states the decision, the
evidence, and the consequence for the code. Entries are never rewritten in place — a
later finding that changes or supersedes an earlier one gets a dated **Correction** or
**Addendum** paragraph appended to that same entry, so the record of what was believed and
when stays intact. Numbers are assigned in the order a decision was made, not renumbered
later; a superseded entry still carries useful history and is marked SUPERSEDED rather
than deleted. Code comments cite entries by number (`DECISIONS.md D12`, etc.) — those
references are load-bearing and must stay valid.

## D1 — Clean-room rebuild, licensed assets only (2026-09-18)
No code is copied from any unlicensed source. Assets: MPO-700 MJCF via `FunanShi/swerve-nav`
(MIT, commit `0fb97e36dda5d072f4e4cc6cce87ca1f6a98cfe0`), Franka FR3 + hand from
`google-deepmind/mujoco_menagerie` (Apache-2.0, commit
`c1a4eeb85694ae1dffe33ff1797d4e528928a133`), Poly Haven textures (CC0).

## D2 — Robots are data; kinematics come from MuJoCo (2026-09-18)
A robot is a YAML `RobotSpec`; base and gripper kinds are registered strategies.
Forward kinematics and Jacobians are read from a scratch `MjData` of the compiled
model, so one IK serves every arm and no per-robot kinematics table exists.

## D3 — `test_cited_paths_exist` exempts forward-looking docs and named future deliverables (2026-09-18)
`tests/test_conventions.py::test_cited_paths_exist` checks that any backticked
path cited in a *living* doc (README, CONTRIBUTING, `docs/guide/`,
`docs/reference/`, `docs/benchmarks/`) exists today. `docs/plans/*.md` and
`docs/specs/*.md` are forward-looking by design — they name files the code will
create later on purpose — so they are excluded from the parametrization entirely
rather than requiring a path-by-path exemption list that would need editing constantly.
A few living docs named a handful of paths ahead of the code that created them
(`assets/`, `assets/PROVENANCE.md`, `app.py`, `robots/`, `surfaces/`, `scenarios/`):
`EXEMPT_PLANNED_PATHS` in `tests/test_conventions.py` listed exactly these, each
commented with what would create it, so a *typo'd or genuinely broken* path in a
living doc still fails the test. (Now empty — see D17.)

## D4 — Rover scale is a composition parameter, not a second MJCF (2026-09-18)
`footprint_scale` (2.5, xy) and `module_scale` (1.5) are applied to the vendored MPO-700
in memory by `testbed/sim/base_swerve.py`; the vendored file stays byte-exact. Chassis
inertia is a box model at the scaled envelope (0.65 × 0.505 × 0.38 m mesh bbox).
Arm mount at x = 0.55 m, z = 0.3478 m (deck top, `scripts/measure_deck.py`).

## D5 — The electromagnet is the default end effector (2026-09-18)
`fr3_mpo700` carries an authored 1 kg electromagnet (Table 12 of the published Lunar
Robotic Construction System engineering design cited in `docs/reference/requirements.md`)
on the FR3 flange; attachment is a weld equality engaged within a capture distance/angle of the RCU's top
patch and released on command or above a breakaway force. The Franka hand remains
available as `fr3_mpo700_hand` (parallel kind). Reason: the flight interface is magnetic,
and a top-down magnetic pick removes the aperture/housing constraints a 0.12 m parallel jaw
imposes on a 0.43 × 0.25 × 0.10 m unit.

(Numbered D4/D5 rather than D3/D3a, since D3's slot above was already claimed by the
`test_cited_paths_exist` exemption entry — this ledger is append-only by number, not by
any draft numbering a plan document used first.)

## D6 — The weld equality's `eq_data` layout, confirmed empirically (2026-09-18)
For a `mjEQ_WELD` equality (`testbed/sim/gripper_magnet.py`), `model.eq_data[i]` is an
11-vector: `[0:3]` anchor (unused by this code, left 0), `[3:6]` the relative position of
body2 in body1's frame, `[6:10]` the relative orientation (wxyz quaternion) of body2 in
body1's frame, `[10]` torquescale. Confirmed by welding two free bodies at a known relative
pose (computed with `testbed.control.quat`), stepping dynamics under gravity for 200 ticks,
and reading the realized relative pose back to float64 precision — not asserted from the
MuJoCo docs alone. `testbed/control/quat.py` (`conj`/`mul`/`rotate`, pure numpy, wxyz
Hamilton product) is cross-checked against `mujoco.mju_mulQuat`/`mju_rotVecQuat` over 200
random unit quaternions (max error 0 / 1.1e-15) so `testbed.control` can use the same
algebra without a `mujoco` import. The magnet face normal sign and the exact capture
geometry are re-verified against a real RCU in D10 — this entry only needed the equality to
compose, compile, and (in a synthetic probe) actually hold a weld at the pose it was told
to.

## D7 — `mj.attach()`'s integrator conflict writes `MUJOCO_LOG.TXT` to the CWD, gitignored (2026-09-18)
Attaching the FR3 (which sets `<option integrator="implicitfast"/>`) onto the vendored
MPO-700 (which leaves `integrator` at its MuJoCo default) prints a `UserWarning` AND
appends a duplicate plain-text line to `MUJOCO_LOG.TXT` in the current working directory,
every time `compose()` runs — a stray, un-gitignored file can land in the repo root from
running the test suite. `testbed/sim/compose.py::compose` sets `mj.option.integrator`
explicitly on the live spec *after* every attach and *before* `mj.compile()` (already
required by the singleton rule), so the compiled model's integrator is correct regardless
of the warning; the warning itself is otherwise harmless and not suppressed. `MUJOCO_LOG.TXT`
is added to `.gitignore` rather than silenced: report it, don't hide it, don't let it recur.

## D8 — Per-wheel drive sign, measured from the mirrored vendored geometry (2026-09-20)
The first working runtime drove the swerve base and the base barely moved: commanding all
four wheels the SwerveAllocator's uniform way (same `wheel_radps` sign on every module for
a straight drive, exactly what `tests/test_swerve.py::test_straight_drive_all_wheels_forward`
locks in) produced wheel `qvel` tracking the 2.222 rad/s command almost exactly while the
base crept ~0.04 m in 2 s at a nominal 0.3 m/s (expected ~0.6 m), with ground contact
repeatedly dropping to 0-2 of 4 wheels throughout the drive phase (not just at start-up).
Measured root cause: the vendored MPO-700's left and right wheel bodies are mirror images
of each other, so the identical LOCAL drive-joint axis (`[0, -1, 0]`) resolves to OPPOSITE
WORLD directions per side — `fl`/`bl` world axis `(0, -1, 0)`, `fr`/`br` world axis
`(0, +1, 0)` [measured: scratch probe, 2026-09-20, `xmat @ jnt_axis` at the compiled
default pose] — so commanding the same sign on every module drove left and right sides
toward opposite senses, which mostly cancelled into wheel slip and, through the rigid
(suspension-less) chassis under lunar gravity, kicked the vehicle enough to intermittently
lift wheels off the ground. Fix: `testbed/sim/handles.py::ModuleHandles` gained a
`drive_sign` field, MEASURED once per module at `Handles.resolve()` time from the no-slip
rolling relation `v_center = -omega x r` (contact point straight below the wheel centre,
flat ground, z-up) — never asserted from the URDF/MJCF alone. `Runtime._apply()` writes
`ctrl = drive_sign * wheel_radps`; `sensors.observe()` reads back `wheel_radps = drive_sign
* qvel` so the OBSERVED sign matches the commanded one. Confirms with the correction:
`base_vx` settles at 0.299 m/s for a 0.3 m/s command, `ncon == 4` (all wheels grounded)
throughout, `base_vy ~ 0` [measured: scratch probe, 2026-09-20]. The fix deliberately does
NOT touch `testbed/control/swerve.py` — that module's own tests assert the uniform-sign
kinematics, and per D2 a per-vendor hardware sign convention belongs in `sim` (where the
compiled model's actual mirrored geometry is known), not in the robot-agnostic allocator.

## D9 — Keyboard teleop in the viewer at RTF ≈0.97; scripted headless demo saves an episode (2026-09-20)
`testbed/control/keyboard_rc.py` (pure: `KeyState` -> `TaskAction`/`JointAction`, held keys
drive, release or lost focus brakes — a deadman, not an event queue), `testbed/viz/
hud_text.py` (pure: `HudSnapshot` -> two aligned text columns), `testbed/sim/keyboard.py`
(GLFW physical-key polling, edge-detects `pressed` itself), and `testbed/sim/viewer.py` (a
real GLFW window + `mjr_overlay` HUD, physics paced to wall time, render decoupled at
60 fps). `app.py run --controller keyboard` now resolves to the real stack instead of a
not-yet-existing import.

One boundary conflict surfaced against the architecture's own documented shape ("`viz/
hud_text.py` draws a HudSnapshot" from inside the viewer): `tests/test_architecture.py`
mechanically forbids `sim` from importing `testbed.viz` (viz is meant to stay a leaf).
Resolved by dependency injection rather than by loosening the enforced boundary:
`Viewer.__init__` takes an optional `hud_render` callable (`HudSnapshot -> (left, right)`);
`testbed/bench/launch.py::run_mode` — which, unlike `sim`, may import both `sim` and
`viz` — passes `testbed.viz.hud_text.render` in. A `Viewer` built with no `hud_render`
(the viewer-tier test) still renders the 3D scene; only the text overlay is skipped.

Measured, not assumed: a real GLFW window is creatable from this execution context
(`glfw.init()`/`glfw.create_window()` both succeeded), and the full render path
(`MjvCamera`/`MjvScene`/`MjrContext`/`mjv_updateScene`/`mjr_render`/`mjr_overlay`) was
probed end-to-end against the composed `fr3_mpo700` model before `viewer.py` was written
[measured 2026-09-20: mujoco 3.13.0, glfw 2.10.2]. `tests/test_viewer.py` (`@pytest.mark.
viewer`) opens a real window and runs 250 ticks in `Viewer(rt).loop(max_s=0.5)` — passed on
this machine. A 2.0 s scripted-hold run through the real `Viewer` measured overall RTF
`rt.t_s / wall_s` = 2.0 / 2.063 ≈ **0.969** [measured 2026-09-20, this machine, cold GLFW
window creation included in the wall-clock denominator] — close to the pacing loop's
real-time target. `app.py run --controller keyboard --duration-s 0.3` end-to-end smoke run
(no simulated key input available in this environment) opened a window, ran 150 ticks, and
saved `episode.npz` + `meta.json` with no exception.

**Not covered by this measurement, and explicitly flagged as pending**: a human confirming at a
real desktop that specific keys produce specific robot behavior (W drives forward, U lifts
the flange, M+digit+`=`/`-` jogs a joint, SPACE toggles the gripper, X/R stop/re-arm, ESC/
Ctrl+C save and quit) — `KeyboardRc`'s behavior is unit-tested against synthetic `KeyState`
objects (`tests/test_keyboard_rc.py`), which proves the pure logic is correct, not that a
physical keypress reaches it correctly through a live GLFW window.

## D10 — magnet capture geometry: the face-normal sign was wrong, fixed and measured (2026-09-20)
The first real RCU body (`testbed/sim/worksite.py`: a superellipsoid-hull mesh via
`MjSpec.add_mesh(uservert=...)`, a freejoint, a `<rcu_id>_patch` site on the top face) let
`gripper_magnet.py`'s face-normal sign and capture geometry (D6's own note flagged this as
unverified) be checked against a real body for the first time. It was wrong. `attachment()`
computed `normal = -site_xmat[:, 2]` ("the magnet face points along -z of the site") and
then `ang = arccos(-normal @ patch_normal)`. With a real RCU seated exactly where the tool
would hold it (`Worksite.seat_in_gripper`: patch centre at the magnet face, patch normal
opposed to the tool's approach direction by construction), the measured angle came back
**pi rad (3.1416)** against a `capture_angle_rad` of 0.35 — exactly anti-aligned, so the
weld never activates for ANY real pose, not just an edge case.

Root cause: the authored magnet cylinder spans local z in `[0, height_m]` from the flange
mount (z=0) to the face (z=`height_m`, matching `interface_offset_m`), so the direction
OUT of the magnet into free space — where a target sits — continues in local **+z**, not
-z. Flipping the sign (`normal = +site_xmat[:, 2]`, `attachment()`'s dot product
unchanged) measures **7.5e-8 rad** (aligned) on the identical seated pose. With the fix,
`tests/test_worksite.py::test_energised_magnet_on_the_patch_captures_holds_lifts_and_breaks_away`
passes end to end: the weld activates (`attached_id == "rcu_0"`, `eq_active == 1`), holds
through a 0.25 m IK lift (measured lift 0.225 m in 4 s — the same ~0.025 m short of the
full 0.25 m commanded move that the IK's own 4 s convergence budget costs on every gate,
not a grip artifact), reads **|F_flange| = 40.7 N** against the RCU's lunar weight of
22 kg x 1.62 m/s² = **35.6 N** (arm dynamics account for the remaining ~5 N), and the
breakaway releases correctly: pinning the unit's mass to 1000 kg and continuing the lift
command drives `|F_flange|` above `hold_force_n` (200 N), which deactivates the weld
(`eq_active -> 0`, `attached_id -> None`) within the same tick budget. D6's weld
`eq_data` layout itself needed no change — only the face-normal sign that D6 explicitly
left for this verification.

`scenarios/single_placement.yaml`'s `rcu_0` placement also moved during this same
capture test: at an earlier draft's `x=2.5` the interface site sits 1.70 m from the RCU's
patch at spawn (the x1.5 FR3's `home` keyframe folds joints 4/6 to +-1.57 rad, a
STRETCHED-forward posture, not tucked — the flange sits ~1.38 m forward of `base_link`,
not near the arm mount), which is INSIDE `tag_read_range_m` (2.0 m) and made
`test_units_rest_where_staged_and_are_observable`'s "nothing tagged at spawn" assertion
fail. Moved to `x=3.0` — a measured correction, not an authored one; see
`docs/guide/surfaces_and_scenarios.md`.

**Correction (2026-09-21):** the distance this entry originally cited for
`x=3.0` (2.06 m) did not reproduce from a from-scratch re-measurement and needed tracking
down — not a wrong conclusion (both readings clear the 2.0 m gate), but an imprecise
citation, against this project's own convention that a `[measured]` number must
reproduce. Two DIFFERENT points on the RCU are 2.0 m-ish from the interface site, and the
original entry didn't say which one it meant:
  - **Interface site to the RCU's `<id>_patch` site** (the top face, `z = extents_m[2]/2`
    above the body origin) = **2.0325 m** [measured: `d.site_xpos[interface] -
    d.site_xpos[patch]` after `rt.reset()`, fresh compile, 2026-09-21] — the physically
    apt point (the patch is what a real proximity/tag sensor would be reading), and the
    figure an independent re-measurement landed on.
  - **Interface site to the RCU BODY ORIGIN** (`z = extents_m[2]/2` above ground, i.e. the
    `xyz_W_m` spawn point itself) = **2.0632 m** [measured: same script, `d.site_xpos[
    interface] - ws.rcu_pose_W(h, "rcu_0").pos_m`, 2026-09-21] — the point this entry
    originally meant, and, more importantly, the point `Worksite.observe()`'s tag-range
    check ITSELF uses: `OracleObject.pose_W` is built from `rcu_pose_W()` (the freejoint
    `qpos`, i.e. the body origin), never the patch site, so this is the number that
    actually governs whether `test_units_rest_where_staged_and_are_observable`'s
    `o.tags == ()` assertion passes or fails.
  Both exceed `tag_read_range_m` (2.0 m) either way, so the placement decision is
  unaffected — but the number that belongs in this entry, because it is the one the code
  path being tested actually computes, is **2.0632 m, not 2.06 m rounded from a different
  point**. `docs/guide/surfaces_and_scenarios.md` is corrected to match.

**Correction (2026-09-21):** the "40.7 N steady hold" above is an
AT-REST number, and it is not evidence that the magnet's 58.6 N contact-force budget had
margin. The 22 kg RCU's own weight is 35.6 N at lunar g, so 40.7 N against `fr3_mpo700`'s
41.02 N freeze-release threshold (0.7 x 58.6 N) was a ~0.3 N margin at rest; in motion the
flange sensor reads 100+ N (measured 140.5 N peak during transport, D15), so the budget
tripped throughout the carry. D12 later repeated the "thin but real margin" wording for
the magnet. D15 re-derives the budget from a measurement in motion.

## D11 — parallel-gripper payload: `gain_override` retuned from a guess to a measured plateau (2026-09-20)
`fr3_mpo700_hand.yaml` shipped a placeholder `gain_override: 4.0` before anything had
actually measured it. The hand gate
(`tests/test_worksite.py::test_hand_variant_grasps_the_unit_by_its_end`) swept
4/8/16/24/32/48/64/96/128 against a real seated 22 kg RCU (0.10 m face across the pads,
mu=1.0 per `RcuSpec.friction`) through the same 0.25 m / 4 s lift the magnet gate uses:

| gain_override | lift (0.25 m commanded, 4 s) | outcome |
|---|---|---|
| 4.0  | -0.177 m | slips out of the gripper entirely (net LOWER than start) |
| 8.0  | 0.185 m  | grips, but slip inside the fingers eats most of the margin |
| 16.0 | 0.205 m  | grips, thin margin over the 0.20 m gate |
| 32.0 | 0.209 m  | |
| 48.0 | 0.210 m  | |
| 64.0 | 0.2110 m | **plateau reached** — slip fully arrested |
| 96.0 | 0.2112 m | plateau (no further gain) |
| 128.0| 0.2111 m | plateau |

The squeeze-force/lift curve plateaus from 64x: the remaining ~0.039 m shortfall against
the commanded 0.25 m is the resolved-rate IK's own 4 s convergence (the SAME gap the
magnet gripper shows on the identical move, 0.2248 m of 0.25 m), not grip — so 64.0, the
smallest value ON the plateau, is the measured value, not merely "large enough to pass."
`robots/fr3_mpo700_hand.yaml`'s `gain_override` is set to `64.0` with this measurement
cited. The gate is NOT xfailed — the Franka hand variant holds the RCU at lunar gravity;
the default single-placement scenario (magnet) is unaffected either way.

`interface_offset_m` (the pad-centre offset already in the file, `[0, 0, 0.16]`) needed no
change — the seated pose it implies (patch flush at the pad centre) already grips
correctly once `gain_override` is high enough; the earlier `4.0` failure was purely a
force-authority problem, not a geometry one.

**Addendum (2026-09-21) — why 64x is a design choice, not a still-tuning
number:** `gain_override: 64.0` pushes `hand_actuator8`'s `forcerange` to +-6400 N, which
deserves an explicit rationale rather than an implied "the vendored servo is too weak."
Read directly off `assets/franka_hand/hand.xml` via `MjSpec` [confirmed: 2026-09-21], the
vendored tendon actuator's own `forcerange` is `[-100, 100]` N — a real Franka hand, sized
for real near-Earth, few-kilogram payloads. 64x is not a correction toward that hardware's
true capability; it is the SAME kind of move this whole robot YAML already makes with
`arm.scale: 1.5` and `base.footprint_scale: 2.5`/`module_scale: 1.5` (both tagged
`[authored-geometry: rover scale]`) — a deliberate surrogate scaling decision for a
FICTIONAL lunar-construction-strength actuator, because the mission payload (a 22 kg RCU)
was never in scope for the vendored hand's real hardware, geometrically scaled up 1.5x on
top of that. `gain_override` is a force/servo-authority analogue of the geometric scale
factors, made explicit rather than left to be inferred from "too weak" — see the expanded
comment in `robots/fr3_mpo700_hand.yaml` itself, which now states this directly rather
than only in this ledger.

**Tracking note (2026-09-21):** two numbers landed here with a real
but thin measured margin over their respective gates — the hand gate's 0.2110 m lift over
its 0.20 m threshold (1.1 cm) and, for the body-origin distance that actually governs the
`tags` gate (2.0632 m, per D10's correction), a margin over `tag_read_range_m`'s 2.0 m
of **6.32 cm** (`2.0632 - 2.0000`; the OTHER point on the RCU, the patch site at 2.0325 m,
margins narrower at 3.25 cm, but that is not the distance the gate computes against — see
D10's correction for why). Both are exactly reproducible today (nothing random sits
in either code path), but both would be the first two things to re-check if a future
change to `IkConfig`'s defaults, the physics timestep, `arm.scale`, or `hand`/`RcuSpec`
geometry ever makes one of these gates start flaking — neither margin was designed in on
purpose, both are simply where the measured plateau/geometry happened to land.

## D12 — The safety envelope: measured reach, F/T sensor sign, and a per-robot force budget (2026-09-21)
`testbed/control/envelope.py` (`Envelope.clamp_task`/`clamp_joint`, wired into
`Runtime._resolve_task`/`_resolve`, replacing an inline twist clip) gave `MujocoKinematics`
an `arm_base_B_m` attribute (`_arm_base_B_m`: the compiled arm root body's own `xpos`, read
once via `mj_kinematics` at the all-zero scratch qpos — not assumed equal to the spec's
`mount_pos_B_m`, though on this model it is, since the FR3's root body carries no local
offset of its own). Three things were measured rather than assumed:

**1. Measured reach exceeds the vendor's nominal figure.** `measure_reach_m` samples 2000
configurations uniformly over the FULL compiled joint-limit box (`kin.limits()`, the real
FR3 hardware ranges, not a "typical" subset) and takes the max `|flange - arm_base|`. A
naive estimate of `0.855 m x 1.5 (arm.scale) = 1.28 m` undersells it; the actual
measurement is **1.7747 m** (deterministic at `n=2000, seed=0`) — the arm's J5
(+-2.8065 rad) and J6 (0.5445..4.5169 rad, a >4 rad range) admit wrist configurations that
put the flange further from the base than any "arm reaching straight out" pose does. Cross-
checked: `fk(home)` gives 1.253 m, `fk(zeros)` gives 1.395 m, both well under the sampled
max. `tests/test_envelope.py::test_reach_is_measured_from_the_kinematics` asserts
`1.5 < r < 2.0` with the measurement cited, per D2's stance (kinematics come from MuJoCo,
not a table) and the "measure it, don't assert it" convention this ledger follows.

**2. The F/T sensor's sign convention needed measuring, not guessing.** A `Push` test
(drive the EE target to `z = -0.5` in B, straight down through the wheel-contact plane)
was built to trip the force limit; with the naive `force_backoff_sign = +1.0` (back
off ALONG the raw `ft_flange_B` vector), the commanded target diverged deeper into the
ground every tick instead of retreating (final `|F| = 132.7 N`, target z reaching -0.39 m,
well past `FLOOR_Z_B_M`). With `force_backoff_sign = -1.0` the same scenario is
deterministic and settles: `|F| = 26.5 N` after 6 s (105 "force" interventions logged), so
`Envelope`'s default is `-1.0`, documented at the constructor with this measurement.

**3. `force_budget_n` is per-robot, and reusing the magnet's number broke the hand
variant.** Both `robots/fr3_mpo700_hand.yaml` and `fr3_mpo700.yaml` shipped the SAME
`force_budget_n: 58.6` (the drawbar-pull-derived contact-force budget,
`docs/reference/requirements.md`, unrelated to the hand's own grip force). Wiring the envelope in made
`tests/test_worksite.py::test_hand_variant_grasps_the_unit_by_its_end` regress from a
0.211 m lift to **sinking** the RCU to -0.83 m: closing the x1.5 hand's 64x-gain grip
(D11's measured plateau) on the 22 kg RCU alone transmits ~50-52 N of reaction back through
the rigid hand/flange structure at REST, above the 58.6 N budget's 70 %-of-budget release
threshold (41.02 N) — so the freeze engaged on the very first tick and never released,
after which its own per-tick back-off (mechanism #2 above) fought the lift instead of a
real hazard, for every subsequent tick, regardless of what the controller commanded.
Measured with the force clamp neutralised (`force_budget_n=1e9`) to isolate the grip's own
true operating force: the SAME D11 grip-and-lift peaks at **83.6 N** transiently (acceleration)
and settles to **43.2 N** steady, reaching the expected 0.211 m lift. This is not a hazard —
it is the correctly-tuned grip D11 already measured and adopted — so the fix was not to
weaken the envelope but to give the hand variant its OWN, correctly-derived
`force_budget_n`: **125.0 N**, a ~1.5x margin over the measured PEAK transient (83.6 N, not
the lower steady value, so ordinary acceleration transients during a valid grasp never trip
it) — see `robots/fr3_mpo700_hand.yaml`'s own comment. The magnet's 58.6 N budget needed no
change yet (D10's own magnet hold measured 40.7 N steady, under its 41.02 N release threshold
by a thin but real margin — unaffected by this entry). Both gates
(`test_energised_magnet_on_the_patch_captures_holds_lifts_and_breaks_away` and the hand
gate) pass with the envelope active.

Suite after this entry: 71 passed / 1 deselected (was 67 / 1; +4 for
`tests/test_envelope.py`), viewer tier unaffected.

**Addendum (2026-09-21):** three findings, all closed, no redesign needed.

1. **The reach clamp and the reach bound were sourced from two different reference
   points.** `Envelope.__init__` set `self.mount_pos_B_m = robot.arm.mount_pos_B_m` (the
   YAML spec value) but `self.reach_m = measure_reach_m(kin)` already measured against
   `kin.arm_base_B_m` (the compiled model's true root-body position, added earlier in this
   same entry for exactly this reason). The two are numerically identical on this robot (the
   FR3's root body carries no local offset), so nothing caught that they were two unrelated
   values with nothing tying them together — a future arm whose root body DOES carry an
   offset would get a reach BOUND measured correctly but a reach CLAMP centred on the wrong
   point. Fixed by sourcing both from the same call, `_arm_base(kin)`.
   `tests/test_envelope.py::test_mount_reference_matches_the_true_arm_base` pins
   `Envelope.mount_pos_B_m == kin.arm_base_B_m` so this can't silently split again.
2. **`clamp_joint` had zero direct test coverage** despite being one of `Envelope`'s four
   documented interface methods and a shipped safety mechanism. Added
   `tests/test_envelope.py::test_clamp_joint_clamps_to_limits_and_reports` (direct
   invocation: a `q` 1 rad past every joint's own upper limit clamps to `kin.limits()`,
   reports `"joint"`, increments `counts["joint"]`; a `q` already inside limits reports
   nothing and leaves the count unchanged) and
   `test_joint_action_past_limit_is_clamped_by_the_runtime_and_counted` (a `Runtime`-level
   check: a scripted `JointAction` requesting past-limit joints comes back clamped in
   `rt.cmd.q_target_rad`, with `rt.obs.envelope_active` and `rt.interventions` both
   reflecting it).
3. **The force freeze's bypass was broader than first scoped.** The frozen back-off target
   skipped not just the floor clamp (as first flagged) but ALSO the reach clamp —
   `tgt = self._frozen` unconditionally overwrote whatever reach/floor-clamped value had
   been computed earlier in the same `clamp_task` call, and `self._frozen` accumulated its
   raw per-tick backoff vector with no re-clamp ever applied. Fixed rather than deferred,
   since it was cheap: `clamp_task` now computes the force-freeze branch FIRST and
   re-clamps `self._frozen` itself through a shared `_clamp_reach_floor` helper every tick
   (the same helper the ordinary non-frozen path uses), so the frozen pose can never walk
   past either bound no matter how many ticks it accumulates backoff for.
   `tests/test_envelope.py::test_the_force_freeze_never_escapes_reach_or_floor` drives the
   same `Push`-into-ground scenario, records EVERY frozen pose `clamp_task` produces across
   the whole run (not just the final one, since the freeze can release again before the run
   ends), and asserts each one sits on or inside both bounds.

Suite after the fix: 75 passed / 1 deselected (was 71 / 1; +4 for the new tests
above), all previously-green tests (including both worksite grip-and-lift gates) still
green.

**Correction (2026-09-21):** point 3 above concluded that "the
magnet's 58.6 N budget needed no change" because D10's at-rest hold read 40.7 N. That held
only at rest; the magnet's budget was never measured in motion, which is exactly the defect
this entry diagnosed and fixed for the hand variant. It tripped during every carry and, together
with the envelope writing its output back into the standing EE target (also fixed), dragged
the payload along the ground. See D15 -- the magnet's budget is now 210 N, derived the way
the hand's 125 N was.

## D13 — Placement geometry and the first scripted placement (2026-09-21)
Top-down magnetic pick from the ground; the slot is the second course centred on a
two-unit foundation (z = 0.15 m) so the placed-unit disturbance limit is measurable.
`testbed/control/placement_skills.py` implements `approach`/`park`/`acquire`/`transport`/
`place`/`seat`/`release`/`retract`/`verify`; `quat.from_matrix` and
`Observation.tool_offset_F`/`tool_kind` were added, `Runtime.reset()` measures the
flange->tool offset once off the settled compiled model (the same "measure, don't trust
the spec" pattern `Envelope`/`MujocoKinematics` already use for the arm base, D12).

Measured on flat, seed 0, `fr3_mpo700` (magnet): **cycle 67.1 s, pos err 6.6 mm, yaw err
0.15 deg, 1082 interventions** (957 `twist`, 125 `force`), **peak |F| 411.4 N** (at tick
9632, t=19.264 s) — see `docs/benchmarks/single_placement.md` for the full row and the
choreography notes.

**Correction (2026-09-21): the peak force this entry first recorded was
124.2 N, undersampled by ~3.3x.** `Runtime.tick()` calls `mujoco.mj_step()` at 500 Hz but
only refreshes `Observation` (and anything read off `Observation.ft_flange_B`, which the
first measurement used) at the 50 Hz control-tick rate. Re-measuring the SAME run by
sampling `sensors.ft_flange_B(h)` directly after every physics tick finds the true peak at
that same event is 411.4 N, not 124.2 N — the coarser sampling did not miss a different
spike, it under-resolved this one. This also means **the safety envelope's own force clamp
(`Envelope.clamp_task`) is blind to the same gap** — it only ever reacts to the 50 Hz
signal, by construction — a real, separate finding about the envelope, recorded because
the same event exposed it. Whether the episode log should record force at physics-tick
resolution for future benchmark accuracy is resolved (yes, already, by construction) in D14.

With `--robot fr3_mpo700_hand` the same script does NOT complete: `acquire` stalls forever
on its own first (pregrasp) waypoint, the flange drooping under gravity from world z ~0.99 m
(t=36 s) to ~0.69 m (t=60-100 s) rather than holding at the target. **Correction: an earlier
pass of this entry claimed "no self-collision measured at any sampled tick" — that was true
only of the first ~30-35 s actually checked at the time.** Extending the same stalled run to
t=40-100 s finds SUSTAINED self-collision throughout (`base_link` vs `fr3_link5`, roughly
-0.01 to -0.5 mm penetration, plus `fr3_link1`/`fr3_link0` vs `fr3_link6` at various points)
— the SAME `base_link`/`fr3_link5` signature the magnet path's own `standoff_m` fix
resolved, raising a real, untried possibility that a standoff adjustment could also help the
hand variant (its very different horizontal, near-ground end-grasp geometry gives no
guarantee the magnet's own value would transfer without its own sweep). The parallel jaw is
allowed to fail here (it is a documented surrogate, not the default path); the honest
diagnosis is "stalls at a shallow, gravity-drooping approach and DOES self-collide once
settled into it, same signature a standoff fix resolved elsewhere — untried here," not a
clean workspace limit.

Three real, measured problems, not tuning guesses, stood between a first-draft script (whose
`script_steps` needed only one number changed) and a passing gate:

1. **Self-collision, not a convergence issue.** At the scenario's original `standoff_m:
   0.9`, the arm's own forearm (`fr3_link5`/`fr3_link6`) physically collided with
   `base_link` while reaching down to ground level for `acquire`'s grasp pose — four-digit
   N·m servo torques fighting a real contact, the flange stalling ~3 cm short and never
   settling (confirmed by directly reading `data.contact` body-name pairs at the stuck
   tick, not inferred from the position error alone). A standoff sweep (0.9/1.0/1.1/1.2/1.3)
   found 1.0-1.2 clean (zero self-collision depth, `acquire` completing in ~22-23 s); 1.3
   clears self-collision but is too far out for the `up` lift to complete. `1.1` is now in
   `scenarios/single_placement.yaml` for both `approach` and `transport`.
2. **Two step-input problems, same shape, one level apart in the kinematic chain.**
   Energising the magnet the instant a fast, still-decelerating approach first swung
   through the ordinary `EE_TOL_M` (1.5 cm) band captured the 22 kg unit off-centre (up to
   1.34 cm) and while still moving; ground friction pinned a corner and the lift tipped the
   unit about it, growing to a stalled 10 deg tilt. Fixed with a tighter grasp-only
   tolerance (`GRASP_TOL_M`) plus CONTINUOUS-residency dwell (reset the "reached" clock the
   moment error leaves tolerance, not a first-crossing check) so energising only ever
   happens once genuinely settled. That alone still left a **751 N peak flange force**
   (about 21x the unit's 35.6 N lunar weight, measured with the envelope's own force clamp
   temporarily neutralised to isolate the cause — like the 124.2 N peak-force figure above,
   this 751 N reading was also taken from the 50 Hz `Observation.ft_flange_B` signal rather
   than the true 500 Hz physics-tick force, so it is itself likely an under-resolved read of
   a higher true transient; not re-measured, since doing so means reverting the fix it
   justified) at the tick `acquire`'s target stepped from a static hold to the `up` lift —
   `IkConfig.v_max_mps` bounds steady-state speed, not how fast the command can step there
   from zero, and `SwerveAllocator`/`ModuleFirmware` rate-limit only the steer angle, not
   drive wheel speed, so the BASE'S commanded twist has the identical gap. Fixed by
   feeding IK a rate-limited reference at both levels (`RAMP_RATE_MPS` for arm tool-position
   waypoints, `ACCEL_MAX_MPS2`/`YAW_ACCEL_MAX_RADPS2` for the base twist in
   `approach`/`park`) rather than the raw target — cheap because it only changes what an
   unmodified IK/envelope/allocator are handed, not either of those shared modules. The
   carried unit's tilt during `transport` fell from 37 deg (before the twist ramp) to under
   1 deg entering it.
3. **`seat`'s "stopped descending" check compared the wrong time scale.** A first-draft
   tick-to-tick comparison (`abs(z - last_z) < SEAT_STILL_M` every 0.02 s control tick)
   reports "landed" on its very first tick regardless of whether the unit ever touched
   anything, because a genuinely-still-descending unit only moves
   `SEAT_RATE_MPS * 0.02 s = 0.0004 m` per tick — already below `SEAT_STILL_M` (0.002 m).
   Measured: `seat` completed ~0.4 s after entering, with the unit still 0.28 m up (its
   `place` hover height); `release` then dropped it from there, and toppling on landing
   produced an initial 111.5 mm / 18.5 deg failure. Fixed by comparing TOTAL movement
   across one whole `SEAT_STILL_S` (0.4 s) window instead — the SAME constants, a
   different (and now dimensionally sane) comparison.

**Also found, recorded, not fixed:** the seated unit's final resting tilt (~4 deg) is a
real centre-of-mass/support-polygon edge case, not a control bug — its slot sits centred
exactly on the seam between `found_0`/`found_1`'s rounded (superellipsoid) top edges.
Raising the RCU's own torsional/rolling friction 10x changed nothing (ruling out friction
as the mechanism, since friction resists sliding/spin, not overturning), and the unit
slowly slides and topples if the simulation is simply left running past the script's own
completion (measured to t~210 s). `tests/test_single_placement.py::run_placement` now ticks until the script
itself reports done (capped at `sc.timeout_s` as a hang guard) rather than running
`rt.run(sc.timeout_s)` unconditionally and checking placement afterward — `Runtime.run`'s
own docstring is explicit that an explicit duration always runs the full time regardless of
`controller.done()`, so a first-draft `run_placement` would have always measured "does the
placement survive ~3.5 idle minutes," not "did the placement succeed" or "what was the
cycle time" (it would always print `t=285.0s` for the latter, never the actual completion
time). Fixing the seam itself needs a worksite-geometry change or a settle skill this
script doesn't call for — out of scope here.

Also fixed along the way, both measured: `spec.add_exclude` between the magnet body and
every candidate RCU (`gripper_magnet.add_weld_candidates`) — without it, a just-welded
RCU's mesh sat in permanent ~0.5 mm contact with the magnet cylinder, and the weld (holding
them together) and the contact (pushing them apart) fought every tick, the flange F/T
sensor oscillating 8-111 N while stationary; and a stiffer weld `solref`
(`WELD_SOLREF`, 0.004 s vs MuJoCo's own 0.02 s default — 2x the physics timestep, the
solver's own minimum-stable timeconst), a payload welded to a magnet is meant to behave
close to rigid. `testbed/bench/launch.py`'s `build()` also had a pre-existing,
never-exercised bug (`Worksite(sc)` instead of `Worksite(sc.worksite)`) that
`app.py run --scenario ...`'s first real smoke pass caught and fixed — it fails
immediately inside `Worksite.__init__` (`self.filled = [False] * len(spec.slots)` raises
`AttributeError`, since a bare `ScenarioSpec` has no `.slots`), not at `reset()` as an
earlier pass of this entry claimed; `Worksite.__init__` does not, in fact, duck-type enough
of `ScenarioSpec` to get past construction.

Suite after this entry: 80 passed / 1 deselected (was 75 / 1; +4 for `tests/test_skills.py`
and `tests/test_single_placement.py`, +1 because `test_cited_paths_exist` is parametrized one case per
living doc and this entry added `docs/benchmarks/single_placement.md`).

**Correction (2026-09-21): the placement numbers this entry recorded
(cycle 67.1 s, pos err 6.6 mm, yaw err 0.15 deg, 1082 interventions) are SUPERSEDED.** That
run never actually carried the payload: the force clamp's freeze back-off ratcheted the EE
target down until the unit was dragged along the ground for ~6 s / ~2 m (min carried z
0.0691 m against a 0.05 m rest height), and the run pushed the foundation blocks off their
blueprint positions (217.5 mm on the benchmark row). The placement gate only measured the placed
unit's own final pose, so it passed regardless. The write-back and the 58.6 N budget are
fixed in D15, which records the new headline numbers. The "**Also found, recorded, not
fixed**" paragraph's diagnosis of the ~4 deg residual tilt (the slot "on the seam") is not
verified either: it was measured with a foundation that had been knocked off its blueprint
positions. It is re-examined with an undisturbed foundation in D16.

## D14 — `testbed/bench/metrics.py`, `app.py bench`, and the force-sampling question resolved (2026-09-21)

**The peak-force undersampling D13 flagged is NOT a bug in the episode log — it never was
one.** `Runtime.tick()`'s call to `self.episode.append(self._observe(), ...)` computes a
FRESH `Observation` right after every single `mujoco.mj_step()`, unconditionally, once per
call to `tick()` — this is entirely decoupled from the coarser `decim`-gated refresh of the
CONTROLLER-facing `self.obs` that D13's 124.2 N figure was read from. Confirmed two ways:
(1) `tests/test_runtime.py::test_episode_records_every_tick` already pinned this before
this entry touched anything — `rt.run(0.1)` at `dt=0.002 s` produces 50 rows, not 5 (the
50 Hz control-rate row count); (2) re-running `tests/test_single_placement.py::run_placement(record=True)` and
reading `arrays["ft_B"]` back from the saved episode reproduces the exact 411.44 N peak at
the exact tick (9632) D13's hand measurement found by manually sampling every physics tick.
So `testbed/bench/metrics.py::placement_metrics`'s `peak_force_n`, computed straight from the
recorded episode, is already at the correct resolution — no undersampling caveat needed,
and no runtime/episode change was required to fix it (the fix was already latent in how
`episode.append` is called). `docs/guide/running.md`'s stale claim ("schema 1: one row per
control tick") was wrong and is corrected alongside `docs/guide/episodes_and_replay.md`.

**Two uncommitted changes found at the start of this work (`Observation.wheel_contact`
in `testbed/control/records.py`, `sensors.wheel_contacts()`) were correct and kept.** They
matched `ModuleHandles.wheel_body` and the existing geom1/geom2-order caution exactly; the
only gap was that nothing actually WIRED `wheel_contacts(h)` into `sensors.observe()`'s
returned `Observation`, so it was dead code until `wheel_contact=wheel_contacts(h)` was
added to the `Observation(...)` call.

`EpisodeLog.append` gained three new columns `placement_metrics` actually needs: `unit_pos_W`/
`unit_quat_W` (recorded whenever `obs.oracle is not None`, i.e. whenever a `Worksite` is
attached) and `wheel_contact`. `Runtime.save` gained `stops`/`helps` in `meta` plus, when a
worksite is attached, `unit_ids`/`slots`/`to_place` — everything `placement_metrics` needs to
find "the unit being placed" and "its target slot" from the episode alone. `SCHEMA`
bumped 1 -> 2 (new columns; `tests/test_runtime.py`'s own schema assertion updated with
it).

`tests/test_single_placement.py::run_placement` gained a `record: bool` parameter (attaches an `EpisodeLog`
before `reset()`, so the WHOLE run is recorded, not a re-attached tail).

`app.py bench --scenario ... --seeds ... [--no-append]` and `testbed/bench/launch.py::
bench_mode` (not `testbed/sim/launch.py` — `launch.py`'s own module docstring already
explains why `bench_mode`'s sibling `run_mode` lives in `bench/`: `sim` may not import
`bench`, and this glue needs `EpisodeLog`) always run `controller=scripted` — a benchmark
row is a baseline number, not an arbitrary run; `app.py run` already covers keyboard/
policy. Measured for real: `app.py bench --scenario single_placement --seeds 0` ->
`pos_err_mm=6.6 yaw_err_deg=0.15 cycle_s=67.1 peak_F_N=411.4 interventions=1082`, matching
D13's own hand-measured row exactly, and appended a machine-generated row under a new
"## Recorded rows" section in `docs/benchmarks/single_placement.md` (the file already
existed as a hand-written prose report with no table header at all — the first
`append_row` call landed a header-less row at the very end of the file; caught before
committing, fixed by adding a proper `## Recorded rows` section with the table header
first, then re-running `app.py bench` to append the real row under it).

Suite after this entry: 84 passed / 1 deselected (was 80 / 1; +2 for `tests/test_metrics.py`,
+2 more `test_cited_paths_exist` parametrizations for the two new guides).

**Correction (2026-09-21) to this entry's headline row:** the row quoted above
(`pos_err_mm=6.6 yaw_err_deg=0.15 cycle_s=67.1 peak_F_N=411.4 interventions=1082`) came from the
payload-dragging run described in D13's correction and is superseded by D15's row. The peak-force
RESOLUTION finding (the episode is at 500 Hz, the observation at 50 Hz) is unaffected.

## D15 — The envelope filters, it does not rewrite: the single-placement run was dragging the payload (2026-09-21)

The single-placement run behind D13/D14's headline numbers did not carry the payload. Two coupled
causes, both re-measured here from a fresh run.

**(a) A one-way ratchet.** `Runtime._resolve_task` stored `Envelope.clamp_task`'s OUTPUT as the
standing EE target (`self.ee_target_B, tw, rep = self.envelope.clamp_task(self.ee_target_B, ...)`).
During a force freeze `clamp_task` returns its `_frozen` pose, which accumulates a 5 mm back-off
per control tick, so the backed-off pose became the standing command; when the freeze released
the envelope was handed the already-ratcheted target. The reach and floor clamps wrote back the
same way (a target clamped to reach while the base was still far from its stance stayed clamped
after the base arrived). A shield that must be transparent when there is no hazard was not.
**Fix:** `self.ee_target_B` is now only ever written by a controller's own `TaskAction`; the
envelope's output (`ee_cmd_B`) goes to IK for that control tick only and is carried on
`ResolvedCommand.ee_target_B`. `Envelope._frozen` stays internal. Reach/floor/twist/joint clamps
and the force freeze all still act on what reaches IK and the actuators, and the freeze still
backs off and still releases. Gated by `tests/test_envelope.py::
test_the_envelope_filters_the_command_and_never_rewrites_the_standing_target` and
`test_a_reach_clamped_target_is_clamped_for_the_tick_not_forever`; both were confirmed to FAIL
against the pre-fix runtime.

**(b) A drawbar figure used as a wrist limit.** `fr3_mpo700.yaml`'s `force_budget_n: 58.6` came
from a drawbar-pull equation (0.21 pull/weight at 344.63 kg lunar, SF 2 — see
`docs/reference/requirements.md`'s contact-force budget entry for the full derivation) and was
used as a limit on the FLANGE F/T sensor, which also reads the carried payload's weight (22 kg =
35.6 N at lunar g) and its inertial load. D10 had measured the steady hold at 40.7 N against a
41.02 N release threshold, at rest; D12 diagnosed and fixed exactly this defect for the hand
variant (125 N, measured in motion) but left the magnet at 58.6 N because it had only been
measured at rest.

**Re-measurement (2026-09-21; flat, seed 0, magnet; force clamp neutralised with
`force_budget_n=1e9` so the loop is undisturbed; `ft_B` at the 500 Hz physics-tick rate):**

| phase | 500 Hz peak F | notes |
|---|---|---|
| approach / park | 5.5 / 42.0 N | |
| acquire | 411.4 N | ONE tick, t=19.264 s: the magnet-capture weld snap (attached 0 -> 1; F 6 -> 411 -> 14 N in three ticks) |
| **transport** | **140.5 N** | the payload carried through free space |
| place | 53.7 N | |
| seat | 357.7 N | the press-in, until the magnet's 200 N `hold_force_n` breakaway starts flickering the attachment |
| release / retract / verify | 358.0 (first ticks) / 7.3 / 5.2 N | |

**An A/B, reproduced in part** (same seed and script, only `force_budget_n`
changed, pre-fix runtime): 58.6 N -> carried unit's min z during `transport` **0.0691 m**
(rest height 0.05 m) and `found_0` ends at (2.395, 3.188), 218 mm
off blueprint; 400 N -> min z **0.2398 m**. The foundation displacement is NOT a symptom of
the dragging: it persists once the payload is carried, and is a separate defect (the
`transport` stance path sweeps the chassis through the foundation), handled in D16.

**New budget: 210.0 N** = 1.5 x the measured in-motion carry peak (140.5 N), following D12's
precedent (hand: 1.5 x 83.6 N = 125 N). The one-tick 411 N capture snap and the seating contact
are contact events, not carry, and are excluded from the derivation. With it, force-kind
interventions over the whole single-placement run are **0** (they were 125 with 58.6 N, 115 of them during
`transport` even after fix (a)).

**Honesty note: the testbed no longer enforces the contact-force budget for the
magnet robot.** The 58.6 N figure (`docs/reference/requirements.md`) is a SEATING contact-force
requirement; the flange sensor reads contact force PLUS the carried payload's weight and
inertial load, so one limit on the flange reading cannot be both "does not trip while carrying
22 kg" and "58.6 N of contact". The measured consequence is in the seat phase: with 58.6 N the
freeze capped the seating press-in (seat-phase peak 82 N, mean over the last 0.5 s 44.8 N); with
210 N the press-in runs up to the magnet's 200 N breakaway (peak 357.7 N, last-0.5 s mean
**163.7 N**). The placement itself is unaffected (pos err 7.5 mm, yaw 0.43 deg), but the seating
force is now ~3.6x the old figure and nothing in the testbed limits it below the magnet's own
breakaway.

**OPEN OPTION, not implemented (see `docs/reference/requirements.md` for the tradeoffs):**
subtract the known carried-payload load (mass x (g + measured flange acceleration), from
`attached_id`'s mass and the flange kinematics) from the flange reading before the envelope sees
it, so the force clamp monitors CONTACT force only and 58.6 N becomes meaningful again for the
magnet robot. Until it is implemented, the honest statement is: the seating-contact requirement
is unenforced for `fr3_mpo700`; `seat_force_n` is recorded per run (a metric, not a gate).

**Other changes in this fix:** `EpisodeLog` schema 2 -> 3 adds `interventions_by_kind` (one
column per `envelope.KINDS`; `meta.intervention_kinds` names the order), because force and twist
interventions were indistinguishable in the single cumulative `interventions` column. The
benchmark row gained `force interv` and `min carried z m` (a new table header is appended below
the old one; old rows are untouched); `PlacementMetrics` gained `force_interventions`,
`transport_force_interventions`, `carried_min_z_m`. `report.append_row` writes with
`newline="\n"`. New gates in `tests/test_single_placement.py` (one shared recorded run via the `placement_run`
fixture, so the tier still simulates the placement once): the carried unit's min z during `transport`
must stay >= rest height + 0.09 m (about half the measured 0.190 m lift), and `transport` must
trip the force clamp 0 times (measured 0); each was confirmed to fail against the defect it
guards (pre-fix runtime + 58.6 N: min z 0.0691 m; fixed runtime + 58.6 N: 115 force
interventions in `transport`). The placed-unit disturbance limit (<= 10 mm) is gated as well and
is `xfail(strict=True)` at this commit: it does not pass yet, see D16.

**Before / after (flat, seed 0, magnet):**

| | old row (dragging) | new (D15) |
|---|---|---|
| cycle | 67.1 s | 85.1 s |
| pos err / yaw err | 6.6 mm / 0.15 deg | 7.5 mm / 0.43 deg |
| peak F (500 Hz) | 411.4 N | 411.4 N (same one-tick capture snap) |
| seat F (mean, last 0.5 s of `seat`) | 44.8 N | 163.7 N |
| interventions | 1082 (957 twist, 125 force) | 2014 (2014 twist, 0 force) |
| foundation displacement (max over found_0/found_1) | 217.5 mm | 176.5 mm |
| min carried z during `transport` | 0.0691 m | 0.2398 m |
| final z vs slot z | 0.147 m | 0.147 m |

Interventions rose because `transport` now takes 44.4 s rather than 23.6 s, and throughout it
the base twist saturates the 0.5 m/s cap on ~86 % of control ticks (1910 of 2219): the
`approach`/`transport` skills ramp their commanded twist toward an unbounded P-law desired value
and cannot see the cap, so the shield clips it every tick. That is a skill/shield seam (skills
carry no robot spec), recorded and parked (see D19), not fixed here; all 2014
are `twist`, none `force`.

## D16 — The chassis was shoving the foundation; the "seam" tilt diagnosis was wrong (2026-09-21)

**Foundation disturbance (the placed-unit disturbance limit), root-caused here.** The
machine-generated benchmark row carried `found. disp mm = 217.5` on a PASS row (21x the 10 mm
tolerance) and nothing looked at it. After D15 (payload now carried) it persisted at 176.5 mm,
falsifying a guess that it would vanish once the payload stopped dragging.

Mechanism, measured by stepping the pre-fix scenario to the first tick `found_0` moves more
than 0.5 mm: t = 62.488 s, phase `transport`, base at (1.401, 2.282) m, yaw -0.97 deg, and the
only contact on `found_0` is `mpo_700_wheel_front_left_link` at (2.097, 2.891, 0.081) m.
`transport`'s stance for `slot_0` is 1.1 m behind the slot along -x, x_base = 1.4 m. The base
arrives there from rcu_0's stance (1.9, 0) by the skill's P-law, which closes the x error
first and then drives +y at x_base ~ 1.4 m. The front wheel spheres reach x_base + 0.6 m
(module) + 0.135 m (radius) = 2.135 m, and `found_0`'s near face is at x = 2.285 - 0.215 =
2.07 m, so the front-left wheel line (y_base + 0.45 m) crosses `found_0`'s y span [2.875, 3.125]
while the base is still en route, so the wheel plows into the block. At the FINAL stance the
wheels straddle the foundation (they sit at y = 3.0 +- 0.45 m) and touch nothing; only the
transit sweeps it. Both blocks end up rotated (~27-32 deg) as well as displaced.

**Fix (scenario, not skill): a via stance.** `scenarios/single_placement.yaml` gains an
`approach` to `slot_0` at `standoff_m: 1.5` (x_base = 1.0 m; wheels 0.335 m short of the near
face during the transit) before `transport`'s final 1.1 m stance (the last 0.4 m then runs with
the wheel lines 0.19 m clear of the foundation's y span). Measured with it: `found_0` / `found_1`
displacement **0.6 / 0.9 mm** (was 176.5 / 95.8 mm). A via of 1.4 m also measured clean (0.6 /
0.9 mm); 1.5 was kept for the extra margin against stance tolerance (5 cm) and yaw wobble
(0.6 m x sin 4 deg ~ 4 cm). Limits, not solved here: the via step is authored per scenario, the
skills do no obstacle-aware base path planning (parked, D19), and the standoff choice is coupled to
the foundation's footprint.

**The ~4 deg residual tilt and the "topples by t~210 s" claim were artefacts of that
disturbed foundation.** D13 and the benchmark doc attributed them to the slot sitting "exactly
on the seam between found_0 and found_1" and their rounded superellipsoid top edges. Neither
was true of the foundation as actually placed: `found_0` was 110 mm off in x and 188 mm off in
y at seating time, and the unit rested on five contact points in two small clusters, not the
designed symmetric two-block support. The friction sweep (10x torsional/rolling, no effect) was
sound; what nobody checked was whether the foundation was still where the blueprint put it.
Re-measured with the foundation undisturbed (flat, seed 0, magnet, ticked through the full 285 s
timeout, 200 s past script completion): residual tilt **0.010 deg** at completion and 0.003-0.020
deg throughout; z 0.1485 m throughout; the unit does NOT topple; its position creeps 0.076 ->
0.194 mm over the 200 idle seconds (~1 um/s, `found_0` drifting 2.2856 -> 2.2848 m with it). The
slot IS still on the seam (the blocks abut exactly), and that is harmless. The corrected cause of
the old tilt is "a knocked-off foundation", not the seam geometry, and no worksite-geometry
change is needed. D13's paragraph and the benchmark doc's "Known limitation" carry dated
corrections pointing here.

**Before / after D16 (flat, seed 0, magnet):**

| | old row (dragging) | after D15 | after D16 (current) |
|---|---|---|---|
| cycle | 67.1 s | 85.1 s | 81.2 s |
| pos err / yaw err / tilt | 6.6 mm / 0.15 deg / ~4 deg | 7.5 mm / 0.43 deg / 4.5 deg | 0.076 mm / 0.0086 deg / 0.010 deg |
| peak F (500 Hz) | 411.4 N | 411.4 N | 411.4 N (the one-tick capture snap) |
| seat F (mean, last 0.5 s of `seat`) | 44.8 N | 163.7 N | 179.8 N |
| interventions | 1082 (125 force) | 2014 (0 force) | 1048 (0 force) |
| foundation displacement | 217.5 mm | 176.5 mm | 0.9 mm |
| min carried z (carry window) | 0.0691 m | 0.2398 m | 0.2402 m |

**Gates (retuned, record-then-gate).** `tests/test_single_placement.py` now gates pos err < 1 mm, yaw < 0.1 deg,
tilt < 0.5 deg, foundation displacement <= 10 mm (the placed-unit disturbance limit), carried-unit
clearance and zero carry-window force interventions. The pos/yaw/tilt gates are NOT 2x the
measured values (0.15 mm / 0.017 deg / 0.021 deg): those sit below the float noise a
contact-rich 81 s run can carry between the ubuntu and windows CI legs, so each is floored
(tagged `[policy default: numerical-noise floor]`) at a value still far tighter than the
placement position/orientation tolerances (25 mm / 6 deg -- 25x and 60x respectively) and
than the old 15 mm / 1 deg gate.
This is a judgment call worth a second look if CI ever disagrees with this box.

**Metric definitions changed with the new scenario:** the carried-unit window is now "after
`acquire` ends, before `seat` begins" (`PlacementMetrics.carried_min_z_m`, `carry_force_interventions`),
not "the phase named `transport`", because the carry is now split across the via `approach`, the
final `transport`, `park` and `place`. **Budget re-check:** the in-motion carry peak re-measures
138.4 N (was 140.5 N), so `fr3_mpo700`'s 210 N is 1.52x -- unchanged. The seat press-in is still
uncapped by the force clamp (D15's honesty note): peak 380 N and last-0.5 s mean 179.8 N.

## D17 — Metrics, determinism and enforcement fixes (2026-09-21)

Each item below was reproduced before it was changed.

- **`phase_s` timed a repeated phase from its first visit's start to its last visit's end.**
  `EpisodeLog` gives one index per phase NAME and the scenario runs `park` twice, so `park` "took" 41.82 s
  (window from the first park to the second, swallowing `acquire` and `transport`) and the phase
  durations summed to 107.36 s against a 67.08 s cycle -- reproduced from the saved episode of the
  shipped run. `metrics.phase_durations_s` now sums each contiguous VISIT; the same episode gives
  67.06 s (`park` 1.52 s). Gated by a direct synthetic-episode unit test and by
  `sum(phase_s) <= cycle_s` on the real one.
- **`--seed` is inert.** `Runtime` builds `np.random.default_rng(seed)` and passes it to
  `Worksite.reset`, which never draws. Reproduced: the seed-0 and seed-1 episodes of the shipped
  run are equal in every column (the only "difference" is NaN != NaN in `gripper_aperture_m`). No
  randomness was invented (YAGNI). Instead: `docs/guide/running.md` and `benchmarks.md` say plainly
  that nothing is stochastic yet and the seed is reserved for condition sampling that is not
  implemented yet; `app.py bench` prints a one-line notice when given more than one seed (its
  example `--seeds 0 1 2` appended three identical rows); `test_same_seed_is_bit_identical`'s
  docstring now says it proves determinism, not seed sensitivity.
- **`help` had two meanings.** `help_count` was both the counter and the latch: with no `rc` it
  incremented on every control tick the flag was up; with one it counted once and then latched
  forever. Now `help_count` counts rising EDGES of `help` (one intervention per help event) and
  the hand-over to `rc` is a separate state (`_rc_active`). Returning control to the controller is
  deliberately undefined for now; no shipped path passes an `rc`, so from `app.py` `H` only
  counts. `docs/guide/teleop.md` said it "hands control to the keyboard"; corrected.
- **`app.py run --controller policy`** raised an ImportError (`PolicyAdapter` is not implemented
  yet). It now prints one line and exits 2; `docs/guide/controllers_and_policies.md` says so.
- **`launch.py` / `report.py` had zero test coverage** (and `launch.build` had already shipped one
  latent `AttributeError`, D13's `Worksite(sc)` bug). Added `tests/test_launch.py` (`app.main` end
  to end for `run` with and without a scenario, the policy failure, cwd-independent scenario
  resolution, `bench` end to end) and `tests/test_report.py` (`row`/`append_row`: header once,
  append-only, new header when the columns change, LF only). `bench` gained `--out` and
  `--runs-dir` so no test touches the tracked benchmark doc (asserted byte for byte) or the repo's
  `runs/`. `launch.build` resolved `scenarios/` against the CWD; it now uses the repo-anchored
  `SCENARIOS_DIR` like `bench_mode`.
- **`docs/reference/records.md`** (cited as THE field reference for policy authors) lacked
  `Observation.tool_offset_F`, `tool_kind` and `wheel_contact`; added, and the `envelope_active`
  and `force_n` descriptions corrected. `tests/test_records_doc.py` now fails when a record field
  is undocumented or a documented one no longer exists.
- **Enforcement that read stronger than it was.** `test_tag_vocabulary_is_closed` only ever
  flagged `[authored:`; `[policy]`, `[measured-ish: x]` and `[derivedfrom: y]` all passed
  (verified). It now requires every tag-shaped token in code, tests, YAML and the living docs to
  match the closed vocabulary, with positive and negative unit tests of the matcher. Fallout it
  found and fixed: five prose tags in `testbed/` written as `[measured 2026-09-20: ...]`.
  `test_locked_package_layout` asserted five `__init__.py` files exist and locked nothing; it now
  asserts the set of subpackage directories under `testbed/` equals {spec, control, sim, bench,
  viz} and that each has a boundary rule. `EXEMPT_PLANNED_PATHS` (D3) is now empty, with a test
  that fails on an exemption for a path that exists.
- **Ctrl+C through the `app.py` shim** (reasoned, not measured -- no real signal is fired in any
  test). `_reexec` ran the child under a bare `subprocess.run`: a terminal Ctrl+C reaches both
  processes, and `subprocess.run`'s interrupt path gives the child ~0.25 s and then kills it,
  before `run_mode` has saved a ~33k-row episode, so `docs/guide/teleop.md`'s "the episode is saved
  either way" was likely false for terminal interrupts. `_reexec` now installs a PYTHON-LEVEL no-op
  SIGINT handler for the duration of the wait (not `SIG_IGN`, which is inherited across exec and
  would stop the child ever seeing Ctrl+C), restores the previous handler afterwards, and relays
  the child's exit code (signal death -N -> 128+N); the child (`Viewer`'s SIGINT handler -> close
  -> save) owns shutdown. Unit-tested at the handler-installation level. Not exercised on a real
  Windows console, so the guide's sentence was reworded to say exactly that.
- **Smaller:** the scenario YAML's "x=3.0 measures 2.06 m" under a `[measured]` tag now says
  2.0632 m and names the reference point (RCU body origin; the patch site is 2.0325 m, D10);
  `docs/guide/surfaces_and_scenarios.md` no longer shows `standoff_m: 0.9` (D13 measured that as a
  hard self-collision; the scenario uses 1.1) and explains the via stance; the unused `field`
  import is gone from `testbed/spec/robot.py`.

## D18 — The CRLF / autocrlf episode, recorded (2026-09-21)

`.gitattributes` used to cite a decision entry that never existed under the slot its own note
expected (the intended slot went to D4's "Rover scale" instead, and the original CI-fix report
lived only in a private working notebook), so from a clone that citation dangled. This is the
entry that closes the gap.

**What happened (2026-09-18; commits `908bab9`, `2b7565f`, `2920fc6`).** The dev box has
`core.autocrlf=true`. A vendoring script cloned `mujoco_menagerie` and `swerve-nav` there, and
git's checkout-time CRLF conversion turned the working-tree copies of the vendored text files
into CRLF versions that byte-differ from the (correct, LF) committed blobs. `assets/PROVENANCE.md`'s
sha256 ledger was computed from those converted working-tree copies. At commit time autocrlf
normalised the files back to LF, so what git stored was LF while the ledger's digests described
CRLF bytes -- bytes the Linux CI leg, which checks out the LF blobs, never sees.

**Fix, in three steps.** (1) `assets/** binary` (-diff -merge -text) so git never re-encodes a
vendored file's line endings on any platform, matching the byte-exact vendoring invariant; (2) the
working tree was restored from the LF blobs and the ledger regenerated (53 digest lines changed);
(3) `scripts/vendor_assets.py::ledger()` wrote PROVENANCE.md with `Path.write_text()` and no
`newline=`, so on Windows every `\n` became `\r\n` on write -- a smaller instance of the same bug
that autocrlf's commit-time normalisation had been silently masking (the regenerated ledger from
step 2 landed CRLF in the committed blob, and CI went red again). Fixed at the root
(`write_text(..., newline="\n")`) and guarded with `assets/PROVENANCE.md text eol=lf`.

**The follow-on lesson.** `binary` REMOVES autocrlf's silent safety net for repo-AUTHORED files
under its glob, which is why PROVENANCE.md needed an explicit eol rule the moment `assets/**` went
binary -- and why any other file an unattended script appends to needs one too. The benchmark docs
are exactly that (`testbed/bench/report.py::append_row` appends rows to tracked
`docs/benchmarks/*.md`; confirmed during this fix that the Windows checkout of
`single_placement.md` was CRLF, so an LF-writing append produced a mixed-EOL working file).
`.gitattributes` now pins `docs/benchmarks/*.md text eol=lf` and `report.py` writes with
`newline="\n"`. Rule: a script that writes a tracked text file passes an explicit `newline=`, and
a tracked file such a script writes gets an explicit eol rule.

## D19 — Parked, deliberately unresolved limitations (2026-09-21)

Findings that were consciously NOT fixed, so nothing is silently dropped. Each has a ruling; none
blocked the working single-placement baseline.

- **Stale FK in `sensors.observe`.** Post-`mj_step` FK arrays (`site_xpos` etc.) are one step stale
  while `qpos` is fresh; measured 0.598 mm at 0.3 m/s. Real but sub-millimetre; the base is parked
  during seating, so it is not the placement-error source. DEFERRED, since it needs consistent
  state for a base attitude/coupling analysis that is not built yet. Cheap future fix:
  `mj_kinematics` before reading site poses (no dynamics side effects).
- **`envelope.clamp_joint` clamps position only;** the architecture also anticipates velocity
  limits (`docs/design.md`). DEFERRED to whenever a `PolicyAdapter` lands (the only source of
  unbounded `JointAction`s; the keyboard jog and the IK are already rate-limited).
- **`KEY_DT_S = 0.02` in `keyboard_rc.py`** hardcodes the 50 Hz control tick independent of
  `control_hz`, so jog speed mis-scales at other rates. LEAVE: changing it alters the `KeyboardRc`
  constructor contract.
- **The envelope's force clamp only sees the 50 Hz observation** (D13's finding), so a one-tick
  500 Hz transient (the 411 N magnet-capture snap) can fall between control ticks. Unchanged; the
  force budget was derived from the 500 Hz record instead. A fix (checking the max over the last
  `decim` physics ticks) belongs with whatever eventually implements D15's open option.
- **Skills cannot see the base speed caps.** `approach`/`transport` ramp their commanded twist
  toward an unbounded P-law value, so the shield clips the twist on most carry-leg control ticks
  and the carry is slower than the caps alone imply (D15 measurements). Fixing it needs the skills
  to know the robot's limits (a `start()` signature/seam change) — not done here.
- **No obstacle-aware base path planning.** The scenario authors the via stance (D16); a worksite
  the chassis can hit needs a route authored per scenario until a planner exists.
- **A human at a real desktop confirming the interactive keyboard path is PENDING.** Pressing keys
  in the live window (W drives, U lifts, M+digit+`=`/`-` jogs a joint, SPACE toggles the gripper,
  X/R stop/re-arm, ESC/Ctrl+C save and quit) cannot be verified by an automated test — D9's list of
  what its own automated smoke run does NOT cover still stands, as does the Ctrl+C path through a
  real Windows console (D17).
- **The 58.6 N contact-force budget (`docs/reference/requirements.md`) is unenforced for
  the magnet robot,** and D15's "subtract the carried-payload load" option awaits a ruling
  and an implementation.
- **Everything already marked LEAVE, unchanged:** the vendoring script's incidental log output,
  `fetch()` partial failure, skill-duration-during-stop semantics, the `drive_sign` flat-ground
  assumption, the `wants_rearm` test, the hand-gate and spawn margins (D11's tracking note), and
  the hand variant not completing the placement (D13).
