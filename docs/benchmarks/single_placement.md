# Single placement (scripted baseline)

Scenario: `scenarios/single_placement.yaml`. Robot: `robots/fr3_mpo700.yaml` (default,
top-down electromagnet). Surface: `surfaces/flat.yaml` (lunar gravity, flat ground).
Controller: `ScriptedSequencer` running the scenario's own `script_steps` through the
`approach`/`park`/`acquire`/`transport`/`place`/`seat`/`release`/`retract`/`verify` skills
(`testbed/control/placement_skills.py`).

> The scenario, this file and the metrics were renamed on 2026-10-01 (`b1_single_placement`
> -> `single_placement`, `b1_metrics` -> `placement_metrics`, `B1Metrics` ->
> `PlacementMetrics`). Rows and `[measured: ...]` tags dated before then were produced by
> this same scenario under the old name; the commands as written below reproduce them.

A run is measured by ticking until the script itself reports done (`ScriptedSequencer.done`),
capped at `sc.timeout_s` (285 s) as a hang guard — not by running the full timeout duration and
checking afterward (see `tests/test_single_placement.py::run_placement`'s own docstring: `Runtime.run(duration_s)`
always runs the full duration, so it could not report when the script actually finished).

## Current measured row (after DECISIONS.md D15 + D16: payload carried, foundation undisturbed)

The sections below ("Measured row, original baseline" and friends) are kept as history and are
**superseded**: that run never carried the payload. The envelope's force freeze back-off was
written back into the standing EE target, and the magnet's 58.6 N force budget (a drawbar
figure, and an at-rest measurement) tripped throughout the carry, so the target ratcheted down
until the unit was dragged along the ground (min carried z 0.0691 m against a 0.05 m rest
height). Both are fixed in D15. A second, independent defect -- the base's transit to
`transport`'s stance sweeping its front wheel through `found_0` -- is fixed in D16 by a via
stance in the scenario.

| metric | old row (superseded) | after D15 | after D16 (current) |
|---|---|---|---|
| cycle time | 67.1 s | 85.1 s | 81.2 s |
| position error (xy) | 6.6 mm | 7.5 mm | 0.076 mm |
| yaw error | 0.15 deg | 0.43 deg | 0.0086 deg |
| residual tilt | ~4 deg | 4.5 deg | 0.010 deg |
| peak `\|F\|` at the flange (500 Hz `ft_B`) | 411.4 N | 411.4 N | 411.4 N (the one-tick magnet-capture weld snap, t=19.264 s) |
| seat force (mean, last 0.5 s of `seat`) | 44.8 N | 163.7 N | 179.8 N (the 58.6 N budget no longer caps the press-in; D15) |
| interventions | 1082 (957 twist, 125 force) | 2014 (all twist) | 1048 (all twist, 0 force) |
| foundation displacement (placed-unit disturbance limit, tolerance 10 mm) | 217.5 mm | 176.5 mm | 0.9 mm |
| min carried z over the carry window | 0.0691 m (dragged) | 0.2398 m | 0.2402 m |
| final z vs slot z (0.150 m) | 0.147 m | 0.147 m | 0.1485 m |

`[measured: app.py bench --scenario single_placement --seeds 0, 2026-09-21; the rows are
appended under the table headers at the end of this file]`. The force budget for `fr3_mpo700`
is 210 N (1.5 x the ~140 N in-motion carry peak); the contact-force budget
(`docs/reference/requirements.md`, 58.6 N) is NOT enforced for this robot (D15's honesty
note and the open option recorded there).

## Measured row, original baseline -- SUPERSEDED (seed 0, magnet gripper, flat surface)

| metric | value |
|---|---|
| cycle time | 67.1 s |
| position error (xy) | 6.6 mm |
| yaw error | 0.15 deg |
| final z vs slot z | 0.147 m vs 0.150 m (2.8 mm) |
| peak `\|F\|` at the flange, sampled at the 500 Hz PHYSICS-tick rate | **411.4 N**, at tick 9632 (t=19.264 s) |
| peak `\|F\|` at the flange, sampled at the 50 Hz `Observation`-refresh rate | 124.2 N (the same event, under-resolved) |
| envelope interventions | 1082 (957 `twist`, 125 `force`, 0 `reach`/`floor`/`joint`) |
| gripper attached at end | `False` (released cleanly) |

`[measured: tests/test_single_placement.py::test_scripted_places_the_unit_on_the_slot`'s own run,
re-measured at physics-tick resolution, 2026-09-21]`. `tests/test_single_placement.py`'s own gate is 2x
the position/yaw row above: `pos_err < 15 mm`, `yaw_err < 1.0 deg`.

**Two peak-force numbers are listed on purpose, not as a rounding note.**
`Runtime.tick()` calls `mujoco.mj_step()` once per physics tick (500 Hz, `dt=0.002 s`), but
only refreshes `self.obs` (and therefore anything read off `Observation.ft_flange_B`, which
is what the original measurement of this row used) once every `decim=10` ticks — the 50 Hz
control rate. Sampling `sensors.ft_flange_B(h)` directly after every physics tick instead
(not just at the decimated `Observation` refresh) finds the TRUE peak at this same event is
**411.4 N**, not 124.2 N — the 50 Hz sampling didn't miss a different spike, it under-resolved
the SAME one by landing on a coarser point of its rise/fall. This has a real consequence
beyond the benchmark number: **the safety envelope's own force clamp (`Envelope.clamp_task`,
`testbed/control/envelope.py`) only ever sees the 50 Hz-refreshed `obs.ft_flange_B` too** —
it is blind to whatever the true 500 Hz transient does between control ticks, by
construction. This is a real, separate finding about the envelope's blind spot, not a defect
introduced or fixed here (the envelope's own tests and design predate it); recording
it here since the SAME event that produced this benchmark's peak force is the evidence for it.
Whether the episode log itself should record force at physics-tick resolution for future
benchmark accuracy was an open question at the time; it is resolved (yes, already, by
construction) in DECISIONS.md D14.

411.4 N exceeds `fr3_mpo700`'s own `force_budget_n` (58.6 N) by ~7x — this is EXPECTED, not a
bug: the envelope's force clamp is a freeze-and-back-off (D12) reacting to the (50 Hz) signal
it can see, not a hard force cap on the true signal, so the sensed force can transiently
exceed budget while the envelope is actively backing the target off; `interventions.force =
125` records exactly how often the 50 Hz signal itself crossed budget. The 957 `twist`
interventions are the base's own `max_vx/vy/wz` cap engaging during `approach`/`transport`'s
stance P-control, expected on every run that drives the base at all.

## Choreography notes (what the numbers above required)

Four real, measured problems stood between the scripted script and this result — none of
them were about the params in `scenarios/single_placement.yaml`'s `script_steps` (which
did not need to change) except the standoff distance and, later, the via stance of item 4:

1. **Self-collision at `standoff_m: 0.9`.** The arm's own forearm (`fr3_link5`/`fr3_link6`)
   physically collided with `base_link` while reaching down to ground level for the
   `acquire` grasp pose — the servo saturated at four-digit N·m torques fighting itself, and
   the flange stalled ~3 cm short of the target and never settled. A standoff sweep
   (0.9/1.0/1.1/1.2/1.3) found 1.0–1.2 clean (zero self-collision depth); `1.1` is now in
   the scenario file for both `approach` (`rcu_0`) and `transport` (`slot_0`).
2. **A magnet-energised-mid-swing grasp tips a 22 kg unit.** Energising the magnet the
   instant a fast approach first swung through the ordinary `EE_TOL_M` (1.5 cm) tolerance
   band captured the unit off-centre (up to 1.34 cm) and while still moving; ground friction
   pinned a corner and the subsequent lift tipped the unit about it, growing to a stalled
   10 deg tilt. Fixed with (a) a tighter, grasp-only tolerance (`GRASP_TOL_M`), (b) requiring
   CONTINUOUS residency inside tolerance for the whole dwell (not a first-crossing check), so
   the magnet only energises once the arm has genuinely stopped, and (c) a rate-limited
   reference (`RAMP_RATE_MPS`, `ACCEL_MAX_MPS2`) so neither the arm's own tool-position
   targets nor the base's twist step instantaneously from zero, which measured a 751 N peak
   flange force (about 21x the unit's 35.6 N lunar weight) at the old step-input transition
   — like the 124.2 N figure this document originally (wrongly) cited as the run's peak
   force (see the corrected 411.4 N row above), this 751 N diagnostic was also read off the
   50 Hz `Observation.ft_flange_B` signal, not the true 500 Hz physics-tick force; it is very
   likely an under-resolved read of a higher true transient, not re-measured here since
   reproducing it means reverting the very fix it justified.
3. **`seat`'s "stopped descending" check compared the wrong time scale.** The original
   tick-to-tick comparison (`abs(z - last_z) < SEAT_STILL_M` every 0.02 s control tick)
   reports "landed" on its very first tick regardless of whether the unit ever touched
   anything, because a genuinely-still-descending unit only moves `SEAT_RATE_MPS * 0.02s =
   0.0004 m` per tick — already below `SEAT_STILL_M` (0.002 m). Measured: `seat` completed
   ~0.4 s after entering, with the unit still 0.28 m up (its `place` hover height); `release`
   then dropped it from there, and toppling on landing produced this file's original
   111.5 mm / 18.5 deg failure. Fixed by comparing TOTAL movement across one whole
   `SEAT_STILL_S` (0.4 s) window instead (expected genuine-descent movement over that window,
   8 mm, is comfortably above `SEAT_STILL_M`).

4. **The base's transit swept its front wheel through the foundation (DECISIONS.md D16).**
   Driving straight from `rcu_0`'s stance to `transport`'s final
   stance (1.1 m behind the slot, x_base = 1.4 m) plows the front-left wheel sphere (reaching
   x_base + 0.735 m) into `found_0`'s near face at x = 2.07 m while the wheel line crosses the
   block's y span, ending 176 mm / 96 mm off blueprint. The scenario now has a via `approach` to
   `slot_0` at `standoff_m: 1.5` (x_base = 1.0 m) before `transport`; measured displacement
   0.6 / 0.9 mm.

See `DECISIONS.md` D13 for the full ledger entry and the self-collision/weld-compliance
measurements in detail.

## Known limitation (measured, not fixed here) -- SUPERSEDED: the diagnosis below was wrong

**Correction (2026-09-21, DECISIONS.md D16):** everything in this
section was measured with a foundation the chassis had already shoved off its blueprint
(`found_0` 110 mm off in x and 188 mm in y at seating time), so the unit rested on five contact
points in two small clusters instead of the designed two-block support. With the foundation
undisturbed the seated unit rests flat (tilt 0.010 deg), does not topple, and stays on the slot
through the full 285 s timeout (position creep 0.076 -> 0.194 mm over 200 idle seconds). The
slot being centred on the seam is harmless. The text below is left as the original record.

The seated unit rests with a small residual tilt (~4 deg) from its own release/settling
event — its target slot is centred exactly on the seam between the two `found_0`/`found_1`
foundation blocks (`scenarios/single_placement.yaml`'s own comment: "makes the
placed-unit disturbance limit measurable"), and the two blocks' rounded (superellipsoid)
top edges meeting at that seam is a genuine centre-of-mass/support-polygon edge case.
Measured: raising the RCU's own torsional/rolling friction 10x (`ws.rcu.friction`) had
ZERO effect on the outcome — this rules out friction as the mechanism (friction resists
sliding/spin, not overturning) and confirms a real geometric instability, not a control or
tuning issue. Left alone at ~4 deg tilt the unit is stable for the ~67 s the script itself
takes, but slowly slides and topples by t≈210 s if the simulation is simply left running
(measured directly, not inferred). Fixing this needs either a worksite-geometry change (a
wider or single-piece foundation, or a slot not centred exactly on the seam) — it touches
`testbed/sim/worksite.py`'s worksite design, not the placement skills — or an explicit
post-placement settle/verify skill this scenario's own script doesn't call for. Recorded
here rather than
silently worked around.

## Hand-variant outcome (`fr3_mpo700_hand`, parallel jaw)

`.venv/Scripts/python.exe` `app.py run --robot fr3_mpo700_hand --scenario single_placement
--controller scripted --headless` — **does not complete**: the script stalls in `acquire`,
on its very first (`pre`, pregrasp-standoff) waypoint, for the entire 285 s timeout.

**Correction (2026-09-21): this is not a clean, self-collision-free reach
limit.** An earlier pass of this document claimed "no self-collision measured at any
sampled tick," checked only over the first ~30-35 s of the stall. Extending the same
stalled run to t=40-100 s (sampled every 1000 physics ticks = 2 s) finds SUSTAINED
self-collision from t=40 s onward: `base_link` vs `fr3_link5` (penetration roughly
-0.01 to -0.5 mm across the window) and, at various points, `fr3_link1`/`fr3_link0` vs
`fr3_link6` — present at every sampled tick from t=40 s through t=100 s, not a transient.
The flange itself settles into a shallow, drooping stall (world z falling from ~0.99 m at
t=36 s to ~0.69 m by t=60-100 s) rather than holding still at the position/orientation the
`pre` waypoint calls for — it is drooping under gravity into a self-collision, not cleanly
parked short of an unreachable target. This is the SAME collision signature
(`base_link`/`fr3_link5`) the magnet path's own `standoff_m` fix (0.9 -> 1.1) resolved —
raising a real, plausible possibility that a standoff adjustment might also help the hand
variant. This was NOT tried: fixing the hand variant's own `acquire` completion was out
of scope here (the parallel jaw is a surrogate this testbed's own docs acknowledge, and
is allowed to fail), and the standoff value that works for the magnet's
top-down grasp has no guarantee of also working for the parallel jaw's very different
(horizontal, near-ground) end-grasp geometry without its own sweep. The honest diagnosis is
therefore: **stalls at a shallow, gravity-drooping approach and DOES self-collide once it
settles into that stall (same signature a standoff fix resolved for the magnet path) —
untried here**, not "a clean workspace limit."

## Recorded rows (`app.py bench`)

Appended by `app.py bench --scenario single_placement --seeds ...` (`testbed/bench/
report.py::append_row`) — never hand-edited; a new row is recorded here before any test
gate is changed against it (`docs/guide/benchmarks.md`, CONTRIBUTING.md house rule 6). The
"Measured row" table above is the hand-written baseline this section continues
from; its numbers match the first row below (peak F re-measured at the corrected,
physics-tick-resolution 411.4 N, not the original under-resolved 124.2 N — see the
"Two peak-force numbers" section above).

| date | commit | seed | surface | success | pos err mm | yaw err deg | cycle s | peak F N | seat F N | found. disp mm | interventions | attach/release |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2026-09-21 | e9c1e5c | 0 | flat | PASS | 6.6 | 0.15 | 67.1 | 411.4 | 44.8 | 217.5 | 1082 | 1/1 |

Columns changed since the table above (see the header below); rows above are unchanged history.

| date | commit | seed | surface | success | pos err mm | yaw err deg | cycle s | peak F N | seat F N | found. disp mm | interventions | force interv | min carried z m | attach/release |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2026-09-21 | e2b550c | 0 | flat | PASS | 7.5 | 0.43 | 85.1 | 411.4 | 163.7 | 176.5 | 2014 | 0 | 0.240 | 1/1 |
| 2026-09-21 | 30d0184 | 0 | flat | PASS | 0.1 | 0.01 | 81.2 | 411.4 | 179.8 | 0.9 | 1048 | 0 | 0.240 | 1/1 |
