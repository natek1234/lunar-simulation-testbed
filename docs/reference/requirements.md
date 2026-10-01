# Requirements

The numeric requirements this testbed scores runs against, or is meant to eventually score
runs against, each stated in the tool's own words and traced to a published source or to a
derivation given here in full. Every entry gives a short name, the value, where the value
comes from, how (if at all) the testbed checks it, and whether it is enforced today. Code,
tests and YAML that need one of these cite it by short name and point back here.

**Provenance.** Three of the tool's own prior publications supply the domain requirements
(RCU geometry/mass, the electromagnet interface, the cycle-time budget):

- Vasudeva, K., Emami, M. R., & Dickinson, C. (2025). *Lunar construction: A
  state-of-the-art survey.* Progress in Aerospace Sciences.
- Dickinson, C., Shi, F., Vasudeva, K., Mukherjee, R. M., Blanchard, J., Dubrule, S.,
  Empey, J., Kugler, J., Maghoul, P., Ryan, A. J., van Susante, P., & Thangavelautham, J.
  (2026). *A Pathfinder Lunar Construction Mission Concept Using Regolith Filled Bags.*
  Aerospace, 13(3), 223.
- Vasudeva, K., & Emami, M. R. (2026). *Lunar Robotic Construction System Using Raw
  Regolith: System Engineering.* Aerospace, 13(6), 492.

The contact-force budget instead derives from a classical wheeled-vehicle traction figure:

- Costes, N. C., Farmer, J. E., & George, E. B. (1972). *Mobility Performance of the
  Lunar Roving Vehicle: Terrestrial Studies — Apollo 15 Results.* NASA Technical Report
  R-401, Marshall Space Flight Center.

The two placement tolerances come from a reliability argument about overturning in a
stacked, unreinforced structure (below), with the inter-layer overlap minimum taken from:

- EN 1996-1-1 (Eurocode 6), *Design of masonry structures — Part 1-1: General rules for
  reinforced and unreinforced masonry structures.*

| short name | value | enforced today |
|---|---|---|
| [placement position tolerance](#placement-position-tolerance) | σ ≤ 25 mm | yes (as a per-placement bound) |
| [placement orientation tolerance](#placement-orientation-tolerance) | ≤ 6° yaw | yes |
| [cycle-time budget](#cycle-time-budget) | ≤ 190 s (285 s hang guard) | recorded, not gated |
| [placed-unit disturbance limit](#placed-unit-disturbance-limit) | ≤ 10 mm | yes |
| [failure-detection coverage](#failure-detection-coverage) | ≥ 99.9 %, ≤ 5 % false alarms | no |
| [envelope enforcement](#envelope-enforcement) | limits enforced outside the controller | yes |
| [contact-force budget](#contact-force-budget) | 58.6 N | no (magnet robot); see below |
| [escalation (help) action](#escalation-help-action) | available from every phase | yes |
| [RCU geometry and mass](#rcu-geometry-and-mass) | 0.43 × 0.25 × 0.10 m, 22 kg | yes (fixture, not a gate) |

## Placement position tolerance

**Value:** σ ≤ 25 mm — a standard deviation of the xy error between a placed unit's centre
and its target slot, not a hard per-placement bound. The testbed applies it as a hard bound
(see *Verification*).

**Derivation.** Treat the units as rigid and the placement errors of separate units as
independent, each with standard deviation σ. Failure of a stacked, unreinforced course is
taken as **overturning**, not as partial opening of a bed joint: every unit below the top
layer is confined by its neighbours and spans a joint beneath it, so the only free-standing
elements are the units of the topmost course. Such a unit overturns once its offset from
the unit beneath exceeds half the unit's bearing width, i.e. `b/2 = 125 mm` for the RCU's
0.25 m width (see [RCU geometry and mass](#rcu-geometry-and-mass)).

The relative offset of two independently placed units has standard deviation `√2·σ`, so
`σ = 25 mm` puts overturning at `(b/2)/(√2·σ) = 125 / (√2 × 25) ≈ 3.5` standard deviations
— a probability of about `4×10⁻⁴` per joint.

Interior stability is limited instead by the inter-layer overlap, which Eurocode 6 sets at
≥ 40 mm for units of this height, against the nominal 125 mm overlap of a half-unit bond.
At the same tolerance, a small fraction of units fall below that minimum — a loss of bond,
not of stability.

**Verification.** `testbed/bench/metrics.py`'s `placement_metrics` computes `pos_err_m` (final xy
offset from the target slot) from the recorded episode; `judge()` requires
`pos_err_m <= EPS_P_M` (`testbed/bench/metrics.py`, `EPS_P_M = 0.025`). Note the proxy:
the requirement is a **distribution** parameter, while a single run has a single error, so
the testbed checks each placement against the value of σ directly. That is stricter than
the requirement for the typical run and weaker than it for the tail; scoring the
requirement as stated needs the standard deviation over a seed sweep, which
`app.py bench --seeds` records row by row (`docs/guide/benchmarks.md`) but no test yet
aggregates.

**Enforced today:** yes, as the per-placement bound above — `tests/test_single_placement.py` gates the single-placement
scenario well inside it (see the recorded rows in
`docs/benchmarks/single_placement.md`). The sweep-level σ is not gated.

## Placement orientation tolerance

**Value:** ≤ 6° yaw.

**Derivation.** The yaw that keeps a unit's corners inside the position tolerance:
`(L/2)·sinψ ≤ 25 mm` for the RCU's 0.43 m length gives `ψ ≈ 6°`. The orientation
tolerance is therefore not an independent requirement — it is the position tolerance
expressed at the ends of the longest axis.

**Verification.** `placement_metrics` computes `yaw_err_rad` (final yaw offset from the target
slot's yaw) and `judge()` requires `yaw_err_rad <= EPS_YAW_RAD`
(`testbed/bench/metrics.py`, `EPS_YAW_RAD = radians(6.0)`).

**Enforced today:** yes, alongside the position tolerance.

## Cycle-time budget

**Value:** ≤ 190 s per placement.

**Derivation.** The published LRCS mission concept's per-unit placement-cycle estimate
(Vasudeva & Emami, 2026; Dickinson et al., 2026).

**Verification.** `scenarios/single_placement.yaml`'s `timeout_s` is `285.0` — 1.5× the
190 s budget, used as a hang guard (a run that has not finished by then is stalled, not
slow) rather than as the budget itself. `testbed/bench/metrics.py`'s `placement_metrics`'s
`cycle_s` field is computed and recorded on every benchmark row
(`docs/guide/benchmarks.md`).

**Enforced today:** recorded on every row, not gated — no test currently asserts
`cycle_s <= 190`. The measured cycle time (`docs/benchmarks/single_placement.md`,
81.2 s) is comfortably inside the budget, but nothing fails the build if a future change
made it slower.

## Placed-unit disturbance limit

**Value:** ≤ 10 mm (maximum displacement of any already-placed unit while another unit is
placed near it).

**Derivation.** The same stability argument as the placement position tolerance, applied to
units already in the course: an already-seated unit is itself a footing for the units placed
around or above it, so a construction sequence that nudges it must not undo the accuracy it
already achieved. The limit is deliberately tighter than the 25 mm placement tolerance —
placement error is a one-off draw per unit, whereas disturbance accumulates over every
later unit placed nearby, so it is held to a value that many disturbances can sum to
without leaving the placement tolerance.

**Verification.** `placement_metrics.foundation_disp_m` is the max motion of any unit other than
the one being placed, from its first to its last recorded pose in the episode.

**Enforced today:** yes — `tests/test_single_placement.py` gates `foundation_disp_m <= FOUNDATION_TOL_M`
(0.010 m). This is the requirement DECISIONS.md D16 traces a real defect against (the
chassis sweeping a wheel through the foundation during transport); the scenario's via
stance fixes it, measured at 0.9 mm.

## Failure-detection coverage

**Value:** ≥ 99.9 % of gross placement errors detected, ≤ 5 % false-alarm rate.

**Derivation.** An acceptable budget for undetected gross errors accumulated over the
courses placed above a given unit, so that a genuinely bad placement is very unlikely to
go unnoticed before it is built over, while a healthy run is rarely stopped for nothing.

**Verification.** None implemented. Nothing in this testbed currently computes a
detection or false-alarm rate; there is no fault-injection mechanism to measure detection
against.

**Enforced today:** no. This is the one requirement in this table with no code, test, or
metric behind it yet — recorded here so a future implementation has a stated target
instead of an invented one.

## Envelope enforcement

**Value:** safety limits (reach, workspace floor, base twist, joint limits, flange load)
are enforced by a runtime shield the control policy cannot bypass, not by the controller
itself.

**Derivation.** Verifiability: a limit a controller is merely supposed to respect cannot
be trusted for an untested or learned policy; a limit enforced by code the controller
cannot see or influence can be checked once and relied on for every controller.

**Verification.** `testbed/control/envelope.py`'s `Envelope.clamp_task`/`clamp_joint` sit
between every controller's output and the actuators (`testbed/sim/runtime.py`); every
intervention is counted by kind (`twist`, `reach`, `floor`, `force`, `joint`) and recorded
per tick (`Observation.envelope_active`, the episode's `interventions_by_kind` column).

**Enforced today:** yes — `tests/test_envelope.py` exercises every clamp directly and at
the `Runtime` level; DECISIONS.md D12/D15 record two real defects this shield had (a
one-way ratchet that rewrote the standing target, and a budget that tripped on ordinary
carrying) and how they were fixed without weakening the shield itself.

## Contact-force budget

**Value:** 58.6 N.

**Derivation.** A drawbar-pull / vehicle-weight coefficient of 0.21 (Costes, Farmer &
George, 1972 — representative of an LRV-class wheel's tractive performance on lunar
regolith simulant) applied to the rover-scaled base's lunar weight (344.63 kg × 1.62 m/s²
= 558.3 N; 0.21 × 558.3 N = 117.2 N), halved by a safety factor of 2: `117.2 / 2 ≈ 58.6 N`.
This is a **seating contact-force** requirement — how hard the end effector may press an
RCU into place — not a limit on everything the flange sensor reads.

**Why it is not enforced for the magnet robot.** The flange force/torque sensor
(`Observation.ft_flange_B`) reads the SUM of contact force, the carried unit's weight, and
its inertial load — it cannot distinguish them. 58.6 N is smaller than the carried 22 kg
unit's own lunar weight plus ordinary motion transients (DECISIONS.md D15 measures an
in-motion carry peak of ~140 N with nothing wrong), so applying it to the raw flange
reading trips the envelope's force freeze during ordinary carrying, not just during a
hazardous contact — and once tripped, backs the target off into the ground instead of
toward safety (the defect D15 fixes). `robots/fr3_mpo700.yaml` instead sets
`force_budget_n: 210.0`, derived the same way as the hand variant's 125 N
(DECISIONS.md D12): 1.5× the measured in-motion carry peak. This is a flange-LOAD limit,
not the seating contact-force budget above, and the testbed says so directly wherever it
appears (`testbed/spec/robot.py`, `testbed/control/envelope.py`).

**Verification.** `placement_metrics.seat_force_n` (mean flange-force magnitude over the last
0.5 s of the `seat` phase) is recorded on every benchmark row
(`docs/benchmarks/single_placement.md`) as a metric, not checked against 58.6 N.

**Enforced today:** no, for `fr3_mpo700` (magnet) or `fr3_mpo700_hand` (parallel) — both
ship a robot-specific flange-load budget (210 N, 125 N) sized to the carried payload
instead.

**Open option (not implemented):** subtract the carried payload's known weight and
inertial load (mass × (g + measured flange acceleration), read from the gripper's
`attached_id` and flange kinematics) from the flange reading before the envelope sees it,
so the force clamp monitors contact force only and 58.6 N becomes a meaningful limit again
for the magnet robot. Tradeoffs: it needs an acceleration estimate (the sensor runs at
500 Hz, the controller's observation at 50 Hz), it couples the MuJoCo-free `testbed/control`
package to payload-mass knowledge a real flight system may not have, and an error in the
subtraction leaks directly into a safety limit.

## Escalation (help) action

**Value:** an escalation ("help") action must be available from every phase of a run, not
only some of them.

**Derivation.** A meaningful human- or operator-intervention-rate measurement requires
that "ask for help" always be a legal action, regardless of what the controller happens to
be doing — otherwise a low intervention count could mean either a smoothly running system
or one where escalation was simply unavailable at the moment it was needed.

**Verification.** Every `Action` variant (`SkillAction`, `TaskAction`, `JointAction`)
carries its own `help: bool` field (`testbed/control/records.py`); `Runtime` counts rising
edges of `help` regardless of the current phase or action level
(`testbed/sim/runtime.py::Runtime._controller_action`, `help_count`).

**Enforced today:** yes, structurally — because `help` lives on the action type itself
rather than being gated by phase, no phase can omit it.

## RCU geometry and mass

**Value:** 0.43 × 0.25 × 0.10 m, 22 kg nominal.

**Derivation.** Published Lunar Robotic Construction System engineering design (Vasudeva &
Emami, 2026); the Regolith Containment Unit itself is described further in Dickinson et
al. (2026).

**Verification.** `scenarios/single_placement.yaml`'s `worksite.rcu.extents_m` /
`mass_kg`; the RCU body is authored from these values in `testbed/sim/worksite.py`.

**Enforced today:** yes, as a fixture value every scenario builds its RCU from — not a
pass/fail gate in itself, but every other placement metric is computed against a unit of
exactly this size and mass.
