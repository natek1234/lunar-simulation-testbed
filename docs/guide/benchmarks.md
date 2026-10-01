# Benchmarks

```bash
python app.py bench --scenario single_placement --seeds 0
```

**One seed is all there is to run today:** nothing in the testbed is stochastic yet, so `--seeds 0 1 2`
would append three identical rows (`bench` prints a one-line notice when given more than one seed). The
seed is reserved for condition sampling that is not implemented yet. `--out FILE` appends to FILE instead of
`docs/benchmarks/<scenario>.md` and `--runs-dir DIR` saves the episodes under DIR instead of `runs/`
(the tests use both so they never modify tracked files).

Runs the scenario's scripted baseline once per seed (`controller=scripted`, always —
`app.py run` already covers keyboard/policy), ticking until the script itself reports
done or `sc.timeout_s` as a hang guard (`tests/test_single_placement.py::run_placement`'s own pattern, not
`Runtime.run(duration_s)`'s fixed-duration mode). Each seed's episode is saved under
`runs/`, metrics are computed from the SAVED episode (`testbed/bench/metrics.py`, never
from live `Runtime` state), printed one line per seed, and one row per seed is appended
to `docs/benchmarks/<scenario>.md` unless `--no-append`.

## Placement metrics
| metric | meaning | requirement |
|---|---|---|
| success | position ≤ 25 mm AND yaw ≤ 6° AND seated on the course (`|z - slot_z| < 20 mm`) | placement position/orientation tolerance |
| pos/yaw/tilt error | final unit pose vs the slot | placement position/orientation tolerance |
| cycle, per-phase time | from the episode's phase column | cycle-time budget |
| peak / seat force | flange force magnitude; seat = mean over the last 0.5 s of `seat` | contact-force budget |
| attach / release confirmed | attachment seen; released and the unit stayed seated | — |
| foundation displacement | max motion of any OTHER (already-placed) unit | placed-unit disturbance limit |
| interventions, stops, helps | envelope intervention count; stop/help actions | envelope enforcement / escalation (help) action |
| force interventions | how many of those were `force`-kind (cumulative, and during the carry window); carrying a payload must not trip the force clamp | contact-force budget |
| min carried z | lowest world z of the placed unit while attached during the carry window (after `acquire` ends, before `seat` begins); a payload being dragged along the ground reads near its rest height (`tests/test_single_placement.py` gates it) | D15 |
| wheel contact fraction | mean fraction of wheels touching ground, over the whole run | — |

See `docs/reference/requirements.md` for each requirement's value and derivation.

## Peak force is measured at physics-tick resolution, not undersampled

`Runtime.tick()` calls `mujoco.mj_step()` once per physics tick (500 Hz, `dt=0.002 s`) but
only refreshes the CONTROLLER-facing `self.obs` once every `decim=10` ticks (the 50 Hz
control rate) — reading `rt.obs.ft_flange_B` directly (as an earlier hand measurement of
this scenario did, see `docs/benchmarks/single_placement.md`) under-resolves a fast
force transient by landing on a coarser point of its rise/fall (124.2 N measured vs the
true 411.4 N at the same event). The EPISODE does not share this limitation:
`Runtime.tick()`'s `self.episode.append(self._observe(), ...)` call computes a fresh
`Observation` after every single `mujoco.mj_step()`, independently of the decimated
`self.obs` the controller sees — confirmed both by `tests/test_runtime.py::
test_episode_records_every_tick` (one row per PHYSICS tick: 50 rows for `rt.run(0.1)` at
`dt=0.002 s`, not 5 for the 50 Hz control rate) and by re-measurement (2026-09-21): reading
`arrays["ft_B"]` back from a saved placement episode reproduces the exact same 411.44 N peak at
the exact same tick (9632) the hand measurement found by sampling every physics tick
directly. `testbed/bench/metrics.py::placement_metrics`'s `peak_force_n` is therefore computed at
the correct resolution already — no undersampling caveat applies to a benchmark number
read from the episode file.

## Record-then-gate
A number is appended to the benchmark file first; a test gate is set against the recorded
row afterwards (`tests/test_single_placement.py`). Never widen a gate to make a run pass — record the
new row and change the gate in the same commit, naming the row.
