# Running

All modes: `python app.py --help`. Each mode has `--help`.

## `physics` — inspect a composed robot

```bash
python app.py physics --robot fr3_mpo700 --surface flat
```

Prints body/joint/actuator counts, timestep and gravity as compiled, total mass, the
flange position at q = 0, and the force budget. Fails with the candidate list if any name
in the robot YAML is missing from the compiled model.

`bench` runs a scenario's scripted baseline and computes/records placement metrics —
`docs/guide/benchmarks.md`; `python app.py test [pytest args]` is a thin passthrough to
`pytest` under the repo's own `.venv` (see `app.py`).

## `run` — drive a robot

```bash
python app.py run --controller scripted --headless --duration-s 8      # built-in demo script, no window
python app.py run --controller keyboard                                 # viewer + keyboard, see docs/guide/teleop.md
python app.py run --scenario single_placement --controller scripted  # the scripted placement episode; see also `app.py bench`
```

`--controller keyboard` (the default) opens a real GLFW window: MuJoCo's own render loop
(paced to wall time) with an `mjr_overlay` text HUD on top; `--headless` skips the window
entirely and is the only mode that "runs as fast as the CPU allows" rather than at RTF ~1.
Key bindings, HUD fields, and the task/joint mode split are documented in
`docs/guide/teleop.md`.

Every run writes `runs/<robot>_<scenario>_s<seed>_<utc>/episode.npz` + `meta.json`
(`testbed/bench/episode.py`; schema 3: one row per PHYSICS tick, not control tick --
pose, twist, joint state, commanded action, gripper, and the phase name; full column list
in `docs/guide/episodes_and_replay.md`). `--duration-s` bounds sim time; if
omitted, a headless run continues until the controller itself reports done (e.g. a
scripted script running out of skills). **`--seed` currently affects nothing:** no part of the
testbed is stochastic yet (`Runtime` builds a seeded `rng` and hands it to `Worksite.reset`, which never
draws from it), so every seed yields the same episode. Runs ARE fully deterministic -- the same
command twice gives a bit-identical episode (`tests/test_runtime.py::test_same_seed_is_bit_identical`,
which proves determinism, not seed sensitivity) -- and the seed is reserved for condition
sampling that is not implemented yet (object placement noise, friction draws, ...). Passing several seeds to `app.py bench` therefore
just repeats one row; `bench` says so.
