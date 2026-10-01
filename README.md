# Lunar Simulation Testbed

A physics-based (MuJoCo) testbed for autonomous lunar construction: a mobile
manipulator placing Regolith Containment Units (RCUs — regolith-filled bags, analogous
to sandbags) into a structure. It exists to produce the measurements needed to develop
and evaluate autonomous RCU placement: robots are described by YAML files and swapped
without code changes; controllers — a human at the keyboard, a scripted skill sequence,
or (in the future) a learned policy — plug into one seam at three action levels;
scenarios and surfaces are configuration; benchmarks are scored from recorded episodes.

Design: `docs/design.md`. Requirements this tool scores against, and where each one comes
from: `docs/reference/requirements.md`.

## Quickstart (5 minutes)

```bash
uv venv .venv --python 3.11
uv pip install --python .venv/Scripts/python.exe -e ".[dev]"   # Linux: .venv/bin/python
python app.py run --controller keyboard                          # drive it: W/A/S/D, U/O lifts the arm, ESC quits
python app.py run --controller scripted --headless --duration-s 8  # scripted demo, no window, episode saved under runs/
python app.py bench --scenario single_placement --seeds 0        # scripted single-placement baseline, metrics appended to docs/benchmarks/
python app.py physics                                            # what got compiled
python app.py test                                               # the default test tier
```

Guides: `docs/guide/install.md`, `docs/guide/running.md`, `docs/guide/teleop.md`,
`docs/guide/benchmarks.md`, `docs/guide/episodes_and_replay.md`.

## Repository map

- `testbed/spec` — robot / surface / scenario specs (pure data)
- `testbed/control` — MuJoCo-free control: records, IK, swerve, envelope, controllers
- `testbed/sim` — the only MuJoCo importer: composition, sensors, runtime, viewer
- `testbed/bench` — episodes, metrics, benchmark reports, launcher glue (`app.py run`/`bench`)
- `testbed/viz` — HUD rendering from snapshots
- `robots/`, `surfaces/`, `scenarios/` — the YAML that describes a run
- `assets/` — vendored models (see `assets/PROVENANCE.md`)
- `docs/` — `design.md` (architecture and rationale), `guide/` (task-oriented how-tos),
  `reference/` (requirements and record field tables), `benchmarks/` (recorded runs)
- `DECISIONS.md` — the decision ledger: why the code is shaped the way it is

## Status

**Scope today:** a flat, rigid surface; object poses read from ground truth (no rendered
perception); scripted control only (no learned policy yet); a magnetic pick-and-place
interface modelled as a rigid weld constraint, not a real electromagnet field. Within that
scope, the scripted single-placement baseline places its RCU at 0.08 mm / 0.009° error in
81.2 s, with the foundation it's placed beside disturbed by 0.9 mm
(`docs/benchmarks/single_placement.md`; see `DECISIONS.md` D15/D16 for how it got
there). Read that number as "control and physics introduce essentially no error in the
absence of disturbances," not as demonstrated real-world placement capability — nothing
here yet stresses perception, terrain, or off-nominal conditions. The 58.6 N contact-force
budget (`docs/reference/requirements.md`) is not enforced for the default magnet robot;
see that entry and `DECISIONS.md` D15 for why and what a fix would need.

The interactive keyboard control path (`app.py run --controller keyboard`) is exercised by
an automated test that opens a real window and drives the loop for a fixed duration — that
proves the window opens and the loop runs, not that a specific key produces the specific
behavior documented in `docs/guide/teleop.md`. It has not been verified by a human at a
real desktop pressing keys and confirming the robot's response; treat it as probably
correct, not as verified.

## Future work

- **Terrain relief and suspension** — only a flat, rigid-chassis surface exists today.
- **Condition/disturbance sampling** — surface, mass, and sensing variation from a
  declared distribution, exercising the `--seed` argument that is currently inert.
- **Rendered RGB-D and sensor degradation**, in place of the oracle object poses every
  controller currently sees.
- **Contact-force isolation**, so the published 58.6 N seating-contact budget can be
  enforced for the magnet robot instead of recorded as an unenforced metric
  (`docs/reference/requirements.md`).
- **A learned policy substituted in at the skill level**, exercising the control seam's
  third action level end to end.
- **Additional robot descriptors** — a second arm or base kind, to exercise the
  swap-without-code-changes claim beyond the two gripper kinds it's exercised with today.

## Acknowledgements

Vendored assets and their licenses (full provenance and sha256 ledger:
`assets/PROVENANCE.md`):

- Neobotix MPO-700 MJCF and meshes, via `FunanShi/swerve-nav` (MIT)
- Franka FR3 arm and Franka hand, from `google-deepmind/mujoco_menagerie` (Apache-2.0)
- Lunar regolith diffuse textures, Poly Haven (CC0)

This testbed's architecture and its `docs/design.md`-documented measured MuJoCo findings
were informed by an earlier MuJoCo build of this same platform,
`FunanShi/lunar_robot_mujoco` — a courtesy acknowledgement for that influence; no code was
copied from it (`DECISIONS.md` D1).

## License

MIT for this repository's code; vendored assets keep their own licenses.
