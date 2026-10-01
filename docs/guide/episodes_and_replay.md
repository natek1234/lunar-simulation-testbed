# Episodes

Every run writes `runs/<name>/episode.npz` and `meta.json` (`testbed/bench/episode.py`,
schema 3). **One row per PHYSICS tick** (2 ms, 500 Hz) — not per control tick (50 Hz):
`Runtime.tick()` calls `self.episode.append(self._observe(), ...)` unconditionally, once
per call to `tick()`, with a freshly computed `Observation` taken right after that tick's
`mujoco.mj_step()` — this is independent of the coarser `decim`-gated refresh of the
CONTROLLER-facing `self.obs` (`tests/test_runtime.py::test_episode_records_every_tick`
pins this: 50 rows for `rt.run(0.1)` at `dt=0.002 s`). See `docs/guide/benchmarks.md`'s
"Peak force is measured at physics-tick resolution" section for why this distinction
matters for anything computed from a force column.

Columns: `t_s`, `tick`, `base_pos_W`, `base_quat_W`, `base_twist_B`, `q_rad`, `qd_radps`,
`tau_nm`, `steer_rad`, `wheel_radps`, `gripper_aperture_m`, `attached`, `ft_B`, `ee_pos_W`,
`ee_quat_W`, `ee_pos_B`, `cmd_twist_B`, `cmd_q_rad`, `cmd_modules`, `cmd_gripper`, `level`,
`envelope_active`, `interventions`, `interventions_by_kind` (cumulative count per envelope kind, in `meta.intervention_kinds` order: twist/reach/floor/force/joint), `phase` (index into `meta.phases`), and, when a
`Worksite` is attached: `unit_pos_W`/`unit_quat_W` (one pose per staged RCU, in
`meta.unit_ids` order) and `wheel_contact` (per-module bool, from
`testbed/sim/sensors.py::wheel_contacts`). `meta.json` carries robot, surface, scenario,
seed, git commit, control rate, `stops`/`helps` counts, and (with a worksite) `unit_ids`,
`slots` (`[{"id", "xyz_W_m", "yaw_rad"}, ...]`), `to_place`. Load with
`testbed/bench/episode.py::load_episode`.

`testbed/bench/metrics.py::placement_metrics` computes the placement benchmark numbers
(`docs/guide/benchmarks.md`) purely from these two objects — never from a live `Runtime` —
so a saved episode is a complete, reproducible record of one run.

Replay/verify is not implemented yet.
