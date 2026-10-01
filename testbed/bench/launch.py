"""Compose robot + surface (+ scenario), pick the controller, run headless or in the
viewer -- the glue `app.py run` calls into.

Lives in `testbed/bench/`, not `testbed/sim/`: `tests/test_architecture.py::
test_package_import_boundaries` forbids `sim` from importing `bench` (a one-directional
boundary -- `bench` may import `sim`, never the reverse, per `docs/design.md`'s own
statement that `bench` imports `spec`, `control`, and `sim`), and this module needs
`EpisodeLog` from `testbed.bench.episode` to hand a fresh one to every `Runtime`. `bench`
is exactly the layer above `sim` this composition belongs in."""
import datetime as _dt
import math
import sys
from pathlib import Path
from types import SimpleNamespace

from testbed.bench.episode import EpisodeLog, load_episode
from testbed.control.controller import ScriptedSequencer
from testbed.sim import compose as C, handles as H, runtime as RT
from testbed.spec import robot as R, surface as S

REPO = Path(__file__).resolve().parents[2]
# [policy default: built-in scripted demo for `app.py run --controller scripted`, no scenario needed]
DEMO_SCRIPT = (("hold", {"duration_s": 0.5}), ("drive", {"vx_mps": 0.3, "duration_s": 3.0}),
               ("jog", {"dpos_B_m": [0.0, 0.0, 0.15], "duration_s": 2.0}), ("drive", {"wz_radps": 0.3, "duration_s": 2.0}))


def seed_notice(n: int) -> str:
    return (f"note: --seed currently affects nothing (no part of the testbed is stochastic yet; the seed is reserved "
            f"for condition sampling that is not implemented yet), so the rows for these {n} seeds will be identical")


def policy_unavailable() -> str:
    return "app.py run: --controller policy is not available yet (policy controllers are not implemented yet)"


def default_run_dir(robot: str, scenario: str | None, seed: int, root: Path | None = None) -> Path:
    stamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path(root if root is not None else REPO / "runs") / f"{robot}_{scenario or 'demo'}_s{seed}_{stamp}"


def build(args):
    robot = R.load_robot(R.ROBOTS_DIR / f"{args.robot}.yaml")
    surface = S.load_surface(S.SURFACES_DIR / f"{args.surface}.yaml")
    worksite, script = None, DEMO_SCRIPT
    if args.scenario:
        from testbed.spec.scenario import SCENARIOS_DIR, load_scenario
        from testbed.sim.worksite import Worksite
        # Repo-anchored, like `bench_mode`: a CWD-relative "scenarios/" only worked from the repo root.
        sc = load_scenario(Path(args.scenario) if Path(args.scenario).is_file() else SCENARIOS_DIR / f"{args.scenario}.yaml")
        # `Worksite(sc)` (not `Worksite(sc.worksite)`) was a pre-existing bug in this line --
        # untested until `app.py run --scenario ...` needed it for real. `Worksite.__init__`
        # does NOT duck-type past construction: it immediately does
        # `self.filled = [False] * len(spec.slots)`, and a bare `ScenarioSpec` has no `.slots`
        # (only `ScenarioSpec.worksite.slots` does) -- AttributeError('ScenarioSpec' object has
        # no attribute 'slots') at construction, not at `reset()` [measured: directly,
        # `Worksite(sc)` on this scenario, 2026-09-21]. No prior test exercised
        # `testbed.bench.launch` with a scenario.
        worksite, script = Worksite(sc.worksite), sc.script
    h = H.resolve(C.compose(robot, surface, None if worksite is None else worksite.spec))
    if args.controller == "scripted":
        controller = ScriptedSequencer(script)
    elif args.controller == "keyboard":
        from testbed.control.keyboard_rc import KeyboardRc
        controller = KeyboardRc(robot)
    else:
        raise NotImplementedError(policy_unavailable())             # PolicyAdapter is not implemented yet
    rt = RT.Runtime(h, controller, seed=args.seed, episode=EpisodeLog(robot), worksite=worksite)
    return rt


def bench_mode(args) -> int:
    """Run a scenario's scripted baseline once per seed, save each episode under `runs/`,
    compute `testbed.bench.metrics.placement_metrics` from the SAVED episode (never from live
    `Runtime` state -- the whole point of a benchmark row is that it is reproducible from
    what got written to disk), print one line per seed, and append a row to
    `docs/benchmarks/<scenario>.md` unless `--no-append`. `--controller` is deliberately
    not exposed here (always `scripted`): a benchmark row is a baseline number, not an
    arbitrary run -- `app.py run` already covers keyboard/policy. `--out` redirects the appended markdown and
    `--runs-dir` the saved episodes (tests use both so they never touch tracked files)."""
    from testbed.bench import metrics as MET, report as REP
    from testbed.spec.scenario import SCENARIOS_DIR, load_scenario

    sc = load_scenario(SCENARIOS_DIR / f"{args.scenario}.yaml")
    md_path = Path(args.out) if getattr(args, "out", None) else REPO / "docs" / "benchmarks" / f"{args.scenario}.md"
    runs_dir = getattr(args, "runs_dir", None)
    if len(args.seeds) > 1:
        print(seed_notice(len(args.seeds)))
    for seed in args.seeds:
        run_args = SimpleNamespace(robot=args.robot, surface=args.surface, scenario=args.scenario,
                                    controller="scripted", policy=None, seed=seed)
        rt = build(run_args)
        rt.reset()
        while not rt.controller.done(rt.obs) and rt.t_s < sc.timeout_s:
            rt.tick()
        run_dir = default_run_dir(args.robot, args.scenario, seed, runs_dir)
        rt.save(run_dir, {"scenario": args.scenario, "mode": "bench"})
        arrays, meta = load_episode(run_dir)
        m = MET.placement_metrics(arrays, meta)
        print(f"seed={seed} success={m.success} pos_err_mm={m.pos_err_m*1000:.1f} "
              f"yaw_err_deg={math.degrees(m.yaw_err_rad):.2f} cycle_s={m.cycle_s:.1f} "
              f"peak_F_N={m.peak_force_n:.1f} interventions={m.interventions} run_dir={run_dir}")
        if not args.no_append:
            REP.append_row(md_path, args.scenario, m, meta)
    return 0


def run_mode(args) -> int:
    if args.controller == "policy":
        print(policy_unavailable(), file=sys.stderr)
        return 2
    rt = build(args)
    rt.reset()
    run_dir = Path(args.run_dir) if args.run_dir else default_run_dir(args.robot, args.scenario, args.seed)
    if args.headless:
        res = rt.run(args.duration_s)      # None -> Runtime.run() drives to controller.done()
    else:
        from testbed.sim.viewer import Viewer
        from testbed.viz.hud_text import render as hud_render      # sim may not import viz directly
        res = Viewer(rt, hud_render=hud_render).loop(max_s=args.duration_s)
    rt.save(run_dir, {"scenario": args.scenario, "mode": "headless" if args.headless else "viewer"})
    print(f"run saved: {run_dir}  ticks={res.ticks} stopped={res.stopped} help={res.help_count} interventions={res.interventions}")
    return 0
