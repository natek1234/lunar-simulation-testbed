#!/usr/bin/env python3
"""Launcher and composition root. Modes: physics | test | run | bench.
Re-executes itself under the repo's .venv so `python app.py` works from a bare shell."""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent


def _venv_python() -> Path:
    return REPO / ".venv" / ("Scripts/python.exe" if os.name == "nt" else "bin/python")


def _relay_code(returncode: int) -> int:
    """The child's exit code as this process should return it: a POSIX signal death (`-N`) is
    conventionally `128 + N`; anything else passes through."""
    return returncode if returncode >= 0 else 128 - returncode


def _reexec() -> int | None:
    """Re-run under the repo's .venv interpreter. A terminal Ctrl+C reaches BOTH this shim and
    the child, and `subprocess.run`'s own interrupt path gives the child ~0.25 s and then kills it
    -- long before `run_mode` has saved a (33k-row) episode. So the shim installs a PYTHON-LEVEL
    no-op SIGINT handler for the duration of the wait (NOT `SIG_IGN`: an ignored disposition is
    inherited across exec and the child's Python would then never see Ctrl+C at all; a Python
    handler resets to default on exec), restores the old one afterwards, and relays the child's
    exit code. The CHILD owns graceful shutdown (`Viewer`'s SIGINT handler -> close -> save)."""
    py = _venv_python()
    if not py.is_file() or Path(sys.executable).resolve() == py.resolve():
        return None
    previous = signal.signal(signal.SIGINT, lambda *_: None)
    try:
        return _relay_code(subprocess.run([str(py), *sys.argv]).returncode)
    finally:
        signal.signal(signal.SIGINT, previous)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="app.py", description="lunar simulation testbed launcher")
    sub = p.add_subparsers(dest="mode", required=True)
    ph = sub.add_parser("physics", help="print the compiled model's masses, reach, gravity, handles")
    ph.add_argument("--robot", default="fr3_mpo700", help="robots/<name>.yaml")
    ph.add_argument("--surface", default="flat", help="surfaces/<name>.yaml")
    sub.add_parser("test", help="pytest passthrough: app.py test [pytest args]")

    run = sub.add_parser("run", help="run a robot on a surface under a controller")
    run.add_argument("--robot", default="fr3_mpo700")
    run.add_argument("--surface", default="flat")
    run.add_argument("--scenario", default=None, help="scenarios/<name>.yaml (worksite + script); optional")
    run.add_argument("--controller", default="keyboard", choices=("keyboard", "scripted", "policy"))
    run.add_argument("--policy", default=None, help="with --controller policy: module.path:callable (not implemented yet; exits 2)")
    run.add_argument("--seed", type=int, default=0)
    run.add_argument("--duration-s", type=float, default=None, help="stop after this much sim time")
    run.add_argument("--headless", action="store_true", help="no window; runs as fast as the CPU allows")
    run.add_argument("--run-dir", default=None, help="where the episode lands; default runs/<robot>_<scenario>_s<seed>_<utc>")

    bench = sub.add_parser("bench", help="run a scenario's scripted baseline, compute placement metrics, append a benchmark row")
    bench.add_argument("--scenario", default="single_placement", help="scenarios/<name>.yaml")
    bench.add_argument("--robot", default="fr3_mpo700")
    bench.add_argument("--surface", default="flat")
    bench.add_argument("--seeds", type=int, nargs="+", default=[0])
    bench.add_argument("--no-append", action="store_true", help="print metrics but don't append to docs/benchmarks/<scenario>.md")
    bench.add_argument("--out", default=None, help="append rows to this markdown file instead of docs/benchmarks/<scenario>.md")
    bench.add_argument("--runs-dir", default=None, help="save episodes under this directory instead of runs/")
    return p


def main(argv: list[str] | None = None) -> int:
    rc = _reexec()
    if rc is not None:
        return rc
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == "test":
        return subprocess.run([sys.executable, "-m", "pytest", *argv[1:]], cwd=REPO).returncode
    args = build_parser().parse_args(argv)
    if args.mode == "physics":
        from testbed.sim.report import physics_report
        return physics_report(args.robot, args.surface)
    if args.mode == "run":
        from testbed.bench.launch import run_mode
        return run_mode(args)
    if args.mode == "bench":
        from testbed.bench.launch import bench_mode
        return bench_mode(args)
    return 2


if __name__ == "__main__":
    sys.exit(main())
