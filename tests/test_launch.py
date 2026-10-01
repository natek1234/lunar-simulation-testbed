"""The composition layer `app.py run` / `app.py bench` sit on (`testbed/bench/launch.py`): it shipped one
latent AttributeError (`Worksite(sc)`) that only a human running the CLI found, so drive it end to end.
Every test writes under pytest's tmp dir -- never `docs/benchmarks/` or the repo's `runs/`."""
from types import SimpleNamespace

import numpy as np
import pytest

import app
from testbed.bench import launch as L
from testbed.bench.episode import load_episode
from tests.conftest import REPO


def test_run_scripted_headless_demo_saves_an_episode_and_exits_zero(tmp_path, capsys):
    run_dir = tmp_path / "demo"
    rc = app.main(["run", "--controller", "scripted", "--headless", "--duration-s", "1", "--run-dir", str(run_dir)])
    assert rc == 0
    arrays, meta = load_episode(run_dir)
    assert arrays["t_s"].shape == (500,) and meta["ticks"] == 500 and meta["controller"] == "scripted"
    assert meta["mode"] == "headless" and meta["scenario"] is None
    assert "run saved:" in capsys.readouterr().out


def test_run_with_a_scenario_stages_the_worksite_end_to_end(tmp_path):
    run_dir = tmp_path / "placement"
    assert app.main(["run", "--controller", "scripted", "--scenario", "single_placement", "--headless",
                     "--duration-s", "2", "--run-dir", str(run_dir)]) == 0
    arrays, meta = load_episode(run_dir)
    assert meta["scenario"] == "single_placement" and meta["unit_ids"] == ["rcu_0", "found_0", "found_1"]
    assert arrays["unit_pos_W"].shape == (1000, 3, 3)          # one row per physics tick, one pose per staged unit
    assert np.allclose(arrays["unit_pos_W"][0, 0], [3.0, 0.0, 0.05], atol=0.01)


def test_run_policy_controller_fails_cleanly_with_exit_two(tmp_path, capsys):
    """`--controller policy` is an advertised argparse choice, but `PolicyAdapter` is not implemented yet: one line, exit 2, no traceback."""
    assert app.main(["run", "--controller", "policy", "--headless", "--run-dir", str(tmp_path / "x")]) == 2
    captured = capsys.readouterr()
    assert "not implemented yet" in captured.err and len(captured.err.strip().splitlines()) == 1
    assert not (tmp_path / "x").exists()                       # nothing was built or saved


def test_build_resolves_the_scenario_from_the_repo_not_the_cwd(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)                                # no `scenarios/` here
    rt = L.build(SimpleNamespace(robot="fr3_mpo700", surface="flat", scenario="single_placement",
                                 controller="scripted", policy=None, seed=0))
    assert rt.worksite is not None and rt.controller.name == "scripted"


def test_bench_seed_notice_only_for_more_than_one_seed():
    assert "affects nothing" in L.seed_notice(3) and "3 seeds" in L.seed_notice(3)


def test_bench_end_to_end_appends_a_row_to_the_requested_file_only(tmp_path, capsys):
    tracked = REPO / "docs" / "benchmarks" / "single_placement.md"
    before = tracked.read_bytes()
    out = tmp_path / "bench.md"
    assert app.main(["bench", "--scenario", "single_placement", "--seeds", "0", "--out", str(out),
                     "--runs-dir", str(tmp_path / "runs")]) == 0
    text = out.read_text(encoding="utf-8")
    assert text.count("| date |") == 1 and "| PASS |" in text.splitlines()[-1]
    printed = capsys.readouterr().out
    assert "seed=0 success=True" in printed and "affects nothing" not in printed
    assert len(list((tmp_path / "runs").glob("*/episode.npz"))) == 1
    assert tracked.read_bytes() == before                      # the tracked benchmark doc is untouched


def test_bench_with_several_seeds_prints_the_inert_seed_notice(tmp_path, capsys, monkeypatch):
    """Two seeds must announce that the rows will be identical. `build` is stubbed to stop the run right after the
    notice, so the test doesn't simulate the placement twice."""
    def stop(args):
        raise RuntimeError("stop after the notice")

    monkeypatch.setattr(L, "build", stop)
    with pytest.raises(RuntimeError, match="stop after the notice"):
        L.bench_mode(SimpleNamespace(scenario="single_placement", robot="fr3_mpo700", surface="flat", seeds=[0, 1],
                                     no_append=True, out=None, runs_dir=str(tmp_path)))
    assert "affects nothing" in capsys.readouterr().out
