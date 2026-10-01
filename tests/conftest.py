"""Session-wide fixtures. The default tier is headless by construction: DISPLAY is
scrubbed unless a viewer-marked test was selected."""
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[1]


def pytest_collection_modifyitems(config, items):
    if not any("viewer" in item.keywords for item in items):
        os.environ.pop("DISPLAY", None)


@pytest.fixture(scope="session")
def repo() -> Path:
    return REPO


@pytest.fixture(scope="session")
def placement_run(tmp_path_factory):
    """ONE recorded scripted single-placement episode (magnet robot, flat, seed 0) shared by every test that
    needs one -- a run is ~10 s of wall time, and `tests/test_single_placement.py` and `tests/test_metrics.py`
    both gate on the same episode (saved under a pytest tmp dir, never under the tracked
    `docs/benchmarks/`)."""
    from testbed.bench.episode import load_episode
    from tests.test_single_placement import run_placement
    rt, ws, sc = run_placement(record=True)
    arrays, meta = load_episode(rt.save(tmp_path_factory.mktemp("placement") / "run"))
    return SimpleNamespace(rt=rt, ws=ws, sc=sc, arrays=arrays, meta=meta)
