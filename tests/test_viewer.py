import pytest

from testbed.sim.viewer import Viewer
from tests.test_runtime import make


@pytest.mark.viewer
def test_viewer_opens_runs_and_closes():
    rt = make()
    res = Viewer(rt).loop(max_s=0.5)
    assert res.ticks == 250
