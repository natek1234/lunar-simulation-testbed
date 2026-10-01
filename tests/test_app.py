import pytest

import app


def test_physics_mode_runs(capsys):
    assert app.main(["physics"]) == 0
    assert "total mass" in capsys.readouterr().out


def test_test_mode_is_intercepted_before_argparse(monkeypatch):
    calls = []
    monkeypatch.setattr(app.subprocess, "run", lambda a, **k: (calls.append(a), type("R", (), {"returncode": 0}))[1])
    assert app.main(["test", "-k", "nothing"]) == 0
    assert calls[0][-2:] == ["-k", "nothing"]


def test_reexec_shim_neutralises_sigint_only_while_the_child_runs_and_relays_its_exit_code(monkeypatch, tmp_path):
    """B54-style shim contract, checked WITHOUT firing a real signal: a terminal Ctrl+C reaches both the
    shim and the child, and `subprocess.run` would kill the child ~0.25 s after the shim's own
    KeyboardInterrupt (before the episode is saved). So, while waiting: a Python-level no-op handler
    (NOT SIG_IGN -- inherited across exec, the child would never see Ctrl+C); afterwards: the old handler
    restored; the child's exit code relayed (POSIX signal death -N -> 128+N)."""
    import signal
    import sys

    fake_py = tmp_path / "py"
    fake_py.write_text("", encoding="utf-8")
    monkeypatch.setattr(app, "_venv_python", lambda: fake_py)          # exists, and is not sys.executable -> a re-exec happens
    seen = {}

    def fake_run(argv, **kw):
        h = signal.getsignal(signal.SIGINT)
        seen.update(handler=h, argv=argv)
        h(signal.SIGINT, None)                                         # a Ctrl+C landing mid-wait must be swallowed, not raise
        return type("R", (), {"returncode": seen.get("rc", 0)})()

    monkeypatch.setattr(app.subprocess, "run", fake_run)
    before = signal.getsignal(signal.SIGINT)
    for child_rc, expected in ((0, 0), (3, 3), (-2, 130)):
        seen["rc"] = child_rc
        assert app._reexec() == expected
        assert callable(seen["handler"]) and seen["handler"] not in (signal.SIG_IGN, signal.SIG_DFL, signal.default_int_handler)
        assert signal.getsignal(signal.SIGINT) is before               # restored after the wait
    assert seen["argv"][0] == str(fake_py) and seen["argv"][1:] == sys.argv


def test_reexec_does_nothing_when_already_running_under_the_repo_venv(monkeypatch):
    import sys
    from pathlib import Path
    monkeypatch.setattr(app, "_venv_python", lambda: Path(sys.executable))
    monkeypatch.setattr(app.subprocess, "run", lambda *a, **k: pytest.fail("must not re-exec"))
    assert app._reexec() is None
