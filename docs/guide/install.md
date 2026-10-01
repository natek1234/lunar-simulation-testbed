# Install

## Requirements
- Python 3.11 (any OS). A GPU is optional; physics is CPU-only.
- [`uv`](https://github.com/astral-sh/uv) for the virtual environment.

## Steps

```bash
git clone https://github.com/natek1234/lunar_simulation_testbed.git
cd lunar_simulation_testbed
uv venv .venv --python 3.11
# Windows
uv pip install --python .venv/Scripts/python.exe -e ".[dev]"
# Linux / macOS
uv pip install --python .venv/bin/python -e ".[dev]"
```

Verify:

```bash
.venv/Scripts/python.exe -m pytest -q      # expect: all passed
```

`app.py` re-executes itself under `.venv` automatically, so `python app.py --help`
works from any shell once the venv exists.

## Troubleshooting
- `ModuleNotFoundError: mujoco` — you ran the system Python without the venv, and
  the venv does not exist yet. Create it as above.
- A window fails to open on Linux — install `libglfw3`; headless runs
  (`--headless`) need no display.
