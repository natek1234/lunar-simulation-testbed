# Teleop (keyboard RC)

```bash
python app.py run --controller keyboard [--robot fr3_mpo700] [--surface flat]
```

| keys | action |
|---|---|
| `W`/`S`, `A`/`D`, `Q`/`E` | base forward/back, left/right, yaw (held; release brakes) |
| `I`/`K`, `J`/`L`, `U`/`O` | end-effector target ±x, ±y, ±z in the base frame (task mode) |
| `[` / `]` | end-effector yaw |
| `M` | toggle task / joint mode |
| `1`–`7`, `=`/`-` | select a joint, jog it (joint mode; the base holds its last command) |
| `SPACE` | toggle gripper open/close |
| `X` / `R` | stop (latched: base brakes, arm holds) / re-arm |
| `H` | help — counts one intervention (`helps` in the run summary and `meta.json`). The runtime can hand control to an attached RC device on a help event, but no shipped path attaches one (`launch.py` never passes `rc=`), so from `app.py` `H` only counts; how control would return to the controller is not designed yet |
| `F` | camera follow on/off |
| `ESC`, window close, Ctrl+C | quit; the episode is saved (Ctrl+C: see the note below) |

**Ctrl+C and the launcher.** `python app.py ...` from a shell whose `python` is not the repo's `.venv`
re-executes itself under the venv interpreter. A terminal Ctrl+C reaches BOTH processes; the viewer (the
child) turns it into its own graceful close and saves the episode, and the launcher shim now installs a
no-op Python-level SIGINT handler while it waits (so `subprocess.run` no longer kills the child a quarter
second after the interrupt, before the save) and relays the child's exit code. That handler contract is
unit-tested (`tests/test_app.py`); no test fires a real signal, and the Windows console path was reasoned
about, not exercised -- if a Ctrl+C on Windows ever loses an episode, use `ESC` or close the window, which
save through the same code path.

Holding is what drives: the device polls physical key state every control tick, so a lost
window focus or a released key brakes within one tick (a deadman, not an event queue).

In joint mode `module_targets=None` on the resolved `JointAction` means "keep the last base
command" (`testbed/sim/runtime.py::Runtime._resolve`) — the base is therefore driven only
in task mode; jogging a joint never also freezes or moves the swerve base underneath it.

## HUD fields
`controller [mode]`, `phase` (running skill), base twist commanded/measured, per-module
steer angle and wheel rate, arm `q` (selected joint starred), end-effector position in B and
W and its commanded target, gripper command/aperture/attachment, flange force magnitude
against the robot's force budget (`OVER` when exceeded), envelope intervention count, RTF.

`testbed/sim/viewer.py` never imports `testbed/viz` directly (`sim` may not import `viz`,
enforced by `tests/test_architecture.py`) — the composition root (`testbed/bench/launch.py`)
injects `testbed/viz/hud_text.py`'s `render` function into `Viewer` as the `hud_render`
callable; a `Viewer` built with no `hud_render` (e.g. the viewer-tier test) still renders
the 3D scene, just without the text overlay.
