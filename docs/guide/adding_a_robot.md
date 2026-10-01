# Adding a robot

A robot is one YAML file under `robots/`. `robots/fr3_mpo700.yaml` is the reference;
every field is documented in `testbed/spec/robot.py`.

## The descriptor

| block | what it names | when you need code |
|---|---|---|
| `base` | the chassis MJCF (or an authored box), each wheel module's joints and actuator names, limits, gains | only for a new `kind` (`swerve`, `skid`, `floating` exist) |
| `arm` | the arm MJCF, its joints/actuators, home pose, flange site, mount pose on the base | never — kinematics come from MuJoCo |
| `gripper` | the gripper MJCF, its actuator and open/close commands, the interface site | only for a new `kind` (`parallel`, `magnet`) |
| `sensors` | the F/T site, cameras, tag read range | never |

Names are resolved against the compiled model when the robot loads; a wrong name fails
immediately with the list of candidates.

## Checklist for a new robot
1. Copy `robots/fr3_mpo700.yaml`, rename, point `chassis_mjcf` / `arm.mjcf` /
   `gripper.mjcf` at vendored files (add them to `assets/PROVENANCE.md`).
2. Fill the joint/actuator names from the MJCF files.
3. Run `python app.py physics --robot <name>` — it prints masses, reach and the
   resolved handles, and fails on any missing name.
4. Run `python app.py run --robot <name> --controller keyboard` and drive it.
