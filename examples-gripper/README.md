# examples-gripper — the lab gripper (reference, experimental)

A two-finger gripper for the UR5e in the lab: one Lynxmotion smart servo in a
printed four-bar linkage, controlled by a small ESP32 board (LilyGO T-CAN485)
inside the gripper. The board gets 12 V and its commands over the robot's
**tool connector** — the host talks to it through the robot (`<robot>:54321`),
so nothing extra is plugged into your laptop. The hardware only exists on the
lab arm; in the simulator the examples use a **mock gripper** that just prints
what it would do, so you can develop your sequence offline.

Status: experimental reference, not part of the validated framework. Motion
still goes through the normal `Robot` API and the taught workspace.

| file | what |
| --- | --- |
| `gripper_two_poses.py` | the demo: two poses from the workspace, close / open at each with a pause |
| `gripper_test.py` | open / close / 50 % without moving the arm (link check) |
| `gripper_calibrate.py` | instructor: teach the closed / open servo angles |
| `gripper_link.py` | the `Gripper` / `MockGripper` classes the scripts import |

## Use it in your own script

```python
from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs
from gripper_link import make_gripper          # put your script next to gripper_link.py,
                                               # or copy that file into your folder
args = argument_parser(__doc__).parse_args()
ws = Workspace.load(args.workspace)
kind = "tci" if args.target == "real" else "mock"
with Robot(**robot_kwargs(args), profile=ws) as robot, make_gripper(kind, robot.host) as gripper:
    gripper.open()
    robot.move_to(ws.pose(0.5, 0.5, 0.2))
    gripper.close(force_mA=300)                # stops early when it grips something (with feedback)
    robot.move_to(ws.pose(0.5, 0.5, 0.6))
    gripper.goto(60)                           # 0 = closed .. 100 = open (calibrated stroke)
```

`gripper.open() / close() / goto(pct)` block until the jaws arrive;
`gripper.status()` returns a dict (`pct`, `mA`, …).

## Running

```
python examples-gripper/gripper_two_poses.py                       # URSim, mock gripper
python examples-gripper/gripper_two_poses.py --target real \
    --payload 0.36 --tcp 0 0 0.136 0 0 0                            # lab, with the instructor
```

`--payload` / `--tcp` are the gripper's values (jaw centre 136 mm out from the
flange). Edit `POSE_A` / `POSE_B` at the top of the script to move the two
poses; they are fractions of the taught workspace like everywhere else.

## Relative mode (what the lab demo currently uses)

Until the gripper's position feedback line is wired, the servo cannot report
where it is, so the jaws are driven *relative* to where they are: after
powering up (the servo is limp) the instructor sets the jaws mid-stroke by
hand, then `--relative 10` makes every "close" +10° from there and every
"open" −10° back. Nothing is sent before the first pose, so the hand-set
position is the reference for the whole run. Don't touch the jaws between
runs without re-centring them.
