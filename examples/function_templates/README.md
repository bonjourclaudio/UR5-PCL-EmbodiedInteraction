# function_templates/ — one runnable example per API call

Short scripts (≈ 30 lines each) that show one thing. Run them in order once,
then copy whatever you need into `students/<you>/`.

Every template takes the same flags (`--help` lists them):

```
python examples/function_templates/03_move_to.py                  simulator (default)
python examples/function_templates/03_move_to.py --target real    the lab arm
python examples/function_templates/03_move_to.py --target real --payload 0.35 --tcp 0 0.021 0.108 0 0 0
                                                                  with a mounted tool: mass in kg, TCP in m / rad
python examples/function_templates/03_move_to.py --workspace workspace.real.json
```

| Template | Shows | UR name |
|---|---|---|
| `00_teach_workspace.py` | teach the work area in 4 steps → `workspace.json` | — |
| `01_read_state.py` | `joints()`, `pose()` — no motion | — |
| `02_move_joints.py` | `move_joints(q, speed, accel)` | moveJ |
| `03_move_to.py` | `move_to(pose, speed, accel)`, `ws.pose(u, v, w)` | moveL |
| `04_move_through.py` | `move_through(poses)`, `ws.line(...)` | moveL ×n |
| `05_grid_poses.py` | `ws.grid(nx, ny, w)` + your own per-cell loop | — |
| `06_digital_io.py` | `read_digital_in`, `set_digital_out`, `set_tool_digital_out` | I/O |
| `07_analog_io.py` | `read_analog_in`, `set_analog_out` | I/O |
| `08_safety_rejections.py` | what a rejected move looks like, `SafetyViolation` | — |
| `09_raw_rtde.py` | the `ur_rtde` objects underneath (`conn.receive/control/io`) — sim only, refuses `--target real` | moveJ raw |

## The workspace: teach once, every example scales to it

All templates (and your own scripts) move inside a **taught box**. You
teach it once per setup with `00_teach_workspace.py` — four points, the
script only reads where the arm is, you move it (sim: pendant **Move** tab;
real robot: **freedrive button** on the pendant, supervised):

1. **corner A** — one corner of the work area, tool on the surface
2. **corner B** — the diagonally opposite corner, on the surface
3. **top** — the highest point the tool should reach
4. **home** — a comfortable pose above the area; every example starts and
   ends here (`robot.move_joints(ws.home_joints)`)

Result: `workspace.json` in the repo folder (git-ignored, one per setup).
In a hurry in the sim: `--demo` writes a 300 × 300 × 200 mm box around the
sim's start pose instead.

```python
from ur5e import Robot, Workspace

ws = Workspace.load()                      # workspace.json
with Robot(target="sim", profile=ws) as robot:
    robot.move_joints(ws.home_joints)      # taught home
    robot.move_to(ws.pose(0.5, 0.5, 0.3))  # centre of the area, 30 % of the height
    for cell in ws.grid(3, 3, w=0.1):      # 3 x 3 over the area, 10 % up
        robot.move_to(cell)
```

`ws.pose(u, v, w)` takes **fractions 0…1** of the box along x, y, z —
`(0, 0, 0)` is the low corner on the surface, `(1, 1, 1)` the far top. The
orientation is the one you taught at corner A (tool pointing down). Because
everything is relative to the box, the same script runs on a 300 mm demo
box in the sim and on the 600 × 500 mm cart in the lab.

`Robot(profile=ws)` turns the box into the safety profile for that run: moves
outside it are rejected before they are sent. On the **real robot** the box
is additionally clipped to the measured cell (`cart_against_wall`) and keeps
its speed ceilings — you can make the space smaller, never larger. A
workspace taught in the sim is refused on the real robot: teach it again
there (two minutes with freedrive).

## On the real arm (`--target real`, lab, with the instructor)

The same file, one flag. `--target real` connects to `UR_HOST`
(192.168.1.11 — start the shell with `-e UR_HOST=192.168.1.11`, your laptop
on the robot's `192.168.1.x` network), enables real-arm motion and declares the
tool: `--payload` (kg) and `--tcp x y z rx ry rz` (m, rad) — the same values as
the pendant's *Installation → General → Payload / TCP*. Defaults are 0 kg and
the flange, i.e. **nothing mounted**; if something is mounted, pass its real
values, do not guess. The script prints a `REAL ARM …` line with what it set.

Order in the lab:

1. Pendant: robot ON → START, header icon → **Remote Control**, speed slider
   down (30 %), hand near the stop.
2. Freedrive the arm **over the table** — the cell box is
   x −0.485…−0.201, y −0.286…0.240, z 0.03…0.60 m (`cart_against_wall`);
   anything beyond the table's front edge is outside and would be rejected as
   a target. After teaching, the script prints the box; a `!! clipped` line
   means part of your box lies outside the cell — `ws.pose/grid/line` then use
   the clipped part, and a taught point listed as outside (e.g. `home`) must be
   re-taught, because the arm is not allowed to go there.
3. `python examples/function_templates/00_teach_workspace.py --target real`
   — all four points over the table, `home` included.
4. `python examples/function_templates/01_read_state.py --target real`
   (no motion) then `03_move_to.py --target real`, and so on.

The gate behind this (`allow_real=True` + validated profile + payload + TCP)
is unchanged; the flag only supplies it. Without `--target real` a script can
never move the real arm, even with `UR_HOST` set.

Flagged assumptions: poses are TCP poses with the pendant's active TCP at
teaching time; with a long tool, teach `top` and `home` with extra headroom
(the joint check looks at the flange). The simulator's inputs always read 0.
