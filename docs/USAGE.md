# Usage — working with the framework

Setup done ([SETUP.md](SETUP.md))? This page covers: the Python API, teaching
a workspace and the function templates, sim vs. real robot, the 3D viewer
(scenes, models, toolheads, safety planes), and getting PolyScope programs
from the sim onto the real pendant.

Everything you write goes into **`students/<your-name>/`** (see
`students/README.md`). Files in `src/`, `scripts/`, `examples/` are replaced on
every `git pull` — copy, then edit the copy.

---

## 1. Working loop

1. Sim + viewer running, robot ON → START → Remote Control
   (`docker compose up -d ursim viewer`, see SETUP.md).
2. Shell inside the container, run your script, watch the arm at
   localhost:7001:

   ```bash
   docker compose run --rm dev bash
   python students/anna/my_script.py
   exit
   ```

(Copy commands without a trailing `# comment` — the Mac's default shell
passes the comment words on to the command.)

Edit with VS Code on your laptop — the repo is mounted into the container, so
saved files run immediately, no rebuild, no copying. Keep the dev shell open
and rerun `python …` as often as you like. `Ctrl+C` stops a running script
(the robot stops at the end of the current move).

Several shells at once are fine: `docker compose run --rm dev bash` again in a
second terminal.

## 2. The API — `ur5e.Robot`

One import, one object, used as a context manager (connects on enter,
disconnects cleanly on exit — also when your script crashes):

```python
from ur5e import Robot, grid_poses, line_poses

with Robot(target="sim") as robot:
    print(robot.joints())        # [q0..q5] in radians
    print(robot.pose())          # [x, y, z, rx, ry, rz]  metres + rotation vector (radians)

    here = robot.pose()
    up = list(here); up[2] += 0.05           # 5 cm up, same orientation
    robot.move_to(up)                        # straight-line (linear) move
    robot.move_to(here)
```

**Units** are the robot's own: metres, radians, poses as
`[x, y, z, rx, ry, rz]` in the robot **Base** frame with `rx ry rz` a rotation
vector — the same numbers the pendant shows under *Move → TCP* when set to
"Rotation Vector [rad]". Read `robot.pose()` to get a valid orientation and
modify only the parts you need.

### Motion

| Call | What it does |
|---|---|
| `robot.move_to(pose, speed=None, accel=None)` | Linear move of the TCP to `pose`. `speed` m/s, `accel` m/s². Defaults = the profile's maximum (sim: 0.25 m/s). |
| `robot.move_joints(q, speed=None, accel=None)` | Joint-space move to six joint angles (rad). `speed` rad/s. |
| `robot.move_through(poses, speed=None, accel=None)` | `move_to` each pose in turn, stopping at every point. |

Every move is **checked before it is sent**: target inside the profile's
workspace box, speed/accel under the ceilings, joint angles inside limits.
A violation raises `ur5e.safety.SafetyViolation` with a readable message and
**nothing moves** — the framework never silently clamps your values. In the sim
the profile is `sim_open` (box ±0.9 m, z −0.25…1.0 m — generous; only the speed
guards really bite). Catch it if you want to handle it:

```python
from ur5e.safety import SafetyViolation
try:
    robot.move_to(target)
except SafetyViolation as e:
    print("rejected:", e)
```

### Patterns (geometry only, no motion)

```python
cells = grid_poses(origin, nx=3, ny=4, dx=0.04, dy=0.04)   # 12 poses, snake order, origin's z + orientation
path  = line_poses(start_pose, [x, y, z], n=10)            # 10 poses from start to the xyz point
```

Generators *return lists of poses*; you decide what happens at each one:

```python
for k, cell in enumerate(grid_poses(robot.pose(), nx=3, ny=3, dx=0.04, dy=0.04)):
    robot.move_to(cell)
    if robot.read_digital_in(0):          # sensor says "do it"?
        robot.set_digital_out(0, True)    # fire the actuator
        robot.set_digital_out(0, False)
```

Pure traversal is `robot.move_through(grid_poses(...))`.

### Sensors and actuators (controller I/O)

| Call | Meaning |
|---|---|
| `robot.read_digital_in(pin) -> bool` | standard digital input 0–7 |
| `robot.read_analog_in(pin) -> float` | standard analog input 0 or 1 (volts) |
| `robot.set_digital_out(pin, value)` | standard digital output |
| `robot.set_tool_digital_out(pin, value)` | tool flange digital output 0/1 |
| `robot.set_analog_out(pin, voltage)` | standard analog output |

**In the sim all inputs read 0 — always.** URSim 5.25 has no way to drive the
inputs that RTDE reports (the I/O tab's input checkboxes are read-only; the
pendant's *Simulation* toggle only affects PolyScope programs, not what your
script reads). So to exercise a sensor-dependent branch without hardware,
fake the sensor in code for a moment — e.g. `triggered = True` for one cell,
or a threshold of `-1` — and remove the fake before the lab. Outputs do work:
`set_digital_out(0, True)` shows up as a ticked box in the I/O tab.

`[screenshot: PolyScope I/O tab, Digital Output 0 ticked by a script]`

### Lower levels (when you need them)

`robot.conn.receive` / `robot.conn.control` / `robot.conn.io` are the raw
`ur_rtde` interfaces (`RTDEReceiveInterface`, `RTDEControlInterface`,
`RTDEIOInterface`) — the full ur_rtde API if `Robot` doesn't have what you
need. Raw calls skip the framework's checks.

## 3. Teach your workspace, then use the function templates

`examples/function_templates/` has **one short script per API call**
(`01_read_state.py` … `09_raw_rtde.py`, table in its README). They all move
inside a **taught workspace** — a box you teach once per setup, in four steps,
with `00_teach_workspace.py`:

```bash
python examples/function_templates/00_teach_workspace.py --target sim
```

The script only *reads* where the arm is; you move it — in the sim with the
pendant's **Move** tab, on the real robot with the **freedrive button** on the
back of the pendant (supervised). Press Enter at: **corner A** (on the
surface), **corner B** (diagonally opposite, on the surface), **top** (highest
point the tool should reach), **home** (a comfortable pose above the area —
every example starts and ends there). Result: `workspace.json` in the repo
folder (git-ignored). In a hurry: `--demo` writes a 300 × 300 × 200 mm box
around the sim's start pose instead.

`[screenshot: terminal during the 4 teach steps]`

From then on, scripts address the box in **fractions**:

```python
from ur5e import Robot, Workspace

ws = Workspace.load()
with Robot(target="sim", profile=ws) as robot:
    robot.move_joints(ws.home_joints)         # taught home
    robot.move_to(ws.pose(0.5, 0.5, 0.3))     # centre of the area, 30 % of the height
    robot.move_through(ws.line((0.1, 0.1, 0.2), (0.9, 0.9, 0.2), n=6))
    for cell in ws.grid(3, 3, w=0.1):         # 3 x 3 over the area, 10 % up
        robot.move_to(cell)
```

`ws.pose(u, v, w)`: `u`, `v`, `w` are 0…1 along x, y, z of the box; the
orientation is the one you taught. The same script therefore runs on the
small demo box in the sim and on the full cart in the lab. `Robot(profile=ws)`
makes the box the safety profile for that run — moves outside it are
rejected. On the **real robot** the box is clipped to the measured cell
(`cart_against_wall`) and keeps its speed ceilings; a workspace taught in the
sim is refused there (teach it again in the lab — two minutes).

Run the templates in order once — `02_move_joints.py` (moveJ),
`03_move_to.py` (moveL), `04_move_through.py`, `05_grid_poses.py`,
`06/07` I/O, `08_safety_rejections.py` (what a rejected move looks like) —
then copy whatever you need into `students/<name>/`.

Every template (and `grid_process.py`) takes the same flags — `--help` lists
them: `--target sim|real` (default sim), `--workspace FILE`, `--host`, and for
the real arm `--payload KG` and `--tcp X Y Z RX RY RZ`. Your own scripts get
them with two lines:

```python
from ur5e.cli import argument_parser, robot_kwargs

args = argument_parser(__doc__).parse_args()
ws = Workspace.load(args.workspace)
with Robot(**robot_kwargs(args), profile=ws) as robot:
    ...
```

## 3b. The other examples

| File | What it shows | Watch for |
|---|---|---|
| `scripts/check_connection.py --target sim` | read-only state dump, no motion | run it first whenever something seems off |
| `scripts/smoke_test_move.py` | the smallest possible move (wrist ±10°) via the raw interface | **sim only** — it bypasses the checks on purpose |
| `examples/grid_process.py` | 3×3 grid from the current pose, read sensor at each cell, fire actuator if triggered | every cell prints `skip` in the sim (inputs read 0 — see above); the docstring's "set the input in the I/O tab" does not work in URSim 5.25 |

Run them from the dev shell: `python examples/grid_process.py`. The grid is
anchored at the **current TCP pose**, 4 cm steps in +X/+Y of the Base frame —
fine from URSim's start pose. If you first drove the arm somewhere awkward
with the pendant, bring it back with the Move tab or `move_joints(...)`.

To start your own script: `cp examples/grid_process.py students/<name>/x.py`.

## 4. Sim vs. real robot

The code is identical; **only the host changes**. `Robot(target="sim")` reads
the host from the `URSIM_HOST` variable (set to `ursim` by compose),
`Robot(target="real")` from `UR_HOST`.

| | Sim (default) | Real arm (lab, with the instructor) |
|---|---|---|
| start shell | `docker compose run --rm dev bash` | `docker compose run --rm --no-deps -e UR_HOST=192.168.1.11 dev bash` (`--no-deps`: don't start URSim) |
| examples / templates | `python examples/function_templates/03_move_to.py` | `python examples/function_templates/03_move_to.py --target real` (+ `--payload KG --tcp …` if a tool is mounted) |
| in your own code | `Robot(target="sim", profile=ws)` | `Robot(target="real", profile=ws, allow_real=True, payload_kg=…, tcp=…)` — or the `ur5e.cli` lines from §3, which do exactly this for `--target real` |
| profile | `sim_open` (or the taught box) | `cart_against_wall` — the measured work surface of the cart (or the taught box clipped to it); moves outside it are rejected |
| workspace | `00_teach_workspace.py --target sim` (or `--demo`) | `00_teach_workspace.py --target real` — teach all four points **over the table**, home included |
| check | `python scripts/check_connection.py --target sim` | `python scripts/check_connection.py --target real` |

The lab robot's IP is **static: `192.168.1.11`**. If in doubt, the pendant
shows it under **☰ (top right) → About** — *IP address*, next to the PolyScope
version (*URSoftware 5.9.1*, which is why the simulator is pinned to 5.9.4).

### Your laptop must be on the robot's network — check this first

The robot hangs on its own wired network, **`192.168.1.x`**, with no DHCP
server: a laptop plugged in gets no address by itself, and an address from
another network (Wi-Fi `10.x.x.x`, or `192.168.10.x` — note the **10**) is
*not* the same network even though it looks similar. The symptom is always the
same: `check_connection.py` **hangs** until Ctrl-C and then says `Could not
connect to: 192.168.1.11 at 30004` (the simulator, when it is off, answers
*refused* immediately — hanging means "nobody there on that road").

1. Plug the Ethernet cable (USB-C adapter if needed) into the lab switch /
   the control box. Wi-Fi can stay on.
2. Give the adapter a **manual IP in the robot's subnet**:
   - macOS: System Settings → Network → the Ethernet adapter → Details →
     TCP/IP → *Configure IPv4: Manually* — IP `192.168.1.1NN`, subnet mask
     `255.255.255.0`, router empty. Pick `NN` from your student number /
     seat (101…199) so that no two laptops share an address; the instructor's
     Mac is `.20`, the robot `.11`.
   - Windows: Settings → Network & Internet → Ethernet → *IP assignment: Edit*
     → Manual, IPv4 on — same values.
3. Test **outside Docker**, in a normal terminal:

   ```bash
   ping -c 3 192.168.1.11
   nc -vz -w 3 192.168.1.11 30004
   ```

   (Windows: `ping 192.168.1.11` and `Test-NetConnection 192.168.1.11 -Port 30004`
   in PowerShell.) Ping replies and `succeeded` on 30004 → the laptop is good;
   now the Docker command below works. No replies → recheck the IP you set
   (`ifconfig | grep "inet "` must show a `192.168.1.` line) and the cable.

Set the address back to *DHCP/Automatic* when you leave the lab if you use
that adapter elsewhere.

Real motion is refused
unless **all** of: `allow_real=True`, the profile is validated (the shipped
`cart_against_wall` is), and the tool was declared (`set_tool(...)` or the
`payload_kg=`/`tcp=` arguments) with the mounted tool's **payload (kg)** and
**TCP offset** `[x, y, z, rx, ry, rz]` (m, rad) — the same values as the
pendant's *Installation → General → TCP / Payload*. `--target real` on an
example does all three (defaults: 0 kg, flange = nothing mounted) and prints a
`REAL ARM …` line with what it set. Without `--target real` (or without
`allow_real=True` in your own code) nothing can move the real arm, even with
`UR_HOST` set. The arm must be ON, brakes released, and in **Remote Control**.
You normally work in simulation; driving the real arm happens only in the lab,
supervised.

Where the arm is parked matters: the first target of every script must be
inside the box. The cell box is x −0.485…−0.201, y −0.286…0.240, z 0.03…0.60 m
(Base frame, the table in front of the robot) — an arm hanging beyond the
table's front edge is outside, and a workspace taught there would have no
overlap with the cell (`WorkspaceError`). Freedrive it over the table before
teaching. If only part of your taught box sticks out, `00_teach_workspace.py`
/ `ws.describe()` print a `!! clipped` line: `ws.pose/grid/line` then use the
clipped part automatically, but a taught point listed as outside (`home`,
`top`, …) is a place the arm may not go — teach it again inside.

`scripts/smoke_test_move.py` is wired to the sim host and bypasses the checks —
it is not meant for the real arm.

## 5. The 3D viewer (localhost:7001)

The viewer shows the arm from live robot data (sim or real), with the Base
frame, the tool flange (`tool0`) and TCP frames, and a 10 cm grid. It is
**receive-only** and runs next to your scripts. It replaces PolyScope's own 3D
view, which the pendant hides in Remote Control.

> **Nothing in the viewer limits or protects the robot.** Models, toolheads and
> safety planes here are a planning sketch. Only the controller's own Safety
> Configuration (on the pendant) acts on the robot.

Right-hand panel, top to bottom:

`[screenshot: viewer panel overview]`

### Scenes

A *scene* = your workspace layout: objects, toolhead + TCP, features, safety
planes, camera. Stored as a folder `scenes/<name>/scene.json` + `models/` with
the mesh files next to it — one folder is the whole scene; zip it, commit it,
AirDrop it.

- **Scene panel** → dropdown to switch scenes, **Load / Save / Save as / New /
  Reload**, *store camera*. The viewer starts on `default`.
- Shipped: `default` (empty) and `example_cart` (an illustrative 600 × 500 mm
  table in front of the robot and a wall feature behind it — **placement
  assumed, not measured**; it shows the format, not our cell).
- Make your own: *Save as* → `my_table` → it appears as `scenes/my_table/`.

The scene file is plain JSON (`"format": "ur5e-scene/1"`): lengths in metres,
angles in degrees unless the key says `_rad`, everything in the robot Base
frame. Sections: `robot.mount`, `toolhead`, `objects[]`, `features[]`,
`safety_planes[]`, `view`. You can edit it by hand and *Reload*.

### Your own models (table, fixtures, work pieces)

**Objects** folder → **Upload** an STL / OBJ / GLB / PLY (it is copied into the
scene's `models/`), or **Add** a file already there. Choose the file's
**units** (`mm` — what most CAD exports use — or `m`); the message line shows
the bounding box in mm after adding, so a 1000× mistake is obvious. Then place
it: drag the **gizmo** or type position [mm] / rotation RPY [deg]; colour,
opacity, visible, remove.

`[screenshot: object added with gizmo]`

### Toolhead and TCP

**Toolhead + TCP** folder: the TCP offset in the `tool0` frame — position [mm]
and rotation vector [rad], **the same numbers PolyScope's *Installation →
General → TCP* takes** (RPY shown read-only for the alternate pendant setting).
Optionally a toolhead mesh attached to the flange.

**Presets:** the `toolheads/<id>/` library is shared by all scenes
(`toolhead.json` + mesh). *Toolhead preset → Use selected preset* copies the
mesh into the scene and fills offset + TCP; *Save current toolhead to library*
does the reverse — share your tool with the class by committing/sending the
folder. Shipped preset `tool_example` (a flange plate with a probe; its TCP
`(0, 21, 108) mm` is **assumed from the geometry**, payload `0` is a
placeholder — measure before trusting either).

**TCP check:** the *Robot* folder compares the controller's actual TCP pose with
the viewer's own calculation (joint angles + scene TCP). A mismatch > 3 mm
shows a ⚠ badge: the pendant's active TCP is not the one in your scene. Fix one
of them.

### Safety planes (display only)

**Features** and **Safety planes** folders use the pendant's vocabulary
(*Installation → Safety → Planes*): a plane copies a **feature** (Base or a
plane feature you create), its **Z axis is the normal**, **displacement**
shifts it along that axis, **restriction mode** *Disabled / Normal / Reduced /
Normal & Reduced / Trigger Reduced Mode*, per-plane **Restrict elbow**. The
arrow marks the allowed side. Up to 8 planes, like the pendant.

The point: sketch your planes here, then type the same values into the pendant
1:1. The viewer colours a plane by mode and turns it **red** when the TCP (or
the elbow, if restricted) is on the wrong side — a visual aid while planning,
nothing more. The controller only knows about planes you enter on the pendant.

`[screenshot: safety plane with normal arrow, one turned red]`

### Without a robot

`docker compose run --rm --no-deps -p 7001:7000 dev python scripts/rtde_viewer.py --no-robot --scene my_table`
(stop the normal viewer first: `docker compose stop viewer`) — same page, with
joint sliders instead of live data. Useful for building a scene on the train.

## 6. PolyScope programs: from the sim to the real pendant

You can also program the robot the "normal" way, in PolyScope (the pendant UI
in your browser): **Program** tab → waypoints, Move nodes, I/O nodes → **Save**.

URSim saves to `/ursim/programs`, which is the **`programs/` folder in your
repo** on your laptop. After saving `mytest` you find:

```
programs/mytest.urp        the program  (this is what you carry over)
programs/mytest.script     generated URScript (read-only copy, handy to read)
programs/mytest.txt        text listing
programs/default.installation   the SIM's installation  ← do not copy
programs/default.variables      the SIM's variables     ← do not copy
```

**To the real robot:** copy **only the `.urp` file(s)** to a USB stick (FAT32),
plug it into the pendant, **Open → Program** → the USB appears as a drive →
load it. The pendant will say the program was made with a different
installation — keep the robot's **own** installation. (This works because the
simulator is pinned to the robot's PolyScope line, 5.9 — see the README.)

> ⚠️ **Never copy the sim's `.installation` (or `.variables`) file to the
> robot.** It holds the simulator's TCP, payload, mounting, I/O setup and
> *Safety Configuration* — none of which match the real cell. Loading it would
> overwrite the robot's settings. Programs (`.urp`) only.

Programs made in the sim reference the sim's waypoints; on the real arm check
every waypoint at low speed (speed slider down) before running at full speed.

`[screenshot: PolyScope Save program dialog / USB Open dialog]`

## 7. Updating

`git pull` (or re-download the ZIP), then `docker compose down &&
docker compose up -d ursim viewer`. The code lives in the repo, not in the
container image, so updates are instant. If the instructor announces a new
image version, `docker compose up` downloads it by itself.

## 8. Working with an AI assistant

Use whatever you normally use, but give it the course context first —
otherwise it will invent methods and units. `context/README.md` explains
which file to hand to which tool: at minimum `context/INSTRUCTIONS.md` (the
rules) and `context/KNOWLEDGE.md` (the API and the facts). Editor tools that
read the repo (Claude Code, Codex, Cursor, Copilot, …) find `AGENTS.md`,
`CLAUDE.md` and `.github/copilot-instructions.md` by themselves. After a
`git pull`, run `./context/make_source_snapshot.sh` to refresh
`context/framework_source.md`.

Two things the AI cannot know unless you tell it: your current
`workspace.json` (paste `ws.describe()` or the file) and the tool you have
mounted (payload, TCP). If it proposes a method that is not in
`context/KNOWLEDGE.md` §4, it does not exist.
