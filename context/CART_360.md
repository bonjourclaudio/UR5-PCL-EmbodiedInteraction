# The lab cell `cart_360` — context for an AI assistant

Read together with `INSTRUCTIONS.md` (how to help) and `KNOWLEDGE.md` (API,
facts). This file covers one thing: the lab's safety setup from 2026-11-18 and
how to write scripts that work with it. Human version: `docs/SAFETY_SETUP.md`.

## 1. What the setup is

- The UR5e stands on its cart. It may work **above the cart top, around the
  cart, and below the level of the cart top**, down towards the floor. The
  same setup applies to every group.
- The **cart body is a keep-out**. The floor is a limit. Nothing else is
  modelled.
- Two layers. The pendant's Safety Configuration (installation `cart_360`:
  force 100 N, tool speed 250 mm/s, **no safety planes**) is the authoritative
  one — it limits how hard and fast the arm can hit something, not where it
  may go. The framework's cell profile `cart_360` checks every
  `move_…` call before it is sent and raises `SafetyViolation`; it is the
  secondary one. There is no slow zone: the limits are the same everywhere.
- `cart_360` is the default profile for `target="real"`. A taught `Workspace`
  passed as `profile=ws` is clipped to it and inherits the keep-out.
- Frame: robot **Base**. z = 0 is the robot's mounting plane; the cart top is
  10 mm below it (z = −0.010 m), so **everything below the cart top has
  negative z**. The cart is 690 × 690 × 790 mm: sides at x = −0.566 (front) and
  +0.124 (behind the robot), y = ±0.345, floor at z = −0.800. A handle sticks
  out 60 mm on the −Y side just below the top; the robot stands on a
  180 × 180 mm plate. Handle and plate are keep-outs of their own (the space
  under the handle is free).

## 2. Numbers — do not quote them from memory

The cart's footprint, the floor height and the clearances are measured values
in `src/ur5e/cells/cart_360.json` and can change. Ask the student to run

```bash
python -m ur5e.cell
```

and paste the output, or read the JSON file. What the profile contains:

| item | rule |
|---|---|
| TCP box | reach-sized (about ±0.95 m in x/y), from 50 mm above the floor up to z = 1.0 m |
| keep-outs `cart`, `handle`, `mounting plate` | boxes: the cart from the floor to its top, the handle, the plate |
| tool vs. keep-out | the TCP and the straight line flange → TCP stay `side_clearance` (20 mm) from a keep-out's **sides and underside**; its top surface may be touched |
| arm vs. keep-out | upper arm, elbow, forearm, wrist 1–3, as spheres of `arm_radius` (60 mm) around their centre lines, stay clear of it on **all** sides |
| floor | TCP 50 mm above it (the lower end of the TCP box); arm links above it |
| ceilings | TCP 0.15 m/s, 0.5 m/s²; joints 0.5 rad/s, 0.8 rad/s²; payload 0.5 kg |

## 3. What is checked on each move

| call | checked before anything is sent |
|---|---|
| `robot.move_to(pose)` / `move_through` | target TCP; the **straight TCP line** from the current pose to the target; the flange line; the tool line at the target; arm links at up to 12 points along the line |
| `robot.move_joints(q)` | target (TCP via forward kinematics); arm links every ≤ 3° of joint travel between the **current** joints and `q` |

Consequences for planning:

- A move is judged **from where the arm is now**. The same target can be
  accepted from one start pose and rejected from another.
- A straight line from above the cart top to a point beside the cart cuts
  through the cart's edge → rejected. Route **out past the edge first, then
  down**; leave the same way.
- The arm itself needs room: the forearm passes over the cart's edge when the
  tool is beside the cart. A rejection naming a link (`the forearm would come
  closer than 60 mm …`) is solved by moving the path further out from the
  cart or approaching from another side — not by changing the target height
  alone.
- `move_joints` between distant poses swings the whole arm, often through the
  cart. Use it for `home` and short adjustments; go around the cart in
  `move_to` legs.
- Tool direction matters: the flange sits one tool length behind the TCP.
  Tool pointing **at** the cart side → flange further out (fine). Tool
  pointing **away** from the cart → the flange is between TCP and cart and may
  be the part that is too close.

## 4. Patterns to use

Rehearse in the simulator against the lab cell — the default sim profile has
no cart:

```bash
python students/<name>/my_script.py --cell cart_360
```

(`--cell` comes with the `ur5e.cli` lines every template uses. Without them:
start the shell with `docker compose run --rm -e UR5E_CELL=cart_360 dev bash`.)
Viewer scene `cart_360` draws the cart with its handle and the mounting plate.

Out, then down, and back the same way:

```python
from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs
from ur5e.safety import SafetyViolation

args = argument_parser(__doc__).parse_args()
ws = Workspace.load(args.workspace)

# measured by the student, Base frame, metres — placeholders here
ROT    = ws.orientation                      # taught tool orientation
ABOVE  = [-0.30,  0.10,  0.25, *ROT]         # over the cart top
OUT    = [-0.30,  0.60,  0.25, *ROT]         # past the +Y edge, still above the top
BESIDE = [-0.30,  0.60, -0.20, *ROT]         # beside the cart, 190 mm below its top

with Robot(**robot_kwargs(args), profile=None) as robot:    # None = the cell profile
    robot.move_joints(ws.home_joints)
    legs, done = [ABOVE, OUT, BESIDE], []
    try:
        for pose in legs:
            robot.move_to(pose)
            done.append(pose)
        # ... the process step beside the cart ...
    except SafetyViolation as e:
        print("rejected:", e)                # nothing was sent for this leg
    finally:
        for pose in reversed(done[:-1]):     # back out along the legs already driven
            robot.move_to(pose)
        robot.move_joints(ws.home_joints)
```

- `profile=None` uses the whole cell — right for absolute poses around the
  cart. `profile=ws` restricts the run to the taught box (still inside the
  cell): use it when all poses lie in one taught area.
- A workspace can be taught **beside** the cart (8 corners + home there). Its
  box may reach from below the cart top to above it; `ws.pose(u, v, 0)` is
  then the low side (negative z). `ws.describe()` prints a `note: … reaches
  into the keep-out` line if the box overlaps the cart.
- Catching `SafetyViolation` to print the reason and retreat (as above) is
  fine. The retreat legs were accepted on the way in, so they are accepted on
  the way out.
- Suggest half the ceiling (≈ 0.07 m/s) for any new move beside the cart.
- Where the arm reaches best below the cart top: behind the robot (+X, the
  cart edge is only 124 mm from the robot axis) and the +Y side. The front
  edge (−X) is 566 mm from the axis — little is reachable below the top there.
  The −Y side has the handle: stay under it or further out.
- Reaching **under a table or other object next to the cart**: that object is
  not in the cell. Have the student measure it in the Base frame (touch its
  edges with the tool, read `robot.pose()`) and keep the waypoints clear of it
  with their own margin, as named constants in their script.

## 5. Rejections

| message contains | cause | what to change |
|---|---|---|
| `is inside the keep-out 'cart'` | target in the cart, or below the top and closer than the clearance to a side | move the point out, or above the top |
| `TCP path: the straight line … passes through the keep-out` | the leg cuts through the cart | add a waypoint above the top or outside the corner |
| `flange path: …` / `tool (flange to TCP) …` | the flange or the tool line is the part that crosses | turn the tool towards the cart, or move further out |
| `the forearm` / `upper arm` / `elbow` / `wrist N` `would come closer than 60 mm` | an arm link too close at some point of the move (the message gives the percentage of the way and the point) | route further out from the cart, or from another side |
| `would touch the floor` | an arm link at floor level | stay higher |
| `no joint solution at …` | the straight line leaves the reach or runs into a singularity (arm fully stretched, or wrist straight above/below the base) | shorter legs, different route, or a `move_joints` for that leg |
| `the arm is ALREADY there` | the arm stands inside the keep-out (moved by hand) | the student freedrives it clear; no script fix |
| `profile 'cart_360' is not validated` | the cell is not released for the real arm | instructor; nothing to change in the script |
| `outside workspace box of profile` | outside the reach box or the taught box | check the numbers / the taught box |

## 6. Do not

- Do not edit `src/ur5e/cells/cart_360.json`, clearances, or `safety.py`, and
  do not pass a custom `SafetyLimits`.
- Do not set `UR5E_CELL=open`, `--cell cart_against_wall` or `--cell sim_open`
  to get a move through, and do not use `robot.conn.control.moveL/moveJ` to go
  around the check.
- Do not assume the checks cover the student's tool. Only the line flange →
  TCP is checked, and the clearance is measured from that line: a gripper body,
  a hose or a camera beside it is invisible. Ask how wide the tool is and keep
  that much extra distance to the cart's side in the waypoints.
- Do not describe the viewer as a safety device; it only draws the cart.
- Do not rely on the pendant to stop a move at the cart or the floor: it has
  no planes. Position limits exist only in the framework's checks.
- Do not add warnings or rules beyond `docs/SAFETY_SETUP.md`. The checks do
  not know where people are; who stands where is the instructor's decision.
