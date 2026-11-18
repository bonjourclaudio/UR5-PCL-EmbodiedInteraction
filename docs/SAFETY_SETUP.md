# Safety setup — the arm on the cart (`cart_360`)

One setup for all groups. The robot stands on its cart and may work **above
the cart top, around the cart, and below the level of the cart top** — down
towards the floor. The cart itself is the one thing in the middle the arm has
to stay out of.

It replaces the earlier table-only setup (`cart_against_wall`). Scripts and
workspaces that worked over the table keep working.

## The two layers

| | where | what it does |
|---|---|---|
| **Pendant** — installation `cart_360` | the robot's own Safety Configuration | Force and speed limits (100 N, 250 mm/s), the same everywhere — also beside and below the cart top. No safety planes: the pendant does not know where the cart or the floor is. This is the layer that counts when the arm touches something. |
| **Framework** — cell profile `cart_360` | `ur5e`, on every `move_…` call | Checks each move *before* it is sent and rejects it with a `SafetyViolation` if the tool or the arm would run into the cart or the floor. Catches planning mistakes early. |

The pendant installation is set up by the instructor and identified by its
**safety checksum** (four characters, top right of the pendant). Do not change
or reload it. If the pendant shows a different installation or checksum than
the one posted at the robot, stop and ask.

Print the numbers the framework uses (cart size, clearances, floor):

```bash
python -m ur5e.cell
```

## What is allowed

- **Above the cart top:** everything as before; the tool may touch the surface.
- **Beside and below the cart top:** allowed, at a distance —
  the tool (the line from flange to TCP) stays **20 mm** away from the cart's
  sides; the arm itself (upper arm, elbow, forearm, wrists) stays **60 mm**
  away from the cart's sides and top.
- **Towards the floor:** the TCP stays 50 mm above it; the arm itself stays clear of it.
- **The handle** on the cart's −Y side and the robot's mounting plate are
  keep-outs of their own; the space under the handle is usable.
- **Speed:** TCP ≤ 0.15 m/s, joints ≤ 0.5 rad/s, payload ≤ 0.5 kg — unchanged.

Every move is checked along its **whole path**, not only at the target: the
straight line of a `move_to`, the swing of a `move_joints`.

## Rehearse in the simulator first

The simulator normally has no cart. Add `--cell cart_360` and it checks your
script against the lab cell — the same rejections you would get on the real
arm:

```bash
python students/<name>/my_script.py --cell cart_360
```

(Scripts without the `ur5e.cli` lines: start the shell with
`docker compose run --rm -e UR5E_CELL=cart_360 dev bash`.)

In the viewer (localhost:7001) load the scene **`cart_360`**: it shows the cart
with its handle. The viewer only draws them.

## Getting around the cart

A straight line from above the table to a point beside the cart cuts through
the cart's edge. Go **out first, then down**:

```python
above  = [-0.30, 0.10, 0.25, rx, ry, rz]     # over the cart top
out    = [-0.30, 0.60, 0.25, rx, ry, rz]     # past the +Y edge, still above the top
beside = [-0.30, 0.60, -0.20, rx, ry, rz]    # down beside the cart

robot.move_to(above)
robot.move_to(out)
robot.move_to(beside)
robot.move_to(out)       # and back the same way
```

- Use your own measured positions — the numbers above are only the pattern.
- The cart top is at z = −0.010 m (the robot stands on a 10 mm plate), its
  sides at x = −0.566 / +0.124 and y = ±0.345 m. Behind the robot (+X) and on
  the +Y side the arm gets closest to the cart and lowest; the front edge is
  far from the robot, so little is reachable below the top there; the −Y side
  has the handle.
- Keep some distance: the forearm has to pass over the cart's edge. If a move
  is rejected because of the forearm or upper arm, move the path further out.
- `move_joints` between two far-apart poses swings the whole arm — around the
  cart that often passes through it. Prefer several short `move_to` legs.
- For area work beside the cart, teach a workspace there
  (`00_teach_workspace.py --target real`): the box may reach from below the
  cart top to above it; `home` inside the box.

## Reading a rejection

| message contains | meaning | fix |
|---|---|---|
| `is inside the keep-out 'cart'` | the target is in the cart or closer than 20 mm to its side | move the point out, or above the top |
| `TCP path: the straight line … passes through the keep-out` | the line to the target cuts through the cart | add a waypoint above the top or outside the corner |
| `the forearm` / `upper arm` / `wrist …` `would come closer than 60 mm` | at some point of the move the arm itself is too close to the cart | move further out, or approach from another side |
| `would touch the floor` | a wrist or the forearm at floor level | stay higher |
| `no joint solution at …` | the straight line leaves the arm's reach | shorter legs, or a different route |
| `the arm is ALREADY there` | the arm stands inside the keep-out (moved by hand) | freedrive it clear, then start again |
| `profile 'cart_360' is not validated` | the cell has not been released for the real arm yet | ask the instructor |

A rejection means nothing was sent — the arm has not moved.

## What the checks do not see

- **Your tool's shape.** Only the line from the flange to the TCP is checked,
  and the 20 mm are measured from that line. A wide gripper, a hose, a camera
  on the side is invisible to the framework — with a tool wider than about
  40 mm, keep your own extra distance to the cart's side.
- **Cables** to the tool.
- **Anything you place next to the cart** — a second table, a stand, a bucket.
  Reaching under a table beside the cart is checked against the cart and the
  floor, not against that table.
- **People.** Nothing in the framework or the viewer knows where anyone is.
  The pendant's force and speed limits are what apply when the arm meets
  something it was not told about.

The tool data you pass (`--payload`, `--tcp`) must be the real values of what
is mounted, the same as on the pendant — the checks are computed from them.
