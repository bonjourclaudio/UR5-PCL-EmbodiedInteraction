"""Place the wax frame: the arm points at where the canvas should go.

The TCP hovers at --z (absolute, Base frame) over the centre and then the four
corners of a --size-mm square around --center. At each point it waits for
Enter, so you can slide the frame on the table until the tip is over the
frame's inner corner. No workspace needed: the cell profile (real) /
sim_open (sim) box applies.

The default centre is the middle of the lab cell box (x -0.485..-0.201,
y -0.286..+0.240), where an 18 cm canvas has ~5 cm of room on each side in x.
--z must be well above the frame's top edge — measure first.

The tool must already point straight down (tilt < 10 deg) — the script keeps
the current orientation and refuses otherwise.

Usage:
  python students/claudio/wax/place_frame.py
  python students/claudio/wax/place_frame.py --target real --payload KG --tcp X Y Z RX RY RZ --z 0.15
"""

from __future__ import annotations

import math

import numpy as np

from ur5e import Robot
from ur5e.cli import argument_parser, robot_kwargs


def tool_tilt_deg(rotvec) -> float:
    """Angle between the tool's z axis and straight down."""
    r = np.asarray(rotvec, float)
    a = np.linalg.norm(r)
    if a < 1e-9:
        z = np.array([0.0, 0.0, 1.0])
    else:
        k = r / a
        z = np.array([0, 0, 1.0]) * math.cos(a) + np.cross(k, [0, 0, 1.0]) * math.sin(a) \
            + k * k[2] * (1 - math.cos(a))
    return math.degrees(math.acos(np.clip(-z[2], -1, 1)))


def main() -> None:
    ap = argument_parser(__doc__)
    ap.add_argument("--center", type=float, nargs=2, default=(-0.343, -0.023), metavar=("X", "Y"),
                    help="canvas centre, m, Base frame")
    ap.add_argument("--size-mm", type=float, default=180.0, help="inner size of the frame")
    ap.add_argument("--z", type=float, default=0.15, help="hover height of the tip, m (Base z)")
    ap.add_argument("--speed", type=float, default=0.05, help="m/s")
    args = ap.parse_args()

    cx, cy = args.center
    h = args.size_mm / 2000
    points = [("centre", cx, cy), ("corner +X+Y (far left from the front)", cx + h, cy + h),
              ("corner +X-Y (far right)", cx + h, cy - h), ("corner -X-Y (near right)", cx - h, cy - h),
              ("corner -X+Y (near left)", cx - h, cy + h), ("centre", cx, cy)]

    with Robot(**robot_kwargs(args)) as robot:
        start = robot.pose()
        tilt = tool_tilt_deg(start[3:])
        print(f"tool tilt {tilt:.1f} deg from straight down, tip at "
              f"{[round(v * 1000) for v in start[:3]]} mm")
        if tilt > 10:
            raise SystemExit("point the tool straight down first (pendant Move tab / freedrive), then rerun")
        rot = list(start[3:])
        if start[2] < args.z:
            print(f"up to z = {args.z * 1000:.0f} mm first")
            robot.move_to([start[0], start[1], args.z, *rot], speed=args.speed)
        for name, x, y in points:
            print(f"-> {name}: x {x * 1000:.0f}  y {y * 1000:.0f}  z {args.z * 1000:.0f} mm")
            ok = robot.move_to([x, y, args.z, *rot], speed=args.speed)
            if ok is False:
                raise SystemExit("controller refused the move — check the pendant (protective stop?)")
            input("   align the frame under the tip, then Enter ")
    print("done. Fix the frame to the table (clamps / tape) before teaching the workspace.")


if __name__ == "__main__":
    main()
