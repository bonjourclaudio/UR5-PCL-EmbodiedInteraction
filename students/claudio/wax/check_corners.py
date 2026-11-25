"""Hover over the 4 corners of the drawable area:
home -> each corner at the clearing height, down to --near-mm ABOVE THE WAX SURFACE, hold, up -> home.

The surface is the cutting level + the engraving depth (cut_offset_mm in the setup file),
so --near-mm 10 stops the tip 10 mm above the wax.

Usage:
  python students/claudio/wax/check_corners.py --target real --payload 0.35 --tcp 0 0 0 0 0 0
  python students/claudio/wax/check_corners.py --target real --hold 0   # wait for Enter at each corner
"""

from __future__ import annotations

import json
import time

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs

from draw_wax import HERE, apply_setup, check_move


def main() -> None:
    ap = argument_parser(__doc__)
    ap.add_argument("--near-mm", type=float, default=10.0, help="lowest hover, above the wax surface")
    ap.add_argument("--hold", type=float, default=4.0, help="s at each corner; 0 = wait for Enter")
    args = ap.parse_args()
    args.margin_mm = args.clear_mm = 0.0      # unused here
    args = apply_setup(args)
    setup = json.loads((HERE / f"wax_setup.{args.target}.json").read_text())
    wall, top, border = setup["wall_mm"], setup["ceiling_mm"], setup["border_mm"]
    depth = -setup.get("cut_offset_mm", 0.0)            # mm the cutting level lies below the surface
    clear_mm = setup.get("clear_mm", wall + 15)
    near_cut = args.near_mm + depth
    if near_cut > clear_mm or near_cut > top:
        raise SystemExit(f"--near-mm {args.near_mm} is above the clearing height")
    clear, near = clear_mm / top, near_cut / top
    ws = Workspace.load(args.workspace)
    print(f"corners: clearing height {clear_mm - depth:.0f} mm, then down to {args.near_mm:.0f} mm above the wax")

    with Robot(**robot_kwargs(args), profile=ws) as robot:
        print("home")
        check_move(robot, robot.move_joints(ws.home_joints, speed=0.3), "home")
        for name, (u, v) in zip(("bottom-left", "top-left", "top-right", "bottom-right"),
                                ((0, 1), (1, 1), (1, 0), (0, 0))):
            print(f"-> {name} corner, {clear_mm - depth:.0f} mm above the wax")
            check_move(robot, robot.move_to(ws.pose(u, v, clear), speed=0.03, accel=0.2), name)
            print(f"   down to {args.near_mm:.0f} mm above the wax")
            check_move(robot, robot.move_to(ws.pose(u, v, near), speed=0.01, accel=0.2), name)
            print(f"   check: tip {border:.0f} mm from both walls, {args.near_mm:.0f} mm above the wax", flush=True)
            if args.hold > 0:
                time.sleep(args.hold)
            else:
                input("   Enter to continue")
            check_move(robot, robot.move_to(ws.pose(u, v, clear), speed=0.01, accel=0.2), name)
        print("home")
        check_move(robot, robot.move_joints(ws.home_joints, speed=0.3), "home")
        print("done.")


if __name__ == "__main__":
    main()
