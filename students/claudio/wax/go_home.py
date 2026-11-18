"""Move the arm to the taught home of the wax setup (joint move), nothing else.

Usage:
  python students/claudio/wax/go_home.py --target real
  python students/claudio/wax/go_home.py            # sim
"""

from __future__ import annotations

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs

from draw_wax import apply_setup, check_move


def main() -> None:
    ap = argument_parser(__doc__)
    args = ap.parse_args()
    args.margin_mm = args.clear_mm = 0.0      # unused here; keeps apply_setup quiet
    args = apply_setup(args)
    ws = Workspace.load(args.workspace)
    with Robot(**robot_kwargs(args), profile=ws) as robot:
        print("home")
        check_move(robot, robot.move_joints(ws.home_joints, speed=0.3), "home")
        print("at home.")


if __name__ == "__main__":
    main()
