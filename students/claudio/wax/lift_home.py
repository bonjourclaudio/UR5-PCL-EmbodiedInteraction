"""After a stop: lift the tool straight up out of the wax, then go to the taught home.

  1. straight up by --up-mm (default 20) at 10 mm/s, no sideways movement
  2. joint move to the saved home of the wax setup

Usage:
  python students/claudio/wax/lift_home.py --target real
"""

from __future__ import annotations

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs

from draw_wax import apply_setup, check_move


def main() -> None:
    ap = argument_parser(__doc__)
    ap.add_argument("--up-mm", type=float, default=20.0, help="lift straight up first")
    args = ap.parse_args()
    args.margin_mm = args.clear_mm = 0.0      # unused here; keeps apply_setup quiet
    args = apply_setup(args)
    ws = Workspace.load(args.workspace)
    # the lift may start below the workspace's travel heights -> check it against the lab cell
    with Robot(**robot_kwargs(args), profile=None) as robot:
        p = robot.pose()
        if p[2] < ws.usable_box.z[1]:                  # down at the canvas -> out of the wax first
            print(f"lift {args.up_mm:.0f} mm from z {p[2] * 1000:.1f} mm", flush=True)
            check_move(robot, robot.move_to([p[0], p[1], p[2] + args.up_mm / 1000, *p[3:]],
                                            speed=0.01, accel=0.1), "lift")
        else:
            print(f"tool at z {p[2] * 1000:.0f} mm, above the canvas box — no lift needed", flush=True)
    with Robot(**robot_kwargs(args), profile=ws) as robot:
        print("home", flush=True)
        check_move(robot, robot.move_joints(ws.home_joints, speed=0.3), "home")
        print("at home.")


if __name__ == "__main__":
    main()
