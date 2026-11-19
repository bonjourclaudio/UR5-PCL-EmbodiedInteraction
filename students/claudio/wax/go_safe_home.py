"""Safe home for the wax setup — for a new tool / canvas whose heights are not measured yet.

  1. straight up by --up-mm from wherever the arm is (linear move, slow)
  2. joint move to the taught home RAISED by --raise-mm, tool exactly vertical, same wrist
     configuration as home (wrist 2 ~ +90 deg), wrist 3 turned the short way (cable)

Each move is printed and waits for Enter. Moves go through the framework's checks with the
lab cell (profile=None -> cart_360), not the taught workspace (that box is about to change).
Afterwards: pendant to Local, Move tab -> Base Z down to ~10 mm above the wax (keeps the tool
vertical), and read the position.

Usage (pass the TCP / payload that are ACTIVE ON THE PENDANT):
  python students/claudio/wax/go_safe_home.py --target real --payload 0.18 --tcp 0 0 0.015 0 0 0
"""

from __future__ import annotations

import json
import math

import numpy as np

from ur5e import Robot
from ur5e.cli import argument_parser, robot_kwargs
from ur5e.kinematics import fk_frames, ik_near, matrix_to_rotvec

from draw_wax import HERE, apply_setup, check_move
from geometry import straight_down


def main() -> None:
    ap = argument_parser(__doc__)
    ap.add_argument("--up-mm", type=float, default=200.0, help="straight up first")
    ap.add_argument("--raise-mm", type=float, default=150.0, help="safe home = taught home + this")
    args = ap.parse_args()
    args.margin_mm = args.clear_mm = 0.0
    args = apply_setup(args)
    setup = json.loads((HERE / f"wax_setup.{args.target}.json").read_text())
    home_q = np.array(setup["home"]["q"])
    deg = lambda q: [round(math.degrees(a), 1) for a in q]           # noqa: E731

    with Robot(**robot_kwargs(args), profile=None) as robot:
        pose = robot.pose()
        up = [pose[0], pose[1], pose[2] + args.up_mm / 1000, *pose[3:]]
        print(f"now: TCP {[round(v * 1000) for v in pose[:3]]} mm, joints {deg(robot.joints())}")
        input(f"1) straight up {args.up_mm:.0f} mm at 50 mm/s — Enter to move, Ctrl-C to stop ")
        check_move(robot, robot.move_to(up, speed=0.05, accel=0.2), "straight up")

        q1 = np.array(robot.joints())
        T = fk_frames(home_q)[-1]
        R = straight_down(matrix_to_rotvec(T[:3, :3]))
        flange = [*(T[:3, 3] + [0, 0, args.raise_mm / 1000]), *R]
        q2 = ik_near(flange, home_q, None)
        if q2 is None:
            raise SystemExit("no joint solution for the raised home — lower --raise-mm")
        q2 = np.array(q2)
        q2[5] += 2 * math.pi * round((q1[5] - q2[5]) / (2 * math.pi))   # wrist 3: the short way
        print(f"   now joints {deg(q1)}")
        print(f"2) joint move to safe home {deg(q2)}")
        print(f"   (taught home raised {args.raise_mm:.0f} mm, flange at z {flange[2] * 1000:.0f} mm, tool vertical; "
              f"wrist 1 and 2 turn about {abs(math.degrees(q2[3] - q1[3])):.0f} / "
              f"{abs(math.degrees(q2[4] - q1[4])):.0f} deg)")
        input("   Enter to move at 0.3 rad/s, Ctrl-C to stop ")
        check_move(robot, robot.move_joints(list(q2), speed=0.3), "safe home")
        p = robot.pose()
        print(f"at safe home: TCP {[round(v * 1000) for v in p[:3]]} mm. Next: pendant to Local, Move tab, "
              f"Base Z down to ~10 mm above the wax.")


if __name__ == "__main__":
    main()
