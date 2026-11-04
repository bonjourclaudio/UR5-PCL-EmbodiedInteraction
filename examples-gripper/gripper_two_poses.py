"""Gripper demo — two poses, open/close at each, dwell a few seconds.

STATUS: experimental — reference example for the lab gripper (LSS servo + T-CAN485). Not
part of the validated framework. Motion goes through the normal Robot API; the gripper
link is REAL-ARM ONLY (TCI, TCP <robot>:54321) and is mocked in the simulator.

SEQUENCE
    connect robot + gripper
    gripper.open()                      <- start of program: known state, jaws open
    move_joints(home)
    for pose in (A, B):
        move_to(pose)
        gripper.close(force)   wait DWELL
        gripper.open()         wait DWELL
    move_joints(home)

POSES are defined ONCE at the top as fractions of the taught workspace box
(u along x, v along y, w = height, 0..1) — edit POSE_A / POSE_B.

ASSUMPTIONS (flagged per project rules)
    - Tool = gripper v2: pass --payload 0.35 --tcp 0 0 0.136 0 0 0 with --target real
      (mass to be weighed after printing; TCP = jaw centre on the flange axis).
    - The taught workspace orientation points the tool (= fingers) down; w = 0.3
      keeps the jaws ~30 % of the box height above the surface. Nothing is picked
      up here — the jaws just open/close in free air (or around an object you hold).
    - Gripper calibrated once with gripper_calibrate.py (otherwise geometric
      defaults, which assume the LSS origin offset was set — see Documentation/15).

USAGE
    python examples-gripper/gripper_two_poses.py                      # URSim + mock gripper
    python examples-gripper/gripper_two_poses.py --target real \\
        --payload 0.35 --tcp 0 0 0.136 0 0 0                                   # lab: real arm + real gripper
    ... --gripper mock --target real      # real arm, gripper simulated (e.g. gripper not wired yet)

RELATIVE MODE (no calibration, no servo feedback — the quick demo):
    power-cycle the gripper (servo boots limp), set the jaws MID-STROKE BY HAND, then
    ... --relative 10      # "close" = +10 deg from there, "open" = -10 deg back; use -10 if it
                           # opens first. Nothing is sent at program start, so the hand-set
                           # position is the reference for the whole run.
"""

from __future__ import annotations

import time

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs
from gripper_link import make_gripper

# --- the two poses, as fractions of the workspace box (u, v, w) ---------------
POSE_A = (0.3, 0.5, 0.3)
POSE_B = (0.7, 0.5, 0.3)

DWELL_S = 3.0          # seconds to wait after each open / close
GRIP_FORCE_MA = 300    # LSS holding-current limit while closing (grip force)
SPEED = 0.10           # m/s for the moves between the poses


def main() -> None:
    ap = argument_parser(__doc__)
    ap.add_argument("--gripper", choices=("auto", "tci", "mock"), default="auto",
                    help="auto = tci on --target real, mock in the sim (default)")
    ap.add_argument("--dwell", type=float, default=DWELL_S, help="seconds to wait at each state")
    ap.add_argument("--relative", type=float, default=None, metavar="DEG",
                    help="uncalibrated demo: close = +DEG relative to the hand-set position, open = -DEG")
    args = ap.parse_args()

    def do_close(g):
        if args.relative is not None:
            return g.move_relative(+args.relative)
        return g.close(force_mA=GRIP_FORCE_MA)

    def do_open(g):
        if args.relative is not None:
            return g.move_relative(-args.relative)
        return g.open()

    ws = Workspace.load(args.workspace)
    kind = args.gripper if args.gripper != "auto" else ("tci" if args.target == "real" else "mock")

    with Robot(**robot_kwargs(args), profile=ws) as robot, \
            make_gripper(kind, robot.host) as gripper:

        if args.relative is None:
            print("start: open the gripper so we begin from a known state")
            gripper.open()
        else:
            print(f"relative mode: jaws stay where you set them; close = {args.relative:+.0f} deg, "
                  f"open = {-args.relative:+.0f} deg")
        robot.move_joints(ws.home_joints)

        for name, uvw in (("A", POSE_A), ("B", POSE_B)):
            pose = ws.pose(*uvw)
            print(f"pose {name} {uvw} -> {[round(v, 3) for v in pose[:3]]} m")
            robot.move_to(pose, speed=SPEED)

            st = do_close(gripper)
            print(f"  closed  (jaws at {st.get('pct')} %, {st.get('mA')} mA) — waiting {args.dwell:.0f} s")
            time.sleep(args.dwell)

            st = do_open(gripper)
            print(f"  open    (jaws at {st.get('pct')} %) — waiting {args.dwell:.0f} s")
            time.sleep(args.dwell)

        robot.move_joints(ws.home_joints)
        print("done — gripper left open." if args.relative is None else "done — jaws back at the start position.")


if __name__ == "__main__":
    main()
