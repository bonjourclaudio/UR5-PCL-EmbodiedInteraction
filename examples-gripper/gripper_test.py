"""Gripper link test — talk to the gripper over the robot's RS485 (TCI), no arm motion.

STATUS: experimental / internal. Real arm only (URSim has no tool RS485).

Checks the whole path host -> TCP <robot>:54321 -> tool RS485 -> T-CAN485 -> LSS:
prints the status line, then open / close / 50 % with a pause in between.

Before running: PolyScope > Installation > General > Tool I/O: 12 V (HT1: Dual
Pin Power), Communication Interface ON, 115200 baud, parity none, 1 stop bit;
the rs485 URCap (Tool Communication Forwarder) installed and running.

USAGE (dev container, robot reachable):
    python examples-gripper/gripper_test.py --host 192.168.1.11
    python examples-gripper/gripper_test.py --host 192.168.1.11 --cycles 3 --force 300
"""

from __future__ import annotations

import argparse
import os
import time

from gripper_link import Gripper, GripperError


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=os.environ.get("UR_HOST"), help="robot IP (TCI forwarder on :54321)")
    ap.add_argument("--cycles", type=int, default=1, help="open/close cycles (default 1)")
    ap.add_argument("--force", type=int, default=None, help="holding-current limit in mA to store (e.g. 300)")
    ap.add_argument("--speed", type=int, default=None, help="speed limit in deg/s to store (e.g. 90)")
    ap.add_argument("--pause", type=float, default=1.5, help="seconds between steps")
    args = ap.parse_args()
    if not args.host:
        raise SystemExit("need --host or UR_HOST")

    print(f"connecting to {args.host}:54321 ...")
    try:
        with Gripper(args.host) as g:
            print("status:", g.status())
            if args.force is not None:
                print("force ->", g.set_force(args.force).get("f"), "mA")
            if args.speed is not None:
                print("speed ->", g.set_speed(args.speed).get("v"), "deg/s")
            for i in range(args.cycles):
                print(f"cycle {i + 1}: open");   g.open();     time.sleep(args.pause)
                print("         close");         g.close();    time.sleep(args.pause)
                print("         50 %");          g.goto(50);   time.sleep(args.pause)
            print("open and done");               g.open()
            print("final:", g.status())
    except (GripperError, OSError) as exc:
        raise SystemExit(f"FAILED: {exc}\n"
                         "  no connection -> rs485 URCap not running / wrong host\n"
                         "  no reply      -> Tool I/O communication interface off or baud mismatch, "
                         "A/B swapped, board unpowered (tool voltage 12 V?)")


if __name__ == "__main__":
    main()
