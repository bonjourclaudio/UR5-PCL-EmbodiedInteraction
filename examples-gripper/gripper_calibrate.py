"""Gripper calibration — teach the servo angles for CLOSED (min) and OPEN (max).

STATUS: experimental / internal. Real gripper only (needs the TCI link; no
robot motion involved — the arm can stay where it is, but power the tool).

HOW IT WORKS
    1. The servo is set limp (#0L) so you can move the jaws by hand.
    2. Push the jaws TOGETHER until they touch -> Enter  -> firmware stores
       the current angle as CLOSED (0 %).
    3. Pull the jaws fully APART until the linkage stops -> Enter -> stored as
       OPEN (100 %).  (Design stroke: 52 deg of servo travel, 0 -> 61 mm.)
    4. The gripper then runs open -> closed -> open once to verify.
    Values live in the T-CAN485's flash (NVS) and survive power cycles; every
    status line reports cal=1 and closed=/open= in tenths of a degree.

Alternative without this script: set the LSS origin offset so that the
"links straight ahead" pose reads 0.0 deg (#0CO<offset>), then the firmware's
geometric defaults (-20.0 / +32.0 deg) already match the CAD.

USAGE
    python examples-gripper/gripper_calibrate.py --host 192.168.1.11
    python examples-gripper/gripper_calibrate.py --host 192.168.1.11 --reset
"""

from __future__ import annotations

import argparse
import os
import time

from gripper_link import Gripper, GripperError


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=os.environ.get("UR_HOST"), help="robot IP (TCI forwarder)")
    ap.add_argument("--reset", action="store_true", help="drop the stored calibration (back to geometric defaults)")
    ap.add_argument("--force", type=int, default=None, help="also store a new holding-current limit (mA)")
    ap.add_argument("--speed", type=int, default=None, help="also store a new speed limit (deg/s)")
    ap.add_argument("--start", type=float, default=None, metavar="DEG",
                    help="open-loop: first angle to move to (default: don't move until you type one)")
    args = ap.parse_args()
    if not args.host:
        raise SystemExit("need --host or UR_HOST")

    with Gripper(args.host) as g:
        if args.reset:
            print("reset ->", g.calibrate("reset"))
            return
        if args.force is not None:
            g.set_force(args.force)
        if args.speed is not None:
            g.set_speed(args.speed)

        if g.open_loop:
            # no servo Tx line: find the limits by stepping the raw angle and looking
            print("OPEN LOOP (servo Tx not wired): step the raw servo angle until the jaws just touch,\n"
                  "then from there until they are fully open. Nothing moves until you type an angle;\n"
                  "nudges count from 0 deg if no angle was given yet.\n"
                  "   +2 / -2   nudge by N degrees        =-30   jump to an absolute angle\n"
                  "   ok        accept this position       (if the board stops answering, you are\n"
                  "   pushing against a stop: the supply sagged and the board rebooted — back off)")
            # The servo is NOT moved until you type the first angle, and the "open"
            # phase starts from the accepted "closed" angle, so the jaws never jump
            # into a stop on their own. --start sets the first angle if you know it.
            limits = {}
            ang = args.start                     # None = unknown, nothing sent yet
            for name in ("closed", "open"):
                print(f"-- {name}: {'start by typing an absolute angle (=N) or a nudge (+N/-N from 0)' if ang is None else f'starting at {ang:+.1f} deg'}")
                moved_to = None
                while True:
                    if ang is not None and moved_to != ang:
                        try:
                            g.move_raw(int(round(ang * 10)))
                            moved_to = ang
                        except GripperError as exc:
                            print(f"   !! {exc}\n   -> the servo most likely stalled at {ang:+.1f} deg; "
                                  "move back the other way")
                    where = f"{ang:+.1f} deg" if ang is not None else "unknown (not moved yet)"
                    ans = input(f"  {name}: now at {where}  [+N / -N / =N / ok]: ").strip().lower()
                    if ans == "ok":
                        if ang is None:
                            print("   ? move it first")
                            continue
                        limits[name] = int(round(ang * 10))
                        break
                    try:
                        if ans.startswith("="):
                            ang = float(ans[1:])
                        elif ans[:1] in "+-" and len(ans) > 1:
                            ang = (ang or 0.0) + float(ans)
                        else:
                            ang = float(ans)
                    except ValueError:
                        print("   ? use +N, -N, =N or ok")
            if limits["closed"] == limits["open"]:
                raise SystemExit("closed and open are the same angle — nothing stored")
            st = g.set_limits(limits["closed"], limits["open"])
            print(f"  stored closed {st['closed'] / 10:.1f} deg, open {st['open'] / 10:.1f} deg")
        else:
            print("servo limp — move the jaws by hand.")
            g.limp()
            input("push the jaws TOGETHER until they touch, then press Enter ...")
            st = g.calibrate("min")
            print(f"  closed stored at {st['closed'] / 10:.1f} deg")
            input("pull the jaws fully APART, then press Enter ...")
            st = g.calibrate("max")
            print(f"  open stored at {st['open'] / 10:.1f} deg  (stroke {(st['open'] - st['closed']) / 10:.1f} deg)")

        print("verify: open -> close -> open")
        g.open(); time.sleep(0.5)
        st = g.close(); print("  closed:", st.get("pct"), "%"); time.sleep(0.5)
        st = g.open(); print("  open:", st.get("pct"), "%")
        print("done:", g.status())


if __name__ == "__main__":
    main()
