"""00 — Teach the workspace (9 steps). Receive-only: this script never moves the arm.

You move the arm; the script records where it is when you press Enter:

    1-4. LOW corners  — walk around the work area: corner 1, 2, 3, 4 in order,
                        tool touching the surface at each
    5-8. HIGH corners — the ceiling: the same four corners at the highest
                        height the tool should ever reach (straight above
                        low corner 1, 2, 3, 4)
    9.   home         — a comfortable pose above the area, inside the box,
                        where every example starts and ends (joint angles
                        are stored too)

The workspace is the largest axis-aligned box that FITS INSIDE the eight
corners — so if the table is a bit rotated against the robot or not quite
level, the box stays within what you taught, never beyond it.

How to move the arm:
    SIM   pendant tab (localhost:6080): Move tab → arrows / hold the arm icon.
          Works in Local AND Remote mode.
    REAL  hold the freedrive button on the back of the pendant and guide the
          arm by hand (supervised, hand near the e-stop).

The result is saved to workspace.json in the repo folder (git-ignored; one per
setup). All other templates load it and work in FRACTIONS of this box, so
they scale to whatever area you taught.

Usage (dev shell):
    python examples/function_templates/00_teach_workspace.py --target sim
    python examples/function_templates/00_teach_workspace.py --target sim --demo   # skip teaching: 300x300 mm box around the start pose
    python examples/function_templates/00_teach_workspace.py --target real        # lab, supervised
"""

from __future__ import annotations

import argparse

from ur5e.config import resolve_host
from ur5e.connection import RobotConnection
from ur5e.workspace import Workspace, WorkspaceError, demo_sim_workspace

STEPS = [
    ("low1",  "LOW corner 1 — a corner of the work area, tool on the surface"),
    ("low2",  "LOW corner 2 — the next corner around the area, on the surface"),
    ("low3",  "LOW corner 3 — the next corner, on the surface"),
    ("low4",  "LOW corner 4 — the last corner, on the surface"),
    ("high1", "HIGH corner 1 — above low corner 1, at the highest height the tool should reach"),
    ("high2", "HIGH corner 2 — above low corner 2, same height"),
    ("high3", "HIGH corner 3 — above low corner 3, same height"),
    ("high4", "HIGH corner 4 — above low corner 4, same height"),
    ("home",  "HOME — a comfortable pose above the area, inside the box (examples start/end here)"),
]


def fmt(p):
    return "xyz [mm] = " + str([round(v * 1000) for v in p[:3]])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", choices=["sim", "real"], default="sim")
    ap.add_argument("--out", default="workspace.json", help="file to write (default workspace.json)")
    ap.add_argument("--demo", action="store_true",
                    help="sim only: write a ready-made box around the current pose instead of teaching")
    args = ap.parse_args()

    host = resolve_host(args.target)
    print(f"connecting (receive-only) to {host} ...")
    with RobotConnection(host) as r:          # receive only — no control, no motion
        if args.demo:
            if args.target != "sim":
                raise SystemExit("--demo is for the simulator only")
            ws = demo_sim_workspace(r.receive.getActualTCPPose(), r.receive.getActualQ())
        else:
            captured = {}
            print("\nMove the arm to each point, then press Enter here. Ctrl-C aborts.\n")
            for key, text in STEPS:
                input(f"[{len(captured) + 1}/{len(STEPS)}] {text}\n      ... move the arm there, then press Enter: ")
                pose = list(r.receive.getActualTCPPose())
                captured[key] = pose
                if key == "home":
                    home_joints = list(r.receive.getActualQ())
                print(f"      recorded {fmt(pose)}\n")
            try:
                ws = Workspace.from_points(args.target,
                                           [captured[k] for k in ("low1", "low2", "low3", "low4")],
                                           [captured[k] for k in ("high1", "high2", "high3", "high4")],
                                           captured["home"], home_joints, host=host)
            except WorkspaceError as e:
                raise SystemExit(f"not saved: {e}")

    path = ws.save(args.out)
    print(ws.describe())
    print(f"\nsaved -> {path}. Next: python examples/function_templates/01_read_state.py")


if __name__ == "__main__":
    main()
