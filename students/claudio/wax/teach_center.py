"""Teach the canvas from ONE point: the tool tip held above the canvas centre.

Hold the tip --above-mm (default 10) above the wax, centred over the canvas, tool vertical
(pendant Move tab, Base Z). This script READS that position (no motion) and builds:

  wax surface   = that height - above-mm             (in TCP heights, so the tool length
  cutting level = surface - depth-mm  (default 2)     does not matter — only the TCP must
  drawable area = centre +/- (frame/2 - border)        be the same here and in every run)
  travel        = travel-mm above the surface between marks (draw_wax --lift-mm)
  home          = home-mm above the surface at the centre, same wrist configuration

and saves workspace.wax-real.json + wax_setup.real.json (previous versions kept as .bak),
the same way setup_canvas.py does. canvas_tool.py can fine-tune it afterwards.
Check with check_corners.py before engraving.

Usage (TCP and payload as ACTIVE ON THE PENDANT):
  python students/claudio/wax/teach_center.py --target real --payload 0.35 --tcp 0 0 0 0 0 0
"""

from __future__ import annotations

import json
import math
import shutil
from pathlib import Path

import numpy as np

from ur5e.cli import argument_parser
from ur5e.config import cell_profile_name, resolve_host
from ur5e.connection import RobotConnection
from ur5e.kinematics import fk_frames, ik_near
from ur5e.safety import get_profile
from ur5e.workspace import Workspace, WorkspaceError

from draw_wax import HERE
from geometry import straight_down, tilt_deg

JOINT_NAMES = ("base", "shoulder", "elbow", "wrist 1", "wrist 2", "wrist 3")


def main() -> None:
    ap = argument_parser(__doc__)
    ap.add_argument("--above-mm", type=float, default=10.0, help="tip height above the wax while reading")
    ap.add_argument("--depth-mm", type=float, default=2.0, help="engrave this far below the surface")
    ap.add_argument("--travel-mm", type=float, default=10.0, help="travel height above the surface between marks")
    ap.add_argument("--frame-mm", type=float, default=180.0, help="inner frame size, front-back (x)")
    ap.add_argument("--width-mm", type=float, default=None, help="inner frame size, left-right (y); default = frame")
    ap.add_argument("--border-mm", type=float, default=15.0, help="free border inside the frame")
    ap.add_argument("--wall-mm", type=float, default=7.0, help="frame wall height above the wax")
    ap.add_argument("--ceiling-mm", type=float, default=70.0, help="workspace top, above the cutting level")
    ap.add_argument("--home-mm", type=float, default=50.0, help="home height above the surface")
    args = ap.parse_args()
    width = args.width_mm or args.frame_mm
    tcp = list(args.tcp)
    if args.target == "real" and not args.payload:
        raise SystemExit("pass --payload (kg) and --tcp as set on the pendant")
    host = args.host or resolve_host(args.target)

    with RobotConnection(host) as r:                  # receive only — nothing moves
        pose = list(r.receive.getActualTCPPose())
        q = list(r.receive.getActualQ())
    T = fk_frames(q)[-1]
    expect = T[:3, :3] @ np.array(tcp[:3]) + T[:3, 3]
    err = float(np.linalg.norm(expect - np.array(pose[:3]))) * 1000
    tilt = tilt_deg(pose[3:])
    print(f"read: TCP {[round(v * 1000, 1) for v in pose[:3]]} mm, tilt {tilt:.1f} deg, "
          f"pendant TCP vs --tcp off by {err:.1f} mm")
    if err > 8:
        raise SystemExit("!! the pendant's active TCP is not the --tcp you passed — fix one of them")
    if tilt > 3:
        raise SystemExit("!! tool tilted more than 3 deg — straighten it (Move tab) and run again")

    cx, cy = pose[0], pose[1]
    surface = pose[2] - args.above_mm / 1000
    zs = surface - args.depth_mm / 1000
    zt = zs + args.ceiling_mm / 1000
    dx, dy = args.frame_mm / 2000 - args.border_mm / 1000, width / 2000 - args.border_mm / 1000
    if dx < 0.01 or dy < 0.01:
        raise SystemExit("frame minus border leaves no drawable area")
    orient = straight_down(pose[3:])
    corners = [(cx - dx, cy + dy), (cx + dx, cy + dy), (cx + dx, cy - dy), (cx - dx, cy - dy)]  # BL TL TR BR
    if args.ceiling_mm < args.wall_mm + 40 or args.travel_mm + args.depth_mm > args.ceiling_mm - 5:
        raise SystemExit("ceiling too low for the walls / travel height")
    if not args.travel_mm > args.wall_mm:
        print(f"!! travel height {args.travel_mm} mm is not above the {args.wall_mm} mm walls — fine inside the "
              f"border, but entering/leaving uses the clearing height (walls + 15 mm)")

    home = [cx, cy, surface + args.home_mm / 1000, *orient]
    home_q = ik_near(home, q, tcp)
    if home_q is None:
        raise SystemExit("no joint solution for home — lower --home-mm")
    far = [(n, math.degrees(a)) for n, a in zip(JOINT_NAMES, home_q) if abs(math.degrees(a)) > 270]
    if far:
        raise SystemExit(f"!! home joint near its limit: {far} — unwind on the pendant and read again")

    cell = get_profile(cell_profile_name(args.target)).workspace
    bad = [(x, y, z) for z in (zs, zt) for x, y in corners if not cell.contains([x, y, z])]
    if bad:
        raise SystemExit(f"!! corners outside the '{cell_profile_name(args.target)}' cell: {bad}")
    low = [[x, y, zs, *orient] for x, y in corners]
    high = [[x, y, zt, *orient] for x, y in corners]
    try:
        ws = Workspace.from_points(args.target, low, high, home, home_q, host=host,
                                   note=f"wax canvas from centre {args.frame_mm:.0f}x{width:.0f} mm, "
                                        f"border {args.border_mm:.0f}, depth {args.depth_mm:.1f}")
    except WorkspaceError as e:
        raise SystemExit(f"not saved: {e}")

    setup_file = HERE / f"wax_setup.{args.target}.json"
    s = json.loads(setup_file.read_text()) if setup_file.exists() else {}
    out = Path(s.get("workspace") or f"workspace.wax-{args.target}.json")
    print(f"\nsurface   TCP z {surface * 1000:.1f} mm (tip {args.above_mm:.0f} mm below the reading)")
    print(f"cutting   TCP z {zs * 1000:.1f} mm ({args.depth_mm:.1f} mm into the wax) — the workspace stops here")
    print(f"travel    {args.travel_mm:.0f} mm above the surface ({args.travel_mm + args.depth_mm:.0f} mm plunge per mark)")
    print(f"drawable  {2 * dx * 1000:.0f} x {2 * dy * 1000:.0f} mm around x {cx * 1000:.1f}, y {cy * 1000:.1f} mm "
          f"(frame {args.frame_mm:.0f} x {width:.0f}, border {args.border_mm:.0f})")
    print(f"home      {args.home_mm:.0f} mm above the surface, joints {[round(math.degrees(a), 1) for a in home_q]}")
    if input(f"\nsave {out} and {setup_file.name}? [y/N] ").strip().lower() != "y":
        raise SystemExit("not saved.")
    for f in (out, setup_file):
        if f.exists():
            shutil.copy(f, f.with_suffix(".json.bak"))
    ws.save(out)
    # canvas_tool / setup_canvas view: TCP heights, corners = inner frame corners at the surface, gap 0
    fbl = [cx - args.frame_mm / 2000, cy + width / 2000, surface]
    ftl = [cx + args.frame_mm / 2000, cy + width / 2000, surface]
    s.update({"target": args.target, "workspace": str(out), "payload_kg": args.payload, "tcp": tcp,
              "tip_mm": tcp[2] * 1000, "frame_mm": args.frame_mm, "frame_width_mm": width,
              "cut_offset_mm": -args.depth_mm, "border_mm": args.border_mm, "wall_mm": args.wall_mm,
              "ceiling_mm": args.ceiling_mm, "clear_mm": args.wall_mm + 15,
              "corners": {"bl": fbl, "tl": ftl, "gap_mm": 0.0},
              "home": {"pose": home, "q": list(home_q), "tcp": tcp}})
    s.setdefault("draw", {})["lift_mm"] = args.travel_mm + args.depth_mm
    setup_file.write_text(json.dumps(s, indent=2))
    print(ws.describe())
    print(f"\nsaved. Next: python students/claudio/wax/check_corners.py --target real "
          f"--payload {args.payload} --tcp {' '.join(f'{v:g}' for v in tcp)}")


if __name__ == "__main__":
    main()
