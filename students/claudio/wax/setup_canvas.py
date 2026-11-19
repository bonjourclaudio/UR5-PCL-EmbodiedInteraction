"""Guided setup for the wax canvas — run once per setup, step by step.

  1  tool: payload + TCP (same values on the pendant)
  2  connect (read-only) and check the pendant's TCP matches
  3  FREEDRIVE: tip on the wax in the BOTTOM-LEFT and TOP-LEFT inner corners (remembered)
  4  canvas geometry: drawable area = frame minus border, checked against the lab cell
  5  home, above the canvas centre (remembered)
  6  save workspace + setup file
  7  check: the arm hovers over the 4 corners of the drawable area (moves, slowly)

Bottom-left = seen from the front of the table (standing opposite the robot,
facing it): the corner nearest to you, on your left. That is how the image
is laid out (image top = far side, towards the robot).

Freedrive needs the pendant in LOCAL mode (Remote Control blocks freedrive and
the Move tab, manual 29.4.5); this script only READS positions during
steps 2-5. Step 7 moves the arm and needs REMOTE mode; the script tells you
when to switch.

Afterwards draw_wax.py picks the workspace, payload and TCP up from
wax_setup.<target>.json, so the first run needs no extra flags.

Usage:
  python students/claudio/wax/setup_canvas.py --target real
  python students/claudio/wax/setup_canvas.py --target sim          (try it in URSim: Move tab instead of freedrive)
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
from pathlib import Path

import numpy as np

from ur5e import Robot
from ur5e.config import resolve_host
from ur5e.connection import RobotConnection
from ur5e.kinematics import fk_frames
from ur5e.config import cell_profile_name
from ur5e.safety import get_profile
from ur5e.workspace import Workspace, WorkspaceError, inner_box

from geometry import (estimate_gap_mm, frame_geometry, mat_to_rotvec, rotvec_to_mat,
                      straight_down, tilt_deg)

HERE = Path(__file__).resolve().parent
TCP_TOLERANCE_MM = 8.0          # nominal FK vs. controller differ by a few mm
MAX_TILT_DEG = 15.0
JOINT_NAMES = ("base", "shoulder", "elbow", "wrist 1", "wrist 2", "wrist 3")
JOINT_WARN_DEG = 270.0          # joints turn +/-360 (363); keep 90 deg of room for the drawing moves


# ------------------------------------------------------------------ helpers --
def step(n, title):
    print(f"\n{'=' * 70}\nSTEP {n}/7  {title}\n{'=' * 70}")


def ask(prompt, default=None, cast=float):
    d = "" if default is None else f" [{default}]"
    while True:
        raw = input(f"  {prompt}{d}: ").strip()
        if not raw and default is not None:
            return default
        try:
            return cast(raw)
        except ValueError:
            print("    please enter a number")


def wait(prompt="Enter to continue"):
    input(f"  -> {prompt} ")


def yes(prompt, default=True) -> bool:
    raw = input(f"  {prompt} [{'Y/n' if default else 'y/N'}]: ").strip().lower()
    return default if not raw else raw.startswith("y")


def mm(p):
    return "[" + ", ".join(f"{v * 1000:.0f}" for v in p[:3]) + "] mm"


def record_corner(r, name, extra):
    """Freedrive to a corner, Enter -> REAL tip position (corrected for tilt and TCP difference)."""
    while True:
        wait(f"Enter when the tip touches the wax in the {name} corner")
        p = list(r.receive.getActualTCPPose())
        tilt = tilt_deg(p[3:])
        tip = np.array(p[:3]) + rotvec_to_mat(p[3:])[:, 2] * extra
        print(f"  recorded: real tip at {mm(tip)}, tool tilt {tilt:.1f} deg (corrected for)")
        if tilt > MAX_TILT_DEG:
            print(f"  !! tilted more than {MAX_TILT_DEG} deg — straighten the tool and record again")
            continue
        return tip


def migrate_saved(prev, extra):
    """Corners saved by this script; or, from an older run, the bottom-left corner
    rebuilt from the workspace file (drawable corner minus the border, gap 0)."""
    saved = dict(prev.get("corners", {}))
    ws_file = Path(prev.get("workspace", ""))
    if "bl" not in saved and prev.get("border_mm") is not None and ws_file.is_file():
        try:
            c = Workspace.load(ws_file).corners_low[0]
        except WorkspaceError:
            return saved
        b = prev["border_mm"] / 1000
        old_extra = (prev.get("tip_mm", prev["tcp"][2] * 1000) - prev["tcp"][2] * 1000) / 1000
        saved["bl"] = [c[0] - b, c[1] + b, c[2] - old_extra]
        saved["note"] = f"from {ws_file.name}, gap 0 assumed"
    return saved


def check_move(ok, what):
    if ok is False:
        raise SystemExit(f"\n!! controller refused the move ({what}) — check the pendant "
                         "(Remote Control? protective stop?)")


# --------------------------------------------------------------------- main --
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", choices=("sim", "real"), default="sim")
    ap.add_argument("--host", default=None)
    ap.add_argument("--out", default=None, help="workspace file (default workspace.wax-<target>.json)")
    args = ap.parse_args()
    if args.target == "real" and not (args.host or os.environ.get("UR_HOST")):
        raise SystemExit("start the shell with -e UR_HOST=192.168.1.11 or pass --host")
    host = args.host or resolve_host(args.target)
    setup_file = HERE / f"wax_setup.{args.target}.json"
    prev = json.loads(setup_file.read_text()) if setup_file.exists() else {}
    if prev:
        print(f"previous setup found ({setup_file.name}) — its values are the defaults, Enter keeps them")

    # 1 ------------------------------------------------------------- tool --
    step(1, "tool: payload and TCP")
    print("  Measured values only (scale, calipers). TCP = tip position relative to the flange.")
    payload = ask("tool mass incl. mount, kg", prev.get("payload_kg", 0.3 if args.target == "real" else 0.0))
    tcp_prev = prev.get("tcp", [0, 0, 0.085 if args.target == "real" else 0.0, 0, 0, 0])
    tz = ask("TCP Z as set on the pendant (along the tool axis), mm", round(tcp_prev[2] * 1000, 1))
    real = ask("REAL tip distance from the flange face (measured), mm",
               prev.get("tip_mm", round(tcp_prev[2] * 1000, 1)))
    tx = ask("tip sideways offset x, mm", round(tcp_prev[0] * 1000, 1))
    ty = ask("tip sideways offset y, mm", round(tcp_prev[1] * 1000, 1))
    tcp = [tx / 1000, ty / 1000, tz / 1000, 0.0, 0.0, 0.0]
    extra = (real - tz) / 1000          # tip lies this far beyond the TCP, along the tool axis
    if abs(extra) > 0.0005:
        print(f"\n  NOTE: the TCP is {real - tz:+.1f} mm off the real tip along the tool axis.")
        print("  All heights are taught relative to the TCP, so drawing works; the script corrects")
        print("  the corner for this when it straightens the tool. The TCP point is "
              f"{real - tz:.0f} mm above the tip: anything else using absolute heights (place_frame.py)")
        print("  is shifted by that much.")
    print(f"\n  On the pendant now (Local mode): Installation -> General -> TCP:")
    print(f"     X {tx} mm  Y {ty} mm  Z {tz} mm  RX RY RZ 0  -> set as default TCP")
    print(f"  Installation -> General -> Payload: {payload} kg.  Then save the installation.")
    wait("Enter when the pendant shows this TCP")

    # 2 ------------------------------------------------- connect + check --
    step(2, "connect (read-only) and check the TCP")
    print(f"  pendant in LOCAL mode (freedrive needs it). Connecting to {host} ...")
    with RobotConnection(host) as r:
        while True:
            q = r.receive.getActualQ()
            pose = r.receive.getActualTCPPose()
            T = fk_frames(q)[-1]
            expect = T[:3, :3] @ np.array(tcp[:3]) + T[:3, 3]
            err = np.linalg.norm(expect - np.array(pose[:3])) * 1000
            print(f"  controller TCP {mm(pose)}, expected from your TCP {mm(expect)}: off by {err:.1f} mm")
            if err <= TCP_TOLERANCE_MM:
                print("  TCP OK")
                break
            print("  !! the pendant's active TCP is not the one you entered (or not saved as default).")
            if not yes("re-check now (n = continue anyway)?"):
                break

        # 3 ------------------------------------------------------ corners --
        step(3, "FREEDRIVE: two corners of the frame (bottom-left, top-left)")
        saved = migrate_saved(prev, extra)
        print("  Standing opposite the robot, facing it: BOTTOM-LEFT = nearest corner on your left,")
        print("  TOP-LEFT = the far corner on your left (towards the robot). Tip ON the wax, as close")
        print("  to both walls as the tool allows; let go of freedrive and wait a second before Enter.")
        tips = {}
        for key, name in (("bl", "BOTTOM-LEFT"), ("tl", "TOP-LEFT")):
            if key in saved and yes(f"use the saved {name} corner, real tip at {mm(saved[key])}"
                                    f"{' (' + saved['note'] + ')' if saved.get('note') and key == 'bl' else ''}?"):
                tips[key] = np.array(saved[key])
            else:
                tips[key] = record_corner(r, name, extra)

        # 4 ------------------------------------------------------ geometry --
        step(4, "canvas geometry")
        frame = ask("inner frame size (square), mm", prev.get("frame_mm", 180.0))
        border = ask("free border inside the frame, mm", prev.get("border_mm", 20.0))
        wall = ask("frame wall height above the wax, mm", prev.get("wall_mm", 15.0))
        bl_t, tl_t = tips["bl"], tips["tl"]
        if np.linalg.norm(tl_t[:2] - bl_t[:2]) < 0.05:
            raise SystemExit("the two corners are less than 50 mm apart — corners mixed up? rerun")
        tip_dist = float(np.linalg.norm(tl_t[:2] - bl_t[:2])) * 1000
        est_gap = estimate_gap_mm(bl_t, tl_t, frame)
        print(f"  the two tips are {tip_dist:.1f} mm apart; the frame is {frame:.0f} mm -> the tip stayed about "
              f"{est_gap:.1f} mm from the walls")
        print("  (the tool head usually stops the tip before it reaches the wall; the right side is not")
        print("   measured, so this gap matters for where the right edge goes)")
        gap = ask("gap between the tip and the walls when the corners were recorded, mm", est_gap)
        top = ask("workspace ceiling above the wax, mm", prev.get("ceiling_mm", 80.0))
        if top < wall + 40:
            raise SystemExit(f"ceiling must be at least {wall + 40:.0f} mm (walls + 30 mm clearing + margin)")
        width_mm = prev.get("frame_width_mm") or frame
        cut_off = prev.get("cut_offset_mm", 0.0)
        if width_mm != frame or cut_off:
            print(f"  from the setup file (canvas_tool.py): frame width {width_mm:.0f} mm, "
                  f"cutting level {cut_off:+.1f} mm against the taught wax height")
        try:
            geo = frame_geometry(bl_t, tl_t, frame_mm=frame, border_mm=border, gap_mm=gap,
                                 extra_m=extra, ceiling_mm=top, width_mm=width_mm, cut_offset_mm=cut_off)
        except ValueError as e:
            raise SystemExit(str(e))
        corners, zs, zt, rot = geo["drawable"], geo["zs"], geo["zt"], geo["rot_deg"]
        depth, level = geo["depth_mm"], geo["level_mm"]
        print(f"  frame measured: {depth:.0f} mm deep (bottom -> top), rotated {rot:+.1f} deg "
              f"against the robot's x axis")
        if abs(depth - frame) > 5:
            print(f"  !! measured depth {depth:.0f} mm differs from the frame size {frame:.0f} mm — "
                  f"using the measured depth; check the corners / frame size")
        print(f"  wax height: bottom-left {bl_t[2] * 1000:.1f}, top-left {tl_t[2] * 1000:.1f} mm "
              f"(difference {level:.1f} mm)")
        if level > 0.7:
            print("  !! the canvas is not level — with 1 mm of wax the tip may miss or dig in; shim it")
        box = inner_box([[x, y, z] for z in (zs, zt) for x, y in corners])
        bw, bd = (box.y[1] - box.y[0]) * 1000, (box.x[1] - box.x[0]) * 1000
        print(f"  drawable area (inside the border): {width_mm - 2 * border:.0f} x {depth - 2 * border:.0f} mm; "
              f"largest box the robot can use inside it: {bw:.0f} x {bd:.0f} mm")
        if abs(rot) > 1.0:
            print(f"  (rotating the frame {-rot:+.1f} deg to line up with the robot would give the full area)")
        if args.target == "real":
            cell = get_profile(cell_profile_name("real")).workspace
            bad = [f"{n} {mm([x, y, z])}" for z in (zs, zt) for n, (x, y) in zip(("BL", "TL", "TR", "BR"), corners)
                   if not cell.contains([x, y, z])]
            if bad:
                print(f"  !! outside the lab cell box (x {cell.x[0]*1000:.0f}..{cell.x[1]*1000:.0f}, "
                      f"y {cell.y[0]*1000:.0f}..{cell.y[1]*1000:.0f}, z {cell.z[0]*1000:.0f}..{cell.z[1]*1000:.0f} mm):")
                for line in bad:
                    print(f"     {line}")
                if zs < cell.z[0]:
                    print("     wax surface below the cell floor -> raise the frame on a board")
                raise SystemExit("move the frame or lower the ceiling, then rerun")
            print("  inside the lab cell box: OK")

        # 5 ---------------------------------------------------------- home --
        step(5, "home position")
        cx, cy = (box.x[0] + box.x[1]) / 2, (box.y[0] + box.y[1]) / 2
        lo_h, hi_h = wall + 35, top - 5
        sh = prev.get("home")
        if not sh and Path(prev.get("workspace", "")).is_file() and "tcp" in prev:
            try:                                          # older setup file: home from the workspace file
                w0 = Workspace.load(prev["workspace"])
                sh = {"pose": w0.home, "q": w0.home_joints, "tcp": prev["tcp"]}
            except WorkspaceError:
                sh = None
        candidates = []
        if sh and sh.get("tcp") == tcp and yes(f"use the saved home (joints {[round(math.degrees(a)) for a in sh['q']]} deg)?"):
            candidates.append((list(sh["pose"]), list(sh["q"])))
        else:
            print(f"  Freedrive (or Move tab) the tool above the canvas centre (x {cx * 1000:.0f}, "
                  f"y {cy * 1000:.0f} mm), {lo_h:.0f}..{hi_h:.0f} mm above the wax, tool roughly vertical.")
        while True:
            if candidates:
                home, home_q = candidates.pop()
            else:
                wait("Enter when the tool is there")
                home = list(r.receive.getActualTCPPose())
                home_q = list(r.receive.getActualQ())
            h = (home[2] - zs) * 1000
            inside = box.contains(home[:3], tol=0.001) or (box.x[0] <= home[0] <= box.x[1] and box.y[0] <= home[1] <= box.y[1])
            print(f"  home {mm(home)}, {h:.0f} mm above the wax, over the usable area: {inside}")
            home_tilt = tilt_deg(home[3:])
            near_limit = [(n, math.degrees(a)) for n, a in zip(JOINT_NAMES, home_q)
                          if abs(math.degrees(a)) > JOINT_WARN_DEG]
            if near_limit:
                for n, deg in near_limit:
                    print(f"  !! {n} at {deg:.0f} deg — only {363 - abs(deg):.0f} deg from its limit; drawing moves "
                          f"would run into it")
                print("     unwind it on the pendant (Local mode): Move tab -> tap the joint angle -> enter the "
                      "value +/-360, hold 'Move' — the tool only spins about its own axis. Then record again.")
            elif home_tilt > MAX_TILT_DEG:
                print(f"  !! tool tilted {home_tilt:.0f} deg — straighten it and record again")
            elif not inside:
                print("  !! not above the usable area — move it over the canvas and record again")
            elif not lo_h <= h <= hi_h:
                print(f"  !! height must be {lo_h:.0f}..{hi_h:.0f} mm above the wax — record again")
            else:
                break

    # 6 ---------------------------------------------------------------- save --
    step(6, "save")
    # drawing orientation: vertical, with HOME's heading -> moves from home barely turn wrist 3
    down = straight_down(home[3:])
    print(f"  drawing orientation: tool exactly vertical, heading as at home")
    low = [[x, y, zs, *down] for x, y in corners]
    high = [[x, y, zt, *down] for x, y in corners]
    try:
        ws = Workspace.from_points(args.target, low, high, home, home_q, host=host,
                                   note=f"wax canvas {frame:.0f} mm frame, {border:.0f} mm border")
    except WorkspaceError as e:
        raise SystemExit(f"not saved: {e}")
    out = Path(args.out or f"workspace.wax-{args.target}.json")
    if out.exists():
        shutil.copy(out, out.with_suffix(".json.bak"))
        print(f"  previous {out} kept as {out.with_suffix('.json.bak')}")
    ws.save(out)
    print(ws.describe())
    setup = {"target": args.target, "workspace": str(out), "payload_kg": payload, "tcp": tcp,
             "tip_mm": real,
             "frame_mm": frame, "border_mm": border, "wall_mm": wall, "ceiling_mm": top,
             "clear_mm": wall + 15, "frame_width_mm": width_mm, "cut_offset_mm": cut_off,
             "corners": {"bl": [float(v) for v in bl_t], "tl": [float(v) for v in tl_t], "gap_mm": gap},
             "home": {"pose": list(home), "q": list(home_q), "tcp": tcp}}
    setup_file.write_text(json.dumps(setup, indent=2))
    print(f"  saved {out} and {setup_file.relative_to(Path.cwd()) if setup_file.is_relative_to(Path.cwd()) else setup_file}")

    # 7 --------------------------------------------------------------- check --
    step(7, "check: hover over the 4 corners (the arm MOVES)")
    if not yes("run the check now?"):
        print("  skipped. Run setup_canvas.py again later, or go to the first run below.")
    else:
        print("  Switch the pendant to REMOTE control. Speed slider down, hand near the e-stop.")
        wait("Enter when in Remote Control")
        clear = (wall + 15) / top                     # fraction of the box height
        near = 5 / top
        kw = dict(allow_real=True) if args.target == "real" else {}
        with Robot(args.target, host=host, payload_kg=payload, tcp=tcp, profile=ws, **kw) as robot:
            print("  home")
            check_move(robot.move_joints(ws.home_joints, speed=0.3), "home")
            for name, (u, v) in zip(("bottom-left", "top-left", "top-right", "bottom-right"),
                                    ((0, 1), (1, 1), (1, 0), (0, 0))):
                print(f"  -> {name} corner, {wall + 15:.0f} mm above the wax")
                check_move(robot.move_to(ws.pose(u, v, clear), speed=0.03, accel=0.2), name)
                print("     down to 5 mm above the wax")
                check_move(robot.move_to(ws.pose(u, v, near), speed=0.01, accel=0.2), name)
                wait(f"check: tip {border:.0f} mm from both walls, 5 mm above the wax? Enter")
                check_move(robot.move_to(ws.pose(u, v, clear), speed=0.01, accel=0.2), name)
            print("  home")
            check_move(robot.move_joints(ws.home_joints, speed=0.3), "home")
        print("  If a corner was off: rerun this script (the values you typed are remembered).")

    print(f"""
{'=' * 70}
DONE. First runs (pendant in Remote Control):

  preview only:
    python students/claudio/wax/draw_wax.py --target {args.target} --size-mm 60 --dry-run
  cold, 10 mm in the air:
    python students/claudio/wax/draw_wax.py --target {args.target} --size-mm 60 --no-heat --hover-mm 10
  cold, touching:
    python students/claudio/wax/draw_wax.py --target {args.target} --size-mm 60 --no-heat
""")


if __name__ == "__main__":
    main()
