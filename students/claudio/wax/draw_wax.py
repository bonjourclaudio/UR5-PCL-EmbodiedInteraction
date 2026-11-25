"""Wax engraving: draw an image into a wax canvas with a heated tool.

Pipeline:  image (webcam snapshot in inbox/)  ->  marks (imaging.py)
           ->  fitted into the taught workspace  ->  home, heat up, engrave
           mark by mark, home  —  while HeaterGuard (heater.py) holds the
           temperature and cuts the heater at --max-temp.

Styles:
  --style dots   (default) stipple: error-diffused dots on a --dot-mm grid;
                 per dot: travel above -> plunge -> melt -> lift. The melt time
                 goes from --dwell-min (light) to --dwell-max (dark) = dot size.
  --style hatch  straight lines through the dark parts, every --hatch-mm;
                 per line: travel -> plunge -> --dwell -> engrave to end -> lift.

Workspace: teach the LOW corners with the tool tip at engraving depth in the
wax (w = 0 is where the tip cuts) and the HIGH corners at least --lift-mm
above that. The image is fitted into the box with --margin-mm, keeping its
aspect ratio. Default orientation: image top = far side (+X, towards the
robot), image left = +Y; fix with --rotate / --mirror.
No blending: the arm stops at every pose.

Usage:
  python students/claudio/wax/draw_wax.py --demo-image --dry-run
  python students/claudio/wax/draw_wax.py --demo-image --size-mm 80
  python students/claudio/wax/draw_wax.py --dot-mm 2 --gamma 1.3
  python students/claudio/wax/draw_wax.py --style hatch --hatch-mm 3
  python students/claudio/wax/draw_wax.py --sim-overheat

After setup_canvas.py, the workspace, payload and TCP come from
wax_setup.<target>.json (flags you pass still win).
"""

from __future__ import annotations

import json
import math
import time
from contextlib import nullcontext
from pathlib import Path

import numpy as np

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs

from heater import FakeHotend, HeaterGuard
from imaging import demo_image, dot_marks, hatch_strokes, load_gray, newest_image, write_svg

HERE = Path(__file__).resolve().parent
MOVE_OVERHEAD_S = 0.12     # per-move stop/start cost, for the time estimate
JOB_OVERHEAD_S = 20.0      # home, enter, leave, home
# fitted to 3 real runs: 33 dots 2.7 min (lift 12), 440 dots 8.8 min (lift 4), 440 dots 11.8 min (lift 7)

ROBOT_MODES = {-1: "NO_CONTROLLER", 0: "DISCONNECTED", 1: "CONFIRM_SAFETY", 2: "BOOTING",
               3: "POWER_OFF", 4: "POWER_ON", 5: "IDLE", 6: "BACKDRIVE", 7: "RUNNING",
               8: "UPDATING_FIRMWARE"}


def check_move(robot, ok, what: str) -> None:
    """robot.move_to / move_joints return False when the controller refuses the
    move (the arm then stays put, silently). Stop instead and say why."""
    if ok is not False:
        return
    r = robot.conn.receive
    mode = r.getRobotMode()
    raise SystemExit(
        f"\n!! controller refused the move ({what}) — the arm did not move.\n"
        f"   robot mode {ROBOT_MODES.get(mode, mode)}, protective stop {r.isProtectiveStopped()}, "
        f"emergency stop {r.isEmergencyStopped()}\n"
        "   check the pendant: a popup to confirm / unlock protective stop, ON -> START (mode RUNNING),\n"
        "   Remote Control. Also try: python examples/function_templates/03_move_to.py")


def build_parser():
    ap = argument_parser(__doc__)
    g = ap.add_argument_group("image")
    g.add_argument("--image", type=Path, help="image file (default: newest file in --inbox)")
    g.add_argument("--inbox", type=Path, default=HERE / "inbox", help="folder for webcam snapshots")
    g.add_argument("--demo-image", action="store_true", help="use a built-in test picture")
    g.add_argument("--style", choices=("dots", "hatch"), default="dots")
    g.add_argument("--invert", action="store_true", help="engrave the light parts instead")
    g.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=0, help="counter-clockwise")
    g.add_argument("--mirror", action="store_true", help="flip left/right")
    g.add_argument("--size-mm", type=float, default=None, help="max drawing size (default: fit box)")
    g.add_argument("--margin-mm", type=float, default=None,
                   help="free border inside the box (default 15, or 0 with a setup_canvas.py workspace)")
    g.add_argument("--max-marks", type=int, default=1500, help="refuse bigger jobs")
    g.add_argument("--dry-run", action="store_true", help="plan + preview only, no robot")

    g = ap.add_argument_group("dots (--style dots)")
    g.add_argument("--dot-mm", type=float, default=2.0, help="grid spacing = finest dot distance")
    g.add_argument("--sharpen", type=float, default=1.0, help="edge boost, 0 = off")
    g.add_argument("--no-levels", action="store_true", help="don't stretch the contrast")
    g.add_argument("--gamma", type=float, default=1.0, help=">1 lighter / fewer dots, <1 darker")
    g.add_argument("--min-dark", type=float, default=0.08, help="0..1, lighter cells stay empty")
    g.add_argument("--dwell-min", type=float, default=0.2, help="s melt time for the lightest dots")
    g.add_argument("--dwell-max", type=float, default=0.8, help="s melt time for the darkest dots")

    g = ap.add_argument_group("lines (--style hatch)")
    g.add_argument("--threshold", type=float, default=0.5, help="0..1, darker than this is engraved")
    g.add_argument("--hatch-mm", type=float, default=2.0, help="distance between engraved lines")
    g.add_argument("--pixel-mm", type=float, default=1.0, help="resolution along a line")
    g.add_argument("--min-stroke-mm", type=float, default=1.5, help="skip shorter strokes")
    g.add_argument("--dwell", type=float, default=0.3, help="s to wait after plunging")
    g.add_argument("--engrave-speed", type=float, default=0.005, help="m/s while cutting (5 mm/s)")

    g = ap.add_argument_group("motion")
    g.add_argument("--plunge-speed", type=float, default=0.01, help="m/s down into / out of the wax")
    g.add_argument("--travel-speed", type=float, default=0.06, help="m/s between marks")
    g.add_argument("--accel", type=float, default=0.3, help="m/s^2 for all moves")
    g.add_argument("--lift-mm", type=float, default=None,
                   help="travel height above the cut (default: 3 for dots, 8 for hatch)")
    g.add_argument("--clear-mm", type=float, default=None,
                   help="height above the cut that clears the frame walls; used to enter and leave "
                        "the canvas (default 30, or from the setup file)")
    g.add_argument("--hover-mm", type=float, default=0.0,
                   help="rehearsal: run everything this much ABOVE the cut (no contact)")

    g = ap.add_argument_group("heater")
    g.add_argument("--no-heat", action="store_true", help="draw cold: heater never on, no sensor")
    g.add_argument("--setpoint", type=float, default=120.0, help="tool temperature, degC")
    g.add_argument("--max-temp", type=float, default=180.0, help="cutoff, degC (latched)")
    g.add_argument("--heater-out", choices=("tool", "std"), default="tool",
                   help="tool = tool flange digital out, std = control box digital out")
    g.add_argument("--heater-pin", type=int, default=0)
    g.add_argument("--sensor-pin", type=int, default=0, help="standard analog input 0 or 1")
    g.add_argument("--sensor-volts", type=float, nargs=2, default=(0.0, 10.0), metavar=("V0", "V1"))
    g.add_argument("--sensor-temp", type=float, nargs=2, default=(0.0, 300.0), metavar=("T0", "T1"),
                   help="transmitter range: V0 -> T0 degC, V1 -> T1 degC")
    g.add_argument("--heat-timeout", type=float, default=300.0, help="s allowed to reach setpoint")
    g.add_argument("--sim-overheat", action="store_true",
                   help="SIM: fake a stuck heater to test the cutoff")
    return ap


def parse_args():
    ap = build_parser()
    pre, _ = ap.parse_known_args()
    f = HERE / f"wax_setup.{pre.target}.json"
    draw = json.loads(f.read_text()).get("draw", {}) if f.exists() else {}
    known = {a.dest for a in ap._actions}
    ap.set_defaults(**{k: v for k, v in draw.items() if k in known})
    if draw:
        print(f"image/dot settings from {f.name} (canvas_tool.py): "
              + ", ".join(f"{k}={v}" for k, v in draw.items() if k in known))
    return apply_setup(ap.parse_args())


def apply_setup(args):
    """Defaults from setup_canvas.py (wax_setup.<target>.json); flags you type win."""
    f = HERE / f"wax_setup.{args.target}.json"
    setup = json.loads(f.read_text()) if f.exists() else {}
    if setup:
        print(f"using {f.name}: workspace {setup['workspace']}, payload {setup['payload_kg']} kg, "
              f"TCP {[round(v * 1000, 1) for v in setup['tcp'][:3]]} mm")
        if args.workspace is None:
            args.workspace = setup["workspace"]
        if args.target == "real" and args.payload == 0.0 and not any(args.tcp):
            args.payload, args.tcp = setup["payload_kg"], tuple(setup["tcp"])
    if args.margin_mm is None:
        args.margin_mm = 0.0 if setup else 15.0
    if args.clear_mm is None:
        args.clear_mm = setup.get("clear_mm", 30.0)
    return args


def load_picture(args) -> tuple[np.ndarray, str]:
    if args.demo_image:
        gray, name = demo_image(), "demo image"
    else:
        path = Path(args.image) if args.image else newest_image(args.inbox)
        if not path.is_absolute() and not path.exists():
            path = HERE / path                     # saved relative to this folder (canvas_tool.py)
        gray, name = load_gray(path), str(path)
    if args.mirror:
        gray = np.fliplr(gray)
    if args.rotate:
        gray = np.rot90(gray, k=args.rotate // 90)
    return gray, name


class Canvas:
    """Maps canvas mm (x right, y down) to workspace fractions (u, v)."""

    def __init__(self, ws: Workspace, aspect: float, margin_mm: float, size_mm: float | None):
        b = ws.usable_box
        self.ws, self.b = ws, b
        span_x_mm = (b.x[1] - b.x[0]) * 1000      # image height runs along base X
        span_y_mm = (b.y[1] - b.y[0]) * 1000      # image width runs along base Y
        avail_w, avail_h = span_y_mm - 2 * margin_mm, span_x_mm - 2 * margin_mm
        if avail_w <= 0 or avail_h <= 0:
            raise SystemExit(f"workspace {ws.size_mm} mm is too small for a {margin_mm} mm margin")
        self.width_mm = min(avail_w, avail_h / aspect)
        if size_mm:
            self.width_mm = min(self.width_mm, size_mm, size_mm / aspect)
        self.height_mm = self.width_mm * aspect
        self.cx = (b.x[0] + b.x[1]) / 2
        self.cy = (b.y[0] + b.y[1]) / 2

    def pose(self, x_mm: float, y_mm: float, w: float):
        bx = self.cx + (self.height_mm / 2 - y_mm) / 1000     # image top -> +X
        by = self.cy + (self.width_mm / 2 - x_mm) / 1000      # image left -> +Y
        u = (bx - self.b.x[0]) / (self.b.x[1] - self.b.x[0])
        v = (by - self.b.y[0]) / (self.b.y[1] - self.b.y[0])
        return self.ws.pose(min(max(u, 0.0), 1.0), min(max(v, 0.0), 1.0), w)


def make_marks(gray, width_mm, args):
    """Marks for the chosen style -> (marks, width_mm, height_mm, summary, hint if too many)."""
    if args.style == "dots":
        marks, w_mm, h_mm = dot_marks(gray, width_mm, dot_mm=args.dot_mm, gamma=args.gamma,
                                      min_dark=args.min_dark, invert=args.invert,
                                      levels=not args.no_levels, sharpen=args.sharpen)
        return (marks, w_mm, h_mm, f"{len(marks)} dots on a {args.dot_mm} mm grid, "
                f"melt {args.dwell_min}-{args.dwell_max} s", "raise --dot-mm / --gamma")
    marks, w_mm, h_mm = hatch_strokes(gray, width_mm, hatch_mm=args.hatch_mm, pixel_mm=args.pixel_mm,
                                      threshold=args.threshold, min_len_mm=args.min_stroke_mm,
                                      invert=args.invert)
    return (marks, w_mm, h_mm, f"{len(marks)} lines, hatch {args.hatch_mm} mm, "
            f"engrave {args.engrave_speed * 1000:.0f} mm/s", "raise --hatch-mm / --min-stroke-mm")


def dwell_s(mark, args) -> float:
    (p0, p1, dark) = mark
    if p0 != p1:
        return args.dwell
    return args.dwell_min + (args.dwell_max - args.dwell_min) * dark


def estimate_s(marks, args) -> float:
    total, last = 0.0, None
    lift = args.lift_mm / 1000
    for mark in marks:
        p0, p1, _ = mark
        if last:
            total += math.dist(last, p0) / 1000 / args.travel_speed
        total += 2 * lift / args.plunge_speed + dwell_s(mark, args)
        moves = 3
        if p0 != p1:
            total += math.dist(p0, p1) / 1000 / args.engrave_speed
            moves = 4
        total += moves * MOVE_OVERHEAD_S
        last = p1
    return total + (JOB_OVERHEAD_S if marks else 0.0)


def cut_heights(ws, args):
    """Box fractions (w) for cutting, lifting between marks and clearing the frame walls."""
    box_h = (ws.usable_box.z[1] - ws.usable_box.z[0]) * 1000
    w_cut = args.hover_mm / box_h                                  # 0 = engraving depth
    w_lift = min((args.hover_mm + args.lift_mm) / box_h, 1.0)      # heights incl. hover
    w_clear = min((args.hover_mm + args.clear_mm) / box_h, 1.0)
    return w_cut, w_lift, w_clear, box_h


# A step is what the robot is sent, in order:
#   ("joints", q, speed or None, label)      robot.move_joints(q)        (None = profile default)
#   ("line", pose, speed, label)             robot.move_to(pose, speed, accel=args.accel)
#   ("dwell", seconds, None, label)          time.sleep — the tool melts the wax
# main() and the 3D simulation (sim3d.py) both use these functions, so the
# simulation shows exactly the moves a run sends.
def mark_steps(canvas, mark, i, args, w_cut, w_lift):
    (x0, y0), (x1, y1), _ = mark
    steps = [("line", canvas.pose(x0, y0, w_lift), args.travel_speed, f"mark {i}: travel above start"),
             ("line", canvas.pose(x0, y0, w_cut), args.plunge_speed, f"mark {i}: plunge"),
             ("dwell", dwell_s(mark, args), None, f"mark {i}: melt")]
    if (x0, y0) != (x1, y1):
        steps.append(("line", canvas.pose(x1, y1, w_cut), args.engrave_speed, f"mark {i}: engrave"))
    steps.append(("line", canvas.pose(x1, y1, w_lift), args.plunge_speed, f"mark {i}: lift"))
    return steps


def enter_step(canvas, marks, args, w_clear):
    (fx, fy), _, _ = marks[0]
    return ("line", canvas.pose(fx, fy, w_clear), args.travel_speed, "enter canvas")


def leave_step(canvas, last_xy, args, w_clear):
    return ("line", canvas.pose(*last_xy, w_clear), args.plunge_speed, "leave canvas")


def plan_steps(ws, canvas, marks, args):
    """The whole run without heater waits: home, enter, every mark, leave, home."""
    w_cut, w_lift, w_clear, _ = cut_heights(ws, args)
    steps = [("joints", ws.home_joints, None, "home"), enter_step(canvas, marks, args, w_clear)]
    for i, mark in enumerate(marks, 1):
        steps += mark_steps(canvas, mark, i, args, w_cut, w_lift)
    steps += [leave_step(canvas, marks[-1][1], args, w_clear), ("joints", ws.home_joints, None, "home")]
    return steps


def run_step(robot, step, args) -> None:
    kind, target, speed, label = step
    if kind == "joints":
        ok = robot.move_joints(target) if speed is None else robot.move_joints(target, speed=speed)
        check_move(robot, ok, label)
    elif kind == "line":
        check_move(robot, robot.move_to(target, speed=speed, accel=args.accel), label)
    else:
        time.sleep(target)


def main() -> None:
    args = parse_args()
    ws = Workspace.load(args.workspace)
    print(ws.describe())

    gray, name = load_picture(args)
    canvas = Canvas(ws, gray.shape[0] / gray.shape[1], args.margin_mm, args.size_mm)
    if args.lift_mm is None:
        args.lift_mm = 3.0 if args.style == "dots" else 8.0
    strokes, w_mm, h_mm, detail, too_many = make_marks(gray, canvas.width_mm, args)
    out_dir = HERE / "out"
    out_dir.mkdir(exist_ok=True)
    svg = write_svg(strokes, w_mm, h_mm, out_dir / "preview.svg", dot_r_mm=args.dot_mm * 0.4)

    w_cut, w_lift, w_clear, box_h = cut_heights(ws, args)
    if w_cut >= w_lift:
        raise SystemExit(f"--hover-mm {args.hover_mm} leaves no room in a {box_h:.0f} mm high box")
    print(f"\nimage    {name}  ({gray.shape[1]} x {gray.shape[0]} px)")
    print(f"drawing  {w_mm:.0f} x {h_mm:.0f} mm, {detail}")
    print(f"motion   travel {args.travel_speed * 1000:.0f} mm/s, plunge {args.plunge_speed * 1000:.0f} mm/s,"
          f" lift {w_lift * box_h:.1f} mm, clear {w_clear * box_h:.0f} mm")
    if args.hover_mm:
        print(f"REHEARSAL: cutting height raised by {args.hover_mm} mm — no contact with the wax")
    print(f"estimate {estimate_s(strokes, args) / 60:.1f} min")
    print(f"preview  {svg}")
    if w_clear * box_h < args.hover_mm + args.clear_mm - 0.1:
        raise SystemExit(f"workspace is only {box_h:.0f} mm high: teach the high corners at least "
                         f"{args.hover_mm + args.clear_mm + 10:.0f} mm above the cut (frame walls!)")
    if not strokes:
        raise SystemExit("nothing to engrave — try --invert, --gamma / --threshold")
    if len(strokes) > args.max_marks:
        raise SystemExit(f"{len(strokes)} marks > --max-marks {args.max_marks}: "
                         f"{too_many}, or --max-marks if intended")
    home_above = (ws.home[2] - ws.usable_box.z[0]) * 1000
    if home_above < args.hover_mm + args.clear_mm:
        raise SystemExit(f"home is only {home_above:.0f} mm above the cut — re-teach it higher than "
                         f"--clear-mm ({args.clear_mm:.0f} mm) so the way in/out clears the frame walls")
    if args.dry_run:
        print("dry run — no robot.")
        return

    fake = None
    if args.target == "sim":
        # SIM ONLY: URSim analog inputs read 0 V -> fake sensor (thermal model).
        fake = FakeHotend(runaway=args.sim_overheat)

    with Robot(**robot_kwargs(args), profile=ws) as robot:
        guard = nullcontext(None) if args.no_heat else HeaterGuard(
            robot, setpoint_c=args.setpoint, max_c=args.max_temp,
            out=args.heater_out, pin=args.heater_pin, sensor_pin=args.sensor_pin,
            volts_range=tuple(args.sensor_volts), temp_range=tuple(args.sensor_temp), fake=fake)
        with guard as heater:
            print("\nhome")
            run_step(robot, ("joints", ws.home_joints, None, "home"), args)

            if heater:
                print(f"heating to {args.setpoint:.0f} degC (cutoff {args.max_temp:.0f}) — tool at home")
                heater.enable()
                if not heater.wait_until_hot(args.heat_timeout):
                    print(f"aborted before drawing: {heater.tripped}")
                    run_step(robot, ("joints", ws.home_joints, None, "home"), args)
                    return

            warned, last = False, None
            t_start = time.monotonic()
            print("above the first mark at clearing height")
            run_step(robot, enter_step(canvas, strokes, args, w_clear), args)
            for i, mark in enumerate(strokes, 1):
                (x0, y0), (x1, y1), _ = mark
                is_dot = (x0, y0) == (x1, y1)
                if heater and heater.tripped:
                    break
                if heater and not heater.is_hot(tolerance_c=10.0):
                    print(f"\n  tool cooled to {heater.temperature:.0f} degC — waiting above the wax")
                    if not heater.wait_until_hot(args.heat_timeout):
                        break
                temp = f"  {heater.temperature:5.1f} degC" if heater else ""
                dwell = dwell_s(mark, args)
                what = (f"dot {i}/{len(strokes)}  ({x0:.0f},{y0:.0f}) mm  melt {dwell:.2f} s" if is_dot else
                        f"line {i}/{len(strokes)}  ({x0:.0f},{y0:.0f}) -> ({x1:.0f},{y1:.0f}) mm")
                print(what + temp)
                for step in mark_steps(canvas, mark, i, args, w_cut, w_lift):
                    run_step(robot, step, args)
                last = (x1, y1)
                if heater and heater.max_gap > 1.0 and not warned:
                    warned = True
                    print(f"!! heater watchdog paused {heater.max_gap:.1f} s during a move — "
                          "the temperature is NOT watched while the arm moves; tell the instructor")

            if heater and heater.tripped:
                print(f"\nABORTED at mark {i}: {heater.tripped}")
            else:
                print(f"\nfinished in {(time.monotonic() - t_start) / 60:.1f} min")
            if last:
                print("straight up to clearing height")
                run_step(robot, leave_step(canvas, last, args, w_clear), args)
            if heater:
                heater.disable()
            print("home")
            check_move(robot, robot.move_joints(ws.home_joints), "home")
    print("done.")


if __name__ == "__main__":
    main()
