"""Canvas tool: see and adjust the wax canvas in the browser, and simulate the run in 3D.
No robot needed.

2D view (http://localhost:8765): from above, standing opposite the robot — the cart, the
frame, the border, the box the robot will use, home, and the image as dots/lines exactly
where draw_wax.py will put them. Drag the two taught corners or type coordinates, change
frame / border / gap / width / cutting level and the image + dot settings, and save: the
workspace and wax_setup.<target>.json are written the way setup_canvas.py does, and
draw_wax.py then uses the saved image and dot settings as its defaults.

3D view (http://localhost:8766, also the "3D" tab): "Simulate in 3D" replays the exact move
list draw_wax.py would send (draw_wax.plan_steps), runs every move through the same checks
Robot runs, and animates the arm (sim3d.py). A move the checks would refuse is shown.

The preview uses the real code paths: geometry.frame_geometry, ur5e.Workspace,
draw_wax.Canvas / make_marks / plan_steps, and the cell profile from ur5e.config.

Run in the dev container, with both ports published to your laptop only:
  docker compose run --rm --no-deps -p 127.0.0.1:8765:8765 -p 127.0.0.1:8766:8766 dev \\
      python students/claudio/wax/canvas_tool.py
then open http://localhost:8765
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import re
import shutil
import struct
import threading
import traceback
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

from ur5e.config import cell_profile_name
from ur5e.safety import get_profile
from ur5e.workspace import Workspace, WorkspaceError

import draw_wax
from geometry import estimate_gap_mm, frame_geometry, straight_down
from imaging import IMAGE_SUFFIXES, load_gray

HERE = Path(__file__).resolve().parent
INBOX = HERE / "inbox"
WEB = HERE / "web" / "index.html"
MAX_UPLOAD = 20 * 1024 * 1024
_gray_cache: dict = {}


def cached_load_gray(path):
    """Decoding a PNG in pure Python takes a moment — keep the last few."""
    p = Path(path)
    key = (str(p), p.stat().st_mtime)
    if key not in _gray_cache:
        if len(_gray_cache) > 6:
            _gray_cache.clear()
        _gray_cache[key] = load_gray(p)
    return _gray_cache[key]


draw_wax.load_gray = cached_load_gray                # draw_wax.load_picture uses it

# draw_wax options the tool edits and saves into setup["draw"]
DRAW_KEYS = ("image", "style", "dot_mm", "gamma", "sharpen", "no_levels", "min_dark", "dwell_min",
             "dwell_max", "invert", "rotate", "mirror", "size_mm", "margin_mm", "hatch_mm", "threshold",
             "lift_mm", "travel_speed", "plunge_speed", "accel")


# ------------------------------------------------------------------ state --
def setup_path(target):
    return HERE / f"wax_setup.{target}.json"


def load_setup(target):
    f = setup_path(target)
    return json.loads(f.read_text()) if f.exists() else {}


def tcp_extra(s):
    tcp = s.get("tcp", [0, 0, 0, 0, 0, 0])
    return tcp, (s.get("tip_mm", tcp[2] * 1000) - tcp[2] * 1000) / 1000


def draw_defaults(setup):
    ns = draw_wax.build_parser().parse_args([])
    d = {k: getattr(ns, k) for k in DRAW_KEYS}
    d["image"] = None
    d["margin_mm"] = 0.0 if setup else 15.0
    d.update({k: v for k, v in setup.get("draw", {}).items() if k in DRAW_KEYS})
    if d.get("image"):
        d["image"] = Path(d["image"]).name          # saved as inbox/<name>
    return d


def saved_workspace(s):
    try:
        return Workspace.load(s["workspace"]) if s.get("workspace") else None
    except (WorkspaceError, OSError):
        return None


def infer_fixes(s, bl, tl, frame, border, gap):
    """Frame width + cutting offset that reproduce the saved workspace — for setups whose
    workspace was corrected by hand before the tool knew these two settings."""
    w = saved_workspace(s)
    if w is None:
        return None, 0.0
    _, extra = tcp_extra(s)
    c = w.corners_low
    width = math.dist(c[0][:2], c[3][:2]) * 1000 + 2 * border
    geo0 = frame_geometry(bl, tl, frame_mm=frame, border_mm=border, gap_mm=gap, extra_m=extra,
                          ceiling_mm=s.get("ceiling_mm", 80.0))
    cut = (c[0][2] - geo0["zs"]) * 1000
    return round(width * 2) / 2, round(cut * 2) / 2


def initial_params(target):
    """Everything the page edits, in mm (positions) — from the setup file or sensible defaults."""
    s = load_setup(target)
    c = s.get("corners", {})
    if "bl" in c and "tl" in c:
        bl, tl = c["bl"], c["tl"]
    else:                                       # no corners yet: a 180 mm frame in front of the robot
        bl, tl = [-0.433, 0.067, -0.01], [-0.253, 0.067, -0.01]
    frame = s.get("frame_mm", 180.0)
    border = s.get("border_mm", 20.0)
    gap = c.get("gap_mm", estimate_gap_mm(bl, tl, frame))
    width, cut = s.get("frame_width_mm"), s.get("cut_offset_mm")
    inferred = False
    if (width is None or cut is None) and "bl" in c:
        w_i, c_i = infer_fixes(s, np.array(bl), np.array(tl), frame, border, gap)
        if w_i is not None:
            inferred = (width is None and abs(w_i - frame) > 0.5) or (cut is None and abs(c_i) > 0.25)
            width = w_i if width is None else width
            cut = c_i if cut is None else cut
    return {
        "bl": [round(v * 1000, 2) for v in bl], "tl": [round(v * 1000, 2) for v in tl],
        "frame_mm": frame, "frame_width_mm": width or frame, "cut_offset_mm": cut or 0.0,
        "border_mm": border, "wall_mm": s.get("wall_mm", 15.0), "ceiling_mm": s.get("ceiling_mm", 80.0),
        "gap_mm": gap, "draw": draw_defaults(s), "inferred": inferred,
    }


def images():
    INBOX.mkdir(exist_ok=True)
    files = sorted((p for p in INBOX.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES),
                   key=lambda p: p.stat().st_mtime, reverse=True)
    return [p.name for p in files]


# --------------------------------------------------------------- compute --
def png_data_url(gray: np.ndarray, max_px=320) -> str:
    """Small grayscale PNG (stdlib only) for the image overlay."""
    h, w = gray.shape
    step = max(1, int(math.ceil(max(h, w) / max_px)))
    g = (np.clip(gray[::step, ::step], 0, 1) * 255).astype(np.uint8)
    h, w = g.shape
    raw = b"".join(b"\x00" + g[y].tobytes() for y in range(h))

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
    return "data:image/png;base64," + base64.b64encode(png).decode()


def build(target, p):
    """Geometry + workspace from the page's parameters -> (result, ws or None, preview ws, setup)."""
    s = load_setup(target)
    tcp, extra = tcp_extra(s)
    bl = np.array(p["bl"], float) / 1000
    tl = np.array(p["tl"], float) / 1000
    errors, warnings = [], []
    geo = frame_geometry(bl, tl, frame_mm=p["frame_mm"], border_mm=p["border_mm"], gap_mm=p["gap_mm"],
                         extra_m=extra, ceiling_mm=p["ceiling_mm"], width_mm=p.get("frame_width_mm"),
                         cut_offset_mm=p.get("cut_offset_mm", 0.0))
    if p["ceiling_mm"] < p["wall_mm"] + 40:
        errors.append(f"ceiling must be at least {p['wall_mm'] + 40:.0f} mm (walls + 30 mm clearing + margin)")
    if abs(geo["depth_mm"] - p["frame_mm"]) > 5:
        warnings.append(f"frame depth from the corners is {geo['depth_mm']:.0f} mm, frame size "
                        f"{p['frame_mm']:.0f} mm — check the gap / corners")
    if geo["level_mm"] > 0.7:
        warnings.append(f"canvas not level: corners differ by {geo['level_mm']:.1f} mm in height")
    zs, zt = geo["zs"], geo["zt"]
    home = s.get("home")
    home_ok = bool(home) and home.get("tcp") == tcp
    if home_ok:
        orient = straight_down(home["pose"][3:])
        hp, hq = home["pose"], home["q"]
    else:
        errors.append("no home saved for this TCP yet — run setup_canvas.py once (step 5) before saving")
        orient = [0.0, math.pi, 0.0]
    low = [[x, y, zs, *orient] for x, y in geo["drawable"]]
    high = [[x, y, zt, *orient] for x, y in geo["drawable"]]
    ws = None
    try:
        if home_ok:
            ws = Workspace.from_points(target, low, high, hp, hq)
    except WorkspaceError as e:
        errors.append(str(e))
    preview_ws = ws
    if preview_ws is None:              # still preview the drawing, with a stand-in home
        xs, ys = [c[0] for c in geo["drawable"]], [c[1] for c in geo["drawable"]]
        stand_in = [sorted(xs)[1] / 2 + sorted(xs)[2] / 2, sorted(ys)[1] / 2 + sorted(ys)[2] / 2,
                    (zs + zt) / 2, *orient]
        try:
            preview_ws = Workspace.from_points(target, low, high, stand_in, [0.0] * 6)
        except WorkspaceError as e:
            errors.append(str(e))
    res = {"geo": geo, "errors": errors, "warnings": warnings, "extra_mm": extra * 1000}
    cell = get_profile(cell_profile_name(target))
    res["cell_name"] = cell.name
    res["keepouts"] = [{"name": k.name, "x": [v * 1000 for v in k.box.x], "y": [v * 1000 for v in k.box.y]}
                       for k in cell.keepout]
    cb = cell.workspace
    outside = [n for n, (x, y) in zip(("BL", "TL", "TR", "BR"), geo["drawable"])
               if not (cb.x[0] <= x <= cb.x[1] and cb.y[0] <= y <= cb.y[1])]
    if outside:
        errors.append(f"outside the '{cell.name}' cell box: " + ", ".join(outside))
    if zs < cb.z[0]:
        errors.append(f"cutting level (TCP {zs * 1000:.0f} mm) below the '{cell.name}' box ({cb.z[0] * 1000:.0f} mm)")
    # does this rebuild the saved workspace?
    sw = saved_workspace(s)
    if sw is not None:
        dev = max(math.dist(a[:3], b[:3]) for a, b in zip(sw.corners_low, low)) * 1000
        res["saved_match_mm"] = round(dev, 2)
    mm_ = tool_mismatch(s)
    if mm_:
        warnings.append(mm_)
    if home:
        res["home"] = [v * 1000 for v in home["pose"][:2]]
    if preview_ws is not None:
        b = preview_ws.usable_box
        res["box"] = {"x": [v * 1000 for v in b.x], "y": [v * 1000 for v in b.y]}
    return res, ws, preview_ws, s


def compose(target, p):
    """build() + image + marks -> (res, ws, preview ws, setup, canvas, marks, ns, gray) or res on error."""
    res, ws, pws, s = build(target, p)
    if pws is None:
        return res
    d = p["draw"]
    ns = draw_wax.build_parser().parse_args([])
    for k in DRAW_KEYS:
        setattr(ns, k, d.get(k, getattr(ns, k)))
    ns.demo_image = not d.get("image")
    ns.image = str(INBOX / d["image"]) if d.get("image") else None
    if not ns.lift_mm:
        ns.lift_mm = 3.0 if ns.style == "dots" else 8.0
    ns.clear_mm = p["wall_mm"] + 15
    ns.hover_mm = 0.0
    try:
        gray, name = draw_wax.load_picture(ns)
        canvas = draw_wax.Canvas(pws, gray.shape[0] / gray.shape[1], float(ns.margin_mm or 0), ns.size_mm)
        marks, w_mm, h_mm, detail, too_many = draw_wax.make_marks(gray, canvas.width_mm, ns)
    except SystemExit as e:
        res["errors"].append(str(e))
        return res
    except Exception as e:
        res["errors"].append(f"image: {e}")
        return res
    res.update(_w_mm=w_mm, _h_mm=h_mm, _detail=detail, _too_many=too_many, _name=name)
    return res, ws, pws, s, canvas, marks, ns, gray


def mark_points(canvas, marks):
    def base(x, y):
        q = canvas.pose(x, y, 0.0)
        return [q[0], q[1]]
    out = []
    for (x0, y0), (x1, y1), dark in marks:
        a = base(x0, y0)
        out.append(a + [dark] if (x0, y0) == (x1, y1) else a + base(x1, y1))
    return out


def preview(target, p):
    try:
        c = compose(target, p)
    except ValueError as e:
        return {"errors": [str(e)], "warnings": []}
    if isinstance(c, dict):
        return c
    res, ws, pws, s, canvas, marks, ns, gray = c
    w_mm, h_mm = res.pop("_w_mm"), res.pop("_h_mm")
    detail, too_many, name = res.pop("_detail"), res.pop("_too_many"), res.pop("_name")
    res["marks"] = [[round(v * 1000, 2) for v in m[:2]] + ([round(m[2], 3)] if len(m) == 3 else
                    [round(v * 1000, 2) for v in m[2:]]) for m in mark_points(canvas, marks)]
    res["style"] = ns.style
    res["dot_r_mm"] = ns.dot_mm * 0.4

    def base_mm(x, y):
        q = canvas.pose(x, y, 0.0)
        return [round(q[0] * 1000, 2), round(q[1] * 1000, 2)]
    res["image_rect"] = [base_mm(0, 0), base_mm(w_mm, 0), base_mm(0, h_mm)]
    res["image_png"] = png_data_url(gray)
    res["image_name"] = name if p["draw"].get("image") else "demo image"
    res["stats"] = {"count": len(marks), "detail": detail, "size_mm": [round(w_mm), round(h_mm)],
                    "minutes": round(draw_wax.estimate_s(marks, ns) / 60, 1)}
    if len(marks) > ns.max_marks:
        res["warnings"].append(f"{len(marks)} marks > --max-marks {ns.max_marks}: {too_many}")
    return res


def save(target, p):
    try:
        res, ws, _, s = build(target, p)
    except ValueError as e:
        return {"ok": False, "errors": [str(e)]}
    if res["errors"] or ws is None:
        return {"ok": False, "errors": res["errors"] or ["workspace not valid"]}
    out = Path(s.get("workspace") or f"workspace.wax-{target}.json")
    if out.exists():
        shutil.copy(out, out.with_suffix(".json.bak"))
    ws.save(out)
    f = setup_path(target)
    if f.exists():
        shutil.copy(f, f.with_suffix(".json.bak"))
    s.update({"workspace": str(out), "frame_mm": p["frame_mm"], "frame_width_mm": p["frame_width_mm"],
              "cut_offset_mm": p["cut_offset_mm"], "border_mm": p["border_mm"],
              "wall_mm": p["wall_mm"], "ceiling_mm": p["ceiling_mm"], "clear_mm": p["wall_mm"] + 15,
              "corners": {"bl": [v / 1000 for v in p["bl"]], "tl": [v / 1000 for v in p["tl"]],
                          "gap_mm": p["gap_mm"]},
              "draw": {k: p["draw"].get(k) for k in DRAW_KEYS}})
    s["draw"]["image"] = f"inbox/{p['draw']['image']}" if p["draw"].get("image") else None
    f.write_text(json.dumps(s, indent=2))
    return {"ok": True, "saved": [str(out), str(f.relative_to(Path.cwd()) if f.is_relative_to(Path.cwd()) else f)],
            "describe": ws.describe()}


# ------------------------------------------------------------- 3D sim --
SIM3D = None                     # sim3d.Sim3D, if viser is available
TOOL_MESH = None                 # (vertices, faces, tip) in the flange frame, from --tool-stl


def tip_vec(s):
    """Flange -> real tip according to the setup: TCP x/y, and the measured tip length along z."""
    tcp, _ = tcp_extra(s)
    return [tcp[0], tcp[1], s.get("tip_mm", tcp[2] * 1000) / 1000]


def tool_mismatch(s):
    if TOOL_MESH is None:
        return None
    m, t = TOOL_MESH[2], np.array(tip_vec(s))
    d = float(np.linalg.norm(m - t)) * 1000
    if d <= 3:
        return None
    return (f"tool STL: its tip is at x {m[0] * 1000:.1f}, y {m[1] * 1000:.1f}, z {m[2] * 1000:.1f} mm from the flange, "
            f"your settings say x {t[0] * 1000:.1f}, y {t[1] * 1000:.1f}, z {t[2] * 1000:.1f} mm ({d:.0f} mm apart). "
            f"Moves and checks use the settings (TCP) — with this tool mounted the dots would land at the "
            f"settings' point, not at the STL tip. Measure and set the TCP for this tool before using it.")
SIM_STATE = {"running": False, "progress": 0.0, "result": None, "error": None}
SIM_LOCK = threading.Lock()


def start_simulation(target, p):
    if SIM3D is None:
        return {"ok": False, "errors": ["3D view not available (viser missing — run the tool in the dev container)"]}
    with SIM_LOCK:
        if SIM_STATE["running"]:
            return {"ok": False, "errors": ["a simulation is already running"]}
    try:
        c = compose(target, p)
    except ValueError as e:
        return {"ok": False, "errors": [str(e)]}
    if isinstance(c, dict):
        return {"ok": False, "errors": c["errors"] or ["nothing to simulate"]}
    res, ws, pws, s, canvas, marks, ns, gray = c
    if ws is None:
        return {"ok": False, "errors": res["errors"] or ["no valid workspace (home?)"]}
    if not marks:
        return {"ok": False, "errors": ["no marks to draw"]}
    tcp, extra = tcp_extra(s)
    geo = res["geo"]
    cell = get_profile(cell_profile_name(target))
    scene = {
        "keepouts": [(k.name, k.box) for k in cell.keepout], "frame": geo["frame"], "drawable": geo["drawable"],
        "box": {"x": list(ws.usable_box.x), "y": list(ws.usable_box.y)},
        "wax_z": geo["zs"] - extra, "wall": p["wall_mm"] / 1000, "rot_deg": geo["rot_deg"],
        "tip_vec": tip_vec(s), "tcp": tcp, "tool_mesh": TOOL_MESH,
        "marks": mark_points(canvas, marks), "style": ns.style, "home": ws.home[:3],
    }
    steps = draw_wax.plan_steps(ws, canvas, marks, ns)

    def run():
        try:
            SIM3D.set_scene(scene)
            sim = __import__("sim3d").simulate(
                ws, steps, ns.accel, tcp,
                progress=lambda f: (SIM_STATE.__setitem__("progress", f), SIM3D.progress(f)))
            SIM3D.set_sim(sim, scene["tip_vec"], scene["wax_z"])
            v = sim["violation"]
            SIM_STATE["result"] = {"moves": len(steps), "checked": len(sim["segments"]) - (1 if v else 0),
                                   "minutes": round(sim["total_s"] / 60, 1), "violation": v}
        except Exception as e:
            traceback.print_exc()
            SIM_STATE["error"] = f"{type(e).__name__}: {e}"
        finally:
            SIM_STATE["running"] = False

    with SIM_LOCK:
        SIM_STATE.update(running=True, progress=0.0, result=None, error=None)
    threading.Thread(target=run, daemon=True).start()
    return {"ok": True, "moves": len(steps)}


# ---------------------------------------------------------------- server --
class Handler(BaseHTTPRequestHandler):
    target = "real"
    port3d = None

    def log_message(self, fmt, *a):
        pass

    def send(self, code, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def body(self):
        n = int(self.headers.get("Content-Length", 0))
        if n > MAX_UPLOAD:
            raise ValueError("too large")
        return self.rfile.read(n)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            return self.send(200, WEB.read_bytes(), "text/html; charset=utf-8")
        if self.path == "/api/state":
            s = load_setup(self.target)
            return self.send(200, {"target": self.target, "params": initial_params(self.target),
                                   "images": images(), "has_setup": bool(s),
                                   "setup_file": setup_path(self.target).name,
                                   "port3d": self.port3d if SIM3D is not None else None})
        if self.path == "/api/simstatus":
            return self.send(200, SIM_STATE)
        self.send(404, {"error": "not found"})

    def do_POST(self):
        try:
            if self.path == "/api/preview":
                return self.send(200, preview(self.target, json.loads(self.body())))
            if self.path == "/api/save":
                return self.send(200, save(self.target, json.loads(self.body())))
            if self.path == "/api/simulate":
                return self.send(200, start_simulation(self.target, json.loads(self.body())))
            if self.path.startswith("/api/upload"):
                name = re.sub(r"[^A-Za-z0-9._-]", "_", Path(self.headers.get("X-Filename", "upload.png")).name)
                if Path(name).suffix.lower() not in IMAGE_SUFFIXES:
                    return self.send(400, {"error": f"only {', '.join(IMAGE_SUFFIXES)}"})
                data = self.body()
                INBOX.mkdir(exist_ok=True)
                (INBOX / name).write_bytes(data)
                try:
                    load_gray(INBOX / name)
                except Exception as e:
                    (INBOX / name).unlink()
                    return self.send(400, {"error": f"can't read {name}: {e} (PNG works without Pillow)"})
                return self.send(200, {"ok": True, "name": name, "images": images()})
        except Exception as e:
            traceback.print_exc()
            return self.send(500, {"errors": [f"{type(e).__name__}: {e}"]})
        self.send(404, {"error": "not found"})


def main():
    global SIM3D, TOOL_MESH
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", choices=("sim", "real"), default="real", help="which setup file to edit")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--port3d", type=int, default=8766, help="3D view (viser); 0 = off")
    ap.add_argument("--bind", default="0.0.0.0", help="inside Docker this must be 0.0.0.0")
    ap.add_argument("--tool-stl", type=Path, default=None,
                    help="tool model for the 3D view (STL; its lowest face = the flange face, tool along +z)")
    ap.add_argument("--tool-units", choices=("mm", "m"), default="mm")
    ap.add_argument("--tool-rot", type=float, default=0.0, help="turn the model about the flange axis, deg")
    args = ap.parse_args()
    Handler.target, Handler.port3d = args.target, args.port3d
    if args.tool_stl:
        from sim3d import tool_from_stl
        TOOL_MESH = tool_from_stl(args.tool_stl, args.tool_units, args.tool_rot)
        t = TOOL_MESH[2] * 1000
        print(f"tool model {args.tool_stl}: {len(TOOL_MESH[1])} triangles, tip at x {t[0]:.1f} y {t[1]:.1f} "
              f"z {t[2]:.1f} mm from the flange")
        mm_ = tool_mismatch(load_setup(args.target))
        if mm_:
            print("!! " + mm_)
    if args.port3d:
        try:
            from sim3d import Sim3D
            SIM3D = Sim3D(port=args.port3d, host=args.bind)
            print(f"3D view -> http://localhost:{args.port3d}")
        except ImportError as e:
            print(f"(3D view off: {e})")
    srv = ThreadingHTTPServer((args.bind, args.port), Handler)
    print(f"canvas tool for '{args.target}' ({setup_path(args.target).name}) -> open http://localhost:{args.port}")
    print("Ctrl-C to stop.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
