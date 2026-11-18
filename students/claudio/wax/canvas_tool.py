"""Canvas tool: see and adjust the wax canvas in the browser. No robot needed.

Shows, from above (standing opposite the robot): the lab cell, the frame, the
border, the box the robot will use, home, and the image as dots/lines exactly
where draw_wax.py will put them. Drag the two taught corners or type
coordinates, change frame / border / gap and the image + dot settings, and
save: the workspace and wax_setup.<target>.json are written the same way
setup_canvas.py does, and draw_wax.py then uses the saved image and dot
settings as its defaults.

The preview uses the real code paths: geometry.frame_geometry, ur5e.Workspace,
draw_wax.Canvas and draw_wax.make_marks.

Run in the dev container, with the port published to your laptop only:
  docker compose run --rm --no-deps -p 127.0.0.1:8765:8765 dev python students/claudio/wax/canvas_tool.py
then open http://localhost:8765
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import math
import re
import shutil
import struct
import traceback
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

from ur5e.safety import get_profile
from ur5e.workspace import Workspace, WorkspaceError

import draw_wax
from geometry import estimate_gap_mm, frame_geometry, straight_down
from imaging import IMAGE_SUFFIXES, load_gray

HERE = Path(__file__).resolve().parent
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
INBOX = HERE / "inbox"
WEB = HERE / "web" / "index.html"
CELL = "cart_against_wall"
MAX_UPLOAD = 20 * 1024 * 1024

# draw_wax options the tool edits and saves into setup["draw"]
DRAW_KEYS = ("image", "style", "dot_mm", "gamma", "sharpen", "no_levels", "min_dark", "dwell_min",
             "dwell_max", "invert", "rotate", "mirror", "size_mm", "margin_mm", "hatch_mm", "threshold")


# ------------------------------------------------------------------ state --
def setup_path(target):
    return HERE / f"wax_setup.{target}.json"


def load_setup(target):
    f = setup_path(target)
    return json.loads(f.read_text()) if f.exists() else {}


def draw_defaults(setup):
    ns = draw_wax.build_parser().parse_args([])
    d = {k: getattr(ns, k) for k in DRAW_KEYS}
    d["image"] = None
    d["margin_mm"] = 0.0 if setup else 15.0
    d.update({k: v for k, v in setup.get("draw", {}).items() if k in DRAW_KEYS})
    if d.get("image"):
        d["image"] = Path(d["image"]).name          # saved as inbox/<name>
    return d


def initial_params(target):
    """Everything the page edits, in mm (positions) — from the setup file or sensible defaults."""
    s = load_setup(target)
    c = s.get("corners", {})
    if "bl" in c and "tl" in c:
        bl, tl = c["bl"], c["tl"]
    else:                                       # no corners yet: a 180 mm frame centred in the lab cell
        bl, tl = [-0.433, 0.067, 0.03], [-0.253, 0.067, 0.03]
    frame = s.get("frame_mm", 180.0)
    return {
        "bl": [round(v * 1000, 2) for v in bl], "tl": [round(v * 1000, 2) for v in tl],
        "frame_mm": frame, "border_mm": s.get("border_mm", 20.0), "wall_mm": s.get("wall_mm", 15.0),
        "ceiling_mm": s.get("ceiling_mm", 80.0),
        "gap_mm": c.get("gap_mm", estimate_gap_mm(bl, tl, frame)),
        "draw": draw_defaults(s),
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
    """Geometry + workspace from the page's parameters. Returns (result dict, Workspace or None)."""
    s = load_setup(target)
    tcp = s.get("tcp", [0, 0, 0, 0, 0, 0])
    extra = (s.get("tip_mm", tcp[2] * 1000) - tcp[2] * 1000) / 1000
    bl = np.array(p["bl"], float) / 1000
    tl = np.array(p["tl"], float) / 1000
    errors, warnings = [], []
    geo = frame_geometry(bl, tl, frame_mm=p["frame_mm"], border_mm=p["border_mm"], gap_mm=p["gap_mm"],
                         extra_m=extra, ceiling_mm=p["ceiling_mm"])
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
    if target == "real":
        cb = get_profile(CELL).workspace
        res["cell"] = {"x": [v * 1000 for v in cb.x], "y": [v * 1000 for v in cb.y]}
        outside = [n for n, (x, y) in zip(("BL", "TL", "TR", "BR"), geo["drawable"])
                   if not (cb.x[0] <= x <= cb.x[1] and cb.y[0] <= y <= cb.y[1])]
        if outside:
            errors.append("outside the lab cell: " + ", ".join(outside))
        if zs < cb.z[0]:
            errors.append(f"wax (TCP {zs * 1000:.0f} mm) below the lab cell floor {cb.z[0] * 1000:.0f} mm")
    if home:
        res["home"] = [v * 1000 for v in home["pose"][:2]]
    if preview_ws is not None:
        b = preview_ws.usable_box
        res["box"] = {"x": [v * 1000 for v in b.x], "y": [v * 1000 for v in b.y]}
    return res, ws, preview_ws, s


def preview(target, p):
    try:
        res, ws, pws, s = build(target, p)
    except ValueError as e:
        return {"errors": [str(e)], "warnings": []}
    if pws is None:
        return res
    d = p["draw"]
    ns = draw_wax.build_parser().parse_args([])
    for k in DRAW_KEYS:
        setattr(ns, k, d.get(k, getattr(ns, k)))
    ns.demo_image = not d.get("image")
    ns.image = str(INBOX / d["image"]) if d.get("image") else None
    ns.lift_mm = 3.0 if ns.style == "dots" else 8.0
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

    def base_mm(x, y):
        q = canvas.pose(x, y, 0.0)
        return [round(q[0] * 1000, 2), round(q[1] * 1000, 2)]
    out = []
    for (x0, y0), (x1, y1), dark in marks:
        a = base_mm(x0, y0)
        out.append(a + [round(dark, 3)] if (x0, y0) == (x1, y1) else a + base_mm(x1, y1))
    res["marks"] = out
    res["style"] = ns.style
    res["dot_r_mm"] = ns.dot_mm * 0.4
    res["image_rect"] = [base_mm(0, 0), base_mm(w_mm, 0), base_mm(0, h_mm)]
    res["image_png"] = png_data_url(gray)
    res["image_name"] = name if d.get("image") else "demo image"
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
    s.update({"workspace": str(out), "frame_mm": p["frame_mm"], "border_mm": p["border_mm"],
              "wall_mm": p["wall_mm"], "ceiling_mm": p["ceiling_mm"], "clear_mm": p["wall_mm"] + 15,
              "corners": {"bl": [v / 1000 for v in p["bl"]], "tl": [v / 1000 for v in p["tl"]],
                          "gap_mm": p["gap_mm"]},
              "draw": {k: p["draw"].get(k) for k in DRAW_KEYS}})
    s["draw"]["image"] = f"inbox/{p['draw']['image']}" if p["draw"].get("image") else None
    f.write_text(json.dumps(s, indent=2))
    return {"ok": True, "saved": [str(out), str(f.relative_to(Path.cwd()) if f.is_relative_to(Path.cwd()) else f)],
            "describe": ws.describe()}


# ---------------------------------------------------------------- server --
class Handler(BaseHTTPRequestHandler):
    target = "real"

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
                                   "setup_file": setup_path(self.target).name})
        self.send(404, {"error": "not found"})

    def do_POST(self):
        try:
            if self.path == "/api/preview":
                return self.send(200, preview(self.target, json.loads(self.body())))
            if self.path == "/api/save":
                return self.send(200, save(self.target, json.loads(self.body())))
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
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", choices=("sim", "real"), default="real", help="which setup file to edit")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--bind", default="0.0.0.0", help="inside Docker this must be 0.0.0.0")
    args = ap.parse_args()
    Handler.target = args.target
    srv = ThreadingHTTPServer((args.bind, args.port), Handler)
    print(f"canvas tool for '{args.target}' ({setup_path(args.target).name}) -> open http://localhost:{args.port}")
    print("Ctrl-C to stop.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
