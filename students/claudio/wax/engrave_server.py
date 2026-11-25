"""Webcam engraving station: one card per webcam, "Update engraving" engraves its newest frame.

The button wall (webcam-buttons/webcams.py) saves a frame per button press into
webcam-buttons/frames/cam1..cam4/. This page shows, per cam: the newest frame, when it was
captured, a dot preview with the estimated time, and the state:

  to be engraved   a newer frame than the one engraved last (or never engraved)
  in progress      the robot is engraving it now (with dot count and time left)
  engraved         the newest frame is the one engraved last

"Update engraving" runs draw_wax.py on that frame — the whole canvas, one job at a time.
"Stop" stops the robot program and the job; "Lift & home" takes the tool out of the wax.

webcams.py on another computer: set WAX_URL = "http://<this laptop>:8770" there — its upload
lands in frames/<cam>/ here (same as a local capture).

Usage (inside the dev container, port published):
  docker compose run --rm --no-deps -p 8770:8770 -e UR_HOST=192.168.1.11 dev \\
      python students/claudio/wax/engrave_server.py --target real
  ... --target sim                                  # against URSim first
  ... --target real -- --hover-mm 0.8               # extra draw_wax.py flags after --
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
WEB = HERE / "web" / "engrave.html"
JOBS = HERE / "out" / "jobs"
PREVIEWS = HERE / "out" / "engrave_previews"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".pgm", ".ppm"}
NAME_TIME = re.compile(r"(\d{8})_(\d{6})")
DOT_RE = re.compile(r"(?:dot|line) (\d+)/(\d+)")


# ---------------------------------------------------------------- frames

def cam_dirs(frames: Path) -> list[Path]:
    return sorted((d for d in frames.glob("cam*") if d.is_dir()),
                  key=lambda d: int(re.sub(r"\D", "", d.name) or 0))


def frame_list(d: Path) -> list[Path]:
    return sorted(f for f in d.iterdir() if f.suffix.lower() in IMAGE_SUFFIXES and f.stat().st_size > 0)


def captured_at(f: Path) -> str:
    m = NAME_TIME.search(f.name)
    t = datetime.strptime("".join(m.groups()), "%Y%m%d%H%M%S") if m else datetime.fromtimestamp(f.stat().st_mtime)
    return t.isoformat(timespec="seconds")


def mono_svg(marks, w_mm: float, h_mm: float, path: Path, dot_r_mm: float) -> None:
    """Dots only, black on white (dot size ~ darkness) — how the engraving will read."""
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w_mm:.2f} {h_mm:.2f}">',
             f'<rect width="{w_mm:.2f}" height="{h_mm:.2f}" fill="#fff"/><g fill="#000">']
    for (x0, y0), (x1, y1), dark in marks:
        if (x0, y0) == (x1, y1):
            parts.append(f'<circle cx="{x0:.2f}" cy="{y0:.2f}" r="{dot_r_mm * (0.4 + 0.6 * dark):.2f}"/>')
        else:
            parts.append(f'<line x1="{x0:.2f}" y1="{y0:.2f}" x2="{x1:.2f}" y2="{y1:.2f}" stroke="#000" '
                         f'stroke-width="0.8" stroke-linecap="round"/>')
    parts.append("</g></svg>")
    path.write_text("".join(parts))


def job_error(text: str, code: int) -> str:
    """The line that says why a job failed: a traceback's last line, or the script's own !! message."""
    lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
    if any(ln.startswith("Traceback") for ln in lines):
        last = lines[-1]
        return re.sub(r"^[\w.]+\.(\w+(Error|Violation|Exception))", r"\1", last)   # drop module path
    msg = next((ln for ln in reversed(lines) if ln.startswith(("!!", "not ", "nothing"))), None)
    return msg or (lines[-1] if lines else f"exit code {code}")


# ---------------------------------------------------------------- robot state (read-only)

def dashboard(host: str, *cmds: str) -> list[str]:
    out = []
    with socket.create_connection((host, 29999), timeout=3) as s:
        f = s.makefile("rw", newline="\n")
        f.readline()                                    # greeting
        for c in cmds:
            f.write(c + "\n")
            f.flush()
            out.append(f.readline().strip())
    return out


def robot_ready_from(r: dict, target: str) -> str | None:
    if r["mode"] != "RUNNING":
        return f"robot {r['mode']} — pendant: ON -> START"
    if r["safety"] != "NORMAL":
        return f"{r['safety']} — clear it on the pendant"
    if not r["remote"]:
        return "pendant in Local mode — switch to Remote Control"
    return None


def robot_ready(host: str, target: str) -> str | None:
    """None if the arm can be driven, else the reason."""
    try:
        mode, safety, remote = dashboard(host, "robotmode", "safetystatus", "is in remote control")
    except OSError as e:
        return f"robot not reachable at {host} ({e})"
    if "RUNNING" not in mode:
        return f"{mode} — pendant: ON -> START"
    if "NORMAL" not in safety:
        return f"{safety} — clear it on the pendant"
    if "true" not in remote.lower():
        return "pendant is in Local mode — switch to Remote Control"
    return None


# ---------------------------------------------------------------- station

class Station:
    def __init__(self, target: str, frames: Path, host: str, extra: list[str]):
        self.target, self.frames, self.host, self.extra = target, frames, host, extra
        self.state_file = HERE / f"engrave_state.{target}.json"
        self.state = json.loads(self.state_file.read_text()) if self.state_file.exists() else {"cams": {}}
        self.job: dict | None = None
        self.lock = threading.Lock()
        self.previews: dict[str, dict] = {}
        self.robot = {"reachable": False, "checked": None, "error": "not checked yet"}
        self.capture_file = frames.parent / "capture_status.json"
        self.capture_posted: dict | None = None             # from webcams.py on another computer
        threading.Thread(target=self._watch_robot, daemon=True).start()
        JOBS.mkdir(parents=True, exist_ok=True)
        PREVIEWS.mkdir(parents=True, exist_ok=True)

    # -- connectivity (read-only)
    def _watch_robot(self):
        while True:
            now = datetime.now().isoformat(timespec="seconds")
            try:
                mode, safety, remote, prog = dashboard(self.host, "robotmode", "safetystatus",
                                                       "is in remote control", "programState")
                self.robot = {"reachable": True, "checked": now,
                              "mode": mode.split(":", 1)[-1].strip(),
                              "safety": safety.split(":", 1)[-1].strip(),
                              "remote": "true" in remote.lower(), "program": prog.split()[0] if prog else ""}
                self.robot["ready"] = robot_ready_from(self.robot, self.target)
            except OSError as e:
                self.robot = {"reachable": False, "checked": now, "error": str(e) or type(e).__name__,
                              "ready": f"not reachable at {self.host}"}
            time.sleep(2)

    def capture_status(self) -> dict:
        """webcams.py heartbeat: file next to frames/ (same computer) or POSTed (other computer)."""
        cands = []
        try:
            cands.append(json.loads(self.capture_file.read_text()))
        except (OSError, ValueError):
            pass
        if self.capture_posted:
            cands.append(self.capture_posted)
        if not cands:
            return {"running": False, "why": "capture script never started"}
        st = max(cands, key=lambda c: c.get("heartbeat") or "")
        try:
            age = (datetime.now() - datetime.fromisoformat(st["heartbeat"])).total_seconds()
        except (KeyError, ValueError):
            age = 1e9
        return {**st, "running": age < 10, "age_s": round(age),
                "why": None if age < 10 else "capture script not running"}

    def save_state(self):
        tmp = self.state_file.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.state, indent=2))
        tmp.replace(self.state_file)

    # -- status
    def job_progress(self) -> dict:
        j = self.job
        lines = j["log"].read_text(errors="ignore").splitlines() if j["log"].exists() else []
        done = total = 0
        for ln in reversed(lines):
            m = DOT_RE.search(ln)
            if m:
                done, total = int(m.group(1)), int(m.group(2))
                break
        elapsed = time.time() - j["started"]
        est = j.get("estimate_s")
        left = max(est - elapsed, 0) if est else None
        if done and total and elapsed > 30:                  # measured pace beats the estimate
            left = elapsed / done * (total - done)
        return {"kind": j["kind"], "cam": j.get("cam"), "frame": j.get("frame"), "done": done, "total": total,
                "elapsed_s": round(elapsed), "left_s": None if left is None else round(left),
                "log": "\n".join(lines[-12:])}

    def status(self) -> dict:
        cams = []
        busy = self.job["cam"] if self.job and self.job["kind"] == "engrave" else None
        for d in cam_dirs(self.frames):
            fl = frame_list(d)
            latest = fl[-1] if fl else None
            st = self.state["cams"].get(d.name, {})
            if busy == d.name:
                state = "in progress"
            elif latest is None:
                state = "no capture"
            elif st.get("engraved_frame") == latest.name:
                state = "engraved"
            else:
                state = "to be engraved"
            cams.append({"cam": d.name, "captures": len(fl), "state": state,
                         "latest": latest.name if latest else None,
                         "captured_at": captured_at(latest) if latest else None,
                         "engraved_frame": st.get("engraved_frame"), "engraved_at": st.get("engraved_at"),
                         "engraved_captured_at": st.get("engraved_captured_at"),
                         "error": st.get("error"), "on_canvas": self.state.get("on_canvas") == d.name})
        cap = self.capture_status()
        for c in cams:
            c["fetch"] = (cap.get("cams") or {}).get(c["cam"])
        return {"target": self.target, "host": self.host, "extra": self.extra, "cams": cams,
                "robot": self.robot, "capture": cap,
                "job": self.job_progress() if self.job else None,
                "last_job": self.state.get("last_job")}

    # -- preview (in-process, cached per frame)
    def preview(self, cam: str, frame: str) -> dict:
        f_setup = HERE / f"wax_setup.{self.target}.json"
        key = f"{cam}/{frame}/{f_setup.stat().st_mtime if f_setup.exists() else 0}"   # new settings -> new preview
        if key in self.previews:
            return self.previews[key]
        path = self.frame_path(cam, frame)
        import draw_wax as dw
        from ur5e import Workspace
        ap = dw.build_parser()
        f = HERE / f"wax_setup.{self.target}.json"
        draw = json.loads(f.read_text()).get("draw", {}) if f.exists() else {}
        known = {a.dest for a in ap._actions}
        ap.set_defaults(**{k: v for k, v in draw.items() if k in known})
        args = dw.apply_setup(ap.parse_args(["--target", self.target, "--image", str(path),
                                             "--no-heat", *self.extra]))
        ws = Workspace.load(args.workspace)
        gray, _ = dw.load_picture(args)
        canvas = dw.Canvas(ws, gray.shape[0] / gray.shape[1], args.margin_mm, args.size_mm)
        if args.lift_mm is None:
            args.lift_mm = 3.0 if args.style == "dots" else 8.0
        marks, w_mm, h_mm, detail, too_many = dw.make_marks(gray, canvas.width_mm, args)
        svg = PREVIEWS / f"{cam}_{Path(frame).stem}_{abs(hash(key)) % 10**8}.svg"
        mono_svg(marks, w_mm, h_mm, svg, dot_r_mm=args.dot_mm * 0.4)
        p = {"svg": f"/preview/{svg.name}", "marks": len(marks), "detail": detail,
             "size_mm": [round(w_mm), round(h_mm)], "estimate_s": round(dw.estimate_s(marks, args)),
             "too_many": (f"{len(marks)} marks > max {args.max_marks}: {too_many}"
                          if len(marks) > args.max_marks else None) if marks else "nothing to engrave"}
        self.previews[key] = p
        return p

    def frame_path(self, cam: str, frame: str) -> Path:
        d = self.frames / cam
        p = d / frame
        if not re.fullmatch(r"cam\d+", cam) or p.parent != d or not p.is_file():
            raise ValueError(f"no frame {cam}/{frame}")
        return p

    # -- jobs
    def _spawn(self, kind: str, cmd: list[str], **info):
        log = JOBS / f"{datetime.now():%Y%m%d_%H%M%S}_{kind}{'_' + info['cam'] if 'cam' in info else ''}.log"
        fh = open(log, "w")
        fh.write("$ " + " ".join(cmd) + "\n\n")
        fh.flush()
        proc = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, cwd=HERE.parents[2],
                                env={**os.environ, "PYTHONUNBUFFERED": "1"}, start_new_session=True)
        self.job = {"kind": kind, "proc": proc, "log": log, "started": time.time(), **info}
        threading.Thread(target=self._wait, args=(self.job, fh), daemon=True).start()

    def _wait(self, job: dict, fh):
        code = job["proc"].wait()
        fh.close()
        text = job["log"].read_text(errors="ignore")
        with self.lock:
            ok = code == 0 and not job.get("stopped")
            why = None
            if not ok:
                why = "stopped from the interface" if job.get("stopped") else job_error(text, code)
            self.state["last_job"] = {"kind": job["kind"], "cam": job.get("cam"), "ok": ok, "error": why,
                                      "log": job["log"].name, "ended": datetime.now().isoformat(timespec="seconds")}
            if job["kind"] == "engrave":
                st = self.state["cams"].setdefault(job["cam"], {})
                if ok:
                    st.update(engraved_frame=job["frame"], engraved_captured_at=job["captured_at"],
                              engraved_at=datetime.now().isoformat(timespec="seconds"), error=None)
                    self.state["on_canvas"] = job["cam"]
                else:
                    st["error"] = why
                    self.state["on_canvas"] = None          # partly engraved
            self.save_state()
            self.job = None

    def engrave(self, cam: str, frame: str) -> dict:
        path = self.frame_path(cam, frame)
        with self.lock:
            if self.job:
                return {"error": f"busy: {self.job['kind']} {self.job.get('cam') or ''} is running"}
            p = self.preview(cam, frame)
            if p["too_many"]:
                return {"error": p["too_many"]}
            why = robot_ready(self.host, self.target)
            if why:
                return {"error": f"robot not ready: {why}"}
            cmd = [sys.executable, str(HERE / "draw_wax.py"), "--target", self.target,
                   "--image", str(path), "--no-heat", *self.extra]
            self._spawn("engrave", cmd, cam=cam, frame=frame, captured_at=captured_at(path),
                        estimate_s=p["estimate_s"])
            self.state["cams"].setdefault(cam, {})["error"] = None
            return {"ok": True}

    def lift_home(self) -> dict:
        with self.lock:
            if self.job:
                return {"error": "busy — stop the running job first"}
            why = robot_ready(self.host, self.target)
            if why:
                return {"error": f"robot not ready: {why}"}
            self._spawn("lift", [sys.executable, str(HERE / "lift_home.py"), "--target", self.target])
            return {"ok": True}

    def stop(self) -> dict:
        try:
            dashboard(self.host, "stop")                   # halts the robot program right away
        except OSError as e:
            print(f"dashboard stop failed: {e}")
        with self.lock:
            j = self.job
        if j:
            j["stopped"] = True
            try:
                os.killpg(j["proc"].pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        return {"ok": True, "note": "stopped — if the tip is in the wax, use Lift & home"}

    def upload(self, name: str, data: bytes) -> dict:
        """POST from webcams.py (WAX_URL): X-Filename 'cam2_20261125_113030.png'."""
        m = re.match(r"(cam\d+)", name)
        if not m:
            return {"error": f"filename must start with camN: {name}"}
        suffix = Path(name).suffix.lower()
        if suffix not in IMAGE_SUFFIXES or not data.startswith((b"\x89PNG", b"\xff\xd8")):
            return {"error": "not a PNG/JPEG image"}
        d = self.frames / m.group(1)
        d.mkdir(parents=True, exist_ok=True)
        out = d / (datetime.now().strftime("frame_%Y%m%d_%H%M%S_%f")[:-3] + suffix)
        out.write_bytes(data)
        return {"ok": True, "name": f"{d.name}/{out.name}"}


# ---------------------------------------------------------------- http

class Handler(BaseHTTPRequestHandler):
    station: Station

    def log_message(self, fmt, *a):
        if "/api/status" not in (a[0] if a else ""):
            sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % a))

    def send(self, code: int, body: bytes, ctype: str):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def json(self, obj, code: int = 200):
        self.send(code, json.dumps(obj).encode(), "application/json")

    def do_GET(self):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        st = self.station
        try:
            if u.path in ("/", "/index.html"):
                return self.send(200, WEB.read_bytes(), "text/html; charset=utf-8")
            if u.path == "/api/status":
                return self.json(st.status())
            if u.path == "/api/preview":
                return self.json(st.preview(q["cam"], q["frame"]))
            m = re.fullmatch(r"/frame/(cam\d+)/([^/]+)", u.path)
            if m:
                p = st.frame_path(*m.groups())
                ctype = "image/png" if p.suffix.lower() == ".png" else "image/jpeg"
                return self.send(200, p.read_bytes(), ctype)
            m = re.fullmatch(r"/preview/([\w.-]+\.svg)", u.path)
            if m and (PREVIEWS / m.group(1)).is_file():
                return self.send(200, (PREVIEWS / m.group(1)).read_bytes(), "image/svg+xml")
            return self.json({"error": "not found"}, 404)
        except (ValueError, KeyError) as e:
            return self.json({"error": str(e)}, 400)
        except Exception as e:                                # preview of an odd image etc.
            return self.json({"error": f"{type(e).__name__}: {e}"}, 500)

    def do_POST(self):
        st = self.station
        n = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(n) if n else b""
        try:
            if self.path.startswith("/api/upload"):
                return self.json(st.upload(Path(self.headers.get("X-Filename", "")).name, body))
            req = json.loads(body or b"{}")
            if self.path == "/api/capture_status":
                st.capture_posted = req
                return self.json({"ok": True})
            if self.path == "/api/engrave":
                return self.json(st.engrave(req["cam"], req["frame"]))
            if self.path == "/api/stop":
                return self.json(st.stop())
            if self.path == "/api/lift_home":
                return self.json(st.lift_home())
            return self.json({"error": "not found"}, 404)
        except (ValueError, KeyError) as e:
            return self.json({"error": str(e)}, 400)
        except SystemExit as e:                               # draw_wax checks raise SystemExit
            return self.json({"error": str(e)}, 400)


def main() -> None:
    argv = sys.argv[1:]
    extra = argv[argv.index("--") + 1:] if "--" in argv else []
    argv = argv[:argv.index("--")] if "--" in argv else argv
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", choices=("sim", "real"), default="sim")
    ap.add_argument("--frames", type=Path, default=HERE / "webcam-buttons" / "frames")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--bind", default="0.0.0.0")
    args = ap.parse_args(argv)
    os.environ.setdefault("TZ", "Europe/Zurich")      # the container runs in UTC; frame names are local time
    time.tzset()
    sys.path.insert(0, str(HERE))
    from ur5e.config import resolve_host
    host = resolve_host(args.target)
    args.frames.mkdir(parents=True, exist_ok=True)
    Handler.station = Station(args.target, args.frames, host, extra)
    srv = ThreadingHTTPServer((args.bind, args.port), Handler)
    print(f"engraving station '{args.target}' (robot {host}), frames {args.frames}"
          + (f", extra draw_wax flags: {' '.join(extra)}" if extra else ""))
    print(f"open http://localhost:{args.port}   — Ctrl-C to stop the server (also stops a running job)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if Handler.station.job:
            Handler.station.stop()


if __name__ == "__main__":
    main()
