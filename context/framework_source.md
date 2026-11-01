# ur5e framework — source snapshot

Generated 2026-10-29T10:10Z from git 3fa684c
by context/make_source_snapshot.sh. The files below are the student-facing
framework (src/ur5e/, without the viewer) and the function templates, verbatim.
When this snapshot and the repo disagree, the repo is right — regenerate.

---

## `src/ur5e/__init__.py`

```python
"""ur5e — UR5e teaching framework (ZHdK Embodied Interaction HS26).

Student-facing surface: ``Robot``, ``Workspace``, ``grid_poses``, ``line_poses``
(+ ``ur5e.cli`` for the examples' ``--target`` flag) — validated for student use
in simulation (HS26). Real-arm motion stays gated (see motion.py); the examples
open the gate only with an explicit ``--target real``.
"""

from __future__ import annotations

from .connection import RobotConnection
from .config import resolve_host
from .robot import Robot, grid_poses, line_poses
from .workspace import Workspace

__all__ = ["Robot", "Workspace", "grid_poses", "line_poses",
           "RobotConnection", "resolve_host", "connect"]


def connect(target_or_host: str, **kwargs) -> RobotConnection:
    """Connect by named target ('sim'/'real') or a raw host/IP.

    Receive-only unless control=True / io=True are passed through.
    Returns an already-connected RobotConnection.
    """
    host = resolve_host(target_or_host) if target_or_host in ("sim", "real") else target_or_host
    return RobotConnection(host, **kwargs).connect()

```

---

## `src/ur5e/robot.py`

```python
"""Student-facing robot API — the base functions student projects build on.

STATUS: validated for student use in simulation (HS26). Real-arm motion is gated:
target="real" needs allow_real=True, a validated profile and set_tool() first.

This is the only module students should need. It wraps the lower-level
connection / safety / motion / IO plumbing behind a small, friendly API:

    from ur5e import Robot, grid_poses

    with Robot(target="sim") as robot:
        here = robot.pose()
        for p in grid_poses(here, nx=3, ny=3, dx=0.04, dy=0.04):
            robot.move_to(p)                  # validated Cartesian move
            v = robot.read_analog_in(0)       # sensor
            if v > 0.5:
                robot.set_digital_out(0, True)   # actuator
                robot.set_digital_out(0, False)

Safety: all motion is validated against a safety profile and is sim-by-default.
Driving the real arm requires Robot(target="real", allow_real=True) with a
*validated* profile and the tool declared via set_tool(...). See safety.py — the
robot's PolyScope Safety Configuration remains the authoritative layer.

This file is meant to GROW: add reusable base functions here as the module
develops, keeping signatures simple and well-documented for students.
"""

from __future__ import annotations

from typing import Iterable, Sequence

from .config import resolve_host
from .connection import RobotConnection
from .motion import MotionController
from .safety import SafetyLimits, get_profile
from .workspace import Workspace


class Robot:
    """High-level UR5e handle. Use as a context manager."""

    def __init__(self, target: str = "sim", *, profile=None,
                 allow_real: bool = False, host: str | None = None,
                 payload_kg: float | None = None, tcp: Sequence[float] | None = None):
        """target 'sim' | 'real'; profile: name, SafetyLimits or Workspace;
        payload_kg / tcp: if given, set_tool() is called right after connecting
        (what the examples do for --target real)."""
        self.host = host or resolve_host(target)
        self.target = target
        self._tool = (payload_kg, tuple(tcp) if tcp is not None else (0, 0, 0, 0, 0, 0))
        if profile is None:
            profile = "sim_open" if target == "sim" else "cart_against_wall"
        if isinstance(profile, Workspace):          # a taught workspace -> its profile
            profile = profile.profile(target)
        self.limits: SafetyLimits = get_profile(profile) if isinstance(profile, str) else profile
        self._allow_real = allow_real
        self.conn = RobotConnection(self.host, control=True, io=True)
        self.motion: MotionController | None = None

    # --- lifecycle ---
    def connect(self) -> "Robot":
        self.conn.connect()
        self.motion = MotionController(self.conn, self.limits, allow_real=self._allow_real)
        payload_kg, tcp = self._tool
        if payload_kg is not None:
            self.set_tool(payload_kg, tcp)
        if self.target == "real" or (self._allow_real and not self.motion.is_sim):
            print(f"[ur5e] REAL ARM {self.host} — profile '{self.limits.name}', "
                  f"payload {payload_kg if payload_kg is not None else 'not set'} kg, "
                  f"TCP {tuple(round(v, 4) for v in tcp) if payload_kg is not None else 'not set'}")
        return self

    def disconnect(self) -> None:
        self.conn.disconnect()
        self.motion = None

    def __enter__(self) -> "Robot":
        return self.connect()

    def __exit__(self, *exc) -> bool:
        self.disconnect()
        return False

    # --- tool / payload (required before real-arm motion) ---
    def set_tool(self, payload_kg: float, tcp: Sequence[float] = (0, 0, 0, 0, 0, 0)) -> None:
        """Declare the mounted tool: payload mass (kg) and TCP offset
        [x, y, z, rx, ry, rz] (m, rad). Required before driving the real arm."""
        self.conn.set_payload(payload_kg)
        self.conn.set_tcp(tcp)

    # --- state ---
    def joints(self):
        """Current joint angles (rad)."""
        return self.conn.receive.getActualQ()

    def pose(self):
        """Current TCP pose [x, y, z, rx, ry, rz] (m, rad)."""
        return self.conn.receive.getActualTCPPose()

    # --- motion (validated against the safety profile) ---
    def move_joints(self, q, speed=None, accel=None):
        return self.motion.move_j(q, speed, accel)

    def move_to(self, pose, speed=None, accel=None):
        """Linear Cartesian move to a pose."""
        return self.motion.move_l(pose, speed, accel)

    def move_through(self, poses: Iterable, speed=None, accel=None) -> None:
        """Move through a sequence of poses (a path or grid)."""
        for p in poses:
            self.motion.move_l(p, speed, accel)

    # --- sensors (read) ---
    def read_digital_in(self, pin: int) -> bool:
        bits = self.conn.receive.getActualDigitalInputBits()
        return bool((bits >> pin) & 1)

    def read_analog_in(self, pin: int) -> float:
        if pin == 0:
            return self.conn.receive.getStandardAnalogInput0()
        if pin == 1:
            return self.conn.receive.getStandardAnalogInput1()
        raise ValueError("standard analog input pin must be 0 or 1")

    # --- actuator (write) ---
    def set_digital_out(self, pin: int, value: bool):
        return self.conn.io.setStandardDigitalOut(pin, bool(value))

    def set_tool_digital_out(self, pin: int, value: bool):
        return self.conn.io.setToolDigitalOut(pin, bool(value))

    def set_analog_out(self, pin: int, voltage: float):
        return self.conn.io.setAnalogOutputVoltage(pin, voltage)


# --- pattern generators -----------------------------------------------------
# Reusable path/grid helpers. Grow this section as processes need new patterns.

def grid_poses(origin, nx: int, ny: int, dx: float, dy: float):
    """Grid of poses stepping in base X/Y from an origin pose, keeping the
    origin's orientation and z. Snake (boustrophedon) order so the path doesn't
    jump back at the start of each row.

    origin: [x, y, z, rx, ry, rz]. Returns a list of poses.
    """
    ox, oy, oz, rx, ry, rz = origin
    poses = []
    for j in range(ny):
        cols = range(nx) if j % 2 == 0 else range(nx - 1, -1, -1)
        for i in cols:
            poses.append([ox + i * dx, oy + j * dy, oz, rx, ry, rz])
    return poses


def line_poses(start, end_xyz, n: int):
    """n poses evenly spaced from `start` to `end_xyz`, keeping start's
    orientation. start: full pose; end_xyz: [x, y, z]."""
    sx, sy, sz, rx, ry, rz = start
    ex, ey, ez = end_xyz
    if n < 1:
        return []
    out = []
    for k in range(n):
        t = k / (n - 1) if n > 1 else 0.0
        out.append([sx + (ex - sx) * t, sy + (ey - sy) * t, sz + (ez - sz) * t, rx, ry, rz])
    return out

```

---

## `src/ur5e/cli.py`

```python
"""Command-line plumbing shared by the examples: ``--target sim|real`` and friends.

STATUS: validated for student use (HS26) — thin argparse glue, no robot logic.

Every example script does::

    from ur5e import Robot, Workspace
    from ur5e.cli import argument_parser, robot_kwargs

    args = argument_parser(__doc__).parse_args()
    ws = Workspace.load(args.workspace)
    with Robot(**robot_kwargs(args), profile=ws) as robot:
        ...

so the same file runs in the simulator (default) and, in the lab, on the real
arm with ``--target real``.  ``--target real`` is the conscious act that enables
real-arm motion: it sets ``allow_real=True`` and declares the tool
(``--payload`` kg, ``--tcp`` x y z rx ry rz; defaults = nothing mounted, TCP at
the flange).  The host comes from ``UR_HOST`` (lab: 192.168.1.11) or ``--host``.
"""

from __future__ import annotations

import argparse
import os


def argument_parser(description: str | None = None) -> argparse.ArgumentParser:
    """ArgumentParser with the standard example options. Add your own on top."""
    ap = argparse.ArgumentParser(description=description,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", choices=("sim", "real"), default="sim",
                    help="sim (default) or real = the lab arm, supervised (implies allow_real)")
    ap.add_argument("--host", default=None,
                    help="override the robot IP/hostname (default: URSIM_HOST / UR_HOST)")
    ap.add_argument("--workspace", default=None, metavar="FILE",
                    help="workspace file (default: workspace.json or $UR5E_WORKSPACE)")
    ap.add_argument("--payload", type=float, default=0.0, metavar="KG",
                    help="real only: mass of the mounted tool in kg (default 0.0 = nothing mounted)")
    ap.add_argument("--tcp", type=float, nargs=6, default=(0, 0, 0, 0, 0, 0),
                    metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                    help="real only: TCP offset in m / rad, as on the pendant (default: flange)")
    return ap


def robot_kwargs(args) -> dict:
    """Keyword arguments for ``Robot(...)`` from parsed arguments."""
    kw = {"target": args.target, "host": args.host}
    if args.target == "real":
        if not args.host and not os.environ.get("UR_HOST"):
            raise SystemExit("--target real needs the robot's address: start the shell with\n"
                             "  docker compose run --rm --no-deps -e UR_HOST=192.168.1.11 dev bash\n"
                             "or pass --host 192.168.1.11")
        kw.update(allow_real=True, payload_kg=args.payload, tcp=tuple(args.tcp))
    return kw

```

---

## `src/ur5e/workspace.py`

```python
"""Taught workspace — the box the examples scale to.

STATUS: HS26 student framework (sim-validated).

A *workspace* is the part of the table the robot may use, taught in four
steps by moving the arm there by hand (freedrive button on the pendant) or,
in the simulator, with the pendant's Move tab — the teach script only READS
poses (receive-only), it never moves the arm:

    1. corner A   — one corner of the work area, tool touching the surface
    2. corner B   — the diagonally opposite corner, on the surface
    3. top        — the highest point the tool should ever reach
    4. home       — a comfortable pose above the area; examples start/end here

Saved as ``workspace.json`` (git-ignored, one per setup). Examples address the
box in FRACTIONS, so the same script runs on any table size:

    ws = Workspace.load()
    ws.pose(0.5, 0.5, 0.3)   # centre of the area, 30 % of the way up
    ws.grid(3, 3, w=0.1)     # 3 x 3 cells, 10 % above the surface

Safety (flagged): the taught box is turned into a SafetyLimits profile with
``ws.profile(target)``. On the REAL arm it is the INTERSECTION of the taught
box with the measured cell profile ``cart_against_wall`` — students can only
shrink the space, never enlarge it — and the profile's speed ceilings are
kept. A box taught in the sim is refused for the real arm (and vice versa).
As everywhere in this framework, PolyScope's own safety configuration stays
the authoritative layer; this is the secondary software guard.

Assumptions (flagged): poses are TCP poses as the controller reports them, so
the box is in terms of the pendant's ACTIVE TCP at teaching time. The joint
check in the safety layer uses the flange position — with a long tool the
flange sits above the TCP; teach ``top`` and ``home`` with that headroom.
"""

from __future__ import annotations

import dataclasses
import json
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Sequence

from .safety import Box, SafetyLimits, get_profile

FORMAT = "ur5e-workspace/1"
DEFAULT_FILE = "workspace.json"
MIN_SIDE_M = 0.02          # a taught box must be at least 20 mm in x, y and z


class WorkspaceError(Exception):
    """Workspace file missing/invalid, or used for the wrong target."""


@dataclass
class Workspace:
    target: str                      # "sim" or "real" — where it was taught
    corner_a: List[float]            # TCP pose [x,y,z,rx,ry,rz] (m, rad)
    corner_b: List[float]
    top: List[float]
    home: List[float]                # TCP pose at home
    home_joints: List[float]         # joint angles at home (rad) — what go_home() uses
    taught_at: str = ""
    host: str = ""
    note: str = ""
    path: str = field(default="", compare=False)

    # ----------------------------------------------------------------- box --
    @property
    def box(self) -> Box:
        """Axis-aligned bounding box of the four taught points (base frame)."""
        pts = [self.corner_a, self.corner_b, self.top, self.home]
        xs, ys, zs = ([p[i] for p in pts] for i in range(3))
        return Box(x=(min(xs), max(xs)), y=(min(ys), max(ys)), z=(min(zs), max(zs)))

    @property
    def orientation(self) -> List[float]:
        """Tool orientation used for every generated pose: the one taught at
        corner A (rotation vector, rad). Teach with the tool pointing the way
        you want it to work."""
        return list(self.corner_a[3:6])

    @property
    def size_mm(self):
        b = self.box
        return tuple(round((hi - lo) * 1000) for lo, hi in (b.x, b.y, b.z))

    @property
    def usable_box(self) -> Box:
        """The box poses are generated in: the taught box intersected with the
        cell profile on the real arm (= what Robot(profile=ws) enforces). Same as
        `box` in the sim or when the taught box lies inside the cell."""
        try:
            return self.profile(self.target).workspace
        except WorkspaceError:
            return self.box

    # ------------------------------------------------------------ geometry --
    def pose(self, u: float, v: float, w: float) -> List[float]:
        """Pose at fractions of the box: u along x, v along y, w along z, each
        0.0–1.0 (0 = low side of the box). Orientation = taught orientation."""
        for name, val in (("u", u), ("v", v), ("w", w)):
            if not 0.0 <= val <= 1.0:
                raise ValueError(f"{name}={val} is outside 0..1 — fractions of the workspace box")
        b = self.usable_box

        def along(lo_hi, f):
            lo, hi = lo_hi
            return min(max(lo + f * (hi - lo), lo), hi)   # clamp: u=1.0 lands exactly on the edge

        return [along(b.x, u), along(b.y, v), along(b.z, w), *self.orientation]

    def grid(self, nx: int, ny: int, w: float = 0.1, margin: float = 0.05) -> List[List[float]]:
        """nx x ny poses spread over the area at height fraction w, keeping a
        `margin` fraction away from the box edges. Snake order (row by row,
        alternating direction) like ur5e.grid_poses."""
        if nx < 1 or ny < 1:
            raise ValueError("nx and ny must be >= 1")
        us = [margin + (1 - 2 * margin) * (i / (nx - 1) if nx > 1 else 0.5) for i in range(nx)]
        vs = [margin + (1 - 2 * margin) * (j / (ny - 1) if ny > 1 else 0.5) for j in range(ny)]
        poses = []
        for j, v in enumerate(vs):
            row = us if j % 2 == 0 else list(reversed(us))
            poses.extend(self.pose(u, v, w) for u in row)
        return poses

    def line(self, start_uvw: Sequence[float], end_uvw: Sequence[float], n: int) -> List[List[float]]:
        """n poses evenly spaced from one fractional point to another."""
        if n < 1:
            return []
        out = []
        for k in range(n):
            t = k / (n - 1) if n > 1 else 0.0
            uvw = [a + (b - a) * t for a, b in zip(start_uvw, end_uvw)]
            out.append(self.pose(*uvw))
        return out

    # -------------------------------------------------------------- safety --
    def profile(self, target: str) -> SafetyLimits:
        """SafetyLimits for `target` whose workspace box is this workspace —
        clamped to the cell profile on the real arm (never larger than
        cart_against_wall); sim_open's speed guards in the sim."""
        if target != self.target:
            raise WorkspaceError(
                f"this workspace was taught on target '{self.target}' but you are "
                f"connecting to '{target}'. Teach a workspace on the real setup "
                f"before using it there (examples/function_templates/00_teach_workspace.py).")
        base = get_profile("cart_against_wall" if target == "real" else "sim_open")
        b, c = self.box, base.workspace
        clamped = Box(x=(max(b.x[0], c.x[0]), min(b.x[1], c.x[1])),
                      y=(max(b.y[0], c.y[0]), min(b.y[1], c.y[1])),
                      z=(max(b.z[0], c.z[0]), min(b.z[1], c.z[1])))
        for axis, (lo, hi) in zip("xyz", (clamped.x, clamped.y, clamped.z)):
            if hi - lo < MIN_SIDE_M:
                raise WorkspaceError(
                    f"taught workspace does not overlap the '{base.name}' cell profile "
                    f"along {axis} (taught {getattr(b, axis)}, cell {getattr(c, axis)}). "
                    f"Teach again inside the measured cell.")
        return dataclasses.replace(base, name=f"workspace:{Path(self.path).name or 'taught'}",
                                   workspace=clamped)

    # ---------------------------------------------------------- persistence --
    @classmethod
    def from_points(cls, target: str, corner_a, corner_b, top, home, home_joints,
                    host: str = "", note: str = "") -> "Workspace":
        """Build + validate from the four taught poses. Raises WorkspaceError
        with a plain message if the box is degenerate."""
        ws = cls(target=target, corner_a=list(corner_a), corner_b=list(corner_b),
                 top=list(top), home=list(home), home_joints=list(home_joints),
                 taught_at=time.strftime("%Y-%m-%d %H:%M"), host=host, note=note)
        if target not in ("sim", "real"):
            raise WorkspaceError(f"target must be 'sim' or 'real', not {target!r}")
        for name, p in (("corner_a", corner_a), ("corner_b", corner_b), ("top", top), ("home", home)):
            if len(p) != 6:
                raise WorkspaceError(f"{name}: expected a 6-element pose, got {len(p)}")
        if len(home_joints) != 6:
            raise WorkspaceError("home_joints: expected 6 joint angles")
        b = ws.box
        for axis, (lo, hi) in zip("xyz", (b.x, b.y, b.z)):
            if hi - lo < MIN_SIDE_M:
                raise WorkspaceError(
                    f"the taught points span only {round((hi - lo) * 1000)} mm along {axis} "
                    f"(need >= {MIN_SIDE_M * 1000:.0f} mm). Did the arm actually move between steps?")
        surface = min(corner_a[2], corner_b[2])
        if top[2] <= surface + MIN_SIDE_M:
            raise WorkspaceError("'top' must be clearly above the surface corners")
        return ws

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        d.pop("path", None)
        return {"format": FORMAT, "units_note": "poses [x,y,z,rx,ry,rz] in metres / rotation-vector rad, robot Base frame; joints rad", **d}

    def save(self, path: str | os.PathLike = DEFAULT_FILE) -> Path:
        p = Path(path)
        tmp = p.with_suffix(p.suffix + ".tmp")
        tmp.write_text(json.dumps(self.to_dict(), indent=2))
        tmp.replace(p)
        self.path = str(p)
        return p

    @classmethod
    def load(cls, path: str | os.PathLike | None = None) -> "Workspace":
        """Load workspace.json (or $UR5E_WORKSPACE / an explicit path)."""
        p = Path(path or os.environ.get("UR5E_WORKSPACE", DEFAULT_FILE))
        if not p.exists():
            raise WorkspaceError(
                f"no workspace file at {p.resolve()} — teach one first:\n"
                f"    python examples/function_templates/00_teach_workspace.py --target sim\n"
                f"(or --demo for a ready-made sim workspace)")
        try:
            d = json.loads(p.read_text())
        except json.JSONDecodeError as e:
            raise WorkspaceError(f"{p}: not valid JSON ({e})") from None
        if d.get("format") != FORMAT:
            raise WorkspaceError(f"{p}: unknown format {d.get('format')!r}, expected {FORMAT!r}")
        try:
            ws = cls(target=d["target"], corner_a=d["corner_a"], corner_b=d["corner_b"],
                     top=d["top"], home=d["home"], home_joints=d["home_joints"],
                     taught_at=d.get("taught_at", ""), host=d.get("host", ""),
                     note=d.get("note", ""), path=str(p))
        except KeyError as e:
            raise WorkspaceError(f"{p}: missing field {e}") from None
        return ws

    def describe(self) -> str:
        b = self.box
        sx, sy, sz = self.size_mm
        out = (f"workspace '{self.path or '(unsaved)'}' taught on {self.target} {self.taught_at}\n"
               f"  box x {b.x[0]:+.3f}..{b.x[1]:+.3f}  y {b.y[0]:+.3f}..{b.y[1]:+.3f}  "
               f"z {b.z[0]:+.3f}..{b.z[1]:+.3f} m   ({sx} x {sy} x {sz} mm)\n"
               f"  home joints [deg]: {[round(q * 57.2958, 1) for q in self.home_joints]}")
        # On the real arm the usable box is the intersection with the cell profile:
        # say so when that clips anything, naming the taught points that fall outside.
        try:
            c = self.profile(self.target).workspace
        except WorkspaceError as e:
            return out + f"\n  !! {e}"
        if (c.x, c.y, c.z) != (b.x, b.y, b.z):
            outside = [n for n in ("corner_a", "corner_b", "top", "home")
                       if not c.contains(getattr(self, n)[:3])]
            out += (f"\n  !! clipped to the cell profile -> usable x {c.x[0]:+.3f}..{c.x[1]:+.3f}  "
                    f"y {c.y[0]:+.3f}..{c.y[1]:+.3f}  z {c.z[0]:+.3f}..{c.z[1]:+.3f} m "
                    f"(ws.pose/grid/line use this box)"
                    + (f"\n  !! taught points outside the cell: {', '.join(outside)} — "
                       f"move_joints(home_joints) / moves to them are rejected; teach again inside"
                       if outside else ""))
        return out


def demo_sim_workspace(start_pose: Sequence[float], start_joints: Sequence[float]) -> Workspace:
    """A ready-made workspace around the simulator's start pose, for trying
    the templates without teaching: a 300 x 300 mm area, 200 mm high, with the
    start pose as home in the middle above it. Sim only."""
    x, y, z = start_pose[0], start_pose[1], start_pose[2]
    rot = list(start_pose[3:6])
    surface = z - 0.10
    a = [x - 0.15, y - 0.15, surface, *rot]
    b = [x + 0.15, y + 0.15, surface, *rot]
    top = [x, y, surface + 0.20, *rot]
    return Workspace.from_points("sim", a, b, top, list(start_pose), list(start_joints),
                                 note="demo workspace around the URSim start pose (not taught)")

```

---

## `src/ur5e/safety.py`

```python
"""Safety limits — the framework's SECONDARY software guardrail.

STATUS: HS26 student framework, lower layer (sim-validated; cart_against_wall box
captured on the real cell).

IMPORTANT: this is NOT the robot's safety system. The authoritative, certified
safety layer is the robot's PolyScope Safety Configuration (joint limits, TCP
speed/force limits, safety planes). These software limits exist to catch
mistakes earlier and must always be set *more conservative than* the controller
configuration. They never replace it.

Profiles are swappable by name (like the host parameter) so the same code adapts
to different cell setups (cart against a wall now; 360°/truss later).

`validated=False` means the profile's geometry has not been confirmed against the
real cell (e.g. cart-top height not yet measured). Real-arm motion is refused
until a profile is validated; sim motion is allowed (geometry checks still run).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .kinematics import fk_tcp_position


class SafetyViolation(Exception):
    """Raised when a requested motion would exceed the active safety limits."""


@dataclass(frozen=True)
class Box:
    """Axis-aligned workspace box in the robot base frame (meters)."""
    x: tuple[float, float]
    y: tuple[float, float]
    z: tuple[float, float]

    def contains(self, p, tol: float = 0.0) -> bool:
        """True if p=(x,y,z) lies inside the box, grown by `tol` metres on every side."""
        return (self.x[0] - tol <= p[0] <= self.x[1] + tol
                and self.y[0] - tol <= p[1] <= self.y[1] + tol
                and self.z[0] - tol <= p[2] <= self.z[1] + tol)


# Joint targets are checked through the NOMINAL UR5e kinematics (kinematics.py),
# the controller uses its calibrated model: the two differ by up to a few mm
# (measured 2026-10-29 on the lab arm: 0.2-1.0 mm). A point taught as a box corner
# therefore sits exactly on the edge and would be rejected without this slack.
# Cartesian targets (move_to) are controller coordinates and get no tolerance.
FK_TOLERANCE_M = 0.005


@dataclass(frozen=True)
class SafetyLimits:
    name: str
    # Cartesian ceilings
    tcp_speed_max: float          # m/s   (moveL)
    tcp_accel_max: float          # m/s^2
    # Joint ceilings
    joint_speed_max: float        # rad/s (moveJ)
    joint_accel_max: float        # rad/s^2
    joint_limits: tuple[tuple[float, float], ...]   # per-joint (min,max) rad, len 6
    # Workspace (coarse guard; PolyScope planes are authoritative)
    workspace: Box
    # Payload
    payload_max_kg: float
    # Enforcement (only enforced on the REAL arm; sim is kinematic-only)
    require_payload: bool = True
    require_tcp: bool = True
    # Has this profile's geometry been confirmed against the real cell?
    validated: bool = False

    # --- pure checks: each returns None or raises SafetyViolation ---
    def check_joint_target(self, q) -> None:
        if len(q) != 6:
            raise SafetyViolation(f"expected 6 joint values, got {len(q)}")
        for i, (lo, hi) in enumerate(self.joint_limits):
            if not (lo <= q[i] <= hi):
                raise SafetyViolation(
                    f"joint {i} = {q[i]:.3f} rad outside limit [{lo:.3f}, {hi:.3f}]")
        self.check_workspace(fk_tcp_position(q), tol=FK_TOLERANCE_M, what="joint target (flange FK)")

    def check_pose_target(self, pose) -> None:
        if len(pose) != 6:
            raise SafetyViolation(f"expected 6-element pose, got {len(pose)}")
        self.check_workspace(pose[:3])

    def check_workspace(self, xyz, tol: float = 0.0, what: str = "target") -> None:
        if not self.workspace.contains(xyz, tol):
            raise SafetyViolation(
                f"{what} {tuple(round(float(v), 3) for v in xyz)} outside "
                f"workspace box of profile '{self.name}'")

    def check_joint_speed(self, speed, accel) -> None:
        if not (0 < speed <= self.joint_speed_max):
            raise SafetyViolation(
                f"joint speed {speed} out of (0, {self.joint_speed_max}] rad/s")
        if not (0 < accel <= self.joint_accel_max):
            raise SafetyViolation(
                f"joint accel {accel} out of (0, {self.joint_accel_max}] rad/s^2")

    def check_tcp_speed(self, speed, accel) -> None:
        if not (0 < speed <= self.tcp_speed_max):
            raise SafetyViolation(
                f"tcp speed {speed} out of (0, {self.tcp_speed_max}] m/s")
        if not (0 < accel <= self.tcp_accel_max):
            raise SafetyViolation(
                f"tcp accel {accel} out of (0, {self.tcp_accel_max}] m/s^2")


_TAU = 2 * math.pi
_FULL_JOINTS = tuple((-_TAU, _TAU) for _ in range(6))  # UR5e hardware range

PROFILES = {
    # UR5e on a cart, ~600 x 500 mm work surface in front (wide x depth), table
    # surface at the mounting height (z = 0 in the base frame). Cart against a
    # wall (~180deg). SLOW speeds; tcp_speed_max 0.15 m/s stays under the PolyScope
    # tool-speed limit (0.25 m/s), keeping software stricter than the controller.
    #
    # VALIDATED: x/y of the workspace box captured via scripts/teach_workspace.py
    # against the real cell (front = -X), deliberately conservative (slightly
    # inside the physical surface). z max raised from the captured 0.234 m to
    # 0.60 m on 2026-10-29 (first lab teach: home/top at 0.57 m; nothing above the
    # table within that height). Re-capture / edit if the cell layout changes.
    "cart_against_wall": SafetyLimits(
        name="cart_against_wall",
        tcp_speed_max=0.15,        # 150 mm/s  (< PolyScope tool-speed limit 250 mm/s)
        tcp_accel_max=0.5,
        joint_speed_max=0.5,
        joint_accel_max=0.8,
        joint_limits=_FULL_JOINTS,
        workspace=Box(x=(-0.4853, -0.2013), y=(-0.2859, 0.2401), z=(0.0287, 0.60)),  # x/y captured; z max set 2026-10-29
        payload_max_kg=0.5,
        require_payload=True,
        require_tcp=True,
        validated=True,            # box captured against the real cell (front = -X)
    ),
    # Generous box for free sim exploration: keeps the slow speed/joint guards
    # but won't reject normal reachable poses. Sim use only.
    "sim_open": SafetyLimits(
        name="sim_open",
        tcp_speed_max=0.25,
        tcp_accel_max=0.8,
        joint_speed_max=0.8,
        joint_accel_max=1.2,
        joint_limits=_FULL_JOINTS,
        workspace=Box(x=(-0.9, 0.9), y=(-0.9, 0.9), z=(-0.25, 1.0)),
        payload_max_kg=0.5,
        require_payload=False,
        require_tcp=False,
        validated=False,
    ),
}


def get_profile(name: str) -> SafetyLimits:
    try:
        return PROFILES[name]
    except KeyError:
        raise KeyError(f"unknown safety profile {name!r}; known: {sorted(PROFILES)}") from None

```

---

## `src/ur5e/motion.py`

```python
"""Validated motion primitives.

STATUS: HS26 student framework, lower layer (sim-validated). Students use ur5e.Robot.

Wraps a connected RobotConnection with a SafetyLimits profile. Every motion is
validated against the profile and REJECTED (raises SafetyViolation) on any
violation — never silently clamped, so mistakes surface.

Real-arm gate: motion to a non-sim host requires BOTH allow_real=True AND a
validated profile AND (if the profile requires) payload + TCP already set.
Sim motion runs the geometry/speed checks but skips the real-arm gate.

These software checks are SECONDARY to the robot's PolyScope Safety
Configuration, which is the authoritative protection. See safety.py.
"""

from __future__ import annotations

from . import config
from .connection import RobotConnection
from .safety import SafetyLimits, SafetyViolation


class MotionController:
    def __init__(self, conn: RobotConnection, limits: SafetyLimits, *, allow_real: bool = False):
        if conn.control is None:
            raise RuntimeError("RobotConnection has no control interface (construct with control=True)")
        self.conn = conn
        self.limits = limits
        self.allow_real = allow_real
        self.is_sim = config.is_sim(conn.host)

    def _preflight(self) -> None:
        if not self.conn.connected:
            raise RuntimeError("not connected")
        if self.is_sim:
            return
        # Real-arm gates.
        if not self.allow_real:
            raise SafetyViolation(
                f"refusing real-arm motion to {self.conn.host}: construct "
                "MotionController(..., allow_real=True) to permit hardware motion")
        if not self.limits.validated:
            raise SafetyViolation(
                f"profile '{self.limits.name}' is not validated; confirm cell "
                "geometry (and PolyScope config) and set validated=True first")
        if self.limits.require_payload and not self.conn.payload_set:
            raise SafetyViolation("payload not set; call conn.set_payload(...) before real motion")
        if self.limits.require_tcp and not self.conn.tcp_set:
            raise SafetyViolation("TCP not set; call conn.set_tcp(...) before real motion")

    def move_j(self, q_target, speed=None, accel=None):
        self._preflight()
        speed = self.limits.joint_speed_max if speed is None else speed
        accel = self.limits.joint_accel_max if accel is None else accel
        self.limits.check_joint_speed(speed, accel)
        self.limits.check_joint_target(q_target)
        return self.conn.control.moveJ(list(q_target), speed, accel)

    def move_l(self, pose_target, speed=None, accel=None):
        self._preflight()
        speed = self.limits.tcp_speed_max if speed is None else speed
        accel = self.limits.tcp_accel_max if accel is None else accel
        self.limits.check_tcp_speed(speed, accel)
        self.limits.check_pose_target(pose_target)
        return self.conn.control.moveL(list(pose_target), speed, accel)

```

---

## `src/ur5e/connection.py`

```python
"""RTDE connection layer for the UR5e.

STATUS: HS26 student framework, lower layer (sim-validated). Students use ur5e.Robot.

This is the single sim<->real swap point for the whole framework. The same
object targets URSim or the real arm; only `host` differs. Code written
against this layer runs unchanged in simulation and on hardware because
URSim exposes the identical RTDE ports as the controller.

No safety model is applied here. Payload, TCP, and workspace limits are NOT
set automatically and there are NO silent "safe" defaults. Set payload/TCP
explicitly via set_payload()/set_tcp() once a safety model is defined for
this project. Do not add defaults here before that decision is made.
"""

from __future__ import annotations

import contextlib
from typing import Optional, Sequence

from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
from rtde_io import RTDEIOInterface


class RobotConnection:
    """Lazily-constructed, host-parameterized RTDE connection.

    By default opens RECEIVE-ONLY (safe to inspect state, uploads no script,
    moves nothing). Pass control=True to enable motion commands, io=True for
    digital/analog IO.

    Use as a context manager to guarantee clean teardown:

        with RobotConnection("127.0.0.1") as r:        # receive-only
            print(r.receive.getActualQ())

        with RobotConnection(host, control=True) as r: # motion-capable
            r.set_payload(1.2)                         # explicit assumption
            r.control.moveJ(target_q)
    """

    def __init__(
        self,
        host: str,
        *,
        frequency: float = -1.0,
        control: bool = False,
        io: bool = False,
    ) -> None:
        self.host = host
        self.frequency = frequency
        self._want_control = control
        self._want_io = io
        self.receive: Optional[RTDEReceiveInterface] = None
        self.control: Optional[RTDEControlInterface] = None
        self.io: Optional[RTDEIOInterface] = None
        self._payload_set = False
        self._tcp_set = False

    @property
    def payload_set(self) -> bool:
        return self._payload_set

    @property
    def tcp_set(self) -> bool:
        return self._tcp_set

    # -- lifecycle -------------------------------------------------------
    def connect(self) -> "RobotConnection":
        self.receive = RTDEReceiveInterface(self.host, self.frequency)
        if self._want_control:
            self.control = RTDEControlInterface(self.host, self.frequency)
        if self._want_io:
            self.io = RTDEIOInterface(self.host)
        return self

    @property
    def connected(self) -> bool:
        live = [p for p in (self.receive, self.control, self.io) if p is not None]
        return bool(live) and all(p.isConnected() for p in live)

    def disconnect(self) -> None:
        # Control first: stopScript releases control on the arm before the
        # socket closes, avoiding a dangling control script.
        if self.control is not None:
            with contextlib.suppress(Exception):
                self.control.stopScript()
            with contextlib.suppress(Exception):
                self.control.disconnect()
            self.control = None
        for attr in ("io", "receive"):
            obj = getattr(self, attr)
            if obj is not None:
                with contextlib.suppress(Exception):
                    obj.disconnect()
                setattr(self, attr, None)

    def __enter__(self) -> "RobotConnection":
        return self.connect()

    def __exit__(self, *exc) -> bool:
        self.disconnect()
        return False  # do not suppress exceptions

    # -- explicit, opt-in robot assumptions (NO silent defaults) ---------
    def set_payload(self, mass_kg: float, cog_xyz_m: Sequence[float] = (0.0, 0.0, 0.0)) -> None:
        """ASSUMPTION: caller-supplied payload mass + center of gravity.

        Wrong values produce unsafe motion. There is no default; the caller
        must state the assumption explicitly.
        """
        self._require_control()
        self.control.setPayload(mass_kg, list(cog_xyz_m))
        self._payload_set = True

    def set_tcp(self, pose6_m_rad: Sequence[float]) -> None:
        """ASSUMPTION: caller-supplied TCP offset [x, y, z, rx, ry, rz]
        in meters / radians relative to the tool flange. No default.
        """
        self._require_control()
        self.control.setTcp(list(pose6_m_rad))
        self._tcp_set = True

    def _require_control(self) -> None:
        if self.control is None:
            raise RuntimeError(
                "control interface not connected; construct with control=True"
            )

```

---

## `src/ur5e/config.py`

```python
"""Named connection targets.

The ONLY place the sim<->real host differs. Everything else in the framework
is host-agnostic.
"""

from __future__ import annotations

import os

# Env-overridable so the same code works from the host, from the dev container
# (URSIM_HOST=ursim via compose), and against the real arm (UR_HOST=<lan ip>).
HOSTS = {
    "sim": os.environ.get("URSIM_HOST", "127.0.0.1"),   # URSim
    "real": os.environ.get("UR_HOST", "0.0.0.0"),       # UR5e LAN IP
}


def resolve_host(target: str) -> str:
    try:
        return HOSTS[target]
    except KeyError:
        raise KeyError(f"unknown target {target!r}; known: {sorted(HOSTS)}") from None


# Hosts treated as simulation (no real-arm gate). Includes the resolved sim host.
_SIM_HOSTS = {"127.0.0.1", "localhost", "ursim"}


def is_sim(host: str) -> bool:
    return host in _SIM_HOSTS or host == HOSTS["sim"]

```

---

## `src/ur5e/kinematics.py`

```python
"""UR5e forward kinematics (DH-based).

Shared by the viewer and the safety/workspace checks. Positions are in the
robot base frame, meters. This computes the FLANGE frame (tool0); a mounted
tool's TCP offset is NOT applied here — callers that care about the true tool
point must account for it, and the authoritative boundary protection is the
robot's PolyScope safety planes, not this.
"""

from __future__ import annotations

import math

import numpy as np

# UR5e DH parameters (meters / radians)
DH_D = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
DH_A = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
DH_ALPHA = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]


def _dh(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,      ca,      d],
        [0.0,     0.0,     0.0,    1.0],
    ])


def fk_frames(q):
    """List of 4x4 cumulative transforms: base + each of the 6 joint frames."""
    T = np.eye(4)
    frames = [T.copy()]
    for i in range(6):
        T = T @ _dh(q[i], DH_D[i], DH_A[i], DH_ALPHA[i])
        frames.append(T.copy())
    return frames


def fk_origins(q):
    """3xN float32 array of frame origins (base + 6 joints) — for the viewer."""
    return np.array([T[:3, 3] for T in fk_frames(q)]).T.astype(np.float32)


def fk_tcp_position(q):
    """xyz position of the flange (tool0) in the base frame."""
    return fk_frames(q)[-1][:3, 3]

```

---

## `examples/function_templates/README.md`

```markdown
# function_templates/ — one runnable example per API call

Short scripts (≈ 30 lines each) that show one thing. Run them in order once,
then copy whatever you need into `students/<you>/`.

Every template takes the same flags (`--help` lists them):

```
python examples/function_templates/03_move_to.py                  simulator (default)
python examples/function_templates/03_move_to.py --target real    the lab arm
python examples/function_templates/03_move_to.py --target real --payload 0.35 --tcp 0 0.021 0.108 0 0 0
                                                                  with a mounted tool: mass in kg, TCP in m / rad
python examples/function_templates/03_move_to.py --workspace workspace.real.json
```

| Template | Shows | UR name |
|---|---|---|
| `00_teach_workspace.py` | teach the work area in 4 steps → `workspace.json` | — |
| `01_read_state.py` | `joints()`, `pose()` — no motion | — |
| `02_move_joints.py` | `move_joints(q, speed, accel)` | moveJ |
| `03_move_to.py` | `move_to(pose, speed, accel)`, `ws.pose(u, v, w)` | moveL |
| `04_move_through.py` | `move_through(poses)`, `ws.line(...)` | moveL ×n |
| `05_grid_poses.py` | `ws.grid(nx, ny, w)` + your own per-cell loop | — |
| `06_digital_io.py` | `read_digital_in`, `set_digital_out`, `set_tool_digital_out` | I/O |
| `07_analog_io.py` | `read_analog_in`, `set_analog_out` | I/O |
| `08_safety_rejections.py` | what a rejected move looks like, `SafetyViolation` | — |
| `09_raw_rtde.py` | the `ur_rtde` objects underneath (`conn.receive/control/io`) — sim only, refuses `--target real` | moveJ raw |

## The workspace: teach once, every example scales to it

All templates (and your own scripts) move inside a **taught box**. You
teach it once per setup with `00_teach_workspace.py` — four points, the
script only reads where the arm is, you move it (sim: pendant **Move** tab;
real robot: **freedrive button** on the pendant, supervised):

1. **corner A** — one corner of the work area, tool on the surface
2. **corner B** — the diagonally opposite corner, on the surface
3. **top** — the highest point the tool should reach
4. **home** — a comfortable pose above the area; every example starts and
   ends here (`robot.move_joints(ws.home_joints)`)

Result: `workspace.json` in the repo folder (git-ignored, one per setup).
In a hurry in the sim: `--demo` writes a 300 × 300 × 200 mm box around the
sim's start pose instead.

```python
from ur5e import Robot, Workspace

ws = Workspace.load()                      # workspace.json
with Robot(target="sim", profile=ws) as robot:
    robot.move_joints(ws.home_joints)      # taught home
    robot.move_to(ws.pose(0.5, 0.5, 0.3))  # centre of the area, 30 % of the height
    for cell in ws.grid(3, 3, w=0.1):      # 3 x 3 over the area, 10 % up
        robot.move_to(cell)
```

`ws.pose(u, v, w)` takes **fractions 0…1** of the box along x, y, z —
`(0, 0, 0)` is the low corner on the surface, `(1, 1, 1)` the far top. The
orientation is the one you taught at corner A (tool pointing down). Because
everything is relative to the box, the same script runs on a 300 mm demo
box in the sim and on the 600 × 500 mm cart in the lab.

`Robot(profile=ws)` turns the box into the safety profile for that run: moves
outside it are rejected before they are sent. On the **real robot** the box
is additionally clipped to the measured cell (`cart_against_wall`) and keeps
its speed ceilings — you can make the space smaller, never larger. A
workspace taught in the sim is refused on the real robot: teach it again
there (two minutes with freedrive).

## On the real arm (`--target real`, lab, with the instructor)

The same file, one flag. `--target real` connects to `UR_HOST`
(192.168.1.11 — start the shell with `-e UR_HOST=192.168.1.11`, your laptop
on the robot's `192.168.1.x` network), enables real-arm motion and declares the
tool: `--payload` (kg) and `--tcp x y z rx ry rz` (m, rad) — the same values as
the pendant's *Installation → General → Payload / TCP*. Defaults are 0 kg and
the flange, i.e. **nothing mounted**; if something is mounted, pass its real
values, do not guess. The script prints a `REAL ARM …` line with what it set.

Order in the lab:

1. Pendant: robot ON → START, header icon → **Remote Control**, speed slider
   down (30 %), hand near the stop.
2. Freedrive the arm **over the table** — the cell box is
   x −0.485…−0.201, y −0.286…0.240, z 0.03…0.60 m (`cart_against_wall`);
   anything beyond the table's front edge is outside and would be rejected as
   a target. After teaching, the script prints the box; a `!! clipped` line
   means part of your box lies outside the cell — `ws.pose/grid/line` then use
   the clipped part, and a taught point listed as outside (e.g. `home`) must be
   re-taught, because the arm is not allowed to go there.
3. `python examples/function_templates/00_teach_workspace.py --target real`
   — all four points over the table, `home` included.
4. `python examples/function_templates/01_read_state.py --target real`
   (no motion) then `03_move_to.py --target real`, and so on.

The gate behind this (`allow_real=True` + validated profile + payload + TCP)
is unchanged; the flag only supplies it. Without `--target real` a script can
never move the real arm, even with `UR_HOST` set.

Flagged assumptions: poses are TCP poses with the pendant's active TCP at
teaching time; with a long tool, teach `top` and `home` with extra headroom
(the joint check looks at the flange). The simulator's inputs always read 0.

```

---

## `examples/function_templates/00_teach_workspace.py`

```python
"""00 — Teach the workspace (4 steps). Receive-only: this script never moves the arm.

You move the arm; the script records where it is when you press Enter:

    1. corner A  — one corner of the work area, tool touching the surface
    2. corner B  — the diagonally opposite corner, on the surface
    3. top       — the highest point the tool should ever go
    4. home      — a comfortable pose above the area where every example
                   starts and ends (joint angles are stored too)

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
    ("corner_a", "CORNER A — one corner of the work area, tool on the surface"),
    ("corner_b", "CORNER B — the diagonally opposite corner, on the surface"),
    ("top",      "TOP — the highest point the tool should ever reach"),
    ("home",     "HOME — a comfortable pose above the area (examples start/end here)"),
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
                input(f"[{len(captured) + 1}/4] {text}\n      ... move the arm there, then press Enter: ")
                pose = list(r.receive.getActualTCPPose())
                captured[key] = pose
                if key == "home":
                    home_joints = list(r.receive.getActualQ())
                print(f"      recorded {fmt(pose)}\n")
            try:
                ws = Workspace.from_points(args.target, captured["corner_a"], captured["corner_b"],
                                           captured["top"], captured["home"], home_joints, host=host)
            except WorkspaceError as e:
                raise SystemExit(f"not saved: {e}")

    path = ws.save(args.out)
    print(ws.describe())
    print(f"\nsaved -> {path}. Next: python examples/function_templates/01_read_state.py")


if __name__ == "__main__":
    main()

```

---

## `examples/function_templates/01_read_state.py`

```python
"""01 — Read the robot's state: joints() and pose(). No motion.

    robot.joints()  -> [q0..q5]            joint angles in RADIANS
    robot.pose()    -> [x, y, z, rx, ry, rz] TCP position in METRES + rotation
                       vector in RADIANS, robot Base frame
                       (UR: the numbers under Move → TCP, "Rotation Vector")

Also shows where the current pose sits inside the taught workspace, as
fractions (0..1 along x, y, z of the box).

Usage:  python examples/function_templates/01_read_state.py
        python examples/function_templates/01_read_state.py --target real   (lab, supervised)
"""

import math

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs



def main() -> None:
    args = argument_parser(__doc__).parse_args()
    ws = Workspace.load(args.workspace)
    print(ws.describe(), "\n")

    with Robot(**robot_kwargs(args), profile=ws) as robot:
        q = robot.joints()
        p = robot.pose()

        print("joints [rad]:", [round(v, 3) for v in q])
        print("joints [deg]:", [round(math.degrees(v), 1) for v in q])
        print("pose   [m, rad]:", [round(v, 4) for v in p])
        print("TCP    [mm]:", [round(v * 1000) for v in p[:3]])

        b = ws.box
        frac = [(p[i] - lo) / (hi - lo) for i, (lo, hi) in enumerate((b.x, b.y, b.z))]
        print("inside workspace as fractions (u, v, w):", [round(f, 2) for f in frac],
              "<- 0..1 means inside the box")


if __name__ == "__main__":
    main()

```

---

## `examples/function_templates/02_move_joints.py`

```python
"""02 — move_joints(): joint-space move (UR: moveJ).

    robot.move_joints(q, speed=None, accel=None)
        q      six joint angles in RADIANS
        speed  rad/s   (default: the profile's maximum)
        accel  rad/s^2

All joints move together and arrive together; the TCP path is NOT a straight
line. Use it to go to a known configuration (home) or to turn one joint.
The framework checks the target joints against the joint limits and the
resulting flange position against the workspace box before sending it.

What this does: go home, turn the last joint (wrist 3) +30° and back, and
go home again. Watch it in the viewer (localhost:7001).

Usage:  python examples/function_templates/02_move_joints.py
        python examples/function_templates/02_move_joints.py --target real   (lab, supervised)
"""

import math

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs

DELTA_DEG = 30.0


def main() -> None:
    args = argument_parser(__doc__).parse_args()
    ws = Workspace.load(args.workspace)
    with Robot(**robot_kwargs(args), profile=ws) as robot:
        print("going home (taught joints) ...")
        robot.move_joints(ws.home_joints)

        q = list(robot.joints())
        q[5] += math.radians(DELTA_DEG)                # wrist 3 only
        print(f"wrist 3 +{DELTA_DEG:.0f} deg, slowly ...")
        robot.move_joints(q, speed=0.3, accel=0.5)     # explicit, gentle values

        print("and back at the default speed ...")
        robot.move_joints(ws.home_joints)
    print("done.")


if __name__ == "__main__":
    main()

```

---

## `examples/function_templates/03_move_to.py`

```python
"""03 — move_to(): straight-line move of the TCP (UR: moveL).

    robot.move_to(pose, speed=None, accel=None)
        pose   [x, y, z, rx, ry, rz]  metres + rotation vector (rad), Base frame
        speed  m/s    (default: the profile's maximum — 0.25 in the sim)
        accel  m/s^2

The TCP travels on a straight line; the joints do whatever that needs.
Poses come from the workspace in fractions:  ws.pose(u, v, w)
    u = 0..1 along x of the box, v along y, w along z (0 = surface height)
The orientation is the one you taught (tool pointing down).

What this does: home → hover over the four corners at 30 % height → centre →
down to 5 % above the surface → up → home.

Usage:  python examples/function_templates/03_move_to.py
        python examples/function_templates/03_move_to.py --target real   (lab, supervised)
"""

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs

HOVER = 0.3      # fraction of the box height
LOW = 0.05


def main() -> None:
    args = argument_parser(__doc__).parse_args()
    ws = Workspace.load(args.workspace)
    with Robot(**robot_kwargs(args), profile=ws) as robot:
        robot.move_joints(ws.home_joints)

        for u, v in [(0, 0), (1, 0), (1, 1), (0, 1)]:
            print(f"corner u={u} v={v} at {HOVER:.0%} height")
            robot.move_to(ws.pose(u, v, HOVER))

        print("centre, slowly")
        robot.move_to(ws.pose(0.5, 0.5, HOVER), speed=0.05, accel=0.2)   # 50 mm/s
        print("down to just above the surface")
        robot.move_to(ws.pose(0.5, 0.5, LOW), speed=0.05)
        print("up")
        robot.move_to(ws.pose(0.5, 0.5, HOVER))

        robot.move_joints(ws.home_joints)
    print("done.")


if __name__ == "__main__":
    main()

```

---

## `examples/function_templates/04_move_through.py`

```python
"""04 — move_through(): a sequence of straight-line moves (a path).

    robot.move_through(poses, speed=None, accel=None)
        poses  any list of poses — the robot does move_to() for each one and
               STOPS at every point (so you can read a sensor or act there)

Paths come from generators that only compute geometry:
    ws.line((u0, v0, w0), (u1, v1, w1), n)   n poses between two fractional points
    ur5e.line_poses(start_pose, end_xyz, n)  the same with absolute poses

What this does: home → a diagonal line across the area in 6 steps →
a zig-zag back along the other edge → home.

Usage:  python examples/function_templates/04_move_through.py
        python examples/function_templates/04_move_through.py --target real   (lab, supervised)
"""

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs

H = 0.2   # height fraction for the whole path


def main() -> None:
    args = argument_parser(__doc__).parse_args()
    ws = Workspace.load(args.workspace)
    with Robot(**robot_kwargs(args), profile=ws) as robot:
        robot.move_joints(ws.home_joints)

        diagonal = ws.line((0.1, 0.1, H), (0.9, 0.9, H), n=6)
        print(f"diagonal, {len(diagonal)} points")
        robot.move_through(diagonal)

        zigzag = [ws.pose(0.9, 0.1, H), ws.pose(0.5, 0.5, H + 0.2), ws.pose(0.1, 0.9, H)]
        print("zig-zag with a hop in the middle, slowly")
        robot.move_through(zigzag, speed=0.08)

        robot.move_joints(ws.home_joints)
    print("done.")


if __name__ == "__main__":
    main()

```

---

## `examples/function_templates/05_grid_poses.py`

```python
"""05 — grid: visit every cell of a grid over the work area.

    ws.grid(nx, ny, w=0.1, margin=0.05)
        nx, ny  cells along x and y
        w       height fraction for all cells
        margin  fraction kept free at the edges
        -> list of poses in snake order (row by row, alternating direction)

    ur5e.grid_poses(origin, nx, ny, dx, dy)   the same from an absolute origin
                                               pose with metric spacing

The loop is yours: move, do something, move on. That is the pattern every
"process over an area" starts from — replace do_something() with yours.

What this does: home → 3 x 4 grid at 10 % height, dwell at each cell → home.

Usage:  python examples/function_templates/05_grid_poses.py
        python examples/function_templates/05_grid_poses.py --target real   (lab, supervised)
"""

import time

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs

NX, NY = 3, 4


def do_something(robot: Robot, k: int) -> None:
    """Your per-cell process. Here: just wait a moment and print."""
    print(f"  cell {k}: at {[round(v * 1000) for v in robot.pose()[:3]]} mm")
    time.sleep(0.3)


def main() -> None:
    args = argument_parser(__doc__).parse_args()
    ws = Workspace.load(args.workspace)
    cells = ws.grid(NX, NY, w=0.1)
    print(f"{len(cells)} cells ({NX} x {NY})")

    with Robot(**robot_kwargs(args), profile=ws) as robot:
        robot.move_joints(ws.home_joints)
        robot.move_to(ws.pose(0.5, 0.5, 0.5))          # come in from above
        for k, cell in enumerate(cells):
            robot.move_to(cell)
            do_something(robot, k)
        robot.move_to(ws.pose(0.5, 0.5, 0.5))
        robot.move_joints(ws.home_joints)
    print("done.")


if __name__ == "__main__":
    main()

```

---

## `examples/function_templates/06_digital_io.py`

```python
"""06 — Digital I/O: read inputs, switch outputs. No motion.

    robot.read_digital_in(pin)         -> True/False   standard inputs 0..7 (controller)
    robot.set_digital_out(pin, value)                  standard outputs 0..7
    robot.set_tool_digital_out(pin, value)             the two outputs on the tool flange (0, 1)

Outputs show up live in the pendant's I/O tab (localhost:6080 → I/O): watch
Digital Output 0 tick on and off while this runs. Inputs in the SIMULATOR
always read 0 — there is nothing wired to them and the sim cannot fake them
for us. On the real robot a button/sensor between 24V and DI0 reads True.

What this does: blink DO0 three times, pulse the tool output once, print
all eight inputs.

Usage:  python examples/function_templates/06_digital_io.py
        python examples/function_templates/06_digital_io.py --target real   (lab, supervised)
"""

import time

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs



def main() -> None:
    args = argument_parser(__doc__).parse_args()
    ws = Workspace.load(args.workspace)
    with Robot(**robot_kwargs(args), profile=ws) as robot:
        print("inputs DI0..7:", [robot.read_digital_in(i) for i in range(8)])

        for k in range(3):
            robot.set_digital_out(0, True)
            print(f"DO0 on  ({k + 1}/3)  <- look at the I/O tab")
            time.sleep(0.7)
            robot.set_digital_out(0, False)
            print("DO0 off")
            time.sleep(0.7)

        robot.set_tool_digital_out(0, True)
        time.sleep(0.5)
        robot.set_tool_digital_out(0, False)
        print("tool output 0 pulsed")

        # a typical sensor-gated pattern (always 'no' in the sim):
        if robot.read_digital_in(0):
            print("DI0 is HIGH -> would run the process now")
        else:
            print("DI0 is LOW  -> skipping (in the sim this is always the case)")
    print("done.")


if __name__ == "__main__":
    main()

```

---

## `examples/function_templates/07_analog_io.py`

```python
"""07 — Analog I/O: read voltages, set an output voltage. No motion.

    robot.read_analog_in(pin)        -> float volts  (standard analog inputs 0 and 1)
    robot.set_analog_out(pin, volts)                 standard analog outputs 0 and 1

Analog inputs are for sensors with a voltage output (distance, force,
potentiometer, 0–10 V). In the simulator they read 0.0 V. Outputs are
visible in the pendant's I/O tab (set its mode to Voltage there).

What this does: print both inputs 10x (half a second apart), ramp AO0 from
0 to 5 V and back to 0.

Usage:  python examples/function_templates/07_analog_io.py
        python examples/function_templates/07_analog_io.py --target real   (lab, supervised)
"""

import time

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs

THRESHOLD_V = 2.5


def main() -> None:
    args = argument_parser(__doc__).parse_args()
    ws = Workspace.load(args.workspace)
    with Robot(**robot_kwargs(args), profile=ws) as robot:
        for _ in range(10):
            a0, a1 = robot.read_analog_in(0), robot.read_analog_in(1)
            state = "above" if a0 > THRESHOLD_V else "below"
            print(f"AI0 = {a0:5.2f} V ({state} {THRESHOLD_V} V)   AI1 = {a1:5.2f} V")
            time.sleep(0.5)

        print("ramping AO0 0 -> 5 V -> 0")
        for v in (1.0, 2.0, 3.0, 4.0, 5.0, 0.0):
            robot.set_analog_out(0, v)
            time.sleep(0.4)
    print("done.")


if __name__ == "__main__":
    main()

```

---

## `examples/function_templates/08_safety_rejections.py`

```python
"""08 — What a REJECTED move looks like, and how to handle it.

Every move is checked BEFORE it is sent: target inside the workspace box,
speed/accel under the profile ceilings, joints inside limits. A violation
raises ur5e.safety.SafetyViolation and the robot does not move — nothing is
silently clamped, so a wrong number is visible immediately.

    from ur5e.safety import SafetyViolation
    try:
        robot.move_to(pose)
    except SafetyViolation as e:
        print("rejected:", e)

What this does: an accepted move, then three deliberately bad ones
(outside the box, too fast, bad joint value) — each is rejected with a
readable message — then home.

Usage:  python examples/function_templates/08_safety_rejections.py
        python examples/function_templates/08_safety_rejections.py --target real   (lab, supervised)
"""

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs
from ur5e.safety import SafetyViolation



def attempt(label, fn):
    try:
        fn()
        print(f"  [moved]    {label}")
    except SafetyViolation as e:
        print(f"  [rejected] {label}\n             -> {e}")


def main() -> None:
    args = argument_parser(__doc__).parse_args()
    ws = Workspace.load(args.workspace)
    with Robot(**robot_kwargs(args), profile=ws) as robot:
        robot.move_joints(ws.home_joints)
        print("profile:", robot.limits.name, "| tcp speed max", robot.limits.tcp_speed_max, "m/s")

        attempt("centre of the box at 30 % height (fine)",
                lambda: robot.move_to(ws.pose(0.5, 0.5, 0.3)))

        outside = ws.pose(1.0, 1.0, 1.0)
        outside[0] += 0.30                                   # 30 cm beyond the box edge
        attempt("30 cm outside the box in x",
                lambda: robot.move_to(outside))

        attempt("centre again but at 2.0 m/s (over the ceiling)",
                lambda: robot.move_to(ws.pose(0.5, 0.5, 0.3), speed=2.0))

        bad_q = list(ws.home_joints)
        bad_q[1] += 10.0                                     # way past +-2*pi rad -> joint limit
        attempt("a joint target beyond its limit",
                lambda: robot.move_joints(bad_q))

        robot.move_joints(ws.home_joints)
    print("done — the robot only did the first move and went home.")


if __name__ == "__main__":
    main()

```

---

## `examples/function_templates/09_raw_rtde.py`

```python
"""09 — Advanced: the raw ur_rtde interfaces underneath Robot. SIM ONLY here.

Robot wraps three ur_rtde objects; they are yours when you need something
Robot does not offer (full API: ur_rtde docs, "RTDE*Interface"):

    robot.conn.receive   RTDEReceiveInterface   getActualQ(), getActualTCPPose(),
                                                getActualTCPForce(), getRobotMode(), ...
    robot.conn.control   RTDEControlInterface   moveJ(), moveL(), servoJ(), speedL(),
                                                getInverseKinematics(), ...
    robot.conn.io        RTDEIOInterface        setStandardDigitalOut(), ...

Raw control calls SKIP the framework's checks (workspace box, speed
ceilings). The arm does exactly what you send. That is why this template
is pinned to the simulator — on the real robot, stay with the Robot methods
unless you know exactly what you are doing and someone is at the e-stop.

What this does: read a few raw values, then a raw moveJ of wrist 3 by 10°
and back.

Usage:  python examples/function_templates/09_raw_rtde.py
"""

import math

from ur5e import Robot, Workspace
from ur5e.cli import argument_parser, robot_kwargs



def main() -> None:
    args = argument_parser(__doc__).parse_args()
    if args.target != "sim":
        raise SystemExit("this template bypasses the safety checks — sim only")
    ws = Workspace.load(args.workspace)
    with Robot(**robot_kwargs(args), profile=ws) as robot:
        rcv, ctl = robot.conn.receive, robot.conn.control
        print("robot mode      :", rcv.getRobotMode(), "(7 = running)")
        print("TCP force [N,Nm]:", [round(v, 1) for v in rcv.getActualTCPForce()])
        print("joint temps [C] :", [round(v, 1) for v in rcv.getJointTemperatures()])
        print("speed slider    :", rcv.getSpeedScaling())

        q = list(rcv.getActualQ())
        q[5] += math.radians(10)
        print("raw moveJ wrist 3 +10 deg (speed 0.5 rad/s, accel 0.5 rad/s^2) ...")
        ctl.moveJ(q, 0.5, 0.5)                 # (q, speed, acceleration) — no checks!
        q[5] -= math.radians(10)
        ctl.moveJ(q, 0.5, 0.5)
    print("done.")


if __name__ == "__main__":
    main()

```

---

## `examples/grid_process.py`

```python
"""First example process — grid traversal with sensor-gated actuation (SIM).

The skeleton students adapt for their digital-fabrication processes: traverse a
grid over the work surface; at each cell read a sensor (analog + digital) and
conditionally run the actuator tool. Built entirely on the student-facing base
functions in `ur5e.robot` — no RTDE or safety internals here.

In SIM the inputs read static 0, so every cell will 'skip'. To see the process
branch fire, set a standard digital/analog input HIGH in URSim's I/O tab, or
wire real sensors. Students replace `process_at_cell` with their actual process.

Run (dev container, URSim ON/STARTED, viewer open to watch):
    python examples/grid_process.py
    python examples/grid_process.py --target real      # lab, supervised: the grid
        # starts at the arm's CURRENT pose, so freedrive it over the table first
        # (inside the cart_against_wall box); --payload/--tcp if a tool is mounted
"""

from ur5e import Robot, grid_poses
from ur5e.cli import argument_parser, robot_kwargs

ANALOG_PIN = 0
DIGITAL_PIN = 0
ACTUATOR_PIN = 0
TRIGGER_THRESHOLD_V = 0.5   # actuate when the analog sensor exceeds this


def process_at_cell(robot: Robot) -> None:
    """The per-cell 'process'. Students replace this body (servo, dwell, etc.)."""
    robot.set_digital_out(ACTUATOR_PIN, True)
    robot.set_digital_out(ACTUATOR_PIN, False)


def main() -> None:
    args = argument_parser(__doc__).parse_args()
    with Robot(**robot_kwargs(args)) as robot:
        # Anchor the grid to the current reachable pose. Students re-anchor this
        # to the measured work surface. Small 3x3 grid, 4 cm spacing.
        origin = robot.pose()
        cells = grid_poses(origin, nx=3, ny=3, dx=0.04, dy=0.04)
        print(f"traversing {len(cells)} cells ...")

        for k, cell in enumerate(cells):
            robot.move_to(cell)
            analog = robot.read_analog_in(ANALOG_PIN)
            digital = robot.read_digital_in(DIGITAL_PIN)
            triggered = analog > TRIGGER_THRESHOLD_V or digital
            print(f"  cell {k}: analog={analog:.3f}V digital={digital} -> "
                  f"{'PROCESS' if triggered else 'skip'}")
            if triggered:
                process_at_cell(robot)

        robot.move_to(origin)  # return to start
    print("done.")


if __name__ == "__main__":
    main()

```

---

## `docker-compose.yml`

```yaml
services:
  # URSim — the simulated UR5e controller (PolyScope in your browser at
  # http://localhost:6080/vnc.html). It is an x86 (amd64) image: native on
  # Intel/Windows, emulated (slower) on Apple Silicon. It exposes the real RTDE
  # ports, so framework code is identical against the sim and the real arm.
  #
  # !!! The tag MUST match the PolyScope version on the real robot's pendant
  # !!! (pendant: hamburger menu -> About -> "URSoftware x.y.z"). The lab robot
  # !!! runs 5.9.1 (checked 2026-10-28); 5.9.4 is the closest image Docker Hub
  # !!! offers (same 5.9 line, so programs carry over). Only the instructor
  # !!! changes this line, after a PolyScope update on the robot.
  ursim:
    image: universalrobots/ursim_e-series:5.9.4
    platform: linux/amd64
    ports:
      - "5900:5900"                # VNC
      - "6080:6080"                # browser UI -> http://localhost:6080/vnc.html
      - "29999:29999"              # dashboard server
      - "30001-30004:30001-30004"  # RTDE / primary / secondary
    environment:
      - ROBOT_MODEL=UR5
    volumes:
      # PolyScope saves programs to /ursim/programs -> ./programs on your laptop,
      # so you can copy .urp files to a USB stick for the real pendant.
      - ./programs:/ursim/programs

  # Dev shell: Python + ur_rtde + the framework. Prebuilt image (amd64 + arm64),
  # nothing to build. The repo is bind-mounted at /workspace, so edits on your
  # laptop are live inside the container; the framework is installed editable
  # from the mount on every start (entrypoint in the image).
  dev:
    image: ghcr.io/reckj/ur5e-dev:hs26.1
    depends_on:
      - ursim
    volumes:
      - .:/workspace
    working_dir: /workspace
    environment:
      URSIM_HOST: ursim       # reach URSim by service name on the compose network
      # UR_HOST: 192.168.1.11 # the lab robot's static IP (pendant: menu -> About) — lab only, see docs/USAGE.md
    stdin_open: true
    tty: true

  # Live 3D viewer + scene editor — browser at http://localhost:7001/ .
  # Receive-only, driven by RTDE state; runs alongside the dev shell.
  # Scenes live in ./scenes/<name>/ and are switched in the viewer page.
  viewer:
    image: ghcr.io/reckj/ur5e-dev:hs26.1
    depends_on:
      - ursim
    volumes:
      - .:/workspace
    working_dir: /workspace
    environment:
      URSIM_HOST: ursim
      VIEWER_SCENE: ${VIEWER_SCENE:-default}
      PYTHONUNBUFFERED: "1"   # so the viewer's messages show up in `docker compose logs viewer`
    ports:
      - "7001:7000"   # host 7001 (avoids macOS AirPlay Receiver on 7000) -> container 7000
    command: python scripts/rtde_viewer.py --target sim

```

