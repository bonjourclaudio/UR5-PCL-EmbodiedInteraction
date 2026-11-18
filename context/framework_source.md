# ur5e framework — source snapshot

Generated 2026-11-18T10:08Z from git 6076c21
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

from .config import cell_profile_name, resolve_host
from .connection import RobotConnection
from .motion import MotionController
from .safety import SafetyLimits, get_profile
from .workspace import Workspace


class Robot:
    """High-level UR5e handle. Use as a context manager."""

    def __init__(self, target: str = "sim", *, profile=None,
                 allow_real: bool = False, host: str | None = None,
                 payload_kg: float | None = None, tcp: Sequence[float] | None = None):
        """target 'sim' | 'real'; profile: name, SafetyLimits or Workspace
        (None = the cell profile: sim_open in the sim, cart_360 on the real arm);
        payload_kg / tcp: if given, set_tool() is called right after connecting
        (what the examples do for --target real)."""
        self.host = host or resolve_host(target)
        self.target = target
        self._tool = (payload_kg, tuple(tcp) if tcp is not None else (0, 0, 0, 0, 0, 0))
        if profile is None:
            profile = cell_profile_name(target)      # sim_open / cart_360 (or $UR5E_CELL)
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
``--cell cart_360`` checks a sim run against the lab cell (the cart keep-out)
— what the real arm will enforce.
"""

from __future__ import annotations

import argparse
import os


class _SetCell(argparse.Action):
    """--cell NAME: select the cell profile for this run (sets UR5E_CELL)."""

    def __call__(self, parser, namespace, values, option_string=None):
        os.environ["UR5E_CELL"] = values
        setattr(namespace, self.dest, values)


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
    ap.add_argument("--cell", action=_SetCell, default=None, metavar="NAME",
                    help="cell profile to check against (default: sim_open in the sim, cart_360 on the "
                         "real arm). '--cell cart_360' in the sim rehearses against the cart keep-out")
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

A *workspace* is the part of the table the robot may use, taught by moving
the arm there by hand (freedrive button on the pendant) or, in the simulator,
with the pendant's Move tab — the teach script only READS poses
(receive-only), it never moves the arm:

    1-4. four LOW corners  — around the work area, tool touching the surface
    5-8. four HIGH corners — the ceiling, e.g. straight above each low corner
                             at the highest height the tool should reach
    9.   home              — a comfortable pose above the area, inside the box;
                             examples start/end here

The box is the LARGEST AXIS-ALIGNED BOX THAT FITS INSIDE the eight corners
(per axis: from the highest of the four low values to the lowest of the four
high values). So a work area that is slightly rotated against the robot base,
or a table that is not perfectly level, still gives a box that lies entirely
within what was taught — never beyond it.

Saved as ``workspace.json`` (git-ignored, one per setup). Examples address the
box in FRACTIONS, so the same script runs on any table size:

    ws = Workspace.load()
    ws.pose(0.5, 0.5, 0.3)   # centre of the area, 30 % of the way up
    ws.grid(3, 3, w=0.1)     # 3 x 3 cells, 10 % above the surface

Safety (flagged): the taught box is turned into a SafetyLimits profile with
``ws.profile(target)``. On the REAL arm it is the INTERSECTION of the taught
box with the measured cell profile (``cart_360``: the arm on the free-standing
cart; see config.cell_profile_name) — students can only shrink the space,
never enlarge it — and the cell's speed ceilings and keep-outs (the cart body)
are kept. A box taught in the sim is refused for the real arm (and vice versa).
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

from .config import cell_profile_name
from .safety import Box, SafetyLimits, get_profile

FORMAT = "ur5e-workspace/2"
FORMAT_V1 = "ur5e-workspace/1"       # old 4-point files (corner_a/corner_b/top/home) still load
DEFAULT_FILE = "workspace.json"
MIN_SIDE_M = 0.02          # a taught box must be at least 20 mm in x, y and z


class WorkspaceError(Exception):
    """Workspace file missing/invalid, or used for the wrong target."""


@dataclass
class Workspace:
    target: str                      # "sim" or "real" — where it was taught
    corners_low: List[List[float]]   # 4 TCP poses [x,y,z,rx,ry,rz] (m, rad) on the surface
    corners_high: List[List[float]]  # 4 TCP poses at the ceiling
    home: List[float]                # TCP pose at home
    home_joints: List[float]         # joint angles at home (rad) — what go_home() uses
    taught_at: str = ""
    host: str = ""
    note: str = ""
    path: str = field(default="", compare=False)

    # ----------------------------------------------------------------- box --
    @property
    def corners(self) -> List[List[float]]:
        """All eight taught corners, low ones first."""
        return list(self.corners_low) + list(self.corners_high)

    @property
    def box(self) -> Box:
        """Largest axis-aligned box INSIDE the eight taught corners (base frame):
        per axis from the highest of the four low values to the lowest of the
        four high values. Equals the taught rectangle when the area is aligned
        with the base axes; shrinks (never grows) when it is rotated or tilted."""
        return inner_box(self.corners)

    @property
    def bounding_box(self) -> Box:
        """Axis-aligned box AROUND all taught corners — for information only;
        poses are generated inside `box`, never here."""
        pts = self.corners
        xs, ys, zs = ([p[i] for p in pts] for i in range(3))
        return Box(x=(min(xs), max(xs)), y=(min(ys), max(ys)), z=(min(zs), max(zs)))

    @property
    def orientation(self) -> List[float]:
        """Tool orientation used for every generated pose: the one taught at
        the FIRST low corner (rotation vector, rad). Teach with the tool
        pointing the way you want it to work."""
        return list(self.corners_low[0][3:6])

    # kept for scripts written against the 4-point format
    @property
    def corner_a(self) -> List[float]:
        return list(self.corners_low[0])

    @property
    def corner_b(self) -> List[float]:
        return list(self.corners_low[2])

    @property
    def top(self) -> List[float]:
        return list(self.corners_high[0])

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
        clamped to the cell profile (never larger than the cell; its keep-outs
        and speed ceilings are kept). Cell: config.cell_profile_name(target)."""
        if target != self.target:
            raise WorkspaceError(
                f"this workspace was taught on target '{self.target}' but you are "
                f"connecting to '{target}'. Teach a workspace on the real setup "
                f"before using it there (examples/function_templates/00_teach_workspace.py).")
        base = get_profile(cell_profile_name(target))
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
    def from_points(cls, target: str, corners_low, corners_high, home, home_joints,
                    host: str = "", note: str = "") -> "Workspace":
        """Build + validate from the taught poses: 4 low corners, 4 high corners,
        home. Raises WorkspaceError with a plain message if the box is
        degenerate or home lies outside it."""
        corners_low = [list(p) for p in corners_low]
        corners_high = [list(p) for p in corners_high]
        ws = cls(target=target, corners_low=corners_low, corners_high=corners_high,
                 home=list(home), home_joints=list(home_joints),
                 taught_at=time.strftime("%Y-%m-%d %H:%M"), host=host, note=note)
        if target not in ("sim", "real"):
            raise WorkspaceError(f"target must be 'sim' or 'real', not {target!r}")
        if len(corners_low) != 4 or len(corners_high) != 4:
            raise WorkspaceError(f"need 4 low + 4 high corners, got {len(corners_low)} + {len(corners_high)}")
        for name, p in [(f"low corner {i + 1}", c) for i, c in enumerate(corners_low)] + \
                       [(f"high corner {i + 1}", c) for i, c in enumerate(corners_high)] + [("home", home)]:
            if len(p) != 6:
                raise WorkspaceError(f"{name}: expected a 6-element pose, got {len(p)}")
        if len(home_joints) != 6:
            raise WorkspaceError("home_joints: expected 6 joint angles")
        surface = max(c[2] for c in corners_low)
        ceiling = min(c[2] for c in corners_high)
        if ceiling <= surface + MIN_SIDE_M:
            raise WorkspaceError(
                f"the high corners must all be clearly above the low corners "
                f"(lowest high corner {ceiling * 1000:.0f} mm, highest low corner {surface * 1000:.0f} mm)")
        b, bb = ws.box, ws.bounding_box
        for axis, (lo, hi), (blo, bhi) in zip("xyz", (b.x, b.y, b.z), (bb.x, bb.y, bb.z)):
            if hi - lo < MIN_SIDE_M:
                raise WorkspaceError(
                    f"no box of at least {MIN_SIDE_M * 1000:.0f} mm fits inside the taught corners along "
                    f"{axis} (inner {round((hi - lo) * 1000)} mm, outer {round((bhi - blo) * 1000)} mm). "
                    f"Either the arm did not move between steps, corners were mixed up, or the area is "
                    f"rotated too far against the robot's base axes — teach the corners roughly aligned "
                    f"with the robot's x/y.")
        if not b.contains(home[:3], tol=0.001):
            raise WorkspaceError(
                f"home {[round(v * 1000) for v in home[:3]]} mm lies outside the box that fits inside "
                f"the corners (x {round(b.x[0] * 1000)}..{round(b.x[1] * 1000)}, "
                f"y {round(b.y[0] * 1000)}..{round(b.y[1] * 1000)}, z {round(b.z[0] * 1000)}..{round(b.z[1] * 1000)} mm). "
                f"Teach home above the area, inside the box.")
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
        fmt = d.get("format")
        if fmt not in (FORMAT, FORMAT_V1):
            raise WorkspaceError(f"{p}: unknown format {fmt!r}, expected {FORMAT!r}")
        try:
            if fmt == FORMAT_V1:
                # old 4-point file: synthesise the 8 corners of its bounding box, so the
                # box (and every pose generated from it) is exactly what it was before
                pts = [d["corner_a"], d["corner_b"], d["top"], d["home"]]
                xs, ys, zs = ([q[i] for q in pts] for i in range(3))
                rot = list(d["corner_a"][3:6])
                lo, hi = [min(xs), min(ys), min(zs)], [max(xs), max(ys), max(zs)]
                corners_low = [[x, y, lo[2], *rot] for x, y in ((lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1]))]
                corners_high = [[c[0], c[1], hi[2], *rot] for c in corners_low]
                note = (d.get("note", "") + " [converted from 4-point format]").strip()
            else:
                corners_low, corners_high, note = d["corners_low"], d["corners_high"], d.get("note", "")
            ws = cls(target=d["target"], corners_low=corners_low, corners_high=corners_high,
                     home=d["home"], home_joints=d["home_joints"],
                     taught_at=d.get("taught_at", ""), host=d.get("host", ""),
                     note=note, path=str(p))
        except KeyError as e:
            raise WorkspaceError(f"{p}: missing field {e}") from None
        return ws

    def describe(self) -> str:
        b, bb = self.box, self.bounding_box
        sx, sy, sz = self.size_mm
        out = (f"workspace '{self.path or '(unsaved)'}' taught on {self.target} {self.taught_at}\n"
               f"  box x {b.x[0]:+.3f}..{b.x[1]:+.3f}  y {b.y[0]:+.3f}..{b.y[1]:+.3f}  "
               f"z {b.z[0]:+.3f}..{b.z[1]:+.3f} m   ({sx} x {sy} x {sz} mm) — fits inside the 8 corners\n"
               f"  home joints [deg]: {[round(q * 57.2958, 1) for q in self.home_joints]}")
        if (bb.x, bb.y, bb.z) != (b.x, b.y, b.z):
            out += (f"\n  corners span x {bb.x[0]:+.3f}..{bb.x[1]:+.3f}  y {bb.y[0]:+.3f}..{bb.y[1]:+.3f}  "
                    f"z {bb.z[0]:+.3f}..{bb.z[1]:+.3f} m (area rotated/tilted vs. base -> box is the inner part)")
        # On the real arm the usable box is the intersection with the cell profile:
        # say so when that clips anything, and which corners lie outside the cell.
        try:
            c = self.profile(self.target).workspace
        except WorkspaceError as e:
            return out + f"\n  !! {e}"
        if (c.x, c.y, c.z) != (b.x, b.y, b.z):
            cell = get_profile(cell_profile_name(self.target)).workspace
            named = [(f"low corner {i + 1}", q) for i, q in enumerate(self.corners_low)] + \
                    [(f"high corner {i + 1}", q) for i, q in enumerate(self.corners_high)]
            outside = [n for n, q in named if not cell.contains(q[:3])]
            out += (f"\n  !! clipped to the cell profile -> usable x {c.x[0]:+.3f}..{c.x[1]:+.3f}  "
                    f"y {c.y[0]:+.3f}..{c.y[1]:+.3f}  z {c.z[0]:+.3f}..{c.z[1]:+.3f} m "
                    f"(ws.pose/grid/line use this box)"
                    + (f"\n     corners outside the cell: {', '.join(outside)} (fine — the box is clipped)"
                       if outside else ""))
        for k in self.profile(self.target).keepout:
            z = k.tool_zone()
            if (c.x[0] < z.x[1] and c.x[1] > z.x[0] and c.y[0] < z.y[1] and c.y[1] > z.y[0]
                    and c.z[0] < z.z[1] and c.z[1] > z.z[0]):
                out += (f"\n  note: part of this box reaches into the keep-out {k.describe()} "
                        f"+ {k.side_clearance * 1000:.0f} mm — poses there are rejected; above the top is fine")
        if not c.contains(self.home[:3], tol=0.001):
            out += ("\n  !! home lies outside the usable box — move_joints(home_joints) would be "
                    "rejected; teach home inside the box")
        return out


def demo_sim_workspace(start_pose: Sequence[float], start_joints: Sequence[float]) -> Workspace:
    """A ready-made workspace around the simulator's start pose, for trying
    the templates without teaching: a 300 x 300 mm area, 200 mm high, with the
    start pose as home in the middle above it. Sim only."""
    x, y, z = start_pose[0], start_pose[1], start_pose[2]
    rot = list(start_pose[3:6])
    surface = z - 0.10
    xy = [(x - 0.15, y - 0.15), (x + 0.15, y - 0.15), (x + 0.15, y + 0.15), (x - 0.15, y + 0.15)]
    low = [[cx, cy, surface, *rot] for cx, cy in xy]
    high = [[cx, cy, surface + 0.20, *rot] for cx, cy in xy]
    return Workspace.from_points("sim", low, high, list(start_pose), list(start_joints),
                                 note="demo workspace around the URSim start pose (not taught)")


def inner_box(points: Sequence[Sequence[float]]) -> Box:
    """Largest axis-aligned box inside a set of 2n corner points: per axis, the
    range from the highest of the n lower values to the lowest of the n upper
    values. For an axis-aligned box's own corners this is the box itself."""
    n = len(points) // 2
    rng = []
    for i in range(3):
        vals = sorted(p[i] for p in points)
        rng.append((vals[n - 1], vals[n]))
    return Box(x=rng[0], y=rng[1], z=rng[2])

```

---

## `src/ur5e/safety.py`

```python
"""Safety limits — the framework's SECONDARY software guardrail.

STATUS: HS26 student framework, lower layer (sim-validated; cart_against_wall box
captured on the real cell). The keep-out / path checks of `cart_360`
(2026-11-18) are sim-validated (offline tests + URSim 5.9.4 with the measured
cart); their first runs on the lab arm are supervised by the instructor.

IMPORTANT: this is NOT the robot's safety system. The authoritative, certified
safety layer is the robot's PolyScope Safety Configuration (joint limits, TCP
speed/force limits, safety planes). These software limits exist to catch
mistakes earlier and must always be set *more conservative than* the controller
configuration. They never replace it.

Profiles are swappable by name (like the host parameter) so the same code adapts
to different cell setups:

    cart_against_wall   table work only, cart against a wall (the first HS26 cell)
    cart_360            arm on the free-standing cart, may reach AROUND and BELOW
                        the cart top; the cart body is a keep-out (cells/cart_360.json)
    real_open           instructor override, taught box only
    sim_open            free simulator exploration

`validated=False` means the profile's geometry has not been confirmed against the
real cell (e.g. cart not yet measured). Real-arm motion is refused until a
profile is validated; sim motion is allowed (geometry checks still run).

What a profile with a keep-out checks (and what not) — see SafetyLimits:
  target TCP, the straight TCP line of a linear move, the flange->TCP line, and
  the arm links as spheres (upper arm .. wrist 3) at sampled joint positions.
  NOT checked: the shape of the mounted tool beyond that one line, cables,
  anything standing next to the cart, and people.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .kinematics import (arm_spheres, fk_frames, fk_tcp_position,
                         flange_position_from_pose, interpolate_pose)


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

    def strictly_contains(self, p) -> bool:
        """True if p lies in the INTERIOR (touching a face does not count)."""
        return (self.x[0] < p[0] < self.x[1] and self.y[0] < p[1] < self.y[1]
                and self.z[0] < p[2] < self.z[1])

    def segment_inside(self, a, b):
        """A point of the straight segment a->b that lies in the box INTERIOR,
        or None if the segment stays outside (touching a face is outside)."""
        t0, t1 = 0.0, 1.0
        for (lo, hi), a_i, b_i in zip((self.x, self.y, self.z), a, b):
            d = b_i - a_i
            if abs(d) < 1e-12:
                if not (lo < a_i < hi):
                    return None
                continue
            ta, tb = (lo - a_i) / d, (hi - a_i) / d
            if ta > tb:
                ta, tb = tb, ta
            t0, t1 = max(t0, ta), min(t1, tb)
            if t1 - t0 < 1e-9:
                return None
        t = (t0 + t1) / 2.0
        return tuple(a_i + (b_i - a_i) * t for a_i, b_i in zip(a, b))


@dataclass(frozen=True)
class KeepOut:
    """A solid body next to the arm (base frame, metres) — the cart the robot
    stands on, or something fixed to it (handle, mounting plate). `box.z[1]` is
    its top surface, `box.z[0]` its underside.

    Two zones are derived from it:
      tool zone  the box grown by `side_clearance` on the sides and underneath,
                 NOT at the top: the TCP and the flange->TCP line may touch the
                 top surface (table work) but must stay `side_clearance` away
                 from the sides and the underside.
      arm zone   the box grown by the arm radius on all sides: no arm link
                 (upper arm .. wrist 3) may come closer than that.
    """
    name: str
    box: Box
    side_clearance: float = 0.05
    top_tolerance: float = 0.005      # the tool may read up to this far "into" the top surface

    def tool_zone(self, shrink: float = 0.0) -> Box:
        c = self.side_clearance - shrink
        b = self.box
        return Box(x=(b.x[0] - c, b.x[1] + c), y=(b.y[0] - c, b.y[1] + c),
                   z=(b.z[0] - c, b.z[1] - self.top_tolerance - shrink))

    def arm_zone(self, radius: float) -> Box:
        b = self.box
        return Box(x=(b.x[0] - radius, b.x[1] + radius), y=(b.y[0] - radius, b.y[1] + radius),
                   z=(b.z[0] - radius, b.z[1] + radius))

    def describe(self) -> str:
        b = self.box
        return (f"'{self.name}' (x {b.x[0]:+.3f}..{b.x[1]:+.3f}, y {b.y[0]:+.3f}..{b.y[1]:+.3f}, "
                f"z {b.z[0]:+.3f}..{b.z[1]:+.3f} m)")


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
    # Solid bodies next to the arm (the cart). Empty = no keep-out checks at all.
    keepout: tuple = ()               # tuple[KeepOut, ...]
    arm_radius: float = 0.06          # arm links are checked as spheres of this radius (m)
    floor_z: float | None = None      # floor height in the base frame; None = no floor check
    flange_min_z: float | None = None # lowest flange height — only for a cell whose pendant installation
                                      # has a floor plane (it limits the FLANGE); None = no check (cart_360)

    @property
    def has_obstacles(self) -> bool:
        """True if this profile has keep-outs or a floor — only then are paths
        and arm links checked (profiles without them behave exactly as before)."""
        return bool(self.keepout) or self.floor_z is not None

    # --- pure checks: each returns None or raises SafetyViolation ---
    def check_joint_target(self, q, tcp=None) -> None:
        """Joint limits + the TOOL POINT (flange FK + the active TCP offset, as
        the taught workspace is in TCP coordinates) inside the workspace box.
        tcp=None checks the flange itself (sim, or no tool declared)."""
        if len(q) != 6:
            raise SafetyViolation(f"expected 6 joint values, got {len(q)}")
        for i, (lo, hi) in enumerate(self.joint_limits):
            if not (lo <= q[i] <= hi):
                raise SafetyViolation(
                    f"joint {i} = {q[i]:.3f} rad outside limit [{lo:.3f}, {hi:.3f}]")
        self.check_workspace(fk_tcp_position(q, tcp), tol=FK_TOLERANCE_M,
                             what="joint target (TCP via FK)" if tcp is not None else "joint target (flange FK, no TCP set)")
        self.check_arm(q, tcp, what="joint target")

    def check_pose_target(self, pose, tcp=None) -> None:
        if len(pose) != 6:
            raise SafetyViolation(f"expected 6-element pose, got {len(pose)}")
        self.check_workspace(pose[:3])
        if self.has_obstacles:
            flange = flange_position_from_pose(pose, tcp)
            self._check_flange_height(flange, "target")
            self._check_line(flange, pose[:3], "tool (flange to TCP) at the target")

    def check_workspace(self, xyz, tol: float = 0.0, what: str = "target") -> None:
        """Point (the TCP) inside the workspace box and outside every keep-out."""
        if not self.workspace.contains(xyz, tol):
            raise SafetyViolation(
                f"{what} {tuple(round(float(v), 3) for v in xyz)} outside "
                f"workspace box of profile '{self.name}'")
        for k in self.keepout:
            if k.tool_zone(shrink=tol).strictly_contains(xyz):
                raise SafetyViolation(
                    f"{what} {_fmt(xyz)} is inside the keep-out {k.describe()} "
                    f"+ {k.side_clearance * 1000:.0f} mm on the sides, profile '{self.name}' — "
                    f"move the point above its top or further out from its side")

    # --- keep-out checks (only profiles with has_obstacles) ---
    def _check_line(self, a, b, what: str) -> None:
        """Straight line a->b (TCP path, flange path, or the tool itself) must
        not pass through a keep-out's tool zone."""
        for k in self.keepout:
            zone = k.tool_zone()
            hit = zone.segment_inside(a, b)
            if hit is not None:
                advice = ("the arm is ALREADY there — move it clear by hand (freedrive) first"
                          if zone.strictly_contains(a) else
                          "go around: add a waypoint above the top or outside the corner")
                raise SafetyViolation(
                    f"{what}: the straight line {_fmt(a)} -> {_fmt(b)} passes through the keep-out "
                    f"{k.describe()} + {k.side_clearance * 1000:.0f} mm near {_fmt(hit)}; {advice}")

    def check_arm(self, q, tcp=None, what: str = "pose") -> None:
        """Arm links (upper arm .. wrist 3, as spheres of arm_radius) clear of
        every keep-out and of the floor, and the flange->TCP line clear of the
        keep-outs' tool zone, for joint angles q. Nominal kinematics."""
        if not self.has_obstacles:
            return
        r = self.arm_radius
        zones = [(k, k.arm_zone(r)) for k in self.keepout]
        for link, p in arm_spheres(q, r):
            for k, zone in zones:
                if zone.strictly_contains(p):
                    raise SafetyViolation(
                        f"{what}: the {link} would come closer than {r * 1000:.0f} mm to the keep-out "
                        f"{k.describe()} (link point {_fmt(p)}) — choose a pose or path that keeps the arm clear")
            if self.floor_z is not None and p[2] - r < self.floor_z:
                raise SafetyViolation(
                    f"{what}: the {link} would touch the floor (link point {_fmt(p)}, floor at z {self.floor_z:+.3f} m)")
        flange = fk_frames(q)[-1][:3, 3]
        self._check_flange_height(flange, what)
        if self.keepout:
            self._check_line(flange, fk_tcp_position(q, tcp), f"{what}, tool (flange to TCP)")

    def _check_flange_height(self, flange, what: str) -> None:
        if self.flange_min_z is not None and flange[2] < self.flange_min_z:
            raise SafetyViolation(
                f"{what}: the flange would be at z {float(flange[2]):+.3f} m, below {self.flange_min_z:+.3f} m — "
                f"this cell's pendant floor plane stops the flange there (the tool tip may go lower than "
                f"the flange, the flange itself may not)")

    def describe(self) -> str:
        """The profile's numbers as text (shown by `python -m ur5e.cell`)."""
        b = self.workspace
        out = [f"profile '{self.name}'   validated for the real arm: {self.validated}",
               f"  TCP box      x {b.x[0]:+.3f}..{b.x[1]:+.3f}   y {b.y[0]:+.3f}..{b.y[1]:+.3f}   z {b.z[0]:+.3f}..{b.z[1]:+.3f} m",
               f"  ceilings     TCP {self.tcp_speed_max} m/s, {self.tcp_accel_max} m/s^2   joints "
               f"{self.joint_speed_max} rad/s, {self.joint_accel_max} rad/s^2   payload {self.payload_max_kg} kg"]
        for k in self.keepout:
            out.append(f"  keep-out     {k.describe()}: tool {k.side_clearance * 1000:.0f} mm from its sides "
                       f"(top may be touched), arm links {self.arm_radius * 1000:.0f} mm from sides and top")
        if self.floor_z is not None:
            out.append(f"  floor        z {self.floor_z:+.3f} m" + (f"; flange not below z {self.flange_min_z:+.3f} m"
                                                                   if self.flange_min_z is not None else ""))
        return "\n".join(out)

    def check_joint_path(self, q_start, q_target, tcp=None) -> None:
        """A joint move (moveJ) turns all joints evenly from q_start to
        q_target: check the arm every <= 3 deg of the largest joint travel."""
        if not self.has_obstacles:
            return
        q0, q1 = np.asarray(q_start, dtype=float), np.asarray(q_target, dtype=float)
        n = int(min(max(math.ceil(float(np.max(np.abs(q1 - q0))) / math.radians(3.0)), 1), 240))
        for i in range(1, n + 1):
            self.check_arm(q0 + (q1 - q0) * (i / n), tcp, what=f"joint move, {i / n:.0%} of the way")

    def check_linear_path(self, pose_start, pose_target, tcp=None, q_start=None, ik=None) -> None:
        """A linear move (moveL): the straight TCP line and the flange line must
        stay out of the keep-outs; with `ik` (pose, q_near -> q or None) and
        q_start the arm links are checked at up to 12 points along the line."""
        if not self.has_obstacles:
            return
        a, b = list(pose_start[:3]), list(pose_target[:3])
        self._check_line(a, b, "linear move, TCP path")
        self._check_line(flange_position_from_pose(pose_start, tcp),
                         flange_position_from_pose(pose_target, tcp), "linear move, flange path")
        self._check_flange_height(flange_position_from_pose(pose_target, tcp), "linear move, target")
        if ik is None or q_start is None:
            return
        n = int(min(max(math.ceil(math.dist(a, b) / 0.05), 1), 12))
        q = list(q_start)
        for i in range(1, n + 1):
            pose = interpolate_pose(pose_start, pose_target, i / n)
            q = ik(pose, q)
            if q is None:
                raise SafetyViolation(
                    f"linear move: no joint solution at {_fmt(pose[:3])} ({i / n:.0%} of the way) — the "
                    f"straight line leaves the arm's reach or runs into a singularity; use shorter legs, "
                    f"a different route, or move_joints for this leg")
            self.check_arm(q, tcp, what=f"linear move, {i / n:.0%} of the way")

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


def _fmt(xyz) -> str:
    return "(" + ", ".join(f"{float(v):+.3f}" for v in xyz[:3]) + ")"


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
    # INSTRUCTOR OVERRIDE for the real arm: no cell clipping — the taught
    # workspace alone bounds the motion (Workspace.profile() intersects with
    # this reach-sized box, i.e. keeps the taught box as is). Selected ONLY via
    # the environment variable UR5E_CELL=open; keeps cart_against_wall's speed
    # ceilings and payload/TCP requirements. Use when the robot works outside
    # the measured cell (demo on the other side, cell moved) — PolyScope's own
    # safety planes remain the authoritative layer. Not for student use.
    "real_open": SafetyLimits(
        name="real_open",
        tcp_speed_max=0.15,
        tcp_accel_max=0.5,
        joint_speed_max=0.5,
        joint_accel_max=0.8,
        joint_limits=_FULL_JOINTS,
        workspace=Box(x=(-0.95, 0.95), y=(-0.95, 0.95), z=(-0.10, 0.90)),
        payload_max_kg=0.5,
        require_payload=True,
        require_tcp=True,
        validated=True,            # deliberate: the taught box is the limit, instructor present
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


CELLS_DIR = Path(__file__).parent / "cells"


def load_cell(name: str) -> dict:
    """Read cells/<name>.json (format ur5e-cell/1) — the measured geometry of a
    cell with a solid body next to the arm. Written by scripts/teach_cart_cell.py."""
    path = CELLS_DIR / f"{name}.json"
    try:
        d = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as e:
        raise RuntimeError(f"cell file {path} missing or not valid JSON: {e}") from None
    if d.get("format") != "ur5e-cell/1":
        raise RuntimeError(f"{path}: unknown format {d.get('format')!r}, expected 'ur5e-cell/1'")
    cart, reach = d["cart"], d["reach"]
    for key, lo_hi in (("cart.x", cart["x"]), ("cart.y", cart["y"]), ("reach.x", reach["x"]), ("reach.y", reach["y"])):
        if not lo_hi[0] < lo_hi[1]:
            raise RuntimeError(f"{path}: {key} must be [low, high], got {lo_hi}")
    if not d["floor_z"] < cart["top_z"]:
        raise RuntimeError(f"{path}: floor_z must be below cart.top_z")
    for e in d.get("extras", []):
        for axis in "xyz":
            if not e[axis][0] < e[axis][1]:
                raise RuntimeError(f"{path}: extras '{e.get('name')}' {axis} must be [low, high], got {e[axis]}")
    return d


def cart_profile(d: dict) -> SafetyLimits:
    """SafetyLimits for a cart cell description (see load_cell): the robot on a
    free-standing cart, allowed to reach around and below the cart top.

      workspace box  reach-sized, from floor + floor_clearance up to reach.z_max
      keep-out       the cart body (footprint, floor .. top) and every box in
                     "extras" (handle, mounting plate ...); the tool keeps
                     side_clearance from their sides, arm links arm_radius
      floor          the TCP stays floor_clearance above it, arm links above it
                     (optional "flange_floor_clearance": a pendant floor plane the
                     flange must stay above — not used by cart_360)
      ceilings       the same slow values as cart_against_wall (TCP 0.15 m/s,
                     below the PolyScope tool-speed limit of 0.25 m/s)
    """
    cart, reach, floor = d["cart"], d["reach"], float(d["floor_z"])
    clearance = float(d.get("side_clearance", 0.05))
    keepout = [KeepOut("cart", Box(x=tuple(cart["x"]), y=tuple(cart["y"]), z=(floor, float(cart["top_z"]))),
                       side_clearance=clearance)]
    keepout += [KeepOut(str(e["name"]), Box(x=tuple(e["x"]), y=tuple(e["y"]), z=tuple(e["z"])),
                        side_clearance=clearance) for e in d.get("extras", [])]
    return SafetyLimits(
        name=str(d["name"]),
        tcp_speed_max=0.15,
        tcp_accel_max=0.5,
        joint_speed_max=0.5,
        joint_accel_max=0.8,
        joint_limits=_FULL_JOINTS,
        workspace=Box(x=tuple(reach["x"]), y=tuple(reach["y"]),
                      z=(floor + float(d.get("floor_clearance", 0.05)), float(reach["z_max"]))),
        payload_max_kg=0.5,
        require_payload=True,
        require_tcp=True,
        validated=bool(d.get("validated", False)),
        keepout=tuple(keepout),
        arm_radius=float(d.get("arm_radius", 0.06)),
        floor_z=floor,
        # only if the cell declares a pendant floor plane ("flange_floor_clearance"): 10 mm
        # above it, so the software rejects before the controller's plane has to act.
        # cart_360 has none (decided 2026-11-18: no planes on the pendant).
        flange_min_z=(floor + float(d["flange_floor_clearance"]) + 0.01
                      if d.get("flange_floor_clearance") is not None else None),
    )


# Arm on the free-standing cart, reaching around and below the cart top. Geometry
# + validated flag come from cells/cart_360.json (measured with
# scripts/teach_cart_cell.py); the same file drives the viewer scene
# scenes/cart_360 and the plane values for the pendant.
PROFILES["cart_360"] = cart_profile(load_cell("cart_360"))


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

Cell profiles with a keep-out (cart_360: the cart the robot stands on) also
check the PATH: a joint move is sampled in joint space, a linear move along
its straight line (arm links via the framework's own inverse kinematics on the
nominal model, iterated from the current joints — kinematics.ik_near). Profiles
without keep-outs behave exactly as before.

These software checks are SECONDARY to the robot's PolyScope Safety
Configuration, which is the authoritative protection. See safety.py.
"""

from __future__ import annotations

from . import config
from .connection import RobotConnection
from .kinematics import ik_near
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
                "geometry (and PolyScope config) and set validated=True first "
                "(instructor: the cell has to be measured — scripts/teach_cart_cell.py)")
        if self.limits.require_payload and not self.conn.payload_set:
            raise SafetyViolation("payload not set; call conn.set_payload(...) before real motion")
        if self.limits.require_tcp and not self.conn.tcp_set:
            raise SafetyViolation("TCP not set; call conn.set_tcp(...) before real motion")

    def move_j(self, q_target, speed=None, accel=None):
        self._preflight()
        speed = self.limits.joint_speed_max if speed is None else speed
        accel = self.limits.joint_accel_max if accel is None else accel
        self.limits.check_joint_speed(speed, accel)
        tcp = getattr(self.conn, "tcp", None)
        self.limits.check_joint_target(q_target, tcp=tcp)
        if self.limits.has_obstacles:
            self.limits.check_joint_path(self.conn.receive.getActualQ(), q_target, tcp=tcp)
        return self.conn.control.moveJ(list(q_target), speed, accel)

    def move_l(self, pose_target, speed=None, accel=None):
        self._preflight()
        speed = self.limits.tcp_speed_max if speed is None else speed
        accel = self.limits.tcp_accel_max if accel is None else accel
        self.limits.check_tcp_speed(speed, accel)
        tcp = getattr(self.conn, "tcp", None)
        self.limits.check_pose_target(pose_target, tcp=tcp)
        if self.limits.has_obstacles:
            start = self.conn.receive.getActualTCPPose()
            q_start = self.conn.receive.getActualQ()
            self.limits.check_linear_path(start, pose_target, tcp=tcp, q_start=q_start,
                                          ik=lambda pose, q_near: ik_near(pose, q_near, tcp))
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
        self.tcp = None                    # [x,y,z,rx,ry,rz] once set_tcp() was called

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
        self.tcp = list(pose6_m_rad)
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


# --- which cell profile bounds the arm -------------------------------------------
# The cell profile is the measured physical setup (safety.py PROFILES). A taught
# Workspace is clipped to it; Robot(profile=None) uses it directly.
REAL_CELL_DEFAULT = "cart_360"       # arm on the free-standing cart (HS26, from 2026-11-18)
SIM_CELL_DEFAULT = "sim_open"

_warned_open = False


def cell_profile_name(target: str) -> str:
    """Name of the cell profile for 'sim' / 'real'.

    Default: sim -> sim_open (free exploration), real -> cart_360.
    Environment variable UR5E_CELL (or the examples' --cell flag) overrides it:
      UR5E_CELL=cart_360            use the cart cell in the SIM too — rehearse a
                                    script against the cart keep-out before the lab
      UR5E_CELL=cart_against_wall   the earlier table-only cell
      UR5E_CELL=open                real arm, INSTRUCTOR OVERRIDE: no cell — only the
                                    taught workspace box limits the arm ('real_open')
    """
    global _warned_open
    choice = os.environ.get("UR5E_CELL", "").strip()
    if choice.lower() in ("open", "none", "off"):
        if target != "real":
            return SIM_CELL_DEFAULT
        if not _warned_open:
            print("[ur5e] !! UR5E_CELL=open: the measured cell profile is NOT applied — only the taught "
                  "workspace box limits the real arm (instructor override; PolyScope planes still apply)")
            _warned_open = True
        return "real_open"
    if choice:
        return choice
    return REAL_CELL_DEFAULT if target == "real" else SIM_CELL_DEFAULT

```

---

## `src/ur5e/kinematics.py`

```python
"""UR5e forward kinematics (DH-based).

Shared by the viewer and the safety/workspace checks. Positions are in the
robot base frame, meters. fk_frames/fk_flange_position give the FLANGE frame
(tool0); fk_tcp_position(q, tcp) applies a TCP offset so the safety check
tests the same point the taught workspace is expressed in. arm_spheres(q)
gives sample points along the physical links (for the keep-out check of cell
profiles that have a solid body next to the arm, e.g. the cart). The
authoritative boundary protection is the robot's PolyScope safety planes, not this.
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


def fk_tcp_position(q, tcp=None):
    """Position of the tool point for joint angles q (base frame, metres).
    tcp = [x, y, z, rx, ry, rz] offset relative to the flange (as set with
    set_tcp / the pendant's active TCP); None = the flange itself."""
    T = fk_frames(q)[-1]
    if tcp is not None:
        return T[:3, :3] @ np.asarray(tcp[:3], dtype=float) + T[:3, 3]
    return T[:3, 3]


def fk_flange_position(q):
    """xyz position of the flange (tool0) in the base frame."""
    return fk_frames(q)[-1][:3, 3]


# --- pose helpers (rotation vector <-> matrix; numpy only) --------------------

def rotvec_to_matrix(rv):
    """3x3 rotation matrix of a rotation vector [rx, ry, rz] (axis * angle, rad)."""
    rv = np.asarray(rv, dtype=float)
    angle = float(np.linalg.norm(rv))
    if angle < 1e-12:
        return np.eye(3)
    k = rv / angle
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def matrix_to_rotvec(R):
    """Rotation vector of a 3x3 rotation matrix (angle in 0..pi)."""
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    angle = math.acos(c)
    if angle < 1e-9:
        return np.zeros(3)
    if math.pi - angle < 1e-6:                      # near 180 deg: axis from the diagonal
        axis = np.sqrt(np.maximum((np.diag(R) + 1.0) / 2.0, 0.0))
        i = int(np.argmax(axis))
        for j in range(3):
            if j != i and R[i, j] + R[j, i] < 0:
                axis[j] = -axis[j]
        return axis / np.linalg.norm(axis) * angle
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * math.sin(angle))
    return axis * angle


def interpolate_pose(p0, p1, t):
    """Pose a fraction t (0..1) of the way from p0 to p1 the way a linear move
    (moveL) travels: position on the straight line, orientation turning evenly
    about one axis. Poses are [x, y, z, rx, ry, rz] (m, rotation vector rad)."""
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    R0, R1 = rotvec_to_matrix(p0[3:6]), rotvec_to_matrix(p1[3:6])
    R = R0 @ rotvec_to_matrix(matrix_to_rotvec(R0.T @ R1) * t)
    return [*(p0[:3] + (p1[:3] - p0[:3]) * t), *matrix_to_rotvec(R)]


def flange_position_from_pose(pose, tcp=None):
    """Flange (tool0) position for a TCP pose, given the TCP offset
    [x, y, z, rx, ry, rz] relative to the flange (None = TCP is the flange)."""
    p = np.asarray(pose[:3], dtype=float)
    if tcp is None:
        return p
    R_tcp = rotvec_to_matrix(pose[3:6])
    R_flange = R_tcp @ rotvec_to_matrix(tcp[3:6]).T
    return p - R_flange @ np.asarray(tcp[:3], dtype=float)


def ik_near(pose, q_near, tcp=None, max_iter=80, tol_pos=1e-5, tol_rot=1e-5):
    """Joint angles that put the TCP at `pose`, found by iterating from q_near
    on the nominal model (damped least squares, steps limited to 0.2 rad per
    joint) — so the answer is the solution CONNECTED to q_near, the one a
    linear move arrives at when it starts from q_near and the pose is close.
    Returns a list of 6 angles, or None if it does not converge (pose out of
    reach, or at a singularity).

    Used for the arm check along linear moves: chain it from the current
    joints through closely spaced poses. Not a general IK solver, and not used
    to command the robot. (The controller's own get_inverse_kin_has_solution
    is not usable through ur_rtde 1.6.3 on PolyScope 5.9: it answers False for
    reachable poses — found 2026-11-18 in URSim 5.9.4.)"""
    q = np.array(q_near, dtype=float)
    R_tcp = rotvec_to_matrix(pose[3:6])
    if tcp is not None:
        R_t = R_tcp @ rotvec_to_matrix(tcp[3:6]).T
        p_t = np.asarray(pose[:3], dtype=float) - R_t @ np.asarray(tcp[:3], dtype=float)
    else:
        R_t, p_t = R_tcp, np.asarray(pose[:3], dtype=float)

    def err(qq):
        T = fk_frames(qq)[-1]
        return np.concatenate([p_t - T[:3, 3], matrix_to_rotvec(R_t @ T[:3, :3].T)])

    eps = 1e-6
    for _ in range(max_iter):
        e = err(q)
        if np.linalg.norm(e[:3]) < tol_pos and np.linalg.norm(e[3:]) < tol_rot:
            return [float(v) for v in q]
        J = np.zeros((6, 6))
        for j in range(6):
            dq = np.zeros(6)
            dq[j] = eps
            J[:, j] = -(err(q + dq) - e) / eps
        step = np.linalg.solve(J.T @ J + 1e-6 * np.eye(6), J.T @ e)
        biggest = float(np.max(np.abs(step)))
        if biggest > 0.2:
            step *= 0.2 / biggest
        q = q + step
    return None


# --- physical arm, as spheres ---------------------------------------------------
# The DH frames above are a skeleton; the real links sit beside it: the upper arm
# tube runs 0.138 m out along the shoulder-lift axis, the forearm 0.007 m, wrist 1
# comes back out to d4 = 0.1333 m (ur_description 4.4.0, visual_parameters.yaml —
# the same numbers the viewer's robot_model uses). Nominal geometry, no calibration.
UPPER_ARM_OFFSET = 0.138
FOREARM_OFFSET = 0.007
_SPHERE_STEP = 0.04          # spacing of sample points along a link (m)


def _along(a, b, step=_SPHERE_STEP):
    n = max(1, int(math.ceil(float(np.linalg.norm(b - a)) / step)))
    return [a + (b - a) * (i / n) for i in range(n + 1)]


def arm_spheres(q, radius=0.06):
    """Sample points on the centre lines of the moving links for joint angles q
    (base frame, metres), as a list of (link name, xyz). A link is treated as
    the set of spheres of `radius` around its points. Covered: upper arm,
    elbow, forearm, wrist 1, wrist 2, wrist 3. NOT covered: base and shoulder
    (fixed on the mount) and whatever is mounted on the flange (see
    SafetyLimits.check_arm, which adds the flange->TCP line).

    Wrist 3 ends in the flange FACE: its last sphere sits one radius behind the
    flange, so the spheres reach exactly to the flange plane, not beyond it."""
    T = fk_frames(q)
    P = [t[:3, 3] for t in T]
    side = T[1][:3, 2]                       # shoulder-lift axis = lateral direction of the arm
    out = []
    a, b = P[1] + side * UPPER_ARM_OFFSET, P[2] + side * UPPER_ARM_OFFSET
    out += [("upper arm", p) for p in _along(a, b)[1:]]            # [0] is the shoulder itself
    out += [("elbow", p) for p in _along(P[2] + side * UPPER_ARM_OFFSET, P[2] + side * FOREARM_OFFSET)]
    out += [("forearm", p) for p in _along(P[2] + side * FOREARM_OFFSET, P[3] + side * FOREARM_OFFSET)]
    out += [("wrist 1", p) for p in _along(P[3], P[4])]
    out += [("wrist 2", p) for p in _along(P[4], P[5])]
    z6 = T[6][:3, 2]
    end = P[6] - z6 * radius
    if float(np.dot(end - P[5], z6)) > 0:
        out += [("wrist 3", p) for p in _along(P[5], end)]
    else:
        out.append(("wrist 3", P[5]))
    return out

```

---

## `src/ur5e/cell.py`

```python
"""Print the cell profile — the measured lab setup the checks run against.

STATUS: HS26 student framework (thin printing helper, no robot logic).

    python -m ur5e.cell                the real-arm cell (cart_360) and the sim default
    python -m ur5e.cell cart_360       one profile by name
    python -m ur5e.cell --all          every profile

The cell is selected by ur5e.config.cell_profile_name: sim_open in the
simulator, cart_360 on the real arm; `--cell NAME` on the examples (or the
environment variable UR5E_CELL) overrides it, e.g. `--cell cart_360` to
rehearse in the simulator against the cart keep-out.
"""

from __future__ import annotations

import sys

from .config import cell_profile_name
from .safety import PROFILES, get_profile


def main(argv=None) -> None:
    args = list(sys.argv[1:] if argv is None else argv)
    if "--all" in args:
        names = sorted(PROFILES)
    elif args:
        names = args
    else:
        names = list(dict.fromkeys([cell_profile_name("real"), cell_profile_name("sim")]))
        print(f"real arm -> '{names[0]}'    simulator -> '{names[-1]}'\n")
    for name in names:
        print(get_profile(name).describe(), "\n")


if __name__ == "__main__":
    main()

```

---

## `src/ur5e/cells/cart_360.json`

```text
{
  "format": "ur5e-cell/1",
  "name": "cart_360",
  "validated": true,
  "units_note": "metres, robot Base frame; z = 0 is the robot's mounting plane. Written by scripts/teach_cart_cell.py (instructor) - do not edit in a student clone.",
  "cart": {
    "x": [
      -0.566,
      0.124
    ],
    "y": [
      -0.345,
      0.345
    ],
    "top_z": -0.01
  },
  "floor_z": -0.8,
  "side_clearance": 0.02,
  "arm_radius": 0.06,
  "floor_clearance": 0.05,
  "reach": {
    "x": [
      -0.95,
      0.95
    ],
    "y": [
      -0.95,
      0.95
    ],
    "z_max": 1.0
  },
  "measured_at": "2026-11-18 09:16",
  "measured_with": "drawing 'cart' by Johannes Reck, 2026-11-18 (Documentation/assets/cart_Drawing.pdf), entered by hand",
  "note": "From the drawing; confirmed by JReck 2026-11-18: base frame aligned with the cart, long part of the top (566 mm from the robot axis) = -X, handle on the -Y side at the drawn position, 790 mm = floor to cart top, robot mounting plane 10 mm above the cart top.",
  "extras": [
    {
      "name": "handle",
      "x": [
        -0.455,
        0.01
      ],
      "y": [
        -0.405,
        -0.345
      ],
      "z": [
        -0.075,
        -0.035
      ],
      "source": "450 x 60 mm from the drawing; position read off the drawing's geometry and padded; confirmed by JReck 2026-11-18"
    },
    {
      "name": "mounting plate",
      "x": [
        -0.09,
        0.09
      ],
      "y": [
        -0.09,
        0.09
      ],
      "z": [
        -0.01,
        0.0
      ],
      "source": "180 x 180 x 10 mm, robot centred on it"
    }
  ]
}

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
| `00_teach_workspace.py` | teach the work area in 9 steps (8 corners + home) → `workspace.json` | — |
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
teach it once per setup with `00_teach_workspace.py` — eight corners and
home, the script only reads where the arm is, you move it (sim: pendant
**Move** tab; real robot: **freedrive button** on the pendant, supervised):

1. **low corners 1–4** — walk around the work area, tool on the surface at
   each corner
2. **high corners 1–4** — the same corners at the highest height the tool
   should reach (the ceiling)
3. **home** — a comfortable pose above the area, inside the box; every
   example starts and ends here (`robot.move_joints(ws.home_joints)`)

The box is the largest axis-aligned box that **fits inside** the eight
corners (per axis: from the highest of the four low values to the lowest of
the four high values). A table that is slightly rotated against the robot's
base axes or not perfectly level therefore gives a box that stays within
what you taught — never beyond it; `describe()` tells you when it had to
shrink. Old 4-point `workspace.json` files still load (same box as before).

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
orientation is the one you taught at low corner 1 (tool pointing down). Because
everything is relative to the box, the same script runs on a 300 mm demo
box in the sim and on the 600 × 500 mm cart in the lab.

`Robot(profile=ws)` turns the box into the safety profile for that run: moves
outside it are rejected before they are sent. On the **real robot** the box
is additionally clipped to the measured cell (`cart_360`: the arm on the cart,
with the cart body as a keep-out — see `docs/SAFETY_SETUP.md`) and keeps
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
2. Freedrive the arm to where you will work. The cell (`cart_360`, print it
   with `python -m ur5e.cell`) allows the space above the cart top and around
   the cart down towards the floor; the cart body itself is a keep-out. After
   teaching, the script prints the box; a `!! clipped` line means part of your
   box lies outside the cell — `ws.pose/grid/line` then use the clipped part,
   and a taught point listed as outside (e.g. `home`) must be re-taught. A
   `note: … reaches into the keep-out` line means the lower part of the box
   overlaps the cart: poses there are rejected.
3. `python examples/function_templates/00_teach_workspace.py --target real`
   — all eight corners where you work, `home` inside the box.
4. `python examples/function_templates/01_read_state.py --target real`
   (no motion) then `03_move_to.py --target real`, and so on.

The gate behind this (`allow_real=True` + validated profile + payload + TCP)
is unchanged; the flag only supplies it. Without `--target real` a script can
never move the real arm, even with `UR_HOST` set.

Flagged assumptions: poses are TCP poses with the pendant's active TCP at
teaching time; with a long tool, teach the high corners and `home` with extra
headroom (the joint check looks at the flange). The simulator's inputs always read 0.

```

---

## `examples/function_templates/00_teach_workspace.py`

```python
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
        # (inside the cell, see `python -m ur5e.cell`); --payload/--tcp if a tool is mounted
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

