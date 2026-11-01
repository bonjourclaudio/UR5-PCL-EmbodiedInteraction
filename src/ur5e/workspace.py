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
