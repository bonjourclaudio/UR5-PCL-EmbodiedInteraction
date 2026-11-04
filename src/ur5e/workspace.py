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
            cell = get_profile("cart_against_wall" if self.target == "real" else "sim_open").workspace
            named = [(f"low corner {i + 1}", q) for i, q in enumerate(self.corners_low)] + \
                    [(f"high corner {i + 1}", q) for i, q in enumerate(self.corners_high)]
            outside = [n for n, q in named if not cell.contains(q[:3])]
            out += (f"\n  !! clipped to the cell profile -> usable x {c.x[0]:+.3f}..{c.x[1]:+.3f}  "
                    f"y {c.y[0]:+.3f}..{c.y[1]:+.3f}  z {c.z[0]:+.3f}..{c.z[1]:+.3f} m "
                    f"(ws.pose/grid/line use this box)"
                    + (f"\n     corners outside the cell: {', '.join(outside)} (fine — the box is clipped)"
                       if outside else ""))
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
