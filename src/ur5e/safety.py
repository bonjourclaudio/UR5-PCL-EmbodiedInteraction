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

