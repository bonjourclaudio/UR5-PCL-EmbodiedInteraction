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
