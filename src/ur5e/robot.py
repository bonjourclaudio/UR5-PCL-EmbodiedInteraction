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
