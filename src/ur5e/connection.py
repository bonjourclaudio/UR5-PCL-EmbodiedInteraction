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
