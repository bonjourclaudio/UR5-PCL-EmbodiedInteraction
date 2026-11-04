"""Gripper v2 (LSS servo + T-CAN485) over the UR Tool Communication Interface.

STATUS: experimental — reference for the lab gripper (HS26). Runs in the sim with the mock gripper.
REAL-ARM ONLY for the live link (TCP <robot_ip>:54321 via the rs485 URCap);
URSim has no tool RS485 -> use MockGripper offline.

Talks the ASCII protocol of the gripper firmware (instructor):
    G<pct>  O  C  F<mA>  V<deg/s>  L  H  S  CAL MIN|MAX|RESET
each answered by one status line  *pct=..,deg=..,mA=..,target=..,cal=..,...

    from gripper_link import Gripper, MockGripper
    with Gripper("192.168.1.11") as g:
        g.open()                 # blocks until the jaws arrive (or timeout)
        g.close(force_mA=300)    # grip with a holding-current limit
        print(g.status())

Opening is a percentage of the calibrated stroke (0 = jaws touching, 100 =
~61 mm). Calibrate once with examples-gripper/gripper_calibrate.py.
"""

from __future__ import annotations

import socket
import time
from typing import Optional

TCI_PORT = 54321


def parse_status(line: str) -> dict:
    """'*pct=40,deg=120,mA=85,...' -> {'pct': 40, 'deg': 120, 'mA': 85, ...}"""
    line = line.strip()
    if not line.startswith("*"):
        raise ValueError(f"not a status line: {line!r}")
    out: dict = {}
    for item in line[1:].split(","):
        if "=" in item:
            k, v = item.split("=", 1)
            try:
                out[k] = int(v)
            except ValueError:
                out[k] = v
    return out


class GripperError(RuntimeError):
    pass


class Gripper:
    """Live gripper on the real arm (TCI forwarder, TCP <host>:54321)."""

    def __init__(self, host: str, port: int = TCI_PORT, timeout: float = 2.0,
                 settle_tol_pct: int = 4, move_timeout: float = 4.0) -> None:
        self.host, self.port, self.timeout = host, port, timeout
        self.settle_tol_pct = settle_tol_pct
        self.move_timeout = move_timeout
        self._sock: Optional[socket.socket] = None
        self._buf = b""
        self.open_loop = False      # True when the servo Tx line is not wired (status fb=0)
        self._speed_dps = 90
        self._stroke_deg = 52.0

    # --- lifecycle ---
    def connect(self) -> "Gripper":
        self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self._sock.settimeout(self.timeout)
        self._buf = b""
        st = self.status()
        self.open_loop = not st.get("fb", 1)
        self._speed_dps = max(5, int(st.get("v", 90)))
        self._stroke_deg = abs(st.get("open", 320) - st.get("closed", -200)) / 10.0
        if self.open_loop:
            print("[gripper] no servo feedback (Tx not wired) — OPEN LOOP: moves are timed, "
                  "no position/current read-back, no stall detection")
        if not st.get("cal"):
            print("[gripper] WARNING: not calibrated — using geometric defaults "
                  "(run examples-gripper/gripper_calibrate.py)")
        return self

    def close_link(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def __enter__(self) -> "Gripper":
        return self.connect()

    def __exit__(self, *exc) -> bool:
        self.close_link()
        return False

    # --- protocol ---
    def _cmd(self, line: str, retries: int = 1) -> dict:
        """Send one line, return the parsed status reply. A missing reply is
        retried once: the board reboots (~1.5 s) if the tool 12 V sags, e.g.
        when the servo stalls against a stop, and comes back within the retry."""
        if self._sock is None:
            raise GripperError("gripper not connected; call connect() first")
        for attempt in range(retries + 1):
            self._sock.sendall((line.strip() + "\n").encode("ascii"))
            t0 = time.time()
            while time.time() - t0 <= self.timeout:
                while b"\n" in self._buf:
                    raw, self._buf = self._buf.split(b"\n", 1)
                    text = raw.decode("ascii", "ignore")
                    if text.startswith("*"):
                        if attempt:
                            print("[gripper] board did not answer at first — it probably reset "
                                  "(servo stalled / supply sag); command repeated")
                        return parse_status(text)
                try:
                    data = self._sock.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    raise GripperError("TCI connection closed")
                self._buf += data
        raise GripperError(f"no reply to {line!r} within {self.timeout * (retries + 1):.0f}s "
                           "(board reset or RS485 link lost)")

    # --- API ---
    def status(self) -> dict:
        return self._cmd("S")

    def goto(self, pct: float, wait: bool = True) -> dict:
        """Move to an opening of 0 (closed) .. 100 (open) percent of the stroke."""
        pct = int(max(0, min(100, round(pct))))
        before = self._last_pct
        st = self._cmd(f"G{pct}")
        self._last_pct = pct
        if not wait:
            return st
        if self.open_loop:
            # timed move: stroke fraction / speed, plus a little settling margin
            frac = abs(pct - before) / 100.0 if before is not None else 1.0
            time.sleep(frac * self._stroke_deg / self._speed_dps + 0.3)
            return self.status()
        return self._wait(pct)

    _last_pct: Optional[int] = None

    def move_raw(self, tenths_deg: int) -> dict:
        """Raw servo angle in tenths of a degree (finding the limits without feedback)."""
        self._last_pct = None
        return self._cmd(f"T{int(tenths_deg)}")

    def set_limits(self, closed_tenths: int, open_tenths: int) -> dict:
        """Store the CLOSED / OPEN servo angles directly (open-loop calibration)."""
        self._cmd(f"MIN{int(closed_tenths)}")
        return self._cmd(f"MAX{int(open_tenths)}")

    def move_relative(self, deg: float, wait: bool = True) -> dict:
        """Move by +/- deg from wherever the servo is now (no calibration needed).
        Demo use: set the jaws mid-stroke by hand while limp, then +N closes, -N opens."""
        self._last_pct = None
        st = self._cmd(f"R{int(round(deg * 10))}")
        if wait:
            time.sleep(abs(deg) / self._speed_dps + 0.3)
        return st

    def open(self, wait: bool = True) -> dict:
        return self.goto(100, wait)

    def close(self, force_mA: Optional[int] = None, wait: bool = True) -> dict:
        """Close the jaws. force_mA limits the servo's holding current (grip force);
        when gripping an object the jaws stop early and pct stays > 0."""
        if force_mA is not None:
            self.set_force(force_mA)
        return self.goto(0, wait)

    def set_force(self, mA: int) -> dict:
        return self._cmd(f"F{int(mA)}")

    def set_speed(self, deg_per_s: int) -> dict:
        return self._cmd(f"V{int(deg_per_s)}")

    def limp(self) -> dict:
        return self._cmd("L")

    def hold(self) -> dict:
        return self._cmd("H")

    def calibrate(self, which: str) -> dict:
        """which = 'min' (jaws closed), 'max' (jaws open) or 'reset'."""
        return self._cmd("CAL " + which.upper())

    def _wait(self, target_pct: int) -> dict:
        """Poll until the jaws are within tolerance of the target, or the
        position stops changing (object gripped), or the move times out."""
        t0 = time.time()
        last = None
        stable = 0
        while True:
            st = self.status()
            pct = st.get("pct", -1)
            if abs(pct - target_pct) <= self.settle_tol_pct:
                return st
            if last is not None and abs(pct - last) <= 1:
                stable += 1
                if stable >= 4:          # ~0.4 s without progress -> blocked by an object
                    return st
            else:
                stable = 0
            last = pct
            if time.time() - t0 > self.move_timeout:
                return st
            time.sleep(0.1)


class MockGripper:
    """Offline stand-in with the same API (prints what it would do)."""

    def __init__(self, move_time: float = 0.6) -> None:
        self.move_time = move_time
        self.pct = 100
        self.force = 350

    def connect(self) -> "MockGripper":
        print("[gripper:mock] connected")
        return self

    def close_link(self) -> None:
        pass

    def __enter__(self) -> "MockGripper":
        return self.connect()

    def __exit__(self, *exc) -> bool:
        return False

    open_loop = False

    def status(self) -> dict:
        return {"pct": self.pct, "deg": -200 + 520 * self.pct // 100, "mA": 60,
                "target": self.pct, "fb": 1, "cal": 1, "f": self.force, "v": 90}

    def move_raw(self, tenths_deg: int) -> dict:
        print(f"[gripper:mock] raw {tenths_deg / 10:.1f} deg")
        return self.status()

    def set_limits(self, closed_tenths: int, open_tenths: int) -> dict:
        print(f"[gripper:mock] limits {closed_tenths} / {open_tenths}")
        return self.status()

    def move_relative(self, deg: float, wait: bool = True) -> dict:
        print(f"[gripper:mock] relative {deg:+.1f} deg")
        if wait:
            time.sleep(self.move_time)
        return self.status()

    def goto(self, pct: float, wait: bool = True) -> dict:
        self.pct = int(max(0, min(100, round(pct))))
        print(f"[gripper:mock] -> {self.pct}%")
        if wait:
            time.sleep(self.move_time)
        return self.status()

    def open(self, wait: bool = True) -> dict:
        return self.goto(100, wait)

    def close(self, force_mA: Optional[int] = None, wait: bool = True) -> dict:
        if force_mA is not None:
            self.force = int(force_mA)
        return self.goto(0, wait)

    def set_force(self, mA: int) -> dict:
        self.force = int(mA)
        return self.status()

    def set_speed(self, deg_per_s: int) -> dict:
        return self.status()

    def limp(self) -> dict:
        print("[gripper:mock] limp")
        return self.status()

    def hold(self) -> dict:
        return self.status()

    def calibrate(self, which: str) -> dict:
        print(f"[gripper:mock] calibrate {which}")
        return self.status()


def make_gripper(kind: str, host: Optional[str] = None):
    """kind: 'tci' (real arm) | 'mock'. Returns an unconnected gripper."""
    if kind == "tci":
        if not host:
            raise ValueError("gripper 'tci' needs the robot host")
        return Gripper(host)
    return MockGripper()
