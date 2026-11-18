"""Hot tool: temperature reading, on/off control and over-temperature cutoff.

HeaterGuard runs in a background thread next to the motion (moves block the
main thread for many seconds at engraving speed, so checks between moves are
not enough). Every PERIOD seconds it:

    1. reads the temperature  (real: robot.read_analog_in(pin) -> volts -> degC
                               sim:  FakeHotend, a simple thermal model, because
                                     URSim analog inputs always read 0 V)
    2. trips (latched, heater OFF, stays off) if
         - temperature >= max_c
         - temperature below min_valid_c (0 V = broken wire / transmitter unpowered)
    3. otherwise, while enabled, keeps the setpoint with on/off control:
         ON below setpoint - hysteresis, OFF at/above setpoint

The heater is switched with a digital output:
    out="tool" -> robot.set_tool_digital_out(pin, ...)   tool flange TO0/TO1
    out="std"  -> robot.set_digital_out(pin, ...)        control box DO0..7
Leaving the `with` block (normally, on an exception or Ctrl-C) switches it OFF.

Sensor: a PT100 is a resistor, it can't go straight into an analog input.
It needs a PT100 transmitter with a 0-10 V output, wired to the control box's
standard analog input (set to Voltage in the pendant I/O tab). Enter the
transmitter's range as volts_range / temp_range.
"""

from __future__ import annotations

import threading
import time

PERIOD = 0.1   # s between temperature checks


class FakeHotend:
    """SIM ONLY — stands in for the PT100 because URSim inputs read 0.

    First-order thermal model: heats at `rate` degC/s when on, cools towards
    ambient. With runaway=True it ignores OFF (a stuck heater / welded relay)
    so you can watch the cutoff trip in the sim.
    """

    def __init__(self, ambient=22.0, rate=6.0, cooling=0.02, runaway=False):
        self.t, self.ambient, self.rate, self.cooling = ambient, ambient, rate, cooling
        self.runaway = runaway
        self.heating = False
        self._last = time.monotonic()

    def read(self) -> float:
        now = time.monotonic()
        dt, self._last = now - self._last, now
        power = self.rate if (self.heating or self.runaway) else 0.0
        self.t += dt * (power - self.cooling * (self.t - self.ambient))
        return self.t


class HeaterGuard(threading.Thread):
    def __init__(self, robot, *, setpoint_c, max_c, hysteresis_c=3.0, out="tool", pin=0,
                 sensor_pin=0, volts_range=(0.0, 10.0), temp_range=(0.0, 300.0),
                 min_valid_c=5.0, fake: FakeHotend | None = None):
        super().__init__(daemon=True, name="heater-guard")
        if not setpoint_c < max_c:
            raise ValueError(f"setpoint {setpoint_c} degC must be below the cutoff {max_c} degC")
        self.robot, self.fake = robot, fake
        self.setpoint_c, self.max_c, self.hyst = setpoint_c, max_c, hysteresis_c
        self.out, self.pin, self.sensor_pin = out, pin, sensor_pin
        self.volts_range, self.temp_range, self.min_valid_c = volts_range, temp_range, min_valid_c
        self.temperature = float("nan")
        self.heater_on = False
        self.tripped: str | None = None      # reason, once tripped (latched)
        self.max_gap = 0.0                   # longest pause between checks (watchdog health)
        self._enabled = False
        self._halt = threading.Event()

    # ----------------------------------------------------------- hardware --
    def _read_c(self) -> float:
        if self.fake is not None:
            return self.fake.read()
        volts = self.robot.read_analog_in(self.sensor_pin)
        (v0, v1), (t0, t1) = self.volts_range, self.temp_range
        return t0 + (volts - v0) * (t1 - t0) / (v1 - v0)

    def _switch(self, on: bool) -> None:
        if self.out == "tool":
            self.robot.set_tool_digital_out(self.pin, on)
        else:
            self.robot.set_digital_out(self.pin, on)
        if self.fake is not None:
            self.fake.heating = on
        self.heater_on = on

    # ------------------------------------------------------------- thread --
    def run(self) -> None:
        last = time.monotonic()
        while not self._halt.is_set():
            now = time.monotonic()
            self.max_gap = max(self.max_gap, now - last)
            last = now
            try:
                t = self._read_c()
            except Exception as e:                       # connection lost etc.
                self.trip(f"sensor read failed: {e}")
                break
            self.temperature = t
            if self.tripped:
                pass
            elif t >= self.max_c:
                self.trip(f"over-temperature {t:.1f} degC >= {self.max_c:.0f} degC")
            elif t < self.min_valid_c:
                self.trip(f"implausible reading {t:.1f} degC — sensor/transmitter disconnected?")
            elif self._enabled:
                if t < self.setpoint_c - self.hyst and not self.heater_on:
                    self._switch(True)
                elif t >= self.setpoint_c and self.heater_on:
                    self._switch(False)
            elif self.heater_on:
                self._switch(False)
            self._halt.wait(PERIOD)

    def trip(self, reason: str) -> None:
        if not self.tripped:
            self.tripped = reason
            print(f"\n[heater] CUTOFF: {reason} -> heater OFF (latched)")
        self._enabled = False
        try:
            self._switch(False)
        except Exception as e:
            print(f"[heater] could not switch the output off: {e}")

    # ---------------------------------------------------------------- API --
    def enable(self) -> None:
        if not self.tripped:
            self._enabled = True

    def disable(self) -> None:
        self._enabled = False

    def is_hot(self, tolerance_c=5.0) -> bool:
        return not self.tripped and self.temperature >= self.setpoint_c - tolerance_c

    def wait_until_hot(self, timeout_s=300.0, tolerance_c=3.0) -> bool:
        """Block until within tolerance of the setpoint. False on trip or timeout."""
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            if self.tripped:
                return False
            if self.is_hot(tolerance_c):
                print(f"\r[heater] {self.temperature:6.1f} degC  ready" + " " * 10)
                return True
            print(f"\r[heater] {self.temperature:6.1f} degC  heating to {self.setpoint_c:.0f} ...",
                  end="", flush=True)
            time.sleep(0.5)
        self.trip(f"did not reach {self.setpoint_c:.0f} degC within {timeout_s:.0f} s")
        return False

    def __enter__(self) -> "HeaterGuard":
        self._switch(False)
        self.start()
        time.sleep(3 * PERIOD)               # first readings before anything moves
        return self

    def __exit__(self, *exc) -> bool:
        self._enabled = False
        self._halt.set()
        self.join(timeout=2)
        try:
            self._switch(False)
        finally:
            print(f"[heater] OFF ({self.temperature:.1f} degC at exit)")
        return False
