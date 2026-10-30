"""Count step pulses and measure their width with the XMEGA's own timer, configured through the Atmel-ICE.

Nothing is wired to the step outputs on the bench, so the chip counts them itself: an event-system
channel takes a motor's step pin as its source and a spare timer/counter (TCC1, unallocated in TinyG)
either counts rising edges (16-bit, one motor at a time) or captures pulse widths at the CPU clock.
This is independent of the firmware's own bookkeeping, which is also readable (en.en[m].encoder_steps,
the signed count of steps the DDA ISR issued). The timer, channel and pin assignments come from the
board file. Arming and reading halt the CPU for a few milliseconds.
"""


def _int(v):
    return int(v, 0) if isinstance(v, str) else int(v)


class StepCounter:
    def __init__(self, ice):
        self.ice, self.board = ice, ice.board
        sc = self.board.data["step_counter"]
        self.base = _int(sc["timer_base"])
        self.reg = {k: self.base + _int(v) for k, v in sc["timer_registers"].items()}
        self.sc = {k: (_int(v) if isinstance(v, (str, int)) and k not in ("note",) and not isinstance(v, dict) else v)
                   for k, v in sc.items()}
        self.motor = None

    def _pin(self, motor):
        m = self.board.data["motors"][str(motor)]
        return m["port"], m["step"]

    def _set_isc(self, motor, isc):
        port, bit = self._pin(motor)
        addr = self.board.pinctrl(port, bit)
        isc_cfg = {k: _int(v) for k, v in self.board.data["mcu"]["pinctrl_isc"].items()}
        ctrl = self.ice.read(addr, 1)[0]
        self.ice.write(addr, bytes([(ctrl & ~isc_cfg["mask"] & 0xFF) | isc_cfg[isc]]))

    def _route(self, motor):
        port, bit = self._pin(motor)
        mux = _int(self.sc["evsys_port_pin_mux"][port]) + bit
        self.ice.write(_int(self.sc["evsys_base"]) + self.sc["event_channel"], bytes([mux]))   # CHnMUX

    # ---- step counting ----------------------------------------------------------------

    def arm(self, motor):
        """Start counting rising edges on a motor's step pin from zero."""
        with self.ice.halted():
            self.ice.write(self.reg["CTRLA"], b"\x00")              # stop the timer while configuring
            self._set_isc(motor, "rising")                          # one event per step pulse
            self._route(motor)
            self.ice.write(self.reg["CTRLB"], b"\x00")              # normal mode
            self.ice.write(self.reg["CTRLD"], b"\x00")              # no event action
            self.ice.write(self.reg["PER"], b"\xff\xff")
            self.ice.write(self.reg["CNT"], b"\x00\x00")
            self.ice.write(self.reg["CTRLA"], bytes([self.sc["clksel_evch0"]]))   # clock = event channel
        self.motor = motor

    def count(self):
        """Step pulses since arm() (16-bit; keep test moves under 65535 steps)."""
        lo, hi = self.ice.read(self.reg["CNT"], 2)
        return lo | (hi << 8)

    def disarm(self):
        with self.ice.halted():
            self.ice.write(self.reg["CTRLA"], b"\x00")
            if self.motor is not None:
                self._set_isc(self.motor, "bothedges")              # the firmware's pin setting
        self.motor = None

    # ---- pulse width ------------------------------------------------------------------

    def arm_pulse_width(self, motor):
        """Capture step pulse widths (high time) on a motor's step pin at the CPU clock."""
        with self.ice.halted():
            self.ice.write(self.reg["CTRLA"], b"\x00")
            self._set_isc(motor, "bothedges")                       # pulse-width capture needs both edges
            self._route(motor)
            self.ice.write(self.reg["PER"], b"\xff\xff")
            self.ice.write(self.reg["CNT"], b"\x00\x00")
            self.ice.write(self.reg["CCA"], b"\x00\x00")
            self.ice.write(self.reg["CTRLB"], bytes([self.sc["ccaen"]]))
            self.ice.write(self.reg["CTRLD"], bytes([self.sc["evact_pw"] | self.sc["evsel_ch0"]]))
            self.ice.write(self.reg["CTRLA"], bytes([self.sc["clksel_div1"]]))
        self.motor = motor

    def last_pulse_width_us(self):
        """Width of the most recently captured pulse, in microseconds (0 if none yet)."""
        lo, hi = self.ice.read(self.reg["CCA"], 2)
        return (lo | (hi << 8)) * 1e6 / self.sc["cpu_hz"]

    # ---- firmware's own count -------------------------------------------------------

    def firmware_steps(self, motor):
        """Signed steps the DDA ISR has issued for a motor (TinyG's step-counting 'encoder')."""
        expr = self.board.symbols["encoder_steps"].format(motor_index=int(motor) - 1)
        return self.ice.peek(expr)
