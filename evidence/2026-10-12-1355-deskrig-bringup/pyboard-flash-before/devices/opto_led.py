from pyControl.hardware import Digital_output


class Opto_LED(Digital_output):
    """TTL-gated LED optogenetics driver using a Plexon LD-1

    burst() and configure() each accept the pulse timing as exactly one of
    three equivalent parameter combinations:
      - freq + duty_cycle
      - freq + pulse_width_ms
      - pulse_width_ms + wait_width_ms
    Whichever combination is given, the others are derived internally and
    passed to Digital_output.pulse().
    """

    def __init__(self, pin, inverted=False):
        super().__init__(pin, inverted=inverted)
        self._freq = None
        self._duty_cycle = None

    def burst(self, n_pulses=None, freq=None, duty_cycle=None, pulse_width_ms=None, wait_width_ms=None):
        """Fires exactly n_pulses at the given timing, then auto-stops.

        Timing must be given as exactly one of: freq+duty_cycle,
        freq+pulse_width_ms, or pulse_width_ms+wait_width_ms.
        """
        if n_pulses is None:
            raise ValueError("burst() requires n_pulses")
        freq, duty_cycle = self._resolve_timing(freq, duty_cycle, pulse_width_ms, wait_width_ms)
        self.pulse(freq=freq, duty_cycle=duty_cycle, n_pulses=n_pulses)

    def configure(self, freq=None, duty_cycle=None, pulse_width_ms=None, wait_width_ms=None):
        """Sets the fixed pulse timing used by start().

        Timing must be given as exactly one of: freq+duty_cycle,
        freq+pulse_width_ms, or pulse_width_ms+wait_width_ms.
        """
        self._freq, self._duty_cycle = self._resolve_timing(
            freq, duty_cycle, pulse_width_ms, wait_width_ms)

    def start(self):
        """Begins a free-running pulse train at the configured timing.

        Safe to call repeatedly across many trials in one session --
        Digital_output allocates its hardware timer once, on the first
        pulse()/start() call, and reuses it for the rest of the session.
        """
        assert self._freq is not None, "call configure() before start()"
        self.pulse(freq=self._freq, duty_cycle=self._duty_cycle, n_pulses=False)

    def stop(self):
        """Ends the pulse train. Safe to call even if not currently running."""
        self.off()

    # --- internal timing-parameter resolution --------------------------------

    def _resolve_timing(self, freq, duty_cycle, pulse_width_ms, wait_width_ms):
        has_freq, has_duty = freq is not None, duty_cycle is not None
        has_pw, has_ww = pulse_width_ms is not None, wait_width_ms is not None

        if has_freq and has_duty and not has_pw and not has_ww:
            return freq, duty_cycle
        if has_freq and has_pw and not has_duty and not has_ww:
            return freq, self._duty_cycle_from_pulse_width(freq, pulse_width_ms)
        if has_pw and has_ww and not has_freq and not has_duty:
            return self._freq_duty_cycle_from_pulse_and_wait(pulse_width_ms, wait_width_ms)

        raise ValueError(
            "Provide exactly one of: (freq, duty_cycle), (freq, pulse_width_ms), "
            "or (pulse_width_ms, wait_width_ms). Got freq={}, duty_cycle={}, "
            "pulse_width_ms={}, wait_width_ms={}."
            .format(freq, duty_cycle, pulse_width_ms, wait_width_ms))

    def _duty_cycle_from_pulse_width(self, freq, pulse_width_ms):
        return self._duty_cycle_from_period(1000 / freq, pulse_width_ms)

    def _freq_duty_cycle_from_pulse_and_wait(self, pulse_width_ms, wait_width_ms):
        period_ms = pulse_width_ms + wait_width_ms
        return 1000 / period_ms, self._duty_cycle_from_period(period_ms, pulse_width_ms)

    def _duty_cycle_from_period(self, period_ms, pulse_width_ms):
        if pulse_width_ms > period_ms:
            raise ValueError(
                "pulse_width_ms={} exceeds the {} ms period".format(pulse_width_ms, period_ms))
        duty_cycle = round(pulse_width_ms / period_ms * 100)
        if duty_cycle % 5 != 0:
            raise ValueError(
                "pulse_width_ms={} over a {} ms period gives a {}% duty cycle, "
                "which is not a multiple of 5 (required by Digital_output.pulse())"
                .format(pulse_width_ms, period_ms, duty_cycle))
        return duty_cycle
