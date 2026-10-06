"""pyControl device class for one mouse-joystick board.

Runs on the pyControl controller in MicroPython. Speaks the protocol in
`../../../mouse-joystick-pcb/docs/uart-protocol.md` to one RP2040 board over one
behavior port. One object per board; two boards means two objects on two ports.

Target controller is **Breakout Board 1.2**, whose UART-capable ports are 1
(`UART4`), 3 (`USART1`) and 4 (`USART3`) — verified on the controller
2026-09-03. This rig uses **1 and 4**; port 3 belongs to `LaserLickPort`, the
only port with both I2C and a free `DIO_C`. The `port.UART` assertion below is what
turns a wrong port into a loud failure at hardware-definition load rather than a
link that silently never receives.

Shaped after `~/Repositories/github/syringe-pump`'s `PumpController`, which is the
lab's working pyControl peripheral: same `machine.UART`, same `UART_handler` plus
`UART.IRQ_RXIDLE`, same `;`-delimited newline framing. It differs where this board
differs - the pump is a request/response dispenser, this drives a motor against an
animal under a safety heartbeat, so sequence numbers, acknowledgements and a fault
model are added.

Hardware definition:

    from devices import *

    board = Breakout_1_2()
    left  = MouseJoystick(port=board.port_1, event_name="joystick_L")
    right = MouseJoystick(port=board.port_4, event_name="joystick_R")

Task:

    def all_states(event):
        if event == "joystick_L":
            for message in hw.left.poll():
                ...

`MouseJoystickProtocol` is named here so pyControl uploads
`mouse_joystick_protocol.py` alongside this file - see that file's header for why
the codec has to be a class.
"""

from machine import UART

from devices.uart_handler import UART_handler

# Package-qualified, matching pyControl's own sibling imports (`from devices.poke
# import _Poke` in five_poke.py, `from devices.MCP import MCP23017` in
# port_expander.py). On the pyboard the uploaded files live inside the `devices`
# package and `/flash/devices` is not on sys.path, so a bare
# `from mouse_joystick_protocol import ...` raises ImportError there while
# working perfectly on a host - which is the worst possible way to find out.
from devices.mouse_joystick_protocol import MouseJoystickProtocol


class MouseJoystick:
    # Every instance's UART_handler event name, so a second board given the same
    # name fails at construction instead of silently stealing the first board's
    # events. `validation-plan.md` Gate C requires that two instances on two
    # ports do not collide, and a class-level registry is the only place that can
    # be checked - neither instance can see the other.
    #
    # A hardware-definition reload re-imports the module and resets this, which
    # is the behaviour you want: reloading is how you legitimately rebuild both.
    registered_event_names = set()

    # rxbuf sized as the draft requires: at least the longest line plus margin.
    # 130 is the syringe pump's value and exceeds MAX_LINE (120).
    # 2048: a 32-line FETCH chunk at 115200 lasts ~70 ms and IRQ_RXIDLE may
    # not fire until it ends (protocol v1, 2026-10-01; was 130).
    RX_BUFFER = 2048

    def __init__(self, port, event_name, baudrate=115200):
        assert port.UART is not None, "! MouseJoystick needs a port with a UART."
        assert event_name not in MouseJoystick.registered_event_names, (
            "! MouseJoystick event_name '%s' is already in use. "
            "Two boards need two distinct names." % event_name
        )
        MouseJoystick.registered_event_names.add(event_name)

        self.event_name = event_name
        self.uart = UART(port.UART)
        # timeout=0: a read must never wait. With timeout=100 an empty buffer
        # held the pyControl main loop for the whole timeout (a 20 ms poll
        # measured at 72 ms, 2026-10-01 review). poll() reads only what any()
        # reports and reassembles partial lines itself.
        self.uart.init(baudrate, bits=8, parity=None, stop=1, timeout=0, rxbuf=self.RX_BUFFER)

        handler = UART_handler(event_name)
        self.uart.irq(trigger=UART.IRQ_RXIDLE, handler=handler.ISR)

        self._rx = ""
        self._seq = 0
        self._pending = {}

        # Where `ev;S` stream samples go: an object with put(t_us, angle, torque),
        # for the AnalogChannels of the run bundle. They are never returned by
        # poll() as messages and must never become framework events (spec v1).
        self.stream_sink = None
        self.caps = frozenset()

        # Filled in by identify(). Left None until then on purpose: a task that
        # reads self.serial before identifying should get None rather than a
        # plausible-looking default it might record as provenance.
        self.serial = None
        self.firmware_version = None
        self.protocol_version = None
        self.boot_reason = None  # ID boot=: por, wdt, reboot
        self.boot_crumb = None  # ID crumb=: last bring-up breadcrumb before a watchdog reboot
        self.axis_count = None

        self._read_available()  # discard anything buffered from before this object existed

    # ------------------------------------------------------------------ link

    def _read_available(self):
        """Return the bytes already received, or None, without ever waiting."""
        pending = self.uart.any()
        if not pending:
            return None
        return self.uart.read(pending)

    def _next_seq(self):
        self._seq = (self._seq + 1) % MouseJoystickProtocol.SEQ_MODULUS
        return self._seq

    def _send(self, verb, *args):
        """Write one command. Returns the sequence number it was sent under."""
        seq = self._next_seq()
        line = MouseJoystickProtocol.encode_command(verb, seq, args)
        self._pending[seq] = verb
        # Bounded: a board that never answers must not grow this without limit.
        if len(self._pending) > MouseJoystickProtocol.SEQ_MODULUS // 8:
            self._pending.clear()
        self.uart.write(line)
        return seq

    def poll(self):
        """Drain the link and return decoded messages, oldest first.

        Call this from the task when the `event_name` event fires. Returns a list
        of tuples as produced by `MouseJoystickProtocol.decode_reply`, so a task
        can dispatch on the kind without string matching.

        Reads rather than readlines: a UART read can return a partial line, and a
        parser that assumes whole lines is a parser that works on the bench and
        fails under load.
        """
        data = self._read_available()
        if data:
            if isinstance(data, (bytes, bytearray)):
                data = data.decode("utf-8")
            self._rx += data

        messages = []
        while "\n" in self._rx:
            line, _, self._rx = self._rx.partition("\n")
            if not line.strip("\r"):
                continue
            message = MouseJoystickProtocol.decode_reply(line)
            kind = message[0]
            if kind in (MouseJoystickProtocol.KIND_ACK, MouseJoystickProtocol.KIND_NAK):
                self._pending.pop(message[1], None)
            if kind == MouseJoystickProtocol.KIND_EVENT and message[1] == "S":
                # Telemetry, not an event: route to the analog sink and drop.
                t_us, data = message[2]
                if self.stream_sink is not None:
                    self.stream_sink.put(t_us, data.get("angle"), data.get("torque"))
                continue
            messages.append(message)

        # A partial line longer than any legal line means the link is desynced;
        # dropping it resynchronises at the next newline instead of wedging.
        if len(self._rx) > MouseJoystickProtocol.MAX_LINE:
            self._rx = ""

        return messages

    # ----------------------------------------------------------- discovery

    def identify(self):
        """Send `ID`. The reply is delivered through poll(), then apply_identity()."""
        return self._send("id")

    def apply_identity(self, data):
        """Record an `ID` reply, refusing a board this class cannot drive.

        The draft: "The pyControl device class must refuse to operate against an
        incompatible major version rather than guessing. The axis count is what
        distinguishes this board from a twin-axis board on the same link - do not
        let the class infer it from the product name."

        Both checks are assertions rather than return codes because continuing
        past either one means commanding a motor through a protocol the board
        does not implement.
        """
        proto = int(data["proto"])
        axes = int(data["axes"])
        # 2026-10-02: why the board booted ("por", "wdt", "reboot") and the
        # last bring-up breadcrumb before a watchdog reboot; a task that sees
        # "wdt" has a board that rebooted itself out of a stuck step.
        self.boot_reason = data.get("boot")
        self.boot_crumb = int(data["crumb"]) if "crumb" in data else None

        # v2 is the protocol v1 proposal of 2026-10-01 (BRAKE/WATCH/CFG/FETCH;
        # FREEWHEEL/ARMPULL gone). Earlier boards are refused rather than driven
        # with verbs they do not have.
        assert proto == MouseJoystickProtocol.PROTOCOL_VERSION, (
            "! MouseJoystick: board speaks protocol v%d, this class speaks v%d."
            % (proto, MouseJoystickProtocol.PROTOCOL_VERSION)
        )
        assert axes == MouseJoystickProtocol.AXIS_COUNT, (
            "! MouseJoystick: board reports %d axes, this class drives %d. "
            "A twin-axis board needs its own device class." % (axes, MouseJoystickProtocol.AXIS_COUNT)
        )

        self.protocol_version = proto
        self.axis_count = axes
        self.serial = data.get("serial")
        self.firmware_version = data.get("fw")
        # Features are gated on capabilities, never on the firmware version.
        self.caps = frozenset((data.get("caps") or "").split("+")) - {""}
        return self.serial

    def has(self, capability):
        return capability in self.caps

    # ------------------------------------------------------------- commands

    def ping(self):
        """The safety heartbeat. Must be sent more often than the host timeout."""
        return self._send("ping")

    def status(self):
        return self._send("status")

    def limits(self):
        return self._send("limits")

    def hv(self, on):
        return self._send("hv", "ON" if on else "OFF")

    def hv_state(self):
        return self._send("hv", "STATE")

    def home(self, timeout_ms=None):
        if timeout_ms is None:
            return self._send("home")
        return self._send("home", timeout_ms)

    def arm(self):
        return self._send("arm")

    def disarm(self):
        return self._send("disarm")

    def move_to(self, degrees):
        """Absolute. Rejected by the board outside its travel limits."""
        return self._send("moveto", degrees)

    def move(self, degrees):
        """Relative. Rejected if the resulting position is outside travel."""
        return self._send("move", degrees)

    def stop(self):
        """Controlled stop. Always permitted."""
        return self._send("stop")

    # --------------------------------------------------- protocol v1 verbs

    def brake(self, torque_ncm):
        """Engage the brake at `torque_ncm` and release the drive, as one board
        operation; `ev;RELEASED` follows the ack. On a board whose caps lack
        `brake` the torque is ignored and only the release happens.

        A subsequent `move_to`, `home` or `stop` re-engages the driver
        implicitly (resyncing position from the encoder first) and emits
        `ev;TRIAL` if a `watch` was armed.
        """
        return self._send("brake", torque_ncm)

    def watch(self, pull_deg, push_deg, timeout_ms, hold_ms=None, touch_ncm=None, maxdur_ms=None):
        """Arm the one-shot classifier (RELEASED only). Exactly one of
        `ev;PULL`, `ev;PUSH`, `ev;NOMOVE`, `ev;ABORT` follows; thresholds are
        relative to the angle at release, pull-positive."""
        args = ["pull=" + self._num(pull_deg), "push=" + self._num(push_deg), "timeout=" + self._num(timeout_ms)]
        if hold_ms is not None:
            args.append("hold=" + self._num(hold_ms))
        if touch_ncm is not None:
            args.append("touch=" + self._num(touch_ncm))
        if maxdur_ms is not None:
            args.append("maxdur=" + self._num(maxdur_ms))
        return self._send("watch", *args)

    def cfg(self, **values):
        """`cfg;s;key=value;...`: wdt_ms, deadband_deg, stream_hz, and the
        capture window around the WATCH outcome, pre_ms and post_ms (each
        0-2047, together at most 2047; defaults 500 and 500). All or nothing:
        one value out of range and the board applies none of them."""
        args = [key + "=" + self._num(value) for key, value in values.items()]
        return self._send("cfg", *args)

    def cfg_get(self):
        return self._send("cfg", "get")

    def fetch(self, k0=0, n=32):
        """One chunk of the frozen capture window of the last WATCH: the ack
        carries n, k0, t0, dt_us, pre and lost (window samples that are grid
        fills after a missed 1 kHz tick on the board; 0 means every sample
        was taken on time); `cap` lines follow through poll(),
        then `cap;end`. The window is the last pre_ms samples before the
        outcome sample, that sample (k == pre) and post_ms after it; after a
        TRIAL with outcome=NONE there is no outcome sample and the window
        holds pre lines. Page with k0 += n until a chunk has fewer than n
        lines; `nak;busy` while the window is still filling. Fetch before the
        next BRAKE, which restarts the ring."""
        return self._send("fetch", k0, n)

    def stream(self, on, rate_hz=None):
        if not on:
            return self._send("stream", "OFF")
        if rate_hz is None:
            return self._send("stream", "ON")
        return self._send("stream", "ON", rate_hz)

    def angle_output(self, on):
        assert self.has("angout"), "! MouseJoystick: this board has no angle output."
        return self._send("angout", "ON" if on else "OFF")

    def angle_scale(self, deg_lo, deg_hi):
        assert self.has("angout"), "! MouseJoystick: this board has no angle output."
        return self._send("angscale", deg_lo, deg_hi)

    def torque_zero(self):
        """Tare the torque reading at rest (`tq;s;zero`). v1.0 boards only."""
        assert self.has("torque"), "! MouseJoystick: this board has no torque sensor."
        return self._send("tq", "zero")

    def torque_get(self):
        """`tq;s;get` -> ack with torque (tared), raw (untared) in N.cm and
        cal (the shunt-calibration switch, 0 or 1)."""
        assert self.has("torque"), "! MouseJoystick: this board has no torque sensor."
        return self._send("tq", "get")

    def torque_cal(self, on):
        """The torque bridge's shunt-calibration switch. On pcb v1.0 as built
        the switch is inert (bring-up log, 2026-10-06), so this is a verb and
        not a calibration there; the board accepts it either way."""
        assert self.has("torque"), "! MouseJoystick: this board has no torque sensor."
        return self._send("tq", "cal", "ON" if on else "OFF")

    @staticmethod
    def _num(value):
        return MouseJoystickProtocol.format_number(value) if not isinstance(value, str) else value

    def faults(self):
        return self._send("faults")

    def clear_fault(self):
        return self._send("clrfault")

    def instrumentation(self, on):
        return self._send("instr", "ON" if on else "OFF")

    def calibration(self):
        return self._send("cal", "get")

    def set_calibration(self, key, value):
        return self._send("cal", "set", key, value)
