"""pyControl device class for one mouse-joystick board.

Runs on the pyControl controller in MicroPython. Speaks the protocol in
`../../../mouse-joystick-pcb/docs/uart-protocol.md` to one RP2040 board over one
behavior port. One object per board; two boards means two objects on two ports.

Target controller is **Breakout Board 1.2**, whose UART-capable ports are 1
(`UART4`), 3 (`USART1`) and 4 (`USART3`). The `port.UART` assertion below is what
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
    right = MouseJoystick(port=board.port_3, event_name="joystick_R")

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
    RX_BUFFER = 130

    def __init__(self, port, event_name, baudrate=115200):
        assert port.UART is not None, "! MouseJoystick needs a port with a UART."
        assert event_name not in MouseJoystick.registered_event_names, (
            "! MouseJoystick event_name '%s' is already in use. "
            "Two boards need two distinct names." % event_name
        )
        MouseJoystick.registered_event_names.add(event_name)

        self.event_name = event_name
        self.uart = UART(port.UART)
        self.uart.init(baudrate, bits=8, parity=None, stop=1, timeout=100, rxbuf=self.RX_BUFFER)

        handler = UART_handler(event_name)
        self.uart.irq(trigger=UART.IRQ_RXIDLE, handler=handler.ISR)

        self._rx = ""
        self._seq = 0
        self._pending = {}

        # Filled in by identify(). Left None until then on purpose: a task that
        # reads self.serial before identifying should get None rather than a
        # plausible-looking default it might record as provenance.
        self.serial = None
        self.firmware_version = None
        self.protocol_version = None
        self.axis_count = None

        self.uart.read()  # discard anything buffered from before this object existed

    # ------------------------------------------------------------------ link

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
        data = self.uart.read()
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
        return self.serial

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

    def arm_pull(self, threshold):
        return self._send("armpull", threshold)

    def analog_output(self, on):
        return self._send("angout", "ON" if on else "OFF")

    def analog_scale(self, degrees_low, degrees_high):
        return self._send("angscale", degrees_low, degrees_high)

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
