"""MicroPython stubs, so the device class can be tested under CPython.

`devices/mouse_joystick.py` imports `machine.UART` and
`devices.uart_handler.UART_handler`, neither of which exists off the pyboard.
These stubs supply them and wire the UART to a `BoardSim` instead of a wire, so a
test exercises the real device class against the real codec against the real
simulator - three of the four layers, with only the physical link faked.

What this does NOT test, and no host test can: interrupt timing, `UART.IRQ_RXIDLE`
actually firing, rxbuf overrun behaviour, and anything about MicroPython's own
UART driver. Those need the Breakout 1.2 and a board.
"""

import sys
import types


class SimLink:
    """A `machine.UART` work-alike backed by a BoardSim.

    Deliberately byte-oriented rather than line-oriented: `read()` can return a
    partial line, exactly as a real UART does, which is the behaviour the device
    class's buffering has to survive.
    """

    IRQ_RXIDLE = 0x10

    def __init__(self, sim, chunk_size=None):
        self.sim = sim
        self.chunk_size = chunk_size  # None = deliver everything pending
        self.handler = None
        self.init_kwargs = None
        self.written = []
        self._pending = ""

    # -- machine.UART surface -------------------------------------------------

    def init(self, baudrate, **kwargs):
        self.init_kwargs = dict(kwargs)
        self.init_kwargs["baudrate"] = baudrate

    def irq(self, trigger=None, handler=None):
        self.handler = handler

    def write(self, text):
        self.written.append(text)
        self._absorb(self.sim.feed(text))
        return len(text)

    def read(self):
        if not self._pending:
            return None
        if self.chunk_size is None:
            data, self._pending = self._pending, ""
        else:
            data = self._pending[: self.chunk_size]
            self._pending = self._pending[self.chunk_size :]
        return data.encode("utf-8")

    def any(self):
        return len(self._pending)

    # -- test control ---------------------------------------------------------

    def advance(self, us):
        """Advance the simulated board's clock and queue anything it emits."""
        self._absorb(self.sim.advance(us))

    def _absorb(self, lines):
        for line in lines:
            self._pending += line + "\n"


def install(link):
    """Install the stub modules and return a teardown callable.

    `mouse_joystick` is purged from `sys.modules` on both install and teardown so
    each test binds to its own link rather than to whichever one imported first.
    """
    saved = {
        name: sys.modules.get(name)
        for name in (
            "machine",
            "devices",
            "devices.uart_handler",
            "devices.mouse_joystick_protocol",
            "mouse_joystick",
        )
    }

    machine = types.ModuleType("machine")

    class UART:
        IRQ_RXIDLE = SimLink.IRQ_RXIDLE

        def __new__(cls, port_uart, *args, **kwargs):
            return link

    machine.UART = UART
    sys.modules["machine"] = machine

    devices = types.ModuleType("devices")
    devices.__path__ = []

    # The codec is registered under its real package path rather than stubbed,
    # so the device class imports the same module the simulator and the corpus
    # tests use. Stubbing it would let the two drift and the tests would not
    # notice. This also mirrors the board, where `devices.mouse_joystick_protocol`
    # is exactly how the file is reached.
    import mouse_joystick_protocol

    devices.mouse_joystick_protocol = mouse_joystick_protocol
    sys.modules["devices.mouse_joystick_protocol"] = mouse_joystick_protocol

    uart_handler = types.ModuleType("devices.uart_handler")

    class UART_handler:
        """Stands in for pyControl's real handler, which raises a framework event."""

        def __init__(self, event_name):
            self.event_name = event_name

        def ISR(self, _):
            pass

    uart_handler.UART_handler = UART_handler
    devices.uart_handler = uart_handler
    sys.modules["devices"] = devices
    sys.modules["devices.uart_handler"] = uart_handler
    sys.modules.pop("mouse_joystick", None)

    def teardown():
        sys.modules.pop("mouse_joystick", None)
        for name, module in saved.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    return teardown


class FakePort:
    """A pyControl behavior port. `UART=None` is a port that has no UART."""

    def __init__(self, uart=4):
        self.UART = uart
