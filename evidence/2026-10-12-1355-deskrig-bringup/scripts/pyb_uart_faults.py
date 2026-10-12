"""Through the pyboard's raw REPL, send `id` to the board on each Breakout port and print the reply.
Port 1 is UART4, port 4 is USART3 (UART(3)). `id` is a read: no state changes on the board.
Usage: pyb_uart_probe.py <pyboard tty>"""
import os, sys, termios, time, select
fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
a = termios.tcgetattr(fd); a[0] = a[1] = a[3] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
a[4] = a[5] = termios.B115200; termios.tcsetattr(fd, termios.TCSANOW, a)
def read_until(marker, timeout=5.0):
    buf = b""; end = time.time() + timeout
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            try: buf += os.read(fd, 65536)
            except BlockingIOError: pass
            if marker in buf: return buf
    raise SystemExit("timeout waiting for %r; got %r" % (marker, buf[-300:]))
def exec_raw(code, timeout=10.0):
    os.write(fd, code.encode() + b"\x04")
    out = read_until(b"\x04\x04", timeout)
    body = out.split(b"OK", 1)[1]
    stdout, _, rest = body.partition(b"\x04")
    stderr = rest.split(b"\x04", 1)[0]
    if stderr.strip(): raise SystemExit("remote error: %s" % stderr.decode(errors="replace"))
    return stdout.decode(errors="replace")
os.write(fd, b"\x03\x03"); time.sleep(0.3)
os.write(fd, b"\x01"); read_until(b"raw REPL; CTRL-B to exit", 3.0)
for port, uart in (("port 4 (right)", 3),):
    out = exec_raw(
        "from machine import UART\nimport time\n"
        "u = UART(%d, 115200, bits=8, parity=None, stop=1, timeout=50, rxbuf=512)\n"
        "u.read()\n"
        "u.write(b'faults;1\\n'); time.sleep_ms(200); u.write(b'hv;2;state\\n'); time.sleep_ms(200); u.write(b'status;3\\n')\n"
        "time.sleep_ms(300)\n"
        "r = u.read()\n"
        "print(repr(r))\n"
        "u.deinit()\n" % uart, 5.0)
    print("%-15s UART(%d): %s" % (port, uart, out.strip()))
os.write(fd, b"\x02"); os.close(fd)
