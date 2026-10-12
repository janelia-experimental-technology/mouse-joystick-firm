"""Read the pyboard's MicroPython version over its USB REPL, stdlib only."""
import os, sys, termios, time, select
dev = sys.argv[1]
fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
attrs = termios.tcgetattr(fd)
attrs[0] = 0; attrs[1] = 0; attrs[3] = 0            # raw
attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
attrs[4] = attrs[5] = termios.B115200
termios.tcsetattr(fd, termios.TCSANOW, attrs)
def read_for(seconds):
    out = b""; end = time.time() + seconds
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.1)
        if r:
            try: out += os.read(fd, 4096)
            except BlockingIOError: pass
    return out
os.write(fd, b"\x03\x03")                             # interrupt anything running
read_for(0.5)
os.write(fd, b"\r\nimport sys, os; print('VER', sys.implementation.version, sys.implementation._machine if hasattr(sys.implementation, '_machine') else os.uname().machine); print('FILES', sorted(os.listdir('/flash')))\r\n")
out = read_for(1.5).decode("utf-8", "replace")
for line in out.splitlines():
    if line.startswith(("VER", "FILES")) or "Error" in line:
        print(line)
os.close(fd)
