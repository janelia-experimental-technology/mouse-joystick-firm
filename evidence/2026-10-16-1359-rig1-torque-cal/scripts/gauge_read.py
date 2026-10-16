"""One reading from the Mark-10 (`?` at 9600 baud), standard library only. Usage: gauge_read.py <tty> [n]"""
import os, select, sys, termios, time
fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
a = termios.tcgetattr(fd); a[0] = a[1] = a[3] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
a[4] = a[5] = termios.B9600; termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIOFLUSH)
for i in range(int(sys.argv[2]) if len(sys.argv) > 2 else 3):
    os.write(fd, b"?\r"); buf = b""; end = time.monotonic() + 0.5
    while time.monotonic() < end and b"\n" not in buf:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            try: buf += os.read(fd, 256)
            except BlockingIOError: pass
    print(time.strftime("%H:%M:%S"), buf.decode("ascii", "replace").strip() or "(no reply)")
    time.sleep(0.2)
os.close(fd)
