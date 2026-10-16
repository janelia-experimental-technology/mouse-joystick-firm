"""Soft-reset the pyboard (Ctrl-D at the friendly REPL, what pyControl does on connect) to drop the
pyControl framework, hardware definition and task that an earlier session left in the REPL namespace,
then report the banner, the flash listing and the free heap. Never run while pyControl holds the port.
Usage: pyb_soft_reset.py <tty>"""
import os, select, sys, termios, time
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
os.write(fd, b"\x03\x03"); time.sleep(0.3); os.write(fd, b"\x02"); time.sleep(0.3)
os.write(fd, b"\x04")
banner = read_until(b">>> ", 6.0)
print("after soft reset:", banner.decode("ascii", "replace").strip().splitlines()[-3:])
time.sleep(0.3)
os.write(fd, b"\x01"); read_until(b"raw REPL; CTRL-B to exit", 3.0)
code = b"import gc, os, sys\ngc.collect()\nprint('mem_free', gc.mem_free(), 'mem_alloc', gc.mem_alloc())\nprint('flash', sorted(os.listdir('/flash')))\nprint('modules', sorted(sys.modules.keys()))\n"
os.write(fd, code); os.write(fd, b"\x04")
out = read_until(b"\x04\x04", 5.0)
print(out.split(b"OK", 1)[1].replace(b"\x04", b"").decode("ascii", "replace").strip())
os.write(fd, b"\x02"); os.close(fd)
