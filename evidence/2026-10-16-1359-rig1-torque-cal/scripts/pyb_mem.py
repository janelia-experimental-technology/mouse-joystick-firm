"""Pyboard heap check through the raw REPL: gc.mem_free()/mem_alloc() before and after gc.collect(),
and the modules loaded. Read-only. Never run while pyControl holds the port. Usage: pyb_mem.py <tty>"""
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
os.write(fd, b"\x03\x03"); time.sleep(0.3); os.write(fd, b"\x01"); read_until(b"raw REPL; CTRL-B to exit", 3.0)
code = b"import gc, sys\nprint('before', gc.mem_free(), gc.mem_alloc())\ngc.collect()\nprint('after ', gc.mem_free(), gc.mem_alloc())\nprint('modules', sorted(sys.modules.keys()))\nprint('globals', sorted(k for k in globals().keys() if not k.startswith('__')))\n"
os.write(fd, code); os.write(fd, b"\x04")
out = read_until(b"\x04\x04", 5.0)
print(out.split(b"OK", 1)[1].replace(b"\x04", b"").decode("ascii", "replace").strip())
os.write(fd, b"\x02"); os.close(fd)
