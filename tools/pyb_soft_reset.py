"""Soft-reset the pyboard and report its free heap.

Sends Ctrl-D at the friendly REPL, which is what pyControl itself does when it
connects, then prints the banner, the flash listing and `gc.mem_free()`.

Why: pyControl leaves its framework, the hardware definition, the task and the
`devices.*` modules loaded in the REPL namespace when a session ends, about half
of the PYBv1.1's heap. A REPL-driven run on top of that
(`tools/torque_cal_run.py`) ran out of memory four minutes in at Rig 1 on
2026-10-16 (`MemoryError` on the pyboard, then `host_timeout` on the board when
the pings stopped). After this reset the heap is 102 KB free of 104 KB. Run it
before every REPL-driven run.

Standard library only. Never run it while pyControl (the GUI or a runner) holds
the pyboard's port. Nothing on the pyboard's flash is changed.

Usage: python tools/pyb_soft_reset.py /dev/serial/by-id/usb-MicroPython_Pyboard_*-if00

First used as evidence/2026-10-16-1359-rig1-torque-cal/scripts/pyb_soft_reset.py."""
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
