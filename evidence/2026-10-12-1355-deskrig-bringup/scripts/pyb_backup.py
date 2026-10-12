"""Copy every file under /flash off a pyboard through the raw REPL, stdlib only.
Usage: pyb_backup.py <tty> <out dir>"""
import os, sys, termios, time, select, base64, pathlib, json
dev, out = sys.argv[1], pathlib.Path(sys.argv[2])
fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
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
    raise SystemExit("timeout waiting for %r; got %r" % (marker, buf[-200:]))
def exec_raw(code, timeout=10.0):
    os.write(fd, code.encode() + b"\x04")
    out = read_until(b"\x04\x04", timeout)
    body = out.split(b"OK", 1)[1]            # <stdout> \x04 <stderr> \x04 >
    stdout, _, rest = body.partition(b"\x04")
    stderr = rest.split(b"\x04", 1)[0]
    if stderr.strip(): raise SystemExit("remote error: %s" % stderr.decode(errors="replace"))
    return stdout
os.write(fd, b"\x03\x03"); time.sleep(0.3)
os.write(fd, b"\x01"); read_until(b"raw REPL; CTRL-B to exit", 3.0)
listing = exec_raw("import os\n"
    "def walk(p):\n"
    "  for n in os.listdir(p):\n"
    "    if n == 'System Volume Information': continue\n"
    "    q = p + '/' + n\n"
    "    if os.stat(q)[0] & 0x4000: walk(q)\n"
    "    else: print(q, os.stat(q)[6])\n"
    "walk('/flash')\n")
files = [l.rsplit(None, 1) for l in listing.decode().splitlines() if l.strip()]
manifest = []
for path, size in files:
    if "System Volume Information" in path: continue
    b64 = exec_raw("import ubinascii\nf=open(%r,'rb')\nwhile True:\n  c=f.read(768)\n  if not c: break\n  print(ubinascii.b2a_base64(c).decode().strip())\nf.close()\n" % path, 30.0)
    data = b"".join(base64.b64decode(line) for line in b64.decode().split())
    rel = path[len("/flash/"):]
    dest = out / rel; dest.parent.mkdir(parents=True, exist_ok=True); dest.write_bytes(data)
    manifest.append({"path": path, "size": int(size), "copied": len(data)})
    print("%-50s %6s bytes%s" % (path, size, "" if len(data) == int(size) else "  SIZE MISMATCH %d" % len(data)))
os.write(fd, b"\x02")  # back to the friendly REPL
(out / "manifest.json").write_text(json.dumps(manifest, indent=1))
os.close(fd)
print("files:", len(manifest), "total bytes:", sum(m["copied"] for m in manifest))
