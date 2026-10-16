#!/usr/bin/env python3
"""Torque calibration run on one desk-rig board, a Mark-10 gauge logged alongside.

Drives the board on Breakout port 1 (UART(4), the left mechanism) or port 4
(UART(3), the right) through the pyboard's raw REPL, standard library only:
clrfault, id, hv on, home, arm, present at PRESENT deg, tq get, stream on at
50 Hz, then for each brake level L: brake L -> ev;RELEASED, tq get, hold while
the gauge is pulled, leave on hands-off (torque back near its RELEASED value
and the angle still for QUIET s, after HOLD_MIN s; HOLD_MAX s at most), then
moveto PRESENT to leave RELEASED. Finally stream off, park at 0, disarm,
hv off. Pings every 40 ms throughout (the board's host watchdog is 100 ms).
No `tq zero` is sent, so the streamed torque is the raw reading about the
nominal zero.

Meanwhile a host thread polls the Mark-10 (`?` at 9600 baud, gF) every 100 ms.

Log lines, one per event, host monotonic ns first:
  B <host_ns> <pyb_ms> > <command>      sent to the board (pings omitted)
  B <host_ns> <pyb_ms> < <line>         received from the board (ping acks omitted)
  B <host_ns> <pyb_ms> H <note>         the pyboard program's own notes
  G <host_ns> <reading>                 the gauge, as it printed it
  H <host_ns> <note>                    host notes

Usage: torque_cal_run.py --board right --radius-mm 75 --out DIR [--levels 3.5,0,1,2,0.5]
Never run while pyControl's runner holds the pyboard port.
"""
import argparse, json, os, select, sys, termios, threading, time

PYB = "/dev/serial/by-id/usb-MicroPython_Pyboard_Virtual_Comm_Port_in_FS_Mode_3968348E3233-if00"
GAUGE = "/dev/serial/by-id/usb-Silicon_Labs_MARK-10_USB_Device_3706367-if00-port0"
UARTS = {"left": 4, "right": 3}  # Breakout port 1 = UART(4); port 4 = UART(3)

PROGRAM = r'''
from machine import UART
import time
U = UART(%(uart)d, 115200, bits=8, parity=None, stop=1, timeout=0, rxbuf=2048)
U.read()
SEQ = [1]
RX = [b""]
PINGS = set()
FAULT = [None]
LASTP = [time.ticks_ms()]
def out(tag, s):
    print("%%d %%s %%s" %% (time.ticks_ms(), tag, s))
def send(verb, args=(), quiet=False):
    SEQ[0] = (SEQ[0] + 1) %% 65536
    if SEQ[0] == 0:
        SEQ[0] = 1
    n = SEQ[0]
    parts = [verb, str(n)] + [str(a) for a in args]
    line = ";".join(parts)
    U.write((line + "\n").encode())
    if quiet:
        PINGS.add(n)
    else:
        out(">", line)
    return n
def pump():
    d = U.read()
    lines = []
    if d:
        RX[0] += d
        while b"\n" in RX[0]:
            raw, RX[0] = RX[0].split(b"\n", 1)
            try:
                s = str(raw.strip(), "utf-8")
            except Exception:
                s = repr(raw)
            if not s:
                continue
            f = s.split(";")
            if f[0] == "ack" and len(f) == 2 and f[1].isdigit() and int(f[1]) in PINGS:
                PINGS.discard(int(f[1]))
                continue
            out("<", s)
            if f[0] == "ev" and len(f) > 1 and f[1] == "FAULT":
                FAULT[0] = s
            lines.append(s)
    if len(PINGS) > 50:
        PINGS.clear()
    return lines
def tick():
    now = time.ticks_ms()
    if time.ticks_diff(now, LASTP[0]) >= 40:
        send("ping", quiet=True)
        LASTP[0] = now
    return pump()
def wait_for(pred, timeout_ms):
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        for s in tick():
            if pred(s):
                return s
        if FAULT[0]:
            return None
        time.sleep_ms(2)
    return None
def pause(ms):
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < ms:
        tick()
        time.sleep_ms(2)
def is_reply(n):
    a = "ack;%%d" %% n
    k = "nak;%%d" %% n
    return lambda s: s == a or s.startswith(a + ";") or s.startswith(k + ";")
def is_ev(name):
    p = "ev;%%s;" %% name
    return lambda s: s.startswith(p)
def cmd(verb, args=(), timeout_ms=2000):
    n = send(verb, args)
    r = wait_for(is_reply(n), timeout_ms)
    if r is None or r.startswith("nak"):
        raise Exception("%%s -> %%r (fault=%%r)" %% (verb, r, FAULT[0]))
    return r
def ev(name, timeout_ms):
    r = wait_for(is_ev(name), timeout_ms)
    if r is None:
        raise Exception("no ev;%%s within %%d ms (fault=%%r)" %% (name, timeout_ms, FAULT[0]))
    return r
def kv(s, key):
    for part in s.split(";"):
        k, _, v = part.partition("=")
        if k == key:
            return float(v)
    return None
def hold(level, min_ms, max_ms, quiet_ms, tq_tol, ang_tol, rest_tq):
    out("H", "hold %%s start rest_tq=%%s" %% (level, rest_tq))
    t0 = time.ticks_ms()
    hist = []
    while True:
        for s in tick():
            if s.startswith("ev;S;"):
                a = kv(s, "angle")
                t = kv(s, "torque")
                if a is not None and t is not None:
                    hist.append((time.ticks_ms(), a, t))
                    while hist and time.ticks_diff(hist[-1][0], hist[0][0]) > quiet_ms:
                        hist.pop(0)
        if FAULT[0]:
            raise Exception("fault during hold: %%s" %% FAULT[0])
        el = time.ticks_diff(time.ticks_ms(), t0)
        if el >= max_ms:
            out("H", "hold %%s end max" %% level)
            return
        if el >= min_ms and len(hist) > 10 and time.ticks_diff(hist[-1][0], hist[0][0]) >= quiet_ms - 200:
            tq = [h[2] for h in hist]
            an = [h[1] for h in hist]
            if max(tq) - min(tq) < tq_tol and max(an) - min(an) < ang_tol and abs(tq[-1] - rest_tq) < tq_tol:
                out("H", "hold %%s end quiet" %% level)
                return
        time.sleep_ms(2)
LEVELS = %(levels)s
PRESENT = %(present)s
try:
    cmd("clrfault")
    out("H", cmd("id"))
    cmd("hv", ("on",))
    pause(300)
    cmd("home")
    ev("HOMED", 30000)
    cmd("arm")
    cmd("moveto", (PRESENT,))
    ev("TARGET", 8000)
    out("H", cmd("tq", ("get",)))
    cmd("stream", ("on", 50))
    for L in LEVELS:
        cmd("brake", (L,))
        rel = ev("RELEASED", 4000)
        out("H", cmd("tq", ("get",)))
        rest = kv(rel, "torque")
        hold(L, %(hold_min_ms)d, %(hold_max_ms)d, %(quiet_ms)d, %(tq_tol)s, %(ang_tol)s, rest if rest is not None else 0.0)
        cmd("moveto", (PRESENT,), 4000)
        ev("TARGET", 8000)
        pause(500)
    cmd("stream", ("off",))
    cmd("moveto", (0.0,))
    ev("TARGET", 8000)
    cmd("disarm")
    cmd("hv", ("off",))
    out("H", "sequence complete")
except Exception as e:
    out("H", "ABORT %%r" %% e)
    try:
        send("stream", ("off",))
        time.sleep_ms(50)
        send("stop")
        time.sleep_ms(300)
        pump()
        send("disarm")
        time.sleep_ms(200)
        pump()
        send("hv", ("off",))
        time.sleep_ms(300)
        pump()
    except Exception as e2:
        out("H", "shutdown error %%r" %% e2)
finally:
    out("H", "program end")
    U.deinit()
'''

DRY_PROGRAM = r"""
import time
t0 = time.ticks_ms()
for i in range(30):
    print("%d H dry %d" % (time.ticks_ms(), i))
    time.sleep_ms(100)
print("%d H program end" % time.ticks_ms())
"""


def open_tty(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = a[1] = a[3] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[4] = a[5] = baud
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


class Log:
    def __init__(self, path):
        self.f = open(path, "w", buffering=1)
        self.lock = threading.Lock()

    def write(self, tag, text):
        line = "%s %d %s" % (tag, time.monotonic_ns(), text)
        with self.lock:
            self.f.write(line + "\n")
        return line

    def close(self):
        self.f.close()


def gauge_thread(path, log, stop, period_s):
    try:
        fd = open_tty(path, termios.B9600)
    except OSError as e:
        log.write("H", "gauge open failed: %r" % e)
        return
    buf = b""
    n = 0
    while not stop.is_set():
        t_req = time.monotonic()
        try:
            os.write(fd, b"?\r")
        except OSError as e:
            log.write("H", "gauge write failed: %r" % e)
            break
        deadline = t_req + period_s
        while time.monotonic() < deadline:
            r, _, _ = select.select([fd], [], [], max(0.0, deadline - time.monotonic()))
            if r:
                try:
                    buf += os.read(fd, 4096)
                except BlockingIOError:
                    pass
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    s = raw.strip().decode("ascii", "replace")
                    if s:
                        log.write("G", s)
                        n += 1
    os.close(fd)
    log.write("H", "gauge thread end, %d readings" % n)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--board", choices=("left", "right"), required=True)
    ap.add_argument("--radius-mm", type=float, required=True, help="lever axis to the gauge's contact point, recorded in meta")
    ap.add_argument("--out", required=True, help="directory for the log and meta")
    ap.add_argument("--name", default="torque_cal", help="log file stem")
    ap.add_argument("--levels", default="3.5,0,1.0,2.0,0.5", help="brake levels in N.cm, in order")
    ap.add_argument("--present", type=float, default=-5.0)
    ap.add_argument("--hold-min", type=float, default=40.0, help="seconds each level is held at least")
    ap.add_argument("--hold-max", type=float, default=150.0)
    ap.add_argument("--quiet", type=float, default=5.0, help="seconds of hands-off before the lever moves back")
    ap.add_argument("--tq-tol", type=float, default=0.15, help="N.cm: quiet means torque within this of the RELEASED reading")
    ap.add_argument("--ang-tol", type=float, default=0.2, help="deg: quiet means the angle moved less than this")
    ap.add_argument("--gauge-period", type=float, default=0.1)
    ap.add_argument("--pyb", default=PYB)
    ap.add_argument("--gauge", default=GAUGE)
    ap.add_argument("--note", default="")
    ap.add_argument("--dry-run", action="store_true", help="run a 3 s print loop on the pyboard instead; no UART, no board traffic")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    log_path = os.path.join(args.out, args.name + ".log")
    log = Log(log_path)
    levels = [float(x) for x in args.levels.split(",") if x.strip()]
    program = PROGRAM % dict(
        uart=UARTS[args.board], levels=repr(levels), present=repr(args.present),
        hold_min_ms=int(args.hold_min * 1000), hold_max_ms=int(args.hold_max * 1000),
        quiet_ms=int(args.quiet * 1000), tq_tol=repr(args.tq_tol), ang_tol=repr(args.ang_tol))
    if args.dry_run:
        program = DRY_PROGRAM
    meta = dict(board=args.board, uart=UARTS[args.board], radius_mm=args.radius_mm, levels=levels,
                present_deg=args.present, hold_min_s=args.hold_min, hold_max_s=args.hold_max,
                quiet_s=args.quiet, tq_tol_ncm=args.tq_tol, ang_tol_deg=args.ang_tol,
                gauge="Mark-10 M3-05, serial 3706367, `?` at 9600 baud, gF", gauge_period_s=args.gauge_period,
                started=time.strftime("%Y-%m-%d %H:%M:%S"), note=args.note, log=os.path.basename(log_path))
    with open(os.path.join(args.out, args.name + ".meta.json"), "w") as f:
        json.dump(meta, f, indent=1)
    print("logging to", log_path)
    print(log.write("H", "start board=%s radius_mm=%s levels=%s" % (args.board, args.radius_mm, levels)))

    stop = threading.Event()
    gt = threading.Thread(target=gauge_thread, args=(args.gauge, log, stop, args.gauge_period), daemon=True)
    gt.start()

    fd = open_tty(args.pyb, termios.B115200)

    def read_until(marker, timeout, echo=False):
        buf = b""
        end = time.time() + timeout
        while time.time() < end:
            r, _, _ = select.select([fd], [], [], 0.05)
            if r:
                try:
                    buf += os.read(fd, 65536)
                except BlockingIOError:
                    pass
                if marker in buf:
                    return buf
        raise SystemExit("timeout waiting for %r; got %r" % (marker, buf[-300:]))

    rc = 0
    try:
        os.write(fd, b"\x03\x03")
        time.sleep(0.3)
        os.write(fd, b"\x01")
        read_until(b"raw REPL; CTRL-B to exit", 3.0)
        data = program.encode()
        for i in range(0, len(data), 256):
            os.write(fd, data[i:i + 256])
            time.sleep(0.01)
        os.write(fd, b"\x04")
        head = read_until(b"OK", 5.0)
        rest = head.split(b"OK", 1)[1]
        # Stream stdout line by line until the first \x04 (end of stdout).
        buf = rest
        done = False
        timeout_s = args.hold_max * len(levels) + 120
        end = time.time() + timeout_s
        while not done and time.time() < end:
            r, _, _ = select.select([fd], [], [], 0.05)
            if r:
                try:
                    buf += os.read(fd, 65536)
                except BlockingIOError:
                    pass
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                if b"\x04" in raw:
                    raw, _, tail = raw.partition(b"\x04")
                    buf = tail + b"\n" + buf
                    done = True
                s = raw.decode("ascii", "replace").rstrip("\r")
                if s:
                    line = log.write("B", s)
                    print(line, flush=True)
                if done:
                    break
            if b"\x04" in buf and not done:
                pre, _, tail = buf.partition(b"\x04")
                if pre.strip():
                    line = log.write("B", pre.decode("ascii", "replace").rstrip("\r"))
                    print(line, flush=True)
                buf = tail
                done = True
        if not done:
            log.write("H", "timed out waiting for the program to end")
            rc = 2
        else:
            tail = read_until(b"\x04", 5.0) if b"\x04" not in buf else buf
            stderr = tail.split(b"\x04", 1)[0].decode("ascii", "replace").strip()
            if stderr:
                print(log.write("H", "remote stderr: %s" % stderr))
                rc = 3
    finally:
        try:
            os.write(fd, b"\x02")
        except OSError:
            pass
        os.close(fd)
        stop.set()
        gt.join(timeout=3.0)
        print(log.write("H", "end rc=%d" % rc))
        log.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
