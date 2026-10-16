#!/usr/bin/env python3
"""Torque calibration run on one desk-rig board, choreographed with spoken and on-screen cues,
the Mark-10 gauge logged alongside.

Drives the board on Breakout port 1 (UART(4), the left mechanism) or port 4 (UART(3), the
right) through the pyboard's raw REPL, standard library only: clrfault, id, hv on, home, arm,
present at PRESENT deg, tq get, stream on at 50 Hz, then for each brake level L: brake L ->
ev;RELEASED, tq get, hold while the host runs that level's choreography of cues (spoken
through spd-say, shown in the cue window, logged), then wait for hands-off (torque back
near the RELEASED value and the angle still for QUIET s) and moveto PRESENT to leave
RELEASED. Finally stream off, park at 0, disarm, hv off. Pings every 40 ms throughout (the
board's host watchdog is 100 ms). No `tq zero` is sent, so the streamed torque is the raw
reading about the nominal zero.

Log lines, host monotonic ns first:
  B <host_ns> <pyb_ms> > <command>      sent to the board (pings omitted)
  B <host_ns> <pyb_ms> < <line>         received from the board (ping acks omitted)
  B <host_ns> <pyb_ms> H <note>         the pyboard program's notes (hold L start/end ...)
  G <host_ns> <reading>                 the gauge, as it printed it (gF)
  C <host_ns> <level> <phase> <seconds> <text>   a cue, as it was given
  H <host_ns> <note>                    host notes

Usage: torque_cal_run2.py --board right --radius-mm 72 --out DIR [--levels 3.5,0,1.0,2.0,0.5]
Never run while pyControl's runner holds the pyboard port.
"""
import argparse, json, os, select, subprocess, sys, termios, threading, time

PYB = "/dev/serial/by-id/usb-MicroPython_Pyboard_Virtual_Comm_Port_in_FS_Mode_3968348E3233-if00"
GAUGE = "/dev/serial/by-id/usb-Silicon_Labs_MARK-10_USB_Device_3706367-if00-port0"
UARTS = {"left": 4, "right": 3}  # Breakout port 1 = UART(4); port 4 = UART(3)
SPD_SAY = "/usr/bin/spd-say"

# Choreography: (phase, seconds, spoken and displayed text). The lever is at PRESENT and the
# brake is on when the first cue is given. "slide" phases are where the friction is kinetic
# and the sensor-to-gauge comparison is cleanest; "ramp" and "steady" phases are static.
def choreography_pinion(level):
    if level >= 3.4:
        return [
            ("remove_pinion", 300, "Brake at full, drive released, the brake holds the lever. Remove the pinion now. Take your time: the pull cycles start when this countdown ends."),
            ("slip_cycles", 90, "With the string: pull slowly until it slips, slide a little, ease off. Repeat until the countdown ends."),
        ]
    if level == 0:
        return [
            ("free_slide", 75, "Brake off. The lever is now completely free, so hold it. Slide it slowly each way, over and over, until the countdown ends."),
        ]
    return [
        ("slip_cycles", 75, "Brake at %s. Pull slowly until it slips, slide a little, ease off. Repeat until the countdown ends." % level),
    ]


MODES = {"normal": None, "pinion": choreography_pinion}


def choreography(level):
    if level == 0:
        return [
            ("slide_pull", 7, "Brake off. Pull the free lever slowly, a few degrees."),
            ("slide_push", 7, "Push it slowly back."),
            ("slide_pull", 7, "Again. Pull slowly, a few degrees."),
            ("slide_push", 7, "Push slowly back."),
        ]
    if level >= 3.4:
        return [
            ("ramp_up", 10, "Brake at full. Pull slowly, up to forty grams over ten seconds. Do not let it slip."),
            ("steady", 3, "Hold it steady."),
            ("ramp_down", 10, "Ease off slowly, over ten seconds."),
            ("rest", 3, "Hands off."),
            ("slide_pull", 10, "Now pull harder until it slips, and slide slowly, a few degrees."),
            ("rest", 4, "Let go."),
            ("slide_push", 10, "Now push the other way until it slips, and slide slowly back."),
        ]
    if level >= 1.9:
        return [
            ("ramp_up", 10, "Brake at two. Pull slowly, up to twenty grams over ten seconds. Do not let it slip."),
            ("ramp_down", 6, "Ease off slowly."),
            ("rest", 3, "Hands off."),
            ("slide_pull", 10, "Now pull until it slips, and slide slowly, a few degrees."),
            ("rest", 4, "Let go."),
            ("slide_push", 10, "Now push the other way until it slips, and slide slowly back."),
        ]
    return [
        ("slide_pull", 10, "Brake at %s. Pull until it slips, and slide slowly, a few degrees." % level),
        ("rest", 4, "Let go."),
        ("slide_push", 10, "Now push the other way until it slips, and slide slowly back."),
    ]


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
PENDING = []
def wait_for(pred, timeout_ms):
    # Lines that arrived in the same batch as an earlier match are kept, not
    # dropped: brake 0 answers ack and RELEASED in one read (run3, 2026-10-16).
    for i in range(len(PENDING)):
        if pred(PENDING[i]):
            return PENDING.pop(i)
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        found = None
        for s in tick():
            if found is None and pred(s):
                found = s
            elif not s.startswith("ev;S;"):
                PENDING.append(s)
        if len(PENDING) > 100:
            del PENDING[:len(PENDING) - 100]
        if found is not None:
            return found
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
        if %(fixed_hold)s and el >= min_ms:
            out("H", "hold %%s end fixed" %% level)
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
    for L, min_ms in LEVELS:
        cmd("brake", (L,))
        rel = ev("RELEASED", 4000)
        out("H", cmd("tq", ("get",)))
        rest = kv(rel, "torque")
        hold(L, min_ms, %(hold_max_ms)d, %(quiet_ms)d, %(tq_tol)s, %(ang_tol)s, rest if rest is not None else 0.0)
        cmd("moveto", (PRESENT,), 4000)
        ev("TARGET", 8000)
        pause(500)
    cmd("stream", ("off",))
    if %(park)s:
        cmd("moveto", (0.0,))
        ev("TARGET", 8000)
    else:
        cmd("stop", (), 4000)
        pause(500)
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


class Cues:
    """Speak, show and log cues. Speaking is asynchronous and never blocks the run."""

    def __init__(self, log, cue_file, speak, choreo=None):
        self.log, self.cue_file, self.speak = log, cue_file, speak
        self.choreo = choreo or choreography
        self.abort = threading.Event()
        self.thread = None

    def say(self, level, phase, seconds, text):
        self.log.write("C", "%s %s %s %s" % (level, phase, seconds, text))
        try:
            with open(self.cue_file, "w") as f:
                f.write("  level %s   [%s]\n\n  %s\n" % (level, phase, text))
        except OSError:
            pass
        if self.speak and os.path.exists(SPD_SAY):
            try:
                subprocess.Popen([SPD_SAY, "-C", "-r", "-15", text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except OSError:
                pass
        print("CUE level %s %s: %s" % (level, phase, text), flush=True)

    def show(self, level, phase, text, remaining):
        try:
            with open(self.cue_file, "w") as f:
                f.write("  level %s   [%s]   %3d s left\n\n  %s\n" % (level, phase, remaining, text))
        except OSError:
            pass

    def run_level(self, level):
        def body():
            for phase, seconds, text in self.choreo(level):
                if self.abort.is_set():
                    return
                self.say(level, phase, seconds, text)
                end = time.monotonic() + seconds
                while time.monotonic() < end and not self.abort.is_set():
                    self.show(level, phase, text, int(round(end - time.monotonic())))
                    time.sleep(0.25)
            if not self.abort.is_set():
                self.say(level, "handsoff", 0, "Hands off. Wait for the lever to move back.")
        self.thread = threading.Thread(target=body, daemon=True)
        self.thread.start()

    def stop(self):
        self.abort.set()


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
    ap.add_argument("--radius-mm", type=float, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="torque_cal")
    ap.add_argument("--levels", default="3.5,0,1.0,2.0,0.5")
    ap.add_argument("--present", type=float, default=-5.0)
    ap.add_argument("--hold-max", type=float, default=150.0)
    ap.add_argument("--quiet", type=float, default=5.0)
    ap.add_argument("--tq-tol", type=float, default=0.15)
    ap.add_argument("--ang-tol", type=float, default=0.2)
    ap.add_argument("--gauge-period", type=float, default=0.1)
    ap.add_argument("--cue-file", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cue.txt"))
    ap.add_argument("--no-speech", action="store_true")
    ap.add_argument("--countdown", type=int, default=0, help="seconds of countdown shown in the cue window before the lever moves")
    ap.add_argument("--mode", choices=sorted(MODES), default="normal", help="pinion: long fixed holds, the pinion removed during the first; no quiet detection")
    ap.add_argument("--no-park", action="store_true", help="end with stop, disarm, hv off and no move to 0 (the pinion is off)")
    ap.add_argument("--pyb", default=PYB)
    ap.add_argument("--gauge", default=GAUGE)
    ap.add_argument("--note", default="")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    log_path = os.path.join(args.out, args.name + ".log")
    log = Log(log_path)
    levels = [float(x) for x in args.levels.split(",") if x.strip()]
    choreo = MODES[args.mode] or choreography
    level_specs = [(L, int((sum(sec for _, sec, _ in choreo(L)) + 2) * 1000)) for L in levels]
    fixed_hold = args.mode == "pinion"
    hold_max_ms = max(int(args.hold_max * 1000), max(ms for _, ms in level_specs) + 60000)
    program = PROGRAM % dict(
        uart=UARTS[args.board], levels=repr(level_specs), present=repr(args.present),
        hold_max_ms=hold_max_ms, quiet_ms=int(args.quiet * 1000),
        tq_tol=repr(args.tq_tol), ang_tol=repr(args.ang_tol),
        fixed_hold=repr(fixed_hold), park=repr(not args.no_park))
    meta = dict(board=args.board, uart=UARTS[args.board], radius_mm=args.radius_mm, levels=levels,
                hold_min_s={str(L): ms / 1000.0 for L, ms in level_specs},
                choreography={str(L): choreo(L) for L in levels}, mode=args.mode, park=not args.no_park,
                present_deg=args.present, hold_max_s=args.hold_max, quiet_s=args.quiet,
                tq_tol_ncm=args.tq_tol, ang_tol_deg=args.ang_tol,
                gauge="Mark-10 M3-05, serial 3706367, `?` at 9600 baud, gF; a pull on the hook reads negative",
                gauge_period_s=args.gauge_period, started=time.strftime("%Y-%m-%d %H:%M:%S"),
                note=args.note, log=os.path.basename(log_path))
    with open(os.path.join(args.out, args.name + ".meta.json"), "w") as f:
        json.dump(meta, f, indent=1)
    print("logging to", log_path)
    print(log.write("H", "start board=%s radius_mm=%s levels=%s" % (args.board, args.radius_mm, levels)))

    cues = Cues(log, os.path.abspath(args.cue_file), not args.no_speech, choreo)
    stop = threading.Event()
    gt = threading.Thread(target=gauge_thread, args=(args.gauge, log, stop, args.gauge_period), daemon=True)
    gt.start()

    fd = open_tty(args.pyb, termios.B115200)

    def read_until(marker, timeout):
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

    def on_board_line(s):
        f = s.split(" ", 2)
        if len(f) < 3 or f[1] != "H":
            return
        note = f[2]
        if note.startswith("hold ") and " start" in note:
            cues.run_level(float(note.split()[1]))
        elif note.startswith("hold ") and " end" in note:
            cues.say(float(note.split()[1]), "moving", 0, "Hands off. The lever is moving back.")
        elif note.startswith("sequence complete"):
            cues.say("-", "done", 0, "Done. The rail is off.")
        elif note.startswith("ABORT"):
            cues.stop()
            cues.say("-", "abort", 0, "Stopped. Hands off.")

    rc = 0
    try:
        for n in range(args.countdown, 0, -1):
            cues.say("-", "countdown", 1, "Starting in %d seconds. Hands off, the lever will move." % n)
            time.sleep(1.0)
        cues.say("-", "start", 0, "Starting. Hands off, the lever is about to move.")
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
        buf = head.split(b"OK", 1)[1]
        done = False
        end = time.time() + hold_max_ms / 1000.0 * len(levels) + 120
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
                    log.write("B", s)
                    on_board_line(s)
                if done:
                    break
            if b"\x04" in buf and not done:
                pre, _, tail = buf.partition(b"\x04")
                if pre.strip():
                    s = pre.decode("ascii", "replace").rstrip("\r")
                    log.write("B", s)
                    on_board_line(s)
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
        cues.stop()
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
