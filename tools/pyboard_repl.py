#!/usr/bin/env python3
"""Drive the pyControl pyboard's REPL to talk to this board over the RJ-45.

Bench harness. Kept in the repo because the previous scratch version was lost
between sessions and missed -- see the 2026-09-02 handoff.

    pixi run -e pycontrol python tools/pyboard_repl.py <pyboard-tty> [delay] [slow]

Statements are read from stdin, one per line. Four traps, all of which cost a
bench cycle on 2026-09-03:

1. **`UART.read()` blocks for its configured `timeout` on an empty buffer.** A
   `timeout=50` silently turns "20 ms pings" into 72 ms, and against the 100 ms
   host-loss watchdog that produces faults which look like firmware bugs. Set it
   deliberately; `timeout=20` is a reasonable default for polling.
2. **The REPL treats a one-line `for`/`if` block as needing continuation.** Use
   simple statements, or wrap a loop in `exec("def f():\n ...")`.
3. **Do not filter reads on the absence of a heartbeat ack.** Asynchronous events
   arrive in the same read as an ack; `b'ack;9\n' not in r` discarded a real
   `PULL` event. Filter on `b'ev;' in r or ...` instead.
4. **Anything after `arm` must keep pinging** inside the host-loss timeout, so a
   whole armed sequence has to live in ONE statement.

Output is flushed per statement so a timeout cannot swallow the transcript.

    repl_raw.py <port> [default_delay]

A statement whose line ends with `# slow` waits `slow_delay` (30 s) instead of
the default, so one long-running call does not make every other statement slow.
Output is flushed per statement: a timeout must not swallow the transcript.
"""
import sys, time, serial

path = sys.argv[1]
delay = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
slow_delay = float(sys.argv[3]) if len(sys.argv) > 3 else 45.0
lines = [l for l in sys.stdin.read().split("\n") if l.strip()]

with serial.Serial(path, 115200, timeout=0.3) as s:
    time.sleep(0.4)
    s.write(b"\r\x03\x03")
    time.sleep(0.6)
    s.reset_input_buffer()
    for stmt in lines:
        wait = slow_delay if stmt.rstrip().endswith("# slow") else delay
        s.write(stmt.encode() + b"\r\n")
        s.flush()
        deadline = time.time() + wait
        buf = []
        while time.time() < deadline:
            data = s.read(65536)
            if data:
                buf.append(data.decode("utf-8", "replace"))
        print(f"--- sent: {stmt[:64]}", flush=True)
        print("".join(buf), flush=True)
