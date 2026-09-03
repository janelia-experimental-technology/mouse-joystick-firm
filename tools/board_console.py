#!/usr/bin/env python3
"""Minimal mouse-joystick-pcb console harness.

    mjcon.py <port> <read_seconds> [keys ...]

Opens at 115200 -- NEVER 1200, which the earlephilhower core treats as a
BOOTSEL reset touch. Sends each `keys` argument followed by LF, then reads for
read_seconds and prints whatever arrived.
"""
import sys
import time

import serial

port = sys.argv[1]
seconds = float(sys.argv[2])
keys = sys.argv[3:]

with serial.Serial(port, 115200, timeout=0.1) as ser:
    time.sleep(0.2)
    ser.reset_input_buffer()
    for k in keys:
        ser.write((k + "\n").encode())
        ser.flush()
        time.sleep(0.05)
    deadline = time.time() + seconds
    chunks = []
    while time.time() < deadline:
        data = ser.read(4096)
        if data:
            chunks.append(data)
    sys.stdout.write(b"".join(chunks).decode("utf-8", "replace"))
