"""Show the current cue file in a terminal, big and alone, refreshing when it changes."""
import os, sys, time
path = sys.argv[1]
last = None
while True:
    try:
        s = open(path).read()
    except OSError:
        s = ""
    if s != last:
        sys.stdout.write("\x1b[2J\x1b[H\n\n" + s + "\n")
        sys.stdout.flush()
        last = s
    time.sleep(0.2)
