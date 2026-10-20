#!/usr/bin/env python3
"""Compile BoardPins against a PCB-owned GPIO export, without Arduino or hardware.

The binding file contains names, never a second set of GPIO numbers. Pass the
PCB export explicitly; this development check does not add a sibling checkout
dependency to firmware builds.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def assertions(facts, bindings):
    if facts.get("schema") != 1 or bindings.get("schema") != 1:
        raise ValueError("unsupported facts/bindings schema")
    for field in ("board", "board_revision", "firmware_board_rev"):
        if facts.get(field) != bindings.get(field):
            raise ValueError(f"{field}: hardware export does not match firmware binding")
    expected = bindings["signals"]
    if set(expected.values()) != set(facts["signals"]):
        raise ValueError("binding must cover exactly the exported hardware signals")
    lines = []
    for constant, signal in expected.items():
        if not re.fullmatch(r"k[A-Za-z0-9]+", constant):
            raise ValueError("invalid C++ pin constant")
        value = facts["signals"][signal]["gpio"]
        if type(value) is not int or not 0 <= value <= 29:
            raise ValueError(f"{signal}: invalid GPIO")
        lines.append(f'static_assert(mouse_joystick::pins::{constant} == {value}, "{constant} differs from PCB");')
    for signal in bindings["active_low"]:
        if facts["signals"][signal]["active_low"] is not True:
            raise ValueError(f"{signal}: active-low contract mismatch")
    spares = facts["spare_gpios"]
    if any(type(v) is not int or not 0 <= v <= 29 for v in spares):
        raise ValueError("invalid spare GPIO")
    lines.append(f"static_assert(mouse_joystick::pins::kSpareCount == {len(spares)}, \"spare count differs\");")
    for index, value in enumerate(spares):
        lines.append(f"static_assert(mouse_joystick::pins::kSpares[{index}] == {value}, \"spare differs\");")
    return "\n".join(lines)


def check(facts_path, bindings_path, cxx, header=ROOT / "src/BoardPins.hpp"):
    facts = json.loads(Path(facts_path).read_text())
    binding = json.loads(Path(bindings_path).read_text())
    body = assertions(facts, binding)
    with tempfile.TemporaryDirectory(prefix="board-pins-") as directory:
        source = Path(directory) / "check.cpp"
        source.write_text('#include "BoardPins.hpp"\n' + body + "\n")
        result = subprocess.run([*shlex.split(cxx), "-std=c++17", "-fsyntax-only",
                                 f"-DMJ_BOARD_REV={binding['firmware_board_rev']}",
                                 "-I", str(header.parent), str(source)], check=False)
        if result.returncode:
            return result.returncode
    print(f"{len(binding['signals'])} pin constants and spares agree; "
          f"facts sha256={hashlib.sha256(Path(facts_path).read_bytes()).hexdigest()}")
    print(f"Evidence scope: {facts['scope']}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--facts", type=Path, required=True)
    ap.add_argument("--bindings", type=Path, default=ROOT / "instrument/board-pin-bindings.json")
    ap.add_argument("--cxx", default=os.environ.get("CXX", "c++"))
    args = ap.parse_args()
    try:
        return check(args.facts, args.bindings, args.cxx)
    except (ValueError, KeyError, OSError) as exc:
        ap.exit(1, f"{exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
