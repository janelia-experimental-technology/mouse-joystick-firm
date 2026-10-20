"""Check rejection of a wrong export and of implicit/invalid revisions."""
import copy
import os
from pathlib import Path
import shlex
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from check_board_pins import assertions


class BoardPinChecks(unittest.TestCase):
    def setUp(self):
        # Small independent test export, not the production hardware record.
        self.binding = {
            "schema": 1, "board": "test", "board_revision": "test",
            "firmware_board_rev": 2, "signals": {"kPycUartTx": "/PYC_UART_TX"},
            "active_low": [],
        }
        self.facts = dict(self.binding, signals={"/PYC_UART_TX": {"gpio": 0}},
                          spare_gpios=[20, 22])

    def compile(self, body="", revision=2):
        command = [*shlex.split(os.environ.get("CXX", "c++")),
                   "-std=c++17", "-fsyntax-only", "-x", "c++",
                   "-I", str(ROOT / "src")]
        if revision is not None:
            command.append(f"-DMJ_BOARD_REV={revision}")
        return subprocess.run(command + ["-"], input='#include "BoardPins.hpp"\n' + body,
                              text=True, capture_output=True)

    def test_all_revisions_compile_and_missing_or_unknown_fails(self):
        for revision in (0, 1, 2):
            result = self.compile(revision=revision)
            self.assertEqual(result.returncode, 0, result.stderr)
        for revision in (None, -1, 3):
            self.assertNotEqual(self.compile(revision=revision).returncode, 0)

    def test_hardware_gpio_and_spare_mismatch_fail_at_compile(self):
        result = self.compile(assertions(self.facts, self.binding))
        self.assertEqual(result.returncode, 0, result.stderr)
        for bad in ("gpio", "spare"):
            facts = copy.deepcopy(self.facts)
            if bad == "gpio":
                facts["signals"]["/PYC_UART_TX"]["gpio"] = 1
            else:
                facts["spare_gpios"] = [20, 23]
            self.assertNotEqual(self.compile(assertions(facts, self.binding)).returncode, 0)

    def test_revision_polarity_and_missing_signal_fail(self):
        wrong = dict(self.facts, firmware_board_rev=1)
        with self.assertRaisesRegex(ValueError, "firmware_board_rev"):
            assertions(wrong, self.binding)
        self.binding["active_low"] = ["/PYC_UART_TX"]
        self.facts["signals"]["/PYC_UART_TX"]["active_low"] = False
        with self.assertRaisesRegex(ValueError, "active-low"):
            assertions(self.facts, self.binding)
        with self.assertRaisesRegex(ValueError, "cover exactly"):
            assertions(dict(self.facts, signals={}), self.binding)


if __name__ == "__main__":
    unittest.main()
