"""Verify this Pixi environment can actually run pyControl v2.1.1.

This is the environment's own gate, not a test of any code in this repository.
It exists because pyControl fails its import check *silently*: `pyControl_GUI.pyw`
catches the exception, writes it to `ErrorLog.txt`, and calls `sys.exit()`. A
missing dependency therefore looks like "the GUI did not open" with nothing on
the terminal. Failing here instead gives the reason.

Run with:  pixi run -e pycontrol pycontrol-test
"""

import importlib

import pytest


# Exactly the three modules pyControl_GUI.pyw imports before launching.
PYCONTROL_GATE = ("numpy", "serial", "pyqtgraph")


@pytest.mark.parametrize("module_name", PYCONTROL_GATE)
def test_pycontrol_dependency_gate(module_name):
    """pyControl_GUI.pyw exits before opening a window if any of these fail."""
    assert importlib.import_module(module_name) is not None


def test_serial_tools_list_ports_importable():
    """A separate import path from `serial`, and pyControl needs it too.

    `source/gui/GUI_main.py` does `from serial.tools import list_ports` to
    populate the setup dropdown. A pyserial install that satisfies `import
    serial` but not this would pass pyControl's own check and then fail at GUI
    construction.
    """
    from serial.tools import list_ports

    # Enumerating zero ports is a valid result; raising is not.
    assert isinstance(list(list_ports.comports()), list)


def test_qt_binding_resolves_through_pyqtgraph():
    """pyControl reaches Qt only via `pyqtgraph.Qt`, so pyqtgraph picks the binding.

    Asserting on the binding pyqtgraph *chose* rather than on `import PyQt6`
    is the point: installing PyQt6 does not guarantee pyqtgraph selects it, and
    the selected one is what pyControl will use.
    """
    from pyqtgraph.Qt import QT_LIB, QtCore, QtWidgets

    assert QT_LIB in ("PyQt6", "PySide6", "PyQt5", "PySide2"), f"unexpected Qt binding: {QT_LIB}"
    assert QtWidgets is not None
    assert QtCore.QT_VERSION_STR


def test_numpy_is_version_2_or_later():
    """Guards against the rig's camera pin leaking into this environment.

    `joystick-rig/scripts/constraints.txt` pins `numpy<2` because PySpin
    4.3.0.190 is built against numpy 1.x. That constraint belongs to the camera
    acquisition process, not to pyControl, and pyqtgraph 0.14 expects numpy 2.
    If this assertion ever fails, someone merged the two environments.
    """
    import numpy

    major = int(numpy.__version__.split(".")[0])
    assert major >= 2, f"numpy {numpy.__version__} - is the PySpin numpy<2 pin leaking in?"
