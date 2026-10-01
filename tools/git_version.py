"""PlatformIO pre-build script: define MJ_FW_VERSION from git.

`ID` reports it as `fw`. `git describe --tags --always --dirty` gives a tag,
or the short commit, with `-dirty` when the tree has uncommitted changes; the
value can contain no `;` or whitespace, which the protocol forbids in a field.
Without git, or outside a checkout, the define is left out and
src/FirmwareVersion.hpp falls back to "unversioned".
"""

import subprocess

Import("env")  # noqa: F821 - provided by PlatformIO

try:
    version = subprocess.check_output(
        ["git", "describe", "--tags", "--always", "--dirty"],
        cwd=env.subst("$PROJECT_DIR"),
        text=True,
        stderr=subprocess.DEVNULL,
    ).strip()
except Exception:  # noqa: BLE001 - any failure means "no version", never a broken build
    version = ""

if version:
    version = version.replace(";", "_").replace(" ", "_")
    env.Append(CPPDEFINES=[("MJ_FW_VERSION", env.StringifyMacro(version))])
