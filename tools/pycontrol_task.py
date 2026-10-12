#!/usr/bin/env python3
"""
pyControl host task wrapper used by pixi tasks in the `pycontrol` environment.

Why this exists:
- pyControl is not an installable package. It is a checkout you run in place,
  and it MUST run with the working directory set to its own root: `settings.py`
  builds every default folder path from `os.getcwd()`, and `pyControl_GUI.pyw`
  writes `ErrorLog.txt` into the current directory.
- The location of that checkout differs per machine (a Linux development box, the
  Windows rig PC), so it cannot be hard-coded in `pixi.toml`.
- Pointing an *unforked* pyControl at this repository's Stage 4 code takes two
  different mechanisms, for reasons explained at LINKED_FOLDERS, and one of them
  is a JSON file with no UI for absolute paths. `link` does both, reversibly.

Typical usage (via pixi tasks):
  pixi run -e pycontrol pycontrol-check
  pixi run -e pycontrol gui
  pixi run -e pycontrol pycontrol-link
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

# Folders pyControl resolves through `get_setting("folders", ...)` that this
# repository can safely own outright. `data` and `experiments` stay with
# pyControl: they are session output and rig configuration, not firmware.
#
# `devices` is deliberately ABSENT and must stay absent. pyControl scans exactly
# one devices folder to build its device-class map
# (`pycboard.make_device_class2file_map`, which does
# `os.listdir(user_folder("devices"))`), so redirecting it to this repository
# would hide `Breakout_1_2`, `UART_handler` and every other stock driver, and the
# hardware definition would fail to import. A custom device is instead placed
# INTO pyControl's own devices folder, which is the upstream mechanism: any
# `class` in any `.py` there is auto-discovered, with no registration step.
LINKED_FOLDERS = ("hardware_definitions", "tasks")


def _project_root() -> Path:
    # Pixi sets PIXI_PROJECT_ROOT when running tasks.
    env_root = os.environ.get("PIXI_PROJECT_ROOT")
    if env_root:
        return Path(env_root).resolve()

    # Fallback: tools/pycontrol_task.py -> repo root is parent of tools/
    return Path(__file__).resolve().parents[1]


def _candidate_dirs(root: Path) -> list[Path]:
    """Where a pyControl checkout might be, in priority order."""
    candidates: list[Path] = []
    env_dir = os.environ.get("PYCONTROL_DIR")
    if env_dir:
        candidates.append(Path(env_dir).expanduser())

    home = Path.home()
    candidates += [
        home / "Repositories" / "github" / "pycontrol-code",
        home / "Repositories" / "github" / "joystick-rig" / "pycontrol",
        root.parent / "pycontrol-code",
    ]
    return candidates


def _is_pycontrol_checkout(path: Path) -> bool:
    # Two markers, not one: a bare `pyControl_GUI.pyw` could be a stray copy.
    return (path / "pyControl_GUI.pyw").is_file() and (path / "source" / "gui" / "GUI_main.py").is_file()


def _resolve_pycontrol_dir(root: Path, explicit: str | None) -> Path:
    if explicit:
        path = Path(explicit).expanduser().resolve()
        if not _is_pycontrol_checkout(path):
            raise SystemExit(f"ERROR: not a pyControl checkout: {path}\n  (expected pyControl_GUI.pyw and source/gui/GUI_main.py)")
        return path

    tried: list[Path] = []
    for candidate in _candidate_dirs(root):
        resolved = candidate.resolve()
        tried.append(resolved)
        if _is_pycontrol_checkout(resolved):
            return resolved

    listing = "\n".join(f"  {p}" for p in tried)
    raise SystemExit(
        "ERROR: no pyControl checkout found. Tried:\n"
        f"{listing}\n\n"
        "Set PYCONTROL_DIR, or pass --pycontrol-dir /path/to/pycontrol-code."
    )


def _pycontrol_version(pycontrol_dir: Path) -> str:
    settings = pycontrol_dir / "source" / "gui" / "settings.py"
    try:
        for line in settings.read_text(encoding="utf-8").splitlines():
            if line.startswith("VERSION"):
                return line.split("=", 1)[1].strip().strip('"').strip("'")
    except OSError:
        pass
    return "unknown"


def cmd_check(args: argparse.Namespace, root: Path) -> int:
    """Replicate pyControl's own dependency gate, and report versions."""
    print(f"python      {sys.version.split()[0]}  ({sys.executable})")

    failed = False
    # These three are exactly what pyControl_GUI.pyw imports before launching.
    for module_name, label in (("numpy", "numpy"), ("serial", "pyserial"), ("pyqtgraph", "pyqtgraph")):
        try:
            module = __import__(module_name)
            version = getattr(module, "__version__", getattr(module, "VERSION", "?"))
            print(f"{label:<11} {version}")
        except Exception as exc:  # noqa: BLE001 - mirrors pyControl's broad catch
            print(f"{label:<11} MISSING - {exc}")
            failed = True

    # GUI_main.py imports Qt via pyqtgraph.Qt, so the binding is pyqtgraph's
    # choice. Report which one it picked rather than assuming pyqt6 won.
    try:
        from pyqtgraph.Qt import QT_LIB, QtCore

        qt_version = getattr(QtCore, "QT_VERSION_STR", "?")
        print(f"Qt binding  {QT_LIB} (Qt {qt_version})")
    except Exception as exc:  # noqa: BLE001
        print(f"Qt binding  MISSING - {exc}")
        failed = True

    if args.pycontrol_dir or os.environ.get("PYCONTROL_DIR"):
        pycontrol_dir = _resolve_pycontrol_dir(root, args.pycontrol_dir)
        print(f"pyControl   v{_pycontrol_version(pycontrol_dir)}  ({pycontrol_dir})")

    if failed:
        print("\nFAILED: pyControl would refuse to launch with this environment.", file=sys.stderr)
        return 1

    print("\nOK: pyControl's dependency gate is satisfied.")
    return 0


def cmd_gui(args: argparse.Namespace, root: Path) -> int:
    pycontrol_dir = _resolve_pycontrol_dir(root, args.pycontrol_dir)
    print(f"pyControl v{_pycontrol_version(pycontrol_dir)} at {pycontrol_dir}")
    print(f"+ cd {pycontrol_dir} && python pyControl_GUI.pyw")

    # cwd matters: settings.py derives every default folder from os.getcwd().
    completed = subprocess.run(
        [sys.executable, "pyControl_GUI.pyw", *args.extra],
        cwd=str(pycontrol_dir),
        check=False,
    )
    if completed.returncode != 0:
        print(
            f"\npyControl exited {completed.returncode}. It logs import failures to\n"
            f"  {pycontrol_dir / 'ErrorLog.txt'}\n"
            "rather than to the terminal.",
            file=sys.stderr,
        )
    return int(completed.returncode)


def _link_device_files(root: Path, pycontrol_dir: Path, *, unlink: bool, dry_run: bool) -> None:
    """Place this repo's device classes into pyControl's own devices folder.

    A symlink keeps one copy of the file, the one under version control here.
    Windows refuses symlinks without developer mode or elevation, so fall back to
    a copy there and say so - a silent copy is how a stale device class ends up
    on the pyboard while the repo says otherwise.
    """
    # The LEAN copies (tools/pycontrol_lean.py), not the sources: the pyboard's
    # flash is 95 KB and pyControl uploads files verbatim (2026-10-06).
    source_dir = root / "pycontrol" / "dist" / "devices"
    target_dir = pycontrol_dir / "devices"
    device_files = sorted(source_dir.glob("*.py")) if source_dir.is_dir() else []

    if not device_files:
        print(f"No device classes in {source_dir} yet; nothing to place in {target_dir}.")
        return

    for source in device_files:
        target = target_dir / source.name
        if unlink:
            if target.is_symlink() or target.exists():
                if not dry_run:
                    target.unlink()
                print(f"{'Would remove' if dry_run else 'Removed'} {target}")
            continue

        if target.is_symlink() or target.exists():
            if not dry_run:
                target.unlink()

        if dry_run:
            print(f"Would link {target} -> {source}")
            continue

        try:
            target.symlink_to(source)
            print(f"Linked {target} -> {source}")
        except (OSError, NotImplementedError):
            target.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
            print(
                f"COPIED {source} -> {target}\n"
                "  (symlink unavailable on this platform; re-run `pycontrol-link` after\n"
                "   every edit to the device class, or the pyboard gets a stale copy)"
            )


def cmd_link(args: argparse.Namespace, root: Path) -> int:
    """Point an unforked pyControl checkout at this repository's Stage 4 code.

    This is what `mouse-joystick-pcb/docs/validation-plan.md` Gate E requires:
    an unforked upstream pyControl loading the device class through its normal
    supported path. Two mechanisms, because pyControl treats the folders
    differently - see LINKED_FOLDERS.
    """
    pycontrol_dir = _resolve_pycontrol_dir(root, args.pycontrol_dir)
    settings_path = pycontrol_dir / "config" / "settings.json"

    # Regenerate the lean copies first: pyControl is pointed at
    # pycontrol/dist/, never at the commented sources (see pycontrol_lean.py).
    if not args.unlink and not args.dry_run:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from pycontrol_lean import generate  # noqa: E402

        generate(root, verbose=False)

    folders = {}
    for name in LINKED_FOLDERS:
        source_dir = root / "pycontrol" / "dist" / name
        if not source_dir.is_dir():
            raise SystemExit(f"ERROR: missing {source_dir}. Run tools/pycontrol_lean.py first.")
        folders[name] = str(source_dir)

    existing: dict = {}
    if settings_path.exists():
        try:
            existing = json.loads(settings_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            raise SystemExit(f"ERROR: {settings_path} is not valid JSON: {exc}")

        already = existing.get("folders", {})
        conflicts = {k: v for k, v in already.items() if k in folders and v != folders[k]}
        if conflicts and not args.force:
            print(f"{settings_path} already redirects these folders elsewhere:")
            for key, value in conflicts.items():
                print(f"  {key}:\n    is  {value}\n    new {folders[key]}")
            print("\nRefusing to overwrite. Re-run with --force to replace them.")
            return 1

    if args.unlink:
        _link_device_files(root, pycontrol_dir, unlink=True, dry_run=args.dry_run)
        if not settings_path.exists():
            print(f"No {settings_path} to clean up.")
            return 0
        remaining = existing.get("folders", {})
        for name in LINKED_FOLDERS:
            remaining.pop(name, None)
        existing["folders"] = remaining
        if not args.dry_run:
            settings_path.write_text(json.dumps(existing, indent=4) + "\n", encoding="utf-8")
        print(f"Removed {', '.join(LINKED_FOLDERS)} overrides from {settings_path}")
        print("pyControl falls back to its own folders on next launch.")
        return 0

    # Preserve any settings we do not own, including other folder overrides.
    merged = dict(existing)
    merged_folders = dict(existing.get("folders", {}))
    merged_folders.update(folders)
    merged["folders"] = merged_folders
    # Required, not tidiness. `settings.get_setting` indexes
    # `custom_settings[setting_type]` before it falls back to its defaults, so a
    # settings.json holding only "folders" raises KeyError the first time
    # pyControl asks for a plotting or GUI setting. Empty dicts let the fallback
    # work as intended.
    for required in ("plotting", "GUI"):
        merged.setdefault(required, {})

    if settings_path.exists() and not args.dry_run:
        backup = settings_path.with_suffix(".json.bak")
        backup.write_text(settings_path.read_text(encoding="utf-8"), encoding="utf-8")
        print(f"Backed up existing settings to {backup}")

    payload = json.dumps(merged, indent=4) + "\n"
    if args.dry_run:
        print(f"Would write {settings_path}:\n{payload}")
    else:
        settings_path.parent.mkdir(parents=True, exist_ok=True)
        settings_path.write_text(payload, encoding="utf-8")
        print(f"Wrote {settings_path}")
        for name in LINKED_FOLDERS:
            print(f"  {name} -> {folders[name]}")

    _link_device_files(root, pycontrol_dir, unlink=False, dry_run=args.dry_run)

    print(
        "\npyControl reads config/settings.json relative to its working directory,\n"
        "so this only takes effect when pyControl is launched from its own root -\n"
        "which is what `pixi run -e pycontrol gui` does."
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(prog="pycontrol_task.py")
    sub = parser.add_subparsers(dest="command", required=True)

    def add_dir_arg(sp: argparse.ArgumentParser) -> None:
        sp.add_argument(
            "--pycontrol-dir",
            default=None,
            help="Path to a pyControl checkout. Defaults to $PYCONTROL_DIR, then known locations.",
        )

    sp_check = sub.add_parser("check", help="Verify pyControl's dependency gate is satisfied.")
    add_dir_arg(sp_check)

    sp_gui = sub.add_parser("gui", help="Launch the pyControl GUI from a checkout.")
    add_dir_arg(sp_gui)

    sp_link = sub.add_parser("link", help="Repoint a pyControl checkout's user folders at this repo.")
    add_dir_arg(sp_link)
    sp_link.add_argument("--force", action="store_true", help="Replace conflicting folder overrides.")
    sp_link.add_argument("--unlink", action="store_true", help="Remove this repo's overrides again.")
    sp_link.add_argument("--dry-run", action="store_true", help="Print the settings.json without writing it.")

    args, extra = parser.parse_known_args(argv)
    args.extra = extra
    root = _project_root()

    if args.command == "check":
        return cmd_check(args, root)
    if args.command == "gui":
        return cmd_gui(args, root)
    if args.command == "link":
        return cmd_link(args, root)

    parser.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
