"""Render the Graphviz sources in docs/diagrams/ to SVG and PNG.

    python tools/render_diagrams.py                 # every .dot under docs/diagrams
    python tools/render_diagrams.py docs/diagrams/architecture.dot

The third step of `pixi run docs` (`make docs`), after the two generators.
Needs `dot` on the path; the pixi environment provides Graphviz.
"""

import pathlib
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
DIAGRAMS = ROOT / "docs" / "diagrams"


def main(argv):
    dot = shutil.which("dot")
    if not dot:
        sys.exit("render_diagrams: `dot` (Graphviz) not found; run `pixi run docs`, or install graphviz")
    files = [pathlib.Path(a).resolve() for a in argv] or sorted(DIAGRAMS.glob("*.dot"))
    if not files:
        sys.exit(f"render_diagrams: no .dot files under {DIAGRAMS}")
    for src in files:
        for fmt, extra in (("svg", []), ("png", ["-Gdpi=144"])):   # the PNG at 1.5x for dense screens
            out = src.with_suffix("." + fmt)
            subprocess.run([dot, f"-T{fmt}", *extra, str(src), "-o", str(out)], check=True)
            print(f"wrote {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main(sys.argv[1:])
