"""
The figures used by ``docs/LINEAR_METHOD.md``.

Each module draws one figure and exposes a ``draw()`` function that returns a Matplotlib ``Figure``.
Running ``python -m crx.figures`` redraws all of them into ``docs/images``.
"""

from __future__ import annotations

from pathlib import Path

from . import axis_case, branches, f_over_a_turn, free_parameter, issue_one
from ._common import IMAGE_DIR

FIGURES = {
    "free_parameter": free_parameter.draw,
    "o3_branches": branches.draw,
    "f_over_a_turn": f_over_a_turn.draw,
    "axis_case": axis_case.draw,
    "issue_one": issue_one.draw,
}
"""The figure functions, keyed by their output file stems."""


def save_all(out_dir: Path | None = None) -> list[Path]:
    """
    Redraw every figure and write it as a PNG.

    :param out_dir: where to write them, defaulting to ``docs/images``
    :return: the paths written, in the order the figures are declared
    """
    out_dir = IMAGE_DIR if out_dir is None else Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    written = []
    for name, draw in FIGURES.items():
        path = out_dir / f"{name}.png"
        draw().savefig(path)
        written.append(path)
    return written
