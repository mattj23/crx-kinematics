"""Redraw every figure into ``docs/images``: ``python -m crx.figures``."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")

from . import save_all  # noqa: E402

if __name__ == "__main__":
    for path in save_all():
        print(path)
