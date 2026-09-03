"""
Smoke tests for the figure tooling.

These tests cannot assess visual quality. They detect a figure that fails to build after a reference
solver change, a construction that no longer satisfies its documented geometry, or a registry that
has diverged from the adjacent modules.

Building each figure runs its assertions and verifies its geometric claims.
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")

import pytest

from crx.figures import FIGURES, save_all

EXPECTED = {"free_parameter", "o3_branches", "f_over_a_turn", "axis_case"}
"""
The figures embedded in ``docs/LINEAR_METHOD.md``. Listing them independently ensures that the test
fails if a figure is removed from the registry.
"""


def test_every_expected_figure_is_registered():
    assert set(FIGURES) == EXPECTED


@pytest.mark.parametrize("name", sorted(EXPECTED))
def test_each_figure_builds(name: str):
    figure = FIGURES[name]()

    # A blank figure remains a valid Figure object, so check that it contains axes and artists.
    axes = figure.get_axes()
    assert axes, f"{name} produced no axes"
    assert any(ax.get_children() for ax in axes), f"{name} produced no artists"


def test_saving_writes_every_figure(tmp_path):
    written = save_all(tmp_path)

    assert {path.stem for path in written} == EXPECTED
    for path in written:
        assert path.is_file()
        # A PNG of a blank axes is a few kilobytes; these are line drawings and much larger.
        assert path.stat().st_size > 20_000, f"{path.name} looks empty at {path.stat().st_size} B"
