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

EXPECTED = {"free_parameter", "o3_branches", "f_over_a_turn", "axis_case", "issue_one"}
"""
The project publishes the four figures embedded in ``docs/LINEAR_METHOD.md`` and the ``issue_one``
figure, which demonstrates the solutions for the pose in issue #1. This independent list makes the
test fail if a figure is removed from the registry.
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


def test_the_issue_one_figure_shows_every_reachable_solution():
    """
    Verify the claims printed on the ``issue_one`` figure so its caption remains accurate when the
    solver changes. The figure is posted on the issue as a demonstration, making its caption a
    public claim about the solver.
    """
    from crx.figures.issue_one import JOINT_LIMITS, is_generating, solutions, within_limits
    from crx.ik_reference import CrxParams, SolutionKind, fk, ik

    params = CrxParams.crx10ia()
    joints = [10.0, -80.0, 10.0, 20.0, -20.0, 45.0]
    everything = ik(params, fk(params, joints))
    shown = solutions()

    # Every solution the solver returns for this pose is inside the robot's motion range, so the
    # filter removes none of them and the figure shows the complete set.
    assert len(everything) == 16
    assert len(shown) == 16
    assert all(within_limits(s.joints) for s in shown)

    # Confirm that the results include the generating configuration, which the surveyed libraries
    # failed to return.
    assert sum(is_generating(s.joints) for s in shown) == 1

    # Confirm that the figure marks the four solutions classified as degenerate by the solver.
    assert sum(s.kind is SolutionKind.AXIS_DEGENERATE for s in shown) == 4

    # Keep the panels in a stable base-angle order so labels 1 through 16 continue to match the
    # numbering in the issue text.
    assert [float(s.joints[0]) for s in shown] == sorted(float(s.joints[0]) for s in shown)

    # Require positive limits and verify that their maximum remains a valid bound for the results.
    assert all(limit > 0 for limit in JOINT_LIMITS)
    assert max(abs(v) for s in shown for v in s.joints) <= max(JOINT_LIMITS)
