"""
Interoperability with engeom.

The packages remain independent and interoperate through the `as_numpy()` method on engeom's
`Iso3`. The binding calls this method when a target is not already an array. These tests skip
themselves when engeom is not installed.
"""

from __future__ import annotations

import numpy
import pytest

from crx_kinematics import Crx

engeom = pytest.importorskip("engeom")
from engeom.geom3 import Iso3  # noqa: E402


def test_an_engeom_isometry_can_be_used_as_a_target():
    robot = Crx.crx10ia()
    joints = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
    target = Iso3(robot.fk(joints))

    solutions = robot.ik(target)
    assert len(solutions) > 0
    for solution in solutions:
        assert numpy.allclose(robot.fk(solution), target.as_numpy(), atol=1e-9)


def test_forward_kinematics_output_can_become_an_engeom_isometry():
    """
    An `Iso3` rejects a matrix that is not a rigid transformation. Building one from every frame of
    the chain therefore verifies that each matrix remains valid through the round trip.
    """
    robot = Crx.crx10ia()
    frames = robot.fk_all([10.0, -20.0, 30.0, -40.0, 50.0, -60.0])

    for frame in frames:
        assert numpy.allclose(Iso3(frame).as_numpy(), frame, atol=1e-12)
