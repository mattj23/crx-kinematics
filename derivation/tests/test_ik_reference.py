"""
Tests for the NumPy reference inverse kinematics.

The forward kinematics in `crx.ik_reference` is a NumPy-only restatement of the model in
`crx.robot`, so the tests first check it against the same recorded controller data. The remaining
tests use round trips: select joint angles, calculate their pose, and require the inverse
kinematics to include those joint angles in its solutions. Every returned solution must reach the
target pose.

The random draws in the first group are overwhelmingly unlikely to produce a degenerate
configuration, so the families that need one are set up deliberately in the second group.
"""

import json
import math
from pathlib import Path

import numpy
import pytest

from crx.ik_reference import (
    ALL_MODELS,
    SolutionKind,
    CrxParams,
    Setup,
    dft_coefficients,
    fk,
    fk_all,
    ik,
    ik_closest,
    joints_to_radians,
    wrap_pi,
)

_POSE_TOL = 1e-9
"""
Millimeters. This tolerance is tighter than the solver's acceptance threshold, so the test rejects
a solution that barely meets the solver threshold.
"""

_ORIGIN_EXCLUSION = 1.0
"""
Millimeters. How near the world origin O4 may come before a pose is left out of the on-axis tests.

One of the two configurations the solver cannot recover, described in the module docstring of
`crx.ik_reference`. It arises only where ``z1`` equals ``x1``.
"""

_SLICE_EXCLUSION = 5.0
"""
Millimeters. How cleanly the plane must cut the circle of candidate O3 points.

This exclusion covers the second unrecoverable configuration. O3's position around the circle is
undetermined when the circle has no extent or the plane only grazes it.
"""

_SINGULAR_POSE_TOL = 1e-8
"""
Millimeters, for the poses where a joint is free. The pose is insensitive to motion along the free
direction, so the polish has no gradient in that direction and stops slightly short.
"""

_JOINT_TOL = 1e-3
"""Degrees, for recognizing the joint vector that generated the target."""


_RECORDED_DATA = Path(__file__).parents[2] / "crx-kinematics" / "tests" / "data"
"""
The recorded controller poses are stored with the Rust crate so both test suites can access the
same data. `include_bytes!` can access only files inside the crate, and publication packages only
files under the crate root. Both suites therefore check the same bytes from this location.
"""


def _load_known(name: str) -> list[list]:
    with open(_RECORDED_DATA / f"{name}.json", "r") as handle:
        return json.load(handle)


def _random_joints(rng: numpy.random.Generator, count: int) -> numpy.ndarray:
    return rng.uniform(-180.0, 180.0, size=(count, 6))


def _joint_distance(a, b) -> float:
    """Largest per-joint difference in degrees, treating angles a full turn apart as equal."""
    diff = [wrap_pi(x - y) for x, y in zip(joints_to_radians(a), joints_to_radians(b))]
    return float(numpy.degrees(numpy.abs(diff)).max())


@pytest.mark.parametrize("name", ["fanuc_crx_5ia", "fanuc_crx_10ia"])
def test_forward_matches_recorded_data(name):
    params = CrxParams.crx5ia() if name.endswith("5ia") else CrxParams.crx10ia()
    for joints, expected in _load_known(name):
        result = fk(params, joints).flatten()
        assert result == pytest.approx(expected, abs=1e-8)


@pytest.mark.parametrize("model", sorted(ALL_MODELS))
def test_trigonometric_degree_is_four(model):
    """
    Sampling at twice the density required by the assumed degree must leave the extra coefficients
    at zero. If this fails for a new model, the entire solution-count argument also changes.
    """
    params = ALL_MODELS[model]
    rng = numpy.random.default_rng(11)
    for joints in _random_joints(rng, 25):
        setup = Setup(params, fk(params, joints))
        coefficients = dft_coefficients(setup, count=32)
        magnitude = numpy.abs(coefficients)
        above_degree = numpy.concatenate([magnitude[5:16], magnitude[17:28]])
        assert above_degree.max() < 1e-9 * magnitude.max()


@pytest.mark.parametrize("model", sorted(ALL_MODELS))
def test_round_trip_random_poses(model):
    params = ALL_MODELS[model]
    rng = numpy.random.default_rng(4)
    counts = {}
    for joints in _random_joints(rng, 250):
        target = fk(params, joints)
        solutions = ik(params, target)
        counts[len(solutions)] = counts.get(len(solutions), 0) + 1

        assert solutions, f"no solutions for {joints}"
        assert len(solutions) <= 16
        assert len(solutions) % 2 == 0, "solutions come in J1 front/back pairs"

        for solution in solutions:
            assert solution.residual < _POSE_TOL, f"{solution.joints} misses {joints}"

        closest = min(_joint_distance(s.joints, joints) for s in solutions)
        assert closest < _JOINT_TOL, f"original joints {joints} not recovered"

    print(f"\n{model} solution counts: {dict(sorted(counts.items()))}")


def test_solutions_are_distinct():
    """Verify that each returned solution is a unique configuration."""
    params = CrxParams.crx10ia()
    rng = numpy.random.default_rng(19)
    for joints in _random_joints(rng, 50):
        solutions = ik(params, fk(params, joints))
        for i, first in enumerate(solutions):
            for second in solutions[i + 1:]:
                assert _joint_distance(first.joints, second.joints) > 1e-4


# ---------------------------------------------------------------------------------------------
# Degenerate configurations
#
# Each family contains poses that a real robot reaches and a naive solver mishandles. The selected
# joint values place the arm exactly on the relevant geometry. For cases where nearby poses are
# more difficult, the tests also move the arm slightly away from that geometry.
# ---------------------------------------------------------------------------------------------

_MODELS = sorted(ALL_MODELS)


def _sweep(params, mutate, count=40, seed=3):
    """
    Round trip over `count` random poses, each put onto a family by `mutate`.

    A mutation may return None to refuse a draw that cannot be put onto its family, which is how
    the on-axis case handles a shoulder angle the forearm cannot reach around.
    """
    rng = numpy.random.default_rng(seed)
    used = 0
    for joints in _random_joints(rng, count):
        joints = mutate(joints)
        if joints is None:
            continue
        used += 1
        target = fk(params, joints)
        solutions = ik(params, target)

        assert solutions, f"no solutions for {joints}"
        assert len(solutions) <= 16

        # A pose with a free joint has a continuum of solutions. The returned representative can
        # differ from the joint values that produced the pose, so the test verifies only that the
        # solutions reach it. The polish also stops slightly short because the pose has no gradient
        # along the free direction, so these solutions use the looser tolerance.
        has_free_joint = any(s.kind is SolutionKind.SINGULAR_FAMILY for s in solutions)
        tolerance = _SINGULAR_POSE_TOL if has_free_joint else _POSE_TOL
        for solution in solutions:
            assert solution.residual < tolerance

        if has_free_joint:
            continue

        assert len(solutions) % 2 == 0, f"{len(solutions)} solutions for {joints}"
        closest = min(_joint_distance(s.joints, joints) for s in solutions)
        assert closest < _JOINT_TOL, f"original joints {list(joints)} not recovered"

    assert used > count // 8, f"only {used} of {count} draws were usable"


def _on_axis(params, joints):
    """
    Put O4 exactly on the J1 axis, or return None when this shoulder angle cannot be compensated.

    In the frame J1 turns to, O4 sits at ``z1 sin(J2) + x1 cos(J3)`` off the axis, with J3 as the
    controller reports it. Setting that to zero fixes J3 from J2. On a model whose ``z1`` exceeds
    its ``x1``, some shoulder angles put the axis out of the forearm's reach, and those are refused.

    Note that ``J3 = J2 + 90`` solves this only where the two lengths are equal, which is to say on
    the CRX-3iA and the CRX-10iA and nowhere else. An earlier version of this helper used that
    form, and on the other four models it produced ordinary poses without indicating the error.
    """
    joints = numpy.array(joints, dtype=float)
    ratio = -params.z1 * math.sin(math.radians(joints[1])) / params.x1
    if abs(ratio) > 1.0:
        return None
    joints[2] = math.degrees(math.acos(ratio))

    return joints if _is_testable(params, joints) else None


def _is_testable(params, joints) -> bool:
    """
    Whether a configuration is one the on-axis tests can ask about.

    Refuses the two geometries the solver cannot recover, which are described in the module
    docstring of `crx.ik_reference`: O4 on or beside the world origin, and a plane that fails to
    cut the circle of candidate O3 points cleanly.
    """
    frames = fk_all(params, joints)
    o4 = (frames[3] @ numpy.array([params.x1, 0.0, 0.0, 1.0]))[:3]
    o5 = (frames[4] @ numpy.array([0.0, 0.0, 0.0, 1.0]))[:3]

    distance = float(numpy.linalg.norm(o4))
    if distance < _ORIGIN_EXCLUSION:
        return False

    along = (params.z1 ** 2 - params.x1 ** 2 + distance ** 2) / (2.0 * distance)
    radius = math.sqrt(max(params.z1 ** 2 - along ** 2, 0.0))
    center_z = along * numpy.sign(o4[2])
    u = (o4 - o5) / params.y1

    reach = radius * math.hypot(u[0], u[1])
    wanted = abs((o4[2] - center_z) * u[2])
    return min(reach, reach - wanted) >= _SLICE_EXCLUSION


@pytest.mark.parametrize("model", _MODELS)
def test_o4_on_the_j1_axis(model):
    """
    The plane constraint provides no information when O4 is on the axis, so O3 determines the base
    angle. The pose in issue #1 belongs to this family.
    """
    params = ALL_MODELS[model]
    _sweep(params, lambda j: _on_axis(params, j))


@pytest.mark.parametrize("offset", [1e-9, 1e-7, 1e-5, 1e-3, 1e-1])
def test_near_the_j1_axis(offset):
    """
    Passing near the axis is harder than passing through it. The vertical plane through O4 remains
    defined but swings through a wide arc as O4 moves past. The sweep covers offsets from a
    nanometer-scale miss to a comfortable distance.
    """
    params = CrxParams.crx10ia()

    def mutate(joints):
        on_axis = _on_axis(params, joints)
        if on_axis is None:
            return None
        # The offset moves O4, so apply the exclusions again afterward.
        moved = on_axis + numpy.array([0, 0, offset, 0, 0, 0])
        return moved if _is_testable(params, moved) else None

    _sweep(params, mutate)


@pytest.mark.parametrize("value", [0.0, 1e-9, 1e-6, 1e-3])
def test_j4_at_zero(value):
    """Where the elbow-up and elbow-down solutions merge, f has a double root."""
    def mutate(joints):
        joints = numpy.array(joints, dtype=float)
        joints[3] = value
        return joints

    _sweep(CrxParams.crx10ia(), mutate)


def test_j5_at_zero():
    """A wrist singularity: J4 and J6 turn against each other with no effect on the pose."""
    def mutate(joints):
        joints = numpy.array(joints, dtype=float)
        joints[4] = 0.0
        return joints

    _sweep(CrxParams.crx10ia(), mutate)


@pytest.mark.parametrize("model", _MODELS)
def test_fully_stretched_arm(model):
    """
    With the upper arm and forearm in line, the two spheres that locate O3 are tangent and the
    radius of their intersection circle is zero, or a rounding error below it.
    """
    def mutate(joints):
        joints = numpy.array(joints, dtype=float)
        joints[1] = 90.0
        joints[2] = 0.0
        return joints

    _sweep(ALL_MODELS[model], mutate)


@pytest.mark.parametrize("model", _MODELS)
@pytest.mark.parametrize("elbow", [90.0, -90.0])
def test_j1_free(model, elbow):
    """
    With the shoulder upright and the forearm turned a quarter turn, O3 and O4 are both on the J1
    axis and the base angle no longer changes the pose. There is a continuum of solutions, so the
    test checks that a representative is reported, marked, and able to reach the target. The
    representative can differ from the joint values that produced the pose because J1 is free.
    """
    params = ALL_MODELS[model]
    joints = [35.0, 0.0, elbow, 20.0, -40.0, 15.0]
    solutions = ik(params, fk(params, joints))

    assert any(s.kind is SolutionKind.SINGULAR_FAMILY for s in solutions)
    for solution in solutions:
        assert solution.residual < _SINGULAR_POSE_TOL


def test_issue_one_pose():
    """
    The CRX-10iA pose from https://github.com/mattj23/crx-kinematics/issues/1. O4 lands exactly on
    the J1 axis here, which caused failures in the libraries surveyed in the README. All
    sixteen solutions exist, and the joint values that produced the pose are among them.
    """
    params = CrxParams.crx10ia()
    joints = [10.0, -80.0, 10.0, 20.0, -20.0, 45.0]
    solutions = ik(params, fk(params, joints))

    assert len(solutions) == 16
    assert any(s.kind is SolutionKind.AXIS_DEGENERATE for s in solutions)
    assert min(_joint_distance(s.joints, joints) for s in solutions) < _JOINT_TOL


def test_out_of_reach():
    """Verify that an unreachable target produces no solution."""
    params = CrxParams.crx10ia()
    target = fk(params, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    target[0, 3] += 10_000.0
    assert ik(params, target) == []


def test_solutions_reach_full_precision():
    """
    Every solution must reach the limit of double precision. This requirement is stricter than the
    acceptance tolerance and verifies the benefit of the joint-space polish.
    """
    params = CrxParams.crx10ia()
    rng = numpy.random.default_rng(21)
    worst = 0.0
    for joints in _random_joints(rng, 60):
        for solution in ik(params, fk(params, joints)):
            worst = max(worst, solution.residual)
    assert worst < 1e-11, f"worst residual was {worst:.2e}"


def test_ik_closest_follows_a_reference():
    """
    Picking the solution nearest the robot's current joints is how a caller keeps the arm from
    reconfiguring itself mid-path, so asking for the pose it is already in must return it.
    """
    params = CrxParams.crx10ia()
    rng = numpy.random.default_rng(5)
    for joints in _random_joints(rng, 40):
        solution = ik_closest(params, fk(params, joints), joints)
        assert solution is not None
        assert _joint_distance(solution.joints, joints) < _JOINT_TOL


def test_ik_closest_returns_nothing_when_unreachable():
    params = CrxParams.crx10ia()
    target = fk(params, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    target[2, 3] += 10_000.0
    assert ik_closest(params, target, [0.0] * 6) is None
