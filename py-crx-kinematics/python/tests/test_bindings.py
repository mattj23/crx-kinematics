"""
Tests for the Python bindings.

The kinematics are tested in Rust and in the NumPy reference. These tests verify that every accepted
argument form reaches the solver, results have the documented shapes, and invalid input raises an
exception. The small round trips verify the Python-to-Rust integration without repeating the full
solver tests.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy
import pytest

from crx_kinematics import Crx, CrxModel, IkSolution, LinkMeshes, SolutionKind

_RECORDED_DATA = Path(__file__).parents[3] / "crx-kinematics" / "tests" / "data"
"""
The recorded controller data lives inside the Rust crate, which is the only copy of it. Reading it
here means both languages check the same bytes.
"""

_POSE_TOL = 1e-9
"""Millimeters, and tighter than the solver's own acceptance threshold."""

_MODELS = {
    "fanuc_crx_5ia": Crx.crx5ia,
    "fanuc_crx_10ia": Crx.crx10ia,
}
"""The two models with recorded controller data."""


def _load_recorded(name: str) -> list:
    with open(_RECORDED_DATA / f"{name}.json", "r") as handle:
        return json.load(handle)


def _random_joints(rng: numpy.random.Generator) -> numpy.ndarray:
    return rng.uniform(-180.0, 180.0, size=6)


@pytest.fixture(scope="module")
def robot() -> Crx:
    return Crx.crx10ia()


def test_the_models_have_their_published_dimensions():
    """
    Each constructor is checked against its link dimensions in millimeters, in the order
    (z1, x1, x2, y1). The CRX-10iA/L and the CRX-20iA/L share an arm, so their dimensions are
    identical and only the payload differs.
    """
    expected = {
        Crx.crx3ia: (280.0, 280.0, 123.0, 111.0),
        Crx.crx5ia: (410.0, 430.0, 145.0, 130.0),
        Crx.crx10ia: (540.0, 540.0, 160.0, 150.0),
        Crx.crx10ial: (710.0, 540.0, 160.0, 150.0),
        Crx.crx20ial: (710.0, 540.0, 160.0, 150.0),
        Crx.crx30ia: (950.0, 750.0, 180.0, 185.0),
    }

    for build, dimensions in expected.items():
        robot = build()
        assert (robot.z1, robot.x1, robot.x2, robot.y1) == dimensions


def test_from_params_matches_a_named_model():
    named = Crx.crx10ia()
    built = Crx.from_params(named.z1, named.x1, named.x2, named.y1)
    joints = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
    assert numpy.allclose(named.fk(joints), built.fk(joints), atol=_POSE_TOL)


@pytest.mark.parametrize("name", sorted(_MODELS))
def test_forward_kinematics_matches_the_controller(name: str):
    robot = _MODELS[name]()
    records = _load_recorded(name)
    assert len(records) > 0

    for joints, expected in records:
        result = robot.fk(joints)
        assert numpy.allclose(result, numpy.array(expected).reshape(4, 4), atol=1e-6)


def test_joints_may_be_a_list_a_tuple_or_an_array(robot: Crx):
    joints = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
    reference = robot.fk(joints)

    assert numpy.allclose(robot.fk(tuple(joints)), reference)
    assert numpy.allclose(robot.fk(numpy.array(joints)), reference)

    # Integers are a common way to write joint angles by hand, and are accepted by conversion.
    assert numpy.allclose(robot.fk([10, -20, 30, -40, 50, -60]), reference)


def test_the_chain_ends_at_the_flange(robot: Crx):
    joints = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
    frames = robot.fk_all(joints)

    assert frames.shape == (6, 4, 4)
    assert numpy.allclose(frames[5], robot.fk(joints), atol=_POSE_TOL)

    # Every frame is a rigid transformation, so each rotation block is orthonormal and the last row
    # is the homogeneous one.
    for frame in frames:
        rotation = frame[:3, :3]
        assert numpy.allclose(rotation @ rotation.T, numpy.eye(3), atol=1e-12)
        assert numpy.allclose(frame[3], [0.0, 0.0, 0.0, 1.0])


def test_inverse_kinematics_recovers_the_joints_it_was_given(robot: Crx):
    rng = numpy.random.default_rng(20260903)

    for _ in range(50):
        joints = _random_joints(rng)
        target = robot.fk(joints)
        solutions = robot.ik(target)

        assert solutions.ndim == 2 and solutions.shape[1] == 6
        assert len(solutions) > 0

        # Verify that every solution reaches the target and that the solutions include the joints
        # used to produce the pose.
        for solution in solutions:
            assert numpy.allclose(robot.fk(solution), target, atol=_POSE_TOL)

        assert any(_same_joints(solution, joints) for solution in solutions)


def _same_joints(a, b, tol: float = 1e-5) -> bool:
    """Compare joint vectors as angles, since two values a full turn apart drive the same pose."""
    return all(abs(math.remainder(x - y, 360.0)) < tol for x, y in zip(a, b))


def test_an_unreachable_target_gives_no_solutions(robot: Crx):
    target = numpy.eye(4)
    target[0, 3] = 10_000.0

    assert robot.ik(target).shape == (0, 6)
    assert robot.ik_detailed(target) == []
    assert robot.ik_closest(target, [0.0] * 6) is None


def test_detailed_solutions_carry_their_diagnostics(robot: Crx):
    joints = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
    target = robot.fk(joints)
    detailed = robot.ik_detailed(target)

    assert len(detailed) == len(robot.ik(target))
    for solution in detailed:
        assert isinstance(solution, IkSolution)
        assert solution.joints.shape == (6,)
        assert solution.residual < _POSE_TOL
        assert solution.kind is SolutionKind.Regular
        # `theta` is the angle the root finder worked in, and the origin shift it applies means
        # the value is not confined to a canonical range.
        assert math.isfinite(solution.theta)
        assert "IkSolution(" in repr(solution)


def test_the_pose_from_issue_1_is_the_on_axis_degeneracy():
    """
    The pose in issue #1 puts the wrist center on the J1 axis, creating a degenerate configuration.
    The solver still returns all sixteen solutions, including the joint values used to build the
    pose.
    """
    robot = Crx.crx10ia()
    joints = [10.0, -80.0, 10.0, 20.0, -20.0, 45.0]
    target = robot.fk(joints)
    detailed = robot.ik_detailed(target)

    assert len(detailed) == 16
    assert any(s.kind is SolutionKind.AxisDegenerate for s in detailed)
    assert any(_same_joints(s.joints, joints) for s in detailed)


def test_closest_selects_the_nearby_configuration(robot: Crx):
    rng = numpy.random.default_rng(20260904)

    for _ in range(25):
        joints = _random_joints(rng)
        target = robot.fk(joints)

        # Given the joints themselves as the reference, the nearest solution must be those joints.
        closest = robot.ik_closest(target, joints)
        assert closest is not None
        assert _same_joints(closest.joints, joints)

        # A small perturbation must not change which solution is nearest.
        nudged = joints + rng.uniform(-0.5, 0.5, size=6)
        closest = robot.ik_closest(target, nudged)
        assert closest is not None
        assert _same_joints(closest.joints, joints)


def test_a_target_may_be_any_object_offering_as_numpy(robot: Crx):
    """
    The `as_numpy()` protocol is how a pose type from another library is accepted without this
    package depending on it. Any object implementing it works, which this stand-in demonstrates.
    """

    class Pose:
        def __init__(self, matrix):
            self._matrix = matrix

        def as_numpy(self):
            return self._matrix

    joints = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
    target = robot.fk(joints)

    assert numpy.allclose(robot.ik(Pose(target)), robot.ik(target))


@pytest.mark.parametrize(
    "bad",
    [
        numpy.eye(3),
        numpy.zeros((4, 3)),
        numpy.zeros(16),
    ],
)
def test_a_target_which_is_not_a_4x4_matrix_is_rejected(robot: Crx, bad):
    with pytest.raises(ValueError):
        robot.ik(bad)


def test_a_target_which_is_not_a_rigid_transformation_is_rejected(robot: Crx):
    scaled = numpy.eye(4) * 2.0
    scaled[3, 3] = 1.0

    with pytest.raises(ValueError):
        robot.ik(scaled)


@pytest.mark.parametrize("bad", [[1.0, 2.0, 3.0], [0.0] * 7, []])
def test_a_joint_vector_of_the_wrong_length_is_rejected(robot: Crx, bad):
    with pytest.raises(ValueError):
        robot.fk(bad)


# The link meshes, which are embedded in the library for two of the six models.

MESH_MODELS = [CrxModel.Crx5iA, CrxModel.Crx10iA]
"""The models with embedded geometry."""

MESHLESS_MODELS = [CrxModel.Crx3iA, CrxModel.Crx10iAL, CrxModel.Crx20iAL, CrxModel.Crx30iA]
"""The models without embedded geometry."""


@pytest.mark.parametrize("model", MESH_MODELS)
def test_link_meshes_have_the_documented_shapes_and_dtypes(model: CrxModel):
    """
    A mesh library reads the two arrays directly, so their data types are part of the interface.
    For example, `engeom.geom3.Mesh3` rejects face arrays with a signed data type.
    """
    meshes = LinkMeshes.load(model)
    assert LinkMeshes.is_available(model)
    assert len(meshes) == 7
    assert len(meshes.links) == 7

    for link in meshes.links:
        assert link.vertices.dtype == numpy.float64
        assert link.vertices.ndim == 2 and link.vertices.shape[1] == 3
        assert link.faces.dtype == numpy.uint32
        assert link.faces.ndim == 2 and link.faces.shape[1] == 3

        assert link.vertex_count == len(link.vertices)
        assert link.face_count == len(link.faces)
        assert link.vertex_count > 0 and link.face_count > 0

        # Every triangle must index a vertex which exists.
        assert link.faces.max() < link.vertex_count


@pytest.mark.parametrize("model", MESHLESS_MODELS)
def test_a_model_without_geometry_raises(model: CrxModel):
    assert not LinkMeshes.is_available(model)
    with pytest.raises(ValueError):
        LinkMeshes.load(model)


def test_the_arrays_are_copies_rather_than_views():
    """Writing into a returned array must not change the mesh it came from."""
    link = LinkMeshes.load(CrxModel.Crx5iA).links[0]
    before = link.vertices.copy()

    scribbled = link.vertices
    scribbled[:] = 0.0

    assert numpy.array_equal(link.vertices, before)


def test_an_index_past_the_last_mesh_is_rejected():
    meshes = LinkMeshes.load(CrxModel.Crx5iA)
    assert meshes[6].vertex_count == meshes.links[6].vertex_count
    with pytest.raises(IndexError):
        meshes[7]


def test_the_poses_are_the_identity_followed_by_the_kinematic_frames():
    robot = Crx.crx10ia()
    joints = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
    poses = LinkMeshes.load(CrxModel.Crx10iA).poses(robot, joints)

    assert poses.shape == (7, 4, 4)
    assert numpy.allclose(poses[0], numpy.eye(4))
    assert numpy.allclose(poses[1:], robot.fk_all(joints))


def test_posed_meshes_follow_the_forward_kinematics():
    """The flange mesh must move with the pose the controller reports."""
    robot = Crx.crx10ia()
    joints = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
    meshes = LinkMeshes.load(CrxModel.Crx10iA)
    posed = meshes.posed(robot, joints)

    # The base does not move with the joints.
    assert numpy.allclose(posed[0].vertices, meshes.links[0].vertices)

    flange = robot.fk(joints)
    expected = meshes.links[6].vertices @ flange[:3, :3].T + flange[:3, 3]
    assert numpy.allclose(posed[6].vertices, expected, atol=1e-9)
    assert numpy.array_equal(posed[6].faces, meshes.links[6].faces)


def test_a_mesh_transform_accepts_any_object_offering_as_numpy():
    class Pose:
        def __init__(self, matrix):
            self._matrix = matrix

        def as_numpy(self):
            return self._matrix

    link = LinkMeshes.load(CrxModel.Crx5iA).links[6]
    transform = Crx.crx5ia().fk([10.0, -20.0, 30.0, -40.0, 50.0, -60.0])

    assert numpy.allclose(
        link.transformed(Pose(transform)).vertices, link.transformed(transform).vertices
    )


@pytest.mark.parametrize("bad", [[1.0, 2.0, 3.0], [0.0] * 7, []])
def test_posing_rejects_a_joint_vector_of_the_wrong_length(bad):
    meshes = LinkMeshes.load(CrxModel.Crx5iA)
    robot = Crx.crx5ia()

    with pytest.raises(ValueError):
        meshes.posed(robot, bad)
    with pytest.raises(ValueError):
        meshes.poses(robot, bad)


@pytest.mark.parametrize(
    "model,named",
    [
        (CrxModel.Crx3iA, Crx.crx3ia),
        (CrxModel.Crx5iA, Crx.crx5ia),
        (CrxModel.Crx10iA, Crx.crx10ia),
        (CrxModel.Crx10iAL, Crx.crx10ial),
        (CrxModel.Crx20iAL, Crx.crx20ial),
        (CrxModel.Crx30iA, Crx.crx30ia),
    ],
)
def test_building_a_robot_from_a_model_matches_the_named_constructor(model: CrxModel, named):
    built = Crx.from_model(model)
    expected = named()

    assert (built.z1, built.x1, built.x2, built.y1) == (
        expected.z1,
        expected.x1,
        expected.x2,
        expected.y1,
    )
