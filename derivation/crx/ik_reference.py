"""
Pure NumPy reference implementation of the CRX inverse kinematics method.

This module provides a readable description of the method and the reference results used to check
the Rust implementation. It deliberately depends only on ``numpy``. Its function boundaries match
the modules planned for the Rust crate:

===============================  ==========================================
This module                      Rust module
===============================  ==========================================
``CrxParams``, ``fk``/``fk_all`` ``lib.rs``
``Setup``                        ``ik/setup.rs``
``dft_coefficients``, ``eval_f`` ``ik/trig_poly.rs``
``theta_roots``                  ``ik/roots.rs``
``o3_branch``, ``polish_branch`` ``ik/geometry.rs``
``joints_from_points``           ``ik/joints.rs``
``ik``                           ``ik/mod.rs``
===============================  ==========================================

The method
----------

The target pose fixes the flange origin ``O6`` and, with it, ``O5``. The latter is a pure offset
back along the flange axis by ``x2`` and is unaffected by J6. The origin ``O1`` is the world
origin. Therefore, the inverse kinematics problem consists only of locating ``O3`` and ``O4``.

Rotating J6 moves ``O4`` around a circle of radius ``y1`` centered at ``O5`` in the flange plane.
Call the angle on that circle ``theta``. Once ``theta`` is fixed, three of the robot's geometric
constraints are *linear* in ``O3``:

* Combining ``|O3 - O4| = x1`` with ``|O3| = z1`` gives
  ``O3 . O4 = (z1^2 + |O4|^2 - x1^2) / 2``.
* Because link 4 and link 5 meet at a right angle, ``O3 . u = O4 . u``, where
  ``u = (O4 - O5) / y1``.
* Because ``O1``, ``O3``, and ``O4`` share a vertical plane, ``O3 . w = 0``, where
  ``w = z_hat x O4``.

By Cramer's rule, these three linear equations in three unknowns give
``O3 = N(theta) / det(theta)``. The one remaining constraint, ``|O3| = z1``, becomes a single
scalar equation:

    f(theta) = |N(theta)|^2 - z1^2 det(theta)^2 = 0.

``f`` is a trigonometric polynomial of degree exactly 4, so it has at most 8 roots. Each root
yields two joint solutions through the J1 front/back flip, giving the familiar bound of 16.

Because the degree is known, 16 evenly spaced samples recover the nine Fourier coefficients of
``f`` exactly. A polynomial root solve then finds the roots without a search.

The method reconstructs ``O3`` geometrically as the intersection of the sphere of radius ``z1``
about ``O1`` and the sphere of radius ``x1`` about ``O4``, restricted to the vertical plane through
``O4``. This approach avoids evaluating ``O3`` as ``N / det``, an expression that is 0/0 in
configurations where two solutions merge. The intersection has two branches. Each branch is
polished on its own residual, which stays simple where ``f`` has a double root.

Degenerate configurations
-------------------------

Four geometries need their own treatment. All of them are ordinary poses that a real robot reaches.

* ``O4`` lands on the J1 axis. The vertical plane through ``O1`` and ``O4`` is then undefined, the
  plane constraint provides no information, and ``f`` has a double root. The solver finds these
  angles directly by determining where the circle of ``O4`` crosses the axis. It then reconstructs
  ``O3`` as the intersection of its horizontal circle and the plane perpendicular to ``u``. The
  pose reported in issue #1 is an example:
  on a CRX-10iA at ``[10, -80, 10, 20, -20, 45]``, ``O4`` sits exactly on the axis.
* ``O3`` also lands on the J1 axis. J1 is then free, producing a continuum of joint solutions. The
  solver reports and marks one representative.
* The arm is fully stretched, with ``O1``, ``O3``, and ``O4`` collinear. The two spheres that locate
  ``O3`` are tangent, so the radius of their intersection circle is zero and rounding can carry it
  below zero. The solver clamps the radius and evaluates the solution by its pose error.
* ``O4`` lands on ``O1``. On a robot built with ``z1`` equal to ``x1``, which is the CRX-3iA and the
  CRX-10iA, the two spheres become concentric and both J1 and J2 lose their effect. The solver
  reports and marks one representative.

**Known limitations.** For each degenerate geometry above, the solver treats a nearby configuration
as the corresponding mathematical case because the polish absorbs the small resulting error. Two
configurations remain unrecoverable because the plane used to slice the circle of candidate ``O3``
points does not intersect that circle transversally.

The circle collapses when ``O4`` sits on the axis at a distance ``|z1 - x1|`` from the origin,
where the two spheres are tangent. The arm is folded back on itself with the wrist center on the
base axis, and the collapsed circle leaves no points to slice.

The circle remains large while its plane becomes unstable when ``O4`` comes within a fraction of a
millimeter of the origin. This condition requires ``z1`` equal to ``x1`` and therefore occurs only
on the CRX-3iA and the CRX-10iA. The direction of ``O4`` determines the plane containing the
circle, and that direction is weakly determined near the origin. Consequently, an error in ``O4``
far below a micron moves ``O3`` by hundreds of millimeters.

In both cases, the solver returns solutions that reach the requested pose without recovering the
particular configuration that produced it. A real arm cannot reach either configuration because
both place the wrist center inside the robot's base casting. The tests deliberately exclude them.

The solver finishes every solution with a few Gauss-Newton steps in joint space against the target
pose.
Where two solutions have nearly merged, the angle on the ``O4`` circle is poorly determined even
though the joint angles are not. This last step carries those solutions from about a tenth of a
micron to the limit of double precision. It also lets acceptance be decided on the pose error
itself, which directly measures the requested result. Earlier calculations use a residual as a
proxy for the pose error.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import numpy

# ---------------------------------------------------------------------------------------------
# Robot model and forward kinematics
# ---------------------------------------------------------------------------------------------

_TWO_PI = 2.0 * math.pi


def _rotation(axis, angle: float) -> numpy.ndarray:
    """Return the rotation matrix for `angle` radians about `axis`, using Rodrigues' formula."""
    axis = numpy.asarray(axis, dtype=float)
    axis = axis / numpy.linalg.norm(axis)
    k = numpy.array([[0.0, -axis[2], axis[1]],
                     [axis[2], 0.0, -axis[0]],
                     [-axis[1], axis[0], 0.0]])
    return numpy.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def _matrix(rot: numpy.ndarray, trans=(0.0, 0.0, 0.0)) -> numpy.ndarray:
    """Return a 4x4 homogeneous transform from a rotation matrix and a translation."""
    m = numpy.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = trans
    return m


def _end_adjust() -> numpy.ndarray:
    """
    Return the fixed reorientation at the end of the chain that converts the world-axis convention
    to the FANUC flange convention: Z points out of the flange, X points up, and Y points opposite
    world Y.
    """
    return _matrix(_rotation([1.0, 0.0, 1.0], math.pi))


_END_ADJUST = _end_adjust()


def wrap_pi(angle: float) -> float:
    """Wrap an angle in radians into [-pi, pi), where pi wraps to the included bound at -pi."""
    return (angle + math.pi) % _TWO_PI - math.pi


@dataclass(frozen=True)
class CrxParams:
    """
    The four link lengths distinguish one CRX model from another. Every robot in the family shares
    the same kinematic layout, so these four numbers define the complete model.

    :param z1: distance from the J2 axis to the J3 axis
    :param x1: distance from the J3 axis to the J5 axis
    :param x2: distance from the J5 axis to the flange
    :param y1: offset between the J1 and J6 axes
    """

    z1: float
    x1: float
    x2: float
    y1: float

    @staticmethod
    def crx3ia() -> "CrxParams":
        return CrxParams(280.0, 280.0, 123.0, 111.0)

    @staticmethod
    def crx5ia() -> "CrxParams":
        return CrxParams(410.0, 430.0, 145.0, 130.0)

    @staticmethod
    def crx10ia() -> "CrxParams":
        return CrxParams(540.0, 540.0, 160.0, 150.0)

    @staticmethod
    def crx10ial() -> "CrxParams":
        return CrxParams(710.0, 540.0, 160.0, 150.0)

    @staticmethod
    def crx20ial() -> "CrxParams":
        return CrxParams(710.0, 540.0, 160.0, 150.0)

    @staticmethod
    def crx30ia() -> "CrxParams":
        return CrxParams(950.0, 750.0, 180.0, 185.0)


ALL_MODELS = {
    "crx3ia": CrxParams.crx3ia(),
    "crx5ia": CrxParams.crx5ia(),
    "crx10ia": CrxParams.crx10ia(),
    "crx10ial": CrxParams.crx10ial(),
    "crx20ial": CrxParams.crx20ial(),
    "crx30ia": CrxParams.crx30ia(),
}


def joints_to_radians(joints) -> numpy.ndarray:
    """
    Convert joint angles from FANUC controller degrees to kinematic radians.

    Apply the J2/J3 coupling: on a FANUC arm, driving J2 carries J3 with it. Therefore, the
    kinematic J3 rotation is the sum of the two controller values.
    """
    rad = numpy.radians(numpy.asarray(joints, dtype=float))
    rad[2] += rad[1]
    return rad


def radians_to_joints(radians) -> numpy.ndarray:
    """Apply the inverse of :func:`joints_to_radians` and return FANUC controller degrees."""
    deg = numpy.degrees(numpy.asarray(radians, dtype=float))
    deg[2] -= deg[1]
    return deg


def fk_all(params: CrxParams, joints) -> list[numpy.ndarray]:
    """
    Compute forward kinematics and return each kinematic link-origin frame as a 4x4 matrix.

    :param params: the robot model
    :param joints: six joint angles in FANUC controller degrees
    :return: six 4x4 matrices; the last one is the flange pose and matches the controller
    """
    j = joints_to_radians(joints)
    f1 = _matrix(_rotation([0.0, 0.0, 1.0], j[0]))
    f2 = f1 @ _matrix(_rotation([0.0, 1.0, 0.0], j[1]))
    f3 = f2 @ _matrix(_rotation([0.0, -1.0, 0.0], j[2]), [0.0, 0.0, params.z1])
    f4 = f3 @ _matrix(_rotation([-1.0, 0.0, 0.0], j[3]))
    f5 = f4 @ _matrix(_rotation([0.0, -1.0, 0.0], j[4]), [params.x1, -params.y1, 0.0])
    f6 = f5 @ _matrix(_rotation([-1.0, 0.0, 0.0], j[5]), [params.x2, 0.0, 0.0]) @ _END_ADJUST
    return [f1, f2, f3, f4, f5, f6]


def fk(params: CrxParams, joints) -> numpy.ndarray:
    """Compute forward kinematics and return only the 4x4 flange pose."""
    return fk_all(params, joints)[5]


def pose_error(params: CrxParams, joints, target: numpy.ndarray) -> float:
    """Return the largest absolute element difference between the pose of `joints` and `target`."""
    return float(numpy.abs(fk(params, joints) - target).max())


def _solve_linear_trig(a: float, b: float, c: float,
                       tol: float = 1e-12) -> list[float] | None:
    """
    Every angle satisfying ``a cos(x) + b sin(x) = c``.

    :return: the solutions, or None when the equation is satisfied by every angle because both
        coefficients and the right hand side all vanish
    """
    radius = math.hypot(a, b)
    if radius < tol:
        return None if abs(c) < tol else []
    ratio = c / radius
    if abs(ratio) > 1.0 + 1e-12:
        return []
    base = math.atan2(b, a)
    offset = math.acos(max(-1.0, min(1.0, ratio)))
    if offset < tol:
        return [wrap_pi(base)]
    return [wrap_pi(base + offset), wrap_pi(base - offset)]


# ---------------------------------------------------------------------------------------------
# Per-target setup
# ---------------------------------------------------------------------------------------------

AXIS_TOL = 1e-6
"""
How close to the J1 axis, as a fraction of ``z1``, counts as being on it.

A point nearer than this is treated as exactly on the axis. The joint-space polish at the end
absorbs the resulting error. This tolerance is therefore set generously according to the limit of
the branch construction. The required result accuracy does not determine it.
"""


class Setup:
    """
    Values derived from the target pose that do not depend on `theta`.

    ``o5`` is the origin of the fifth frame. It is reached from the flange by backing off ``x2``
    along the flange Z axis. ``ax`` and ``ay`` are the flange X and Y axes. These axes span the
    plane of the circle that ``O4`` travels on.
    """

    def __init__(self, params: CrxParams, target: numpy.ndarray):
        self.params = params
        self.target = numpy.asarray(target, dtype=float)
        self.o5 = self.target[:3, 3] - self.target[:3, 2] * params.x2
        self.o6 = self.target[:3, 3]
        self.ax = self.target[:3, 0]
        self.ay = self.target[:3, 1]

    def o4_and_axis(self, theta: float) -> tuple[numpy.ndarray, numpy.ndarray]:
        """
        Return the candidate ``O4`` at angle `theta` on its circle and the unit vector ``u`` along
        the J5 axis from ``O5`` to ``O4``.
        """
        u = math.cos(theta) * self.ax + math.sin(theta) * self.ay
        return self.o5 + self.params.y1 * u, u

    def f(self, theta: float) -> float:
        """
        Return the scalar constraint whose roots are the solutions.

        Lengths are scaled by ``1 / z1`` before the products are formed. This keeps the magnitudes
        near unity and makes the quantities being subtracted comparable in size.
        """
        scale = 1.0 / self.params.z1
        u = math.cos(theta) * self.ax + math.sin(theta) * self.ay
        o4 = (self.o5 + self.params.y1 * u) * scale
        w = numpy.cross([0.0, 0.0, 1.0], o4)

        rhs_dot = (1.0 + o4 @ o4 - (self.params.x1 * scale) ** 2) / 2.0
        rhs_perp = u @ o4
        numerator = rhs_dot * numpy.cross(u, w) + rhs_perp * numpy.cross(w, o4)
        determinant = o4 @ numpy.cross(u, w)
        return float(numerator @ numerator - determinant ** 2)

    def circle_of_o3(self, o4: numpy.ndarray) -> tuple[float, float, float]:
        """
        Return the circle on which ``O3`` must lie. This circle is the intersection of the sphere
        of radius ``z1`` about the origin and the sphere of radius ``x1`` about `o4`.

        :return: the distance to `o4`, the distance from the origin to the circle's plane along
            that direction, and the *square* of the circle's radius (which may be negative when
            the spheres do not reach each other)
        """
        d = float(numpy.linalg.norm(o4))
        if d < AXIS_TOL * self.params.z1:
            # O4 has landed on O1. The two spheres are then concentric, so they either coincide or
            # miss entirely, and neither is a circle. See `origin_thetas`.
            return d, 0.0, -1.0
        along = (self.params.z1 ** 2 - self.params.x1 ** 2 + d ** 2) / (2.0 * d)
        return d, along, self.params.z1 ** 2 - along ** 2

    def o3_branch(self, o4: numpy.ndarray, branch: int,
                  clamp: float = 0.0) -> numpy.ndarray | None:
        """
        Return one of the two candidate ``O3`` points for a given ``O4``. The point is in the
        vertical plane that contains ``O1`` and `o4`.

        :param branch: +1 or -1, selecting one of the two intersection points
        :param clamp: how far the squared radius of the intersection circle may fall below zero,
            as a fraction of ``z1**2``, before the spheres are considered unable to reach each other
        :return: the point, or None when `o4` is on the J1 axis (no unique vertical plane) or the
            two spheres do not reach each other
        """
        rho = math.hypot(o4[0], o4[1])
        if rho < AXIS_TOL * self.params.z1:
            return None
        radial = numpy.array([o4[0] / rho, o4[1] / rho, 0.0])
        d, along, radius_sq = self.circle_of_o3(o4)
        if radius_sq < -clamp * self.params.z1 ** 2:
            return None
        # A tangency is a reachable configuration: the fully stretched arm. Clamp a radius that
        # rounds just below zero to preserve this configuration.
        radius = math.sqrt(max(radius_sq, 0.0))
        # In the (radial, z) plane of the vertical section, `ex` points at o4 and `ey` is square
        # to it, so the two intersection points are `along` out and `radius` to either side.
        ex = numpy.array([rho, o4[2]]) / d
        ey = numpy.array([-ex[1], ex[0]])
        q = along * ex + branch * radius * ey
        return q[0] * radial + numpy.array([0.0, 0.0, q[1]])

    def axis_distance(self, theta: float) -> tuple[float, float, float]:
        """
        Return the distance from ``O4`` to the J1 axis at `theta` and the first two derivatives of
        the squared distance. The search below uses the squared-distance derivatives.

        The shadow of the ``O4`` circle on the floor is an ellipse, so the squared distance is a
        trigonometric polynomial of degree two and has at most two minima.
        """
        y1 = self.params.y1
        a = self.ax[:2]
        b = self.ay[:2]
        cos = math.cos(theta)
        sin = math.sin(theta)
        q = self.o5[:2] + y1 * (cos * a + sin * b)
        dq = y1 * (-sin * a + cos * b)
        ddq = -y1 * (cos * a + sin * b)
        return float(numpy.linalg.norm(q)), float(2.0 * q @ dq), float(2.0 * (dq @ dq + q @ ddq))

    def axis_thetas(self, tol: float = AXIS_TOL, samples: int = 16) -> list[float]:
        """
        Return the angles at which ``O4`` is on the J1 axis or near enough to be treated as on it.

        The test measures distance from the axis and includes near crossings. A pose that misses
        the axis by a nanometer causes the same difficulty for the branch construction as a pose
        that hits it. Near the axis, the direction of the vertical plane swings through a wide arc
        while ``O4`` moves a short distance. The solver handles both cases by putting ``O4`` on the
        axis, constructing ``O3`` there, and using the final joint-space polish to absorb the small
        ignored distance.

        :param tol: how near the axis counts as on it, as a fraction of ``z1``
        :param samples: starting points for the search, which only has to separate two minima
        :return: the qualifying angles
        """
        found: list[float] = []
        for start in _TWO_PI * numpy.arange(samples) / samples:
            theta = float(start)
            for _ in range(20):
                _, slope, curvature = self.axis_distance(theta)
                if curvature == 0.0:
                    break
                step = -slope / curvature
                theta += step
                if abs(step) < 1e-15:
                    break
            distance, _, curvature = self.axis_distance(theta)
            if curvature <= 0.0 or distance > tol * self.params.z1:
                continue
            theta = wrap_pi(theta)
            if not any(abs(wrap_pi(theta - seen)) < 1e-9 for seen in found):
                found.append(theta)
        return found

    def is_on_origin(self, theta: float) -> bool:
        """
        Return whether ``O4`` has landed on ``O1`` at `theta`.

        This occurs on models with equal ``z1`` and ``x1``. The two spheres that locate ``O3`` then
        become concentric, leaving both J1 and J2 free.
        """
        o4, _ = self.o4_and_axis(theta)
        return float(numpy.linalg.norm(o4)) < AXIS_TOL * self.params.z1

    def o3_on_origin(self, theta: float) -> list[numpy.ndarray]:
        """
        Return a representative ``O3`` for the case where ``O4`` is on ``O1``.

        The two spheres coincide when ``z1`` equals ``x1`` and miss each other otherwise. Where
        they coincide, ``O3`` may be anywhere on the sphere that is also perpendicular to ``u``.
        This produces a circle of solutions, and one point represents the full circle.
        """
        if abs(self.params.z1 - self.params.x1) > AXIS_TOL * self.params.z1:
            return []
        _, u = self.o4_and_axis(theta)
        # Any direction perpendicular to u is valid. Select the larger of two candidates to avoid
        # the case where u points along the axis used to construct the direction.
        options = [numpy.cross(u, [0.0, 0.0, 1.0]), numpy.cross(u, [1.0, 0.0, 0.0])]
        direction = max(options, key=numpy.linalg.norm)
        return [self.params.z1 * direction / numpy.linalg.norm(direction)]

    def o3_on_axis(self, theta: float) -> list[numpy.ndarray]:
        """
        Return the candidate ``O3`` points when ``O4`` is on the J1 axis.

        With no effective plane constraint, ``O3`` can be anywhere on the horizontal circle formed
        by the intersection of the two spheres. The perpendicularity constraint intersects that
        circle with a plane, leaving two points or one point when the plane is tangent.

        :return: the points, which is empty when the plane misses the circle, and also empty when
            ``O3`` is on the axis too and J1 is free (see :meth:`is_singular_family`)
        """
        o4, u = self.o4_and_axis(theta)
        height = o4[2]
        on_axis = numpy.array([0.0, 0.0, height])
        _, along, radius_sq = self.circle_of_o3(on_axis)
        radius = math.sqrt(max(radius_sq, 0.0))
        center_z = along * numpy.sign(height)

        # Points on the circle satisfy (O3 - O4) . u = 0, which in the circle's own angle is
        # another `A cos + B sin = C`.
        solutions = _solve_linear_trig(radius * u[0], radius * u[1], (height - center_z) * u[2])
        if solutions is None:
            return []  # every point on the circle qualifies: the J1-free family
        return [numpy.array([radius * math.cos(a), radius * math.sin(a), center_z])
                for a in solutions]

    def is_singular_family(self, theta: float) -> bool:
        """
        Return whether both ``O3`` and ``O4`` are on the J1 axis at `theta`. This configuration
        leaves J1 free and produces a continuous solution set.
        """
        o4, u = self.o4_and_axis(theta)
        on_axis = numpy.array([0.0, 0.0, o4[2]])
        _, along, radius_sq = self.circle_of_o3(on_axis)
        radius = math.sqrt(max(radius_sq, 0.0))
        return radius < AXIS_TOL * self.params.z1

    def branch_residual(self, theta: float, branch: int) -> float | None:
        """
        Return the perpendicularity residual for one branch, ``(O3 - O4) . u / x1``.

        The residual is zero exactly at a solution and passes through zero transversally where two
        solutions merge. By contrast, ``f`` has a double root at the merge. This behavior makes
        the branch residual the correct value to polish.
        """
        o4, u = self.o4_and_axis(theta)
        o3 = self.o3_branch(o4, branch)
        if o3 is None:
            return None
        return float((o3 - o4) @ u / self.params.x1)


# ---------------------------------------------------------------------------------------------
# The trigonometric polynomial and its roots
# ---------------------------------------------------------------------------------------------

TRIG_DEGREE = 4
"""The degree of ``f`` as a trigonometric polynomial. See the module docstring."""

_MAX_NEWTON_STEP = 0.05
"""Largest angle, in radians, that a single Newton step on ``f`` is allowed to move."""

_SAMPLE_COUNT = 4 * TRIG_DEGREE
"""Samples needed to recover the coefficients exactly: two per coefficient, plus Nyquist room."""


def dft_coefficients(setup: Setup, count: int = _SAMPLE_COUNT) -> numpy.ndarray:
    """
    Return the complex Fourier coefficients ``c_k`` of ``f``, indexed so that ``c[k]`` and ``c[-k]``
    are the coefficients of ``exp(+i k theta)`` and ``exp(-i k theta)``.

    Because the degree is known and finite, sampling at `count` evenly spaced angles recovers the
    coefficients exactly. Afterwards, ``f`` and its derivative can be evaluated without using the
    geometry again. An approximate recovery is unnecessary.
    """
    thetas = _TWO_PI * numpy.arange(count) / count
    values = numpy.array([setup.f(t) for t in thetas])
    return numpy.fft.fft(values) / count


def eval_f(coefficients: numpy.ndarray, theta: float) -> tuple[float, float]:
    """Evaluate ``f`` and its derivative at `theta` from the Fourier coefficients."""
    value = 0.0 + 0.0j
    derivative = 0.0 + 0.0j
    for k in range(-TRIG_DEGREE, TRIG_DEGREE + 1):
        c = coefficients[k % len(coefficients)]
        e = numpy.exp(1j * k * theta)
        value += c * e
        derivative += 1j * k * c * e
    return float(value.real), float(derivative.real)


def _shifted_coefficients(coefficients: numpy.ndarray, shift: float) -> numpy.ndarray:
    """Coefficients of ``f(phi + shift)`` as a function of ``phi``."""
    n = len(coefficients)
    out = numpy.zeros(n, dtype=complex)
    for k in range(n):
        signed = k if k <= n // 2 else k - n
        out[k] = coefficients[k] * numpy.exp(1j * signed * shift)
    return out


def _choose_shift(coefficients: numpy.ndarray, count: int = 64) -> float:
    """
    Choose an origin shift that puts the half-angle substitution's point at infinity, ``phi = pi``,
    where ``f`` has its largest magnitude. This prevents a root from being pushed to infinity in
    ``t``.
    """
    thetas = _TWO_PI * numpy.arange(count) / count
    values = [abs(eval_f(coefficients, t)[0]) for t in thetas]
    return float(thetas[int(numpy.argmax(values))] - math.pi)


def half_angle_polynomial(coefficients: numpy.ndarray) -> numpy.ndarray:
    """
    Rewrite the trigonometric polynomial as an ordinary real polynomial in ``t = tan(theta / 2)``.

    Substituting ``exp(i theta) = (1 + i t)^2 / (1 + t^2)`` and clearing the denominator with a
    factor of ``(1 + t^2)^degree`` gives a real polynomial of degree ``2 * degree`` whose real
    roots are exactly the roots of ``f`` other than ``theta = pi``.

    :return: coefficients in ascending powers of ``t``
    """
    poly = numpy.polynomial.polynomial
    result = numpy.zeros(2 * TRIG_DEGREE + 1)
    one_plus_it = numpy.array([1.0, 1.0j])
    one_plus_t2 = numpy.array([1.0, 0.0, 1.0])
    for k in range(TRIG_DEGREE + 1):
        term = poly.polypow(one_plus_it, 2 * k) if k > 0 else numpy.array([1.0 + 0.0j])
        term = poly.polymul(term, poly.polypow(one_plus_t2, TRIG_DEGREE - k))
        term = numpy.pad(term, (0, 2 * TRIG_DEGREE + 1 - len(term)))
        # The k and -k terms are conjugates of one another, so summing them is twice the real part.
        weight = 2.0 if k > 0 else 1.0
        result += weight * (coefficients[k] * term).real
    return result


def theta_roots(coefficients: numpy.ndarray, imaginary_tol: float = 1e-2) -> list[float]:
    """
    Return every real root of ``f`` by solving the half-angle polynomial.

    The tolerance on the imaginary part is deliberately loose. Where two solutions merge, ``f``
    has a double root. In floating-point arithmetic, a double root splits into a conjugate pair
    whose imaginary part is near the square root of the rounding error. The final pose check
    removes admitted candidates that are not solutions. A strict imaginary-part test would discard
    merged configurations.

    A few Newton steps on ``f`` polish the roots and remove the conditioning cost of the
    substitution. Duplicate roots are merged: a merged pair arrives as two angles a hair apart, and
    one root is sufficient because the following per-branch polish separates the two solutions it
    represents.
    """
    shift = _choose_shift(coefficients)
    shifted = _shifted_coefficients(coefficients, shift)
    ascending = half_angle_polynomial(shifted)

    companion_input = ascending[::-1]  # numpy.roots wants descending powers
    raw = numpy.roots(companion_input) if numpy.any(companion_input) else []

    thetas = []
    for root in raw:
        scale = max(1.0, abs(root.real))
        if abs(root.imag) > imaginary_tol * scale:
            continue
        thetas.append(wrap_pi(2.0 * math.atan(root.real) + shift))

    polished = []
    for theta in thetas:
        for _ in range(20):
            value, derivative = eval_f(coefficients, theta)
            if derivative == 0.0:
                break
            step = -value / derivative
            # At a double root, the value and slope approach zero together, so rounding error
            # controls their ratio. Two guards preserve the angle: reject steps longer than the
            # cap and undo steps that do not reduce the magnitude of ``f``. These guards prevent a
            # valid polynomial root from moving onto its neighbor and being lost when duplicate
            # roots are merged below.
            if abs(step) > _MAX_NEWTON_STEP:
                break
            moved, _ = eval_f(coefficients, theta + step)
            if abs(moved) >= abs(value):
                break
            theta += step
            if abs(step) < 1e-15:
                break
        theta = wrap_pi(theta)
        if not any(abs(wrap_pi(theta - seen)) < 1e-9 for seen in polished):
            polished.append(theta)
    return polished


# ---------------------------------------------------------------------------------------------
# Branch polishing
# ---------------------------------------------------------------------------------------------


def polish_branch(setup: Setup, theta: float, branch: int, max_steps: int = 30,
                  max_step_size: float = 0.05) -> float | None:
    """
    Refine `theta` with Newton's method so that one branch's perpendicularity residual is zero.
    Use a numeric derivative and a step limit.

    A root of ``f`` is a root of one or both branch residuals. Starting from that root, this method
    converges to the branch that owns the solution. It moves away from any branch that does not own
    the solution.

    :return: the refined angle, or None if the branch does not exist near `theta`
    """
    for _ in range(max_steps):
        value = setup.branch_residual(theta, branch)
        if value is None:
            return None
        nudge = 1e-7
        ahead = setup.branch_residual(theta + nudge, branch)
        if ahead is None:
            return None
        derivative = (ahead - value) / nudge
        if derivative == 0.0:
            break
        step = -value / derivative
        step = max(-max_step_size, min(max_step_size, step))
        theta += step
        if abs(step) < 1e-15:
            break
    return theta


def polish_joints(params: CrxParams, joints, target: numpy.ndarray,
                  max_steps: int = 25, tol: float = 1e-13) -> numpy.ndarray:
    """
    Refine a joint vector directly against the target pose by Gauss-Newton, with the Jacobian taken
    by finite difference of the forward kinematics.

    The twelve residuals are the elements of the rotation and translation blocks. At a singular
    configuration, the Jacobian is rank deficient. The least-squares calculation uses the smallest
    fitting step, which keeps the solution in place and prevents movement along the free direction.

    :param joints: the starting joint vector, in FANUC controller degrees
    :param target: the desired 4x4 flange pose
    :param max_steps: how many iterations to take at most, which most solutions never approach
    :param tol: residual below which the vector is already as good as it will get
    :return: the refined joint vector, in FANUC controller degrees
    """
    joints = numpy.asarray(joints, dtype=float).copy()

    def residuals(vector):
        return (fk(params, vector)[:3, :4] - target[:3, :4]).flatten()

    for _ in range(max_steps):
        current = residuals(joints)
        if numpy.abs(current).max() < tol:
            break
        jacobian = numpy.zeros((12, 6))
        nudge = 1e-6
        for column in range(6):
            shifted = joints.copy()
            shifted[column] += nudge
            jacobian[:, column] = (residuals(shifted) - current) / nudge
        step, *_ = numpy.linalg.lstsq(jacobian, -current, rcond=None)
        moved = joints + step
        # Near a merged pair, the step is long and improvement is slow, so the loop can run for
        # several iterations. A step that stops improving the result indicates that the
        # finite-difference Jacobian has reached its noise floor. Further iterations would only
        # move the answer.
        if numpy.abs(residuals(moved)).max() >= numpy.abs(current).max():
            break
        joints = moved
    return joints


# ---------------------------------------------------------------------------------------------
# Recovering joint angles from the O3 / O4 pair
# ---------------------------------------------------------------------------------------------


def joints_from_points(setup: Setup, o3: numpy.ndarray, o4: numpy.ndarray) -> list[numpy.ndarray]:
    """
    Work forward along the chain from a located ``O3`` and ``O4``. Read each joint angle from the
    position of the next origin in the frame built so far.

    Return both members of the J1 front/back pair. The same flange pose is reached when J1 turns by
    half a turn and the shoulder, elbow, and wrist roll are reflected to match.

    :return: two joint vectors in FANUC controller degrees
    """
    params = setup.params
    j = [0.0] * 6

    # J1 turns the arm into the vertical plane that contains O3 and O4, so either point determines
    # the angle. Use O4 when it is off the axis. If O4 lands on the axis and loses its azimuth, O3
    # determines the angle. J1 is free when both points are on the axis, so any value is valid.
    on_axis = AXIS_TOL * params.z1
    if math.hypot(o4[0], o4[1]) > on_axis:
        j[0] = math.atan2(o4[1], o4[0])
    elif math.hypot(o3[0], o3[1]) > on_axis:
        j[0] = math.atan2(o3[1], o3[0])
    else:
        j[0] = 0.0
    f1 = _matrix(_rotation([0.0, 0.0, 1.0], j[0]))

    local_o3 = (numpy.linalg.inv(f1) @ numpy.append(o3, 1.0))[:3]
    j[1] = math.atan2(local_o3[0], local_o3[2])
    f3 = f1 @ _matrix(numpy.eye(3), local_o3) @ _matrix(_rotation([0.0, 1.0, 0.0], j[1]))

    local_o4 = (numpy.linalg.inv(f3) @ numpy.append(o4, 1.0))[:3]
    j[2] = math.atan2(local_o4[2], local_o4[0])
    f4 = f3 @ _matrix(numpy.eye(3), local_o4) @ _matrix(_rotation([0.0, -1.0, 0.0], j[2]))

    local_o5 = (numpy.linalg.inv(f4) @ numpy.append(setup.o5, 1.0))[:3]
    j[3] = math.atan2(local_o5[2], -local_o5[1])
    f5 = f4 @ _matrix(numpy.eye(3), local_o5) @ _matrix(_rotation([-1.0, 0.0, 0.0], j[3]))

    local_o6 = (numpy.linalg.inv(f5) @ numpy.append(setup.o6, 1.0))[:3]
    j[4] = math.atan2(local_o6[2], local_o6[0])
    f6 = (f5 @ _matrix(numpy.eye(3), local_o6)
          @ _matrix(_rotation([0.0, -1.0, 0.0], j[4])) @ _END_ADJUST)

    flange_x = (numpy.linalg.inv(f6) @ setup.target @ numpy.array([1.0, 0.0, 0.0, 0.0]))[:3]
    j[5] = -math.atan2(flange_x[1], flange_x[0])

    return [radians_to_joints(v) for v in alternate_pair([wrap_pi(v) for v in j])]


def alternate_pair(radians: list[float]) -> list[numpy.ndarray]:
    """
    Return the J1 front/back pair. Swinging the base half a turn and mirroring J2, J3, and J4 leaves
    the wrist center and the flange in the same positions.
    """
    flipped = list(radians)
    flipped[0] = wrap_pi(radians[0] - math.pi)
    flipped[1] = wrap_pi(-radians[1])
    flipped[2] = wrap_pi(math.pi - radians[2])
    flipped[3] = wrap_pi(radians[3] - math.pi)
    return [numpy.array(radians), numpy.array(flipped)]


# ---------------------------------------------------------------------------------------------
# Top level
# ---------------------------------------------------------------------------------------------


class SolutionKind(Enum):
    """Describe the type of configuration that produced a solution."""

    REGULAR = "regular"
    """``O4`` is off the J1 axis and the solution is isolated."""

    AXIS_DEGENERATE = "axis_degenerate"
    """``O4`` is on the J1 axis. The solution is isolated, and ``f`` has a double root."""

    SINGULAR_FAMILY = "singular_family"
    """``O3`` and ``O4`` are both on the J1 axis, J1 is free, and this is one representative of a
    continuum of solutions."""


@dataclass
class IkSolution:
    """One inverse kinematics solution."""

    joints: numpy.ndarray
    """The six joint angles in FANUC controller degrees."""

    residual: float
    """Largest absolute element difference between this solution's forward pose and the target."""

    kind: SolutionKind
    """What sort of configuration it came from."""

    theta: float
    """The angle on the ``O4`` circle that produced it, kept for diagnostics."""

    branch: int
    """Which ``O3`` branch produced it, or 0 for the on-axis construction. For diagnostics."""


def _candidates(setup: Setup, residual_tol: float):
    """
    Return each ``(O3, O4, theta, branch, kind)`` candidate for conversion to joint angles.

    Candidate generation uses broad criteria because omitted solutions cannot be recovered later.
    The final validation pose check rejects invalid candidates after a few Gauss-Newton steps.
    """
    coefficients = dft_coefficients(setup)
    axis_angles = setup.axis_thetas()

    for theta in axis_angles:
        o4, _ = setup.o4_and_axis(theta)
        if setup.is_on_origin(theta):
            for o3 in setup.o3_on_origin(theta):
                yield o3, o4, theta, 0, SolutionKind.SINGULAR_FAMILY
            continue
        if setup.is_singular_family(theta):
            # O3 is on the axis too, so J1 is free. One representative stands for the family.
            height = o4[2]
            _, along, _ = setup.circle_of_o3(numpy.array([0.0, 0.0, height]))
            o3 = numpy.array([0.0, 0.0, along * numpy.sign(height)])
            yield o3, o4, theta, 0, SolutionKind.SINGULAR_FAMILY
            continue
        for o3 in setup.o3_on_axis(theta):
            yield o3, o4, theta, 0, SolutionKind.AXIS_DEGENERATE

    for theta in theta_roots(coefficients):
        if any(abs(wrap_pi(theta - a)) < 1e-5 for a in axis_angles):
            continue  # already covered by the on-axis construction above
        o4, u = setup.o4_and_axis(theta)
        for branch in (1, -1):
            refined = polish_branch(setup, theta, branch)
            residual = None if refined is None else setup.branch_residual(refined, branch)
            if residual is not None and abs(residual) <= residual_tol:
                polished_o4, _ = setup.o4_and_axis(refined)
                o3 = setup.o3_branch(polished_o4, branch)
                if o3 is not None:
                    yield o3, polished_o4, refined, branch, SolutionKind.REGULAR
                    continue
            # The polish fails where the two spheres are tangent, because the branch residual has
            # no zero crossing to find there. The root of `f` is already the answer in that case.
            o3 = setup.o3_branch(o4, branch, clamp=1e-6)
            if o3 is not None:
                yield o3, o4, theta, branch, SolutionKind.REGULAR


def ik(params: CrxParams, target: numpy.ndarray, pose_tol: float = 1e-8,
       residual_tol: float = 1e-9, duplicate_tol: float = 1e-5) -> list[IkSolution]:
    """
    Return every joint configuration that puts the flange at `target`.

    :param params: the robot model
    :param target: the desired 4x4 flange pose
    :param pose_tol: how near a candidate's forward pose must land, after polishing, to be
        accepted. Real solutions reach the last few digits of double precision, so this is set far
        above what they need and far below what a candidate that is not a solution can reach. The
        gap between the two is what lets candidate generation be generous.
    :param residual_tol: how small a branch's perpendicularity residual must be for its angle to be
        taken as converged
    :param duplicate_tol: joint-space radius, in radians, within which two solutions are the same.
        Wrist configurations that put J5 at zero allow J4 and J6 to offset each other. The polish
        therefore reaches the same configuration from slightly different directions, and this
        tolerance must identify those results as one solution. Distinct configurations within this
        tolerance form a merged pair.
    :return: the solutions, in no particular order
    """
    target = numpy.asarray(target, dtype=float)
    setup = Setup(params, target)
    solutions: list[IkSolution] = []

    for o3, o4, theta, branch, kind in _candidates(setup, residual_tol):
        for joints in joints_from_points(setup, o3, o4):
            joints = polish_joints(params, joints, target)
            residual = pose_error(params, joints, target)
            if residual > pose_tol:
                continue
            if any(joint_distance(joints, s.joints) < duplicate_tol for s in solutions):
                continue
            solutions.append(IkSolution(joints, residual, kind, theta, branch))

    return solutions


def joint_distance(first, second) -> float:
    """
    Largest per-joint difference between two joint vectors, in radians, treating angles a full
    turn apart as the same angle.
    """
    difference = [wrap_pi(a - b)
                  for a, b in zip(joints_to_radians(first), joints_to_radians(second))]
    return float(numpy.abs(difference).max())


def ik_closest(params: CrxParams, target: numpy.ndarray, reference,
               **kwargs) -> IkSolution | None:
    """
    Return the solution whose joints are nearest to `reference`. A caller following a path uses
    this result to avoid reconfiguring the robot during a move.

    :param reference: a joint vector in FANUC controller degrees, typically the current position
    :return: the nearest solution, or None if the target cannot be reached
    """
    solutions = ik(params, target, **kwargs)
    if not solutions:
        return None
    return min(solutions, key=lambda s: joint_distance(s.joints, reference))
