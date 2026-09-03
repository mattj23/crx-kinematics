"""
Type stubs for the native `crx_kinematics` module.

These stubs are maintained by hand and checked against the compiled module by
`tests/test_stub_drift.py`, which fails when either interface contains a name missing from the other.
"""

from __future__ import annotations

from enum import Enum
from typing import Any, Iterable, List, Optional, Union

import numpy
from numpy.typing import NDArray

Transform = Union[NDArray[numpy.float64], Any]
"""A pose given as a 4x4 array or an object with an `as_numpy()` method that returns one, such as
an `engeom.geom3.Iso3`, which is supported without requiring an engeom dependency."""

Joints = Union[NDArray[numpy.float64], Iterable[float]]
"""Six joint angles in degrees, as they appear in the robot controller."""


class SolutionKind(Enum):
    """The configuration category of a solution."""

    Regular = ...
    """The solution is isolated and the arm is in an ordinary configuration."""

    AxisDegenerate = ...
    """The wrist center is on the J1 axis. The solution is still isolated, but the constraint the
    solver roots has a double root there."""

    SingularFamily = ...
    """A joint has no effect on the pose, so this solution is one representative of a continuum."""


class IkSolution:
    """One inverse kinematics solution and its diagnostics."""

    @property
    def joints(self) -> NDArray[numpy.float64]:
        """The six joint angles in FANUC controller degrees."""
        ...

    @property
    def residual(self) -> float:
        """Largest absolute element difference between this solution's pose and the target."""
        ...

    @property
    def kind(self) -> SolutionKind:
        """The configuration category of the solution."""
        ...

    @property
    def theta(self) -> float:
        """The angle on the wrist center's circle that produced the solution, for diagnostics."""
        ...


class Crx:
    """A FANUC CRX robot defined by the four link dimensions that distinguish the models."""

    @staticmethod
    def from_params(z1: float, x1: float, x2: float, y1: float) -> Crx:
        """Build a robot from its four link dimensions, in millimeters."""
        ...

    @staticmethod
    def crx3ia() -> Crx:
        """The CRX-3iA."""
        ...

    @staticmethod
    def crx5ia() -> Crx:
        """The CRX-5iA."""
        ...

    @staticmethod
    def crx10ia() -> Crx:
        """The CRX-10iA."""
        ...

    @staticmethod
    def crx10ial() -> Crx:
        """The CRX-10iA/L."""
        ...

    @staticmethod
    def crx20ial() -> Crx:
        """The CRX-20iA/L."""
        ...

    @staticmethod
    def crx30ia() -> Crx:
        """The CRX-30iA."""
        ...

    @property
    def z1(self) -> float:
        """The height of the J3 origin above the J2 origin, in millimeters."""
        ...

    @property
    def x1(self) -> float:
        """The length of the forearm, from the J3 origin to the J5 origin, in millimeters."""
        ...

    @property
    def x2(self) -> float:
        """The distance from the J5 origin to the flange, in millimeters."""
        ...

    @property
    def y1(self) -> float:
        """The lateral offset of the J5 origin from the forearm axis, in millimeters."""
        ...

    def fk(self, joints: Joints) -> NDArray[numpy.float64]:
        """
        Compute the pose of the flange for a set of joint angles.

        The returned 4x4 matrix matches the robot controller in both position and orientation.
        """
        ...

    def fk_all(self, joints: Joints) -> NDArray[numpy.float64]:
        """
        Compute the pose of every frame in the kinematic chain, as a (6, 4, 4) array.

        The last of the six is the flange and matches `fk`. The others are at the kinematic link
        origins, which have no corresponding values in the controller.
        """
        ...

    def ik(self, target: Transform) -> NDArray[numpy.float64]:
        """
        Every joint configuration which puts the flange at `target`, as an (n, 6) array.

        An unreachable target gives an array with no rows. Use `ik_detailed` when the residual or
        the kind of each solution matters.
        """
        ...

    def ik_detailed(self, target: Transform) -> List[IkSolution]:
        """The same solutions as `ik`, each carrying its residual and its kind."""
        ...

    def ik_closest(self, target: Transform, reference: Joints) -> Optional[IkSolution]:
        """
        The solution whose joints are nearest to `reference`, or None if the target is unreachable.

        A caller following a path can pass the robot's current joint vector as `reference` to reach
        the next pose with the least joint movement, which keeps the arm from reconfiguring during
        the move.
        """
        ...
