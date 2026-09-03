"""
Figure: the two branches of $O_3$.

With $O_4$ fixed at one $\\theta$, constraint A puts $O_3$ on a sphere of radius $z_1$ about the
origin and constraint B puts it on a sphere of radius $x_1$ about $O_4$. The two spheres meet in a
circle, and constraint D confines $O_3$ to the vertical plane through $O_1$ and $O_4$, which cuts
that circle at two points.

The view follows the normal of that vertical plane. This orientation shows the plane face-on, the
two spheres as great circles, and their intersection circle edge-on as a segment.
"""

from __future__ import annotations

import numpy
from engeom.geom3 import Circle3, Point3, Vector3
from matplotlib.figure import Figure

from ..ik_reference import Setup
from ..robot import Robot
from ._common import (CONSTRUCT_COLOR, FREE_COLOR, POINT_COLOR, draw_robot, finish, frame_on,
                      new_figure, view)

JOINTS = (0.0, -35.0, 65.0, 0.0, -50.0, 0.0)
"""
The configuration drawn, with the base at zero so the arm lies in the world XZ plane and the view
is square to it. The shoulder and elbow angles were chosen to keep the two branches well apart.
"""


def draw() -> Figure:
    robot = Robot.crx10ia(JOINTS)
    params = robot.params
    setup = Setup(params, robot.frames[5].as_numpy())

    o1 = Point3(0.0, 0.0, 0.0)
    o4_coords = numpy.array([*robot.frame_origin(3)])
    o4 = Point3(*o4_coords)

    # The vertical plane through O1 and O4 has this normal, which is also the view direction.
    normal = Vector3(*numpy.cross([0.0, 0.0, 1.0], o4_coords)).normalized()

    # The circle where the two spheres meet: centered along the direction of O4, square to it.
    distance, along, radius_sq = setup.circle_of_o3(o4_coords)
    direction = o4_coords / distance
    radius = float(numpy.sqrt(radius_sq))
    meeting = Circle3(*(along * direction), *direction, radius)

    branches = {sign: Point3(*setup.o3_branch(o4_coords, sign)) for sign in (1, -1)}
    actual = numpy.array([*robot.frame_origin(2)])
    distances = {s: float(numpy.linalg.norm(numpy.array([*p]) - actual)) for s, p in branches.items()}
    used = min(distances, key=distances.get)
    # One branch contains the arm's current elbow. Verify that the other branch is far enough away
    # to appear as a visibly different posture.
    assert distances[used] < 1e-9, "neither branch is the configuration drawn"
    assert distances[-used] > params.x1 * 0.5, "the two branches are too close to tell apart"

    figure, helper = new_figure(9.0, 6.5)
    port = helper.viewport(view(towards=normal, focus=Point3(*(0.5 * o4_coords))))

    draw_robot(port, robot)

    # The J1 axis, which orients the drawing: the plane being viewed is vertical and contains it.
    helper.draw_arrow(port.view @ Point3(0.0, 0.0, -params.z1 * 0.25),
                      port.view @ Point3(0.0, 0.0, params.z1 * 1.35),
                      arrow="-", color="0.5", linewidth=0.8, linestyle="-.")

    # The two spheres, each seen as the great circle where it meets the drawing plane.
    port.draw_circle(Circle3(*o1, *normal, params.z1), color=CONSTRUCT_COLOR, linestyle="--",
                     linewidth=1.0, alpha=0.8)
    port.draw_circle(Circle3(*o4, *normal, params.x1), color=CONSTRUCT_COLOR, linestyle="--",
                     linewidth=1.0, alpha=0.8)

    # Their intersection circle, edge-on, which is the segment joining the two branches.
    port.draw_circle(meeting, color=FREE_COLOR, linewidth=2.0)

    port.draw_dimension_arrow(o1, branches[used], "$z_1$", label_position=0.55, fontsize=13)
    port.draw_dimension_arrow(branches[used], o4, "$x_1$", label_position=0.45, fontsize=13)

    for point, label, offset in (
            (o1, "$O_1$", (-70.0, -40.0)),
            (o4, "$O_4$", (60.0, -55.0)),
            (branches[1], "$O_3^{+}$", (55.0, 45.0)),
            (branches[-1], "$O_3^{-}$", (25.0, -75.0)),
    ):
        color = FREE_COLOR if label.startswith("$O_3") else POINT_COLOR
        port.draw_labeled_point(point, label, offset_2d=offset, color=color, arrow=True,
                                linewidth=0.8, fontsize=15)

    frame_on(port, [o1, o4, branches[1], branches[-1],
                    Point3(*(o4_coords + params.x1 * numpy.array([0.0, 0.0, -1.0]))),
                    Point3(0.0, 0.0, params.z1 * 1.35), Point3(0.0, 0.0, -params.z1 * 0.25)])
    return finish(figure, helper)
