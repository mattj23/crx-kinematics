"""
Figure: the one free parameter.

The target pose fixes the flange, including $O_5$ and $O_6$. The remaining unknown for $O_4$ is its
position on a circle of radius $y_1$ around $O_5$ in the flange plane, so one angle represents the
remaining problem. The figure shows the circle, the flange axis from which $\\theta$ is measured,
and the unit vector $\\vec{u}$ at one value of $\\theta$.
"""

from __future__ import annotations

import math

import numpy
from engeom.geom3 import Circle3, Curve3, Iso3, Point3, Vector3
from matplotlib.figure import Figure

from ..ik_reference import Setup
from ..robot import Robot
from ._common import (CONSTRUCT_COLOR, FREE_COLOR, POINT_COLOR, draw_robot, finish, frame_on,
                      new_figure, theta_of, view)

JOINTS = (35.0, -50.0, 40.0, -60.0, -55.0, 20.0)
"""
This configuration orients the flange across the view, showing the candidate circle open instead of
edge-on and keeping the forearm clear of the annotated area.
"""


def draw() -> Figure:
    robot = Robot.crx10ia(JOINTS)
    setup = Setup(robot.params, robot.frames[5].as_numpy())
    y1 = setup.params.y1
    o5 = Point3(*setup.o5)
    o6 = Point3(*setup.o6)
    normal = Vector3(*numpy.cross(setup.ax, setup.ay))

    actual_o4 = numpy.array([*robot.frame_origin(3)])
    theta = theta_of(setup, actual_o4)
    o4_coords, u = setup.o4_and_axis(theta)
    o4 = Point3(*o4_coords)
    # Verify that the selected angle places O4 at the arm's actual O4 position.
    assert numpy.allclose(o4_coords, actual_o4, atol=1e-9), "theta did not reproduce O4"

    figure, helper = new_figure(8.0, 6.0)
    port = helper.viewport(view(towards=(0.55, 1.0, -0.45), focus=o5))

    draw_robot(port, robot)

    # The circle of candidate O4 positions, with a scattering of the candidates themselves.
    port.draw_circle(Circle3(*o5, *normal, y1), color=FREE_COLOR, linestyle="--", linewidth=1.2)
    others = [Point3(*setup.o4_and_axis(theta + k * math.pi / 8)[0]) for k in range(1, 16)]
    helper.draw_point(*[port.view @ p for p in others], color=FREE_COLOR, marker=".",
                      markersize=4.0)

    # The flange axes, drawn at O5 rather than at the flange itself, because the circle lies in the
    # plane they span and theta is measured from the X axis.
    port.draw_coordinate_system(robot.frames[5] @ Iso3.from_translation(0.0, 0.0, -setup.params.x2),
                                length=y1 * 0.6, fontsize=13)

    # Theta, swept from the flange X axis around to u.
    arc = numpy.array([setup.o5 + 0.5 * y1 * (math.cos(a) * setup.ax + math.sin(a) * setup.ay)
                       for a in numpy.linspace(0.0, theta, 64)])
    port.draw_curve(Curve3(arc), color=CONSTRUCT_COLOR, linewidth=1.4)
    mid = arc[len(arc) // 2]
    port.draw_labeled_point(Point3(*mid), r"$\theta$", offset_3d=Point3(*(0.28 * (mid - setup.o5))),
                            color=CONSTRUCT_COLOR, marker_size=0.0, fontsize=16)

    # u, which is both the direction from O5 to O4 and the direction of the J5 axis.
    helper.draw_arrow(port.view @ o5, port.view @ o4, color=FREE_COLOR, linewidth=1.8)
    port.draw_labeled_point(Point3(*(setup.o5 + 0.6 * y1 * u)), r"$\vec{u}$", offset_2d=(-46.0, -20.0),
                            color=FREE_COLOR, marker_size=0.0, fontsize=16)

    # O5 sits x2 back along the flange axis from O6, which is the only reason it is known.
    helper.draw_arrow(port.view @ o6, port.view @ o5, arrow="-", color=POINT_COLOR, linewidth=0.8,
                      linestyle="dotted")

    for point, label, offset in (
            (o4, "$O_4$", (-55.0, 20.0)),
            (o5, "$O_5$", (-30.0, -45.0)),
            (o6, "$O_6$", (58.0, 6.0)),
    ):
        port.draw_labeled_point(point, label, offset_2d=offset, color=POINT_COLOR, arrow=True,
                                linewidth=0.8, fontsize=15)

    frame_on(port, [o4, o5, o6, *others,
                    robot.frames[5] @ Iso3.from_translation(0.0, 0.0, -setup.params.x2)
                    @ Point3(y1 * 0.6, 0.0, 0.0)])
    return finish(figure, helper)
