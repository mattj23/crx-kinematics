"""
Figure: $O_4$ on the J1 axis.

This is the configuration behind
[issue #1](https://github.com/mattj23/crx-kinematics/issues/1). With $O_4$ on the axis the vertical
plane through $O_1$ and $O_4$ has no defined orientation, so constraint D provides no restriction
on $O_3$. It can lie anywhere on the horizontal circle where the two spheres meet. The perpendicularity
constraint supplies the missing information: it confines $O_3$ to the plane square to $\\vec{u}$
through $O_4$, which cuts the circle at two points.

The base angle then comes from the azimuth of $O_3$, because $O_4$ on the axis has none of its own.
"""

from __future__ import annotations

import numpy
from engeom.geom3 import Circle3, Plane3, Point3
from matplotlib.figure import Figure

from ..ik_reference import Setup
from ..robot import Robot
from ._common import (CONSTRUCT_COLOR, FREE_COLOR, POINT_COLOR, draw_robot, finish, frame_on,
                      new_figure, theta_of, view)

JOINTS = (10.0, -80.0, 10.0, 20.0, -20.0, 45.0)
"""The CRX-10iA configuration from issue #1, which puts O4 exactly on the J1 axis."""


def draw() -> Figure:
    robot = Robot.crx10ia(JOINTS)
    params = robot.params
    setup = Setup(params, robot.frames[5].as_numpy())

    theta = theta_of(setup, numpy.array([*robot.frame_origin(3)]))
    o4_coords, u = setup.o4_and_axis(theta)
    assert abs(o4_coords[0]) < 1e-6 and abs(o4_coords[1]) < 1e-6, "O4 should be on the J1 axis"

    o1 = Point3(0.0, 0.0, 0.0)
    o4 = Point3(*o4_coords)
    candidates = [Point3(*p) for p in setup.o3_on_axis(theta)]

    # The circle the two spheres meet on is horizontal, because O4 is on the vertical axis.
    _, along, radius_sq = setup.circle_of_o3(o4_coords)
    center_z = along * numpy.sign(o4_coords[2])
    radius = float(numpy.sqrt(radius_sq))
    meeting = Circle3(0.0, 0.0, center_z, 0.0, 0.0, 1.0, radius)

    # Derive the view from u so that the plane perpendicular to u appears almost edge-on as a band
    # cutting the circle. The camera also looks down far enough to show the horizontal circle as an
    # ellipse.
    across = numpy.cross([0.0, 0.0, 1.0], u)
    across /= numpy.linalg.norm(across)
    towards = across + 0.3 * u + numpy.array([0.0, 0.0, -0.55])

    figure, helper = new_figure(9.0, 7.0)
    port = helper.viewport(view(towards=towards, focus=Point3(0.0, 0.0, center_z)))

    draw_robot(port, robot)

    # The J1 axis, which O4 has landed on and which therefore gives it no azimuth.
    helper.draw_arrow(port.view @ Point3(0.0, 0.0, -160.0), port.view @ Point3(0.0, 0.0, 780.0),
                      arrow="-", color="0.5", linewidth=1.0, linestyle="-.")
    port.draw_labeled_point(Point3(0.0, 0.0, 760.0), r"$J_1$ axis", offset_2d=(0.0, 55.0),
                            color="0.4", marker_size=0.0, fontsize=13)

    # Every point of this circle satisfies constraints A and B, and D no longer narrows it.
    port.draw_circle(meeting, color=FREE_COLOR, linewidth=1.8, linestyle="--")

    # The perpendicularity constraint: the plane square to u through O4 cuts the circle at two
    # points, and those are the only candidates left.
    port.draw_plane(Plane3.from_point_normal(*o4, *u), center=o4, size=radius * 1.6,
                    fill=True, edgecolor=CONSTRUCT_COLOR, facecolor=CONSTRUCT_COLOR, alpha=0.18,
                    linewidth=1.2)
    helper.draw_arrow(port.view @ o4, port.view @ Point3(*(o4_coords + 260.0 * u)),
                      color=CONSTRUCT_COLOR, linewidth=1.8)
    port.draw_labeled_point(Point3(*(o4_coords + 260.0 * u)), r"$\vec{u}$", offset_2d=(45.0, 20.0),
                            color=CONSTRUCT_COLOR, marker_size=0.0, fontsize=16)

    # The azimuth of O3 is where the base angle comes from, since O4 on the axis has none.
    center = Point3(0.0, 0.0, center_z)
    for candidate in candidates:
        helper.draw_arrow(port.view @ center, port.view @ candidate, arrow="-", color=FREE_COLOR,
                          linewidth=0.9, linestyle=":")

    for point, label, offset in (
            (o1, "$O_1$", (-90.0, -55.0)),
            (o4, "$O_4$", (-105.0, 35.0)),
            (candidates[0], "$O_3$", (60.0, 55.0)),
            (candidates[1], "$O_3'$", (-95.0, 40.0)),
    ):
        color = FREE_COLOR if label.startswith("$O_3") else POINT_COLOR
        port.draw_labeled_point(point, label, offset_2d=offset, color=color, arrow=True,
                                linewidth=0.8, fontsize=15)

    frame_on(port, [o1, o4, *candidates, Point3(0.0, 0.0, 780.0),
                    Point3(radius, 0.0, center_z), Point3(-radius, 0.0, center_z),
                    Point3(0.0, radius, center_z), Point3(0.0, -radius, center_z)])
    return finish(figure, helper)
