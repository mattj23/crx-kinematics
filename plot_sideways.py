"""
    Assemble the robot side view
"""

import numpy
from engeom.geom3 import Iso3, Point3, Vector3
from engeom.geom2 import Circle2, Aabb2, Point2
from engeom.plot import MatplotlibAxesHelper, TraceBuilder, PyvistaPlotterHelper
from matplotlib.pyplot import figure, Figure, Axes
from display_forward import upper_lower, corner_xy
from pyvista import Plotter

from crx.robot import Robot


def o4_candidates(robot: Robot):
    raw_points = numpy.array([[numpy.cos(x), numpy.sin(x), 0] for x in numpy.linspace(0, 2 * numpy.pi, 500)]) * robot.y1
    raw_points[:, 2] = -robot.x2
    return robot.frames[-1].transform_points(raw_points)


def main():
    robot = Robot.crx5ia()
    robot.set_joints([-10, -15, 15, 20, 45, 10])

    # There are some things we can set up that we know about the robot
    # ========================================================================
    # The expected hypotenuse of the O3-O4-O5 triangle
    expected_len = numpy.sqrt(robot.y1 ** 2 + robot.x1 ** 2)

    # O4 candidates
    # ========================================================================
    o4_points = o4_candidates(robot)

    # candidate_point = o4_points[0]
    candidate_point = robot.frame_origin(3)

    # Get the vector between the candidate point and the origin
    x_v = -Vector3(*candidate_point)
    x_n = Vector3(x_v.x, x_v.y, 0.0)

    x = x_v.normalized()
    z = x_v.cross(x_n).normalized()
    y = z.cross(x).normalized()
    matrix = numpy.array([
        [x.x, y.x, z.x, -x_v.x],
        [x.y, y.y, z.y, -x_v.y],
        [x.z, y.z, z.z, -x_v.z],
        [0.0, 0.0, 0.0, 1.0]
    ])
    view = Iso3(matrix).inverse()
    view_points = view.transform_points(o4_points)

    fig: Figure = figure(figsize=(8, 6))
    ax: Axes = fig.subplots()
    helper = MatplotlibAxesHelper(ax)

    # Plot the robot
    plot_line_robot(ax, robot, view)

    # Plot the O4 candidate circle
    ax.plot(view_points[:, 0], view_points[:, 1], color="red", linewidth=0.75, linestyle="--")

    # Compute the O1, O4, and O5 origins
    o1 = view @ robot.frame_origin(1)
    o4 = view @ robot.frame_origin(3)
    o5 = view @ robot.frame_origin(4)

    # Compute the circles
    c3 = Circle2(0, 0, robot.x1)
    c2 = Circle2(o1.x, 0, robot.z1)

    text_props = dict(ha="center", va="center", fontsize=14)
    arrow_props = dict(fontsize=10, linewidth=0.75, arrow="<->")

    helper.text("$O_4$", o4, shift=(-30, -30), **text_props)
    helper.text("$O_1$", o1, shift=(30, -30), **text_props)
    helper.text("$O_5$", o5, shift=(0, 30), color="red", **text_props)
    helper.labeled_arrow(o4, o1, "$d$", shift=(0, -30), **arrow_props)

    # Plot the arm and forearm radii
    helper.plot_circle(c3, fill=False, linewidth=0.75, edgecolor="black", linestyle="--", in_layout=False)
    helper.plot_circle(c2, fill=False, linewidth=0.75, edgecolor="black", linestyle="--", in_layout=False)
    helper.labeled_arrow(o1, c2.point_at_angle(numpy.radians(-110)), "$r_2$", shift=(25, 0), **arrow_props)
    helper.labeled_arrow(o4, c3.point_at_angle(numpy.radians(-60)), "$r_3$",  shift=(-25, 0), **arrow_props)

    # Find a and h to identify O3
    a = (o1.x ** 2 + c3.r ** 2 - c2.r ** 2) / (2 * o1.x)
    h = numpy.sqrt(c3.r ** 2 - a ** 2)
    o3 = Point2(a, h)

    # Plot the a and h distances
    helper.labeled_arrow((a, 0), o3, "$h$", shift=(20, 10), **arrow_props)
    helper.labeled_arrow((0, 20), (a, 20), "$a$", shift=(0, 10), **arrow_props)

    # Plot all origin points
    helper.text("$O_3$", (a, h), shift=(30, 0), **text_props)
    helper.points(o1, o4, o3, color="black")
    helper.points(o5, color="red")

    # Plot the error distance
    helper.labeled_arrow(o5, o3, "$e$", shift=(-15, 15), color="red", linestyle="--", **arrow_props)

    box = Aabb2(0, -200, c2.x, 200).expand(200)
    helper.set_bounds(box)
    fig.tight_layout()
    fig.show()
    fig.savefig("images/o3-plane.png", dpi=150)


def plot_line_robot(ax: Axes, robot: Robot, view: Iso3):
    mesh = robot.posed_single_mesh()
    mesh.transform_by(view)
    visible = TraceBuilder()
    points, edge_types = mesh.visual_outline(Vector3(0, 0, 1), 1.0, numpy.pi / 6)
    for row, edge_type in zip(points, edge_types):
        p0 = Point3(*row[0:3])
        p1 = Point3(*row[3:6])
        if edge_type == 0:
            visible.add_segment(p0, p1)
    ax.plot(*visible.xy, color="black", linewidth=0.5, alpha=0.25)


if __name__ == '__main__':
    main()
