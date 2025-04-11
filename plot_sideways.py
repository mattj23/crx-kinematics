"""
    Assemble the robot side view
"""

import numpy
from engeom.geom3 import Iso3, Point3, Vector3
from engeom.geom2 import Circle2, Aabb2
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
    expected_len = numpy.sqrt(robot.y1**2 + robot.x1**2)

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

    c0 = Circle2(0, 0, robot.x1)
    c1 = Circle2(x_v.norm(), 0, robot.z1)

    fig: Figure = figure(figsize=(8, 6))
    ax: Axes = fig.subplots()
    helper = MatplotlibAxesHelper(ax)

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

    ax.plot(view_points[:, 0], view_points[:, 1], color="red", linewidth=0.75, linestyle="--")

    o5 = view @ robot.frame_origin(4)
    ax.plot([0, x_v.norm(), o5.x], [0, 0, o5.y], "x", color="black", markersize=7)
    helper.annotate_text_only("$O_4$", (0, -50), color="black", horizontalalignment="center", fontsize=14)
    helper.annotate_text_only("$O_1$", (x_v.norm(), -50), color="black", horizontalalignment="center", fontsize=14)
    helper.annotate_text_only("$O_5$", (o5.x, o5.y + 30), color="black", horizontalalignment="center", fontsize=14)
    ax.annotate("", xy=(x_v.norm(), 0), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="black"))
    # ax.annotate("", xy=(o5.x, o5.y), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="blue"))

    helper.plot_circle(c0, fill=False, linewidth=0.75, edgecolor="black", linestyle="--", in_layout=False)
    helper.plot_circle(c1, fill=False, linewidth=0.75, edgecolor="black", linestyle="--", in_layout=False)

    box = Aabb2(0, -200, c1.x, 200).expand(200)
    helper.set_bounds(box)

    fig.tight_layout()
    fig.show()
    # fig.savefig("images/cover.png", dpi=150)


if __name__ == '__main__':
    main()
