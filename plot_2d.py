"""
    Display the forward kinematics
"""

import numpy
from engeom.geom3 import Iso3, Point3, Vector3
from engeom.plot import MatplotlibAxesHelper, TraceBuilder
from matplotlib.pyplot import figure, Figure, Axes
from display_forward import upper_lower, corner_xy

from crx.robot import Robot


def main():
    view = (Iso3.from_rotation(numpy.pi / 6, 1, 0, 0) @
            Iso3.from_rotation(-numpy.pi / 4, 0, 1, 0) @
            Iso3.from_rotation(-numpy.pi / 2, 1, 0, 0) @
            Iso3.from_rotation(-numpy.pi / 2, 0, 0, 1))

    robot = Robot.crx5ia()
    robot.set_joints([-10, -15, 15, 20, 45, 10])
    # robot.set_joints([0, 0, 0, 0, 0, 0])

    fig: Figure = figure(figsize=(8, 8))
    ax: Axes = fig.subplots()
    helper = MatplotlibAxesHelper(ax, hide_axes=True)

    origins = []
    frame_origins = [(robot.frame_origin(i), f"$O_{i + 1}$") for i in [0, 2, 3, 4, 5]]
    shift = Vector3(1, 0, 0) * 25
    for p, label in frame_origins:
        p = view @ p
        origins.append(list(p))
        helper.annotate_text_only(label, p + shift, color="red", horizontalalignment="center",
                                  verticalalignment="center", fontsize=18, weight="bold")

    origins = numpy.array(origins)
    print(origins)
    ax.plot(origins[:, 0], origins[:, 1], "o", color="red", markersize=7)

    # Robot mesh
    # ========================================================================
    mesh = robot.posed_single_mesh()
    mesh.transform_by(view)

    visible = TraceBuilder()
    hidden = TraceBuilder()
    points, edge_types = mesh.visual_outline(Vector3(0, 0, 1), 1.0, numpy.pi / 6)
    for row, edge_type in zip(points, edge_types):
        p0 = Point3(*row[0:3])
        p1 = Point3(*row[3:6])

        if edge_type == 0:
            visible.add_segment(p0, p1)
        else:
            hidden.add_segment(p0, p1)

    ax.plot(*visible.xy, color="black", linewidth=1.0)
    ax.plot(*hidden.xy, color="black", linewidth=0.25, alpha=0.125)

    # O4 candidates
    # ========================================================================
    raw_points = numpy.array([[numpy.cos(x), numpy.sin(x), 0] for x in numpy.linspace(0, 2 * numpy.pi, 500)]) * robot.y1
    raw_points[:, 2] = -robot.x2
    points = robot.frames[-1].transform_points(raw_points)
    view_points = view.transform_points(points)
    ax.plot(view_points[:, 0], view_points[:, 1], color="red", linewidth=0.75, linestyle="--")

    # O3 candidates
    upper = TraceBuilder()
    lower = TraceBuilder()
    for xyz in points:
        p = Point3(*xyz)
        u, l = upper_lower(p, robot)
        upper.add_points(view @ u)
        lower.add_points(view @ l)

    ax.plot(*upper.xy, color="blue", linewidth=0.75, linestyle="--")
    ax.plot(*lower.xy, color="green", linewidth=0.75, linestyle="--")

    fig.tight_layout()
    fig.show()
    # fig.savefig("images/cover.png", dpi=150)


if __name__ == '__main__':
    main()
