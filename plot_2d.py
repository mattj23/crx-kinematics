"""
    Display the forward kinematics
"""

import numpy
from engeom.geom3 import Iso3, Point3, Vector3
from engeom.plot import MatplotlibAxesHelper, TraceBuilder
from matplotlib.pyplot import figure, Figure, Axes

from crx.robot import Robot


def main():
    view = (Iso3.from_rotation(numpy.pi / 6, 1, 0, 0) @
            Iso3.from_rotation(-numpy.pi / 4, 0, 1, 0) @
            Iso3.from_rotation(-numpy.pi / 2, 1, 0, 0) @
            Iso3.from_rotation(-numpy.pi / 2, 0, 0, 1))

    robot = Robot.crx5ia()
    # robot.set_joints([-90, 10, 10, 10, 10, 10])
    robot.set_joints([0, 0, 0, 0, 0, 0])

    fig: Figure = figure(figsize=(10, 10))
    ax: Axes = fig.subplots()
    helper = MatplotlibAxesHelper(ax, hide_axes=True)

    origins = []
    frame_origins = [(robot.frame_origin(i), f"$O_{i + 1}$") for i in [0, 2, 3, 4, 5]]
    shift = Vector3(-12, 12, 0)
    for p, label in frame_origins:
        p = view @ p
        origins.append(list(p))
        helper.annotate_text_only(label, p + shift, color="red", horizontalalignment="center",
                                  verticalalignment="center", fontsize=14)

    origins = numpy.array(origins)
    print(origins)
    ax.plot(origins[:, 0], origins[:, 1], "x", color="red", markersize=7)

    # Robot mesh
    meshes = robot.posed_meshes()
    mesh = meshes[0]
    for m in meshes[1:]:
        mesh.append(m)
    mesh.transform_by(view)

    visible = TraceBuilder()
    hidden = TraceBuilder()
    points, edge_types = mesh.visual_outline()
    for row, edge_type in zip(points, edge_types):
        p0 = Point3(*row[0:3])
        p1 = Point3(*row[3:6])

        if edge_type == 0:
            visible.add_segment(p0, p1)
        else:
            hidden.add_segment(p0, p1)

    ax.plot(*visible.xy, color="black", linewidth=1.0)
    ax.plot(*hidden.xy, color="black", linewidth=0.25, alpha=0.125)

    fig.tight_layout()
    fig.show()


if __name__ == '__main__':
    main()
