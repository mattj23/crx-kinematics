"""
    Script for generating some of the documentation images
"""

import numpy
from engeom.geom2 import Vector2
from engeom.geom3 import Iso3
from engeom.plot import MatplotlibAxesHelper
from matplotlib.pyplot import figure, Figure, Axes

from crx.robot import Robot


def main():
    view = (Iso3.from_rotation(numpy.pi / 6, 1, 0, 0) @
            Iso3.from_rotation(-numpy.pi / 4, 0, 1, 0) @
            Iso3.from_rotation(-numpy.pi / 2, 1, 0, 0) @
            Iso3.from_rotation(-numpy.pi / 2, 0, 0, 1))

    robot = Robot.crx10ia()
    robot.set_joints([0, 0, 0, 0, 0, 0])

    fig: Figure = figure(figsize=(8, 8))
    ax: Axes = fig.subplots()
    helper = MatplotlibAxesHelper(ax, hide_axes=True)
    view_port = helper.get_3d_viewport(view)

    # Robot mesh
    # ========================================================================
    mesh = robot.posed_single_mesh()
    view_port.mesh_outline(mesh)

    # Dimension labels
    # ========================================================================
    view_port.dimension_arrow(robot.frame_origin(0), robot.frame_origin(2), "$z_1$", leader_shift=(-50, 0, 0),)
    view_port.dimension_arrow(robot.frame_origin(2), robot.frame_origin(3), "$x_1$", leader_shift=(0, 0, -100),)
    view_port.dimension_arrow(robot.frame_origin(3), robot.frame_origin(4), "$y_1$", leader_shift=(250, 0, 0),)
    view_port.dimension_arrow(robot.frame_origin(4), robot.frame_origin(5), "$x_2$", leader_shift=(0, -100, 0),)

    view_port.coordinate_system(Iso3.identity(), length=50)

    # Origin labels
    # ========================================================================
    origins = []
    frame_origins = [(robot.frame_origin(i), f" $O_{i + 1}$ ") for i in [0, 2, 3, 4, 5]]
    frame_shifts = [Vector2(*x) * 25.4 for x in [(0, -1.5), (-1.0, 1.0), (1, 0), (0, 1.0), (-3, 0)]]
    for (p, label), shift in zip(frame_origins, frame_shifts):
        view_port.labeled_point(p, label, offset_2d=shift, fontsize=18, color="red", weight="bold", marker_size=7)

    fig.tight_layout()
    # fig.show()
    fig.savefig("images/parameters.png", dpi=100)


if __name__ == '__main__':
    main()
