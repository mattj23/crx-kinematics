"""
    Script for generating some of the documentation images
"""

import numpy
from engeom.geom2 import Vector2
from engeom.geom3 import Iso3, Mesh, Line3
from engeom.plot import MatplotlibAxesHelper
from matplotlib.pyplot import figure, Figure, Axes

from crx.robot import Robot

DISPLAY_INSTEAD_OF_SAVE = False


def main():
    view = iso_view()
    robot = Robot.crx10ia()
    robot.set_joints([0, 0, 0, 0, 0, 0])

    origins_and_parameters(view, robot)
    links_and_joints(view, robot)


def links_and_joints(view: Iso3, robot: Robot):
    """ This function draws a diagram which labels the links and joints of a CRX robot """
    fig: Figure = figure(figsize=(8, 8))
    ax: Axes = fig.subplots()
    helper = MatplotlibAxesHelper(ax, hide_axes=True)
    view_port = helper.get_3d_viewport(view)

    # Robot mesh
    # ========================================================================
    mesh = robot.posed_single_mesh()
    link_meshes: list[Mesh] = robot.posed_meshes()
    view_port.mesh_outline(mesh)

    # Link labels
    # ========================================================================
    link_data = [
        ("Base", (-3, 0)),
        ("Link 1", (3, 0)),
        ("Link 2", (-3, -2)),
        ("Link 3", (2, 2)),
        ("Link 4", (0, -2)),
        ("Link 5", (0, 2)),
        ("Flange", (-1, -2)),
    ]
    for (label, shift), mesh in zip(link_data, link_meshes):
        anchor_point = view_port.mesh_edge_point_in_dir(*shift, mesh)
        view_port.labeled_point(anchor_point, label, offset_2d = Vector2(*shift) * 25.4, marker_size=0,
                                arrow=True, box=True, color="blue")

    # Joint Axes
    # ========================================================================
    joint_args = dict(color="red", linewidth=1.0, linestyle="--")
    label_args = dict(fontsize=18, color="red", weight="bold", marker_size=3)

    def _plot_j(_l: Line3, _offset, _n: int):
        view_port.line(_l, 75, **joint_args)
        view_port.labeled_point(_l.origin, f"$\\overrightarrow{{J_{_n}}}$", offset_2d=_offset, **label_args)

    j1 = Line3(0, 0, link_meshes[0].aabb.max.z, 0, 0, 1)
    _plot_j(j1, (-30, 30), 1)

    j2 = Line3(0, link_meshes[1].aabb.min.y, 0, 0, 1, 0)
    _plot_j(j2, (-30, -30), 2)

    j3 = Line3(0, link_meshes[3].aabb.min.y, robot.z1, 0, 1, 0)
    _plot_j(j3, (30, 30), 3)

    j4 = Line3(link_meshes[3].aabb.max.x, 0.0, robot.z1, 1, 0, 0)
    _plot_j(j4, (-30, 15), 4)

    j5 = Line3(robot.x1, link_meshes[5].aabb.max.y, robot.z1, 0, 1, 0)
    _plot_j(j5, (40, -40), 5)

    j6 = Line3(link_meshes[5].aabb.max.x, -robot.y1, robot.z1, 1, 0, 0)
    _plot_j(j6, (40, 50), 6)

    view_port.coordinate_system(Iso3.identity(), length=50)

    fig.tight_layout()
    if DISPLAY_INSTEAD_OF_SAVE:
        fig.show()
    else:
        fig.savefig("images/links_and_joints.png", dpi=100)


def origins_and_parameters(view: Iso3, robot: Robot):
    """ This function draws the origins and kinematic parameters of a CRX robot """

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
    view_port.dimension_arrow(robot.frame_origin(0), robot.frame_origin(2), "$z_1$", leader_shift=(-50, 0, 0), )
    view_port.dimension_arrow(robot.frame_origin(2), robot.frame_origin(3), "$x_1$", leader_shift=(0, 0, -100), )
    view_port.dimension_arrow(robot.frame_origin(3), robot.frame_origin(4), "$y_1$", leader_shift=(250, 0, 0), )
    view_port.dimension_arrow(robot.frame_origin(4), robot.frame_origin(5), "$x_2$", leader_shift=(0, -100, 0), )

    view_port.coordinate_system(Iso3.identity(), length=50)

    # Origin labels
    # ========================================================================
    frame_origins = [(robot.frame_origin(i), f" $O_{i + 1}$ ") for i in [0, 2, 3, 4, 5]]
    frame_shifts = [Vector2(*x) * 25.4 for x in [(0, -1.5), (-1.0, 1.0), (1, 0), (0, 1.0), (-3, 0)]]
    for (p, label), shift in zip(frame_origins, frame_shifts):
        view_port.labeled_point(p, label, offset_2d=shift, fontsize=18, color="red", weight="bold", marker_size=7)

    fig.tight_layout()
    if DISPLAY_INSTEAD_OF_SAVE:
        fig.show()
    else:
        fig.savefig("images/parameters.png", dpi=100)


def iso_view():
    return (Iso3.from_rotation(numpy.pi / 6, 1, 0, 0) @
            Iso3.from_rotation(-numpy.pi / 4, 0, 1, 0) @
            Iso3.from_rotation(-numpy.pi / 2, 1, 0, 0) @
            Iso3.from_rotation(-numpy.pi / 2, 0, 0, 1))


if __name__ == '__main__':
    main()
