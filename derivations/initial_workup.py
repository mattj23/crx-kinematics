"""
    Display the forward kinematics
"""

import numpy
from engeom.geom3 import Iso3, Point3, Vector3
from engeom.plot import MatplotlibAxesHelper, TraceBuilder
from matplotlib.pyplot import figure, Figure, Axes
from crx.derivation_tools import *

ROBOT_JOINTS = [10, -80, 10, 20, -20, 45]
# ROBOT_JOINTS = [0, 0, 0, 0, 0, 0]
ROBOT = Robot.crx10ia()


def view_from_behind(robot: Robot, o4_angle: float):
    end_frame = robot.frames[-1]
    o6 = end_frame @ Point3(0, 0, 0)
    o5 = end_frame @ Point3(0, 0, -robot.x2)
    o4 = o4_candidate(robot, o4_angle)
    f_0 = Iso3.from_basis_xz(o5 - o6, o4 - o5, o5)
    f_1 = f_0 @ Iso3.from_translation(0, 0, robot.y1)

    view = Iso3.from_basis_xy(Vector3(0, 1, 0), Vector3(0, 0, 1)).inverse() @ f_1.inverse()

    # Create plot
    fig: Figure = figure(figsize=(8, 8))
    ax: Axes = fig.subplots()
    helper = MatplotlibAxesHelper(ax, hide_axes=True)

    # Robot mesh
    draw_robot(robot, helper, view, show_origins=True)

    # ax.axvline(0, color="black", linewidth=0.75)
    # ax.axhline(0, color="black", linewidth=0.75)

    # Draw the o4 candidate circle
    o4_circle = xy_circle(robot.y1, 1000, x=(view @ o5).x, y=(view @ o5).y)
    ax.plot(o4_circle[:, 0], o4_circle[:, 1], color="red", linewidth=0.75, linestyle="--")

    # Draw the o1 sphere
    o1_view = view @ Point3(0, 0, 0)
    o1_sphere = xy_circle(robot.z1, 1000, x=o1_view.x, y=o1_view.y)
    ax.plot(o1_sphere[:, 0], o1_sphere[:, 1], color="blue", linewidth=0.75, linestyle="--")

    # Draw the o3 candidate circle
    o3_circle = f_1.transform_points(xy_circle(robot.x1, 1000))
    o3_circle_view = view.transform_points(o3_circle)
    ax.plot(o3_circle_view[:, 0], o3_circle_view[:, 1], color="green", linewidth=0.75, linestyle="--")


    optional_axis_label(helper, f_1, view, length=100)

    ax.set_xlim(-700, 700)
    ax.set_ylim(-700, 700)

    fig.tight_layout()
    fig.show()


def main():
    robot = ROBOT
    robot.set_joints(ROBOT_JOINTS)
    o4_angle = numpy.pi / 1

    # view_from_behind(robot, o4_angle)
    view_from_above(robot, o4_angle)


def view_from_above(robot: Robot, o4_angle: float):
    end_frame = robot.frames[-1]
    o6 = end_frame @ Point3(0, 0, 0)
    o5 = end_frame @ Point3(0, 0, -robot.x2)
    o4 = o4_candidate(robot, o4_angle)
    f_0 = Iso3.from_basis_xz(o5 - o6, o4 - o5, o5)
    f_1 = f_0 @ Iso3.from_translation(0, 0, robot.y1)

    # To look at the intended view
    view = f_1.inverse()

    fig: Figure = figure(figsize=(8, 8))
    ax: Axes = fig.subplots()
    helper = MatplotlibAxesHelper(ax, hide_axes=True)
    draw_robot(robot, helper, view, show_origins=True)

    # Draw the o3 candidate circle
    o3_circle = f_1.transform_points(xy_circle(robot.x1, 1000))
    o3_circle_view = view.transform_points(o3_circle)
    ax.plot(o3_circle_view[:, 0], o3_circle_view[:, 1], color="green", linewidth=0.75, linestyle="--")

    # Draw the o1 sphere
    o1_view = view @ Point3(0, 0, 0)
    o1_sphere = xy_circle(robot.z1, 1000, x=o1_view.x, y=o1_view.y)
    ax.plot(o1_sphere[:, 0], o1_sphere[:, 1], color="blue", linewidth=0.5, linestyle="dotted")

    ax.axvline(0, color="green", linewidth=0.5)
    ax.axhline(0, color="red", linewidth=0.5)
    optional_axis_label(helper, f_1, view, length=100)

    fig.tight_layout()
    fig.show()
    # fig.savefig("images/cover.png", dpi=150)


if __name__ == '__main__':
    main()
