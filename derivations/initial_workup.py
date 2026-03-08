"""
    Display the forward kinematics
"""

import numpy
from pathlib import Path
from engeom.geom3 import Iso3, Point3, Vector3
from engeom.plot import MatplotlibAxesHelper, TraceBuilder
from matplotlib.pyplot import figure, Figure, Axes, close as close_figure
from crx.derivation_tools import *
from tqdm import tqdm
import imageio.v3 as imageio

ROBOT_JOINTS = [10, -10, 5, 20, 20, 45]
# ROBOT_JOINTS = [0, 0, 0, 0, 0, 0]
ROBOT = Robot.crx10ia()

BUILD_PATH = Path().cwd() / "build"


def main():
    if not BUILD_PATH.exists():
        BUILD_PATH.mkdir()

    robot = ROBOT
    robot.set_joints(ROBOT_JOINTS)

    front_images = []
    above_images = []

    steps = 360
    for i in tqdm(range(steps)):
        o4_angle = i * 360 / steps * numpy.pi / 180
        front_file = BUILD_PATH / f"front-{i:03d}.png"
        f_behind = view_from_behind(robot, o4_angle)
        f_behind.savefig(front_file)
        front_images.append(front_file)
        close_figure(f_behind)

        f_above = view_from_above(robot, o4_angle)
        above_file = BUILD_PATH / f"above-{i:03d}.png"
        f_above.savefig(above_file)
        above_images.append(above_file)
        close_figure(f_above)

    front_loaded = [imageio.imread(str(p)) for p in front_images]
    above_loaded = [imageio.imread(str(p)) for p in above_images]

    imageio.imwrite(BUILD_PATH / "front.gif", front_loaded, duration=5, loop=0)
    imageio.imwrite(BUILD_PATH / "above.gif", above_loaded, duration=5, loop=0)

    for p in front_images + above_images:
        p.unlink()


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
    return fig


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
    ax.plot(o3_circle_view[:, 0], o3_circle_view[:, 1], color="green", linewidth=1)

    # Draw the o1 sphere
    o1_view = view @ Point3(0, 0, 0)
    o1_sphere = xy_circle(robot.z1, 1000, x=o1_view.x, y=o1_view.y)
    ax.plot(o1_sphere[:, 0], o1_sphere[:, 1], color="blue", linewidth=0.5, linestyle="dotted")

    # Find the o1 intersection circle in the o4 xy plane
    o1_proj = f_1.inverse() @ Point3(0, 0, 0)
    center = f_1 @ o1_proj.with_z(0)
    d = center.coords.norm()
    if d < robot.z1:
        r = numpy.sqrt(robot.z1 ** 2 - d ** 2)
        o1_circle = f_1.transform_points(xy_circle(r, 1000, x=o1_proj.x, y=o1_proj.y))
        o1_circle_view = view.transform_points(o1_circle)
        ax.plot(o1_circle_view[:, 0], o1_circle_view[:, 1], color="blue", linewidth=1)

    ax.axvline(0, color="green", linewidth=0.5)
    ax.axhline(0, color="red", linewidth=0.5)
    optional_axis_label(helper, f_1, view, length=100)

    fig.tight_layout()
    return fig


if __name__ == '__main__':
    main()
