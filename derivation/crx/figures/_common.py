"""
Shared output paths, view construction, and visual styles for the derivation figures.

The figures use line drawings. Engeom's ``ViewPort3`` projects 3D entities onto Matplotlib axes
with parallel projection, producing an elliptical projection of a circle and a hidden-line outline
of the robot.
"""

from __future__ import annotations

from pathlib import Path

import math

import numpy
from engeom.geom3 import Iso3, Mesh3, Point3, Vector3
from engeom.plot.matplotlib import AxesHelper
from matplotlib.figure import Figure

IMAGE_DIR = Path(__file__).parents[3] / "docs" / "images"
"""Where the figures are written, beside the documents that use them."""

ROBOT_STYLE = {"color": "0.72", "linewidth": 0.8}
"""
Draw the arm as context in light gray without hidden edges so that the overlaid construction remains
legible.
"""

POINT_COLOR = "black"
"""Kinematic origins, which are the fixed quantities in each figure."""

FREE_COLOR = "tab:purple"
"""The parameterized circle and the quantities that move with it."""

CONSTRUCT_COLOR = "tab:blue"
"""Constructions derived from the free parameter: spheres, planes, and the points they produce."""


def theta_of(setup, o4) -> float:
    """
    Return the angle on the circle of candidates that corresponds to an actual ``O4``.

    A figure draws a real configuration, so it starts from joint angles and needs the `theta` the
    solver would have found for them.

    :param setup: the :class:`crx.ik_reference.Setup` for the pose
    :param o4: the actual position of ``O4``, in world coordinates
    """
    u = (numpy.asarray(o4, dtype=float) - setup.o5) / setup.params.y1
    return math.atan2(float(u @ setup.ay), float(u @ setup.ax))


def view(towards, up=(0.0, 0.0, 1.0), focus=(0.0, 0.0, 0.0)) -> Iso3:
    """
    Build the world-to-view isometry for a camera looking in a given direction.

    :param towards: the direction the camera looks along, from the camera into the scene
    :param up: the direction to place upwards in the image, orthogonalized against `towards`
    :param focus: the world point that lands at the center of the image
    :return: the isometry to hand to :meth:`AxesHelper.viewport`
    """
    out_of_image = -Vector3(*towards).normalized()
    right = Vector3(*up).cross(out_of_image)
    return Iso3.from_basis_xz(right, out_of_image, Point3(*focus)).inverse()


def new_figure(width: float = 8.0, height: float = 6.0) -> tuple[Figure, AxesHelper]:
    """
    Create a figure with hidden axes and the 1:1 aspect required by these diagrams.

    :return: the figure and the helper wrapping its axes
    """
    figure = Figure(figsize=(width, height), dpi=150)
    axes = figure.subplots()
    return figure, AxesHelper(axes, hide_axes=True)


def finish(figure: Figure, helper: AxesHelper) -> Figure:
    """
    Tighten the layout and expand the axis limits so the drawing fills the figure.

    ``fill_available_space`` reads the current limits and rendered size, so call it after drawing
    all elements and applying ``tight_layout``.
    """
    figure.tight_layout()
    helper.fill_available_space()
    return figure


def frame_on(port, points, margin: float = 0.18) -> None:
    """
    Crop the axes to the region the construction occupies.

    The arm is a meter long, while most constructions are a few hundred millimeters across. Framing
    the complete robot would make the construction too small to read. The given points set the
    limits, and :func:`finish` only widens them afterward.

    :param port: the ``ViewPort3`` the points are drawn in
    :param points: the world points the figure is about
    :param margin: padding around them, as a fraction of the larger dimension
    """
    projected = [port.view @ point for point in points]
    xs = [p.x for p in projected]
    ys = [p.y for p in projected]
    pad = margin * max(max(xs) - min(xs), max(ys) - min(ys))
    port.helper.ax.set_xlim(min(xs) - pad, max(xs) + pad)
    port.helper.ax.set_ylim(min(ys) - pad, max(ys) + pad)


def draw_robot(viewport, robot, style: dict | None = None) -> None:
    """
    Draw the arm as a single hidden-line outline behind the construction.

    The links are merged into one mesh first, because outlines calculated per link would show the
    seams where one link disappears inside another.

    :param viewport: the ``ViewPort3`` to draw into
    :param robot: a posed :class:`crx.robot.Robot`
    :param style: overrides for the visible-edge style, defaulting to :data:`ROBOT_STYLE`
    """
    mesh: Mesh3 = robot.posed_single_mesh()
    viewport.draw_mesh_outline(mesh, visible_kwargs=style or ROBOT_STYLE, no_hidden=True)
