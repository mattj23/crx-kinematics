"""
Figure: every reachable configuration that reaches the issue #1 pose.

This figure demonstrates the solution to
[issue #1](https://github.com/mattj23/crx-kinematics/issues/1). The companion ``axis_case`` figure
explains the difficulty: $O_4$ lies on the J1 axis, the vertical plane through $O_1$ and $O_4$ has
no orientation, and constraint D provides no information. The solver returns all sixteen
configurations within the robot's motion range. Each panel draws the flange frame in the same
location, confirming that every configuration reaches the same pose.

Panel borders mark the four configurations that the solver flags as axis-degenerate. Their base
angles come from the azimuth of $O_3$; ordinary configurations use $O_4$. The figure also identifies
the generating configuration because the surveyed libraries failed to recover it.
"""

from __future__ import annotations

import numpy
from engeom.geom3 import Point3
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
from engeom.plot.matplotlib import AxesHelper

from ..ik_reference import CrxParams, SolutionKind, fk, ik
from ..robot import Robot
from ._common import view

JOINTS = (10.0, -80.0, 10.0, 20.0, -20.0, 45.0)
"""The CRX-10iA configuration from issue #1, which puts O4 exactly on the J1 axis."""

JOINT_LIMITS = (190.0, 180.0, 285.0, 190.0, 180.0, 225.0)
"""
Half of each axis's motion range for the CRX-10iA, in controller degrees.

The FANUC datasheet gives the range of each axis as a total sweep, 380 through J1, 360 through J2,
570 through J3, 380 through J4, 360 through J5, and 450 through J6. The controller reports a signed
angle about the zero of each axis, so half of each total is the bound a solution is checked against.
"""

GRID = (4, 4)
"""Rows and columns of panels, which is exactly the sixteen configurations this pose has."""

FRAME_LENGTH = 190.0
"""The length of each drawn axis of the flange frame, in millimeters."""

FRAME_COLORS = ("tab:red", "tab:green", "tab:blue")
"""The colors of the flange frame's X, Y, and Z axes."""

PANEL_STYLE = {"color": "0.55", "linewidth": 0.7}
"""
The arm style for an ordinary panel. It is darker than the shared style in ``_common`` because the
arm is the subject and each panel is one quarter of the figure width. The shared style is designed
for an arm behind a geometric construction.
"""

GENERATING_STYLE = {"color": "0.10", "linewidth": 1.0}
"""The configuration the pose was built from, drawn darker than the rest."""

DEGENERATE_COLOR = "tab:purple"
"""The panel border of a configuration the solver flags as axis-degenerate."""


def within_limits(joints) -> bool:
    """
    Report whether a configuration lies inside the robot's motion range.

    The solver returns every configuration that reaches the pose, including configurations outside
    the physical arm's motion range. Filter those configurations when demonstrating physically
    attainable solutions.

    :param joints: six joint angles in controller degrees
    :return: True when every joint is within its half-range
    """
    return all(abs(value) <= limit for value, limit in zip(joints, JOINT_LIMITS))


def solutions() -> list:
    """
    Return the reachable solutions for the issue #1 pose, in a stable order.

    :return: the solutions inside the motion range, ordered by base angle
    """
    params = CrxParams.crx10ia()
    found = ik(params, fk(params, numpy.asarray(JOINTS, dtype=float)))
    reachable = [s for s in found if within_limits(s.joints)]
    return sorted(reachable, key=lambda s: float(s.joints[0]))


def is_generating(joints, tol: float = 1e-6) -> bool:
    """Report whether a configuration is the one the pose was built from."""
    return all(abs(a - b) < tol for a, b in zip(joints, JOINTS))


def draw() -> Figure:
    rows, columns = GRID
    found = solutions()
    assert len(found) == rows * columns, f"expected {rows * columns} panels, got {len(found)}"
    assert any(is_generating(s.joints) for s in found), "the generating configuration is missing"

    # Pose and merge every panel before drawing so the shared camera can include all sixteen arms.
    robots = [Robot.crx10ia(tuple(float(v) for v in s.joints)) for s in found]
    meshes = [robot.posed_single_mesh() for robot in robots]

    # View every panel from the front, left, and slightly above. The shared camera makes the panels
    # directly comparable and places the flange frame on the same pixels in each panel.
    camera = view((-1.0, -0.55, -0.30), focus=(0.0, 0.0, 380.0))

    # Derive one set of limits from the union of all sixteen arms. These limits prevent clipping and
    # keep every panel at the same scale.
    corners = numpy.vstack([camera.transform_points(mesh.points)[:, :2] for mesh in meshes])
    low = corners.min(axis=0)
    high = corners.max(axis=0)
    span = high - low
    pad = 0.04 * span.max()
    half = 0.5 * span + pad
    middle = 0.5 * (low + high)

    # Match the window proportions to the region occupied by the arms, which is wider than it is
    # tall. A square window would add unused space to every panel and reduce the size of the arms.
    panel_width = 3.0
    figure = Figure(
        figsize=(columns * panel_width, rows * panel_width * half[1] / half[0] + 1.8),
        dpi=150,
    )
    axes = figure.subplots(rows, columns)

    flange = robots[0].frames[5]
    for index, (solution, robot, mesh) in enumerate(zip(found, robots, meshes)):
        ax = axes[index // columns][index % columns]
        helper = AxesHelper(ax, hide_axes=True)
        port = helper.viewport(camera)

        generating = is_generating(solution.joints)
        port.draw_mesh_outline(
            mesh,
            visible_kwargs=GENERATING_STYLE if generating else PANEL_STYLE,
            no_hidden=True,
        )

        # Draw the same flange frame in every panel to show that all configurations reach it.
        origin = flange @ Point3(0.0, 0.0, 0.0)
        for axis, color in enumerate(FRAME_COLORS):
            offset = [0.0, 0.0, 0.0]
            offset[axis] = FRAME_LENGTH
            tip = flange @ Point3(*offset)
            start = port.view @ origin
            end = port.view @ tip
            ax.plot([start.x, end.x], [start.y, end.y], color=color, linewidth=1.6)

        # Preserve the shared limits by letting Matplotlib adjust the axes box to achieve an equal
        # aspect ratio. Allowing Matplotlib to adjust the limits would widen them separately for
        # each panel and make the framing inconsistent.
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(middle[0] - half[0], middle[0] + half[0])
        ax.set_ylim(middle[1] - half[1], middle[1] + half[1])

        degenerate = solution.kind is SolutionKind.AXIS_DEGENERATE
        label = f"{index + 1}"
        if generating:
            label += "  generating"
        if degenerate:
            label += "  axis-degenerate"
        ax.set_title(label, fontsize=9, color=DEGENERATE_COLOR if degenerate else "0.25")

        # Use a border as a secondary marker so readers can identify the four degenerate
        # configurations across the grid. Draw a rectangle because the helper disables the entire
        # axis, which prevents an individual spine from being restored.
        if degenerate:
            ax.add_patch(
                Rectangle(
                    (0.0, 0.0),
                    1.0,
                    1.0,
                    transform=ax.transAxes,
                    fill=False,
                    edgecolor=DEGENERATE_COLOR,
                    linewidth=1.2,
                    clip_on=False,
                )
            )

    worst = max(float(s.residual) for s in found)
    figure.suptitle(
        "Every configuration reaching the CRX-10iA pose of issue #1\n"
        f"{len(found)} solutions, all within the robot's motion range, "
        f"worst residual {worst:.1e}",
        fontsize=12,
    )
    # Use enough vertical padding to associate each title with its panel. Insufficient padding
    # places a title against the row above and can make a marked border appear to identify the wrong
    # configuration.
    figure.tight_layout(h_pad=2.2)
    return figure
