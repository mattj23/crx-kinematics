import numpy
from .robot import Robot
from engeom.geom3 import Point3, Vector3, Iso3
from engeom.plot import MatplotlibAxesHelper, TraceBuilder


def o4_candidate(robot: Robot, angle: float) -> Point3:
    return robot.frames[-1] @ Point3(numpy.cos(angle) * robot.y1, numpy.sin(angle) * robot.y1, -robot.x2)


def xy_circle(radius: float, n: int, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    alpha = numpy.linspace(0.0, 2.0 * numpy.pi, n)
    points = numpy.array([[numpy.cos(t) * radius + x, numpy.sin(t) * radius + y, z] for t in alpha])
    return points


def draw_robot(robot: Robot, helper: MatplotlibAxesHelper, view: Iso3, show_origins: bool = False):
    if show_origins:
        origins = []
        frame_origins = [(robot.frame_origin(i), f"$O_{i + 1}$") for i in [0, 2, 3, 4, 5]]
        shift = Vector3(1, 0, 0) * 25
        for p, label in frame_origins:
            p = view @ p
            origins.append(list(p))
            helper.text(label, p + shift, color="red", horizontalalignment="center",
                        verticalalignment="center", fontsize=18, weight="bold")

        origins = numpy.array(origins)
        helper.ax.plot(origins[:, 0], origins[:, 1], "o", color="red", markersize=7)

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

    helper.ax.plot(*visible.xy, color="black", linewidth=0.5)
    helper.ax.plot(*hidden.xy, color="black", linewidth=0.25, alpha=0.125)


def optional_axis_label(helper: MatplotlibAxesHelper, working_iso: Iso3, view_iso: Iso3, length: float):
    x_label = working_iso @ Point3(length, 0, 0)
    y_label = working_iso @ Point3(0, length, 0)
    z_label = working_iso @ Point3(0, 0, length)

    x_view = view_iso @ x_label
    y_view = view_iso @ y_label
    z_view = view_iso @ z_label
    origin_view = view_iso @ working_iso @ Point3(0, 0, 0)

    if _visible(x_view):

        helper.arrow(origin_view.to_2d(), x_view.to_2d(), color="red", linewidth=2.0)
        helper.text(" $x$ ", x_view.to_2d(), color="red", fontsize=14)
    if _visible(y_view):
        helper.arrow(origin_view.to_2d(), y_view.to_2d(), color="green", linewidth=2.0)
        helper.text(" $y$ ", y_view.to_2d(), color="green", fontsize=14)
    if _visible(z_view):
        helper.arrow(origin_view.to_2d(), z_view.to_2d(), color="blue", linewidth=2.0)
        helper.text(" $z$ ", z_view.to_2d(), color="blue", fontsize=14)


def _visible(p0: Point3):
    return p0.with_z(0).coords.norm() > 1.0
