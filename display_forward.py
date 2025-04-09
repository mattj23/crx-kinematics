"""
    Display the forward kinematics
"""

import numpy
from pyvista import Plotter
from pathlib import Path
from engeom.geom3 import Iso3, Mesh, Vector3, Point3
from engeom.plot import PyvistaPlotterHelper

from crx.robot import Robot


def main():
    robot = Robot.crx5ia()
    robot.set_joints([10, 10, 10, 10, 10, 10])

    plotter = Plotter()
    helper = PyvistaPlotterHelper(plotter)

    robot.plot(helper)

    origins = [(list(robot.frame_origin(i)), f"o{i + 1}") for i in [0, 2, 3, 4, 5]]
    points, labels = zip(*origins)
    plotter.add_point_labels(numpy.array(points), labels, point_color="black", point_size=10, font_size=16,
                             render_points_as_spheres=True)

    # O4 candidates
    raw_points = numpy.array([[numpy.cos(x), numpy.sin(x), 0] for x in numpy.linspace(0, 2 * numpy.pi, 500)]) * robot.y1
    raw_points[:, 2] = -robot.x2
    points = robot.frames[-1].transform_points(raw_points)
    plotter.add_lines(points, connected=True, color="red", width=1.0)

    # O3 candidates
    upper_trace = []
    lower_trace = []
    for xyz in points:
        p = Point3(*xyz)
        u, l = upper_lower(p, robot)
        upper_trace.append(list(u))
        lower_trace.append(list(l))

    plotter.add_lines(numpy.array(upper_trace), connected=True, color="green", width=1.0)
    plotter.add_lines(numpy.array(lower_trace), connected=True, color="blue", width=1.0)

    # Add the coordinate frame
    helper.coordinate_frame(Iso3.identity(), size=100)
    plotter.add_axes()
    plotter.show()


def upper_lower(p: Point3, robot: Robot):
    # Consider the local coordinate system ul (origin to point) and vl (orthogonal to ul) and the rotated coordinate
    # system ur (ul projected to x-y plane) and z (same as the global z-axis)
    d = p.coords
    ur = Vector3(d.x, d.y, 0).normalized()

    ul = d.normalized()
    vl = ((ul.dot(ur) * ul) - ur).normalized()

    # The height of the target point
    x, y = corner_xy(robot.z1, robot.x1, d.norm())

    return ul * x + vl * y, ul * x - vl * y


def corner_xy(r0: float, r1: float, d: float) -> tuple[float, float]:
    a = (d ** 2 + r0 ** 2 - r1 ** 2) / (2 * d)
    h = numpy.sqrt(r0 ** 2 - a ** 2)
    return a, h


if __name__ == '__main__':
    main()
