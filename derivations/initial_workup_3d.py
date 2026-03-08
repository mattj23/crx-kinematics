"""
    Display the forward kinematics
"""

import numpy
from pyvista import Plotter
from pathlib import Path
from engeom.geom3 import Iso3, Mesh, Vector3, Point3
from engeom.plot import PyvistaPlotterHelper

from crx.robot import Robot, fanuc_end

_o4_n = 10000


def main():
    # robot = Robot.crx10ia()
    robot = Robot.crx5ia()
    # robot.set_joints([10, -80, 10, 20, -20, 45])
    # robot.set_joints([10, -80, 10, 60, -20, 45])
    robot.set_joints([0, 0, 0, 0, 0, 0])
    target_frame = robot.frames[-1]

    # Do the calculations
    # ========================================================================
    # The expected hypotenuse of the O3-O4-O5 triangle
    expected_len = numpy.sqrt(robot.y1 ** 2 + robot.x1 ** 2)

    # Find the position of O5
    o5 = target_frame @ Point3(0, 0, -robot.x2)
    o6 = target_frame @ Point3(0, 0, 0)
    print(o5)
    print(robot.frame_origin(4))

    # Plot
    # ========================================================================
    plotter = Plotter(window_size=(2000, 1500))
    helper = PyvistaPlotterHelper(plotter)
    robot.plot(helper)


    # Plot the robot origins of interest
    origins = [(list(robot.frame_origin(i)), f"o{i + 1}") for i in [0, 2, 3, 4, 5]]
    points, labels = zip(*origins)
    plotter.add_point_labels(
        numpy.array(points),
        labels,
        point_color="black",
        point_size=10,
        font_size=16,
        render_points_as_spheres=True
    )

    # Calculate the O4 candidate circle points
    alpha = numpy.linspace(0.0, 2.0 * numpy.pi, _o4_n)
    o4_points = numpy.array([[numpy.cos(x), numpy.sin(x), 0] for x in alpha]) * robot.y1
    o4_points[:, 2] = -robot.x2
    o4_points = robot.frames[-1].transform_points(o4_points)
    plotter.add_lines(o4_points, connected=True, color="red", width=1.0)

    # For a random O4, plot the O3 candidate circle
    p = Point3(*o4_points[_o4_n // 2])
    helper.add_points(p, color="blue", point_size=10, render_points_as_spheres=True)

    o4_iso = Iso3.from_basis_zy(p - o5, o6 - o5, o5) @ Iso3.from_translation(0, 0, robot.y1)
    helper.coordinate_frame(o4_iso, size=100)
    o3_points = o4_iso.transform_points(_xy_circle(robot.x1, _o4_n))
    plotter.add_lines(o3_points, connected=True, color="blue", width=1.0)

    # Figure out the position and radius of the O1 sphere intersection with the o4 xy plane
    o1_proj = o4_iso.inverse() @ Point3(0, 0, 0)
    center = o4_iso @ o1_proj.with_z(0)
    helper.add_points(center, color="red", point_size=10)

    # d is the distance from the center of the O1 sphere to the O4 xy plane. If this is greater than the radius of the
    # O1 sphere, there are no solutions for O2 and we can skip this O4 candidate.
    d = center.coords.norm()
    if d > robot.z1:
        print("No solutions for O2")
        return
    # The radius of the O1 circle in the O4 xy plane is the radius of the O1 sphere projected onto the plane, which is sqrt(r^2 - d^2)
    r = numpy.sqrt(robot.z1 ** 2 - d ** 2)
    o1_points = o4_iso.transform_points(_xy_circle(r, _o4_n, x=o1_proj.x, y=o1_proj.y))
    plotter.add_lines(o1_points, connected=True, color="green", width=3)


    # helper.coordinate_frame(o4_iso.inverse(), size=75)
    # print(o1_center)

    # O1 sphere
    sphere = Mesh.create_sphere(robot.z1, 100, 100)
    helper.add_mesh(sphere, color="green", opacity=0.25)

    # Figure out the cross-section




    # Add the coordinate frame
    helper.coordinate_frame(Iso3.identity(), size=100)
    plotter.add_axes()

    plotter.show()

def _xy_circle(radius: float, n: int, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    alpha = numpy.linspace(0.0, 2.0 * numpy.pi, n)
    points = numpy.array([[numpy.cos(t) * radius + x, numpy.sin(t) * radius + y, z] for t in alpha])
    return points


if __name__ == '__main__':
    main()
