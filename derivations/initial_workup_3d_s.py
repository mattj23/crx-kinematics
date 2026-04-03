"""
    Display the forward kinematics
"""
import math

import numpy
from pyvista import Plotter
from pathlib import Path
from engeom.geom3 import Iso3, Mesh, Vector3, Point3, Circle3, Plane3, Sphere3, Line3
from engeom.geom2 import Circle2
from engeom.plot import PyvistaPlotterHelper
from derivations.initial_workup import *

from crx.robot import Robot, fanuc_end

_o4_n = 10000

# ROBOT_JOINTS = [10, 10, 10, 60, -90, 45]
# ROBOT_JOINTS = [10, -10, 5, 20, 20, 45]
# ROBOT_JOINTS = [0, 0, 0, 0, 0, 0]

# Known bad
# ROBOT_JOINTS = [10, -80, 10, 60, -20, 45]

ROBOT_JOINTS = [10, -90, 10, 20, 30, 45]
# ROBOT_JOINTS = [10, -50, 10, 0, 30, 45]

# ROBOT = Robot.crx10ia()
ROBOT = Robot.crx5ia()

PLOT_ROBOT_SETUP = False
PLOT_REDUCED = True


def main():
    robot = ROBOT
    robot.set_joints(ROBOT_JOINTS)

    point_o6 = robot.frame_origin(5)
    point_o5 = robot.frame_origin(4)
    frame_o6 = robot.frames[5]

    # This is the candidate o4 circle, consisting of all the points where the o4 center could be if the flange
    # rotates through its full 360-degree sweep
    o4_circle = Circle3(*point_o5, *frame_o6.z_direction, robot.y1)

    # The o4 sphere is the sphere that contains the o4 circle
    o4_sphere = Sphere3(*point_o5, radius=robot.y1)

    # The o3 sphere is the set of points that satisfy the o5 -> o4 -> o3 constraint. Imagine the flange is fixed in
    # space at the target position. The o4 joint spins like a propeller, rotating o3 around o4 like a propeller. Then,
    # while this is happening, the wrist joint rotates its full 360 degrees. The full set of points that o3 sweeps
    # through is the o3 sphere.
    o3_sphere = Sphere3(*point_o5, math.sqrt(robot.y1 ** 2 + robot.x1 ** 2))

    # The o1 sphere is the set of points that o3 can reach just by the first two joints moving through all of their
    # various configurations
    o1_sphere = Sphere3(*Point3.origin(), radius=robot.z1)

    # The o3 circle is the set of points at the intersection of the o3 sphere and the o1 sphere
    o3_circle = o3_sphere.intersect_sphere(o1_sphere)

    # Find the point closest to the o3 circle plane
    z_minus = o3_circle.center - point_o5
    theta = o4_circle.max_extent_angle(*z_minus)
    x_plus = o4_circle.at_angle(theta).point - point_o5

    reduced = Iso3.from_basis_xz(o3_circle.plane.project_vector(x_plus), -z_minus, o3_circle.center).inverse()


    # =================================================================================================================
    # Plot basic setup
    # =================================================================================================================
    if PLOT_ROBOT_SETUP:
        plotter = PyvistaPlotterHelper.with_new_plotter(window_size=(2000, 1500))
        robot.plot(plotter)

        # Plot the robot origins of interest
        origins = [(list(robot.frame_origin(i)), f"o{i + 1}") for i in [0, 2, 3, 4, 5]]
        points, labels = zip(*origins)
        plotter.pv.add_point_labels(
            numpy.array(points),
            labels,
            point_color="black",
            point_size=10,
            font_size=16,
            render_points_as_spheres=True
        )

        plotter.circle(o4_circle, edge_color="red", face_color=None, edge_width=1.5)
        plotter.sphere(o4_sphere, color="red", opacity=0.1)
        plotter.sphere(o3_sphere, color="orange", opacity=0.1)
        plotter.sphere(o1_sphere, color="green", opacity=0.1)
        plotter.circle(o3_circle, n=1000, edge_color="blue", face_color=None, edge_width=1.5)

        plotter.coordinate_frame(Iso3.identity(), size=100)
        plotter.show()

    # =================================================================================================================
    # Plot the reduced problem
    # =================================================================================================================
    if PLOT_REDUCED:
        plotter = PyvistaPlotterHelper.with_new_plotter(window_size=(2000, 1500))

        o3_circle_r: Circle3 = reduced @ o3_circle
        o4_circle_r: Circle3 = reduced @ o4_circle

        # Check if we go beyond x1
        if o4_circle_r.normal.dot(Vector3.x_axis()) < 0:
            o4_circle_r.flip_normal()

        p = o4_circle_r.plane.new_parallel(-robot.x1)
        end_trim = o3_circle_r.intersect_plane(p)
        if end_trim:
            plotter.points(*[o3_circle_r.at_angle(x).point for x in end_trim], color="green")


        plotter.circle(o3_circle_r, edge_color="blue", face_color=None, edge_width=1.5)
        plotter.circle(o4_circle_r, edge_color="red", face_color=None, edge_width=1.5)
        plotter.coordinate_frame(Iso3.identity(), size=100)


        plotter.show()



if __name__ == '__main__':
    main()
