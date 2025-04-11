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
    robot = Robot.crx5ia()
    robot.set_joints([10, -80, 10, 20, -20, 45])
    target_frame = robot.frames[-1]

    # Do the calculations
    # ========================================================================
    # The expected hypotenuse of the O3-O4-O5 triangle
    expected_len = numpy.sqrt(robot.y1 ** 2 + robot.x1 ** 2)

    # Find the position of O5
    o5 = target_frame @ Point3(0, 0, -robot.x2)
    print(o5)
    print(robot.frame_origin(4))

    # Calculate the O4 candidate circle points
    alpha = numpy.linspace(0.0, 2.0 * numpy.pi, _o4_n)
    o4_points = numpy.array([[numpy.cos(x), numpy.sin(x), 0] for x in alpha]) * robot.y1
    o4_points[:, 2] = -robot.x2
    o4_points = robot.frames[-1].transform_points(o4_points)

    # O3 candidates
    upper_trace = []
    lower_trace = []
    valid_pairs = []

    for xyz in o4_points:
        p = Point3(*xyz)
        u, l = upper_lower(p, robot)

        if abs((o5 - Point3(*u)).norm() - expected_len) < 0.05:
            valid_pairs.append((p, u))

        if abs((o5 - Point3(*l)).norm() - expected_len) < 0.05:
            valid_pairs.append((p, l))

        upper_trace.append(list(u))
        lower_trace.append(list(l))

    # Valid pairs are the functioning O4 and O3 combinations which meet the expected
    # length of the distance between O3 and O5. We can use them to calculate the
    # joint angles.
    joint_sets = []
    for o4, o3 in valid_pairs:
        joints = []
        # Joint 1 is based on the projection of either O3 or O4 onto the x-y plane
        j1 = numpy.arctan2(o4.y, o4.x)
        print(f"Joint 1: {numpy.degrees(j1)}")
        joints.append(numpy.degrees(j1))

        # Now we can make a frame for J1 which will bring O3 into the X-Z plane
        frame1 = Iso3.from_rotation(j1, 0, 0, 1)
        o3_j1 = frame1.inverse() @ o3

        # J2 is the angle from the Z axis to O3, clockwise
        j2 = numpy.arctan2(o3_j1.x, o3_j1.z)
        print(f"Joint 2: {numpy.degrees(j2)}")
        joints.append(numpy.degrees(j2))

        # Now we can make a frame for J3 which will bring O4 into the X-Z plane
        # with X aligned by J2
        frame3 = frame1 @ Iso3.from_translation(*o3_j1) @ Iso3.from_rotation(j2, 0, 1, 0)
        o4_j3 = frame3.inverse() @ o4
        print(o4_j3)

        # J3 is the angle from the X axis to O4, counter-clockwise
        j3 = numpy.arctan2(o4_j3.z, o4_j3.x)
        print(j3)
        print(f"Joint 3: {numpy.degrees(j3 - j2)}")
        joints.append(numpy.degrees(j3 - j2))

        # Now we can create the frame for J4 which will bring O5 into the Y-Z plane
        frame4 = frame3 @ Iso3.from_translation(*o4_j3) @ Iso3.from_rotation(j3, 0, -1, 0)
        o5_j4 = frame4.inverse() @ o5

        # J4 is the angle from O5 to the Y axis
        j4 = numpy.arctan2(o5_j4.z, -o5_j4.y)
        print(f"Joint 4: {numpy.degrees(j4)}")
        joints.append(numpy.degrees(j4))

        # Now we can create the frame for J5 which will bring O6 into the X-Z plane
        frame5 = frame4 @ Iso3.from_translation(*o5_j4) @ Iso3.from_rotation(j4, -1, 0, 0)
        test_o6 = target_frame @ Point3(0, 0, 0)
        o6_j5 = frame5.inverse() @ test_o6

        # J5 is the angle from x in the X-Z plane to O6
        j5 = numpy.arctan2(o6_j5.z, o6_j5.x)
        print(f"Joint 5: {numpy.degrees(j5)}")
        joints.append(numpy.degrees(j5))

        # Finally, we can create the frame for J5 which will overlap with the target frame
        # and allow us to calculate the final J6 rotation. We'll need to apply the fanuc end
        # orientation isometry to make the axes line up.
        frame6 = frame5 @ Iso3.from_translation(*o6_j5) @ Iso3.from_rotation(j5, 0, -1, 0) @ fanuc_end()

        # Now we can find the z rotation difference between the target frame and frame6
        o6_ztest = frame6.inverse() @ target_frame @ Point3(1, 0, 0)
        j6 = numpy.arctan2(-o6_ztest.y, o6_ztest.x)
        print(f"Joint 6: {numpy.degrees(j6)}")
        joints.append(numpy.degrees(j6))

        joint_sets.append(joints)

    # Plot
    # ========================================================================
    plotter = Plotter()
    helper = PyvistaPlotterHelper(plotter)

    helper.coordinate_frame(frame6, size=50)
    # helper.coordinate_frame(frame5, size=75)

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

    # O4 candidates
    plotter.add_lines(o4_points, connected=True, color="red", width=1.0)

    # O3 candidates
    plotter.add_lines(numpy.array(upper_trace), connected=True, color="green", width=1.0)
    plotter.add_lines(numpy.array(lower_trace), connected=True, color="blue", width=1.0)

    # Pair candidates
    pairs = []
    for a, b in valid_pairs:
        pairs.append(list(a))
        pairs.append(list(b))
    plotter.add_lines(numpy.array(pairs), color="orange", width=1.0)

    # Add the coordinate frame
    helper.coordinate_frame(Iso3.identity(), size=100)
    plotter.add_axes()

    i = 0
    def next_solution():
        nonlocal i
        i = (i + 1) % len(joint_sets)
        robot.set_joints(joint_sets[i])
        robot.plot(helper)

    plotter.add_key_event("n", next_solution)

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
