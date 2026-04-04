import numpy
import math
from crx.robot import Robot, RobotKinematics
from engeom.geom2 import Circle2, Point2, signed_angle
from engeom.geom3 import Circle3, Iso3, Plane3, Sphere3, Point3, Vector3, Line3
from engeom.plot import PyvistaPlotterHelper, TraceBuilder
from tqdm import tqdm

from derivations.initial_workup_3d import intersections


def main():
    random_joints = numpy.random.uniform(-180, 180, size=(20_000, 6))
    robots = [RobotKinematics.crx5ia(), RobotKinematics.crx10ia(), RobotKinematics.crx10ial(), RobotKinematics.crx30ia()]

    process(robots[2], random_joints)

    # robot = RobotKinematics.crx10ial()
    # for robot in robots:
    #     process(robot, random_joints)

def process(robot: RobotKinematics, random_joints: numpy.ndarray):
    for j in tqdm(random_joints):
        robot.set_joints(j)
        point_o5 = robot.frame_origin(4)
        frame_o6 = robot.frames[5]

        # The o3 sphere is the set of points that satisfy the o5 -> o4 -> o3 constraint. Imagine the flange is fixed in
        # space at the target position. The o4 joint spins like a propeller, rotating o3 around o4 like a propeller. Then,
        # while this is happening, the wrist joint rotates its full 360 degrees. The full set of points that o3 sweeps
        # through is the o3 sphere.
        s3 = Sphere3(*point_o5, math.sqrt(robot.y1 ** 2 + robot.x1 ** 2))

        # The o1 sphere is the set of points that o3 can reach just by the first two joints moving through all of their
        # various configurations
        s1 = Sphere3(*Point3.origin(), radius=robot.z1)

        # The o3 circle is the set of points at the intersection of the o3 sphere and the o1 sphere
        c3 = s3.intersect_sphere(s1)
        c3_o5 = point_o5 - c3.center

        # This is the candidate o4 circle, consisting of all the points where the o4 center could be if the flange
        # rotates through its full 360-degree sweep. If the normal points in the opposite direction as c3_o5 we'll
        # flip the normal.
        c4 = Circle3(*point_o5, *frame_o6.z_direction, robot.y1)
        if c4.normal.dot(c3_o5) < 0:
            c4.flip_normal()

        # Find the point closest to the o3 circle plane
        z_minus = c3.center - point_o5  # Get the new -z direction
        theta = c4.max_extent_angle(*z_minus)  # Get the angle of the "lowest" point in the circle
        x_plus = c4.at_angle(theta).point - c4.center

        # Now we'll get the transformation to the reduced problem space. The origin will be at the center of C3, the
        # x direction points towards the lowest point of the circle, and the Z direction points from C3 to C4.
        # TODO: Check for and handle parallel or perpendicular circles
        reduced = Iso3.from_basis_xz(c3.plane.project_vector(x_plus), -z_minus, c3.center)

        # Extract reduced problem parameters
        r_3 = c3.r
        h = c3_o5.norm()
        y1 = robot.y1
        x1 = robot.x1
        phi = c3_o5.angle(c4.normal)

        points = reduced_problem(r_3, h, y1, x1, phi, reduced)[:, :3]

        # plot = PyvistaPlotterHelper.with_new_plotter(window_size=(1000, 1000))
        # plot.circle(c4, edge_color="red", face_color=None)
        # plot.circle(c3, edge_color="blue", face_color=None)
        # plot.pv.add_points(reduced.transform_points(points), color="green", point_size=10)
        # plot.coordinate_frame(Iso3.identity(), size=100)
        # plot.show()


def reduced_problem(r_3: float, h: float, y1: float, x1: float, phi: float, iso_reduced: Iso3):
    # We'll create the C4 circle at the origin so that we can clock its theta, then we'll pitch it by phi and
    # lift it up by h
    c4_at_origin = Circle3(0, 0, 0, 0, 0, 1, y1)
    angle_x = c4_at_origin.max_extent_angle(1, 0, 0)
    c4_at_origin.set_zero_angle(angle_x)
    c4 = Iso3.from_translation(0, 0, h) @ Iso3.from_ry(phi) @ c4_at_origin

    # We'll perform some orientation assertions to make sure the C4 circle is oriented correctly
    c4_orientation_check(c4)

    # The C3 circle is just a circle at the origin with radius r_3
    c3 = Circle3(0, 0, 0, 0, 0, 1, r_3)

    # Now we're going to perform the toroid equivalent check to see if we need to clip C4
    upr, lwr = toroid_equiv(h, r_3, x1, y1)

    upr_theta = c4_limit_check(upr, c4, math.pi)
    lwr_theta = c4_limit_check(lwr, c4, 0.0)

    # Get the relevant O4 candidates
    angles = numpy.linspace(upr_theta, lwr_theta, 500)
    cand_o4 = c4.at_angles(angles)

    # Perform the intersection
    trace_cw = TraceBuilder()
    trace_ccw = TraceBuilder()
    cand_o3_cw = []
    cand_o3_ccw = []
    for row in cand_o4:
        plane = Plane3.from_point_normal(*row[:6])
        intr = c3.intersect_plane(plane)
        intr_points = [c3.at_angle(t).point.coords for t in intr]

        # assert len(intr) > 0
        if len(intr) == 0:
            # display_reduced(c3, c4, cand_o4)
            continue

        # There will now be either 1 or 2 intersections. If there is only one, we will add it to both the clockwise
        # and counter-clockwise lists. If there are two, we'll identify cw and ccw and then add the appropriate
        # point to each list.
        if len(intr_points) == 1:
            cand_o3_cw.append(intr_points[0])
            cand_o3_ccw.append(intr_points[0])
            trace_cw.add_segment(row[:3], intr_points[0])
            trace_ccw.add_segment(row[:3], intr_points[0])
        else:
            center_vector = Vector3(*row[3:6]).to_2d()
            if signed_angle(center_vector, intr_points[0].to_2d()) < 0:
                cand_o3_cw.append(intr_points[0])
                cand_o3_ccw.append(intr_points[1])
                trace_cw.add_segment(row[:3], intr_points[0])
                trace_ccw.add_segment(row[:3], intr_points[1])
            else:
                cand_o3_cw.append(intr_points[1])
                cand_o3_ccw.append(intr_points[0])
                trace_cw.add_segment(row[:3], intr_points[1])
                trace_ccw.add_segment(row[:3], intr_points[0])

        # This is a check to make sure that they are indeed symmetrical around the projected center vector, which a
        # stress test on every robot seemed to confirm
        # angles = sorted([signed_angle(center_vector, x.to_2d()) for x in intr_points])
        # assert abs(sum(angles)) < 1e-6

        # print(angles)
    # results = numpy.array(results)
    plot = PyvistaPlotterHelper.with_new_plotter(window_size=(1000, 1000))
    plot.circle(c4, edge_color="red", face_color=None)
    plot.circle(c3, edge_color="blue", face_color=None)
    plot.pv.add_lines(cand_o4[:, :3], connected=True, color="green", width=10)
    plot.pv.add_lines(trace_cw., color="purple")
    plot.coordinate_frame(Iso3.identity(), size=100)
    plot.show()


    raise NotImplementedError()

    return cand_o4

def display_reduced(c3: Circle3, c4: Circle3, cand_o4: numpy.ndarray):
    plot = PyvistaPlotterHelper.with_new_plotter(window_size=(1000, 1000))
    plot.circle(c4, edge_color="red", face_color=None)
    plot.circle(c3, edge_color="blue", face_color=None)
    plot.pv.add_lines(cand_o4[:, :3], connected=True, color="green", width=10)
    plot.coordinate_frame(Iso3.identity(), size=100)
    plot.show()


def c4_limit_check(value: float | None, c4: Circle3, default_value: float) -> float:
    if value is None:
        return default_value

    # Note that this may not produce any intersections. This is because the toroidal equivalent check was performed on
    # a sphere, and the actual C4 circle may not go as high or low as the intersection with the sphere. That's OK; the
    # point was to preclude a check if it wasn't necessary.
    angles = c4.intersect_plane(Plane3(0, 0, 1, value))
    if len(angles) == 0:
        return default_value

    # We'll return the positive angle
    return max(angles)


def toroid_equiv(h: float, r_3: float, x1: float, y1: float):
    # To do the toroid equivalent check, we'll start with the original sphere that C4 is a subset of, making the
    # check entirely axisymmetric. We imagine any cross-section of the donut and the sphere through the Z axis. The
    # sphere is a 2d circle sitting at z=h and with radius y1. The donut D is a circle of radius x1 centered at
    # (r_3, 0).
    d = Circle2(r_3, 0, x1)
    s4 = Circle2(0, h, y1)
    intr = d.intersections_with(s4)
    if len(intr) > 0:
        intr.sort(key=lambda p: p.y)

    # If the topmost point of S4 is inside the circle, the entirety of C4 is reachable from C3. However, if it isn't,
    # all points of C4 above the z value of the topmost intersection of S4 and D are unreachable.
    upper_limit = None
    if not d.contains_point(0, h + y1):
        upper_limit = intr[-1].y

    # The lower limit can be reached if the lowermost point of S4 is no longer inside the circle. This can happen on
    # the /L models, with their long limbs allowing for the case where the only reachable points left on C4 are
    # on the diagonal.
    lower_limit = None
    if not d.contains_point(0, h - y1):
        lower_limit = intr[0].y

    return upper_limit, lower_limit


def c4_orientation_check(c4: Circle3):
    # Check that the clocking of the C4 circle is correct. The point at theta=0 should have a positive x component, no
    # y component, and a z component that is less than or equal to the point at theta=pi

    p = c4.at_angle(0).point
    assert abs(p.y) < 1e-6
    assert abs(p.x) > -1e-6
    assert p.z <= c4.at_angle(math.pi).point.z


if __name__ == '__main__':
    main()
