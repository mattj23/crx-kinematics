//! This module contains the tools to go backwards from a valid O3/O4 pair to the joint angles
//! of the solution which created it.

use crate::{end_adjust, Crx};
use engeom::{Point3, Iso3, Vector3};
use crate::internals::reduced::get_point_o5;

/// Given a robot configuration and a known valid pair of (O3, O4) points, calculate the angles of
/// the individual joint/frame rotations in radians.  This will then have to be converted to
/// degrees and the J2/J3 interaction accounted for.
///
/// To convert this to degrees, use `rad_to_joints`.
///
/// # Arguments
///
/// * `robot`: the robot model
/// * `o3`: the O3 point in the robot's local coordinate system
/// * `o4`: the O4 point in the robot's local coordinate system
/// * `iso`: the orientation of the end effector in the robot's local coordinate system
///
/// returns: [f64; 6]
pub fn calc_joint_radians(robot: &Crx, o3: &Point3, o4: &Point3, iso: &Iso3) -> [f64; 6] {
    let mut joints = [0.0; 6];
    let o6 = iso * Point3::origin();
    let o5 = get_point_o5(robot, iso);

    // Calculate J1 and the frame which brings O3 into the X-Z plane
    joints[0] = o4.y.atan2(o4.x);
    let f1 = Iso3::rotation(Vector3::z() * joints[0]);
    let o3_j1 = f1.inverse() * o3;

    // Calculate J2
    joints[1] = o3_j1.x.atan2(o3_j1.z);
    let f3 = f1
        * Iso3::translation(o3_j1.x, o3_j1.y, o3_j1.z)
        * Iso3::rotation(Vector3::y() * joints[1]);
    let o4_j3 = f3.inverse() * o4;

    // Calculate J3
    joints[2] = o4_j3.z.atan2(o4_j3.x);
    let f4 = f3
        * Iso3::translation(o4_j3.x, o4_j3.y, o4_j3.z)
        * Iso3::rotation(-Vector3::y() * joints[2]);
    let o5_j4 = f4.inverse() * o5;

    // Calculate J4
    joints[3] = o5_j4.z.atan2(-o5_j4.y);
    let f5 = f4
        * Iso3::translation(o5_j4.x, o5_j4.y, o5_j4.z)
        * Iso3::rotation(-Vector3::x() * joints[3]);
    let o6_j5 = f5.inverse() * o6;

    // Calculate J5
    joints[4] = o6_j5.z.atan2(o6_j5.x);
    let f6 = f5
        * Iso3::translation(o6_j5.x, o6_j5.y, o6_j5.z)
        * Iso3::rotation(-Vector3::y() * joints[4])
        * end_adjust();
    let o6_ztest = f6.inverse() * iso * Point3::new(1.0, 0.0, 0.0);

    // Calculate J6
    joints[5] = -o6_ztest.y.atan2(o6_ztest.x);

    joints
}

