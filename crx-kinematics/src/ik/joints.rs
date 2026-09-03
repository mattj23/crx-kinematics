//! This module calculates joint angles from located O3 and O4 points.
//!
//! With both origins known, the chain can be walked forward one frame at a time, each joint angle
//! read from where the next origin sits in the frame built so far.

use super::setup::Setup;
use crate::ik::geometry::AXIS_TOL;
use crate::{Iso3, Point3, Vector3, end_adjust, wrap_pi};
use std::f64::consts::PI;

/// The two joint vectors that put the flange on the target through a given O3 and O4.
///
/// Both members of the J1 front and back pair are returned: the same pose is reached with the base
/// turned half a turn and the shoulder, elbow and forearm roll reflected to match.
///
/// # Arguments
///
/// * `setup`: the prepared target, which supplies O5 and O6
/// * `o3`: where O3 has been located
/// * `o4`: where O4 has been located
///
/// returns: [[f64; 6]; 2], joint angles in FANUC controller degrees
pub fn joints_from_points(setup: &Setup, o3: &Point3, o4: &Point3) -> [[f64; 6]; 2] {
    let mut j = [0.0f64; 6];

    // J1 turns the arm into the vertical plane that holds O3 and O4, so either point fixes it. O4
    // usually provides this angle. When O4 is on the axis and has no azimuth, O3 provides it. If
    // both points are on the axis, J1 is free and any value is valid.
    let on_axis = AXIS_TOL * setup.robot.z1();
    j[0] = if o4.x.hypot(o4.y) > on_axis {
        o4.y.atan2(o4.x)
    } else if o3.x.hypot(o3.y) > on_axis {
        o3.y.atan2(o3.x)
    } else {
        0.0
    };

    let f1 = Iso3::rotation(Vector3::z() * j[0]);
    let local_o3 = f1.inverse_transform_point(o3);
    j[1] = local_o3.x.atan2(local_o3.z);
    let f3 = f1
        * Iso3::translation(local_o3.x, local_o3.y, local_o3.z)
        * Iso3::rotation(Vector3::y() * j[1]);

    let local_o4 = f3.inverse_transform_point(o4);
    j[2] = local_o4.z.atan2(local_o4.x);
    let f4 = f3
        * Iso3::translation(local_o4.x, local_o4.y, local_o4.z)
        * Iso3::rotation(-Vector3::y() * j[2]);

    let local_o5 = f4.inverse_transform_point(&setup.o5);
    j[3] = local_o5.z.atan2(-local_o5.y);
    let f5 = f4
        * Iso3::translation(local_o5.x, local_o5.y, local_o5.z)
        * Iso3::rotation(-Vector3::x() * j[3]);

    let local_o6 = f5.inverse_transform_point(&setup.o6);
    j[4] = local_o6.z.atan2(local_o6.x);
    let f6 = f5
        * Iso3::translation(local_o6.x, local_o6.y, local_o6.z)
        * Iso3::rotation(-Vector3::y() * j[4])
        * end_adjust();

    let flange_x = f6.inverse_transform_vector(&(setup.target * Vector3::x()));
    j[5] = -flange_x.y.atan2(flange_x.x);

    for angle in j.iter_mut() {
        *angle = wrap_pi(*angle);
    }
    alternate_pair(&j).map(|radians| rad_to_joints(&radians))
}

/// The J1 front and back pair.
///
/// Swinging the base half a turn and mirroring the shoulder, elbow and forearm roll leaves the
/// wrist center and the flange exactly where they were.
///
/// # Arguments
///
/// * `radians`: kinematic joint angles, not controller angles
///
/// returns: [[f64; 6]; 2]
pub fn alternate_pair(radians: &[f64; 6]) -> [[f64; 6]; 2] {
    let mut flipped = *radians;
    flipped[0] = wrap_pi(radians[0] - PI);
    flipped[1] = wrap_pi(-radians[1]);
    flipped[2] = wrap_pi(PI - radians[2]);
    flipped[3] = wrap_pi(radians[3] - PI);
    [*radians, flipped]
}

/// Converts kinematic joint angles in radians back to FANUC controller degrees, undoing the J2/J3
/// coupling.
///
/// # Arguments
///
/// * `radians`: the kinematic joint angles
///
/// returns: [f64; 6]
pub fn rad_to_joints(radians: &[f64; 6]) -> [f64; 6] {
    let mut joints = radians.map(f64::to_degrees);
    joints[2] -= joints[1];
    joints
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ik::polish::joint_distance;
    use crate::tests::{all_robots, random_joints};
    use crate::{Crx, Point3};

    /// The O3 and O4 that a known configuration puts in space.
    fn origins(robot: &Crx, joints: &[f64; 6]) -> (Point3, Point3) {
        let frames = robot.fk_all(joints);
        (
            frames[2] * Point3::origin(),
            frames[3] * Point3::new(robot.x1(), 0.0, 0.0),
        )
    }

    /// The candidate nearest `expected`. Both members of the pair reach the same pose, so
    /// recovering the configuration that produced it requires identifying the corresponding one.
    fn closest(expected: &[f64; 6], candidates: &[[f64; 6]; 2]) -> [f64; 6] {
        if joint_distance(&candidates[0], expected) <= joint_distance(&candidates[1], expected) {
            candidates[0]
        } else {
            candidates[1]
        }
    }

    #[test]
    fn recover_zero_joints() {
        let robot = Crx::new_5ia();
        let joints = [0.0; 6];
        let target = robot.fk(&joints);
        let (o3, o4) = origins(&robot, &joints);
        let setup = Setup::new(&robot, &target);

        let result = closest(&joints, &joints_from_points(&setup, &o3, &o4));
        assert!(joint_distance(&result, &joints) < 1e-12);
    }

    #[test]
    fn recover_random_joints() {
        for robot in all_robots() {
            for _ in 0..1000 {
                let joints = random_joints();
                let target = robot.fk(&joints);
                let (o3, o4) = origins(&robot, &joints);
                let setup = Setup::new(&robot, &target);
                let result = closest(&joints, &joints_from_points(&setup, &o3, &o4));

                // Compare these values as angles. The J2/J3 coupling can place the recovered J3 a
                // full turn from the value that produced the pose, while representing the same
                // configuration.
                //
                // The tolerance is one millionth of a radian because the joints are read from the
                // chain one at a time. A near-singular draw in which two joints compensate for each
                // other loses several digits. The joint-space polish outside this module reduces
                // the remaining error.
                assert!(
                    joint_distance(&result, &joints) < 1e-6,
                    "recovered {:?} from {:?}",
                    result,
                    joints
                );
            }
        }
    }
}
