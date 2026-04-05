//! The internals module contains different chunks of the geometric method's implementation. They
//! are made publicly accessible in the main library for testing, debugging, and checking the
//! feasibility and assumptions of the method as new robots are added to the CRX product line.

use crate::{Crx, IkResult};
use engeom::geom3::Circle3;
use engeom::{Iso3, Point3, Sphere3, Vector3};

pub struct CrxReduced {
    /// The radius of circle C3
    pub r3: f64,

    /// The height of the center of circle C4 above the origin
    pub h: f64,

    /// The `y1` kinematic parameter of the CRX robot
    pub y1: f64,

    /// The `x1` kinematic parameter of the CRX robot
    pub x1: f64,

    /// The tilt angle of C4
    pub phi: f64,

    /// The isometry that transforms points from the reduced problem coordinate system to the
    /// robot's world coordinate system.
    pub iso: Iso3,
}

impl CrxReduced {
    pub fn new(r3: f64, h: f64, y1: f64, x1: f64, phi: f64, iso: Iso3) -> Self {
        Self {
            r3,
            h,
            y1,
            x1,
            phi,
            iso,
        }
    }

    pub fn from_target(robot: &Crx, target: &Iso3) -> IkResult<Self> {
        // First, we need to find the point at O5. This is offset backwards from the target
        // frame origin by the `x2` kinematic parameter.
        let point_o5 = Point3::new(0.0, 0.0, -robot.x2());

        // Now we can create the S1 and S3 spheres
        let s1 = Sphere3::new(Point3::origin(), robot.z1());
        let s3 = Sphere3::new(point_o5, (robot.x1().powi(2) + robot.y1().powi(2)).sqrt());

        // Circle C3 is the intersection of the S1 and S3 spheres. If it is empty, the target is
        // out of reach, and we can propagate a `NoReach` result back to the caller.
        let Some(c3) = s1.intersect_sphere(&s3) else {
            return IkResult::NoReach;
        };

        // Circle C4 is the set of possible O4 points, formed by a circle of radius `y1` around
        // point O5 through an axis aligned with the target frame's Z axis.
        let c4_iso = target * Iso3::translation(0.0, 0.0, -robot.x2());
        let c4 = Circle3::new(robot.y1(), c4_iso);

        // The vector from the center of C3 to the point O5 will be used to generate a number of
        // other
        //

        todo!()
    }
}

pub fn get_point_o5(robot: &Crx, target: &Iso3) -> Point3 {
    let point_o5 = target * Point3::new(0.0, 0.0, -robot.x2());
    point_o5
}

pub fn get_circle_c4(robot: &Crx, target: &Iso3) -> Circle3 {
    let c4_iso = target * Iso3::translation(0.0, 0.0, -robot.x2());
    Circle3::new(robot.y1(), c4_iso)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::{all_robots, random_joints};
    use approx::assert_relative_eq;
    use engeom::common::points::dist;

    #[test]
    fn crx_5ia_zero_o5() {
        let robot = Crx::new_5ia();
        let joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let target = robot.fk(&joints);

        let expected_o5 = Point3::new(575.0 - robot.x2(), -130.0, 410.0);
        let o5 = get_point_o5(&robot, &target);
        assert_relative_eq!(o5, expected_o5, epsilon = 1e-12);
    }

    #[test]
    fn stress_check_point_o5() {
        for _ in 0..1000 {
            for robot in all_robots() {
                let joints = random_joints();
                let all_frames = robot.fk_all(&joints);
                let target = all_frames[5];
                let expected_o5 = all_frames[4] * Point3::origin();

                let o5 = get_point_o5(&robot, &target);
                assert_relative_eq!(o5, expected_o5, epsilon = 1e-12);
            }
        }
    }

    fn get_o4(robot: &Crx, joints: &[f64; 6]) -> Point3 {
        let all_frames = robot.fk_all(joints);
        all_frames[3] * Point3::new(robot.x1(), 0.0, 0.0)
    }

    #[test]
    fn crx_5ia_zero_o4() {
        let robot = Crx::new_5ia();
        let joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        let expected_o4 = Point3::new(430.0, 0.0, 410.0);
        let check_o4 = get_o4(&robot, &joints);
        assert_relative_eq!(expected_o4, check_o4, epsilon = 1e-12);
    }

    #[test]
    fn stress_check_circle_c4() {
        for _ in 0..1000 {
            for robot in all_robots() {
                let joints = random_joints();
                let all_frames = robot.fk_all(&joints);

                let target = all_frames[5];
                let c4 = get_circle_c4(&robot, &target);
                let o4 = get_o4(&robot, &joints);

                let closest = c4.closest_position(&o4).point;
                let d = dist(&o4, &closest);
                assert!(d < 1e-10, "Distance is {}", d);
            }
        }
    }
}
