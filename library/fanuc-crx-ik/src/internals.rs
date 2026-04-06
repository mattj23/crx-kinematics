//! The internals module contains different chunks of the geometric method's implementation. They
//! are made publicly accessible in the main library for testing, debugging, and checking the
//! feasibility and assumptions of the method as new robots are added to the CRX product line.

use engeom::{Iso3, Point3};
use engeom::geom3::Circle3;
use crate::Crx;

pub mod reduced;
mod candidate_points;
mod extract_joints;

pub fn get_point_o5(robot: &Crx, target: &Iso3) -> Point3 {
    let point_o5 = target * Point3::new(0.0, 0.0, -robot.x2());
    point_o5
}

pub fn get_circle_c4(robot: &Crx, target: &Iso3) -> Circle3 {
    let c4_iso = target * Iso3::translation(0.0, 0.0, -robot.x2());
    Circle3::new(robot.y1(), c4_iso)
}

#[cfg(test)]
pub mod tests {
    use approx::assert_relative_eq;
    use engeom::Point3;
    use crate::Crx;
    use crate::internals::{ get_point_o5};
    use crate::tests::{all_robots, random_joints};

    pub fn get_o3(robot: &Crx, joints: &[f64; 6]) -> Point3 {
        let all_frames = robot.fk_all(joints);
        all_frames[2] * Point3::origin()
    }

    pub fn get_o4(robot: &Crx, joints: &[f64; 6]) -> Point3 {
        let all_frames = robot.fk_all(joints);
        all_frames[3] * Point3::new(robot.x1(), 0.0, 0.0)
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

    #[test]
    fn crx_5ia_zero_o3() {
        let robot = Crx::new_5ia();
        let joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        let expected_o3 = Point3::new(0.0, 0.0, 410.0);
        let check_o3 = get_o3(&robot, &joints);
        assert_relative_eq!(expected_o3, check_o3, epsilon = 1e-12);
    }

    #[test]
    fn crx_5ia_zero_o4() {
        let robot = Crx::new_5ia();
        let joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        let expected_o4 = Point3::new(430.0, 0.0, 410.0);
        let check_o4 = get_o4(&robot, &joints);
        assert_relative_eq!(expected_o4, check_o4, epsilon = 1e-12);
    }


}