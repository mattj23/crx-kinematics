//! This module derives the target-pose quantities that are independent of the angle being solved.

use crate::{Crx, Iso3, Point3, Vector3};

/// The per-target quantities the solver works from.
///
/// `o5` is the origin of the fifth frame, reached from the flange by backing off `x2` along the
/// flange Z axis. `ax` and `ay` are the flange X and Y axes, which span the plane of the circle
/// that O4 travels on as J6 turns.
pub struct Setup<'a> {
    /// The robot being solved for.
    pub robot: &'a Crx,

    /// The pose the flange is to reach.
    pub target: Iso3,

    /// The origin of the fifth frame.
    pub o5: Point3,

    /// The flange origin, which is the origin of the target pose.
    pub o6: Point3,

    /// The flange X axis.
    pub ax: Vector3,

    /// The flange Y axis.
    pub ay: Vector3,
}

impl<'a> Setup<'a> {
    /// Prepares to solve `robot` for `target`.
    ///
    /// # Arguments
    ///
    /// * `robot`: the robot model
    /// * `target`: the desired pose of the flange, in the robot's world coordinate system
    ///
    /// returns: Setup
    pub fn new(robot: &'a Crx, target: &Iso3) -> Self {
        let rotation = target.rotation.to_rotation_matrix();
        Self {
            robot,
            target: *target,
            // O5 is a pure offset back along the flange axis, unaffected by J6.
            o5: target * Point3::new(0.0, 0.0, -robot.x2()),
            o6: target * Point3::origin(),
            ax: rotation * Vector3::x(),
            ay: rotation * Vector3::y(),
        }
    }

    /// The candidate O4 at angle `theta` on its circle, and the unit vector along the J5 axis that
    /// points from O5 to it.
    ///
    /// # Arguments
    ///
    /// * `theta`: the angle on the O4 circle, measured from the flange X axis
    ///
    /// returns: (Point3, Vector3)
    pub fn o4_and_axis(&self, theta: f64) -> (Point3, Vector3) {
        let u = self.ax * theta.cos() + self.ay * theta.sin();
        (self.o5 + u * self.robot.y1(), u)
    }

    /// The scalar constraint whose roots are the solutions.
    ///
    /// Three of the robot's constraints are linear in O3, so Cramer's rule gives O3 as a ratio of
    /// a vector to a determinant. The one constraint left over, that O3 is `z1` from the origin,
    /// is what this returns: it is zero exactly when the angle is a solution.
    ///
    /// Lengths are scaled by `1 / z1` before the products are formed, which keeps the magnitudes
    /// near unity and the quantities being differenced of comparable size.
    ///
    /// # Arguments
    ///
    /// * `theta`: the angle on the O4 circle
    ///
    /// returns: f64
    pub fn f(&self, theta: f64) -> f64 {
        let scale = 1.0 / self.robot.z1();
        let u = self.ax * theta.cos() + self.ay * theta.sin();
        let o4 = (self.o5.coords + u * self.robot.y1()) * scale;
        let w = Vector3::z().cross(&o4);

        let rhs_dot = (1.0 + o4.dot(&o4) - (self.robot.x1() * scale).powi(2)) / 2.0;
        let rhs_perp = u.dot(&o4);
        let numerator = u.cross(&w) * rhs_dot + w.cross(&o4) * rhs_perp;
        let determinant = o4.dot(&u.cross(&w));

        numerator.dot(&numerator) - determinant * determinant
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Crx;
    use crate::tests::{all_robots, random_joints};
    use approx::assert_relative_eq;

    /// The angle on the O4 circle at which a known configuration actually sits.
    pub fn true_theta(setup: &Setup, frames: &[Iso3; 6]) -> f64 {
        let o4 = frames[3] * Point3::new(setup.robot.x1(), 0.0, 0.0);
        let u = (o4 - setup.o5).normalize();
        u.dot(&setup.ay).atan2(u.dot(&setup.ax))
    }

    #[test]
    fn o5_is_where_the_fifth_frame_sits() {
        // O5 is obtained directly from the target. Compare it with a configuration whose fifth
        // frame is known to verify that calculation.
        for robot in all_robots() {
            for _ in 0..1000 {
                let joints = random_joints();
                let frames = robot.fk_all(&joints);
                let setup = Setup::new(&robot, &frames[5]);
                let expected = frames[4] * Point3::origin();

                assert_relative_eq!(setup.o5, expected, epsilon = 1e-12);
            }
        }
    }

    #[test]
    fn the_origins_of_a_robot_at_rest() {
        let robot = Crx::new_5ia();
        let frames = robot.fk_all(&[0.0; 6]);

        let o3 = frames[2] * Point3::origin();
        let o4 = frames[3] * Point3::new(robot.x1(), 0.0, 0.0);

        assert_relative_eq!(o3, Point3::new(0.0, 0.0, 410.0), epsilon = 1e-12);
        assert_relative_eq!(o4, Point3::new(430.0, 0.0, 410.0), epsilon = 1e-12);
    }

    #[test]
    fn o4_circle_passes_through_the_real_o4() {
        // Whatever configuration produced the pose, the O4 it puts in space has to lie on the
        // circle swept by this parameterization, and the axis vector must point at it.
        for robot in all_robots() {
            for _ in 0..1000 {
                let joints = random_joints();
                let frames = robot.fk_all(&joints);
                let setup = Setup::new(&robot, &frames[5]);
                let expected = frames[3] * Point3::new(robot.x1(), 0.0, 0.0);

                let theta = true_theta(&setup, &frames);
                let (o4, axis) = setup.o4_and_axis(theta);

                assert_relative_eq!(o4, expected, epsilon = 1e-9);
                assert_relative_eq!(axis, (expected - setup.o5).normalize(), epsilon = 1e-12);
            }
        }
    }

    #[test]
    fn the_constraint_vanishes_at_a_real_configuration() {
        // The angle a real configuration sits at must be a root, on every model.
        for robot in all_robots() {
            for _ in 0..1000 {
                let joints = random_joints();
                let frames = robot.fk_all(&joints);
                let setup = Setup::new(&robot, &frames[5]);
                let value = setup.f(true_theta(&setup, &frames));

                assert!(
                    value.abs() < 1e-12,
                    "f was {:e} at a real configuration",
                    value
                );
            }
        }
    }
}
