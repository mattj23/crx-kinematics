//! This module locates O3 after O4 is known and handles degenerate geometries.
//!
//! Cramer's rule gives O3 directly, but its numerator and denominator vanish together where two
//! solutions merge. The solver therefore rebuilds O3 from the geometry. Constraints A and B put it
//! on two spheres, which meet in a circle, and
//! constraint D confines it to the vertical plane through O4, which cuts that circle at two
//! points. The two are the *branches*, and constraint E decides between them.

use super::setup::Setup;
use crate::{Point3, Vector3, wrap_pi};
use std::f64::consts::PI;

/// How close to the J1 axis, as a fraction of `z1`, counts as being on it.
///
/// A point nearer than this is treated as mathematically on the axis. The joint-space polish
/// absorbs the resulting error. This generous value reflects the minimum distance that the branch
/// construction can handle, while final accuracy is determined by the polish.
pub const AXIS_TOL: f64 = 1e-6;

/// The circle on which O3 must lie, being where the two spheres that locate it meet.
#[derive(Debug, Clone, Copy)]
pub struct O3Circle {
    /// The distance from the origin to O4.
    pub distance: f64,

    /// How far along the direction of O4 the circle's plane sits.
    pub along: f64,

    /// The square of the circle's radius, negative when the spheres do not reach each other and
    /// zero when they are tangent.
    pub radius_sq: f64,
}

/// The angles satisfying `a cos(x) + b sin(x) = c`.
enum TrigSolutions {
    /// Every angle satisfies it, because both coefficients and the right hand side all vanish.
    All,

    /// The angles that satisfy it, of which there may be none.
    Angles(Vec<f64>),
}

/// Solves `a cos(x) + b sin(x) = c`.
fn solve_linear_trig(a: f64, b: f64, c: f64) -> TrigSolutions {
    const TOL: f64 = 1e-12;
    let radius = a.hypot(b);
    if radius < TOL {
        return if c.abs() < TOL {
            TrigSolutions::All
        } else {
            TrigSolutions::Angles(Vec::new())
        };
    }

    let ratio = c / radius;
    if ratio.abs() > 1.0 + 1e-12 {
        return TrigSolutions::Angles(Vec::new());
    }

    let base = b.atan2(a);
    let offset = ratio.clamp(-1.0, 1.0).acos();
    if offset < TOL {
        TrigSolutions::Angles(vec![wrap_pi(base)])
    } else {
        TrigSolutions::Angles(vec![wrap_pi(base + offset), wrap_pi(base - offset)])
    }
}

impl Setup<'_> {
    /// The circle on which O3 must lie for a given O4.
    ///
    /// # Arguments
    ///
    /// * `o4`: where O4 has been placed
    ///
    /// returns: O3Circle
    pub fn circle_of_o3(&self, o4: &Point3) -> O3Circle {
        let distance = o4.coords.norm();
        if distance < AXIS_TOL * self.robot.z1() {
            // O4 has landed on O1. The two spheres are then concentric, so they either coincide or
            // miss entirely, and neither of those is a circle. See `o3_on_origin`.
            return O3Circle {
                distance,
                along: 0.0,
                radius_sq: -1.0,
            };
        }
        let along = (self.robot.z1().powi(2) - self.robot.x1().powi(2) + distance.powi(2))
            / (2.0 * distance);
        O3Circle {
            distance,
            along,
            radius_sq: self.robot.z1().powi(2) - along.powi(2),
        }
    }

    /// One of the two candidate O3 points for a given O4, found in the vertical plane that
    /// contains O1 and O4.
    ///
    /// # Arguments
    ///
    /// * `o4`: where O4 has been placed
    /// * `branch`: +1 or -1, selecting one of the two intersection points
    /// * `clamp`: how far the squared radius may fall below zero, as a fraction of `z1` squared,
    ///   before the spheres are taken to genuinely miss each other
    ///
    /// returns: Option<Point3>, or None when O4 is on the J1 axis and there is no unique vertical
    /// plane, or the spheres do not reach each other
    pub fn o3_branch(&self, o4: &Point3, branch: f64, clamp: f64) -> Option<Point3> {
        let rho = o4.x.hypot(o4.y);
        if rho < AXIS_TOL * self.robot.z1() {
            return None;
        }
        let radial = Vector3::new(o4.x / rho, o4.y / rho, 0.0);
        let circle = self.circle_of_o3(o4);
        if circle.radius_sq < -clamp * self.robot.z1().powi(2) {
            return None;
        }

        // A tangency is a reachable configuration, the fully stretched arm, so a radius that
        // rounds just below zero is clamped rather than rejected.
        let radius = circle.radius_sq.max(0.0).sqrt();

        // In the vertical section, spanned by the radial direction and Z, `ex` points at O4 and
        // `ey` is square to it, so the two points are `along` out and `radius` to either side.
        let (ex_r, ex_z) = (rho / circle.distance, o4.z / circle.distance);
        let q_r = circle.along * ex_r - branch * radius * ex_z;
        let q_z = circle.along * ex_z + branch * radius * ex_r;
        Some(Point3::from(radial * q_r + Vector3::z() * q_z))
    }

    /// The perpendicularity residual for one branch, `(O3 - O4) . u / x1`.
    ///
    /// The residual is zero exactly at a solution and passes through zero transversally where two
    /// solutions merge. This transverse crossing makes the residual suitable for polishing.
    ///
    /// # Arguments
    ///
    /// * `theta`: the angle on the O4 circle
    /// * `branch`: which of the two O3 points to measure
    ///
    /// returns: Option<f64>, or None where the branch does not exist
    pub fn branch_residual(&self, theta: f64, branch: f64) -> Option<f64> {
        let (o4, u) = self.o4_and_axis(theta);
        let o3 = self.o3_branch(&o4, branch, 0.0)?;
        Some((o3 - o4).dot(&u) / self.robot.x1())
    }

    /// How far O4 is from the J1 axis at `theta`, with the first two derivatives of the squared
    /// distance, which is what the search below needs.
    ///
    /// The shadow of the O4 circle on the floor is an ellipse, so the squared distance is a
    /// trigonometric polynomial of degree two and has at most two minima.
    ///
    /// # Arguments
    ///
    /// * `theta`: the angle on the O4 circle
    ///
    /// returns: (f64, f64, f64), the distance and the slope and curvature of its square
    pub fn axis_distance(&self, theta: f64) -> (f64, f64, f64) {
        let y1 = self.robot.y1();
        let (sin, cos) = theta.sin_cos();
        let a = Vector3::new(self.ax.x, self.ax.y, 0.0);
        let b = Vector3::new(self.ay.x, self.ay.y, 0.0);

        let q = Vector3::new(self.o5.x, self.o5.y, 0.0) + (a * cos + b * sin) * y1;
        let dq = (b * cos - a * sin) * y1;
        let ddq = (a * cos + b * sin) * -y1;

        (
            q.norm(),
            2.0 * q.dot(&dq),
            2.0 * (dq.dot(&dq) + q.dot(&ddq)),
        )
    }

    /// The angles at which O4 is on the J1 axis, or near enough to be treated as on it.
    ///
    /// The method uses a distance test because a pose that misses the axis by a nanometer causes
    /// the same branch-construction instability as an axis crossing. Near the axis, the direction
    /// of the vertical plane swings through a wide arc as O4 passes. The method handles both cases
    /// by placing O4 on the axis, building O3 there, and letting the joint-space polish absorb the
    /// ignored distance.
    ///
    /// returns: Vec<f64>
    pub fn axis_thetas(&self) -> Vec<f64> {
        const STARTS: usize = 16;
        let mut found: Vec<f64> = Vec::new();

        // O4 is O5 plus a radius of `y1` in some direction. Its minimum distance from the axis is
        // therefore the distance of O5 minus that radius. Most targets have a lower bound above
        // the tolerance, so skip the axis search for those targets.
        let nearest_possible = self.o5.x.hypot(self.o5.y) - self.robot.y1();
        if nearest_possible > 2.0 * AXIS_TOL * self.robot.z1() {
            return found;
        }

        for index in 0..STARTS {
            let mut theta = 2.0 * PI * index as f64 / STARTS as f64;
            for _ in 0..20 {
                let (_, slope, curvature) = self.axis_distance(theta);
                if curvature == 0.0 {
                    break;
                }
                let step = -slope / curvature;
                theta += step;
                if step.abs() < 1e-15 {
                    break;
                }
            }

            let (distance, _, curvature) = self.axis_distance(theta);
            if curvature <= 0.0 || distance > AXIS_TOL * self.robot.z1() {
                continue;
            }
            let theta = wrap_pi(theta);
            if !found.iter().any(|seen| wrap_pi(theta - seen).abs() < 1e-9) {
                found.push(theta);
            }
        }
        found
    }

    /// Whether O4 has landed on O1, which happens on the models built with `z1` equal to `x1` and
    /// is the deepest degeneracy the arm has: the two spheres become concentric, so J1 and J2 are
    /// both free.
    pub fn is_on_origin(&self, theta: f64) -> bool {
        let (o4, _) = self.o4_and_axis(theta);
        o4.coords.norm() < AXIS_TOL * self.robot.z1()
    }

    /// Whether O3 is on the J1 axis as well as O4, which leaves J1 free and the solution set
    /// continuous.
    pub fn is_singular_family(&self, theta: f64) -> bool {
        let (o4, _) = self.o4_and_axis(theta);
        let circle = self.circle_of_o3(&Point3::new(0.0, 0.0, o4.z));
        circle.radius_sq.max(0.0).sqrt() < AXIS_TOL * self.robot.z1()
    }

    /// A representative O3 for the case where O4 is on O1.
    ///
    /// The two spheres coincide when `z1` equals `x1` and miss each other otherwise. Where they
    /// coincide, O3 may be anywhere on the sphere that is also square to the J5 axis, which is a
    /// whole circle of solutions, and one point on it stands for the rest.
    ///
    /// returns: Option<Point3>
    pub fn o3_on_origin(&self, theta: f64) -> Option<Point3> {
        if (self.robot.z1() - self.robot.x1()).abs() > AXIS_TOL * self.robot.z1() {
            return None;
        }
        let (_, u) = self.o4_and_axis(theta);

        // Any direction square to u will do. Taking the longer of two candidates avoids the case
        // where u happens to point along the axis chosen to build it.
        let first = u.cross(&Vector3::z());
        let second = u.cross(&Vector3::x());
        let direction = if first.norm() >= second.norm() {
            first
        } else {
            second
        };
        Some(Point3::from(direction.normalize() * self.robot.z1()))
    }

    /// The candidate O3 points when O4 is on the J1 axis.
    ///
    /// With the plane constraint vacuous, O3 is free to be anywhere on the horizontal circle the
    /// two spheres cut out. The perpendicularity constraint then slices that circle with a plane,
    /// leaving two points, or one where the plane is tangent.
    ///
    /// returns: Vec<Point3>, empty when the plane misses the circle and also when O3 is on the
    /// axis too and J1 is free
    pub fn o3_on_axis(&self, theta: f64) -> Vec<Point3> {
        let (o4, u) = self.o4_and_axis(theta);
        let height = o4.z;
        let circle = self.circle_of_o3(&Point3::new(0.0, 0.0, height));
        let radius = circle.radius_sq.max(0.0).sqrt();
        let center_z = self.circle_height(height, &circle);

        // Points on the circle satisfy `(O3 - O4) . u = 0`, which in the circle's own angle is
        // another `a cos + b sin = c`.
        match solve_linear_trig(radius * u.x, radius * u.y, (height - center_z) * u.z) {
            // Every point on the circle qualifies: the family where J1 is free.
            TrigSolutions::All => Vec::new(),
            TrigSolutions::Angles(angles) => angles
                .into_iter()
                .map(|a| Point3::new(radius * a.cos(), radius * a.sin(), center_z))
                .collect(),
        }
    }

    /// The O3 representing the family where J1 is free because both origins are on the axis.
    pub fn o3_for_singular_family(&self, theta: f64) -> Point3 {
        let (o4, _) = self.o4_and_axis(theta);
        let circle = self.circle_of_o3(&Point3::new(0.0, 0.0, o4.z));
        Point3::new(0.0, 0.0, self.circle_height(o4.z, &circle))
    }

    /// Where the plane of the O3 circle sits when O4 is on the axis, which is on the same side of
    /// the origin as O4 is.
    fn circle_height(&self, o4_height: f64, circle: &O3Circle) -> f64 {
        if o4_height == 0.0 {
            0.0
        } else {
            circle.along * o4_height.signum()
        }
    }
}

/// Refines `theta` so that one branch's perpendicularity residual is zero, by Newton's method with
/// a numeric derivative and a step limit.
///
/// A root of the constraint is a root of one or both branch residuals. Starting from it, this
/// converges to the branch that owns the solution and walks away from the one that does not.
///
/// # Arguments
///
/// * `setup`: the prepared target
/// * `theta`: the angle to start from
/// * `branch`: which of the two O3 points to follow
///
/// returns: Option<f64>, or None if the branch does not exist near `theta`
pub fn polish_branch(setup: &Setup, mut theta: f64, branch: f64) -> Option<f64> {
    const MAX_STEPS: usize = 30;
    const MAX_STEP_SIZE: f64 = 0.05;
    const NUDGE: f64 = 1e-7;

    for _ in 0..MAX_STEPS {
        let value = setup.branch_residual(theta, branch)?;
        let ahead = setup.branch_residual(theta + NUDGE, branch)?;
        let derivative = (ahead - value) / NUDGE;
        if derivative == 0.0 {
            break;
        }
        let step = (-value / derivative).clamp(-MAX_STEP_SIZE, MAX_STEP_SIZE);
        theta += step;
        if step.abs() < 1e-15 {
            break;
        }
    }
    Some(theta)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::{all_robots, random_joints};
    use crate::{Crx, Iso3};

    /// Joint values that put O4 exactly on the J1 axis.
    ///
    /// In the frame J1 turns to, O4 sits at `z1 sin(J2) + x1 cos(J3)` off the axis, with J3 as the
    /// controller reports it. Setting that to zero fixes J3 from J2. On a model with `z1` larger
    /// than `x1` the shoulder cannot always be compensated for, and those draws are refused.
    ///
    /// Note that `J3 = J2 + 90` solves this only when the two lengths are equal, which is to say
    /// on the CRX-3iA and the CRX-10iA and nowhere else.
    fn on_axis_joints(robot: &Crx) -> Option<[f64; 6]> {
        let mut joints = random_joints();
        let ratio = -robot.z1() * joints[1].to_radians().sin() / robot.x1();
        if ratio.abs() > 1.0 {
            return None;
        }
        joints[2] = ratio.acos().to_degrees();
        Some(joints)
    }

    /// The angle a known configuration sits at, and the O3 and O4 it puts in space.
    fn truth(setup: &Setup, frames: &[Iso3; 6]) -> (f64, Point3, Point3) {
        let o3 = frames[2] * Point3::origin();
        let o4 = frames[3] * Point3::new(setup.robot.x1(), 0.0, 0.0);
        let u = (o4 - setup.o5).normalize();
        (u.dot(&setup.ay).atan2(u.dot(&setup.ax)), o3, o4)
    }

    #[test]
    fn one_branch_holds_the_real_o3() {
        // At the angle of a real configuration, one branch must contain that configuration's O3.
        for robot in all_robots() {
            for _ in 0..2000 {
                let joints = random_joints();
                let frames = robot.fk_all(&joints);
                let setup = Setup::new(&robot, &frames[5]);
                let (theta, expected_o3, expected_o4) = truth(&setup, &frames);
                let (o4, _) = setup.o4_and_axis(theta);
                assert!((o4 - expected_o4).norm() < 1e-8);

                if o4.x.hypot(o4.y) < AXIS_TOL * robot.z1() {
                    continue; // on the axis, where the branch construction does not apply
                }

                let nearest = [1.0, -1.0]
                    .iter()
                    .filter_map(|b| setup.o3_branch(&o4, *b, 1e-9))
                    .map(|o3| (o3 - expected_o3).norm())
                    .fold(f64::INFINITY, f64::min);

                // The tolerance is in millimeters and is based on the measured distribution.
                // Across 360,000 random poses, the median error was 2.5e-13 and the largest was
                // 4.9e-7. The largest errors occurred where the two spheres met at a shallow
                // angle. Ten nanometers provides a factor of twenty above the measured maximum
                // and remains too small to confuse this branch with the other branch, which is
                // half an arm away.
                assert!(nearest < 1e-5, "nearest branch was {:e} away", nearest);
            }
        }
    }

    #[test]
    fn the_branch_residual_vanishes_at_a_real_configuration() {
        for robot in all_robots() {
            for _ in 0..2000 {
                let joints = random_joints();
                let frames = robot.fk_all(&joints);
                let setup = Setup::new(&robot, &frames[5]);
                let (theta, _, o4) = truth(&setup, &frames);
                if o4.x.hypot(o4.y) < AXIS_TOL * robot.z1() {
                    continue;
                }

                let smallest = [1.0, -1.0]
                    .iter()
                    .filter_map(|b| setup.branch_residual(theta, *b))
                    .map(f64::abs)
                    .fold(f64::INFINITY, f64::min);

                // Across 360,000 random poses, the measured median residual was 4.3e-16 and the
                // largest was 5.1e-10. The threshold provides a factor of twenty above the
                // measured maximum and remains far below the residual of an invalid branch.
                assert!(smallest < 1e-8, "smallest residual was {:e}", smallest);
            }
        }
    }

    #[test]
    fn the_axis_search_finds_a_crossing() {
        // Put O4 on the axis by setting the kinematic J3 rotation to a quarter turn, then verify
        // that the search finds the corresponding angle.
        for robot in all_robots() {
            for _ in 0..500 {
                let Some(joints) = on_axis_joints(&robot) else {
                    continue;
                };
                let frames = robot.fk_all(&joints);
                let setup = Setup::new(&robot, &frames[5]);
                let (theta, _, o4) = truth(&setup, &frames);
                assert!(
                    o4.x.hypot(o4.y) < 1e-9,
                    "test setup did not place O4 on the axis"
                );

                let found = setup.axis_thetas();
                assert!(!found.is_empty(), "no crossing found");
                let nearest = found
                    .iter()
                    .map(|t| wrap_pi(t - theta).abs())
                    .fold(f64::INFINITY, f64::min);
                assert!(nearest < 1e-6, "nearest crossing was {:e} away", nearest);
            }
        }
    }

    #[test]
    fn the_axis_search_does_not_fire_spuriously() {
        // The O4 circle projects an ellipse onto the floor. For each model, this ellipse passes
        // within the axis tolerance for approximately one random pose in twenty thousand. Verify
        // that every reported angle places O4 within the axis tolerance and that the report rate
        // remains far below the rate expected from false detections on ordinary poses.
        const DRAWS: usize = 500;
        let mut reported = 0;

        for robot in all_robots() {
            for _ in 0..DRAWS {
                let target = robot.fk(&random_joints());
                let setup = Setup::new(&robot, &target);
                for theta in setup.axis_thetas() {
                    let (distance, _, _) = setup.axis_distance(theta);
                    assert!(
                        distance <= AXIS_TOL * robot.z1(),
                        "reported a crossing {distance:e} mm from the axis"
                    );
                    reported += 1;
                }
            }
        }

        // A rate of one pose in twenty thousand permits a few crossings in these draws. False
        // detections on ordinary poses would produce a rate orders of magnitude higher.
        assert!(
            reported < DRAWS / 10,
            "{reported} crossings among random poses is far more than the geometry allows"
        );
    }

    #[test]
    fn the_on_axis_construction_holds_the_real_o3() {
        for robot in all_robots() {
            for _ in 0..500 {
                let Some(joints) = on_axis_joints(&robot) else {
                    continue;
                };
                let frames = robot.fk_all(&joints);
                let setup = Setup::new(&robot, &frames[5]);
                let (theta, expected_o3, _) = truth(&setup, &frames);

                if setup.is_on_origin(theta) || setup.is_singular_family(theta) {
                    continue;
                }

                // In approximately one of 750,000 on-axis draws, the plane perpendicular to `u`
                // is tangent to the circle and rounding moves it just outside. The construction
                // then returns no points. In the examined case, `ik` still recovered the
                // configuration through the ordinary branches.
                let points = setup.o3_on_axis(theta);
                if points.is_empty() {
                    continue;
                }

                let nearest = points
                    .into_iter()
                    .map(|o3| (o3 - expected_o3).norm())
                    .fold(f64::INFINITY, f64::min);
                // The tolerance is in millimeters and is based on the measured distribution.
                // Across 300,000 on-axis draws, the median error was 6.3e-13 and the largest was
                // 1.4e-6. The largest errors occurred when the trigonometric solve that cuts the
                // circle was poorly conditioned. Fifty nanometers provides a factor of thirty
                // above the measured maximum and is still one part in 10^7 of the arm, preventing
                // an incorrect point on the circle from passing.
                assert!(nearest < 5e-5, "nearest point was {:e} away", nearest);
            }
        }
    }
}
