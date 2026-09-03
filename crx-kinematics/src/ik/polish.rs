//! This module refines solutions against the target pose and identifies duplicate solutions.

use crate::na::{SMatrix, SVD, SVector};
use crate::{Crx, Iso3, wrap_pi};

/// How near a candidate's pose must land, after polishing, to be accepted.
///
/// Real solutions reach the last digits of double precision, while invalid candidates remain well
/// above this tolerance. The gap permits broad candidate-generation criteria because rejecting an
/// invalid candidate costs only a few steps, while an omitted solution cannot be recovered.
pub const POSE_TOL: f64 = 1e-8;

/// Joint-space radius, in radians, within which two solutions are the same one.
///
/// Loose enough to recognize a wrist that reached the same configuration from slightly different
/// directions: with J5 at zero, J4 and J6 trade against each other with no effect on the pose. Two
/// genuinely distinct configurations this close together would be nearly merged.
pub const DUPLICATE_TOL: f64 = 1e-5;

/// Largest absolute element difference between the pose of `joints` and `target`.
///
/// The rotation and translation blocks are compared together. Mixing the two is deliberate: it is
/// the quantity used to determine whether the flange reaches the requested pose.
///
/// # Arguments
///
/// * `robot`: the robot model
/// * `joints`: a joint vector in FANUC controller degrees
/// * `target`: the pose that was asked for
///
/// returns: f64
pub fn pose_error(robot: &Crx, joints: &[f64; 6], target: &Iso3) -> f64 {
    residuals(robot, joints, target)
        .iter()
        .fold(0.0f64, |worst, r| worst.max(r.abs()))
}

/// The twelve differences that the polish drives to zero: nine from the rotation and three from
/// the translation.
fn residuals(robot: &Crx, joints: &[f64; 6], target: &Iso3) -> SVector<f64, 12> {
    let pose = robot.fk(joints);
    let rotation = pose.rotation.to_rotation_matrix();
    let wanted = target.rotation.to_rotation_matrix();
    let mut out = SVector::<f64, 12>::zeros();
    for index in 0..9 {
        out[index] = rotation[(index / 3, index % 3)] - wanted[(index / 3, index % 3)];
    }
    for index in 0..3 {
        out[9 + index] = pose.translation.vector[index] - target.translation.vector[index];
    }
    out
}

/// Refines a joint vector directly against the target pose by Gauss-Newton, with the Jacobian
/// taken by finite difference of the forward kinematics.
///
/// Where the Jacobian is rank deficient, which is what a singular configuration means, the least
/// squares step is the smallest valid step, which keeps the solution at its initial position along
/// the free direction.
///
/// # Arguments
///
/// * `robot`: the robot model
/// * `joints`: the starting joint vector, in FANUC controller degrees
/// * `target`: the pose to reach
///
/// returns: [f64; 6]
pub fn polish_joints(robot: &Crx, joints: &[f64; 6], target: &Iso3) -> [f64; 6] {
    const MAX_STEPS: usize = 25;
    const NUDGE: f64 = 1e-6;

    // Gauss-Newton converges quadratically, so one step from a good candidate reaches a pose within
    // a few rounding errors of the target. On an arm with a reach of one or two meters, a
    // translation has a rounding error of a few times 1e-13 mm. A lower tolerance adds a step that
    // cannot improve the result. This tolerance is four orders of magnitude below the acceptance
    // tolerance.
    const TOL: f64 = 1e-12;

    // This is the maximum permitted step in degrees. A candidate starts within a small fraction of
    // a degree of its solution. A step of many degrees therefore indicates that the Jacobian is
    // nearly rank deficient and that the step follows a direction with little effect on the pose.
    // Following such steps turns polishing into a random walk that can reach an unrelated solution
    // many turns away.
    const MAX_STEP: f64 = 10.0;

    let mut joints = *joints;
    let mut worst = pose_error(robot, &joints, target);
    for _ in 0..MAX_STEPS {
        if worst < TOL {
            break;
        }
        let current = residuals(robot, &joints, target);

        let mut jacobian = SMatrix::<f64, 12, 6>::zeros();
        for column in 0..6 {
            let mut shifted = joints;
            shifted[column] += NUDGE;
            let column_values = (residuals(robot, &shifted, target) - current) / NUDGE;
            jacobian.set_column(column, &column_values);
        }

        let Some(step) = least_squares_step(&jacobian, &current) else {
            break;
        };
        if step.amax() > MAX_STEP {
            break;
        }

        let mut moved = joints;
        for index in 0..6 {
            moved[index] += step[index];
        }

        // Near a merged pair the step is long and the improvement slow, so the loop is allowed to
        // run for a while, but a step that stops helping means the finite-difference Jacobian has
        // reached its own noise floor and further iterations only move the answer around.
        let improved = pose_error(robot, &moved, target);
        if improved >= worst {
            break;
        }
        joints = moved;
        worst = improved;
    }

    joints
}

/// The least squares step `delta` that minimizes `|jacobian * delta + residual|`.
///
/// Away from a singularity, a Cholesky factorization of the normal equations finds the step at a
/// small fraction of the cost of a singular value decomposition. Squaring the Jacobian squares its
/// condition number. Polishing evaluates the exact residual after every step, so a small directional
/// error can add an iteration without reducing the final accuracy.
///
/// Near a singularity, the squared system loses too much precision. At a singularity, a line of
/// least squares solutions exists, and the normal equations do not select the smallest solution.
/// The singular value decomposition selects the smallest step, which keeps the solution at its
/// initial position along the free direction.
///
/// # Arguments
///
/// * `jacobian`: the derivative of the residuals with respect to the joints
/// * `residual`: the residuals at the current joints
///
/// returns: Option<SVector<f64, 6>>, or None if the decomposition failed
fn least_squares_step(
    jacobian: &SMatrix<f64, 12, 6>,
    residual: &SVector<f64, 12>,
) -> Option<SVector<f64, 6>> {
    // The diagonal of the Cholesky factor approximates the singular values of the Jacobian within
    // a modest factor. This ratio therefore permits condition numbers up to one hundred thousand.
    // Squaring such a condition number costs the normal equations ten of their sixteen digits and
    // leaves enough precision for polishing to converge in a few iterations. Larger condition
    // numbers can cause a near-singular configuration to fail to converge.
    const CONDITION_TOL: f64 = 1e-5;

    let normal = jacobian.transpose() * jacobian;
    if let Some(cholesky) = normal.cholesky() {
        let pivots = cholesky.l().diagonal();
        let largest = pivots.max();
        if largest > 0.0 && pivots.min() > CONDITION_TOL * largest {
            return Some(cholesky.solve(&(-(jacobian.transpose() * residual))));
        }
    }

    let svd = SVD::new(*jacobian, true, true);
    let largest = svd.singular_values.max();
    svd.solve(&(-residual), largest * 1e-12).ok()
}

/// Largest per-joint difference between two joint vectors, in radians, counting angles a full turn
/// apart as the same angle.
///
/// # Arguments
///
/// * `first`: a joint vector in FANUC controller degrees
/// * `second`: another joint vector in the same convention
///
/// returns: f64
pub fn joint_distance(first: &[f64; 6], second: &[f64; 6]) -> f64 {
    first
        .iter()
        .zip(second.iter())
        .map(|(a, b)| wrap_pi((a - b).to_radians()).abs())
        .fold(0.0f64, f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::{all_robots, random_joints};

    #[test]
    fn polishing_a_solution_leaves_it_alone() {
        // A configuration that already reaches the target should come back essentially unmoved.
        for robot in all_robots() {
            for _ in 0..500 {
                let joints = random_joints();
                let target = robot.fk(&joints);
                let polished = polish_joints(&robot, &joints, &target);

                assert!(joint_distance(&polished, &joints) < 1e-9);
                assert!(pose_error(&robot, &polished, &target) < 1e-11);
            }
        }
    }

    #[test]
    fn polishing_recovers_a_nudged_solution() {
        // Verify that polishing moves a nearby joint vector onto the solution. Candidate
        // generation produces vectors much closer than the perturbation used here.
        //
        // Approximately one of 120,000 measured nudged starts exhausted the step budget before
        // reaching the acceptance threshold. Stalls can occur outside the wrist singularity; one
        // measured stall had J5 = 120 degrees, so excluding a singular band does not eliminate
        // them. The measured stalled starts had errors no larger than 2e-6 mm. In every examined
        // case, `ik` recovered the original joints because its geometry-derived candidates were
        // better conditioned than a uniform nudge.
        //
        // Require every start to remain within 1e-4 mm; the largest observed stall error of 2e-6 mm
        // is fifty times smaller. Also limit stalls to a rate that would require a hundredfold
        // increase over the measured rate before this test fails.
        const DRAWS: usize = 500;
        let mut stalled = 0;

        for robot in all_robots() {
            for _ in 0..DRAWS {
                let joints = random_joints();
                let target = robot.fk(&joints);

                let mut nudged = joints;
                for angle in nudged.iter_mut() {
                    *angle += 1e-3;
                }

                let polished = polish_joints(&robot, &nudged, &target);
                let error = pose_error(&robot, &polished, &target);
                assert!(error < 1e-4, "pose error was {error:e}");
                if error >= POSE_TOL {
                    stalled += 1;
                }
            }
        }

        assert!(
            stalled <= 3,
            "{stalled} of {} nudged starts failed to converge",
            DRAWS * 6
        );
    }

    #[test]
    fn polishing_never_makes_things_worse() {
        // From farther away, polishing can exhaust its steps or reach the finite-difference
        // Jacobian's noise floor before converging on a near-singular configuration. It must not
        // increase the pose error because that could turn a near miss into an incorrect solution.
        for robot in all_robots() {
            for _ in 0..500 {
                let joints = random_joints();
                let target = robot.fk(&joints);

                let mut nudged = joints;
                for angle in nudged.iter_mut() {
                    *angle += 0.5;
                }

                let before = pose_error(&robot, &nudged, &target);
                let after = pose_error(&robot, &polish_joints(&robot, &nudged, &target), &target);
                assert!(after <= before, "{before:e} became {after:e}");
            }
        }
    }

    #[test]
    fn joint_distance_counts_a_full_turn_as_nothing() {
        let a = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0];
        let mut b = a;
        b[3] += 360.0;
        assert!(joint_distance(&a, &b) < 1e-12);
    }
}
