//! This module implements the inverse kinematics solver.
//!
//! The derivation is in `docs/LINEAR_METHOD.md`, and the NumPy reference implementation is in
//! `derivation/crx/ik_reference.py`. The modules here mirror the reference implementation. Consult
//! the derivation for the rationale behind each step, and compare the implementations to validate
//! the Rust calculations.
//!
//! In outline: the target pose leaves only two of the six kinematic origins unknown, and
//! parameterizing one of them by its angle on the circle it travels reduces the problem to a
//! single scalar equation. That equation is a trigonometric polynomial of degree four, so it has
//! at most eight roots, and each root gives two joint solutions.

pub mod geometry;
pub mod joints;
pub mod polish;
pub mod roots;
pub mod setup;
pub mod trig_poly;

use crate::ik::geometry::polish_branch;
use crate::ik::joints::joints_from_points;
use crate::ik::polish::{DUPLICATE_TOL, POSE_TOL, joint_distance, polish_joints, pose_error};
use crate::ik::roots::theta_roots;
use crate::ik::setup::Setup;
use crate::ik::trig_poly::TrigPoly;
use crate::{Crx, Iso3, Point3, wrap_pi};

/// How small a branch's perpendicularity residual must be for its angle to count as converged.
const RESIDUAL_TOL: f64 = 1e-9;

/// The maximum branch residual that identifies an accurate root on that branch.
///
/// A double root is located to about half a thousandth of a radian, which leaves a residual of
/// that order on the branch that owns it. This threshold is much lower than the residual produced
/// by an inaccurate root.
const SCREEN_TINY: f64 = 1e-6;

/// The minimum residual that excludes a branch when the other branch is below [`SCREEN_TINY`].
///
/// The four orders of magnitude between the two thresholds are the margin for the two branches
/// having different sensitivities to the angle.
const SCREEN_LARGE: f64 = 1e-2;

/// How far O4 must be from the J1 axis, as a fraction of `z1`, for the branch residuals to be
/// trusted to screen a branch out.
const SCREEN_AXIS_CLEARANCE: f64 = 1e-2;

/// What sort of configuration a solution came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolutionKind {
    /// O4 is off the J1 axis and the solution is isolated.
    Regular,

    /// O4 is on the J1 axis. The solution is still isolated, but the constraint has a double root.
    AxisDegenerate,

    /// A joint is free and this is one representative of a continuum of solutions.
    SingularFamily,
}

/// One inverse kinematics solution.
#[derive(Debug, Clone, Copy)]
pub struct IkSolution {
    /// The six joint angles in FANUC controller degrees.
    pub joints: [f64; 6],

    /// Largest absolute element difference between this solution's pose and the target.
    pub residual: f64,

    /// What sort of configuration it came from.
    pub kind: SolutionKind,

    /// The angle on the O4 circle that produced it, kept for diagnostics.
    pub theta: f64,
}

/// A located pair of origins, before it becomes joint angles.
struct Candidate {
    o3: Point3,
    o4: Point3,
    theta: f64,
    kind: SolutionKind,
}

impl Crx {
    /// Every joint configuration that puts the flange at `target`.
    ///
    /// Returns an empty vector when the target cannot be reached. A solution marked
    /// [`SolutionKind::SingularFamily`] represents a continuum because a joint has no effect on the
    /// pose in that configuration.
    ///
    /// # Arguments
    ///
    /// * `target`: the desired pose of the flange, in the robot's world coordinate system
    ///
    /// returns: Vec<IkSolution>, in no particular order
    pub fn ik(&self, target: &Iso3) -> Vec<IkSolution> {
        let setup = Setup::new(self, target);
        let mut solutions: Vec<IkSolution> = Vec::with_capacity(16);

        for candidate in candidates(&setup) {
            for joints in joints_from_points(&setup, &candidate.o3, &candidate.o4) {
                let joints = polish_joints(self, &joints, target);
                let residual = pose_error(self, &joints, target);
                if residual > POSE_TOL {
                    continue;
                }
                if solutions
                    .iter()
                    .any(|s| joint_distance(&joints, &s.joints) < DUPLICATE_TOL)
                {
                    continue;
                }
                solutions.push(IkSolution {
                    joints,
                    residual,
                    kind: candidate.kind,
                    theta: candidate.theta,
                });
            }
        }

        solutions
    }

    /// The solution whose joints are nearest to `reference`.
    ///
    /// A caller following a path can pass the robot's current joint vector as `reference` to select
    /// the configuration that reaches the next pose with the least joint movement. This selection
    /// prevents the arm from reconfiguring during the move.
    ///
    /// # Arguments
    ///
    /// * `target`: the desired pose of the flange
    /// * `reference`: a joint vector in FANUC controller degrees, typically the current position
    ///
    /// returns: Option<IkSolution>, or None if the target cannot be reached
    pub fn ik_closest(&self, target: &Iso3, reference: &[f64; 6]) -> Option<IkSolution> {
        self.ik(target).into_iter().min_by(|a, b| {
            joint_distance(&a.joints, reference)
                .partial_cmp(&joint_distance(&b.joints, reference))
                .expect("joint distances are finite")
        })
    }
}

/// Every located pair of origins worth turning into joint angles.
///
/// Candidate generation uses broad criteria because the caller can reject an invalid candidate
/// with a few Gauss-Newton steps. A solution omitted at this stage cannot be recovered later.
fn candidates(setup: &Setup) -> Vec<Candidate> {
    let mut out = Vec::with_capacity(16);
    let axis_angles = setup.axis_thetas();

    for theta in &axis_angles {
        let theta = *theta;
        let (o4, _) = setup.o4_and_axis(theta);

        if setup.is_on_origin(theta) {
            // O4 has landed on O1, so both J1 and J2 lose their effect. A pose passing near the
            // origin without reaching it is unrecoverable; the derivation's module documentation
            // explains this limitation.
            if let Some(o3) = setup.o3_on_origin(theta) {
                out.push(Candidate {
                    o3,
                    o4,
                    theta,
                    kind: SolutionKind::SingularFamily,
                });
            }
            continue;
        }

        if setup.is_singular_family(theta) {
            // O3 is on the axis too, so J1 is free. One representative stands for the family.
            out.push(Candidate {
                o3: setup.o3_for_singular_family(theta),
                o4,
                theta,
                kind: SolutionKind::SingularFamily,
            });
            continue;
        }

        for o3 in setup.o3_on_axis(theta) {
            out.push(Candidate {
                o3,
                o4,
                theta,
                kind: SolutionKind::AxisDegenerate,
            });
        }
    }

    let poly = TrigPoly::from_setup(setup);
    for theta in theta_roots(&poly) {
        if axis_angles.iter().any(|a| wrap_pi(theta - a).abs() < 1e-5) {
            continue; // already covered by the on-axis construction above
        }
        let (o4, _) = setup.o4_and_axis(theta);

        // At a simple root, one branch has a zero residual and the other generally has a residual
        // of order one. When an accurate root produces a small residual on one branch and a large
        // residual on the other, the branch with the large residual cannot contain a solution.
        // Skipping that branch avoids polishing it and then detecting a duplicate. Follow both
        // branches when their residuals are moderate because this pattern indicates an inaccurate
        // root at a merged pair. Also follow both branches near the J1 axis, where the vertical
        // plane changes rapidly and the residuals are unreliable.
        let screened = if setup.axis_distance(theta).0 > SCREEN_AXIS_CLEARANCE * setup.robot.z1() {
            [
                setup.branch_residual(theta, 1.0).map(f64::abs),
                setup.branch_residual(theta, -1.0).map(f64::abs),
            ]
        } else {
            [None, None]
        };

        for (index, branch) in [1.0, -1.0].into_iter().enumerate() {
            if let (Some(own), Some(other)) = (screened[index], screened[1 - index]) {
                if own > SCREEN_LARGE && other < SCREEN_TINY {
                    continue;
                }
            }
            let refined = polish_branch(setup, theta, branch);
            let residual = refined.and_then(|t| setup.branch_residual(t, branch));

            let converged = match (refined, residual) {
                (Some(refined), Some(residual)) if residual.abs() <= RESIDUAL_TOL => Some(refined),
                _ => None,
            };
            if let Some(refined) = converged {
                let (polished_o4, _) = setup.o4_and_axis(refined);
                if let Some(o3) = setup.o3_branch(&polished_o4, branch, 0.0) {
                    out.push(Candidate {
                        o3,
                        o4: polished_o4,
                        theta: refined,
                        kind: SolutionKind::Regular,
                    });
                    continue;
                }
            }

            // The polish fails where the two spheres are tangent, because the branch residual has
            // no zero crossing to find there. The root of the constraint is already the answer.
            if let Some(o3) = setup.o3_branch(&o4, branch, 1e-6) {
                out.push(Candidate {
                    o3,
                    o4,
                    theta,
                    kind: SolutionKind::Regular,
                });
            }
        }
    }

    out
}
