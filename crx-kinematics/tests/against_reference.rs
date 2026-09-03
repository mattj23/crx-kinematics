//! These tests compare the polynomial machinery with the NumPy reference implementation.
//!
//! The fixture is written by `derivation/export_fixtures.py` and contains the target and the angles
//! found by the reference implementation for each of several hundred poses. It explicitly includes
//! poses with difficult geometries, including the pose from issue #1.

use crx_kinematics::ik::roots::theta_roots;
use crx_kinematics::ik::setup::Setup;
use crx_kinematics::ik::trig_poly::TrigPoly;
use crx_kinematics::{Crx, Result, row_slice_to_iso, wrap_pi};
use serde::Deserialize;

/// How far apart the two implementations may put the same angle.
///
/// Both implementations use the same arithmetic, so ordinary roots agree to the last few digits.
/// Across the complete fixture, the largest measured difference was three ten-millionths of a
/// radian. These differences occur at merged pairs, where the angle is weakly determined and the
/// two languages round intermediate values differently.
const ANGLE_TOL: f64 = 1e-5;

/// Angles nearer than this are the same root reported twice.
///
/// A double root reaches the two eigenvalue solvers as a conjugate pair, and the pair's real parts
/// can be a fraction of a microradian apart or identical, depending on the solver. One
/// implementation can therefore report one angle while the other reports two. Genuine roots are
/// farther apart by orders of magnitude, so merging at this distance permits a comparison of the
/// distinct roots in each set.
const MERGE_TOL: f64 = 1e-4;

/// The count of distinct roots, treating a double root reported twice as one.
fn distinct(angles: &[f64]) -> usize {
    let mut kept: Vec<f64> = Vec::new();
    for angle in angles {
        if !kept.iter().any(|seen| wrap_pi(angle - seen).abs() < MERGE_TOL) {
            kept.push(*angle);
        }
    }
    kept.len()
}

/// The distance from `angle` to the nearest member of `angles`.
fn nearest(angle: f64, angles: &[f64]) -> f64 {
    angles
        .iter()
        .map(|other| wrap_pi(other - angle).abs())
        .fold(f64::INFINITY, f64::min)
}

#[derive(Deserialize)]
struct Case {
    model: String,
    joints: [f64; 6],
    target: Vec<f64>,
    thetas: Vec<f64>,
}

fn robot_for(model: &str) -> Crx {
    match model {
        "crx3ia" => Crx::new_3ia(),
        "crx5ia" => Crx::new_5ia(),
        "crx10ia" => Crx::new_10ia(),
        "crx10ial" => Crx::new_10ia_l(),
        "crx20ial" => Crx::new_20ia_l(),
        "crx30ia" => Crx::new_30ia(),
        other => panic!("unknown model {other}"),
    }
}

fn cases() -> Result<Vec<Case>> {
    let bytes = include_bytes!("data/theta_roots.json");
    Ok(serde_json::from_slice(bytes)?)
}

#[test]
fn the_forward_kinematics_agree() -> Result<()> {
    // If the two disagreed about the pose, every later comparison would be meaningless.
    for case in cases()? {
        let robot = robot_for(&case.model);
        let expected = row_slice_to_iso(&case.target)?;
        let computed = robot.fk(&case.joints);
        approx::assert_relative_eq!(computed, expected, epsilon = 1e-9);
    }
    Ok(())
}

#[test]
fn the_same_angles_are_found() -> Result<()> {
    let mut worst: f64 = 0.0;
    let mut checked = 0;

    for case in cases()? {
        let robot = robot_for(&case.model);
        let target = row_slice_to_iso(&case.target)?;
        let poly = TrigPoly::from_setup(&Setup::new(&robot, &target));
        let found = theta_roots(&poly);

        assert_eq!(
            distinct(&found),
            distinct(&case.thetas),
            "{} at {:?}: found {:?}, the reference found {:?}",
            case.model,
            case.joints,
            found,
            case.thetas
        );

        // Compare in both directions. The first loop detects a reference root missing from this
        // port, and the second detects a root produced only by this port.
        for expected in &case.thetas {
            let gap = nearest(*expected, &found);
            assert!(
                gap < ANGLE_TOL,
                "{} at {:?}: nothing within {:e} of the reference angle {}",
                case.model,
                case.joints,
                gap,
                expected
            );
            worst = worst.max(gap);
            checked += 1;
        }
        for angle in &found {
            let gap = nearest(*angle, &case.thetas);
            assert!(
                gap < ANGLE_TOL,
                "{} at {:?}: the reference has nothing within {:e} of {}",
                case.model,
                case.joints,
                gap,
                angle
            );
            worst = worst.max(gap);
            checked += 1;
        }
    }

    println!("{checked} angles compared, worst disagreement {worst:e} radians");
    Ok(())
}
