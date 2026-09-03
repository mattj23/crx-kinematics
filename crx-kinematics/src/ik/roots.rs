//! This module finds every root of the scalar constraint.
//!
//! The finite, complete search converts the trigonometric polynomial into an ordinary polynomial
//! of degree eight and finds all of its complex roots at once. Because the method does not sample
//! the polynomial, it cannot miss a root between samples.
//!
//! The Ehrlich-Aberth iteration refines every root simultaneously and usually converges from a
//! circle of starting points in a few passes. If it does not converge, the solver uses the
//! eigenvalues of the polynomial's companion matrix, a slower and more established method.

use super::trig_poly::{DEGREE, MAX_NEWTON_STEP, TrigPoly};
use crate::na::{Complex, DMatrix};
use crate::wrap_pi;
use std::f64::consts::PI;

/// How large an imaginary part a root may carry, relative to its own magnitude, and still count
/// as real.
///
/// This tolerance is deliberately loose because two merged solutions produce a double root. In
/// floating-point arithmetic, a double root splits into a conjugate pair whose imaginary part is
/// near the *square root* of the rounding error, which is larger than the rounding error itself. A
/// strict test would discard these roots and their corresponding configurations. The loose
/// tolerance can also admit candidates that are not solutions; the caller removes them by checking
/// the resulting pose.
pub const IMAGINARY_TOL: f64 = 1e-2;

/// Two angles nearer than this are treated as the same root.
pub const DUPLICATE_TOL: f64 = 1e-9;

/// Candidate angles covering every real root of `f`, in no particular order.
///
/// Every true root has a nearby candidate. Where two solutions have merged, `f` is sufficiently
/// flat that the angle is determined only to about half a thousandth of a radian. The loose
/// tolerance on the imaginary part can also admit an angle that is not a root. These behaviors
/// prevent the solver from discarding a solution; the caller rejects extra candidates with a few
/// Newton steps and a check of the pose that each candidate produces.
///
/// # Arguments
///
/// * `poly`: the coefficients of `f`
///
/// returns: Vec<f64>, angles in the range [-pi, pi)
pub fn theta_roots(poly: &TrigPoly) -> Vec<f64> {
    let shift = poly.origin_shift();
    let coefficients = poly.shifted(shift).half_angle_polynomial();

    let candidates = match real_roots(&coefficients) {
        Some(roots) => roots
            .into_iter()
            .map(|t| wrap_pi(2.0 * t.atan() + shift))
            .collect(),
        // Both root finders are iterative and can fail to converge, although no failures have been
        // observed. If both fail, the slower bracketing fallback provides candidates and avoids
        // returning no solutions.
        None => bracketed_roots(poly),
    };

    let mut polished = Vec::with_capacity(2 * DEGREE);
    for mut theta in candidates {
        for _ in 0..20 {
            let (value, derivative) = poly.eval(theta);
            if derivative == 0.0 {
                break;
            }
            let step = -value / derivative;

            // Both guards protect a double root, where the value and slope approach zero together
            // and rounding determines their ratio. The first rejects a step longer than the cap,
            // and the second rejects a step that does not reduce the magnitude. These guards keep a
            // valid root from moving onto its neighbor and being lost when duplicates are merged.
            if step.abs() > MAX_NEWTON_STEP {
                break;
            }
            if poly.eval(theta + step).0.abs() >= value.abs() {
                break;
            }
            theta += step;
            if step.abs() < 1e-15 {
                break;
            }
        }

        let theta = wrap_pi(theta);
        if !polished
            .iter()
            .any(|seen| wrap_pi(theta - seen).abs() < DUPLICATE_TOL)
        {
            polished.push(theta);
        }
    }

    polished
}

/// The real roots of a polynomial given in ascending powers.
///
/// # Arguments
///
/// * `coefficients`: the polynomial, lowest power first
///
/// returns: Option<Vec<f64>>, or None if neither root finder converged
fn real_roots(coefficients: &[f64; 2 * DEGREE + 1]) -> Option<Vec<f64>> {
    // A leading coefficient of zero is a root at infinity, which is a root the substitution has
    // already lost. Trimming those is what keeps the problem well posed.
    let largest = coefficients.iter().fold(0.0f64, |acc, c| acc.max(c.abs()));
    if largest == 0.0 {
        return Some(Vec::new());
    }
    let degree = coefficients
        .iter()
        .rposition(|c| c.abs() > 1e-14 * largest)?;
    if degree == 0 {
        return Some(Vec::new());
    }
    let coefficients = &coefficients[..=degree];

    let roots = aberth_roots(coefficients).or_else(|| companion_roots(coefficients))?;

    Some(
        roots
            .into_iter()
            .filter(|value| value.im.abs() <= IMAGINARY_TOL * value.re.abs().max(1.0))
            .map(|value| value.re)
            .collect(),
    )
}

/// Every root of a polynomial, by the Ehrlich-Aberth iteration.
///
/// The iteration keeps one estimate per root. It applies a Newton step corrected by repulsion from
/// the other estimates, which prevents two estimates from converging to the same root. Convergence
/// is cubic at a simple root and linear at a multiple root. The iteration accepts an estimate when
/// the polynomial value is within the rounding error of evaluating the polynomial at that point,
/// because further refinement cannot reliably reduce the value.
///
/// # Arguments
///
/// * `coefficients`: the polynomial, lowest power first, with a nonzero leading coefficient
///
/// returns: Option<Vec<Complex<f64>>>, or None if the iteration did not converge
fn aberth_roots(coefficients: &[f64]) -> Option<Vec<Complex<f64>>> {
    const MAX_PASSES: usize = 100;

    // The value of a degree-eight polynomial evaluated by Horner's rule carries a rounding error
    // of a few times the machine epsilon multiplied by the sum of its terms' magnitudes.
    const FLOOR: f64 = 32.0 * f64::EPSILON;

    let degree = coefficients.len() - 1;
    let leading = coefficients[degree];
    let monic: Vec<f64> = coefficients.iter().map(|c| c / leading).collect();

    // By Fujiwara's bound, the starting points lie on a circle whose radius is within a factor of
    // two of the largest root's magnitude. The offset keeps the points off the real axis and
    // prevents symmetry from trapping a pair of estimates against each other.
    let radius = (0..degree)
        .map(|i| monic[i].abs().powf(1.0 / (degree - i) as f64))
        .fold(0.0f64, f64::max);
    if radius == 0.0 {
        return Some(vec![Complex::new(0.0, 0.0); degree]);
    }
    let mut roots: Vec<Complex<f64>> = (0..degree)
        .map(|k| Complex::from_polar(radius, 2.0 * PI * k as f64 / degree as f64 + 0.4))
        .collect();

    for _ in 0..MAX_PASSES {
        let mut all_settled = true;

        for k in 0..degree {
            let z = roots[k];

            // The polynomial and its derivative by Horner's rule, along with the magnitude sum
            // that bounds the rounding error of the value.
            let mut value = Complex::new(monic[degree], 0.0);
            let mut derivative = Complex::new(0.0, 0.0);
            let mut bound = 1.0;
            let magnitude = z.norm();
            for i in (0..degree).rev() {
                derivative = derivative * z + value;
                value = value * z + monic[i];
                bound = bound * magnitude + monic[i].abs();
            }
            if value.norm() <= FLOOR * bound {
                continue;
            }
            all_settled = false;

            if derivative == Complex::new(0.0, 0.0) {
                // A stationary point. Nudging the estimate off it is enough for the next pass.
                roots[k] = z + Complex::new(1e-3 * radius, 1e-3 * radius);
                continue;
            }
            let newton = value / derivative;
            let mut repulsion = Complex::new(0.0, 0.0);
            for (j, other) in roots.iter().enumerate() {
                if j != k {
                    let separation = z - other;
                    if separation == Complex::new(0.0, 0.0) {
                        return None;
                    }
                    repulsion += Complex::new(1.0, 0.0) / separation;
                }
            }
            let denominator = Complex::new(1.0, 0.0) - newton * repulsion;
            let correction = if denominator == Complex::new(0.0, 0.0) {
                newton
            } else {
                newton / denominator
            };
            if !correction.re.is_finite() || !correction.im.is_finite() {
                return None;
            }
            roots[k] = z - correction;
        }

        if all_settled {
            return Some(roots);
        }
    }
    None
}

/// Every root of a polynomial, as the eigenvalues of its companion matrix.
///
/// # Arguments
///
/// * `coefficients`: the polynomial, lowest power first, with a nonzero leading coefficient
///
/// returns: Option<Vec<Complex<f64>>>, or None if the eigenvalue iteration did not converge
fn companion_roots(coefficients: &[f64]) -> Option<Vec<Complex<f64>>> {
    let degree = coefficients.len() - 1;
    let leading = coefficients[degree];
    let mut companion = DMatrix::zeros(degree, degree);
    for row in 1..degree {
        companion[(row, row - 1)] = 1.0;
    }
    for column in 0..degree {
        companion[(0, column)] = -coefficients[degree - 1 - column] / leading;
    }
    Some(companion.complex_eigenvalues().iter().copied().collect())
}

/// Roots found by scanning for sign changes and bisecting, used only when neither root finder
/// converges.
///
/// This fallback cannot find a double root because `f` touches zero there without crossing it.
///
/// # Arguments
///
/// * `poly`: the coefficients of `f`
///
/// returns: Vec<f64>
fn bracketed_roots(poly: &TrigPoly) -> Vec<f64> {
    const SAMPLES: usize = 720;
    let mut found = Vec::new();
    let step = 2.0 * PI / SAMPLES as f64;

    let mut previous = poly.eval(-PI).0;
    for index in 1..=SAMPLES {
        let theta = -PI + step * index as f64;
        let current = poly.eval(theta).0;
        if previous == 0.0 {
            found.push(theta - step);
        } else if previous * current < 0.0 {
            let (mut low, mut high) = (theta - step, theta);
            let mut low_value = previous;
            for _ in 0..80 {
                let middle = 0.5 * (low + high);
                let value = poly.eval(middle).0;
                if value == 0.0 {
                    low = middle;
                    high = middle;
                    break;
                }
                if low_value * value < 0.0 {
                    high = middle;
                } else {
                    low = middle;
                    low_value = value;
                }
            }
            found.push(0.5 * (low + high));
        }
        previous = current;
    }
    found
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ik::setup::Setup;
    use crate::tests::{all_robots, random_joints};
    use crate::{Point3, Vector3};

    /// The angle a known configuration actually sits at.
    fn true_theta(setup: &Setup, frames: &[crate::Iso3; 6]) -> f64 {
        let o4 = frames[3] * Point3::new(setup.robot.x1(), 0.0, 0.0);
        let u: Vector3 = (o4 - setup.o5).normalize();
        u.dot(&setup.ay).atan2(u.dot(&setup.ax))
    }

    #[test]
    fn every_root_is_a_root() {
        for robot in all_robots() {
            for _ in 0..500 {
                let target = robot.fk(&random_joints());
                let setup = Setup::new(&robot, &target);
                let poly = TrigPoly::from_setup(&setup);

                let mut scale = 0.0f64;
                for index in 0..64 {
                    scale = scale.max(poly.eval(2.0 * PI * index as f64 / 64.0).0.abs());
                }

                // This residual check allows a candidate beside a merged pair to carry a real
                // residual. Across half a million measured poses, the worst residual was one
                // thousandth of this bound. The bound also rejects a candidate at an arbitrary
                // angle.
                for theta in theta_roots(&poly) {
                    assert!(
                        setup.f(theta).abs() < 1e-2 * scale,
                        "f was {:e} at a reported root, against a scale of {:e}",
                        setup.f(theta),
                        scale
                    );
                }
            }
        }
    }

    #[test]
    fn the_root_a_real_configuration_sits_at_is_found() {
        for robot in all_robots() {
            for _ in 0..1000 {
                let joints = random_joints();
                let frames = robot.fk_all(&joints);
                let setup = Setup::new(&robot, &frames[5]);
                let poly = TrigPoly::from_setup(&setup);
                let expected = true_theta(&setup, &frames);

                let found = theta_roots(&poly);
                let nearest = found
                    .iter()
                    .map(|t| wrap_pi(t - expected).abs())
                    .fold(f64::INFINITY, f64::min);

                // This completeness check uses a bound of one hundredth of a radian because a
                // merged pair leaves the angle uncertain by about half a thousandth. Per-branch
                // polishing, which occurs outside this module, reduces that uncertainty.
                assert!(
                    nearest < 1e-2,
                    "nearest root was {:e} away from the configuration's own angle",
                    nearest
                );
            }
        }
    }

    #[test]
    fn there_are_never_more_than_eight() {
        for robot in all_robots() {
            for _ in 0..1000 {
                let target = robot.fk(&random_joints());
                let poly = TrigPoly::from_setup(&Setup::new(&robot, &target));
                let found = theta_roots(&poly);
                assert!(found.len() <= 2 * DEGREE, "found {} roots", found.len());
            }
        }
    }

    #[test]
    fn both_root_finders_agree() {
        // The solver reaches the companion matrix only if the iteration fails, and no such failure
        // has been observed. Compare the methods directly to verify that each method finds every
        // root found by the other. Near the J1 axis, up to four roots cluster together. Any method
        // locates a root of multiplicity four only to approximately the fourth root of machine
        // epsilon, so the tolerance accounts for that limit.
        for robot in all_robots() {
            for _ in 0..300 {
                let target = robot.fk(&random_joints());
                let poly = TrigPoly::from_setup(&Setup::new(&robot, &target));
                let coefficients = poly.shifted(poly.origin_shift()).half_angle_polynomial();
                let degree = coefficients
                    .iter()
                    .rposition(|c| c.abs() > 0.0)
                    .expect("a nonzero polynomial");
                let coefficients = &coefficients[..=degree];

                let iterated = aberth_roots(coefficients).expect("the iteration converged");
                let eigen = companion_roots(coefficients).expect("the eigenvalues converged");
                assert_eq!(iterated.len(), degree);
                assert_eq!(eigen.len(), degree);

                let scale = iterated.iter().map(|r| r.norm()).fold(1.0f64, f64::max);
                for root in &iterated {
                    let nearest = eigen
                        .iter()
                        .map(|e| (e - root).norm())
                        .fold(f64::INFINITY, f64::min);
                    assert!(
                        nearest < 1e-3 * scale,
                        "root {root} was {nearest:e} from any eigenvalue"
                    );
                }
            }
        }
    }

    #[test]
    fn the_fallback_finds_the_simple_roots() {
        // The bracketing path is only reached if both root finders fail, which has not been
        // observed, so it is exercised directly here.
        for robot in all_robots() {
            for _ in 0..200 {
                let target = robot.fk(&random_joints());
                let setup = Setup::new(&robot, &target);
                let poly = TrigPoly::from_setup(&setup);

                let mut scale = 0.0f64;
                for index in 0..64 {
                    scale = scale.max(poly.eval(2.0 * PI * index as f64 / 64.0).0.abs());
                }

                for theta in bracketed_roots(&poly) {
                    assert!(setup.f(theta).abs() < 1e-9 * scale);
                }
            }
        }
    }
}
