//! This module represents the scalar constraint as a trigonometric polynomial and converts it to
//! an ordinary polynomial.
//!
//! The constraint `f` is a trigonometric polynomial of degree four because the surplus higher-order
//! terms in its expression cancel. This property is recorded in `docs/LINEAR_METHOD.md` and checked
//! by a test below on every model in the family.
//!
//! A trigonometric polynomial of degree four has nine coefficients, so sixteen evenly spaced
//! samples recover them *exactly*. Afterward, `f` and its derivative can be evaluated at any angle
//! without evaluating the geometry again.

use super::setup::Setup;
use std::f64::consts::PI;

/// The degree of `f` as a trigonometric polynomial.
pub const DEGREE: usize = 4;

/// How many samples are taken to recover the coefficients.
///
/// Nine would be enough in principle. Sixteen is used because it leaves the first harmonic that
/// could alias into the range at twelve, far above anything present, and because the extra cost is
/// seven evaluations of a short expression.
pub const SAMPLE_COUNT: usize = 4 * DEGREE;

/// Largest angle, in radians, that a single Newton step on `f` is allowed to move.
pub const MAX_NEWTON_STEP: f64 = 0.05;

/// The coefficients of `f` written as `a[0] + sum over k of a[k] cos(k theta) + b[k] sin(k theta)`.
///
/// `b[0]` is unused and always zero, so that the two arrays are indexed by harmonic throughout.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrigPoly {
    /// The cosine coefficients, with the constant term at index zero.
    pub a: [f64; DEGREE + 1],

    /// The sine coefficients. Index zero is unused.
    pub b: [f64; DEGREE + 1],
}

impl TrigPoly {
    /// Recovers the coefficients of `f` by sampling it around a full turn.
    ///
    /// # Arguments
    ///
    /// * `setup`: the prepared target, whose `f` is sampled
    ///
    /// returns: TrigPoly
    pub fn from_setup(setup: &Setup) -> Self {
        let mut samples = [0.0; SAMPLE_COUNT];
        for (index, sample) in samples.iter_mut().enumerate() {
            *sample = setup.f(2.0 * PI * index as f64 / SAMPLE_COUNT as f64);
        }
        Self::from_samples(&samples)
    }

    /// Recovers the coefficients from values taken at evenly spaced angles starting at zero.
    ///
    /// This function directly implements a discrete Fourier transform. For this input size, the
    /// direct implementation is faster and avoids adding a transform dependency to the crate.
    ///
    /// # Arguments
    ///
    /// * `samples`: `f` evaluated at `2 pi i / SAMPLE_COUNT` for each index `i`
    ///
    /// returns: TrigPoly
    pub fn from_samples(samples: &[f64; SAMPLE_COUNT]) -> Self {
        let mut a = [0.0; DEGREE + 1];
        let mut b = [0.0; DEGREE + 1];
        let step = 2.0 * PI / SAMPLE_COUNT as f64;

        for (index, sample) in samples.iter().enumerate() {
            let theta = step * index as f64;
            a[0] += sample;
            for k in 1..=DEGREE {
                let (sin, cos) = (k as f64 * theta).sin_cos();
                a[k] += sample * cos;
                b[k] += sample * sin;
            }
        }

        a[0] /= SAMPLE_COUNT as f64;
        for k in 1..=DEGREE {
            a[k] *= 2.0 / SAMPLE_COUNT as f64;
            b[k] *= 2.0 / SAMPLE_COUNT as f64;
        }

        Self { a, b }
    }

    /// Evaluates the polynomial and its derivative at `theta`.
    ///
    /// # Arguments
    ///
    /// * `theta`: where to evaluate
    ///
    /// returns: (f64, f64), the value and the derivative
    pub fn eval(&self, theta: f64) -> (f64, f64) {
        let mut value = self.a[0];
        let mut derivative = 0.0;
        for k in 1..=DEGREE {
            let (sin, cos) = (k as f64 * theta).sin_cos();
            value += self.a[k] * cos + self.b[k] * sin;
            derivative += k as f64 * (self.b[k] * cos - self.a[k] * sin);
        }
        (value, derivative)
    }

    /// The coefficients of `f(phi + shift)` as a polynomial in `phi`.
    ///
    /// # Arguments
    ///
    /// * `shift`: how far to move the origin
    ///
    /// returns: TrigPoly
    pub fn shifted(&self, shift: f64) -> Self {
        let mut out = *self;
        for k in 1..=DEGREE {
            let (sin, cos) = (k as f64 * shift).sin_cos();
            out.a[k] = self.a[k] * cos + self.b[k] * sin;
            out.b[k] = self.b[k] * cos - self.a[k] * sin;
        }
        out
    }

    /// An origin shift that puts the half-angle substitution's point at infinity where `f` is
    /// largest, so that no root is pushed out of reach.
    ///
    /// The substitution below sends `phi = pi` to infinity. Choosing the shift so that `pi` lands
    /// on the largest value of `f` puts the lost point as far from any root as the samples can
    /// tell.
    ///
    /// returns: f64
    pub fn origin_shift(&self) -> f64 {
        const PROBES: usize = 64;
        let mut best = 0.0;
        let mut largest = -1.0;
        for index in 0..PROBES {
            let theta = 2.0 * PI * index as f64 / PROBES as f64;
            let magnitude = self.eval(theta).0.abs();
            if magnitude > largest {
                largest = magnitude;
                best = theta;
            }
        }
        best - PI
    }

    /// Rewrites the polynomial as an ordinary real polynomial in `t = tan(theta / 2)`.
    ///
    /// Substituting `exp(i theta) = (1 + i t)^2 / (1 + t^2)` and clearing the denominator with a
    /// factor of `(1 + t^2)^DEGREE` gives a real polynomial of degree `2 * DEGREE` whose real roots
    /// are the roots of `f`, save for one at `theta = pi` which the substitution sends to infinity.
    ///
    /// Writing `(1 + i t)^(2k) = u_k(t) + i v_k(t)`, the harmonic at `k` contributes
    /// `(a_k u_k + b_k v_k) (1 + t^2)^(DEGREE - k)`, which is real throughout, so no complex
    /// arithmetic is needed.
    ///
    /// returns: [f64; 2 * DEGREE + 1], coefficients in ascending powers of `t`
    pub fn half_angle_polynomial(&self) -> [f64; 2 * DEGREE + 1] {
        let mut out = [0.0; 2 * DEGREE + 1];

        // `(1 + i t)^(2k)`, built up one factor of `(1 + i t)^2 = (1 - t^2) + 2 i t` at a time.
        let mut u = [0.0; 2 * DEGREE + 1];
        let mut v = [0.0; 2 * DEGREE + 1];
        u[0] = 1.0;

        for k in 0..=DEGREE {
            if k > 0 {
                let (prev_u, prev_v) = (u, v);
                u = [0.0; 2 * DEGREE + 1];
                v = [0.0; 2 * DEGREE + 1];
                for i in 0..=(2 * DEGREE - 2) {
                    // multiply by (1 - t^2) + 2 i t
                    u[i] += prev_u[i];
                    v[i] += prev_v[i];
                    u[i + 1] -= 2.0 * prev_v[i];
                    v[i + 1] += 2.0 * prev_u[i];
                    u[i + 2] -= prev_u[i];
                    v[i + 2] -= prev_v[i];
                }
            }

            // The harmonic's contribution, before the remaining powers of (1 + t^2).
            let mut term = [0.0; 2 * DEGREE + 1];
            if k == 0 {
                term[0] = self.a[0];
            } else {
                for i in 0..=2 * DEGREE {
                    term[i] = self.a[k] * u[i] + self.b[k] * v[i];
                }
            }

            // Multiply by (1 + t^2), DEGREE - k times.
            for _ in 0..(DEGREE - k) {
                let previous = term;
                term = [0.0; 2 * DEGREE + 1];
                for i in 0..=(2 * DEGREE - 2) {
                    term[i] += previous[i];
                    term[i + 2] += previous[i];
                }
            }

            for i in 0..=2 * DEGREE {
                out[i] += term[i];
            }
        }

        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ik::setup::Setup;
    use crate::tests::{all_robots, random_joints};
    use approx::assert_relative_eq;
    use rand::RngExt;

    fn random_theta() -> f64 {
        let mut rng = rand::rng();
        rng.random_range(-PI..=PI)
    }

    #[test]
    fn coefficients_reproduce_the_constraint() {
        // Sixteen samples determine `f` at every angle, including angles between the samples.
        for robot in all_robots() {
            for _ in 0..200 {
                let target = robot.fk(&random_joints());
                let setup = Setup::new(&robot, &target);
                let poly = TrigPoly::from_setup(&setup);

                for _ in 0..10 {
                    let theta = random_theta();
                    assert_relative_eq!(poly.eval(theta).0, setup.f(theta), epsilon = 1e-12);
                }
            }
        }
    }

    #[test]
    fn the_derivative_matches_a_finite_difference() {
        for robot in all_robots() {
            for _ in 0..200 {
                let target = robot.fk(&random_joints());
                let poly = TrigPoly::from_setup(&Setup::new(&robot, &target));
                let theta = random_theta();
                let step = 1e-6;

                let numeric =
                    (poly.eval(theta + step).0 - poly.eval(theta - step).0) / (2.0 * step);
                assert_relative_eq!(poly.eval(theta).1, numeric, epsilon = 1e-6);
            }
        }
    }

    #[test]
    fn the_degree_is_four() {
        // Sampling twice as densely as the assumed degree needs must leave the extra harmonics at
        // zero. If a future CRX model broke this, the count of at most sixteen solutions would go
        // with it, so it is worth failing on.
        const DENSE: usize = 2 * SAMPLE_COUNT;
        for robot in all_robots() {
            for _ in 0..100 {
                let target = robot.fk(&random_joints());
                let setup = Setup::new(&robot, &target);

                let mut largest = 0.0f64;
                let mut above = 0.0f64;
                for k in 1..=(DENSE / 2) {
                    let (mut cos_sum, mut sin_sum) = (0.0, 0.0);
                    for index in 0..DENSE {
                        let theta = 2.0 * PI * index as f64 / DENSE as f64;
                        let value = setup.f(theta);
                        cos_sum += value * (k as f64 * theta).cos();
                        sin_sum += value * (k as f64 * theta).sin();
                    }
                    let magnitude = (cos_sum.powi(2) + sin_sum.powi(2)).sqrt() * 2.0 / DENSE as f64;
                    if k <= DEGREE {
                        largest = largest.max(magnitude);
                    } else {
                        above = above.max(magnitude);
                    }
                }

                assert!(
                    above < 1e-9 * largest,
                    "harmonics above {} carried {:e} against {:e}",
                    DEGREE,
                    above,
                    largest
                );
            }
        }
    }

    #[test]
    fn shifting_moves_the_origin() {
        for robot in all_robots() {
            for _ in 0..100 {
                let target = robot.fk(&random_joints());
                let poly = TrigPoly::from_setup(&Setup::new(&robot, &target));
                let shift = random_theta();
                let shifted = poly.shifted(shift);

                for _ in 0..10 {
                    let phi = random_theta();
                    assert_relative_eq!(
                        shifted.eval(phi).0,
                        poly.eval(phi + shift).0,
                        epsilon = 1e-12
                    );
                }
            }
        }
    }

    #[test]
    fn the_half_angle_polynomial_agrees_away_from_the_lost_point() {
        // The substitution multiplies through by (1 + t^2)^DEGREE, so the polynomial equals `f`
        // scaled by that factor rather than `f` itself.
        for robot in all_robots() {
            for _ in 0..200 {
                let target = robot.fk(&random_joints());
                let poly = TrigPoly::from_setup(&Setup::new(&robot, &target));
                let coefficients = poly.half_angle_polynomial();

                for _ in 0..10 {
                    let theta = random_theta() * 0.99; // stay away from the point at infinity
                    let t = (theta / 2.0).tan();

                    let mut evaluated = 0.0;
                    for (power, coefficient) in coefficients.iter().enumerate() {
                        evaluated += coefficient * t.powi(power as i32);
                    }
                    let expected = poly.eval(theta).0 * (1.0 + t * t).powi(DEGREE as i32);

                    // Scale the comparison by the polynomial coefficient magnitude, which
                    // determines the floating-point noise floor. The coefficients carry an
                    // absolute error of a few units in the last place (ulps) relative to their
                    // magnitude, and the conversion applies the same lift factor to that error.
                    // Near a root of `f`, both the value and the terms at this `t` collapse while
                    // the noise remains, so either scale would measure cancellation error instead
                    // of conversion error. Relative to the coefficient scale, the largest error
                    // across 1.2 million samples and every model was 9.9e-16. The threshold is one
                    // order of magnitude higher.
                    let scale: f64 = poly.a.iter().chain(poly.b.iter()).map(|v| v.abs()).sum();
                    let noise_floor = 1e-14 * scale * (1.0 + t * t).powi(DEGREE as i32);
                    assert!(
                        (evaluated - expected).abs() <= noise_floor,
                        "half-angle value differed by {:e} against a noise floor of {:e}",
                        (evaluated - expected).abs(),
                        noise_floor
                    );
                }
            }
        }
    }

    #[test]
    fn the_origin_shift_lands_on_a_large_value() {
        // The point the substitution loses should be nowhere near a root.
        for robot in all_robots() {
            for _ in 0..200 {
                let target = robot.fk(&random_joints());
                let poly = TrigPoly::from_setup(&Setup::new(&robot, &target));
                let shift = poly.origin_shift();

                let at_lost_point = poly.eval(PI + shift).0.abs();
                let mut largest = 0.0f64;
                for index in 0..256 {
                    largest = largest.max(poly.eval(2.0 * PI * index as f64 / 256.0).0.abs());
                }

                assert!(
                    at_lost_point > 0.5 * largest,
                    "{:e} vs {:e}",
                    at_lost_point,
                    largest
                );
            }
        }
    }
}
