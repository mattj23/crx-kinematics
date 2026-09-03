//! This benchmark measures each solver stage and the complete pipeline.
//!
//! The stages are measured separately because they answer different questions. Sampling the
//! constraint evaluates a fixed expression and should have consistent cost. Root recovery runs an
//! eigenvalue iteration whose cost depends on the input matrix. Separate measurements identify
//! which operation caused a performance change.
//!
//! Every measurement runs over the same corpus of poses, drawn from a fixed seed, so that two runs
//! are comparable and measurement changes reflect code changes.

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use crx_kinematics::ik::roots::theta_roots;
use crx_kinematics::ik::setup::Setup;
use crx_kinematics::ik::trig_poly::TrigPoly;
use crx_kinematics::{Crx, Iso3};
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};
use std::hint::black_box;

/// The corpus contains enough poses to prevent one difficult configuration from dominating.
const CORPUS: usize = 512;

/// The fixed seed produces the same corpus for every run.
const SEED: u64 = 20260903;

/// A corpus of reachable poses, taken from random configurations so that every one of them has at
/// least one solution.
fn corpus(robot: &Crx) -> Vec<Iso3> {
    let mut rng = StdRng::seed_from_u64(SEED);
    (0..CORPUS)
        .map(|_| {
            let joints: [f64; 6] = std::array::from_fn(|_| rng.random_range(-180.0..=180.0));
            robot.fk(&joints)
        })
        .collect()
}

/// Run all five measurements in one group, with throughput set to the corpus size so that Criterion
/// reports the cost per pose.
fn solver_stages(c: &mut Criterion) {
    let robot = Crx::new_10ia();
    let targets = corpus(&robot);

    let mut rng = StdRng::seed_from_u64(SEED);
    let configurations: Vec<[f64; 6]> = (0..CORPUS)
        .map(|_| std::array::from_fn(|_| rng.random_range(-180.0..=180.0)))
        .collect();

    let polynomials: Vec<TrigPoly> = targets
        .iter()
        .map(|target| TrigPoly::from_setup(&Setup::new(&robot, target)))
        .collect();

    let mut group = c.benchmark_group("solver stages");
    group.throughput(Throughput::Elements(CORPUS as u64));

    group.bench_function(BenchmarkId::new("forward kinematics", CORPUS), |b| {
        b.iter(|| {
            for joints in &configurations {
                black_box(robot.fk(black_box(joints)));
            }
        })
    });

    group.bench_function(BenchmarkId::new("constraint coefficients", CORPUS), |b| {
        b.iter(|| {
            for target in &targets {
                let setup = Setup::new(&robot, black_box(target));
                black_box(TrigPoly::from_setup(&setup));
            }
        })
    });

    group.bench_function(BenchmarkId::new("origin shift and conversion", CORPUS), |b| {
        b.iter(|| {
            for poly in &polynomials {
                let shift = black_box(poly).origin_shift();
                black_box(poly.shifted(shift).half_angle_polynomial());
            }
        })
    });

    // Root finding performs and includes the origin shift and polynomial conversion.
    group.bench_function(BenchmarkId::new("root finding", CORPUS), |b| {
        b.iter(|| {
            for poly in &polynomials {
                black_box(theta_roots(black_box(poly)));
            }
        })
    });

    // Measures the current pipeline from a target pose to its candidate angles.
    group.bench_function(BenchmarkId::new("target to roots", CORPUS), |b| {
        b.iter(|| {
            for target in &targets {
                let setup = Setup::new(&robot, black_box(target));
                black_box(theta_roots(&TrigPoly::from_setup(&setup)));
            }
        })
    });

    // Measures the complete solve from a target pose to every joint configuration that reaches it.
    group.bench_function(BenchmarkId::new("inverse kinematics", CORPUS), |b| {
        b.iter(|| {
            for target in &targets {
                black_box(robot.ik(black_box(target)));
            }
        })
    });

    // Measures the complete solve and selection of the solution nearest the reference joints.
    let reference = [0.0; 6];
    group.bench_function(BenchmarkId::new("nearest solution", CORPUS), |b| {
        b.iter(|| {
            for target in &targets {
                black_box(robot.ik_closest(black_box(target), &reference));
            }
        })
    });

    group.finish();
}

criterion_group!(benches, solver_stages);
criterion_main!(benches);
