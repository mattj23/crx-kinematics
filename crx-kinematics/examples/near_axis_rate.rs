//! Measures how often the solver recovers the configuration that generated a pose as O4 approaches
//! the J1 axis.
//!
//! The offset is added to the controller's J3 after the joint values have been chosen to put O4
//! exactly on the axis, so an offset of a thousandth of a degree moves O4 a few micrometers off
//! it. A miss is counted whenever no reported solution is within a hundredth of a degree of the
//! generating joints. Poses are also classified by the exclusions the integration tests apply,
//! which keep O4 a millimeter from the world origin and the plane five millimeters clear of a
//! tangency with the circle of O3 candidates.
//!
//! `near_axis_rate <seed> <count>`
use crx_kinematics::Point3;
use crx_kinematics::ik::polish::joint_distance;
use crx_kinematics::{Crx, CrxModel};
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};

fn slice_margin(robot: &Crx, joints: &[f64; 6]) -> f64 {
    let frames = robot.fk_all(joints);
    let o4 = frames[3] * Point3::new(robot.x1(), 0.0, 0.0);
    let o5 = frames[4] * Point3::origin();
    let u = (o4 - o5) / robot.y1();
    let distance = o4.coords.norm();
    if distance == 0.0 {
        return 0.0;
    }
    let along = (robot.z1().powi(2) - robot.x1().powi(2) + distance.powi(2)) / (2.0 * distance);
    let radius = (robot.z1().powi(2) - along.powi(2)).max(0.0).sqrt();
    let center_z = if o4.z == 0.0 {
        0.0
    } else {
        along * o4.z.signum()
    };
    let reach = radius * u.x.hypot(u.y);
    let wanted = ((o4.z - center_z) * u.z).abs();
    reach.min(reach - wanted)
}
fn testable(robot: &Crx, joints: &[f64; 6]) -> bool {
    let frames = robot.fk_all(joints);
    let o4 = frames[3] * Point3::new(robot.x1(), 0.0, 0.0);
    o4.coords.norm() >= 1.0 && slice_margin(robot, joints) >= 5.0
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let seed: u64 = args[1].parse().unwrap();
    let count: usize = args[2].parse().unwrap();
    let models = [
        (Crx::from_model(CrxModel::Crx5iA), "5ia"),
        (Crx::from_model(CrxModel::Crx10iAL), "10ial"),
        (Crx::from_model(CrxModel::Crx30iA), "30ia"),
    ];
    for (robot, name) in &models {
        for offset in [1e-2, 1e-3, 1e-4, 1e-5] {
            let mut rng = StdRng::seed_from_u64(seed);
            let (mut used, mut missing, mut total_solutions, mut testable_n, mut missing_testable) =
                (0usize, 0usize, 0usize, 0usize, 0usize);
            while used < count {
                let mut joints: [f64; 6] =
                    std::array::from_fn(|_| rng.random_range(-180.0..=180.0));
                let ratio = -robot.z1() * joints[1].to_radians().sin() / robot.x1();
                if ratio.abs() > 1.0 {
                    continue;
                }
                joints[2] = ratio.acos().to_degrees() + offset;
                used += 1;
                let sols = robot.ik(&robot.fk(&joints));
                total_solutions += sols.len();
                let ok = sols
                    .iter()
                    .any(|s| joint_distance(&s.joints, &joints) < 0.01f64.to_radians());
                let t = testable(robot, &joints);
                if t {
                    testable_n += 1;
                }
                if !ok {
                    missing += 1;
                    if t {
                        missing_testable += 1;
                        println!(
                            "  missing testable: {joints:?} margin {:.2}",
                            slice_margin(robot, &joints)
                        );
                    }
                }
            }
            println!(
                "{name} offset {offset:e}: generator missing {missing} of {count} (testable: {missing_testable} of {testable_n}); total solutions {total_solutions}"
            );
        }
    }
}
