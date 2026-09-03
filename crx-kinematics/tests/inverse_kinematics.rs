//! These tests cover solver round trips and degenerate configuration families.
//!
//! Every test here works the same way: pick joint values, take the pose they produce, and require
//! the solver to find those joint values again among its solutions while every solution it returns
//! lands on the pose. The families are set up deliberately, because the geometries that a naive
//! solver mishandles are the ones random draws never produce.

use crx_kinematics::ik::SolutionKind;
use crx_kinematics::ik::polish::joint_distance;
use crx_kinematics::{Crx, CrxModel, Iso3};
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};

/// Millimeters. This value is tighter than the solver's acceptance threshold, so the test rejects
/// a solution that only meets that threshold.
const POSE_TOL: f64 = 1e-9;

/// Millimeters, for poses where a joint is free. The pose is insensitive to motion along the free
/// direction, so the polish has no gradient in that direction and stops slightly short.
const SINGULAR_POSE_TOL: f64 = 1e-8;

/// Millimeters, for a pose that sits beside the J1 axis without being on it.
///
/// The on-axis construction places O4 on the axis and leaves the polish to absorb the ignored
/// distance. When the configuration has an additional numerical difficulty, the polish can stop a
/// few nanometers short. Across every measured model and offset, the largest error was 2.1e-9.
const NEAR_AXIS_POSE_TOL: f64 = 1e-8;

/// Radians, for recognizing the joint vector that generated a pose.
const JOINT_TOL: f64 = 1e-5;

/// Millimeters. How cleanly the plane must cut the circle of candidate O3 points.
///
/// With O4 on the J1 axis, O3 lies on a horizontal circle, and the perpendicularity constraint
/// slices that circle with a plane. Two things spoil the slice: a circle with no extent, which
/// happens where the two spheres locating O3 are tangent, and a plane that only grazes the circle.
/// In either case, O3's position along the circle is undetermined and polishing cannot recover it.
/// The on-axis tests deliberately exclude poses within this margin of either condition. See
/// `docs/LINEAR_METHOD.md`.
const SLICE_EXCLUSION: f64 = 5.0;

/// Millimeters. How near the world origin O4 may come before a pose is left out.
///
/// This failure is separate from the collapsed slice above and occurs only where `z1` equals `x1`,
/// because O4 must reach the axis and origin simultaneously. The two spheres locating O3 become
/// concentric there, so J1 and J2 both lose their effect. Passing *near* the origin is less stable
/// than passing through it. See `docs/LINEAR_METHOD.md`.
const ORIGIN_EXCLUSION: f64 = 1.0;

fn all_models() -> [Crx; 6] {
    [
        Crx::from_model(CrxModel::Crx3iA),
        Crx::from_model(CrxModel::Crx5iA),
        Crx::from_model(CrxModel::Crx10iA),
        Crx::from_model(CrxModel::Crx10iAL),
        Crx::from_model(CrxModel::Crx20iAL),
        Crx::from_model(CrxModel::Crx30iA),
    ]
}

fn seeded() -> StdRng {
    StdRng::seed_from_u64(20260903)
}

fn random_joints(rng: &mut StdRng) -> [f64; 6] {
    std::array::from_fn(|_| rng.random_range(-180.0..=180.0))
}

/// How far O4 is from the world origin for a configuration.
fn o4_from_origin(robot: &Crx, joints: &[f64; 6]) -> f64 {
    let frames = robot.fk_all(joints);
    (frames[3] * crx_kinematics::Point3::new(robot.x1(), 0.0, 0.0))
        .coords
        .norm()
}

/// Whether a configuration is one the on-axis tests can ask about, meaning its O4 keeps clear of
/// the world origin and the plane cuts the circle of candidate O3 points cleanly.
fn is_testable(robot: &Crx, joints: &[f64; 6]) -> bool {
    o4_from_origin(robot, joints) >= ORIGIN_EXCLUSION
        && slice_margin(robot, joints) >= SLICE_EXCLUSION
}

/// How cleanly the plane cuts the circle of candidate O3 points, for a configuration whose O4 is
/// on the axis. Zero or less means the slice is degenerate.
///
/// The circle has radius `R` at height `center_z`, and the constraint reads
/// `R u.x cos(a) + R u.y sin(a) = (height - center_z) u.z`. The left side reaches
/// `R hypot(u.x, u.y)`, so that quantity less the right side is how much room the slice has.
fn slice_margin(robot: &Crx, joints: &[f64; 6]) -> f64 {
    let frames = robot.fk_all(joints);
    let o4 = frames[3] * crx_kinematics::Point3::new(robot.x1(), 0.0, 0.0);
    let o5 = frames[4] * crx_kinematics::Point3::origin();
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

/// Round trips over `count` poses, each put onto a family by `mutate`.
///
/// A mutation may refuse a draw it cannot put onto its family, which is how the on-axis case
/// handles a shoulder angle the forearm cannot reach around.
fn sweep<F>(robot: &Crx, count: usize, mutate: F)
where
    F: Fn(&Crx, [f64; 6]) -> Option<[f64; 6]>,
{
    sweep_with(robot, count, POSE_TOL, mutate)
}

/// As `sweep`, with the pose tolerance stated by the caller.
fn sweep_with<F>(robot: &Crx, count: usize, pose_tol: f64, mutate: F)
where
    F: Fn(&Crx, [f64; 6]) -> Option<[f64; 6]>,
{
    let mut rng = seeded();
    let mut used = 0;

    for _ in 0..count {
        let Some(joints) = mutate(robot, random_joints(&mut rng)) else {
            continue;
        };
        used += 1;

        let target = robot.fk(&joints);
        let solutions = robot.ik(&target);
        assert!(!solutions.is_empty(), "no solutions for {joints:?}");
        assert!(solutions.len() <= 16, "{} solutions", solutions.len());

        // A pose with a free joint has a continuum of solutions. The returned representative can
        // differ from the joint values that produced the pose, so this test verifies only that its
        // solutions reach the pose.
        let has_free_joint = solutions
            .iter()
            .any(|s| s.kind == SolutionKind::SingularFamily);
        let tolerance = if has_free_joint {
            SINGULAR_POSE_TOL
        } else {
            pose_tol
        };
        for solution in &solutions {
            assert!(
                solution.residual < tolerance,
                "residual {:e} for {:?}",
                solution.residual,
                joints
            );
        }
        if has_free_joint {
            continue;
        }

        assert_eq!(
            solutions.len() % 2,
            0,
            "solutions come in front and back pairs, got {}",
            solutions.len()
        );
        let nearest = solutions
            .iter()
            .map(|s| joint_distance(&s.joints, &joints))
            .fold(f64::INFINITY, f64::min);
        assert!(
            nearest < JOINT_TOL,
            "{joints:?} not recovered, nearest {nearest:e}"
        );
    }

    assert!(used > count / 8, "only {used} of {count} draws were usable");
}

fn unchanged(_: &Crx, joints: [f64; 6]) -> Option<[f64; 6]> {
    Some(joints)
}

/// Joint values that put O4 exactly on the J1 axis.
///
/// In the frame J1 turns to, O4 sits at `z1 sin(J2) + x1 cos(J3)` off the axis, with J3 as the
/// controller reports it. Setting that to zero fixes J3 from J2. On a model with `z1` larger than
/// `x1` some shoulder angles put the axis out of the forearm's reach, and those are refused, as
/// are draws landing on either of the two geometries the solver cannot recover.
///
/// Note that `J3 = J2 + 90` solves this only where the two lengths are equal, which is to say on
/// the CRX-3iA and the CRX-10iA and nowhere else.
fn on_axis(robot: &Crx, mut joints: [f64; 6]) -> Option<[f64; 6]> {
    let ratio = -robot.z1() * joints[1].to_radians().sin() / robot.x1();
    if ratio.abs() > 1.0 {
        return None;
    }
    joints[2] = ratio.acos().to_degrees();
    if !is_testable(robot, &joints) {
        return None;
    }
    Some(joints)
}

#[test]
fn random_poses_round_trip() {
    for robot in all_models() {
        sweep(&robot, 2000, unchanged);
    }
}

#[test]
fn o4_on_the_j1_axis() {
    // The plane constraint provides no information when O4 is on the axis, so O3 determines the
    // base angle. The pose in issue #1 belongs to this family.
    for robot in all_models() {
        sweep(&robot, 400, on_axis);
    }
}

#[test]
fn near_the_j1_axis() {
    // Passing near the axis is less stable than passing through it because the vertical plane
    // through O4 remains defined but swings through a wide arc as O4 passes. The sweep covers
    // offsets from a nanometer-scale miss to a comfortable distance.
    for offset in [1e-9, 1e-7, 1e-5, 1e-3, 1e-1] {
        for robot in all_models() {
            sweep_with(&robot, 150, NEAR_AXIS_POSE_TOL, |robot, joints| {
                let mut joints = on_axis(robot, joints)?;
                joints[2] += offset;
                // The offset moves O4, so apply the exclusion again afterward.
                is_testable(robot, &joints).then_some(joints)
            });
        }
    }
}

#[test]
fn the_elbow_pair_merged() {
    // Where the elbow-up and elbow-down solutions merge, the constraint has a double root.
    for value in [0.0, 1e-9, 1e-6, 1e-3] {
        for robot in all_models() {
            sweep(&robot, 250, |_, mut joints| {
                joints[3] = value;
                Some(joints)
            });
        }
    }
}

#[test]
fn the_wrist_singular() {
    // J4 and J6 turn against each other with no effect on the pose.
    for robot in all_models() {
        sweep(&robot, 400, |_, mut joints| {
            joints[4] = 0.0;
            Some(joints)
        });
    }
}

#[test]
fn the_arm_fully_stretched() {
    // With the upper arm and forearm in line, the two spheres that locate O3 are tangent and the
    // radius of their intersection circle is zero, or a rounding error below it.
    for robot in all_models() {
        sweep(&robot, 400, |_, mut joints| {
            joints[1] = 90.0;
            joints[2] = 0.0;
            Some(joints)
        });
    }
}

#[test]
fn the_base_free() {
    // With the shoulder upright and the forearm turned a quarter turn, O3 and O4 are both on the
    // J1 axis and the base angle no longer changes the pose. Because the solution set is continuous,
    // verify that the solver reports and marks a representative that reaches the target.
    for robot in all_models() {
        for elbow in [90.0, -90.0] {
            let joints = [35.0, 0.0, elbow, 20.0, -40.0, 15.0];
            let solutions = robot.ik(&robot.fk(&joints));

            assert!(
                solutions
                    .iter()
                    .any(|s| s.kind == SolutionKind::SingularFamily),
                "no representative reported"
            );
            for solution in &solutions {
                assert!(solution.residual < SINGULAR_POSE_TOL);
            }
        }
    }
}

#[test]
fn out_of_reach() {
    let robot = Crx::new_10ia();
    let mut target = robot.fk(&[0.0; 6]);
    target.translation.vector.x += 10_000.0;
    assert!(robot.ik(&target).is_empty());
    assert!(robot.ik_closest(&target, &[0.0; 6]).is_none());
}

#[test]
fn the_issue_one_pose() {
    // https://github.com/mattj23/crx-kinematics/issues/1. O4 lands exactly on the J1 axis here,
    // which caused failures in the libraries surveyed in the README. All sixteen solutions exist,
    // and they include the joint values that produced the pose.
    let robot = Crx::new_10ia();
    let joints = [10.0, -80.0, 10.0, 20.0, -20.0, 45.0];
    let solutions = robot.ik(&robot.fk(&joints));

    assert_eq!(solutions.len(), 16, "got {} solutions", solutions.len());
    assert!(
        solutions
            .iter()
            .any(|s| s.kind == SolutionKind::AxisDegenerate)
    );
    let nearest = solutions
        .iter()
        .map(|s| joint_distance(&s.joints, &joints))
        .fold(f64::INFINITY, f64::min);
    assert!(nearest < JOINT_TOL, "nearest was {nearest:e}");
}

#[test]
fn solutions_reach_full_precision() {
    // Every solution is expected to reach the limit of double precision. This requirement is
    // stricter than the acceptance tolerance and verifies the benefit of joint-space polishing.
    let mut rng = seeded();
    let mut worst = 0.0f64;
    for robot in all_models() {
        for _ in 0..500 {
            let target = robot.fk(&random_joints(&mut rng));
            for solution in robot.ik(&target) {
                worst = worst.max(solution.residual);
            }
        }
    }
    assert!(worst < 1e-11, "worst residual was {worst:e}");
}

#[test]
fn closest_to_a_reference() {
    // Asking for the pose the robot is already in must return the configuration it is in.
    let mut rng = seeded();
    for robot in all_models() {
        for _ in 0..300 {
            let joints = random_joints(&mut rng);
            let solution = robot
                .ik_closest(&robot.fk(&joints), &joints)
                .expect("a reachable pose");
            assert!(joint_distance(&solution.joints, &joints) < JOINT_TOL);
        }
    }
}

#[test]
fn solutions_are_distinct() {
    let mut rng = seeded();
    let robot = Crx::new_10ia();
    for _ in 0..300 {
        let target: Iso3 = robot.fk(&random_joints(&mut rng));
        let solutions = robot.ik(&target);
        for (index, first) in solutions.iter().enumerate() {
            for second in &solutions[index + 1..] {
                assert!(joint_distance(&first.joints, &second.joints) > 1e-6);
            }
        }
    }
}
