//! Dumps the solver's output for a large corpus or compares the output with a previous dump to
//! verify that the solver still finds every previously reported solution.
//!
//! `ik_corpus dump <file>` writes every solution for every pose in the corpus.
//! `ik_corpus check <file>` solves the same corpus and compares the results with the dump. The
//! corpus contains twenty thousand random poses per model and approximately one thousand poses
//! from each degenerate family. The check classifies solutions that the current solver no longer
//! finds as shifts of less than a tenth of a degree, joint values in the thousands of degrees
//! produced by runaway polishing, members of a continuum represented by a single solution, or
//! genuine losses. For each family, it also reports how many poses have generating joints missing
//! from the reference output and the current output.
use crx_kinematics::ik::SolutionKind;
use crx_kinematics::ik::polish::joint_distance;
use crx_kinematics::{Crx, CrxModel};
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::time::Instant;

const RANDOM: usize = 20000;
const FAMILY: usize = 1000;

fn models() -> [(Crx, &'static str); 6] {
    [
        (Crx::from_model(CrxModel::Crx3iA), "3ia"),
        (Crx::from_model(CrxModel::Crx5iA), "5ia"),
        (Crx::from_model(CrxModel::Crx10iA), "10ia"),
        (Crx::from_model(CrxModel::Crx10iAL), "10ial"),
        (Crx::from_model(CrxModel::Crx20iAL), "20ial"),
        (Crx::from_model(CrxModel::Crx30iA), "30ia"),
    ]
}

fn on_axis(robot: &Crx, mut joints: [f64; 6]) -> Option<[f64; 6]> {
    let ratio = -robot.z1() * joints[1].to_radians().sin() / robot.x1();
    if ratio.abs() > 1.0 {
        return None;
    }
    joints[2] = ratio.acos().to_degrees();
    Some(joints)
}

/// Returns every `(family name, joints)` pair in the corpus in a fixed order.
fn corpus(robot: &Crx) -> Vec<(String, [f64; 6])> {
    let mut rng = StdRng::seed_from_u64(20260903);
    let mut draw = || -> [f64; 6] { std::array::from_fn(|_| rng.random_range(-180.0..=180.0)) };
    let mut out = Vec::new();
    for _ in 0..RANDOM {
        out.push(("random".to_string(), draw()));
    }
    for _ in 0..FAMILY {
        if let Some(j) = on_axis(robot, draw()) {
            out.push(("on_axis".to_string(), j));
        }
    }
    for offset in [1e-9, 1e-7, 1e-5, 1e-3, 1e-1] {
        for _ in 0..FAMILY / 2 {
            if let Some(mut j) = on_axis(robot, draw()) {
                j[2] += offset;
                out.push((format!("near_axis_{offset:e}"), j));
            }
        }
    }
    for value in [0.0, 1e-9, 1e-6, 1e-3] {
        for _ in 0..FAMILY / 2 {
            let mut j = draw();
            j[3] = value;
            out.push((format!("elbow_{value:e}"), j));
        }
    }
    for value in [0.0, 180.0, 1e-6, 1e-3] {
        for _ in 0..FAMILY / 2 {
            let mut j = draw();
            j[4] = value;
            out.push((format!("wrist_{value:e}"), j));
        }
    }
    for _ in 0..FAMILY {
        let mut j = draw();
        j[1] = 90.0;
        j[2] = 0.0;
        out.push(("stretched".to_string(), j));
    }
    for _ in 0..FAMILY / 2 {
        let mut j = draw();
        j[1] = 90.0;
        j[2] = 1e-4;
        out.push(("near_stretched".to_string(), j));
    }
    for elbow in [90.0, -90.0] {
        out.push((
            "base_free".to_string(),
            [35.0, 0.0, elbow, 20.0, -40.0, 15.0],
        ));
    }
    out.push((
        "issue_1".to_string(),
        [10.0, -80.0, 10.0, 20.0, -20.0, 45.0],
    ));
    out
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let (mode, path) = (&args[1], &args[2]);
    let start = Instant::now();
    let mut poses = 0usize;

    if mode == "dump" {
        let mut w = BufWriter::new(std::fs::File::create(path).unwrap());
        for (robot, name) in models() {
            for (family, joints) in corpus(&robot) {
                let sols = robot.ik(&robot.fk(&joints));
                poses += 1;
                writeln!(w, "{name} {family} {}", sols.len()).unwrap();
                for s in &sols {
                    let j = s.joints;
                    writeln!(
                        w,
                        "{} {} {} {} {} {} {:?} {:e}",
                        j[0], j[1], j[2], j[3], j[4], j[5], s.kind, s.residual
                    )
                    .unwrap();
                }
            }
        }
    } else {
        let r = BufReader::new(std::fs::File::open(path).unwrap());
        let mut lines = r.lines().map(|l| l.unwrap());
        let (
            mut lost,
            mut gained,
            mut kind_changed,
            mut worse,
            mut shifted,
            mut garbage,
            mut members,
        ) = (0usize, 0usize, 0usize, 0usize, 0usize, 0usize, 0usize);
        let mut worst_residual = 0.0f64;
        let mut generator_missing: std::collections::BTreeMap<String, (usize, usize)> =
            Default::default();
        for (robot, name) in models() {
            for (family, joints) in corpus(&robot) {
                let header = lines.next().unwrap();
                let mut parts = header.split(' ');
                assert_eq!(parts.next().unwrap(), name);
                assert_eq!(parts.next().unwrap(), family);
                let n: usize = parts.next().unwrap().parse().unwrap();
                let mut old = Vec::new();
                for _ in 0..n {
                    let line = lines.next().unwrap();
                    let p: Vec<&str> = line.split(' ').collect();
                    let j: [f64; 6] = std::array::from_fn(|i| p[i].parse().unwrap());
                    old.push((j, p[6].to_string()));
                }
                let new = robot.ik(&robot.fk(&joints));
                poses += 1;
                let tol = 0.01f64.to_radians();
                let old_has = old.iter().any(|(j, _)| joint_distance(j, &joints) < tol);
                let new_has = new.iter().any(|s| joint_distance(&s.joints, &joints) < tol);
                if !old_has || !new_has {
                    let key = format!("{name} {family}");
                    let entry = generator_missing.entry(key).or_insert((0usize, 0usize));
                    if !old_has {
                        entry.0 += 1;
                    }
                    if !new_has {
                        entry.1 += 1;
                        if old_has {
                            println!("GENERATOR-NEWLY-MISSING {name} {family} {joints:?}");
                        }
                    }
                }
                for s in &new {
                    worst_residual = worst_residual.max(s.residual);
                    if s.residual > 1e-8 {
                        worse += 1;
                    }
                }
                for (j, kind) in &old {
                    match new.iter().find(|s| joint_distance(&s.joints, j) < 1e-5) {
                        None if new
                            .iter()
                            .any(|s| joint_distance(&s.joints, j) < 0.1f64.to_radians()) =>
                        {
                            shifted += 1;
                        }
                        None if j.iter().any(|v| v.abs() > 1000.0) => {
                            garbage += 1;
                        }
                        None if new.iter().any(|s| s.kind == SolutionKind::SingularFamily) && {
                            let frames = robot.fk_all(j);
                            let o4 = frames[3] * crx_kinematics::Point3::new(robot.x1(), 0.0, 0.0);
                            o4.x.hypot(o4.y) < 1e-6 * robot.z1()
                        } =>
                        {
                            members += 1;
                        }
                        None => {
                            lost += 1;
                            if lost <= 1000 {
                                let frames = robot.fk_all(j);
                                let o4 =
                                    frames[3] * crx_kinematics::Point3::new(robot.x1(), 0.0, 0.0);
                                let o3 = frames[2] * crx_kinematics::Point3::origin();
                                println!(
                                    "LOST {name} {family} joints {joints:?}: {j:?} ({kind}) o4 rho/z1 {:e} o3 rho/z1 {:e} |o4|/z1 {:e}",
                                    o4.x.hypot(o4.y) / robot.z1(),
                                    o3.x.hypot(o3.y) / robot.z1(),
                                    o4.coords.norm() / robot.z1()
                                );
                            }
                        }
                        Some(s) => {
                            if format!("{:?}", s.kind) != *kind {
                                kind_changed += 1;
                                if kind_changed <= 10 {
                                    println!(
                                        "KIND {name} {family} {joints:?}: {kind} -> {:?}",
                                        s.kind
                                    );
                                }
                            }
                        }
                    }
                }
                for s in &new {
                    if !old
                        .iter()
                        .any(|(j, _)| joint_distance(&s.joints, j) < 0.1f64.to_radians())
                    {
                        gained += 1;
                        if gained <= 1000 {
                            println!(
                                "GAINED {name} {family} joints {joints:?}: {:?} ({:?})",
                                s.joints, s.kind
                            );
                        }
                    }
                }
            }
        }
        for (key, (o, n)) in &generator_missing {
            println!("generator missing in {key}: reference {o}, new {n}");
        }
        println!(
            "lost {lost}, shifted {shifted}, runaway-garbage {garbage}, continuum members {members}, gained {gained}, kind changed {kind_changed}, above POSE_TOL {worse}, worst residual {worst_residual:e}"
        );
    }
    println!("{poses} poses in {:.1} s", start.elapsed().as_secs_f64());
}
