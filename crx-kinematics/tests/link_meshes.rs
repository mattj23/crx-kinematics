// Cargo builds integration test targets for every feature set, so gate the entire file. This gate
// lets a `--no-default-features` build skip the tests instead of failing on the import below.
#![cfg(feature = "meshes")]

//! These tests verify that the embedded link geometry decodes into meshes for the specified robot
//! and that the posed meshes agree with the kinematics.
//!
//! Use exact counts and extents because the decoder is deterministic and the files do not change
//! after they are authored. These checks detect an accidental swap between the geometry files for
//! different models.

use crx_kinematics::{Crx, CrxModel, LinkMesh, LinkMeshes, meshes::LINK_COUNT};
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};

/// Millimeters. Vertex positions are quantized when the file is written, so bounds are compared
/// with the decoded values using a tolerance that accommodates the quantization.
const EXTENT_TOL: f64 = 1.0;

/// Millimeters. Posing applies a rigid transformation to exact vertex values, so this tolerance
/// accommodates only floating-point rounding.
const POSE_TOL: f64 = 1e-9;

/// The models with no embedded geometry.
const UNAVAILABLE: [CrxModel; 4] = [
    CrxModel::Crx3iA,
    CrxModel::Crx10iAL,
    CrxModel::Crx20iAL,
    CrxModel::Crx30iA,
];

/// Vertex and face counts for each of the seven CRX-5iA meshes, in order.
const CRX5IA_COUNTS: [(usize, usize); LINK_COUNT] = [
    (981, 1958),
    (2983, 5671),
    (6084, 11372),
    (2381, 4379),
    (2743, 5158),
    (3078, 5784),
    (1124, 2244),
];

/// Vertex and face counts for each of the seven CRX-10iA meshes, in order.
const CRX10IA_COUNTS: [(usize, usize); LINK_COUNT] = [
    (986, 1968),
    (3557, 6527),
    (7408, 14100),
    (6170, 11504),
    (3184, 6045),
    (3600, 6821),
    (1573, 3142),
];

fn counts_for(model: CrxModel) -> [(usize, usize); LINK_COUNT] {
    match model {
        CrxModel::Crx5iA => CRX5IA_COUNTS,
        CrxModel::Crx10iA => CRX10IA_COUNTS,
        other => panic!("no counts recorded for the {other:?}"),
    }
}

#[test]
fn the_two_available_models_decode_into_seven_coherent_meshes() {
    for model in [CrxModel::Crx5iA, CrxModel::Crx10iA] {
        assert!(LinkMeshes::is_available(model));

        let meshes = LinkMeshes::load(model).unwrap();
        let expected = counts_for(model);

        for (index, mesh) in meshes.links().iter().enumerate() {
            let (vertices, faces) = expected[index];
            assert_eq!(
                mesh.vertices.len(),
                vertices,
                "wrong vertex count for {model:?} mesh {index}"
            );
            assert_eq!(
                mesh.faces.len(),
                faces,
                "wrong face count for {model:?} mesh {index}"
            );

            // A face index beyond the vertex buffer makes the mesh invalid and can cause a renderer
            // to read out of bounds.
            for face in &mesh.faces {
                for index in face {
                    assert!(
                        (*index as usize) < mesh.vertices.len(),
                        "a face of {model:?} mesh {index} refers to a vertex which does not exist"
                    );
                }
            }
        }
    }
}

/// The minimum and maximum corner of one mesh, in millimeters.
type Bounds = ([f64; 3], [f64; 3]);

#[test]
fn the_meshes_have_the_extents_of_the_arm_they_belong_to() {
    // Millimeters, as the minimum and maximum corner of each mesh in its own frame.
    let expected: [(CrxModel, [Bounds; LINK_COUNT]); 2] = [
        (
            CrxModel::Crx5iA,
            [
                ([-100.0, -74.5, -185.0], [74.5, 74.5, -125.0]),
                ([-65.0, -90.5, -124.5], [65.0, 65.4, 65.4]),
                ([-65.0, -230.4, -65.1], [65.0, -67.5, 469.0]),
                ([-61.0, -66.5, -61.0], [120.5, 61.0, 61.0]),
                ([121.5, -59.0, -55.8], [484.0, 55.8, 55.8]),
                ([-55.6, -55.0, -55.0], [84.5, 70.5, 55.0]),
                ([-46.0, -62.5, -60.0], [46.0, 54.5, 0.0]),
            ],
        ),
        (
            CrxModel::Crx10iA,
            [
                ([-99.1, -95.0, -245.0], [95.0, 95.0, -160.0]),
                ([-79.0, -119.0, -159.0], [79.0, 80.2, 80.2]),
                ([-79.0, -298.4, -79.5], [79.0, -76.0, 608.1]),
                ([-70.5, -75.0, -70.2], [153.0, 63.9, 70.2]),
                ([154.0, -64.0, -57.0], [593.1, 58.1, 57.0]),
                ([-58.1, -58.1, -57.0], [99.5, 85.0, 57.0]),
                ([-46.0, -62.5, -59.5], [46.0, 54.6, 0.0]),
            ],
        ),
    ];

    for (model, boxes) in expected {
        let meshes = LinkMeshes::load(model).unwrap();
        for (index, mesh) in meshes.links().iter().enumerate() {
            let (min, max) = mesh.bounds().unwrap();
            for axis in 0..3 {
                assert!(
                    (min[axis] - boxes[index].0[axis]).abs() < EXTENT_TOL,
                    "{model:?} mesh {index} has an unexpected minimum on axis {axis}: {min:?}"
                );
                assert!(
                    (max[axis] - boxes[index].1[axis]).abs() < EXTENT_TOL,
                    "{model:?} mesh {index} has an unexpected maximum on axis {axis}: {max:?}"
                );
            }
        }
    }
}

#[test]
fn each_mesh_sits_where_the_kinematics_place_its_link() {
    // These geometric relationships associate each mesh with its model: the base hangs below its
    // origin, the upper arm extends beyond the J2-to-J3 distance, the forearm extends beyond the
    // J3-to-J5 distance, and the flange mating face lies on the z = 0 plane of the frame reported
    // by the controller. Recorded measurements alone do not establish these relationships.
    for model in [CrxModel::Crx5iA, CrxModel::Crx10iA] {
        let robot = Crx::from_model(model);
        let meshes = LinkMeshes::load(model).unwrap();

        let (_, base_max) = meshes.base().bounds().unwrap();
        assert!(
            base_max[2] < 0.0,
            "the {model:?} base should lie entirely below its origin"
        );

        let (_, upper_arm_max) = meshes.links()[2].bounds().unwrap();
        assert!(
            upper_arm_max[2] > robot.z1(),
            "the {model:?} upper arm should reach past the J2 to J3 distance"
        );

        let (_, forearm_max) = meshes.links()[4].bounds().unwrap();
        assert!(
            forearm_max[0] > robot.x1(),
            "the {model:?} forearm should reach past the J3 to J5 distance"
        );

        let (_, flange_max) = meshes.flange().bounds().unwrap();
        assert!(
            flange_max[2].abs() < 1e-6,
            "the {model:?} flange face should lie on the z = 0 plane of its frame"
        );
    }
}

#[test]
fn a_model_without_geometry_reports_an_error() {
    for model in UNAVAILABLE {
        assert!(!LinkMeshes::is_available(model));
        assert!(
            LinkMeshes::load(model).is_err(),
            "the {model:?} has no meshes and should not load"
        );
        assert!(model.link_meshes().is_err());
    }
}

#[test]
fn posing_moves_each_link_by_its_own_frame() {
    let mut rng = StdRng::seed_from_u64(0x6d657368);

    for model in [CrxModel::Crx5iA, CrxModel::Crx10iA] {
        let robot = Crx::from_model(model);
        let meshes = LinkMeshes::load(model).unwrap();

        for _ in 0..8 {
            let joints: [f64; 6] = std::array::from_fn(|_| rng.random_range(-180.0..=180.0));
            let frames = robot.fk_all(&joints);
            let posed = meshes.posed(&robot, &joints);

            // The base does not move with the joints.
            assert_eq!(&posed[0], meshes.base());

            for index in 1..LINK_COUNT {
                let expected = meshes.links()[index].transformed(&frames[index - 1]);
                assert_meshes_match(&posed[index], &expected, model, index);
            }

            // The flange follows the pose reported by the controller, as required when a caller
            // draws a tool at the end of the arm.
            let expected = meshes.flange().transformed(&robot.fk(&joints));
            assert_meshes_match(&posed[LINK_COUNT - 1], &expected, model, LINK_COUNT - 1);
        }
    }
}

#[test]
fn the_poses_are_the_identity_followed_by_the_kinematic_frames() {
    let robot = Crx::from_model(CrxModel::Crx10iA);
    let joints = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0];
    let poses = LinkMeshes::poses(&robot, &joints);
    let frames = robot.fk_all(&joints);

    assert_eq!(poses[0], crx_kinematics::Iso3::identity());
    for index in 0..6 {
        assert_eq!(poses[index + 1], frames[index]);
    }
}

fn assert_meshes_match(left: &LinkMesh, right: &LinkMesh, model: CrxModel, index: usize) {
    assert_eq!(left.faces, right.faces);
    assert_eq!(left.vertices.len(), right.vertices.len());
    for (a, b) in left.vertices.iter().zip(right.vertices.iter()) {
        for axis in 0..3 {
            assert!(
                (a[axis] - b[axis]).abs() < POSE_TOL,
                "{model:?} mesh {index} landed in the wrong place: {a:?} against {b:?}"
            );
        }
    }
}
