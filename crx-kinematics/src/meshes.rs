//! Visual geometry for the robot links, embedded in the library.
//!
//! Each supported model has seven triangle meshes: the stationary base and the six moving links.
//! The meshes use the same frames produced by the kinematics, so a robot can be drawn by pairing
//! [`LinkMeshes::links`] with [`LinkMeshes::poses`] for a joint configuration. Coordinates are in
//! millimeters, matching the link lengths in [`Crx`].
//!
//! Meshes exist for the CRX-5iA and the CRX-10iA. The inverse kinematics supports all six models,
//! and geometry is unavailable for the other four. [`LinkMeshes::load`] returns an error for those
//! four models. [`LinkMeshes::is_available`] checks availability without producing an error.
//!
//! The geometry is stored in the tcmesh format, which quantizes vertex positions to the narrowest
//! integer width that still guarantees a round trip within a tolerance chosen when the file was
//! written. This compression stores approximately 46,000 vertices and 87,000 triangles in 460 KB
//! of library data. Each call decodes the requested meshes, which avoids startup work for callers
//! that do not use them.

use crate::{Crx, CrxModel, Iso3, Point3, Result};

/// The number of meshes that make up a robot: the stationary base plus one for each of the six
/// frames returned by [`Crx::fk_all`].
pub const LINK_COUNT: usize = 7;

// The file paths are relative to this source file, so an invalid path causes a compile error. This
// prevents consumers of the published crate from encountering the error at run time. Index 0 is
// the base, and indices 1 through 6 correspond in order to the frames from `Crx::fk_all`.
const CRX5IA: [&[u8]; LINK_COUNT] = [
    include_bytes!("../meshes/crx5ia-0.tcmesh"),
    include_bytes!("../meshes/crx5ia-1.tcmesh"),
    include_bytes!("../meshes/crx5ia-2.tcmesh"),
    include_bytes!("../meshes/crx5ia-3.tcmesh"),
    include_bytes!("../meshes/crx5ia-4.tcmesh"),
    include_bytes!("../meshes/crx5ia-5.tcmesh"),
    include_bytes!("../meshes/crx5ia-6.tcmesh"),
];

const CRX10IA: [&[u8]; LINK_COUNT] = [
    include_bytes!("../meshes/crx10ia-0.tcmesh"),
    include_bytes!("../meshes/crx10ia-1.tcmesh"),
    include_bytes!("../meshes/crx10ia-2.tcmesh"),
    include_bytes!("../meshes/crx10ia-3.tcmesh"),
    include_bytes!("../meshes/crx10ia-4.tcmesh"),
    include_bytes!("../meshes/crx10ia-5.tcmesh"),
    include_bytes!("../meshes/crx10ia-6.tcmesh"),
];

/// Return the embedded files for a model, or `None` when no geometry was authored for it.
fn files_for(model: CrxModel) -> Option<&'static [&'static [u8]; LINK_COUNT]> {
    match model {
        CrxModel::Crx5iA => Some(&CRX5IA),
        CrxModel::Crx10iA => Some(&CRX10IA),
        _ => None,
    }
}

/// A single triangle mesh, holding vertex positions in millimeters and the triangles that index
/// into them.
///
/// Plain arrays let callers pass the two buffers to a renderer, mesh library, or file writer
/// without conversion. They also avoid adding a geometry-type dependency to this crate.
#[derive(Debug, Clone, PartialEq)]
pub struct LinkMesh {
    /// The vertex positions, in millimeters.
    pub vertices: Vec<[f64; 3]>,

    /// The triangles, each holding three indices into `vertices`. Every index is guaranteed to
    /// refer to an existing vertex.
    pub faces: Vec<[u32; 3]>,
}

impl LinkMesh {
    /// Decode one mesh from a tcmesh byte stream.
    ///
    /// # Arguments
    ///
    /// * `bytes`: the contents of a tcmesh file holding exactly one mesh
    ///
    /// returns: `Result<LinkMesh>`, failing if the data is not a valid tcmesh container or holds a
    /// number of meshes other than one
    fn decode(bytes: &[u8]) -> Result<Self> {
        let mesh = tol_compress::mesh::read_one_from(&mut &bytes[..])?;
        Ok(Self {
            vertices: mesh.points,
            faces: mesh.faces,
        })
    }

    /// Return a copy of this mesh with every vertex moved by a transformation.
    ///
    /// # Arguments
    ///
    /// * `pose`: the transformation to apply
    ///
    /// returns: LinkMesh
    pub fn transformed(&self, pose: &Iso3) -> Self {
        let mut result = self.clone();
        result.transform_in_place(pose);
        result
    }

    /// Move every vertex of this mesh by a transformation, leaving the triangles untouched.
    ///
    /// # Arguments
    ///
    /// * `pose`: the transformation to apply
    pub fn transform_in_place(&mut self, pose: &Iso3) {
        for vertex in self.vertices.iter_mut() {
            let moved = pose * Point3::new(vertex[0], vertex[1], vertex[2]);
            *vertex = [moved.x, moved.y, moved.z];
        }
    }

    /// Return the axis-aligned bounding box of the vertices, as a minimum and a maximum corner.
    ///
    /// returns: `Option<([f64; 3], [f64; 3])>`, which is `None` when the mesh has no vertices
    pub fn bounds(&self) -> Option<([f64; 3], [f64; 3])> {
        let first = *self.vertices.first()?;
        let mut min = first;
        let mut max = first;
        for vertex in &self.vertices {
            for axis in 0..3 {
                min[axis] = min[axis].min(vertex[axis]);
                max[axis] = max[axis].max(vertex[axis]);
            }
        }
        Some((min, max))
    }
}

/// The complete set of link geometry for one robot model.
///
/// The seven meshes are ordered so that index 0 is the stationary base and index `i` is the link
/// which moves with frame `i - 1` of [`Crx::fk_all`]. The last one is the flange, whose mating face
/// lies on the z = 0 plane of the frame the controller reports.
#[derive(Debug, Clone, PartialEq)]
pub struct LinkMeshes {
    links: [LinkMesh; LINK_COUNT],
}

impl LinkMeshes {
    /// Report whether geometry is embedded for a model, without attempting to decode it.
    ///
    /// # Arguments
    ///
    /// * `model`: the model to ask about
    ///
    /// returns: bool
    pub fn is_available(model: CrxModel) -> bool {
        files_for(model).is_some()
    }

    /// Decode the seven meshes for a model.
    ///
    /// Each call decodes the meshes without caching and takes a few milliseconds. A caller that
    /// draws repeatedly should retain the result and pose it with [`Self::posed`].
    ///
    /// # Arguments
    ///
    /// * `model`: the model whose geometry to load, which must be the CRX-5iA or the CRX-10iA
    ///
    /// returns: `Result<LinkMeshes>`, failing for a model with no embedded geometry
    pub fn load(model: CrxModel) -> Result<Self> {
        let files = files_for(model).ok_or_else(|| {
            format!(
                "No link meshes are embedded for the {model:?}. Geometry is available for the \
                 Crx5iA and the Crx10iA only."
            )
        })?;

        let mut links = Vec::with_capacity(LINK_COUNT);
        for bytes in files {
            links.push(LinkMesh::decode(bytes)?);
        }

        // The vector was built from a fixed-size array of files, so the conversion cannot fail.
        // A fallible conversion supports the `?` operator because `LinkMesh` is not `Copy` and the
        // array cannot be built element by element.
        let links: [LinkMesh; LINK_COUNT] = links
            .try_into()
            .map_err(|_| "Expected seven embedded meshes")?;

        Ok(Self { links })
    }

    /// Return the seven meshes in their authored frames.
    pub fn links(&self) -> &[LinkMesh; LINK_COUNT] {
        &self.links
    }

    /// Consume this set and return the seven meshes in their authored frames.
    pub fn into_links(self) -> [LinkMesh; LINK_COUNT] {
        self.links
    }

    /// Return the mesh of the stationary base, which never moves.
    pub fn base(&self) -> &LinkMesh {
        &self.links[0]
    }

    /// Return the mesh of the flange, which moves with the pose the controller reports.
    pub fn flange(&self) -> &LinkMesh {
        &self.links[LINK_COUNT - 1]
    }

    /// Return the pose of each of the seven meshes for a joint configuration.
    ///
    /// The stationary base uses the identity transform. The six links use the frames from
    /// [`Crx::fk_all`] in order.
    ///
    /// The robot is a separate argument from the model whose meshes were loaded, which lets the
    /// geometry of one model be posed on the kinematics of another. That is an approximation and
    /// the caller is responsible for deciding it is a reasonable one.
    ///
    /// # Arguments
    ///
    /// * `crx`: the robot whose kinematics produce the frames
    /// * `joints`: the six joint angles in degrees, as they appear in the robot controller
    ///
    /// returns: [Isometry<f64, Unit<Quaternion<f64>>, 3>; 7]
    pub fn poses(crx: &Crx, joints: &[f64; 6]) -> [Iso3; LINK_COUNT] {
        let frames = crx.fk_all(joints);
        [
            Iso3::identity(),
            frames[0],
            frames[1],
            frames[2],
            frames[3],
            frames[4],
            frames[5],
        ]
    }

    /// Return copies of the seven meshes, each moved to its pose for a joint configuration.
    ///
    /// # Arguments
    ///
    /// * `crx`: the robot whose kinematics produce the frames
    /// * `joints`: the six joint angles in degrees, as they appear in the robot controller
    ///
    /// returns: [LinkMesh; 7]
    pub fn posed(&self, crx: &Crx, joints: &[f64; 6]) -> [LinkMesh; LINK_COUNT] {
        let poses = Self::poses(crx, joints);
        std::array::from_fn(|i| self.links[i].transformed(&poses[i]))
    }
}

impl CrxModel {
    /// Decode the seven link meshes for this model, as [`LinkMeshes::load`] does.
    ///
    /// returns: `Result<LinkMeshes>`, failing for a model with no embedded geometry
    pub fn link_meshes(self) -> Result<LinkMeshes> {
        LinkMeshes::load(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::na::Translation3;

    #[test]
    fn the_identity_leaves_a_mesh_where_it_was() {
        let mesh = LinkMeshes::load(CrxModel::Crx5iA).unwrap().links[3].clone();
        assert_eq!(mesh.transformed(&Iso3::identity()), mesh);
    }

    #[test]
    fn a_translation_shifts_every_vertex() {
        let mesh = LinkMeshes::load(CrxModel::Crx5iA).unwrap().links[3].clone();
        let shift = Iso3::from_parts(
            Translation3::new(1.0, 2.0, 3.0),
            crate::na::UnitQuaternion::identity(),
        );
        let moved = mesh.transformed(&shift);

        assert_eq!(moved.faces, mesh.faces);
        for (before, after) in mesh.vertices.iter().zip(moved.vertices.iter()) {
            assert_eq!(
                *after,
                [before[0] + 1.0, before[1] + 2.0, before[2] + 3.0],
                "a vertex did not move by the translation"
            );
        }
    }
}
