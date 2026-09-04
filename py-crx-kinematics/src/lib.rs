//! Python bindings for `crx-kinematics`.
//!
//! This crate is a thin wrapper and contains no kinematics calculations. Every calculation runs in
//! the `crx-kinematics` crate, so the Python and Rust interfaces use the same implementation. This
//! crate converts between NumPy arrays and the library's types. It accepts a pose as either a 4x4
//! array or any object that exposes `as_numpy()`, allowing direct use of an `engeom.geom3.Iso3`
//! without adding an engeom dependency.

use crx::{Iso3, row_slice_to_iso};
use numpy::ndarray::{Array1, Array2, Array3};
use numpy::{
    AllowTypeChange, IntoPyArray, PyArray1, PyArray2, PyArray3, PyArrayLike1, PyArrayLike2,
    PyArrayMethods, PyUntypedArrayMethods,
};
use pyo3::exceptions::{PyIndexError, PyValueError};
use pyo3::prelude::*;

/// Read a pose from either a 4x4 array or an object that can produce one.
///
/// An `engeom.geom3.Iso3` is accepted through its `as_numpy()` method, which is the only thing
/// this package assumes about engeom.
fn iso_from_any(obj: &Bound<'_, PyAny>) -> PyResult<Iso3> {
    let array = if obj.hasattr("as_numpy")? {
        obj.call_method0("as_numpy")?
    } else {
        obj.clone()
    };

    let array: PyArrayLike2<'_, f64, AllowTypeChange> = array.extract().map_err(|_| {
        PyValueError::new_err("Expected a 4x4 array, or an object with an as_numpy() method")
    })?;

    let shape = array.shape();
    if shape != [4, 4] {
        return Err(PyValueError::new_err(format!(
            "Expected a 4x4 transformation matrix, got shape {shape:?}"
        )));
    }

    let readonly = array.readonly();
    let view = readonly.as_array();
    let values: Vec<f64> = view.iter().copied().collect();

    row_slice_to_iso(&values).map_err(|e| PyValueError::new_err(e.to_string()))
}

/// Read six joint angles from any sequence of numbers.
fn joints_from_any(joints: PyArrayLike1<'_, f64, AllowTypeChange>) -> PyResult<[f64; 6]> {
    let readonly = joints.readonly();
    let view = readonly.as_array();
    if view.len() != 6 {
        return Err(PyValueError::new_err(format!(
            "Expected 6 joint angles, got {}",
            view.len()
        )));
    }

    let mut out = [0.0; 6];
    for (slot, value) in out.iter_mut().zip(view.iter()) {
        *slot = *value;
    }
    Ok(out)
}

/// Write a pose into a 4x4 block of an array, starting at the given row of the leading axis.
fn iso_into(target: &mut [f64], iso: &Iso3) {
    let m = iso.to_matrix();
    for row in 0..4 {
        for col in 0..4 {
            target[row * 4 + col] = m[(row, col)];
        }
    }
}

/// The configuration category of a solution.
#[pyclass(eq, eq_int, skip_from_py_object, module = "crx_kinematics")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolutionKind {
    /// The solution is isolated and the arm is in an ordinary configuration.
    Regular,

    /// The wrist center is on the J1 axis. The solution is still isolated, but the constraint the
    /// solver roots has a double root there.
    AxisDegenerate,

    /// A joint has no effect on the pose, so this solution is one representative of a continuum.
    SingularFamily,
}

impl From<crx::ik::SolutionKind> for SolutionKind {
    fn from(kind: crx::ik::SolutionKind) -> Self {
        match kind {
            crx::ik::SolutionKind::Regular => SolutionKind::Regular,
            crx::ik::SolutionKind::AxisDegenerate => SolutionKind::AxisDegenerate,
            crx::ik::SolutionKind::SingularFamily => SolutionKind::SingularFamily,
        }
    }
}

/// One inverse kinematics solution and its diagnostics.
#[pyclass(skip_from_py_object, module = "crx_kinematics")]
#[derive(Clone)]
pub struct IkSolution {
    joints: [f64; 6],

    /// Largest absolute element difference between this solution's pose and the target.
    #[pyo3(get)]
    residual: f64,

    /// The configuration category of the solution.
    #[pyo3(get)]
    kind: SolutionKind,

    /// The angle on the wrist center's circle that produced the solution, kept for diagnostics.
    #[pyo3(get)]
    theta: f64,
}

#[pymethods]
impl IkSolution {
    /// The six joint angles in FANUC controller degrees.
    #[getter]
    fn joints<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        Array1::from_iter(self.joints.iter().copied()).into_pyarray(py)
    }

    fn __repr__(&self) -> String {
        let j = self
            .joints
            .iter()
            .map(|v| format!("{v:.4}"))
            .collect::<Vec<_>>()
            .join(", ");
        format!(
            "IkSolution(joints=[{j}], residual={:.3e}, kind={:?})",
            self.residual, self.kind
        )
    }
}

/// One of the models in the FANUC CRX series.
///
/// `LinkMeshes.load` uses the model to select geometry. Two models can share the same four link
/// dimensions, so those dimensions alone do not identify the model represented by a `Crx`.
// This enum is accepted as an argument, so it opts in to the `FromPyObject` implementation derived
// by pyo3. Pyo3 0.28 requires the explicit declaration, and a later release will no longer derive
// the implementation automatically. `SolutionKind` is not accepted as an argument and does not
// need this implementation.
#[pyclass(eq, eq_int, from_py_object, module = "crx_kinematics")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrxModel {
    /// The CRX-3iA.
    Crx3iA,

    /// The CRX-5iA.
    Crx5iA,

    /// The CRX-10iA.
    Crx10iA,

    /// The CRX-10iA/L.
    Crx10iAL,

    /// The CRX-20iA/L.
    Crx20iAL,

    /// The CRX-30iA.
    Crx30iA,
}

impl From<CrxModel> for crx::CrxModel {
    fn from(model: CrxModel) -> Self {
        match model {
            CrxModel::Crx3iA => crx::CrxModel::Crx3iA,
            CrxModel::Crx5iA => crx::CrxModel::Crx5iA,
            CrxModel::Crx10iA => crx::CrxModel::Crx10iA,
            CrxModel::Crx10iAL => crx::CrxModel::Crx10iAL,
            CrxModel::Crx20iAL => crx::CrxModel::Crx20iAL,
            CrxModel::Crx30iA => crx::CrxModel::Crx30iA,
        }
    }
}

/// A FANUC CRX robot defined by the four link dimensions that distinguish the models.
#[pyclass(module = "crx_kinematics")]
pub struct Crx {
    inner: crx::Crx,
}

#[pymethods]
impl Crx {
    /// Build a robot from its four link dimensions, in millimeters.
    #[staticmethod]
    fn from_params(z1: f64, x1: f64, x2: f64, y1: f64) -> Self {
        Self {
            inner: crx::Crx::from_params(z1, x1, x2, y1),
        }
    }

    /// Build a robot with the link dimensions of a FANUC CRX model.
    #[staticmethod]
    fn from_model(model: CrxModel) -> Self {
        Self {
            inner: crx::Crx::from_model(model.into()),
        }
    }

    /// The CRX-3iA.
    #[staticmethod]
    fn crx3ia() -> Self {
        Self {
            inner: crx::Crx::new_3ia(),
        }
    }

    /// The CRX-5iA.
    #[staticmethod]
    fn crx5ia() -> Self {
        Self {
            inner: crx::Crx::new_5ia(),
        }
    }

    /// The CRX-10iA.
    #[staticmethod]
    fn crx10ia() -> Self {
        Self {
            inner: crx::Crx::new_10ia(),
        }
    }

    /// The CRX-10iA/L.
    #[staticmethod]
    fn crx10ial() -> Self {
        Self {
            inner: crx::Crx::new_10ia_l(),
        }
    }

    /// The CRX-20iA/L.
    #[staticmethod]
    fn crx20ial() -> Self {
        Self {
            inner: crx::Crx::new_20ia_l(),
        }
    }

    /// The CRX-30iA.
    #[staticmethod]
    fn crx30ia() -> Self {
        Self {
            inner: crx::Crx::new_30ia(),
        }
    }

    /// The height of the J3 origin above the J2 origin, in millimeters.
    #[getter]
    fn z1(&self) -> f64 {
        self.inner.z1()
    }

    /// The length of the forearm, from the J3 origin to the J5 origin, in millimeters.
    #[getter]
    fn x1(&self) -> f64 {
        self.inner.x1()
    }

    /// The distance from the J5 origin to the flange, in millimeters.
    #[getter]
    fn x2(&self) -> f64 {
        self.inner.x2()
    }

    /// The lateral offset of the J5 origin from the forearm axis, in millimeters.
    #[getter]
    fn y1(&self) -> f64 {
        self.inner.y1()
    }

    /// Compute the pose of the flange for a set of joint angles.
    ///
    /// The joints are in degrees as they appear in the robot controller, and the returned 4x4
    /// matrix matches the controller in both position and orientation.
    fn fk<'py>(
        &self,
        py: Python<'py>,
        joints: PyArrayLike1<'py, f64, AllowTypeChange>,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let joints = joints_from_any(joints)?;
        let mut array = Array2::zeros((4, 4));
        iso_into(
            array.as_slice_mut().expect("array is contiguous"),
            &self.inner.fk(&joints),
        );
        Ok(array.into_pyarray(py))
    }

    /// Compute the pose of every frame in the kinematic chain, as a (6, 4, 4) array.
    ///
    /// The last of the six is the flange and matches `fk`. The others are at the kinematic link
    /// origins, which have no corresponding values in the controller.
    fn fk_all<'py>(
        &self,
        py: Python<'py>,
        joints: PyArrayLike1<'py, f64, AllowTypeChange>,
    ) -> PyResult<Bound<'py, PyArray3<f64>>> {
        let joints = joints_from_any(joints)?;
        let frames = self.inner.fk_all(&joints);

        let mut array = Array3::zeros((6, 4, 4));
        let slice = array.as_slice_mut().expect("array is contiguous");
        for (i, frame) in frames.iter().enumerate() {
            iso_into(&mut slice[i * 16..(i + 1) * 16], frame);
        }
        Ok(array.into_pyarray(py))
    }

    /// Every joint configuration which puts the flange at `target`, as an (n, 6) array.
    ///
    /// The target may be a 4x4 array or any object with an `as_numpy()` method returning one. An
    /// unreachable target gives an array with no rows. Use `ik_detailed` when the residual or the
    /// kind of each solution matters.
    fn ik<'py>(
        &self,
        py: Python<'py>,
        target: &Bound<'py, PyAny>,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let target = iso_from_any(target)?;
        let solutions = self.inner.ik(&target);

        let mut array = Array2::zeros((solutions.len(), 6));
        for (i, solution) in solutions.iter().enumerate() {
            for (j, value) in solution.joints.iter().enumerate() {
                array[[i, j]] = *value;
            }
        }
        Ok(array.into_pyarray(py))
    }

    /// The same solutions as `ik`, each carrying its residual and its kind.
    fn ik_detailed(&self, target: &Bound<'_, PyAny>) -> PyResult<Vec<IkSolution>> {
        let target = iso_from_any(target)?;
        Ok(self
            .inner
            .ik(&target)
            .into_iter()
            .map(|s| IkSolution {
                joints: s.joints,
                residual: s.residual,
                kind: s.kind.into(),
                theta: s.theta,
            })
            .collect())
    }

    /// The solution whose joints are nearest to `reference`, or None if the target is unreachable.
    ///
    /// A caller following a path can pass the robot's current joint vector as `reference` to reach
    /// the next pose with the least joint movement, which keeps the arm from reconfiguring during
    /// the move.
    fn ik_closest<'py>(
        &self,
        target: &Bound<'py, PyAny>,
        reference: PyArrayLike1<'py, f64, AllowTypeChange>,
    ) -> PyResult<Option<IkSolution>> {
        let target = iso_from_any(target)?;
        let reference = joints_from_any(reference)?;
        Ok(self
            .inner
            .ik_closest(&target, &reference)
            .map(|s| IkSolution {
                joints: s.joints,
                residual: s.residual,
                kind: s.kind.into(),
                theta: s.theta,
            }))
    }
}

/// The visual geometry of one robot link, as arrays a mesh library can consume directly.
#[pyclass(skip_from_py_object, module = "crx_kinematics")]
#[derive(Clone)]
pub struct LinkMesh {
    inner: crx::LinkMesh,
}

#[pymethods]
impl LinkMesh {
    /// The vertex positions in millimeters, as an (n, 3) array of float64.
    ///
    /// The array is a fresh copy on every access, so writing into it does not change the mesh.
    #[getter]
    fn vertices<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let rows = self.inner.vertices.len();
        let flat: Vec<f64> = self.inner.vertices.iter().flatten().copied().collect();
        let array = Array2::from_shape_vec((rows, 3), flat)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(array.into_pyarray(py))
    }

    /// The triangles, as an (m, 3) array of uint32 indexing into `vertices`.
    ///
    /// The array is a fresh copy on every access, so writing into it does not change the mesh.
    #[getter]
    fn faces<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<u32>>> {
        let rows = self.inner.faces.len();
        let flat: Vec<u32> = self.inner.faces.iter().flatten().copied().collect();
        let array = Array2::from_shape_vec((rows, 3), flat)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(array.into_pyarray(py))
    }

    /// The number of vertices in the mesh.
    #[getter]
    fn vertex_count(&self) -> usize {
        self.inner.vertices.len()
    }

    /// The number of triangles in the mesh.
    #[getter]
    fn face_count(&self) -> usize {
        self.inner.faces.len()
    }

    /// Return a copy of this mesh with every vertex moved by a transformation.
    ///
    /// The transformation may be a 4x4 array or any object with an `as_numpy()` method returning
    /// one, which is the same input `ik` accepts for a target.
    fn transformed(&self, transform: &Bound<'_, PyAny>) -> PyResult<Self> {
        let pose = iso_from_any(transform)?;
        Ok(Self {
            inner: self.inner.transformed(&pose),
        })
    }

    fn __repr__(&self) -> String {
        format!(
            "LinkMesh(vertices={}, faces={})",
            self.inner.vertices.len(),
            self.inner.faces.len()
        )
    }
}

/// The seven meshes that draw one robot: the stationary base and the six moving links.
///
/// Index 0 is the base and index `i` is the link that moves with frame `i - 1` of `fk_all`. The
/// last one is the flange, whose mating face lies on the z = 0 plane of the pose `fk` reports.
/// Coordinates are in millimeters.
///
/// Geometry is embedded for the CRX-5iA and the CRX-10iA only.
#[pyclass(skip_from_py_object, module = "crx_kinematics")]
pub struct LinkMeshes {
    inner: crx::LinkMeshes,
}

#[pymethods]
impl LinkMeshes {
    /// Decode the seven meshes for a model.
    ///
    /// :param model: the model whose geometry to load, which must be `CrxModel.Crx5iA` or
    ///     `CrxModel.Crx10iA`
    /// :raises ValueError: if no geometry is embedded for the model
    #[staticmethod]
    fn load(model: CrxModel) -> PyResult<Self> {
        crx::LinkMeshes::load(model.into())
            .map(|inner| Self { inner })
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// Report whether geometry is embedded for a model without raising an exception.
    #[staticmethod]
    fn is_available(model: CrxModel) -> bool {
        crx::LinkMeshes::is_available(model.into())
    }

    /// The seven meshes in their authored frames, as a list.
    #[getter]
    fn links(&self) -> Vec<LinkMesh> {
        self.inner
            .links()
            .iter()
            .map(|inner| LinkMesh {
                inner: inner.clone(),
            })
            .collect()
    }

    fn __len__(&self) -> usize {
        crx::meshes::LINK_COUNT
    }

    fn __getitem__(&self, index: usize) -> PyResult<LinkMesh> {
        self.inner
            .links()
            .get(index)
            .map(|inner| LinkMesh {
                inner: inner.clone(),
            })
            .ok_or_else(|| PyIndexError::new_err("index out of range"))
    }

    /// The pose of each of the seven meshes for a joint configuration, as a (7, 4, 4) array.
    ///
    /// The first is the identity, because the base does not move, and the remaining six are the
    /// frames from `fk_all` in order.
    ///
    /// :param robot: the robot whose kinematics produce the frames
    /// :param joints: six joint angles in degrees, as they appear in the robot controller
    fn poses<'py>(
        &self,
        py: Python<'py>,
        robot: PyRef<'_, Crx>,
        joints: PyArrayLike1<'py, f64, AllowTypeChange>,
    ) -> PyResult<Bound<'py, PyArray3<f64>>> {
        let joints = joints_from_any(joints)?;
        let poses = crx::LinkMeshes::poses(&robot.inner, &joints);

        let mut array = Array3::zeros((crx::meshes::LINK_COUNT, 4, 4));
        let slice = array.as_slice_mut().expect("array is contiguous");
        for (i, pose) in poses.iter().enumerate() {
            iso_into(&mut slice[i * 16..(i + 1) * 16], pose);
        }
        Ok(array.into_pyarray(py))
    }

    /// Copies of the seven meshes, each moved to its pose for a joint configuration.
    ///
    /// The robot is a separate argument from the model whose meshes were loaded, so geometry from
    /// one model can be posed on the kinematics of another. That is an approximation, and the
    /// caller decides whether it is a reasonable one.
    ///
    /// :param robot: the robot whose kinematics produce the frames
    /// :param joints: six joint angles in degrees, as they appear in the robot controller
    fn posed(
        &self,
        robot: PyRef<'_, Crx>,
        joints: PyArrayLike1<'_, f64, AllowTypeChange>,
    ) -> PyResult<Vec<LinkMesh>> {
        let joints = joints_from_any(joints)?;
        Ok(self
            .inner
            .posed(&robot.inner, &joints)
            .into_iter()
            .map(|inner| LinkMesh { inner })
            .collect())
    }

    fn __repr__(&self) -> String {
        format!("LinkMeshes(links={})", crx::meshes::LINK_COUNT)
    }
}

/// Forward and inverse kinematics for the FANUC CRX series of collaborative robots.
#[pymodule]
fn crx_kinematics(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Crx>()?;
    m.add_class::<CrxModel>()?;
    m.add_class::<IkSolution>()?;
    m.add_class::<LinkMesh>()?;
    m.add_class::<LinkMeshes>()?;
    m.add_class::<SolutionKind>()?;
    Ok(())
}
