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
use pyo3::exceptions::PyValueError;
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

/// Forward and inverse kinematics for the FANUC CRX series of collaborative robots.
#[pymodule]
fn crx_kinematics(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Crx>()?;
    m.add_class::<IkSolution>()?;
    m.add_class::<SolutionKind>()?;
    Ok(())
}
