# crx-kinematics

Forward and inverse kinematics for the [FANUC CRX](https://www.fanuc.co.jp/en/product/robot/index.html)
family of collaborative robots. These are Python bindings for the Rust crate
[crx-kinematics](https://crates.io/crates/crx-kinematics), and require only `numpy`.

The inverse kinematics is complete and direct. For a reachable flange pose it returns *every* joint
configuration that reaches it, up to the sixteen this architecture allows, and it does so without
seeding, sampling, or iterating toward a single answer. The method is derived in full in
[LINEAR_METHOD.md](https://github.com/mattj23/crx-kinematics/blob/main/docs/LINEAR_METHOD.md).

```bash
pip install crx-kinematics
```

```python
import numpy as np
from crx_kinematics import Crx

robot = Crx.crx10ia()

# Forward: joint angles in controller degrees to a 4x4 flange pose.
target = robot.fk([10, -80, 10, 20, -20, 45])

# Inverse: every configuration that reaches the pose, as an (n, 6) array.
solutions = robot.ik(target)

# Or just the one nearest to where the arm already is.
best = robot.ik_closest(target, [0, 0, 0, 0, 0, 0])
print(np.round(best.joints, 6), best.residual, best.kind)
```

Joint angles are in degrees as the robot controller reports them, including the FANUC convention
where the J3 value is measured against J2 rather than against the world. All six kinematically
distinct models are available as `Crx.crx3ia()` through `Crx.crx30ia()`, and `Crx.from_params`
builds a robot from the four link dimensions directly.

Poses cross the boundary as 4x4 NumPy arrays. A target may also be any object with an `as_numpy()`
method returning one, which is how an [engeom](https://pypi.org/project/engeom/) `Iso3` can be
passed straight through without either package depending on the other:

```python
from engeom.geom3 import Iso3

solutions = robot.ik(Iso3.from_xyzwpr(600, 0, 700, 180, 0, 0))
```

`ik` returns joint angles only. `ik_detailed` returns the same solutions with the residual and the
kind of configuration attached, and `fk_all` returns the pose of every frame in the chain.

Several configurations are mathematically degenerate, including the wrist center landing on the J1
axis, which an operator can enter directly and which defeats other implementations of this
architecture. They are enumerated with their treatment in
[the derivation](https://github.com/mattj23/crx-kinematics/blob/main/docs/LINEAR_METHOD.md#degenerate-configurations).

## Link Meshes

The package carries visual geometry for the CRX-5iA and the CRX-10iA: seven triangle meshes each,
covering the stationary base and the six moving links. Vertices are returned as an `(n, 3)` array
of `float64`, and faces are returned as an `(m, 3)` array of `uint32`. Vertex coordinates are in
millimeters.

```python
from crx_kinematics import Crx, CrxModel, LinkMeshes

robot = Crx.crx10ia()
meshes = LinkMeshes.load(CrxModel.Crx10iA)

for link in meshes.posed(robot, [10, -80, 10, 20, -20, 45]):
    print(link.vertices.shape, link.faces.shape)
```

Index 0 contains the stationary base. Index `i` contains the link that moves with frame `i - 1` of
`fk_all`. The final mesh is the flange, whose mating face lies on the z = 0 plane of the pose
reported by `fk`.

An `engeom` `Mesh3` accepts these array shapes and data types, so drawing the robot requires no
conversion:

```python
from engeom.geom3 import Mesh3

drawable = [Mesh3(link.vertices, link.faces) for link in meshes.posed(robot, joints)]
```

`LinkMeshes.load` raises `ValueError` for the other four models. `LinkMeshes.is_available` checks
whether geometry is present without raising an exception.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option.
