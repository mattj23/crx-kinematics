# FANUC CRX Series Kinematics

Forward and inverse kinematics for the FANUC CRX family of collaborative robots, as a Rust crate
with a Python binary extension module built on it.

> [!WARNING]
> This library only solves the mathematical kinematics problem, it does not check that the robot
> can reach poses without self-collision!

---

This implementation was intended to be fast.  On an AMD Ryzen 7 PRO 8840U (a mid-level mobile 
processor from 2024) the `criterion` benchmark takes roughly 13.7 µs per pose for the inverse 
kinematics and 0.11 µs per pose for the forward kinematics. 

The inverse kinematics method returns all joint configurations for a reachable flange pose, up
to the sixteen configurations this architecture allows. It requires no seeding, sampling, or
iteration toward a single answer. The method reduces the problem to the roots of one scalar
equation whose degree is known in advance, so the count of candidates is fixed before any
arithmetic is done and none can be missed. It is derived in full in
[docs/LINEAR_METHOD.md](https://github.com/mattj23/crx-kinematics/blob/main/docs/LINEAR_METHOD.md).

The work grew out of an attempt to implement the geometric approach of Abbes and Poisson[^abbes],
described under [Prior Work](#prior-work-and-its-problems) below. The derivation here starts from
their observations about the CRX layout and then departs from their method entirely, so it should
not be used to benchmark or evaluate their work.

## Using It from Rust

The crate depends only on [`nalgebra`](https://nalgebra.org/), and poses are plain
`nalgebra::Isometry3<f64>` values, so no conversion is needed to use it alongside another library
built on the same types.

```bash
cargo add crx-kinematics
```

```toml
[dependencies]
crx-kinematics = "0.1"
```

```rust
use crx_kinematics::Crx;

let robot = Crx::new_10ia();

// Forward: joint angles in controller degrees to a flange pose.
let target = robot.fk(&[10.0, -80.0, 10.0, 20.0, -20.0, 45.0]);

// Inverse: every configuration that reaches the pose.
let solutions = robot.ik(&target);

// Select the solution nearest the arm's current position.
let current = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
if let Some(best) = robot.ik_closest(&target, &current) {
    println!("{:?}, residual {:.1e}", best.joints, best.residual);
}
```

Joint angles are in degrees as the robot controller reports them, including the FANUC convention
where the J3 value is measured against J2 rather than against the world. `fk_all` returns the pose
of every frame in the chain, for drawing the arm or attaching geometry to a link.

Each solution carries the pose error it achieved and a `SolutionKind` marking whether it is an
isolated solution or one representative of a continuum. `Crx::from_params` builds a robot from the
four link dimensions directly, for a variant not in the table below.

## Using It from Python

The Python package is a thin wrapper around the Rust implementation, so both languages use the same
kinematics calculations. It requires only `numpy`.

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

# Select the solution nearest the arm's current position.
best = robot.ik_closest(target, [0, 0, 0, 0, 0, 0])
print(np.round(best.joints, 6), best.residual, best.kind)
```

Poses cross the boundary as 4x4 NumPy arrays. A target may also be any object with an `as_numpy()`
method returning one. This protocol accepts an [`engeom`](https://github.com/mattj23/engeom) `Iso3`
without requiring either package to depend on the other:

```python
from engeom.geom3 import Iso3

solutions = robot.ik(Iso3.from_xyzwpr(600, 0, 700, 180, 0, 0))
```

`ik` returns joint angles only. `ik_detailed` returns the same solutions with the residual and kind
attached.

## The Fanuc CRX Kinematic Layout

The FANUC CRX series is a relatively new line of collaborative industrial robot arm with a traditional 6-axis serial layout but forgoing the spherical wrist.
The first model, the CRX-10iA was introduced in 2019, and currently (2026) there are six total kinematically unique models, plus a number of food-safe variants and a paint version of the 10iA/L.
All robots in the series have the same kinematic layout, varying only in link lengths.

Because of the non-spherical wrist and the parallel J2/J3 axes, there aren't any pure analytical general solutions to the CRX inverse kinematics.
With FANUC robots, like most industrial arms, there are multiple joint configurations that can achieve most end effector poses and they must be accounted for during linear motion, meaning that most naive solver-based IK approaches require extra complexity. 

Consider the following diagram of a CRX-10iA robot, showing the general links and joints. The robot base is fixed, and the robot's internal kinematics set the world origin as the intersection of the $\overrightarrow{J_1}$ and $\overrightarrow{J_2}$ axes, which yields some mathematical niceties.

![CRX-10iA](https://raw.githubusercontent.com/mattj23/crx-kinematics/main/docs/images/links_and_joints.png)

The robot's kinematics can be represented with four unique parameters, which are the distances between the robot's kinematic link origins.  The origins are points in the robot's world coordinate system and are labeled $O_1$ through $O_6$. Point $O_1$ is always at $(0, 0, 0)$ regardless of what the joints do, and $O_6$ is at the intersection of $\overrightarrow{J_6}$ and the robot flange. The other origins are located at the intersection of the different axes.

![CRX-10iA](https://raw.githubusercontent.com/mattj23/crx-kinematics/main/docs/images/parameters.png)

| Kinematic Parameter          | Model Parameter |
|------------------------------|-----------------|
| Distance from $O_1$ to $O_3$ | `z1`            |
| Distance from $O_3$ to $O_4$ | `x1`            |
| Distance from $O_4$ to $O_5$ | `y1`            |
| Distance from $O_5$ to $O_6$ | `x2`            |

Because the entire CRX family has the same kinematic layout, the difference in modeling them is only the variation in the four parameters. The following table contains the parameters for the full family.  Be aware that the CR-35iA is not a member of the CRX family, but rather has spherical wrist kinematics like non-collaborative robots in Fanuc's product catalogue. 

| Robot      | J2 -> J3 (`z1`) | J3 -> J5 (`x1`) | J5 -> Flange (`x2`) | J1 -> J6 (`y1`) |
|------------|-----------------|-----------------|---------------------|-----------------|
| CRX-3iA    | 280             | 280             | 123                 | 111             |
| CRX-5iA    | 410             | 430             | 145                 | 130             |
| CRX-10iA   | 540             | 540             | 160                 | 150             |
| CRX-10iA/L | 710             | 540             | 160                 | 150             |
| CRX-20iA/L | 710             | 540             | 160                 | 150             |
| CRX-30iA   | 950             | 750             | 180                 | 185             |

> [!NOTE]
> The CRX-10iA/L and the CRX-20iA/L have the same kinematic parameters; that is not a mistake.

If any of the above is unfamiliar, or if the relationship between what a FANUC controller calls a
frame, a position, or an offset and the mathematics underneath it is not obvious, there is a primer
in [docs/BACKGROUND.md](https://github.com/mattj23/crx-kinematics/blob/main/docs/BACKGROUND.md).

---

## How the Method Works

The summary below is based on the complete derivation and numerical treatment of every degenerate
case in [docs/LINEAR_METHOD.md](https://github.com/mattj23/crx-kinematics/blob/main/docs/LINEAR_METHOD.md).

**The pose fixes three of the six origins.** $O_6$ is the flange itself, $O_5$ is a fixed offset
back along the flange axis, and $O_1$ is the world origin that no joint moves. Only $O_3$ and
$O_4$ remain unknown, and once they are located every joint angle can be read off the chain.

**One parameter represents the remaining problem.** $O_4$ must lie on a circle of radius `y1` around
$O_5$ in the flange plane, so a single angle $\theta$ on that circle describes it.

**With $\theta$ fixed, three of the five constraints are linear in $O_3$.** Cramer's rule solves
them, and imposing the one remaining constraint leaves a single scalar equation $f(\theta) = 0$.
There is no circle-to-circle matching, no tracking of branch pairs, and no search.

**The degree of that equation is known.** $f$ is a trigonometric polynomial of degree exactly four,
which is verified numerically for every model in the family. It therefore has at most eight roots
in a turn, and each root gives an arm posture reachable two ways, because swinging the base half a
turn and mirroring the shoulder, elbow, and wrist leaves the flange where it was. Eight roots and
two postures each give the literature's bound of sixteen.

**The roots come from an ordinary polynomial solve.** Because the degree is known, sixteen samples
of $f$ recover its Fourier coefficients exactly. A half-angle substitution turns it into a real
polynomial of degree eight. An Ehrlich-Aberth iteration finds all eight complex roots simultaneously.
If the iteration does not converge, the solver uses the eigenvalues of the companion matrix. The
method uses neither bracketing nor root sampling, so it cannot lose a root between samples.

**Every candidate is finished in joint space.** A few Gauss-Newton steps against the target pose
take each solution to the last digits of double precision. The resulting pose error provides the
acceptance criterion and permits broad candidate generation. Rejecting an invalid candidate costs a
few Gauss-Newton steps, while an omitted candidate cannot be recovered later.


## Accuracy and Speed

Solutions are polished until they stop improving, which puts them at the floor of double precision:
across the round-trip suites the worst pose error of any returned solution is on the order of
$10^{-12}$ mm. Acceptance is at $10^{-8}$ mm, far above what a valid solution achieves and far
below what an invalid candidate can reach.

The Rust solver takes roughly 13.7 µs per pose to return all solutions, measured with `criterion`
over a corpus of 512 random poses on an AMD Ryzen 7 PRO 8840U (single-thread PassMark 3636); run
`cargo bench` for the figure on your own machine. About a third of that is finding the roots of the
constraint, and the rest is spread over locating $O_3$, reading off the joints, and the joint
polish, which uses an analytic Jacobian rather than a numerical one. Picking the solution nearest a
reference configuration adds a few percent. The forward kinematics costs about 0.11 µs per pose. 

## What's in This Repository

| Path | What it is |
|---|---|
| `crx-kinematics/` | The Rust library. Depends only on `nalgebra`. |
| `py-crx-kinematics/` | The PyO3 binding and the Python package. |
| `docs/LINEAR_METHOD.md` | The complete derivation and definitive description of the method. |
| `docs/ALTERNATE.md` | An earlier, superseded derivation, kept for reference. |
| `docs/BACKGROUND.md` | A primer on FANUC controller concepts and the isometries beneath them, for readers new to either. |
| `derivation/` | A development-only, unpublished Python project containing the NumPy reference solver, its tests, and the tooling that draws the robot. |

The Rust test suites are pose stress tests and should be run in release: `cargo test -r`. They
include a fixture comparison that checks the Rust roots against roots exported from the NumPy
reference, so each implementation is checked against the other.

To build from a clone rather than installing the published packages, `cargo build -r` covers the
Rust side, and the Python package is built and installed into the active virtual environment with
[maturin](https://www.maturin.rs/):

```bash
pip install maturin
maturin develop -r -m py-crx-kinematics/Cargo.toml
```

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option. Unless you explicitly state otherwise, any contribution
intentionally submitted for inclusion in this work, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.

## Prior Work and Its Problems

The Abbes and Poisson paper provides a geometry-based framework for calculating joint positions for
any robot with the CRX layout from a desired flange pose. It can calculate up to 16 redundant joint
configurations that produce the pose.

Although the Abbes and Poisson approach is intuitive, I was unable to make a direct implementation
work. I found two libraries that described themselves as implementations of this approach. As of
April 2026, both had functional problems:

- [Fanuc_RMI_API](https://github.com/vertec-io/Fanuc_RMI_API) is written in Rust and contains a
  `sim/` module. I tried this library first, but `cargo test` failed two of its unit tests: one for a
  solution with multiple results and one for round-trip forward and inverse kinematics.
- [crx_kinematics](https://github.com/danielcranston/crx_kinematics) is a ROS 2 package written in
  C++. I extracted `robot.cpp` and `robot.hpp` to test them separately from the rest of the package.
  The extracted code worked for simple cases but failed a round trip for a
  [known CRX-10iA problem case](https://github.com/mattj23/crx-kinematics/issues/1).

I also tried [ik-geo-rust](https://github.com/Verdant-Evolution/ik-geo-rust). The Rust library does
not appear to implement the CRX family's three-parallel-axis configuration, according to the author
of the paper on which the library is based.

[^abbes]: Abbes, Manel, and Gérard Poisson. "Geometric Approach for Inverse Kinematics of the FANUC CRX Collaborative Robot." *Robotics* 13, no. 6 (June 14, 2024): 91. The article is open access and is published at https://www.mdpi.com/2218-6581/13/6/91.

## Known Issues

### Missed configurations near the J1 Axis

The solver treats $O_4$ as on the J1 axis when it is within `AXIS_TOL`, a millionth of $z_1$, 
and it uses the separate construction described above. Just outside that band, from about 
$10^{-5}$ to $10^{-4}$ degrees of J3 away from an exact crossing, corresponding to a few micrometers 
of $O_4$, the ordinary method becomes unreliable. 

Up to four roots of the constraint cluster together in this region and can be located only to about
$10^{-4}$ radians. The residual used to select between the two $O_3$ branches varies over a range
narrower than that accuracy, so rounding determines whether branch polishing reaches the correct
root. 

Measurements over five thousand poses per model at each offset show that the solver fails to
return the generating configuration for one to two percent of all poses and less than one percent
of poses that avoid the unrecoverable configurations. Every returned solution still reaches the
pose. Widening `AXIS_TOL` does not resolve these misses because the on-axis construction is exact
only on the axis, and joint polishing cannot reliably absorb an offset of that size. The
`near_axis_rate` example in `crx-kinematics/examples` measures the failure rate and prints the
missed poses.

### Unrecoverable configurations

The method cannot recover two configurations documented under
[Unrecoverable configurations](https://github.com/mattj23/crx-kinematics/blob/main/docs/LINEAR_METHOD.md#unrecoverable-configurations).
For both configurations, the solver returns solutions that reach the requested pose without
recovering the configuration that produced it. Both place the wrist center inside the robot's base
casting, so a real arm can't reach them.

### Joint ranges and physical collisions

Candidates are built with J1, J2, J4, J5, and J6 in $[-180, 180)$ degrees and J3 in $(-360, 360)$, 
because the J2/J3 coupling is undone after wrapping, and the joint polish can then carry a value 
slightly past either end. The solver reports every configuration that reaches the pose, including 
configurations outside the joint limits of the physical robot and configurations that pass through 
the base or the floor. For now the caller must filter solutions against the applicable limits.

> [!NOTE]
> The robot datasheets publish the actual joint limits, and they vary by model.  On the 5iA and the 10iA
> (which I have on hand) all joints are capable of at least 360 degrees total, while most are over 
> 380 and J3 is as high as 635 on the 5iA.  This extra angular range isn't returned by this library
> which, for now, is only returning mathematically unique configurations.  I haven't decided how...or even
> _if_...it makes sense to handle the extra range.

Additionally, this library does no collision detection.  Valid joint solutions may produce positions
where the robot hits itself.  Since the library has no way of knowing where you've put your robot
and what you might have attached to it, you'll have to do collision detection on the resulting
solutions to know if they're safe for your setup.
