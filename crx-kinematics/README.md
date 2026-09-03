# crx-kinematics

Forward and inverse kinematics for the [FANUC CRX](https://www.fanuc.co.jp/en/product/robot/index.html)
family of collaborative robots. The only dependency is [`nalgebra`](https://nalgebra.org/).

The inverse kinematics is complete and direct. For a reachable flange pose it returns *every* joint
configuration that reaches it, up to the sixteen this architecture allows, and it does so without
seeding, sampling, or iterating toward a single answer. The method reduces the problem to the roots
of one scalar equation whose degree is known in advance, so the count of candidates is fixed before
any arithmetic is done and none can be missed. It is derived in full in
[LINEAR_METHOD.md](https://github.com/mattj23/crx-kinematics/blob/main/docs/LINEAR_METHOD.md).

```bash
cargo add crx-kinematics
```

```rust
use crx_kinematics::Crx;

let robot = Crx::new_10ia();

// Forward: joint angles in controller degrees to a flange pose.
let target = robot.fk(&[10.0, -80.0, 10.0, 20.0, -20.0, 45.0]);

// Inverse: every configuration that reaches the pose.
let solutions = robot.ik(&target);

// Or just the one nearest to where the arm already is.
let current = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
if let Some(best) = robot.ik_closest(&target, &current) {
    println!("{:?}, residual {:.1e}", best.joints, best.residual);
}
```

Joint angles are in degrees as the robot controller reports them, including the FANUC convention
where the J3 value is measured against J2 rather than against the world. Poses are plain
`nalgebra::Isometry3<f64>` values, so no conversion is needed to use this crate alongside another
built on the same types. `fk_all` returns the pose of every frame in the chain, and
`Crx::from_params` builds a robot from the four link dimensions directly.

All six kinematically distinct models are supported: the CRX-3iA, CRX-5iA, CRX-10iA, CRX-10iA/L,
CRX-20iA/L, and CRX-30iA.

Each solution carries the pose error it achieved and a `SolutionKind` marking whether it is an
isolated solution or one representative of a continuum. Solutions are polished until they stop
improving, which puts them at the floor of double precision; a full solve takes roughly 0.25 ms.

Several configurations are mathematically degenerate, including the wrist center landing on the J1
axis, which an operator can enter directly and which defeats other implementations of this
architecture. They are enumerated with their treatment in
[the derivation](https://github.com/mattj23/crx-kinematics/blob/main/docs/LINEAR_METHOD.md#degenerate-configurations),
along with
[two configurations the method cannot recover](https://github.com/mattj23/crx-kinematics/blob/main/docs/LINEAR_METHOD.md#unrecoverable-configurations),
both of which place the wrist center inside the robot's own base casting.

Python bindings are published separately as
[crx-kinematics](https://pypi.org/project/crx-kinematics/) on PyPI.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option.
