"""
Export fixtures that compare the Rust implementation with this reference.

The Rust solver is a port of the reference implementation. Its tests read the files written by this
script and compare both implementations on the same poses.

Run from the derivation project with its virtual environment:

    .venv/bin/python export_fixtures.py

The output lands in the Rust crate's test data directory, beside the recorded controller poses.
"""

import json
from pathlib import Path

import numpy

from crx.ik_reference import ALL_MODELS, Setup, dft_coefficients, fk, theta_roots

_OUTPUT = Path(__file__).parents[1] / "crx-kinematics" / "tests" / "data" / "theta_roots.json"

_PER_MODEL = 50
"""Fifty random poses per model cover ordinary cases; `_degenerate_families` adds edge cases."""

_SEED = 20260903
"""A fixed seed prevents unnecessary fixture changes when this script is run again."""


def _degenerate_families(rng: numpy.random.Generator) -> list[list[float]]:
    """
    Return joint values for geometries that a naive solver mishandles, ensuring that the fixture
    always covers these cases.
    """
    cases = []
    for _ in range(6):
        base = rng.uniform(-180.0, 180.0, 6)

        on_axis = base.copy()          # O4 lands on the J1 axis
        on_axis[2] = on_axis[1] + 90.0
        cases.append(on_axis)

        near_axis = on_axis.copy()     # and just off it
        near_axis[2] += 1e-5
        cases.append(near_axis)

        merged = base.copy()           # the elbow pair merges
        merged[3] = 0.0
        cases.append(merged)

        wrist = base.copy()            # wrist singularity
        wrist[4] = 0.0
        cases.append(wrist)

        stretched = base.copy()        # upper arm and forearm in line
        stretched[1] = 90.0
        stretched[2] = 0.0
        cases.append(stretched)

    # The pose from issue #1, which is the on-axis case on a CRX-10iA.
    cases.append(numpy.array([10.0, -80.0, 10.0, 20.0, -20.0, 45.0]))
    return [list(c) for c in cases]


def main() -> None:
    rng = numpy.random.default_rng(_SEED)
    records = []

    for name, params in sorted(ALL_MODELS.items()):
        configurations = [list(j) for j in rng.uniform(-180.0, 180.0, size=(_PER_MODEL, 6))]
        if name == "crx10ia":
            configurations += _degenerate_families(rng)

        for joints in configurations:
            target = fk(params, joints)
            setup = Setup(params, target)
            records.append({
                "model": name,
                "joints": joints,
                "target": [float(x) for x in target.flatten()],
                "thetas": sorted(float(t) for t in theta_roots(dft_coefficients(setup))),
            })

    _OUTPUT.write_text(json.dumps(records, indent=1))
    counts: dict[int, int] = {}
    for record in records:
        counts[len(record["thetas"])] = counts.get(len(record["thetas"]), 0) + 1
    print(f"wrote {len(records)} poses to {_OUTPUT}")
    print(f"root counts: {dict(sorted(counts.items()))}")


if __name__ == "__main__":
    main()
