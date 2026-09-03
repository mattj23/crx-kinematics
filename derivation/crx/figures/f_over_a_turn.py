"""
Figure: $f$ over one turn.

$f$ is a trigonometric polynomial of degree four, so it has at most eight roots in a full turn, and
each root is one arm posture. The plot illustrates the degree argument by showing the polynomial's
shape and the roots that correspond to solutions.

Two poses are shown, one with four roots and one with two, which become eight and four solutions
once the base flip doubles them.
"""

from __future__ import annotations

import math

import numpy
from matplotlib.figure import Figure

from ..ik_reference import CrxParams, Setup, dft_coefficients, eval_f, fk, ik, theta_roots

POSES = (
    (90.0, -20.0, -60.0, -30.0, 40.0, -80.0),
    (70.0, 30.0, 100.0, 60.0, -50.0, 30.0),
)
"""
Two CRX-10iA configurations selected by searching random configurations for root counts of four and
two. The assertions below verify these counts.
"""

CURVE_COLOR = "tab:blue"
ROOT_COLOR = "tab:red"


def draw() -> Figure:
    params = CrxParams.crx10ia()
    thetas = numpy.linspace(-math.pi, math.pi, 1024)

    figure = Figure(figsize=(9.0, 5.0), dpi=150)
    axes = figure.subplots(2, 1, sharex=True)

    for ax, joints in zip(axes, POSES):
        target = fk(params, joints)
        setup = Setup(params, target)
        coefficients = dft_coefficients(setup)

        values = numpy.array([eval_f(coefficients, t)[0] for t in thetas])
        roots = sorted(theta_roots(coefficients))
        solutions = len(ik(params, target))
        assert solutions == 2 * len(roots), "the base flip should double every root"

        # f spans many orders of magnitude between poses, and only its zeros carry meaning, so each
        # panel is normalized to its own largest value.
        scale = numpy.abs(values).max()
        ax.axhline(0.0, color="0.6", linewidth=0.8)
        ax.plot(thetas, values / scale, color=CURVE_COLOR, linewidth=1.6)
        ax.plot(roots, numpy.zeros(len(roots)), linestyle="none", marker="o", markersize=7.0,
                markerfacecolor="white", markeredgecolor=ROOT_COLOR, markeredgewidth=1.6)

        ax.set_title(f"J = {[int(v) for v in joints]}:  "
                     f"{len(roots)} roots, {solutions} solutions", fontsize=11)
        ax.set_ylabel(r"$f(\theta)$, normalized", fontsize=10)
        ax.set_ylim(-1.15, 1.15)
        ax.grid(alpha=0.25)

    axes[-1].set_xlim(-math.pi, math.pi)
    axes[-1].set_xticks([-math.pi, -math.pi / 2, 0.0, math.pi / 2, math.pi],
                        [r"$-\pi$", r"$-\pi/2$", r"$0$", r"$\pi/2$", r"$\pi$"])
    axes[-1].set_xlabel(r"$\theta$", fontsize=12)

    figure.tight_layout()
    return figure
