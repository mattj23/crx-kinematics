"""
A CRX robot with visual meshes for drawing the figures used in the derivation.

This module uses the tested forward model from :mod:`crx.ik_reference` to keep figures consistent
with calculated solutions. It adds a mesh for each link and supports two plotting forms:
individual posed meshes for a PyVista scene and a merged mesh for Matplotlib hidden-line drawings.

Meshes are available for the CRX-5iA and CRX-10iA. The inverse kinematics supports the other four
models, but this module cannot draw them.
"""

from __future__ import annotations

from pathlib import Path

import numpy
from engeom.geom3 import Iso3, Mesh3, Point3

from .ik_reference import CrxParams, fk_all

_MESH_PATH = Path(__file__).parent / "meshes"

_LINK_COLORS = ["gray", "white", "white", "white", "white", "white", "gray"]
"""One color per mesh: gray for the base and flange, and white for the moving links."""


class Robot:
    """
    Represent a drawable CRX robot with link geometry, joint configuration, and calculated frames.

    :param params: the four link lengths that identify the model
    :param prefix: the stem of the mesh filenames, as in ``crx10ia-0.tcmesh`` through
        ``crx10ia-6.tcmesh``
    :param joints: the initial joint configuration in FANUC controller degrees
    """

    def __init__(self, params: CrxParams, prefix: str, joints=(0.0,) * 6):
        self.params = params
        self.meshes = [Mesh3.load_tcmesh(_MESH_PATH / f"{prefix}-{i}.tcmesh") for i in range(7)]
        self._joints = numpy.zeros(6)
        self._frames: list[Iso3] | None = None
        self.set_joints(joints)

    @staticmethod
    def crx5ia(joints=(0.0,) * 6) -> "Robot":
        return Robot(CrxParams.crx5ia(), "crx5ia", joints)

    @staticmethod
    def crx10ia(joints=(0.0,) * 6) -> "Robot":
        return Robot(CrxParams.crx10ia(), "crx10ia", joints)

    @property
    def joints(self) -> numpy.ndarray:
        """The current configuration, in FANUC controller degrees."""
        return self._joints.copy()

    def set_joints(self, degrees) -> None:
        """
        Set the robot's joint configuration and invalidate its cached frames.

        :param degrees: six joint angles in FANUC controller degrees
        :raises ValueError: if six angles were not given
        """
        degrees = numpy.asarray(degrees, dtype=float)
        if degrees.shape != (6,):
            raise ValueError(f"Expected six joint angles, got {degrees.shape}")
        self._joints = degrees
        self._frames = None

    @property
    def frames(self) -> list[Iso3]:
        """
        Return the six kinematic link frames, computed on first use and cached until the joints move.

        The last one is the flange pose and matches what the controller reports.
        """
        if self._frames is None:
            self._frames = [Iso3(m) for m in fk_all(self.params, self._joints)]
        return self._frames

    @property
    def mesh_poses(self) -> list[Iso3]:
        """
        Return the poses for the seven meshes. The stationary base uses the identity isometry, and
        the remaining six meshes use the corresponding link frames.
        """
        return [Iso3.identity()] + self.frames

    def frame_origin(self, index: int) -> Point3:
        """
        Return origin $O_{index+1}$ in world coordinates.

        The fourth frame is the exception: its own origin sits at $O_3$, and $O_4$ is a fixed
        offset of ``x1`` along that frame's X axis.

        :param index: 0 through 5, selecting $O_1$ through $O_6$
        """
        frame = self.frames[index]
        if index == 3:
            frame = frame @ Iso3.from_translation(self.params.x1, 0.0, 0.0)
        return frame @ Point3(0.0, 0.0, 0.0)

    def posed_meshes(self) -> list[Mesh3]:
        """Return copies of the link meshes transformed to their current poses."""
        return [mesh.transform_copy(pose) for mesh, pose in zip(self.meshes, self.mesh_poses)]

    def posed_single_mesh(self) -> Mesh3:
        """
        Return all posed links merged into one mesh.

        Hidden-line drawings require the merged mesh because outlines calculated separately for
        each link would include seams where links overlap.
        """
        merged, *rest = self.posed_meshes()
        for mesh in rest:
            merged.append_in_place(mesh)
        return merged

    def draw(self, helper, opacity: float = 0.5, name: str = "robot"):
        """
        Draw the robot into a PyVista scene, one actor per link.

        The method applies each pose at draw time and leaves the source meshes unchanged. Each actor
        has a name, so calling this method after moving the joints replaces the previous drawing
        and prevents actors from accumulating.

        :param helper: an ``engeom.plot.pyvista.PlotterHelper``, which a plotter also exposes as
            its ``engeom`` attribute
        :param opacity: how transparent to draw the links
        :param name: the stem of the actor names, so that two robots can share a scene
        :return: the actors, one per link
        """
        return [
            helper.draw_mesh(mesh, color=color, opacity=opacity, pose=pose, name=f"{name}-{i}")
            for i, (mesh, pose, color) in enumerate(zip(self.meshes, self.mesh_poses, _LINK_COLORS))
        ]
