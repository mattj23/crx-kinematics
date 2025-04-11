import numpy
from engeom._plot.pyvista import PyvistaPlotterHelper
from engeom.geom3 import Mesh, Iso3, Point3
from pathlib import Path

_mesh_path = Path(__file__).parent / "meshes"


class Robot:
    def __init__(self, z1: float, x1: float, x2: float, y1: float, prefix: str):
        """

        :param z1: The height from the J2 axis to the J3 axis
        :param x1: The length from the J3 axis to the J5 axis
        :param x2: The length from the J5 axis to the robot flange
        :param y1: The offset between the J1 and J6 axes
        """
        self.z1 = z1
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1

        self.meshes = [Mesh.load_stl(_mesh_path / f"{prefix}-{i}.stl") for i in range(7)]
        self.h = numpy.array([
            [0.0, 0.0, 0.0, -1.0, 0.0, -1.0],
            [0.0, 1.0, -1.0, 0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ])

        self.p = numpy.zeros(shape=(3, 7), dtype=float)
        self.p[2, 2] = z1
        self.p[0, 4] = x1
        self.p[1, 4] = -y1
        self.p[0, 5] = x2

        self._joint_radians = [0.0] * 6
        self._frames = None
        self._actors = []

    @property
    def frames(self) -> list[Iso3]:
        if self._frames is None:
            self._frames = self._calc_frames()
        return self._frames

    def _calc_frames(self) -> list[Iso3]:
        f1 = Iso3.from_rotation(self._joint_radians[0], self.h[0, 0], self.h[1, 0], self.h[2, 0])
        f2 = f1 @ self._frame_link(1)
        f3 = f2 @ self._frame_link(2)
        f4 = f3 @ self._frame_link(3)
        f5 = f4 @ self._frame_link(4)
        f6 = f5 @ self._frame_link(5) @ fanuc_end()

        return [f1, f2, f3, f4, f5, f6]

    def frame_origin(self, i: int) -> Point3:
        if i == 3:
            f = self.frames[3] @ Iso3.from_translation(self.x1, 0, 0)
        else:
            f = self.frames[i]

        return f @ Point3(0, 0, 0)

    def _frame_link(self, i: int) -> Iso3:
        return (Iso3.from_translation(*self.p[:, i].flatten()) @
                Iso3.from_rotation(self._joint_radians[i], self.h[0, i], self.h[1, i], self.h[2, i]))

    def set_joints(self, degrees: list[float]):
        if len(degrees) != 6:
            raise ValueError("Degrees must be a list of 6 elements")
        self._joint_radians = _joints_to_radians(degrees)
        self._frames = None

    def plot(self, helper: PyvistaPlotterHelper, opacity: float = 0.5):
        for actor in self._actors:
            helper.plotter.remove_actor(actor)
        self._actors.clear()

        colors = ["gray", "white", "white", "white", "white", "white", "gray"]
        for mesh, c in zip(self.posed_meshes(), colors):
            actor = helper.add_mesh(mesh, color=c, opacity=opacity)
            self._actors.append(actor)

    def posed_meshes(self):
        result = []
        frames = [Iso3.identity(), ] + self.frames
        for frame, mesh in zip(frames, self.meshes):
            temp = mesh.cloned()
            temp.transform_by(frame)
            result.append(temp)
        return result

    def posed_single_mesh(self):
        meshes = self.posed_meshes()
        mesh = meshes[0]
        for m in meshes[1:]:
            mesh.append(m)
        return mesh

    @staticmethod
    def crx5ia():
        return Robot(410, 430, 145, 130, "crx5ia")

    @staticmethod
    def crx10ia():
        return Robot(540, 540, 160, 150, "crx10ia")


def _joints_to_radians(joints: list[float]) -> list[float]:
    radians = [numpy.radians(j) for j in joints]
    radians[2] += radians[1]
    return radians


def fanuc_end() -> Iso3:
    return Iso3.from_rotation(-numpy.pi / 2.0, 0, 1, 0) @ Iso3.from_rotation(numpy.pi, 1, 0, 0)
