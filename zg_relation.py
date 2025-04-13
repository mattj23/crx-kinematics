"""
    This script is a test area for exploring the three different working coordinate systems of the kinematics model:
    the global coordinate system, the wrist coordinate system, and the vertical plane coordinate system.

"""

import numpy
from engeom.geom3 import Iso3, Point3, Vector3, Plane3, SurfacePoint3
from engeom.geom2 import Circle2, Aabb2, Point2
from engeom.plot import MatplotlibAxesHelper, TraceBuilder, PyvistaPlotterHelper
from matplotlib.pyplot import figure, Figure, Axes
from display_forward import upper_lower, corner_xy
from pyvista import Plotter
from tqdm import tqdm

from crx.robot import Robot


def main():
    robot = Robot.crx5ia()
    zg = SurfacePoint3(0, 0, 0, 0, 0, 1)
    o4r = robot.y1

    spacing = 2
    limits = [180, 180, 180, 180]

    j2, j3, j4, j5 = [numpy.linspace(-limit, limit, 2 * limit // spacing + 1) for limit in limits]
    results = []
    with tqdm(total=len(j2) * len(j3) * len(j4) * len(j5)) as pbar:
        for a in j2:
            for b in j3:
                for c in j4:
                    for d in j5:
                        pbar.update(1)
                        robot.set_joints([0, a, b - a, c, d, 0])
                        target_frame = robot.frames[-2] @ Iso3.from_rotation(numpy.pi / 2.0, 0, 1, 0)
                        z_plane = (target_frame @ zg).get_plane()
                        t = z_plane.intersection_distance(zg)
                        if t is None:
                            continue
                        ia = SurfacePoint3(*zg.at_distance(t), 0, 0, 1)
                        ia = target_frame.inverse() @ ia
                        if ia.normal.angle_to(zg.normal) < 0.5 * numpy.pi / 180.0:
                            results.append(ia)
                            # continue
                        # if ia.point.coords.norm() < 2 * o4r:
                        #     results.append(ia)

    plotter = Plotter()
    helper = PyvistaPlotterHelper(plotter)
    helper.coordinate_frame(Iso3.identity(), size=200, line_width=1.0)

    circle = numpy.array([[numpy.cos(x), numpy.sin(x), 0] for x in numpy.linspace(0, 2 * numpy.pi, 100)]) * o4r
    plotter.add_lines(circle, connected=True, color="red", width=1.0)

    values = []
    for result in results:
        values.append(list(result.point))
        values.append(list(result.at_distance(50)))
    values = numpy.array(values)

    # numpy.save("zg.npy", values)

    plotter.add_lines(values, color="orange", width=1.0)

    plotter.show()


def check():
    robot = Robot.crx5ia()
    robot.set_joints([-10, -15, 15, 20, 45, 10])

    z_g = SurfacePoint3(0, 0, 0, 0, 0, 1)
    target_frame = robot.frames[-2] @ Iso3.from_rotation(numpy.pi / 2.0, 0, 1, 0)
    z_plane = (target_frame @ z_g).get_plane()
    t = z_plane.intersection_distance(z_g)

    ia = SurfacePoint3(*z_g.at_distance(t), 0, 0, 1)

    plotter = Plotter()
    helper = PyvistaPlotterHelper(plotter)
    robot.plot(helper)
    helper.coordinate_frame(target_frame, size=50, label="Origin")
    # helper.coordinate_frame(target_frame.inverse(), size=50, label="Origin-1")

    plotter.add_lines(numpy.array([list(ia.point), list(ia.at_distance(50))]), color="red")

    ia = target_frame.inverse() @ ia
    plotter.add_lines(numpy.array([list(ia.point), list(ia.at_distance(50))]), color="orange", width=1)

    #
    # plotter.add_lines(o4_points, connected=True, color="red", width=1.0)

    helper.coordinate_frame(Iso3.identity(), size=100, label="Origin")
    plotter.add_axes()
    plotter.show()


if __name__ == '__main__':
    main()
