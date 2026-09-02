import numpy
import math
from crx.robot import Robot, RobotKinematics
from engeom.geom2 import Circle2, Point2, signed_angle
from engeom.geom3 import Circle3, Iso3, Plane3, Sphere3, Point3, Vector3, Line3
from engeom.plot import PyvistaPlotterHelper, TraceBuilder
from tqdm import tqdm

LEFT = [142.54483177183295, -179.52743756893878, -170.389578827691, -72.56376100376687, 103.35096616857578,
         -136.53272323905037]
RIGHT = [-37.455168228167054, 179.52743756893878, -9.610421172309003, 107.43623899623307, 103.35096616857575,
          -136.53272323905037]


def main():
    robot = Robot.crx10ia()
    # robot.set_joints(RIGHT)
    robot.set_joints(LEFT)


    plotter = PyvistaPlotterHelper.with_new_plotter(window_size=(2000, 1500))
    robot.plot(plotter)

    origins = [(list(robot.frame_origin(i)), f"o{i + 1}") for i in [0, 2, 3, 4, 5]]
    points, labels = zip(*origins)
    plotter.pv.add_point_labels(
        numpy.array(points),
        labels,
        point_color="black",
        point_size=10,
        font_size=16,
        render_points_as_spheres=True
    )

    plotter.coordinate_frame(Iso3.identity(), size=100)
    plotter.show()


if __name__ == '__main__':
    main()
