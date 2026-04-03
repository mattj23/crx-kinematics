from crx.robot import Robot

from engeom.geom3 import Circle3, Iso3, Plane3



def reduced_problem(robot: Robot, circle_o4: Circle3, radius_o3: float, iso_reduced: Iso3):
    # The torus equivalence: the O4 circle can only reach the O3 circle where it is within the `x1` distance of at
    # least one of its points. The volume that is reachable from the O3 circle is a torus with a major radius of the
    # O3 circle diameter and a minor dimension of `x1`.
    #
    # Figuring out which parts of the O4 circle are within this torus turns out to be straightforward because
    # the problem is almost axisymmetric. The
    pass

