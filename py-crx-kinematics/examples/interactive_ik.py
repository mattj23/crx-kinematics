"""
An interactive robot driven by its inverse kinematics.

The scene shows a CRX-10iA and the target frame for its flange. Moving the frame with the keyboard
solves the inverse kinematics for the new pose and moves the arm to the selected solution. The
solver selects the solution nearest the arm's current configuration to prevent the arm from
reconfiguring while it follows the frame.

Requires `engeom` and `pyvista` in addition to this package:

    pip install crx-kinematics engeom pyvista

Then run it:

    python interactive_ik.py

Controls, printed again in the window:

    w / s     move the frame along world X
    a / d     move the frame along world Y
    q / e     move the frame along world Z
    i / k     rotate the frame about its own X
    j / l     rotate the frame about its own Y
    u / o     rotate the frame about its own Z
    [ / ]     halve or double the step size
    r         return to the starting pose
    Escape    close the window

The example overrides several default VTK key bindings that conflict with these controls. By
default, `q` and `e` close the window, `w` and `s` change the rendering style, and `r` resets the
camera. Mouse controls for the camera remain available.

For an unreachable pose, the robot remains in its current configuration and the frame turns red.
The frame continues to move, allowing the user to return it to a reachable region.
"""

from __future__ import annotations

import math

import numpy
import pyvista
from crx_kinematics import Crx, CrxModel, LinkMeshes
from engeom.geom3 import Iso3, Mesh3
from pyvista import Plotter

MODEL = CrxModel.Crx10iA
"""The model to draw. Geometry is embedded for the CRX-5iA and the CRX-10iA."""

HOME = [0.0, -40.0, 0.0, 30.0, -30.0, 0.0]
"""
The configuration the arm starts in, in controller degrees.

It reaches forward and upward, stays well inside the workspace, and keeps the wrist center away
from the J1 axis. With the wrist center on that axis, the solutions become degenerate and the
solution nearest the current configuration can be almost a full turn away. Selecting that solution
would make the arm jump between configurations. From this starting configuration, a step of the
default size moves the joints by approximately ten degrees in the worst case.
"""

LINK_COLORS = ["gray", "white", "white", "white", "white", "white", "gray"]
"""One color per mesh: gray for the stationary base and the flange, white for the moving links."""

TRANSLATION_STEP = 25.0
"""Millimeters per key press, before any change with the bracket keys."""

ROTATION_STEP = math.radians(5.0)
"""Radians per key press, before any change with the bracket keys."""

AXIS_COLORS = ("red", "green", "blue")
"""The color of the target frame's X, Y, and Z axes."""

FRAME_LENGTH = 200.0
"""The length of each axis of the target frame, in millimeters."""


def renormalized(pose: Iso3) -> Iso3:
    """
    Return a pose reconstructed with a unit quaternion.

    Each key press composes another rotation with the target, and floating-point error accumulates
    across these operations. Normalize the target quaternion after every move to keep the rotation
    orthonormal within the solver's tolerance throughout the session. The reconstruction preserves
    the translation and limits rotational drift to the floating-point precision floor.

    :param pose: the pose to renormalize
    :return: the same translation and rotation represented by a unit quaternion
    """
    x, y, z, i, j, k, w = pose.to_quaternion()
    norm = math.sqrt(i * i + j * j + k * k + w * w)
    return Iso3.from_quaternion(x, y, z, i / norm, j / norm, k / norm, w / norm)


class InteractiveRobot:
    """
    A robot in a PyVista scene that follows a target frame.

    :param plotter: the PyVista plotter to draw into
    :param model: the CRX model to draw, which must be one with embedded geometry
    :param joints: the configuration to start in, in controller degrees
    """

    def __init__(self, plotter: Plotter, model: CrxModel = MODEL, joints=HOME):
        self.plotter = plotter
        self.robot = Crx.from_model(model)
        self.meshes = LinkMeshes.load(model)

        self.joints = numpy.asarray(joints, dtype=float)
        self.home = self.joints.copy()

        # The initial target is the flange pose produced by the home configuration. Each subsequent
        # target is a transformation of this pose.
        self.target = Iso3(self.robot.fk(self.joints))
        self.home_target = self.target

        self.scale = 1.0
        self.reachable = True

        # `build` initializes these references when it adds the geometry to the scene.
        self.link_actors = []
        self.axis_actors = []
        self.status_actor = None

    # Target and joint updates.

    def try_target(self, target: Iso3) -> None:
        """
        Move the frame, and move the arm onto it if the pose can be reached.

        Select the solution nearest the current configuration so the arm follows the frame without
        jumping between configurations. For an unreachable pose, keep the arm at its current
        configuration while the frame continues to accept movement, allowing the user to return it
        to a reachable region.

        :param target: the pose to ask the flange to reach
        """
        # Every target update passes through this method, so normalize the rotation here.
        target = renormalized(target)

        self.target = target
        solution = self.robot.ik_closest(target, self.joints)

        self.reachable = solution is not None
        if solution is not None:
            self.joints = solution.joints

        self.refresh()

    def translate(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> None:
        """Move the frame along the world axes, in millimeters scaled by the current step size."""
        step = self.scale
        shift = Iso3.from_translation(dx * step, dy * step, dz * step)
        self.try_target(shift @ self.target)

    def rotate(self, rx: float = 0.0, ry: float = 0.0, rz: float = 0.0) -> None:
        """
        Turn the frame about its own axes, in radians scaled by the current step size.

        Apply the rotation on the right to rotate about the frame axes. Rotating about these local
        axes makes the controls behave like a tool turning in the operator's hand; applying the
        rotation on the left would use the world axes.
        """
        step = self.scale
        turn = Iso3.from_rx(rx * step) @ Iso3.from_ry(ry * step) @ Iso3.from_rz(rz * step)
        self.try_target(self.target @ turn)

    def rescale(self, factor: float) -> None:
        """Change how far a single key press moves the frame."""
        self.scale = min(max(self.scale * factor, 0.05), 8.0)
        self.refresh()

    def reset(self) -> None:
        """Return the arm and the frame to where they started."""
        self.joints = self.home.copy()
        self.reachable = True
        self.try_target(self.home_target)

    # Scene rendering.
    #
    # Add the geometry to the scene once in its authored frames. Moving the robot requires only
    # assigning a new matrix to each actor. VTK applies these matrices during rendering, so Python
    # does not transform the forty-six thousand vertices or rebuild and upload their buffers.

    def build(self) -> None:
        """
        Add the robot and the target frame to the scene, then place them.

        Call this once. Every later change goes through `refresh`.
        """
        helper = self.plotter.engeom

        self.link_actors = [
            helper.draw_mesh(
                Mesh3(link.vertices, link.faces),
                color=LINK_COLORS[i],
                name=f"link-{i}",
            )
            for i, link in enumerate(self.meshes.links)
        ]

        # Draw the frame as three axis lines at the origin, then place it with the target matrix in
        # the same way as the links. Create the axes separately because `draw_coordinate_system`
        # assigns the same name to every actor in a frame. Each named actor would replace the
        # previous actor, leaving only one visible axis.
        origin = numpy.zeros(3)
        self.axis_actors = []
        for axis, color in enumerate(AXIS_COLORS):
            tip = numpy.zeros(3)
            tip[axis] = FRAME_LENGTH
            self.axis_actors.append(
                self.plotter.add_mesh(
                    pyvista.Line(origin, tip),
                    color=color,
                    line_width=4.0,
                    name=f"target-axis-{axis}",
                )
            )

        # Create the status actor once and update its text in place during each refresh.
        self.status_actor = self.plotter.add_text(
            "", position="upper_left", font_size=10, name="status"
        )

        self.refresh()

    def refresh(self) -> None:
        """
        Move the actors already in the scene onto the current joints and target.

        Each link takes the pose of its corresponding frame, and the frame axes take the target
        pose. A move updates only the actors' `user_matrix` values.
        """
        for actor, pose in zip(self.link_actors, self.meshes.poses(self.robot, self.joints)):
            actor.user_matrix = pose

        matrix = self.target.as_numpy()
        for axis, actor in enumerate(self.axis_actors):
            actor.user_matrix = matrix
            # A reachable frame keeps its axis colors. An unreachable one turns red, which is the
            # signal that the arm has stopped following.
            actor.prop.color = AXIS_COLORS[axis] if self.reachable else "red"

        self.status_actor.set_text("upper_left", self.status())

        # Moving an actor and rewriting text mark the scene as changed without drawing it. This
        # update adds no geometry to trigger an automatic redraw, so request a render explicitly.
        self.plotter.render()

    def status(self) -> str:
        """The joint values, the step size, and whether the current pose can be reached."""
        joints = "  ".join(f"J{i + 1}{value:+8.2f}" for i, value in enumerate(self.joints))
        reach = "reachable" if self.reachable else "OUT OF REACH, arm holding"
        return (
            f"{joints}\n"
            f"step {TRANSLATION_STEP * self.scale:.1f} mm  "
            f"{math.degrees(ROTATION_STEP * self.scale):.1f} deg\n"
            f"{reach}"
        )

    # Keyboard bindings.

    def bind_keys(self) -> None:
        """
        Attach the keyboard controls to the plotter.

        VTK assigns default actions to most letter keys. Its `q` and `e` actions close the window,
        `w` and `s` select wireframe and surface rendering, and `r` resets the camera. These default
        actions use the interactor's `CharEvent`, while the callbacks below use `KeyPressEvent`.
        Remove the `CharEvent` observers to disable the conflicting VTK actions while retaining
        these callbacks. Removing the observers also removes VTK's keyboard command for closing the
        window, so bind Escape to that action.
        """
        self.plotter.iren.interactor.RemoveObservers("CharEvent")

        bindings = {
            "w": lambda: self.translate(dx=+TRANSLATION_STEP),
            "s": lambda: self.translate(dx=-TRANSLATION_STEP),
            "a": lambda: self.translate(dy=+TRANSLATION_STEP),
            "d": lambda: self.translate(dy=-TRANSLATION_STEP),
            "q": lambda: self.translate(dz=+TRANSLATION_STEP),
            "e": lambda: self.translate(dz=-TRANSLATION_STEP),
            "i": lambda: self.rotate(rx=+ROTATION_STEP),
            "k": lambda: self.rotate(rx=-ROTATION_STEP),
            "j": lambda: self.rotate(ry=+ROTATION_STEP),
            "l": lambda: self.rotate(ry=-ROTATION_STEP),
            "u": lambda: self.rotate(rz=+ROTATION_STEP),
            "o": lambda: self.rotate(rz=-ROTATION_STEP),
            "bracketright": lambda: self.rescale(2.0),
            "bracketleft": lambda: self.rescale(0.5),
            "r": self.reset,
            "Escape": self.plotter.close,
        }
        for key, action in bindings.items():
            self.plotter.add_key_event(key, action)


HELP = """Move the flange target
  w/s  X    a/d  Y    q/e  Z
  i/k  turn X   j/l  turn Y   u/o  turn Z
  bracket keys  step size    r  reset    Esc  quit"""


def main() -> None:
    plotter = Plotter()

    # Engeom declares its PyVista helper as a plugin, so PyVista attaches it to every plotter as
    # `plotter.engeom` without requiring an import from `engeom.plot`.
    robot = InteractiveRobot(plotter)
    robot.bind_keys()
    robot.build()

    plotter.add_text(HELP, position="lower_left", font_size=9, name="help")
    plotter.show()


if __name__ == "__main__":
    main()
