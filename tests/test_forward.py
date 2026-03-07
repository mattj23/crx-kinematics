"""
    Tests for the forward kinematics of the CRX robots
"""

import pytest
import json
from pathlib import Path
from crx.robot import Robot


def test_crx5ia_known():
    robot = Robot.crx5ia()
    known = _load_known("fanuc_crx_5ia")

    for joints, expected in known:
        _compare_robot(robot, joints, expected)


def test_crx10ia_known():
    robot = Robot.crx10ia()
    known = _load_known("fanuc_crx_10ia")

    for joints, expected in known:
        _compare_robot(robot, joints, expected)


def _compare_robot(robot: Robot, joints: list[float], expected: list):
    robot.set_joints(joints)
    end_frame = [float(x) for x in robot.frames[-1].as_numpy().flatten()]
    assert end_frame == pytest.approx(expected,
                                      abs=1e-8), f"Failed for joints {joints} with expected {expected} and got {end_frame}"


def _load_known(name: str) -> list[list]:
    path = Path(__file__).parent / "data" / f"{name}.json"
    with open(path, "r") as f:
        return json.load(f)
