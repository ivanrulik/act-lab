"""Deterministic, dependency-free driver seam for contract tests, not physics."""

from __future__ import annotations

from dataclasses import dataclass

from act_lab.application import SafeCartesianRobot
from act_lab.application.cartesian_control import JointVector
from act_lab.domain import Observation, Pose, RobotState

HOME = Pose("world", (0.2, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0))
ZERO: JointVector = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class DemonstratorLimits:
    pose_response_time_s: float = 0.1
    watchdog_timeout_ns: int = 100_000_000
    workspace_x_m: tuple[float, float] = (-0.2, 0.8)
    workspace_y_m: tuple[float, float] = (-0.35, 0.6)
    workspace_z_m: tuple[float, float] = (0.44, 1.0)
    max_translation_velocity_m_s: float = 0.25
    max_command_translation_velocity_m_s: float = 0.125
    max_translation_acceleration_m_s2: float = 1.0
    max_orientation_velocity_rad_s: float = 1.0
    max_orientation_acceleration_rad_s2: float = 4.0
    max_joint_velocity_rad_s: float = 1.0
    max_joint_acceleration_rad_s2: float = 4.0
    joint_bound_margin_rad: float = 0.02
    max_gripper_velocity_s: float = 2.0


class DeterministicDriver:
    """Stationary pose, explicit 20 ms steps; IK/collision are controllable seams."""

    control_period_s = 0.02
    joint_position_bounds_rad = ((-2.0, 2.0),) * 6

    def __init__(self) -> None:
        self.state = RobotState(0, ZERO, ZERO, HOME, 0.3)
        self.ik_result: JointVector | None = ZERO
        self.is_collision_free = True

    def reset(self, seed: int) -> Observation:
        del seed
        self.state = RobotState(0, ZERO, ZERO, HOME, 0.3)
        return self.observe()

    def observe(self) -> Observation:
        return Observation(self.state.timestamp_ns, self.state, ())

    def solve_ik(self, target_pose: Pose) -> JointVector | None:
        del target_pose
        return self.ik_result

    def collision_free(self, joints: JointVector, gripper: float) -> bool:
        del joints, gripper
        return self.is_collision_free

    def step(
        self, joints: JointVector, joint_velocity: JointVector, gripper: float
    ) -> RobotState:
        del joint_velocity
        self.state = RobotState(
            self.state.timestamp_ns + 20_000_000, joints, ZERO, HOME, gripper
        )
        return self.state


def make_robot() -> tuple[SafeCartesianRobot, DeterministicDriver]:
    driver = DeterministicDriver()
    robot = SafeCartesianRobot(driver, DemonstratorLimits())
    robot.reset(0)
    return robot, driver
