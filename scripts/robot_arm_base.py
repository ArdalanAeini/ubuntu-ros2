"""
Abstract base class for a robotic arm.
Any concrete robot (UR5, myCobot, JetCoBot) must implement this interface.
This is the Hardware Abstraction Layer (HAL).
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List
import math


@dataclass
class JointPositions:
    """
    Represents the angles of all joints in an arm.
    Internal unit is RADIANS (standard in robotics / ROS).
    Use from_degrees() if you want to think in degrees.
    """
    values: List[float]  # radians

    @classmethod
    def from_degrees(cls, degrees: List[float]) -> "JointPositions":
        return cls(values=[math.radians(d) for d in degrees])

    def to_degrees(self) -> List[float]:
        return [math.degrees(r) for r in self.values]

    def __len__(self) -> int:
        return len(self.values)


class RobotArm(ABC):
    """
    Abstract robotic arm. Defines WHAT every arm can do.
    Subclasses define HOW (UR5 via ROS actions, myCobot via serial, etc).
    """

    # ---- Abstract methods every subclass must implement ----

    @property
    @abstractmethod
    def num_joints(self) -> int:
        """How many joints this arm has."""
        ...

    @abstractmethod
    def connect(self) -> None:
        """Establish connection to the robot (or simulator)."""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Cleanly close the connection."""
        ...

    @abstractmethod
    def get_current_positions(self) -> JointPositions:
        """Read the current joint angles, in radians."""
        ...

    @abstractmethod
    def move_to_joint_positions(
        self,
        target: JointPositions,
        duration_sec: float = 3.0,
    ) -> bool:
        """
        Command the arm to move all joints to `target` over `duration_sec`.
        Returns True if motion completed successfully.
        """
        ...

    # ---- Concrete helpers built on the abstract methods ----

    def move_single_joint(
        self,
        joint_index: int,
        target_radians: float,
        duration_sec: float = 2.0,
    ) -> bool:
        """Move ONE joint to a target angle, leaving the rest where they are."""
        current = self.get_current_positions()
        new_values = list(current.values)
        new_values[joint_index] = target_radians
        return self.move_to_joint_positions(
            JointPositions(values=new_values),
            duration_sec=duration_sec,
        )

    def go_home(self, duration_sec: float = 4.0) -> bool:
        """Move all joints to 0 radians (the 'zero' pose)."""
        zeros = JointPositions(values=[0.0] * self.num_joints)
        return self.move_to_joint_positions(zeros, duration_sec=duration_sec)

    def move_to_xyz(
        self,
        x: float,
        y: float,
        z: float,
        duration_sec: float = 3.0,
    ) -> bool:
        """
        Move the end-effector to a target XYZ position in the base frame.
        Orientation is left to the IK solver to decide.

        Subclasses that have kinematics should override this.
        Default implementation raises NotImplementedError because
        not every robot has IK wired up.
        """
        raise NotImplementedError(
            "This robot does not have kinematics support. "
            "Subclass must override move_to_xyz()."
        )

    # ---- Context manager support so we can use `with RobotArm() as arm:` ----

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False