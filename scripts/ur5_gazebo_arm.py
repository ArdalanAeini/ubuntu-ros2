"""
UR5 implementation of RobotArm, talking to Gazebo simulation via ROS 2.

Architecture:
  - ROS 2 communicates via "topics" (continuous data) and "actions" (long-running goals).
  - Reading joint angles -> subscribe to /joint_states TOPIC.
  - Commanding motion -> send a goal to the FollowJointTrajectory ACTION.
  - We run the rclpy executor in a background thread so the main thread
    can call simple blocking methods like move_to_joint_positions().
"""

import threading
import time
import numpy as np
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from robot_arm_base import RobotArm, JointPositions
from kinematics import RobotKinematics


# UR5 joint names — order matters! This is the canonical UR order.
UR5_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

ACTION_NAME = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
JOINT_STATES_TOPIC = "/joint_states"


class UR5GazeboArm(RobotArm):
    """Concrete RobotArm for UR5 running in Gazebo via ros2_control."""

    def __init__(
        self,
        urdf_path: str = "~/ros2_ws/urdf/ur5.urdf",
    ):
        self._node: Optional[Node] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._action_client: Optional[ActionClient] = None
        self._latest_positions: Optional[List[float]] = None
        self._lock = threading.Lock()

        # Kinematics — used by move_to_xyz().
        self._kin = RobotKinematics(urdf_path, UR5_JOINT_NAMES)

    @property
    def num_joints(self) -> int:
        return 6

    # ---------- Connection lifecycle ----------

    def connect(self) -> None:
        if not rclpy.ok():
            rclpy.init()

        self._node = Node("ur5_gazebo_arm_client")

        # Subscribe to joint states so we always know where the arm is.
        self._node.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self._joint_state_callback,
            10,
        )

        # Action client to send trajectory goals.
        self._action_client = ActionClient(
            self._node,
            FollowJointTrajectory,
            ACTION_NAME,
        )

        # Spin the node in a background thread so callbacks fire.
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True
        )
        self._spin_thread.start()

        # Wait for the action server to come up.
        self._node.get_logger().info("Waiting for action server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                f"Action server '{ACTION_NAME}' not available. "
                "Is the UR5 simulation running?"
            )
        self._node.get_logger().info("Action server connected.")

        # Wait until we get at least one joint_states message.
        deadline = time.time() + 5.0
        while self._latest_positions is None and time.time() < deadline:
            time.sleep(0.05)
        if self._latest_positions is None:
            raise RuntimeError("No /joint_states received in 5s.")

    def disconnect(self) -> None:
        """
        Clean shutdown sequence.

        Bug history: previously we just called executor.shutdown() and
        rclpy.shutdown() without waiting for the spin thread to actually
        exit, which caused 'terminate called without an active exception
        / Aborted (core dumped)' on script exit. The C++ side aborted
        because Python destroyed the executor while it was still spinning.

        Fix: explicitly shut down the executor, JOIN the spin thread with
        a timeout, then destroy the node, then shutdown rclpy. Order matters.
        """
        try:
            if self._action_client is not None:
                self._action_client.destroy()
                self._action_client = None
        except Exception:
            pass

        if self._executor is not None:
            try:
                self._executor.shutdown()
            except Exception:
                pass

        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
            self._spin_thread = None

        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    # ---------- Subscriber callback ----------

    def _joint_state_callback(self, msg: JointState) -> None:
        """
        /joint_states may arrive with joints in ANY order, so we must
        reorder by name into the UR5_JOINT_NAMES canonical order.
        """
        try:
            name_to_pos = dict(zip(msg.name, msg.position))
            ordered = [name_to_pos[n] for n in UR5_JOINT_NAMES]
        except KeyError:
            # Message didn't contain all UR5 joints (could be a partial pub).
            return
        with self._lock:
            self._latest_positions = ordered

    # ---------- RobotArm API ----------

    def get_current_positions(self) -> JointPositions:
        with self._lock:
            if self._latest_positions is None:
                raise RuntimeError("No joint state received yet.")
            return JointPositions(values=list(self._latest_positions))

    def move_to_joint_positions(
        self,
        target: JointPositions,
        duration_sec: float = 3.0,
    ) -> bool:
        if len(target) != self.num_joints:
            raise ValueError(
                f"Expected {self.num_joints} joint values, got {len(target)}"
            )

        # Build a single-point trajectory: "be at `target` after duration_sec".
        point = JointTrajectoryPoint()
        point.positions = list(target.values)
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec - int(duration_sec)) * 1e9),
        )

        traj = JointTrajectory()
        traj.joint_names = UR5_JOINT_NAMES
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        # Send goal and wait for it to be accepted.
        send_future = self._action_client.send_goal_async(goal)
        while not send_future.done():
            time.sleep(0.01)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self._node.get_logger().error("Goal rejected by controller.")
            return False

        # Wait for execution to finish.
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.01)

        result = result_future.result().result
        return result.error_code == 0

    # ---------- Cartesian motion via IK ----------

    def move_to_xyz(
        self,
        x: float,
        y: float,
        z: float,
        duration_sec: float = 3.0,
    ) -> bool:
        """
        Move the flange to a target XYZ in base_link frame.
        Solves IK using the current joint positions as the seed,
        then dispatches the result through move_to_joint_positions().
        """
        # Use current joints as the IK seed — gives a smooth, close solution.
        current = self.get_current_positions()
        target_joints_rad = self._kin.inverse_position(
            target_xyz=[x, y, z],
            seed_joints_rad=current.values,
        )

        # Sanity check: did IK actually land near the target?
        achieved = self._kin.forward_position(target_joints_rad)
        err_mm = float(np.linalg.norm(np.array(achieved) - np.array([x, y, z]))) * 1000
        if err_mm > 5.0:
            self._node.get_logger().warning(
                f"IK residual {err_mm:.1f} mm — target may be unreachable. "
                f"Requested ({x:.3f}, {y:.3f}, {z:.3f}), got "
                f"({achieved[0]:.3f}, {achieved[1]:.3f}, {achieved[2]:.3f})."
            )

        # Execute the joint trajectory using the method we already trust.
        return self.move_to_joint_positions(
            JointPositions(values=target_joints_rad),
            duration_sec=duration_sec,
        )