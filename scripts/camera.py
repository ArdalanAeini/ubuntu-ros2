"""
ROS 2 subscriber that captures the latest frame from a camera topic.

Architecture:
    Gazebo publishes images on a gz-transport topic. The ros_gz_image bridge
    relays them to a ROS 2 sensor_msgs/Image topic. This class subscribes
    to that ROS 2 topic and exposes the latest frame as a NumPy array.

    Same pattern as UR5GazeboArm: an rclpy executor spins in a daemon
    thread, the subscriber callback writes the latest frame under a lock,
    the main thread reads via get_latest_frame().

Usage:
    with CameraSubscriber() as cam:
        frame = cam.get_latest_frame()   # numpy array, shape (H, W, 3), BGR
        cv2.imshow("camera", frame)
"""

import threading
import time
import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image


class CameraSubscriber:
    """Subscribes to a sensor_msgs/Image topic and caches the latest frame."""

    def __init__(self, topic: str = "/camera/image"):
        self.topic = topic
        self._node: Optional[Node] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._latest_frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        # Cache for diagnostics
        self._frame_count = 0

    # ---------- Connection lifecycle ----------

    def connect(self, wait_for_first_frame_sec: float = 10.0) -> None:
        """Spin up rclpy, create subscription, wait for the first frame."""
        # Init rclpy only if this is the first user. If UR5GazeboArm already
        # called rclpy.init(), this is a no-op.
        if not rclpy.ok():
            rclpy.init()

        self._node = Node("camera_subscriber")
        self._node.create_subscription(
            Image,
            self.topic,
            self._image_callback,
            10,
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True
        )
        self._spin_thread.start()

        # Block until first frame arrives (so the user doesn't immediately
        # get None from get_latest_frame()).
        self._node.get_logger().info(
            f"Waiting for first frame on {self.topic}..."
        )
        deadline = time.time() + wait_for_first_frame_sec
        while self._latest_frame is None and time.time() < deadline:
            time.sleep(0.05)
        if self._latest_frame is None:
            raise RuntimeError(
                f"No image received on {self.topic} within "
                f"{wait_for_first_frame_sec}s. Is the bridge running?\n"
                "  ros2 run ros_gz_image image_bridge /camera/image"
            )
        self._node.get_logger().info(
            f"Camera connected. Frame shape: {self._latest_frame.shape}"
        )

    def disconnect(self) -> None:
        """Mirror of UR5GazeboArm.disconnect() — clean shutdown order."""
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
        # Note: we DON'T call rclpy.shutdown() here — another component
        # (UR5GazeboArm) may still be using it.

    # ---------- Subscriber callback ----------

    def _image_callback(self, msg: Image) -> None:
        """Convert sensor_msgs/Image to a NumPy array and cache it."""
        # sensor_msgs/Image carries raw bytes. We need to know:
        #   - height x width: dimensions
        #   - encoding: pixel format string (e.g. 'rgb8', 'bgr8')
        #   - data: the byte buffer
        # We reshape and convert RGB -> BGR for OpenCV compatibility.
        try:
            if msg.encoding == "rgb8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
                # OpenCV uses BGR. Gazebo's R8G8B8 sensor format publishes
                # as 'rgb8'. Swap channels so cv2.imshow renders correct colors.
                arr = arr[:, :, ::-1].copy()
            elif msg.encoding == "bgr8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                ).copy()
            else:
                # Fallback: don't try to interpret unknown encodings.
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, -1
                ).copy()
        except Exception as e:
            self._node.get_logger().warning(f"Failed to parse image: {e}")
            return

        with self._lock:
            self._latest_frame = arr
            self._frame_count += 1

    # ---------- Public API ----------

    def get_latest_frame(self) -> np.ndarray:
        """Return the most recent frame as a (H, W, 3) BGR uint8 array."""
        with self._lock:
            if self._latest_frame is None:
                raise RuntimeError("No frame received yet.")
            return self._latest_frame.copy()

    def get_frame_count(self) -> int:
        """How many frames we've received so far (useful for fps measurement)."""
        with self._lock:
            return self._frame_count

    # ---------- Context manager ----------

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False