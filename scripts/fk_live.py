"""
Live Forward Kinematics monitor.

Connects to the running UR5 simulation and continuously prints the
flange's XYZ position in the base_link frame, recomputed every time
new joint states arrive.

Run this in one terminal, then run demo_named_poses.py in another.
Watch the XYZ change as the arm moves.
"""

import os
import time
import numpy as np

from ur5_gazebo_arm import UR5GazeboArm
from ikpy.chain import Chain

URDF_PATH = os.path.expanduser("~/ros2_ws/urdf/ur5.urdf")

UR5_REVOLUTE_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def build_chain():
    """Load URDF with exactly the 6 UR5 revolute joints marked active."""
    chain = Chain.from_urdf_file(URDF_PATH, base_elements=["base_link"])
    mask = [link.name in UR5_REVOLUTE_JOINTS for link in chain.links]
    return Chain.from_urdf_file(
        URDF_PATH,
        base_elements=["base_link"],
        active_links_mask=mask,
    )


def joint_vector(chain, radians_6):
    """Expand 6 joint angles into the full link vector ikpy expects."""
    full = [0.0] * len(chain.links)
    movable_idx = 0
    for i, is_active in enumerate(chain.active_links_mask):
        if is_active:
            full[i] = radians_6[movable_idx]
            movable_idx += 1
    return full


def main():
    chain = build_chain()
    print("Chain loaded. Connecting to robot...")

    with UR5GazeboArm() as arm:
        print("Connected. Press Ctrl+C to stop.\n")
        print(f"{'joint angles (deg)':<55}  {'flange XYZ (m)'}")
        print("-" * 90)

        try:
            while True:
                # Read current joints (radians).
                current = arm.get_current_positions()

                # Compute FK.
                fk = chain.forward_kinematics(joint_vector(chain, current.values))
                x, y, z = fk[:3, 3]

                # Display joints in degrees for readability.
                degs = current.to_degrees()
                deg_str = "[" + ", ".join(f"{d:+6.1f}" for d in degs) + "]"

                # \r at start + end='' overwrites the same line.
                print(f"\r{deg_str}  ->  "
                      f"x={x:+.3f}  y={y:+.3f}  z={z:+.3f}",
                      end="", flush=True)

                time.sleep(0.1)  # 10 Hz update is plenty
        except KeyboardInterrupt:
            print("\n\nStopped.")


if __name__ == "__main__":
    main()