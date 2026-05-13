"""
Demo: exercise each joint independently.

What it does:
1. Connect to the UR5 simulation.
2. Go to the READY pose (a safe starting point).
3. For each joint in order, rotate it +45 deg, back to 0, -45 deg, back to 0.
4. Return to HOME and disconnect.

Why this matters:
This is the most basic "is my robot alive?" test in robotics — drive each
joint individually and confirm it responds. It also catches setup bugs:
wrong joint order, wrong sign convention, wrong units.
"""

import math
import time

from ur5_gazebo_arm import UR5GazeboArm
from joint_configs import UR5_READY, UR5_HOME


def main():
    # `with` ensures connect() runs at start and disconnect() runs at end,
    # even if an exception is raised.
    with UR5GazeboArm() as arm:
        print(f"Connected. Robot has {arm.num_joints} joints.")

        print("\n-> Moving to READY pose...")
        arm.move_to_joint_positions(UR5_READY, duration_sec=4.0)
        time.sleep(0.5)

        # Read where we are now — this becomes our baseline.
        baseline = arm.get_current_positions()
        print(f"Baseline (deg): {[round(d, 1) for d in baseline.to_degrees()]}")

        # Wiggle each joint ±45 degrees from the baseline.
        delta = math.radians(45)
        for joint_idx in range(arm.num_joints):
            print(f"\n-> Joint {joint_idx}: +45 deg")
            arm.move_single_joint(
                joint_idx,
                baseline.values[joint_idx] + delta,
                duration_sec=2.0,
            )
            time.sleep(0.3)

            print(f"-> Joint {joint_idx}: back to baseline")
            arm.move_single_joint(
                joint_idx,
                baseline.values[joint_idx],
                duration_sec=2.0,
            )
            time.sleep(0.3)

            print(f"-> Joint {joint_idx}: -45 deg")
            arm.move_single_joint(
                joint_idx,
                baseline.values[joint_idx] - delta,
                duration_sec=2.0,
            )
            time.sleep(0.3)

            print(f"-> Joint {joint_idx}: back to baseline")
            arm.move_single_joint(
                joint_idx,
                baseline.values[joint_idx],
                duration_sec=2.0,
            )
            time.sleep(0.3)

        print("\n-> Returning to HOME...")
        arm.move_to_joint_positions(UR5_HOME, duration_sec=4.0)
        print("Done.")


if __name__ == "__main__":
    main()