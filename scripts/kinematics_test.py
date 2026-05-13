"""
Sanity test for the kinematics module — no robot needed.

Round-trip test: pick joint angles, compute FK to get pose,
then run IK on that pose and check we recover similar joints.
"""

import os
import numpy as np
from kinematics import RobotKinematics

URDF_PATH = "~/ros2_ws/urdf/ur5.urdf"

UR5_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def main():
    kin = RobotKinematics(URDF_PATH, UR5_JOINTS)
    print(f"Loaded UR5 kinematics. {kin.num_joints} joints.\n")

    # Pick a sensible test pose (READY).
    test_joints = [0.0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0]
    print(f"Test joints (deg): "
          f"{[round(np.degrees(j), 1) for j in test_joints]}")

    # FK: what XYZ does this produce?
    target_xyz = kin.forward_position(test_joints)
    print(f"FK gives XYZ: "
          f"x={target_xyz[0]:+.3f}, y={target_xyz[1]:+.3f}, z={target_xyz[2]:+.3f}")

    # IK: ask for that XYZ back, seeded with the same joints.
    # If everything works, we should recover (close to) the original joints.
    solved = kin.inverse_position(target_xyz.tolist(), seed_joints_rad=test_joints)
    print(f"\nIK solution (deg): "
          f"{[round(np.degrees(j), 1) for j in solved]}")

    # Verify by running FK on the IK solution.
    achieved = kin.forward_position(solved)
    err = np.linalg.norm(achieved - target_xyz)
    print(f"FK of IK solution: "
          f"x={achieved[0]:+.3f}, y={achieved[1]:+.3f}, z={achieved[2]:+.3f}")
    print(f"Position error: {err*1000:.2f} mm")

    # Try a fresh target — somewhere reachable.
    print("\n--- Fresh target ---")
    fresh = [0.5, 0.0, 0.3]  # 50cm forward, centered, 30cm up
    print(f"Target XYZ: {fresh}")
    solved2 = kin.inverse_position(fresh, seed_joints_rad=test_joints)
    print(f"IK solution (deg): "
          f"{[round(np.degrees(j), 1) for j in solved2]}")
    achieved2 = kin.forward_position(solved2)
    print(f"Achieved XYZ: "
          f"x={achieved2[0]:+.3f}, y={achieved2[1]:+.3f}, z={achieved2[2]:+.3f}")
    err2 = np.linalg.norm(np.array(achieved2) - np.array(fresh)) * 1000
    print(f"Position error: {err2:.2f} mm")


if __name__ == "__main__":
    main()