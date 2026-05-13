"""
Demo: command the UR5 to XYZ points in space using inverse kinematics.

The arm traces a triangle in the air above its base, then returns home.
Each waypoint is described in (x, y, z) meters in base_link frame —
no joint angles in this script.

Run the UR5 simulation first:
  ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5
"""

import time

from ur5_gazebo_arm import UR5GazeboArm
from joint_configs import UR5_READY, UR5_HOME


# Triangle in space. Each: (label, x, y, z, seconds_to_move_there).
# Coordinates are in base_link frame: +X forward, +Y left, +Z up.
WAYPOINTS = [
    ("Apex (forward, centered, high)",   0.45,  0.00, 0.45, 4.0),
    ("Right corner",                     0.45, -0.25, 0.25, 4.0),
    ("Left corner",                      0.45,  0.25, 0.25, 4.0),
    ("Back to apex",                     0.45,  0.00, 0.45, 4.0),
]


def main():
    with UR5GazeboArm() as arm:
        print(f"Connected. {arm.num_joints} joints.\n")

        # Start from READY — gives IK a good seed.
        print("-> Moving to READY (joint-space start pose)")
        arm.move_to_joint_positions(UR5_READY, duration_sec=4.0)
        time.sleep(0.5)

        # Walk the triangle.
        for label, x, y, z, duration in WAYPOINTS:
            print(f"-> XYZ target: {label}")
            print(f"   ({x:+.2f}, {y:+.2f}, {z:+.2f}) over {duration}s")
            ok = arm.move_to_xyz(x, y, z, duration_sec=duration)
            if not ok:
                print(f"   FAILED at {label}, aborting.")
                break
            time.sleep(0.5)

        print("\n-> Returning to HOME")
        arm.move_to_joint_positions(UR5_HOME, duration_sec=4.0)
        print("Done.")


if __name__ == "__main__":
    main()