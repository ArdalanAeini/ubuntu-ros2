"""
Demo: cycle through named poses.

What it does:
1. Connect.
2. Go to HOME -> READY -> PICK_HOVER -> READY -> TUCKED -> HOME.
3. Disconnect.

Why this matters:
Real robot programs are built from named poses, not raw joint numbers.
This is what a real pick-and-place sequence will look like at the
top level: a list of named waypoints. The interesting work (planning,
sensing, gripping) plugs in between them.
"""

import time

from ur5_gazebo_arm import UR5GazeboArm
from joint_configs import UR5_HOME, UR5_READY, UR5_PICK_HOVER, UR5_TUCKED


# The sequence of poses to visit. Each entry: (name, JointPositions, seconds).
SEQUENCE = [
    ("HOME",       UR5_HOME,       4.0),
    ("READY",      UR5_READY,      4.0),
    ("PICK_HOVER", UR5_PICK_HOVER, 4.0),
    ("READY",      UR5_READY,      3.0),
    ("TUCKED",     UR5_TUCKED,     5.0),
    ("HOME",       UR5_HOME,       4.0),
]


def main():
    with UR5GazeboArm() as arm:
        print(f"Connected. Robot has {arm.num_joints} joints.\n")

        for name, pose, duration in SEQUENCE:
            print(f"-> Moving to {name} (over {duration}s)")
            ok = arm.move_to_joint_positions(pose, duration_sec=duration)
            if not ok:
                print(f"   FAILED at {name}, aborting.")
                break
            time.sleep(0.5)

        print("\nSequence complete.")


if __name__ == "__main__":
    main()