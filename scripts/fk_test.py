"""
Forward Kinematics test — does ikpy understand our UR5 URDF?
"""

import os
import numpy as np
from ikpy.chain import Chain

URDF_PATH = os.path.expanduser("~/ros2_ws/urdf/ur5.urdf")

# The 6 UR5 revolute joints, in canonical order.
UR5_REVOLUTE_LINKS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def build_chain():
    """Load the URDF and mark exactly the 6 UR5 revolute joints as active."""
    # First load with everything active to inspect what ikpy saw.
    chain = Chain.from_urdf_file(URDF_PATH, base_elements=["base_link"])

    # Now build an explicit active_links_mask: True only for our 6 revolutes.
    mask = [link.name in UR5_REVOLUTE_LINKS for link in chain.links]

    # Reload with our explicit mask.
    chain = Chain.from_urdf_file(
        URDF_PATH,
        base_elements=["base_link"],
        active_links_mask=mask,
    )
    return chain


def joint_vector_from_radians(chain, radians_6):
    """Build the full link vector ikpy needs, given 6 joint angles."""
    full = [0.0] * len(chain.links)
    movable_idx = 0
    for i, is_active in enumerate(chain.active_links_mask):
        if is_active:
            full[i] = radians_6[movable_idx]
            movable_idx += 1
    return full


def print_pose(label, fk_matrix):
    pos = fk_matrix[:3, 3]
    print(f"\n=== {label} ===")
    print(f"Flange position: x={pos[0]:+.3f} m, "
          f"y={pos[1]:+.3f} m, z={pos[2]:+.3f} m")


def main():
    print(f"Loading URDF from: {URDF_PATH}\n")
    chain = build_chain()

    print("=== Chain structure ===")
    for i, link in enumerate(chain.links):
        marker = "  [movable]" if chain.active_links_mask[i] else "  [fixed]"
        print(f"  {i:2d}: {link.name}{marker}")
    print(f"\nTotal links: {len(chain.links)}")
    print(f"Movable joints: {sum(chain.active_links_mask)}")

    # FK at all-zeros
    zeros = [0.0] * 6
    fk = chain.forward_kinematics(joint_vector_from_radians(chain, zeros))
    print_pose("FK at all-zeros joint configuration", fk)

    # FK at READY pose
    ready = [0.0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0]
    fk = chain.forward_kinematics(joint_vector_from_radians(chain, ready))
    print_pose("FK at READY pose [0, -90, 90, -90, -90, 0] deg", fk)

    # FK at PICK_HOVER
    hover = [0.0, np.radians(-60), np.radians(110),
             np.radians(-140), np.radians(-90), 0.0]
    fk = chain.forward_kinematics(joint_vector_from_radians(chain, hover))
    print_pose("FK at PICK_HOVER pose", fk)


if __name__ == "__main__":
    main()