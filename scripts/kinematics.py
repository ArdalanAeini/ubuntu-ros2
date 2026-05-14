"""
Kinematics module — robot-independent FK and IK

Any robot whose geometry is described by a URDF can use this module.
The robot-specific bits (URDF path, joint names) get passed in via the
RobotKinematics constructor.

Usage:
    kin = RobotKinematics(urdf_path, revolute_joint_names)
    pose = kin.forward(joint_angles_rad)        # 6 joints -> 4x4 matrix
    joints = kin.inverse_position(target_xyz, seed_joints_rad)
"""

import os
import numpy as np
from typing import List, Optional
from ikpy.chain import Chain


class RobotKinematics:
    """
    FK + IK for any robot described by a URDF.

    Pass in the URDF path and the canonical list of revolute joint names
    in the order you want angles to be specified.
    """

    def __init__(
        self,
        urdf_path: str,
        revolute_joint_names: List[str],
        base_element: str = "base_link",
    ):
        self.urdf_path = os.path.expanduser(urdf_path)
        self.joint_names = list(revolute_joint_names)
        self.num_joints = len(revolute_joint_names)

        # Two-step load: first to inspect link names, then with the mask.
        probe = Chain.from_urdf_file(self.urdf_path, base_elements=[base_element])
        mask = [link.name in revolute_joint_names for link in probe.links]

        if sum(mask) != self.num_joints:
            found = [link.name for link, m in zip(probe.links, mask) if m]
            raise ValueError(
                f"Expected {self.num_joints} revolute joints, "
                f"found {sum(mask)} in URDF: {found}. "
                f"Check that revolute_joint_names match URDF joint names."
            )

        self.chain = Chain.from_urdf_file(
            self.urdf_path,
            base_elements=[base_element],
            active_links_mask=mask,
        )
        self._mask = mask

    # ---------- helpers ----------

    def _to_full_vector(self, joints_6: List[float]) -> List[float]:
        """Expand the 6 joint angles into ikpy's full link vector."""
        if len(joints_6) != self.num_joints:
            raise ValueError(
                f"Expected {self.num_joints} joint values, got {len(joints_6)}"
            )
        full = [0.0] * len(self.chain.links)
        movable_idx = 0
        for i, is_active in enumerate(self._mask):
            if is_active:
                full[i] = joints_6[movable_idx]
                movable_idx += 1
        return full

    def _from_full_vector(self, full_vector: List[float]) -> List[float]:
        """Extract just the 6 movable joint angles from ikpy's full vector."""
        return [v for v, m in zip(full_vector, self._mask) if m]

    # ---------- Forward Kinematics ----------

    def forward(self, joints_rad: List[float]) -> np.ndarray:
        """
        Compute end-effector pose for given joint angles (radians).
        Returns a 4x4 homogeneous transformation matrix.
        """
        full = self._to_full_vector(joints_rad)
        return self.chain.forward_kinematics(full)

    def forward_position(self, joints_rad: List[float]) -> np.ndarray:
        """Like forward(), but returns just the XYZ position (3-vector)."""
        return self.forward(joints_rad)[:3, 3]

    # ---------- Inverse Kinematics ----------

    def inverse_position(
        self,
        target_xyz: List[float],
        seed_joints_rad: Optional[List[float]] = None,
    ) -> List[float]:
        """
        Solve IK for a target XYZ position (orientation unconstrained).

        target_xyz:        [x, y, z] in meters, in base_link frame.
        seed_joints_rad:   starting guess for the iterative solver.
                           Pass the current joint angles for best results.
                           Defaults to all zeros.

        Returns 6 joint angles in radians.
        """
        if seed_joints_rad is None:
            seed_joints_rad = [0.0] * self.num_joints

        initial = self._to_full_vector(seed_joints_rad)

        solution_full = self.chain.inverse_kinematics(
            target_position=target_xyz,
            initial_position=initial,
        )
        return self._from_full_vector(solution_full)