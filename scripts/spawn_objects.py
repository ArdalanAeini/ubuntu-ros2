"""
Spawn objects into a running Gazebo simulation via the create service.

Uses subprocess to call the `gz service` command-line tool — avoids the
need for Python bindings to gz-transport. Slower (~few hundred ms per spawn)
but dependency-free.

Usage:
    spawner = WorldSpawner(world_name="empty")
    spawner.spawn_table(x=0.5, y=0.0, top_height=0.4)
    spawner.spawn_cup(x=0.5, y=0.0, table_top_height=0.4)
    # OR for pick-and-place:
    spawner.spawn_graspable_cup(x=0.5, y=0.0, table_top_height=0.4)
    spawner.spawn_gripper(wrist_world_pose=(0.0, 0.191, 1.001, -1.5708, 0, 0))
"""

import subprocess
from typing import Tuple


class WorldSpawner:
    """Spawns SDF objects into a running Gazebo world."""

    def __init__(self, world_name: str = "empty"):
        self.world_name = world_name
        self.service = f"/world/{world_name}/create"

    # ---------- low-level: send an SDF string to the create service ----------

    def _spawn_sdf(self, sdf_xml: str, timeout_ms: int = 3000) -> bool:
        """Call /world/<name>/create with an inline SDF model. Returns True on success."""
        request = f'sdf: "{self._escape(sdf_xml)}"'
        result = subprocess.run(
            [
                "gz", "service",
                "-s", self.service,
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", str(timeout_ms),
                "--req", request,
            ],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            print(f"[spawn] FAILED: {result.stderr.strip()}")
            return False
        success = "data: true" in result.stdout
        if not success:
            print(f"[spawn] service returned: {result.stdout.strip()}")
        return success

    @staticmethod
    def _escape(sdf_xml: str) -> str:
        """Escape double quotes for embedding inside the request string."""
        return sdf_xml.replace('"', '\\"')

    @staticmethod
    def _flatten(sdf_xml: str) -> str:
        """Collapse multi-line SDF into a single line, stripping extra whitespace."""
        return " ".join(line.strip() for line in sdf_xml.splitlines() if line.strip())

    # ---------- high-level: typed object spawners ----------

    def spawn_table(
        self,
        x: float = 0.5,
        y: float = 0.0,
        top_height: float = 0.4,
        size_xy: Tuple[float, float] = (0.6, 0.6),
        name: str = "table",
    ) -> bool:
        """
        Spawn a rectangular table.
          - (x, y) is the center of the table in world coords.
          - top_height is the height of the top surface (Z of the top face).
          - size_xy is the table top (width, depth).
        """
        cx, cy, cz = x, y, top_height / 2.0
        sx, sy = size_xy
        sz = top_height

        sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <pose>{cx} {cy} {cz} 0 0 0</pose>
    <link name="link">
      <collision name="c">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
      <visual name="v">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>0.55 0.35 0.18 1</ambient>
          <diffuse>0.55 0.35 0.18 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        return self._spawn_sdf(self._flatten(sdf))

    def spawn_cup(
        self,
        x: float = 0.5,
        y: float = 0.0,
        table_top_height: float = 0.4,
        radius: float = 0.035,
        height: float = 0.10,
        mass: float = 0.05,
        name: str = "cup",
    ) -> bool:
        """
        Spawn a simple cup (modeled as a solid cylinder for now).
        Non-graspable: no plugin attached. For pick-and-place use
        spawn_graspable_cup() instead.
        """
        cz = table_top_height + height / 2.0
        ixx = mass * (3 * radius ** 2 + height ** 2) / 12.0
        izz = mass * radius ** 2 / 2.0

        sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <pose>{x} {y} {cz} 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx>
          <iyy>{ixx}</iyy>
          <izz>{izz}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="c">
        <geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>
      </collision>
      <visual name="v">
        <geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>
        <material>
          <ambient>0.85 0.85 0.95 1</ambient>
          <diffuse>0.85 0.85 0.95 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        return self._spawn_sdf(self._flatten(sdf))

    def spawn_graspable_cup(
        self,
        x: float = 0.5,
        y: float = 0.0,
        table_top_height: float = 0.4,
        radius: float = 0.035,
        height: float = 0.10,
        mass: float = 0.05,
        name: str = "cup",
        robot_model: str = "ur",
        robot_attach_link: str = "wrist_3_link",
        attach_topic: str = "/cup/attach",
        detach_topic: str = "/cup/detach",
    ) -> bool:
        """
        Spawn a cup with a DetachableJoint plugin so it can be 'grasped'.

        NOTE: The DetachableJoint plugin starts in the ATTACHED state at spawn
        time. The cup will get yanked to the robot's wrist immediately. To
        recover, publish to detach_topic right after spawn (see grasp.py's
        prepare_cup_for_pickup()).
        """
        cz = table_top_height + height / 2.0
        ixx = mass * (3 * radius ** 2 + height ** 2) / 12.0
        izz = mass * radius ** 2 / 2.0

        sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <pose>{x} {y} {cz} 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx>
          <iyy>{ixx}</iyy>
          <izz>{izz}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="c">
        <geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>
      </collision>
      <visual name="v">
        <geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>
        <material>
          <ambient>0.85 0.85 0.95 1</ambient>
          <diffuse>0.85 0.85 0.95 1</diffuse>
        </material>
      </visual>
    </link>
    <plugin filename="gz-sim-detachable-joint-system"
            name="gz::sim::systems::DetachableJoint">
      <parent_link>link</parent_link>
      <child_model>{robot_model}</child_model>
      <child_link>{robot_attach_link}</child_link>
      <attach_topic>{attach_topic}</attach_topic>
      <detach_topic>{detach_topic}</detach_topic>
      <suppress_child_warning>true</suppress_child_warning>
    </plugin>
  </model>
</sdf>"""
        return self._spawn_sdf(self._flatten(sdf))

    def spawn_gripper(
        self,
        wrist_world_pose: Tuple[float, float, float, float, float, float] = (
            0.0, 0.191, 1.001, -1.5708, 0.0, 0.0
        ),
        name: str = "gripper",
        robot_model: str = "ur",
        robot_attach_link: str = "wrist_3_link",
        attach_topic: str = "/gripper/attach",
        finger_gap: float = 0.080,
        finger_length: float = 0.080,
    ) -> bool:
        """
        Spawn a simple visual gripper (parallel-jaw style) and attach it
        permanently to the robot's wrist_3_link via DetachableJoint.

        Geometry:
            - Base plate (40x40x15 mm) at the wrist flange.
            - Two parallel finger blocks hanging below, spaced `finger_gap`
              apart inner-to-inner, each `finger_length` long.

        Frame alignment (this is the tricky bit):
            wrist_3_link's local +Y points along world +Z when the arm is in
            UR5_HOME. So to make the fingers hang DOWN in world frame, we
            spawn the gripper rotated +90° about the wrist's local X axis.
            That puts the gripper's local -Z (fingers extending downward in
            gripper-local frame) along world -Z (down).

        We pass `wrist_world_pose` as (x, y, z, roll, pitch, yaw) so the
        gripper spawns AT the wrist, allowing DetachableJoint to weld it in
        place at zero relative offset.

        The plugin starts ATTACHED (same behavior as the cup). For the
        gripper that's actually what we want — it should stay attached
        for the entire simulation. We never publish to a detach topic.
        """
        wx, wy, wz, wr, wp, wyaw = wrist_world_pose

        # Geometry constants (all in metres) -----------------------------
        plate_x, plate_y, plate_z = 0.040, 0.040, 0.015  # base plate
        shoulder_x, shoulder_y, shoulder_z = 0.060, 0.020, 0.020  # spans the plate
        finger_x, finger_y = 0.010, 0.015  # cross-section of each finger
        # Half-gap so each finger sits `finger_gap/2` from the centerline.
        half_gap = finger_gap / 2.0
        finger_inner_offset = half_gap + finger_x / 2.0  # center of finger block from centerline

        # In gripper-local frame, +Z points "into" the wrist (up at home pose),
        # so the fingers extend in -Z. Plate sits at z=0 (the attach face).
        plate_cz = -plate_z / 2.0
        shoulder_cz = -plate_z - shoulder_z / 2.0
        finger_cz = -plate_z - shoulder_z - finger_length / 2.0

        sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <pose>{wx} {wy} {wz} {wr} {wp} {wyaw}</pose>
    <link name="link">
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.0005</ixx>
          <iyy>0.0005</iyy>
          <izz>0.0005</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>

      <!-- Base plate (mounts to the wrist flange) -->
      <visual name="plate_v">
        <pose>0 0 {plate_cz} 0 0 0</pose>
        <geometry><box><size>{plate_x} {plate_y} {plate_z}</size></box></geometry>
        <material>
          <ambient>0.25 0.25 0.28 1</ambient>
          <diffuse>0.25 0.25 0.28 1</diffuse>
        </material>
      </visual>

      <!-- Shoulders span the gap between the fingers -->
      <visual name="shoulder_v">
        <pose>0 0 {shoulder_cz} 0 0 0</pose>
        <geometry><box><size>{shoulder_x} {shoulder_y} {shoulder_z}</size></box></geometry>
        <material>
          <ambient>0.2 0.2 0.22 1</ambient>
          <diffuse>0.2 0.2 0.22 1</diffuse>
        </material>
      </visual>

      <!-- Left finger (-X side) -->
      <visual name="finger_left_v">
        <pose>{-finger_inner_offset} 0 {finger_cz} 0 0 0</pose>
        <geometry><box><size>{finger_x} {finger_y} {finger_length}</size></box></geometry>
        <material>
          <ambient>0.15 0.15 0.18 1</ambient>
          <diffuse>0.15 0.15 0.18 1</diffuse>
        </material>
      </visual>

      <!-- Right finger (+X side) -->
      <visual name="finger_right_v">
        <pose>{finger_inner_offset} 0 {finger_cz} 0 0 0</pose>
        <geometry><box><size>{finger_x} {finger_y} {finger_length}</size></box></geometry>
        <material>
          <ambient>0.15 0.15 0.18 1</ambient>
          <diffuse>0.15 0.15 0.18 1</diffuse>
        </material>
      </visual>

      <!--
        NOTE: NO <collision> elements on the gripper.
        Reason: we don't want the gripper colliding with the cup or the
        table during descent. The DetachableJoint handles the grasp; we
        don't need real contact physics. Adding collisions would cause
        the fingers to push the cup around and ruin the visual.
      -->
    </link>

    <plugin filename="gz-sim-detachable-joint-system"
            name="gz::sim::systems::DetachableJoint">
      <parent_link>link</parent_link>
      <child_model>{robot_model}</child_model>
      <child_link>{robot_attach_link}</child_link>
      <attach_topic>{attach_topic}</attach_topic>
      <suppress_child_warning>true</suppress_child_warning>
    </plugin>
  </model>
</sdf>"""
        return self._spawn_sdf(self._flatten(sdf))