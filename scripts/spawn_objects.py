"""
Spawn objects into a running Gazebo simulation via the create service.

Uses subprocess to call the `gz service` command-line tool — avoids the
need for Python bindings to gz-transport. Slower (~few hundred ms per spawn)
but dependency-free.

Usage:
    spawner = WorldSpawner(world_name="empty")
    spawner.spawn_table(x=0.5, y=0.0, top_height=0.4)
    spawner.spawn_cup(x=0.5, y=0.0, table_top_height=0.4)
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
        - (x, y) is the cup's location on the table.
        - table_top_height is where the table top sits; the cup is placed on it.
        - radius and height define the cup shape.
        - mass is in kg.
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