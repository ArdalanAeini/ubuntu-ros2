"""
Blind pick-and-place demo for the UR5 in Gazebo (no visual gripper).

Scene setup:
    1. Spawn table.
    2. Spawn graspable cup; immediately detach so it falls onto the table.
    3. Run the 9-phase pick-and-place choreography.

The cup snaps to wrist_3_link directly when grasped (kinematic attach via
DetachableJoint). No physical gripper; we'll add a real one when the
JetCoBot hardware arrives.
"""

import time

from ur5_gazebo_arm import UR5GazeboArm
from spawn_objects import WorldSpawner
from grasp import attach_cup, detach_cup, prepare_cup_for_pickup
from joint_configs import UR5_HOME


# ----- Scene geometry -----
TABLE_TOP_HEIGHT = 0.4
CUP_RADIUS       = 0.035
CUP_HEIGHT       = 0.10

PICK_X,  PICK_Y  = 0.5,  0.0      # cup starts here
PLACE_X, PLACE_Y = 0.5, -0.25     # cup ends up 25cm to the right

# ----- Derived heights (no gripper, so wrist_3 itself is the grasp point) -----
CUP_TOP_Z      = TABLE_TOP_HEIGHT + CUP_HEIGHT       # 0.5 m
GRASP_Z        = CUP_TOP_Z - 0.02                    # 0.48 m — wrist sits 2cm down into cup
HOVER_Z        = CUP_TOP_Z + 0.15                    # 0.65 m — 15cm above cup top
PLACE_RELEASE_Z = TABLE_TOP_HEIGHT + 0.05            # 0.45 m — release 5cm above table

# ----- Motion timing -----
MOVE_SLOW = 3.5
MOVE_FAST = 2.0
DWELL_AT_GRASP = 0.5


def ensure_scene():
    """Spawn table and graspable cup; settle the cup on the table."""
    spawner = WorldSpawner(world_name="empty")

    print("Spawning table...")
    spawner.spawn_table(
        x=PICK_X, y=0.0, top_height=TABLE_TOP_HEIGHT, name="table"
    )

    print("Spawning graspable cup...")
    ok = spawner.spawn_graspable_cup(
        x=PICK_X, y=PICK_Y, table_top_height=TABLE_TOP_HEIGHT,
        radius=CUP_RADIUS, height=CUP_HEIGHT, name="cup",
    )
    if not ok:
        print(
            "WARN: graspable cup spawn returned False.\n"
            "      Remove old 'cup' from Entity Tree and re-run."
        )
        return

    # Cup teleported to wrist at spawn (DetachableJoint starts attached).
    # Release it so it falls onto the table and settles.
    prepare_cup_for_pickup()


def pick_and_place(arm: UR5GazeboArm):
    """Run the 9-phase pick-and-place cycle."""

    # --- Phase 1: home ---
    print("\n[1/9] Going home...")
    arm.move_to_joint_positions(UR5_HOME, duration_sec=MOVE_SLOW)

    # --- Phase 2: hover above pick ---
    print(f"[2/9] Hovering above pick @ ({PICK_X}, {PICK_Y}, {HOVER_Z:.3f})...")
    arm.move_to_xyz(PICK_X, PICK_Y, HOVER_Z, duration_sec=MOVE_SLOW)

    # --- Phase 3: descend ---
    print(f"[3/9] Descending to grasp @ z={GRASP_Z:.3f}...")
    arm.move_to_xyz(PICK_X, PICK_Y, GRASP_Z, duration_sec=MOVE_FAST)

    # --- Phase 4: grasp ---
    print("[4/9] GRASP — attaching cup to wrist_3...")
    if not attach_cup():
        print("ERROR: attach_cup() failed — aborting.")
        return
    time.sleep(DWELL_AT_GRASP)

    # --- Phase 5: lift ---
    print(f"[5/9] Lifting back to hover @ z={HOVER_Z:.3f}...")
    arm.move_to_xyz(PICK_X, PICK_Y, HOVER_Z, duration_sec=MOVE_FAST)

    # --- Phase 6: transport ---
    print(f"[6/9] Transporting to place hover @ ({PLACE_X}, {PLACE_Y}, {HOVER_Z:.3f})...")
    arm.move_to_xyz(PLACE_X, PLACE_Y, HOVER_Z, duration_sec=MOVE_SLOW)

    # --- Phase 7: descend ---
    print(f"[7/9] Descending to place @ z={PLACE_RELEASE_Z:.3f}...")
    arm.move_to_xyz(PLACE_X, PLACE_Y, PLACE_RELEASE_Z, duration_sec=MOVE_FAST)

    # --- Phase 8: release ---
    print("[8/9] RELEASE — detaching cup; gravity takes over...")
    if not detach_cup():
        print("ERROR: detach_cup() failed — cup may still be attached.")
    time.sleep(DWELL_AT_GRASP)

    # --- Phase 9: retreat & home ---
    print("[9/9] Retreating and going home...")
    arm.move_to_xyz(PLACE_X, PLACE_Y, HOVER_Z, duration_sec=MOVE_FAST)
    arm.move_to_joint_positions(UR5_HOME, duration_sec=MOVE_SLOW)

    print("\nDone.")


def main():
    print("=" * 60)
    print(" UR5 Blind Pick-and-Place Demo (no gripper)")
    print("=" * 60)

    ensure_scene()

    with UR5GazeboArm() as arm:
        pick_and_place(arm)


if __name__ == "__main__":
    main()