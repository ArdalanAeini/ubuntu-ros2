"""
Helpers to trigger the DetachableJoint plugin on a graspable object.

IMPORTANT BEHAVIOR NOTE:
    Gazebo Harmonic's DetachableJoint plugin starts in the ATTACHED state
    when the model spawns — the fixed joint is created immediately between
    the child model and the robot's link at their current poses.

    For the CUP this is undesirable (cup gets yanked to the wrist), so
    we call prepare_cup_for_pickup() right after spawn to detach + settle.

    For the GRIPPER this is exactly what we want (gripper stays attached
    forever), so we just spawn it at the right pose and let the initial
    attach do its work. No helper function needed for the gripper.

Architecture mirror of spawn_objects.py: we shell out to the `gz topic` CLI
rather than depending on Python gz-transport bindings.

Usage (cup):
    from grasp import attach_cup, detach_cup, prepare_cup_for_pickup
    prepare_cup_for_pickup()  # call right after spawning the cup
    # ... move arm to the cup ...
    attach_cup()
    # ... move arm to destination ...
    detach_cup()
"""

import subprocess
import time


def _publish_empty(topic: str, timeout_ms: int = 2000) -> bool:
    """
    Publish a single gz.msgs.Empty message to `topic`.

    Returns True if the gz CLI exited cleanly. Note: success here means
    "the publish was sent" — it does NOT confirm a subscriber received it.
    """
    result = subprocess.run(
        [
            "gz", "topic",
            "-t", topic,
            "-m", "gz.msgs.Empty",
            "-p", "",
        ],
        capture_output=True,
        text=True,
        timeout=timeout_ms / 1000.0,
    )
    if result.returncode != 0:
        print(f"[grasp] publish to {topic} FAILED: {result.stderr.strip()}")
        return False
    return True


def attach_cup(topic: str = "/cup/attach") -> bool:
    """Weld the cup to the robot's wrist link (creates the fixed joint)."""
    return _publish_empty(topic)


def detach_cup(topic: str = "/cup/detach") -> bool:
    """Release the cup; gravity resumes from its current pose."""
    return _publish_empty(topic)


def prepare_cup_for_pickup(
    detach_topic: str = "/cup/detach",
    settle_time_sec: float = 1.5,
) -> bool:
    """
    Call this once, right after spawning a graspable cup.

    Because Gazebo's DetachableJoint starts ATTACHED at spawn, the cup gets
    yanked to the wrist. We immediately detach so the cup drops to the
    table, then wait for it to physically settle before the arm tries to
    pick it up.
    """
    print("[grasp] Initial detach — releasing cup onto table...")
    ok = detach_cup(detach_topic)
    if not ok:
        return False
    time.sleep(settle_time_sec)
    return True