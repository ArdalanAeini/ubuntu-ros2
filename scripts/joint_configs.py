"""
Named joint configurations (poses) for the UR5.

Why this file exists:
- "Magic numbers" are a code smell. Writing [0, -1.57, 1.57, ...] in a demo
  tells you NOTHING about what the pose means.
- Named poses (HOME, READY, PICK_HOVER) make code self-documenting.
- Changing a pose -> edit ONE place, not every demo script.

All angles are in DEGREES here for human readability,
then converted to radians via JointPositions.from_degrees().
"""

from robot_arm_base import JointPositions


# All joints at zero. The arm points straight up — UR's "zero" pose.
UR5_HOME = JointPositions.from_degrees([0, 0, 0, 0, 0, 0])

# A safe, compact "ready to work" pose. Elbow up, wrist neutral.
# This is a common starting configuration for manipulation tasks.
UR5_READY = JointPositions.from_degrees([0, -90, 90, -90, -90, 0])

# Hovering above a notional pick location — elbow further forward,
# wrist tilted down toward the table.
UR5_PICK_HOVER = JointPositions.from_degrees([0, -60, 110, -140, -90, 0])

# Compact "tucked" pose for stowing / transport.
UR5_TUCKED = JointPositions.from_degrees([0, -180, 0, -90, 0, 0])