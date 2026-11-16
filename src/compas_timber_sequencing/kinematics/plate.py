"""Kinematic constraints for PlateJoint."""

from compas.geometry import Vector
from compas_timber.connections import PlateJoint

from .base import kinematic_constraint


@kinematic_constraint.register(PlateJoint)
def _plate(joint, moving_element, assembled_elements=None):
    """Calculates the escape constraint for the Plate joint."""
    if moving_element not in joint.elements:
        raise ValueError("Element is not part of this joint.")

    # Fallback 3-DOF half-space vector for plates
    if moving_element == joint.plate_a:
        return Vector.from_start_end(joint.plate_b.frame.point, joint.plate_a.frame.point)
    elif moving_element == joint.plate_b:
        return Vector.from_start_end(joint.plate_a.frame.point, joint.plate_b.frame.point)
