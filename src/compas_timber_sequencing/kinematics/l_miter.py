"""Kinematics for :class:`compas_timber.connections.LMiterJoint`."""

from compas_timber.connections import LMiterJoint

from .base import register


@register(LMiterJoint)
def constraint(joint, moving_element, assembled_elements=None):
    """Calculates the escape constraint for the LMiter joint."""
    if moving_element not in joint.elements:
        raise ValueError("Element is not part of this joint.")

    plane_a, plane_b = joint.cutting_planes
    if moving_element == joint.beam_a:
        return [plane_b.normal]
    elif moving_element == joint.beam_b:
        return [plane_a.normal]
