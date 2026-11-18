"""Kinematics for :class:`compas_timber.connections.LLapJoint`."""

from compas_timber.connections import LLapJoint

from .base import register


@register(LLapJoint)
def constraint(joint, moving_element, assembled_elements=None):
    """Calculates the escape constraint for the Lap joint.

    Returns
    -------
    list of :class:`compas.geometry.Vector`
        Inward normals of the half-spaces bounding the lap, whose intersection is the
        sliding freedom along the lap plane.

    """
    if moving_element not in joint.elements:
        raise ValueError("Element is not part of this joint.")

    if moving_element == joint.beam_a:
        return [(joint.cutting_plane_a.normal * -1), joint.cutting_plane_b.normal, (joint.beam_a.ref_sides[joint.ref_side_index_a].normal * -1)]

    elif moving_element == joint.beam_b:
        return [joint.cutting_plane_a.normal, (joint.cutting_plane_b.normal * -1), (joint.beam_b.ref_sides[joint.ref_side_index_b].normal * -1)]
