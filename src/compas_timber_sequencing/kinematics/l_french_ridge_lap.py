"""Kinematics for :class:`compas_timber.connections.LFrenchRidgeLapJoint`."""

import math

from compas_timber.connections import LFrenchRidgeLapJoint

from .base import register


@register(LFrenchRidgeLapJoint)
def constraint(joint, moving_element, assembled_elements=None):
    """Calculates the escape constraint for the L-French-Ridge-Lap joint.

    Returns
    -------
    list of :class:`compas.geometry.Vector`
        Inward normals of the two lap cutting planes, oriented for the moving beam, and of
        the inclined ridge face.

    """
    if moving_element not in joint.elements:
        raise ValueError("Element is not part of this joint.")

    beam_a_vec = joint.beam_a.centerline.direction.unitized()
    beam_b_vec = joint.beam_b.centerline.direction.unitized()
    side_a, _ = joint.beam_a.endpoint_closest_to_point(joint.location)
    side_b, _ = joint.beam_b.endpoint_closest_to_point(joint.location)

    # flip the other beam's vector when the two beams approach the joint from opposite endpoint types
    multiplier = -1 if side_a != side_b else 1

    if moving_element == joint.beam_a:
        moving_beam, other_beam = joint.beam_a, joint.beam_b
        moving_vec = beam_a_vec
        other_vec = beam_b_vec * multiplier
        ref_side_index = joint.ref_side_index_a
        other_ref_side_index = joint.ref_side_index_b
        h_diff_sign = -1 if side_b == "start" else 1
    else:
        moving_beam, other_beam = joint.beam_b, joint.beam_a
        moving_vec = beam_b_vec
        other_vec = beam_a_vec * multiplier
        ref_side_index = joint.ref_side_index_b
        other_ref_side_index = joint.ref_side_index_a
        h_diff_sign = -1 if side_a == "start" else 1

    rot_vec = (moving_vec + other_vec).unitized()
    w, h = moving_beam.get_dimensions_relative_to_side(ref_side_index)
    h_diff = h_diff_sign * h / 3
    cos_half_beta = moving_vec.dot(rot_vec)
    diag_diff = w / cos_half_beta
    frl_angle = math.atan(h_diff / diag_diff)
    ref_frame = other_beam.ref_sides[other_ref_side_index]
    frl_frame = ref_frame.rotated(-frl_angle, rot_vec, ref_frame.point)

    if moving_element == joint.beam_a:
        return [(joint.cutting_plane_a.normal * -1), joint.cutting_plane_b.normal, frl_frame.normal]
    else:
        return [joint.cutting_plane_a.normal, (joint.cutting_plane_b.normal * -1), frl_frame.normal]
