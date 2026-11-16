"""Kinematic constraints for LFrenchRidgeLapJoint, LLapJoint, TLapJoint, XLapJoint."""

import math

from compas.geometry import Line
from compas_timber.connections import LFrenchRidgeLapJoint
from compas_timber.connections import LLapJoint
from compas_timber.connections import TLapJoint
from compas_timber.connections import XLapJoint
from compas_timber.connections.utilities import beam_ref_side_incidence

from .base import kinematic_constraint


@kinematic_constraint.register(LLapJoint)
def _l_lap(joint, moving_element, assembled_elements=None):
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


@kinematic_constraint.register(TLapJoint)
def _t_lap(joint, moving_element, assembled_elements=None):
    """Calculates the escape constraint for the T-Lap joint.

    Returns
    -------
    list of :class:`compas.geometry.Vector`
        Inward normals of the half-spaces bounding the lap, negated for the cross beam.

    """
    if moving_element not in joint.elements:
        raise ValueError("Element is not part of this joint.")

    ref_side_dict = beam_ref_side_incidence(joint.main_beam, joint.cross_beam, ignore_ends=True)
    cis_index = min(ref_side_dict, key=ref_side_dict.get)
    kc_vectors = [
        joint.cross_beam.ref_sides[cis_index].normal,
        joint.main_beam.ref_sides[joint.ref_side_index_a].normal * -1,
        joint.main_beam.ref_sides[(joint.ref_side_index_a + 1) % 4].normal,
        joint.main_beam.ref_sides[(joint.ref_side_index_a - 1) % 4].normal,
    ]

    if moving_element == joint.main_beam:
        return kc_vectors

    elif moving_element == joint.cross_beam:
        return [v * -1 for v in kc_vectors]


@kinematic_constraint.register(XLapJoint)
def _x_lap(joint, moving_element, assembled_elements=None):
    """Calculates the 1-DOF strict linear escape constraint for the X-Lap joint.

    The two halved beams can only be separated along the line where the two cutting planes
    meet, signed: ``beam_a`` takes the axis, ``beam_b`` its reverse.

    Parameters
    ----------
    moving_element : :class:`~compas_timber.elements.Beam`
        The element being extracted from the joint.

    Returns
    -------
    :class:`~compas.geometry.Line`
        A Line along the intersection of the cutting planes representing the single degree of freedom.

    Raises
    ------
    ValueError
        If the moving element is not part of this joint.

    """
    if moving_element not in joint.elements:
        raise ValueError("Element is not part of this joint.")

    axis = joint.cutting_plane_b.normal.cross(joint.cutting_plane_a.normal)
    if moving_element == joint.beam_a:
        return Line(joint.location, joint.location + axis)
    elif moving_element == joint.beam_b:
        return Line(joint.location, joint.location - axis)


@kinematic_constraint.register(LFrenchRidgeLapJoint)
def _l_french_ridge_lap(joint, moving_element, assembled_elements=None):
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
