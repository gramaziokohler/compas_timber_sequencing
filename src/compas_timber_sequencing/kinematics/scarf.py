"""Kinematic constraints for ISimpleScarf."""

import math

from compas_timber.connections import ISimpleScarf

from .base import kinematic_constraint


@kinematic_constraint.register(ISimpleScarf)
def _i_simple_scarf(joint, moving_element, assembled_elements=None):
    """Calculates the escape constraint for the ISimpleScarf joint."""
    if moving_element not in joint.elements:
        raise ValueError("Element is not part of this joint.")

    width, height = joint.main_beam.get_dimensions_relative_to_side(joint.main_beam_ref_side_index)
    scarf_depth = height - (joint.depth_ref_side + joint.depth_opp_side)
    scarf_angle = math.atan((scarf_depth / joint.length))

    if moving_element == joint.main_beam:
        beam_side = joint._get_beam_side(joint.main_beam)
        if beam_side == "start":
            end_vector = joint.main_beam.centerline.direction
            opp_frame = joint.main_beam.opp_side(joint.main_beam_ref_side_index)
            scarf_normal = opp_frame.rotated((scarf_angle * -1), opp_frame.yaxis, opp_frame.point).normal * -1
        else:
            end_vector = joint.main_beam.centerline.direction * -1
            opp_frame = joint.main_beam.opp_side(joint.main_beam_ref_side_index)
            scarf_normal = opp_frame.rotated(scarf_angle, opp_frame.yaxis, opp_frame.point).normal * -1

    elif moving_element == joint.cross_beam:
        beam_side = joint._get_beam_side(joint.cross_beam)
        if beam_side == "start":
            end_vector = joint.cross_beam.centerline.direction
            opp_frame = joint.cross_beam.opp_side(joint.cross_beam_ref_side_index)
            scarf_normal = opp_frame.rotated((scarf_angle * -1), opp_frame.yaxis, opp_frame.point).normal * -1
        else:
            end_vector = joint.cross_beam.centerline.direction * -1
            opp_frame = joint.cross_beam.opp_side(joint.cross_beam_ref_side_index)
            scarf_normal = opp_frame.rotated(scarf_angle, opp_frame.yaxis, opp_frame.point).normal * -1

    return [scarf_normal, end_vector]
