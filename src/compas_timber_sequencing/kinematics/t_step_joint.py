"""Kinematics for :class:`compas_timber.connections.TStepJoint`."""

import math

from compas.geometry import Vector
from compas.geometry import angle_vectors_signed
from compas_timber.connections import TStepJoint
from compas_timber.fabrication import StepShapeType

from .base import register


@register(TStepJoint)
def constraint(joint, moving_element, assembled_elements=None):
    """Calculates the escape constraint for the TStep joint."""
    if moving_element not in joint.elements:
        raise ValueError("Element is not part of this joint.")

    main_beam_ref_side = joint.main_beam.ref_sides[joint.main_beam_ref_side_index]
    cross_beam_ref_side = joint.cross_beam.ref_sides[joint.cross_beam_ref_side_index]
    main_beam_end_side = joint.main_beam.endpoint_closest_to_point(joint.location)[0]

    strut_incl_vector = Vector.cross(main_beam_ref_side.zaxis, cross_beam_ref_side.normal)
    strut_signed_angle = angle_vectors_signed(main_beam_ref_side.zaxis, cross_beam_ref_side.normal, strut_incl_vector, deg=True)
    strut_inclination = 180 - abs(strut_signed_angle)

    dir_mulitplier = -1 if main_beam_end_side == "end" else 1
    main_beam_direction_vector = joint.main_beam.centerline.direction * dir_mulitplier

    if joint.step_shape in (StepShapeType.HEEL, StepShapeType.TAPERED_HEEL):
        if joint.step_shape == StepShapeType.HEEL:
            kc_vectors = [main_beam_ref_side.normal * -1, main_beam_direction_vector, cross_beam_ref_side.normal]

        elif joint.step_shape == StepShapeType.TAPERED_HEEL:
            kc_vectors = [main_beam_ref_side.normal * -1, main_beam_direction_vector]

    else:
        if strut_inclination < 90:
            va_rot_ang = (180 - strut_inclination) / 2
        else:
            va_rot_ang = strut_inclination - 180

        _, height = joint.main_beam.get_dimensions_relative_to_side(joint.main_beam_ref_side_index)
        intersection_length = height / math.sin(math.radians(strut_inclination))
        short_step_side_offset = math.tan(math.radians(90 - va_rot_ang)) * joint.step_depth

        heel_reduction = 0.0
        if joint.step_shape == StepShapeType.DOUBLE:
            heel_reduction = joint.heel_depth / (math.sin(math.radians(strut_inclination)) * math.cos(math.radians(strut_inclination)))

        long_step_side_offset = intersection_length - (heel_reduction + short_step_side_offset)
        vb_rot_ang = math.degrees(math.atan(joint.step_depth / long_step_side_offset))

        vector_a = cross_beam_ref_side.rotated(math.radians(-va_rot_ang * dir_mulitplier), main_beam_ref_side.yaxis, cross_beam_ref_side.point).normal
        vector_b = cross_beam_ref_side.rotated(math.radians(vb_rot_ang * dir_mulitplier), main_beam_ref_side.yaxis, cross_beam_ref_side.point).normal

        if joint.step_shape == StepShapeType.STEP:
            kc_vectors = [vector_a, vector_b]

        elif joint.step_shape == StepShapeType.DOUBLE:
            kc_vectors = [vector_a, vector_b, main_beam_ref_side.normal * -1, main_beam_direction_vector]

    if joint.tenon_mortise_height:
        tm_vector = joint.main_beam.centerline.direction.cross(joint.cross_beam.centerline.direction)
        kc_vectors.extend([tm_vector, tm_vector * -1])
    if moving_element == joint.main_beam:
        return kc_vectors
    elif moving_element == joint.cross_beam:
        return [-v for v in kc_vectors]
