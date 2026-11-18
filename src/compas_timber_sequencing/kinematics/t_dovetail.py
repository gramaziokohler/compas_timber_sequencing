"""Kinematics for :class:`compas_timber.connections.TDovetailJoint`."""

import math

from compas.geometry import Line
from compas_timber.connections import TDovetailJoint

from .base import register


def _dovetail_axis(joint):
    """Calculates the axis direction of the dovetail groove in the cross beam.

    The groove runs along the cross beam's length on the mortise face.  The direction matches
    the tenon frame's x-axis, which is the negative of the ref side's x-axis (``normal × yaxis``).

    Returns
    -------
    :class:`~compas.geometry.Vector`
        Unit vector along the groove direction.

    """
    crs_normal = joint.cross_beam.ref_sides[joint.cross_beam_ref_side_index].normal
    mrs_normal_neg = joint.main_beam.ref_sides[joint.main_beam_ref_side_index].normal
    z_component = crs_normal * mrs_normal_neg.dot(crs_normal)
    projected_vector = mrs_normal_neg - z_component
    if joint.rotation != 0.0:
        projected_vector.rotate(math.radians(joint.rotation), crs_normal, joint.location)
    return projected_vector


@register(TDovetailJoint)
def constraint(joint, moving_element, assembled_elements=None):
    """Calculates the 1-DOF strict linear escape constraint for a dovetail."""
    if moving_element not in joint.elements:
        raise ValueError("Element is not part of this joint.")

    dovetail_axis = _dovetail_axis(joint)

    if moving_element == joint.cross_beam:
        return Line(joint.location, joint.location + (dovetail_axis * -1))

    elif moving_element == joint.main_beam:
        return Line(joint.location, joint.location + dovetail_axis)
