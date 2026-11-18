"""Kinematics for :class:`compas_timber.connections.XLapJoint`."""

from compas.geometry import Line
from compas_timber.connections import XLapJoint

from .base import register


@register(XLapJoint)
def constraint(joint, moving_element, assembled_elements=None):
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
