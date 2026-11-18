"""Kinematics for :class:`compas_timber.connections.MortiseTenonJoint`."""

from compas.geometry import Line
from compas_timber.connections import MortiseTenonJoint

from .base import register


@register(MortiseTenonJoint)
def constraint(joint, moving_element, assembled_elements=None):
    """Calculates the 1-DOF strict linear escape constraint for the L mortise-tenon joint.

    A tenon can only be inserted into and removed from a mortise along the tenon axis.

    Parameters
    ----------
    moving_element : :class:`~compas_timber.elements.Beam`
        The element being extracted from the joint.

    Returns
    -------
    :class:`~compas.geometry.Line`
        A Line along the tenon axis representing the single degree of freedom.

    Raises
    ------
    ValueError
        If the moving element is not part of this joint.

    """
    if moving_element not in joint.elements:
        raise ValueError("Element is not part of this joint.")

    axis = joint.cross_beam.ref_sides[joint.cross_beam_ref_side_index].normal

    if moving_element == joint.main_beam:
        return Line(joint.location, joint.location + axis)
    elif moving_element == joint.cross_beam:
        return Line(joint.location, joint.location - axis)
