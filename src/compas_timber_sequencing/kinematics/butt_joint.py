"""Kinematics for :class:`compas_timber.connections.ButtJoint`."""

from compas.geometry import Plane
from compas_timber.connections import ButtJoint

from .base import register


@register(ButtJoint)
def constraint(joint, moving_element, assembled_elements=None):
    """Calculates the escape constraint for the Butt joint.

    Returns
    -------
    list of :class:`compas.geometry.Vector`
        Inward normals of the half-spaces the moving element may travel in. A plain
        butt gives one, the butt plane normal pointing away from the static element.
        A milled pocket adds the normals of the pocket walls, so the intersection of
        the half-spaces describes the restricted movement.

    """
    if moving_element not in joint.elements:
        raise ValueError("Element is not part of this joint.")

    if moving_element == joint.cross_beam:
        normal = joint.butt_plane.normal

    elif moving_element == joint.main_beam:
        normal = joint.butt_plane.copy().normal * -1

    if joint.mill_depth:
        side_a_normal = Plane.from_frame(joint.main_beam.ref_sides[joint.main_beam_ref_side_index]).normal * -1
        side_b_normal = Plane.from_frame(joint.main_beam.opp_side(joint.main_beam_ref_side_index)).normal * -1

        return [normal, side_a_normal, side_b_normal]
    else:
        return [normal]
