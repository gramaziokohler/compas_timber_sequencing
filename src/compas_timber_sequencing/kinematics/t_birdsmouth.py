"""Kinematics for :class:`compas_timber.connections.TBirdsmouthJoint`."""

from compas.geometry import Line
from compas_timber.connections import TBirdsmouthJoint

from .base import register


@register(TBirdsmouthJoint)
def constraint(joint, moving_element, assembled_elements=None):
    """Calculates the escape constraint for the TBirdsmouth joint.

    Returns
    -------
    :class:`compas.geometry.Line` or list of :class:`compas.geometry.Vector`
        With a mill depth the seat is a housing, so the freedom is a strict 1-DOF
        axis along the main beam, **signed**: the main beam slides out away from the
        joint and the cross beam the opposite way. A member seated at both ends is
        therefore held to two opposite directions and is locked -- the canonical
        hand-placed case.

        Without a mill depth the freedom is the intersection of the half-spaces of
        the two notch faces, negated for the cross beam.

    """
    if moving_element not in joint.elements:
        raise ValueError("Element is not part of this joint.")

    cps = joint._get_cutting_planes()

    endpoint, _ = joint.main_beam.endpoint_closest_to_point(joint.location)
    main_beam_vector = joint.main_beam.centerline.direction
    if endpoint == "end":
        main_beam_vector = -main_beam_vector

    if moving_element == joint.main_beam:
        if joint.mill_depth:
            return Line(joint.location, joint.location + main_beam_vector)
        return [cps[0].normal, cps[1].normal]
    elif moving_element == joint.cross_beam:
        if joint.mill_depth:
            return Line(joint.location, joint.location - main_beam_vector)
        return [cps[0].normal * -1, cps[1].normal * -1]
