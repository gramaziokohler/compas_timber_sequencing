"""Kinematics for :class:`compas_timber.connections.TLapJoint`."""

from compas_timber.connections import TLapJoint
from compas_timber.connections.utilities import beam_ref_side_incidence

from .base import register


@register(TLapJoint)
def constraint(joint, moving_element, assembled_elements=None):
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
