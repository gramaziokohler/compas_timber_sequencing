"""The generic ``kinematic_constraint`` function and its fallback."""

from functools import singledispatch

from compas.geometry import Vector
from compas_timber.connections import Joint


@singledispatch
def kinematic_constraint(joint, moving_element, assembled_elements=None):
    """The freedom a joint leaves `moving_element` to be pulled out, the other members held fixed.

    Implementations are registered per joint class with ``@kinematic_constraint.register(JointType)``
    and are looked up along the class hierarchy, like a method: a joint without its own
    registration uses the one for its nearest registered base class, and a joint with none uses
    this generic fallback.

    The fallback is permissive: a single half-space pointing roughly from the other member to
    this one, which almost always yields a valid extraction. It cannot be told apart from a real
    answer by its value, so use :func:`is_fallback` to find out. A joint without an implementation
    reports near-total freedom, which for a vector feeding a robot is the failure direction that
    hurts; once coverage is good, unregistered joints should return a locked constraint instead.

    Parameters
    ----------
    joint : :class:`~compas_timber.connections.Joint`
    moving_element : :class:`~compas_timber.elements.TimberElement`
        The element being extracted from the joint.
    assembled_elements : list, optional
        The other members of the joint that are still in place. Only joints with more than two
        members read it (the Y-butt); the default is every other member.

    Returns
    -------
    :class:`compas.geometry.Line` or :class:`compas.geometry.Vector` or list of :class:`compas.geometry.Vector`
        A ``Line`` is a strict 1-DOF sliding axis, **signed**: the permitted extraction direction
        runs ``start`` -> ``end``, and its reverse pushes deeper into the joint. A ``Vector`` is the
        inward normal of a permitted half-space; a list of them is the intersection of those
        half-spaces.

    Raises
    ------
    ValueError
        If `moving_element` is not a member of `joint`.

    """
    if moving_element not in joint.elements:
        raise ValueError("Element {} is not part of {}.".format(moving_element, joint.name))

    static_element = joint.element_a if moving_element == joint.element_b else joint.element_b

    v = Vector.from_start_end(static_element.centerline.midpoint, moving_element.centerline.midpoint)
    if v.length < 1e-9:
        # The members cross at their midpoints, as in an X-joint, so there is no
        # beam-to-beam direction. Use the axis perpendicular to both, signed so that the
        # two members still separate in opposite directions.
        v = joint.element_a.centerline.direction.cross(joint.element_b.centerline.direction)
        if moving_element == joint.element_b:
            v = -v
        if v.length < 1e-9:
            raise ValueError("Cannot infer a kinematic constraint for {}: its members are collinear and share a midpoint.".format(joint.name))
    v.unitize()
    return v


def is_fallback(joint):
    """Whether `joint` has no registered implementation and gets the permissive fallback.

    Parameters
    ----------
    joint : :class:`~compas_timber.connections.Joint` or type

    Returns
    -------
    bool

    """
    joint_type = joint if isinstance(joint, type) else type(joint)
    return kinematic_constraint.dispatch(joint_type) is kinematic_constraint.dispatch(Joint)
