"""The registry of joint kinematics, the lookup, and the fallback."""

from compas.geometry import Vector

_REGISTRY = {}
"""dict: Exact joint class -> implementation. Filled by :func:`register`."""


def register(*joint_types):
    """Register a function as the kinematics of one or more joint classes.

    Lookup is by **exact** class: registering a class does not cover its subclasses. A joint
    whose class is not registered gets the permissive fallback, flagged by :func:`is_fallback`.
    That is deliberate. Kinematics are specific to each joint and are written and checked by
    hand, so a new joint type -- including a subclass of a registered one -- is reported as
    unhandled rather than silently given its parent's answer. A subclass that really does
    behave like its parent is registered explicitly with the same function.

    Parameters
    ----------
    *joint_types : type
        The joint classes the decorated function implements.

    Returns
    -------
    callable
        A decorator that registers its function and returns it unchanged. The function is
        called as ``function(joint, moving_element, assembled_elements=None)``.

    Raises
    ------
    ValueError
        If a joint class is already registered.

    Examples
    --------
    >>> from compas_timber_sequencing.kinematics import register
    >>> class MyJoint(object):
    ...     pass
    >>> @register(MyJoint)
    ... def constraint(joint, moving_element, assembled_elements=None):
    ...     return []
    >>> unregister(MyJoint)

    """
    if not joint_types:
        raise ValueError("register() needs at least one joint class.")

    def decorator(function):
        for joint_type in joint_types:
            if joint_type in _REGISTRY:
                raise ValueError("{} already has a kinematics implementation registered.".format(joint_type.__name__))
            _REGISTRY[joint_type] = function
        return function

    return decorator


def unregister(joint_type):
    """Remove a joint class from the registry, for tests and for replacing an implementation.

    Parameters
    ----------
    joint_type : type

    """
    _REGISTRY.pop(joint_type, None)


def registered_joint_types():
    """The joint classes that have their own kinematics.

    Returns
    -------
    frozenset of type

    """
    return frozenset(_REGISTRY)


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
    return joint_type not in _REGISTRY


def kinematic_constraint(joint, moving_element, assembled_elements=None):
    """The freedom a joint leaves `moving_element` to be pulled out, the other members held fixed.

    Looks up the implementation registered for ``type(joint)`` -- the exact class, see
    :func:`register` -- and falls back to :func:`fallback_constraint` when there is none.

    Parameters
    ----------
    joint : :class:`~compas_timber.connections.Joint`
    moving_element : :class:`~compas_timber.elements.Beam`
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
    implementation = _REGISTRY.get(type(joint), fallback_constraint)
    return implementation(joint, moving_element, assembled_elements)


def fallback_constraint(joint, moving_element, assembled_elements=None):
    """The permissive answer for a joint without its own kinematics.

    A single half-space pointing from the other member's centerline midpoint to this one's, which
    almost always yields a valid extraction. It cannot be told apart from a real answer by its
    value, so use :func:`is_fallback` to find out; the adapter tags these constraints as
    *inferred*. A joint without an implementation reports near-total freedom, which for a vector
    feeding a robot is the failure direction that hurts; once coverage is good, unregistered
    joints should return a locked constraint instead.

    Parameters
    ----------
    joint : :class:`~compas_timber.connections.Joint`
    moving_element : :class:`~compas_timber.elements.Beam`
    assembled_elements : list, optional
        Ignored.

    Returns
    -------
    :class:`compas.geometry.Vector`

    Raises
    ------
    ValueError
        If `moving_element` is not a member of `joint`, or the members are collinear and share a
        midpoint.

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
