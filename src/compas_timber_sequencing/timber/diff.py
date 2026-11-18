"""What changed between two plans: the "what did my change do to the sequence" view.

Compare the plan before an adjustment -- to the design or to the overrides -- with the plan after it.
Elements are matched by key, so the two plans can come from two different builds of the model.

Examples
--------
::

    before = plan_sequence(model)
    after = plan_sequence(model, SequenceOverrides().place_before("bottom rail", "strut"))
    print(diff(before, after).describe())

"""

import math

ANGLE_TOLERANCE = 1.0
"""float: Insertion vectors that differ by less than this many degrees count as unchanged."""


def _angle(a, b):
    dot = max(-1.0, min(1.0, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]))
    return math.degrees(math.acos(dot))


class PlanDiff(object):
    """The differences between two plans, from :func:`diff`.

    Attributes
    ----------
    added, removed : list of str
        Elements only in the new plan, only in the old one.
    moved : list of tuple
        ``(key, old position, new position)``; a position is None for an element that was not
        placed.
    turned : list of tuple
        ``(key, angle in degrees)`` for insertion vectors that changed direction, or ``None`` as
        the angle when one plan has a vector and the other does not.
    state_changed : list of tuple
        ``(key, old state, new state)`` -- roomy, tight, locked.
    by_hand_changed : list of tuple
        ``(key, old, new)``.
    geometry_changed, joints_changed : list of str
        Elements whose beam, or whose joints, differ between the models the plans were made from.
    completeness : tuple or None
        ``(old, new)`` when one plan is complete and the other is not.
    overrides_changed : bool
        True when the plans were made with different overrides.

    """

    def __init__(self, added, removed, moved, turned, state_changed, by_hand_changed, geometry_changed, joints_changed, completeness, overrides_changed):
        self.added = added
        self.removed = removed
        self.moved = moved
        self.turned = turned
        self.state_changed = state_changed
        self.by_hand_changed = by_hand_changed
        self.geometry_changed = geometry_changed
        self.joints_changed = joints_changed
        self.completeness = completeness
        self.overrides_changed = overrides_changed

    @property
    def is_empty(self):
        """bool : True when the two plans are the same in every respect compared here."""
        return not (
            self.added
            or self.removed
            or self.moved
            or self.turned
            or self.state_changed
            or self.by_hand_changed
            or self.geometry_changed
            or self.joints_changed
            or self.completeness
            or self.overrides_changed
        )

    def __bool__(self):
        return not self.is_empty

    __nonzero__ = __bool__

    def __repr__(self):
        return "PlanDiff(moved={}, turned={}, state_changed={}, added={}, removed={})".format(
            len(self.moved), len(self.turned), len(self.state_changed), len(self.added), len(self.removed)
        )

    def describe(self):
        """A readable summary, one change per line.

        Returns
        -------
        str

        """
        if self.is_empty:
            return "No change."

        def position(value):
            return "-" if value is None else str(value)

        lines = []
        if self.completeness:
            lines.append("complete: {} -> {}".format(*self.completeness))
        if self.overrides_changed:
            lines.append("made with different overrides")
        for key in self.added:
            lines.append("added: {}".format(key))
        for key in self.removed:
            lines.append("removed: {}".format(key))
        for key, old, new in self.moved:
            lines.append("moved: {}  {} -> {}".format(key, position(old), position(new)))
        for key, angle in self.turned:
            lines.append("insertion changed: {}  {}".format(key, "now with/without a vector" if angle is None else "turned {:.0f} deg".format(angle)))
        for key, old, new in self.state_changed:
            lines.append("fit changed: {}  {} -> {}".format(key, old, new))
        for key, old, new in self.by_hand_changed:
            lines.append("by hand: {}  {} -> {}".format(key, old, new))
        for key in self.geometry_changed:
            lines.append("geometry changed: {}".format(key))
        for key in self.joints_changed:
            lines.append("joints changed: {}".format(key))
        return "\n".join(lines)


def diff(old, new, angle_tolerance=ANGLE_TOLERANCE):
    """Compare two plans.

    Parameters
    ----------
    old, new : :class:`~compas_timber_sequencing.timber.plan.SequencePlan`
    angle_tolerance : float, optional
        Insertion vectors closer than this, in degrees, count as unchanged.

    Returns
    -------
    :class:`PlanDiff`

    """
    old_keys, new_keys = set(old.keys), set(new.keys)
    common = sorted(old_keys & new_keys)

    moved = []
    turned = []
    state_changed = []
    by_hand_changed = []
    for key in common:
        old_position, new_position = old.position(key), new.position(key)
        if old_position != new_position:
            moved.append((key, old_position, new_position))
        if key not in old.steps or key not in new.steps:
            continue
        old_step, new_step = old.steps[key], new.steps[key]
        old_vector, new_vector = old_step["insertion_vector"], new_step["insertion_vector"]
        if (old_vector is None) != (new_vector is None):
            turned.append((key, None))
        elif old_vector is not None:
            angle = _angle(old_vector, new_vector)
            if angle > angle_tolerance:
                turned.append((key, angle))
        if old_step["state"] != new_step["state"]:
            state_changed.append((key, old_step["state"], new_step["state"]))
        if old_step["by_hand"] != new_step["by_hand"]:
            by_hand_changed.append((key, old_step["by_hand"], new_step["by_hand"]))

    moved.sort(key=lambda item: (item[2] is None, item[2] if item[2] is not None else 0, item[0]))

    geometry_changed = []
    joints_changed = []
    for key in common:
        before, after = old.fingerprints.get(key), new.fingerprints.get(key)
        if before is None or after is None:
            continue
        if before.get("geometry") != after.get("geometry"):
            geometry_changed.append(key)
        if before.get("joints") != after.get("joints"):
            joints_changed.append(key)

    completeness = (old.is_complete, new.is_complete) if old.is_complete != new.is_complete else None
    overrides_changed = old.overrides.__data__ != new.overrides.__data__

    return PlanDiff(
        added=sorted(new_keys - old_keys),
        removed=sorted(old_keys - new_keys),
        moved=moved,
        turned=turned,
        state_changed=state_changed,
        by_hand_changed=by_hand_changed,
        geometry_changed=geometry_changed,
        joints_changed=joints_changed,
        completeness=completeness,
        overrides_changed=overrides_changed,
    )
