"""A sequence as data of its own, kept apart from the model it was made for.

:func:`plan_sequence` sequences a model and returns a :class:`SequencePlan`; it does not touch the
model. The plan refers to elements by stable key (see :mod:`~compas_timber_sequencing.timber.keys`),
records what it was made from -- the overrides, the settings, and a fingerprint of every beam's
geometry and joints -- and saves to JSON like any COMPAS data object. That makes it usable across
rebuilds of the model and across scripts: check it against a model with :meth:`SequencePlan.check`,
compare two plans with :func:`~compas_timber_sequencing.timber.diff.diff`, and write it onto a model
for export with :func:`apply_plan` when you want the result in the element attributes.

Examples
--------
::

    plan = plan_sequence(model)
    overrides = SequenceOverrides().place_before("bottom rail", "strut")
    new = plan_sequence(model, overrides)
    print(diff(plan, new).describe())
    new.to_json("plan.json")

"""

import hashlib

from compas.data import Data

from ..core.overrides import SequenceOverrides
from ..core.search import DEFAULT_BEAM_WIDTH
from ..core.search import generate
from ..core.solver import APPROACH_DISTANCE
from ..ranking.strategies import make_strategy
from .adapter import TimberModelAdapter
from .attributes import clear_attributes
from .attributes import write_attributes
from .keys import KEY_TOLERANCE
from .keys import element_keys
from .keys import geometry_token


def _digest(token):
    return hashlib.sha1(repr(token).encode("utf-8")).hexdigest()[:10]


def fingerprints(model, tolerance=KEY_TOLERANCE, keys=None):
    """A fingerprint of every beam's geometry and joints, keyed by element key.

    Two fingerprints are equal when the beam has the same centerline and section (within
    `tolerance`) and the same joints, by joint type and partner key. Comparing them is how a plan
    tells which beams changed since it was made.

    Parameters
    ----------
    model : :class:`~compas_timber.model.TimberModel`
    tolerance : float, optional
    keys : dict, optional
        ``str(guid)`` -> key, from :func:`~compas_timber_sequencing.timber.keys.element_keys`;
        computed when not given.

    Returns
    -------
    dict
        Key -> ``{"geometry": str, "joints": str}``.

    """
    keys = keys if keys is not None else element_keys(model, tolerance)
    prints = {}
    for element in model.elements():
        if not getattr(element, "is_beam", False):
            continue
        key = keys[str(element.guid)]
        joints = sorted(
            (type(joint).__name__, tuple(sorted(keys[str(other.guid)] for other in joint.elements if other is not element))) for joint in model.get_joints_for_element(element)
        )
        prints[key] = {"geometry": _digest(geometry_token(element, tolerance)), "joints": _digest(joints)}
    return prints


class SequencePlan(Data):
    """An assembly sequence, keyed by stable element keys.

    Made by :func:`plan_sequence`; rarely constructed directly.

    Parameters
    ----------
    order : list of str
        Keys of the placed elements, in assembly order.
    steps : dict
        Key -> ``{"insertion_vector": Vector or None, "state": str, "clearance": float or None,
        "by_hand": bool, "inferred": int}``. ``clearance`` is in degrees.
    unplaced : list of str, optional
        Elements the search could not place because it got stuck. They come before everything in
        `order`, in an order nobody found.
    excluded : dict, optional
        Key -> reason, for elements that are not sequenced.
    intrinsic_locks : list of list, optional
        Groups of elements that lock each other in every order.
    proposed_by_hand : list, optional
        What the solver suggests placing by hand: intrinsic locks and tight fits.
    stuck : dict, optional
        ``{"remaining": [key], "reasons": {key: reason}}`` when the search got stuck.
    pin_conflict : dict, optional
        ``{"position": int, "key": str, "reason": str}`` when a pin could not be honoured.
    staleness : list, optional
        ``[kind, key, message]`` for every override that did not apply to the model.
    fingerprints : dict, optional
        Key -> ``{"geometry": str, "joints": str}`` for every beam, from :func:`fingerprints`.
    overrides : :class:`~compas_timber_sequencing.core.overrides.SequenceOverrides`, optional
        The overrides the plan was made with.
    settings : dict, optional
        ``strategy``, ``width``, ``distance``, ``use_geometry``, ``tolerance``.
    version : str, optional
        The package version that made the plan.

    """

    @property
    def __dtype__(self):
        # Saved as compas_timber_sequencing/SequencePlan, which the package root exports, so that loading
        # from JSON finds the class without the subpackage having to import it.
        return "compas_timber_sequencing/SequencePlan"

    @property
    def __data__(self):
        return {
            "order": list(self.order),
            "steps": {key: dict(step) for key, step in self.steps.items()},
            "unplaced": list(self.unplaced),
            "excluded": dict(self.excluded),
            "intrinsic_locks": [list(group) for group in self.intrinsic_locks],
            "proposed_by_hand": list(self.proposed_by_hand),
            "stuck": self.stuck,
            "pin_conflict": self.pin_conflict,
            "staleness": [list(entry) for entry in self.staleness],
            "fingerprints": {key: dict(value) for key, value in self.fingerprints.items()},
            "overrides": self.overrides,
            "settings": dict(self.settings),
            "version": self.version,
        }

    @classmethod
    def __from_data__(cls, data):
        return cls(**data)

    def __init__(
        self,
        order,
        steps,
        unplaced=None,
        excluded=None,
        intrinsic_locks=None,
        proposed_by_hand=None,
        stuck=None,
        pin_conflict=None,
        staleness=None,
        fingerprints=None,
        overrides=None,
        settings=None,
        version=None,
        name=None,
    ):
        super(SequencePlan, self).__init__(name=name)
        self.order = list(order)
        self.steps = {key: dict(step) for key, step in steps.items()}
        self.unplaced = list(unplaced or [])
        self.excluded = dict(excluded or {})
        self.intrinsic_locks = [sorted(group) for group in intrinsic_locks or []]
        self.proposed_by_hand = sorted(proposed_by_hand or [])
        self.stuck = stuck
        self.pin_conflict = pin_conflict
        self.staleness = [tuple(entry) for entry in staleness or []]
        self.fingerprints = {key: dict(value) for key, value in (fingerprints or {}).items()}
        self.overrides = overrides if overrides is not None else SequenceOverrides()
        self.settings = dict(settings or {})
        self.version = version

    def __repr__(self):
        return "SequencePlan({} placed{}, complete={})".format(len(self.order), ", {} unplaced".format(len(self.unplaced)) if self.unplaced else "", self.is_complete)

    def __len__(self):
        return len(self.order)

    # ------------------------------------------------------------------------------------
    # reading the plan
    # ------------------------------------------------------------------------------------

    @property
    def is_complete(self):
        """bool : True when every sequenced element has a position."""
        return self.stuck is None and self.pin_conflict is None and not self.unplaced

    @property
    def inferred_total(self):
        """int : How many constraints behind this plan came from the permissive fallback for joints
        without their own kinematics. Zero means every answer was computed."""
        return sum(step.get("inferred", 0) for step in self.steps.values())

    @property
    def keys(self):
        """list of str : Every sequenced element: the unplaced ones first, then the placed ones in order."""
        return list(self.unplaced) + list(self.order)

    def position(self, key):
        """The absolute assembly position of an element, from 0, or None if it was not placed.

        Placed elements come after the unplaced ones, so a stuck plan still gives every placed
        element its true position in the full sequence.

        Parameters
        ----------
        key : str

        Returns
        -------
        int or None

        """
        if key in self.steps:
            return len(self.unplaced) + self.order.index(key)
        if key in self.unplaced:
            return None
        raise KeyError("{!r} is not in this plan".format(key))

    def insertion_vector(self, key):
        """The unit insertion vector of a placed element, or None (placed by hand, or locked)."""
        return self.steps[key]["insertion_vector"]

    def state(self, key):
        """``"roomy"``, ``"tight"`` or ``"locked"``."""
        return self.steps[key]["state"]

    def describe(self):
        """A multi-line summary of the plan.

        Returns
        -------
        str

        """
        lines = []
        for key in self.unplaced:
            lines.append("  -  {}  (not placed: {})".format(key, (self.stuck or {}).get("reasons", {}).get(key, "the search stopped before it")))
        for key in self.order:
            step = self.steps[key]
            vector = step["insertion_vector"]
            text = "by hand" if vector is None else "({:+.2f}, {:+.2f}, {:+.2f})".format(vector[0], vector[1], vector[2])
            clearance = "" if step["clearance"] is None else " {:.1f} deg".format(step["clearance"])
            hand = " [by hand]" if step["by_hand"] and vector is not None else ""
            lines.append("{:>3}  {}  {}  {}{}{}".format(self.position(key), key, text, step["state"], clearance, hand))
        if self.pin_conflict:
            lines.append("pin conflict at position {position}: {key!r} {reason}".format(**self.pin_conflict))
        for kind, key, message in self.staleness:
            lines.append("stale {}: {}".format(kind, message))
        return "\n".join(lines)

    # ------------------------------------------------------------------------------------
    # against a model
    # ------------------------------------------------------------------------------------

    def check(self, model):
        """How this plan relates to a model: which elements are missing, new, or changed.

        Parameters
        ----------
        model : :class:`~compas_timber.model.TimberModel`

        Returns
        -------
        :class:`PlanCheck`

        """
        tolerance = self.settings.get("tolerance", KEY_TOLERANCE)
        current = fingerprints(model, tolerance)
        planned = set(self.keys)
        changed = {}
        for key in sorted(planned & set(current)):
            before = self.fingerprints.get(key)
            if before is None:
                continue
            parts = [part for part in ("geometry", "joints") if before.get(part) != current[key][part]]
            if parts:
                changed[key] = parts
        return PlanCheck(missing=sorted(planned - set(current)), new=sorted(set(current) - planned), changed=changed)


class PlanCheck(object):
    """How a plan relates to a model, from :meth:`SequencePlan.check`.

    Attributes
    ----------
    missing : list of str
        In the plan, not in the model.
    new : list of str
        In the model, not in the plan.
    changed : dict
        Key -> ``["geometry"]``, ``["joints"]`` or both, for elements whose beam changed since the
        plan was made.

    """

    def __init__(self, missing, new, changed):
        self.missing = list(missing)
        self.new = list(new)
        self.changed = dict(changed)

    @property
    def is_current(self):
        """bool : True when the plan was made for this model exactly."""
        return not (self.missing or self.new or self.changed)

    def __repr__(self):
        return "PlanCheck(current={}, missing={}, new={}, changed={})".format(self.is_current, len(self.missing), len(self.new), len(self.changed))

    def describe(self):
        """A short summary, or ``"The plan matches the model."``."""
        if self.is_current:
            return "The plan matches the model."
        lines = []
        if self.missing:
            lines.append("in the plan but not in the model: " + ", ".join(self.missing))
        if self.new:
            lines.append("in the model but not in the plan: " + ", ".join(self.new))
        for key, parts in sorted(self.changed.items()):
            lines.append("{} changed: {}".format(key, " and ".join(parts)))
        return "\n".join(lines)


def plan_sequence(
    model,
    overrides=None,
    strategy=None,
    width=DEFAULT_BEAM_WIDTH,
    distance=APPROACH_DISTANCE,
    use_geometry=True,
    tolerance=KEY_TOLERANCE,
):
    """Sequence a model into a :class:`SequencePlan`. The model is not changed.

    Parameters
    ----------
    model : :class:`~compas_timber.model.TimberModel`
    overrides : :class:`~compas_timber_sequencing.core.overrides.SequenceOverrides`, optional
        Precedences, pins, hand placement and strategy, keyed by element key.
    strategy : :class:`~compas_timber_sequencing.ranking.context.PreferenceStrategy`, optional
        Overrides ``overrides.strategy``. Defaults to gravity.
    width : int, optional
        How many partial orders the search keeps per step.
    distance : float, optional
        Approach distance for the collision check, in model units.
    use_geometry : bool, optional
        Run the collision check; ``False`` trusts the joints alone.
    tolerance : float, optional
        Rounding for geometric keys and fingerprints, in model units.

    Returns
    -------
    :class:`SequencePlan`

    Raises
    ------
    ValueError
        If two beams cannot be told apart (see :func:`~compas_timber_sequencing.timber.keys.element_keys`),
        or the overrides' precedences contradict each other.

    """
    from .. import __version__

    overrides = overrides if overrides is not None else SequenceOverrides()
    if strategy is None and overrides.strategy:
        strategy = make_strategy(overrides.strategy)

    adapter = TimberModelAdapter(model, use_geometry=use_geometry, tolerance=tolerance)
    result = generate(
        adapter.build(),
        manual_set=overrides.by_hand,
        pinned_order=overrides.pins,
        precedences=overrides.precedences,
        strategy=strategy,
        width=width,
        distance=distance,
    )

    vectors = result.insertion_vectors
    steps = {}
    for key in result.order:
        solution = result.extraction[key]
        steps[key] = {
            "insertion_vector": vectors[key],
            "state": solution.state,
            "clearance": solution.angle_degrees if solution.is_feasible else None,
            "by_hand": key in result.manual_set,
            "inferred": solution.inferred_count,
        }

    placed = set(result.order)
    unplaced = sorted(key for key in adapter.elements_by_id if key not in placed)
    stuck = None
    if result.stuck is not None:
        stuck = {"remaining": list(result.stuck.remaining), "reasons": {key: str(reason) for key, reason in result.stuck.blockers.items()}}
    pin_conflict = None
    if result.pin_conflict is not None:
        pin_conflict = {"position": result.pin_conflict.assembly_index, "key": result.pin_conflict.element_id, "reason": result.pin_conflict.reason}

    return SequencePlan(
        order=result.order,
        steps=steps,
        unplaced=unplaced,
        excluded=result.excluded,
        intrinsic_locks=[sorted(group) for group in result.intrinsic_locks],
        proposed_by_hand=sorted(result.proposed_manual_set),
        stuck=stuck,
        pin_conflict=pin_conflict,
        staleness=list(result.staleness),
        fingerprints=fingerprints(model, tolerance, keys=adapter._ids),
        overrides=SequenceOverrides.__from_data__(overrides.__data__),
        settings={
            "strategy": getattr(strategy, "name", None) or "gravity",
            "width": width,
            "distance": distance,
            "use_geometry": use_geometry,
            "tolerance": tolerance,
        },
        version=__version__,
    )


def apply_plan(model, plan, strict=False):
    """Write a plan into the element attributes of a model, for export.

    Elements are matched by key, so this works on a rebuilt model. Every beam's earlier sequence
    attributes are cleared first; then each placed element gets its absolute position, insertion
    vector, hand-placement flag and fit (see :mod:`~compas_timber_sequencing.timber.attributes`).

    Parameters
    ----------
    model : :class:`~compas_timber.model.TimberModel`
    plan : :class:`SequencePlan`
    strict : bool, optional
        Raise instead of writing when the plan does not match the model exactly.

    Returns
    -------
    :class:`PlanCheck`
        How the plan matched the model. Elements missing from the model are skipped; new ones get
        no sequence.

    Raises
    ------
    ValueError
        With `strict`, when the plan is not current for the model.

    """
    check = plan.check(model)
    if strict and not check.is_current:
        raise ValueError("The plan does not match the model:\n" + check.describe())

    keys = element_keys(model, plan.settings.get("tolerance", KEY_TOLERANCE))
    for element in model.elements():
        if not getattr(element, "is_beam", False):
            continue
        clear_attributes(element)
        key = keys[str(element.guid)]
        if key in plan.steps:
            step = plan.steps[key]
            write_attributes(element, plan.position(key), step["insertion_vector"], step["by_hand"], step["state"])
    return check
