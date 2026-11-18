"""Adapter between :class:`~compas_timber.model.TimberModel` and the sequencer.

Together with :mod:`compas_timber_sequencing.kinematics`, which computes each joint's
constraints, this is the only part of the package that imports ``compas_timber``; the rest
depends on ``compas`` alone. The adapter's job is to flatten a model into a
:class:`~compas_timber_sequencing.core.boundary.SequencingInput` and to write a
:class:`~compas_timber_sequencing.core.result.SequenceResult` back onto the model's elements.

Two facts worth stating out loud, because both are limitations a user will meet:

* **Beams only.** Every element that is not a beam is excluded and reported as such.
  Sequencing it would need a different notion of insertion than "slide a stick along a
  vector", and giving it a height of zero -- which sinks them to the bottom of
  the ranking while looking like a real measurement -- is worse than leaving them out and
  saying so.
* **Excluded elements are invisible to the joint kinematics** but are still treated as
  present by the swept broad-phase check, which is the conservative reading.

"""

from compas.geometry import Line
from compas.geometry import Vector

from ..core.boundary import SequencingInput
from ..core.constraints import HalfSpace
from ..core.constraints import SignedAxis
from ..core.search import generate as generate_sequence
from ..core.solver import APPROACH_DISTANCE
from ..kinematics import is_fallback
from ..kinematics import kinematic_constraint
from .attributes import read_manual_set
from .attributes import write_result
from .collision import SWEEP_START_FRACTION
from .collision import bounds_overlap
from .collision import box_bounds
from .collision import swept_aabb
from .collision import swept_box_hits_box


def _element_id(element):
    return str(element.guid)


def constraints_from_joint_output(raw, inferred=False):
    """Convert what a joint returns into typed constraints.

    Parameters
    ----------
    raw : :class:`compas.geometry.Line` or :class:`compas.geometry.Vector` or list
        A ``Line`` is a signed 1-DOF axis, oriented ``start`` -> ``end``. A ``Vector`` is
        the inward normal of a permitted half-space. A list or tuple is any mixture of the
        two.
    inferred : bool, optional
        Tag the results as having come from the generic fallback.

    Returns
    -------
    list of :class:`~compas_timber_sequencing.core.constraints.Constraint`

    Raises
    ------
    TypeError
        On anything else -- including the ``Plane`` that four docstrings in this package
        used to promise but no implementation ever returned. Silently dropping an
        unrecognised constraint makes an element look *more* free than it is.

    """
    if isinstance(raw, Line):
        return [SignedAxis.from_line(raw, inferred=inferred)]
    if isinstance(raw, Vector):
        return [HalfSpace(raw, inferred=inferred)]
    if isinstance(raw, (list, tuple)):
        constraints = []
        for item in raw:
            constraints.extend(constraints_from_joint_output(item, inferred=inferred))
        return constraints
    raise TypeError(
        "Cannot interpret {!r} as a kinematic constraint. Expected a Line (signed 1-DOF axis), a Vector (half-space normal), or a list of those.".format(type(raw).__name__)
    )


class TimberModelAdapter(object):
    """Builds a :class:`~compas_timber_sequencing.core.boundary.SequencingInput` from a model.

    Parameters
    ----------
    model : :class:`~compas_timber.model.TimberModel`
    use_geometry : bool, optional
        Supply the swept broad-phase callback. Without it the sequencer relies on joint
        constraints alone and can be fooled by an element that is in the way but not
        jointed. Turning it off is useful when element geometry cannot be computed.

    Attributes
    ----------
    elements_by_id : dict
        Maps element id -> element, for every sequenceable element.
    excluded : dict
        Maps element id -> reason, for everything left out.

    """

    def __init__(self, model, use_geometry=True):
        self.model = model
        self.use_geometry = use_geometry
        self.elements_by_id = {}
        self.excluded = {}
        self._excluded_elements = {}
        self._bounds_cache = {}
        self._collect()

    def _collect(self):
        for element in self.model.elements():
            element_id = _element_id(element)
            if getattr(element, "is_group_element", False):
                self.excluded[element_id] = "group containers are not sequenced"
                self._excluded_elements[element_id] = element
            elif getattr(element, "is_beam", False):
                self.elements_by_id[element_id] = element
            else:
                self.excluded[element_id] = "{} elements are not sequenced; beams only, for now".format(type(element).__name__.lower())
                self._excluded_elements[element_id] = element

    def _joints_for(self, element):
        return self.model.get_joints_for_element(element)

    def _neighbors(self):
        neighbors = {element_id: set() for element_id in self.elements_by_id}
        for element_id, element in self.elements_by_id.items():
            for joint in self._joints_for(element):
                for other in joint.elements:
                    other_id = _element_id(other)
                    if other_id != element_id and other_id in self.elements_by_id:
                        neighbors[element_id].add(other_id)
        # Enforce symmetry: a joint recorded on one side only would otherwise be a
        # one-directional constraint, which is not a thing.
        for element_id, others in list(neighbors.items()):
            for other_id in others:
                neighbors[other_id].add(element_id)
        return neighbors

    def _joint_members(self):
        members = {}
        for joint in self.model.joints:
            member_ids = tuple(_element_id(element) for element in joint.elements)
            if any(member_id in self.elements_by_id for member_id in member_ids):
                members[str(joint.guid)] = member_ids
        return members

    def _groups(self):
        groups = {}
        for element in self.model.elements():
            if not getattr(element, "is_group_element", False):
                continue
            label = _element_id(element)
            for child in element.children or ():
                child_id = _element_id(child)
                if child_id in self.elements_by_id:
                    groups[child_id] = label
        return groups

    def constraints(self, element_id, active_neighbor_ids):
        """Gather the constraints on one element from its still-active joints.

        A joint contributes nothing once every other member of it has been removed, which
        is what makes an n-ary joint's contribution depend on the active set rather than on
        the joint alone.

        """
        element = self.elements_by_id[element_id]
        constraints = []
        for joint in self._joints_for(element):
            partners = set(_element_id(other) for other in joint.elements) - {element_id}
            if not partners & set(active_neighbor_ids):
                continue
            # Joints without a registered implementation get the permissive fallback; its
            # constraints are tagged so results can report how much of the answer was a guess.
            raw = kinematic_constraint(joint, element)
            constraints.extend(constraints_from_joint_output(raw, inferred=is_fallback(joint)))
        return constraints

    def _bounds(self, element_id):
        if element_id not in self._bounds_cache:
            element = self.elements_by_id.get(element_id) or self._excluded_elements[element_id]
            self._bounds_cache[element_id] = box_bounds(element.obb)
        return self._bounds_cache[element_id]

    def path_is_clear(self, element_id, direction, distance, active_ids):
        """Swept broad-phase against every other element still in place.

        Jointed neighbours are skipped: their interaction with this element is already
        described exactly by the joint constraints, and a beam sliding along a joint face
        would otherwise veto itself. The check exists to catch the elements a joint cannot
        see -- in a dense lattice a beam can be kinematically free per its joints while its
        extraction path passes straight through a beam it is not jointed to.

        Excluded elements are treated as present, which is the conservative reading.

        Two phases. Axis-aligned bounds around the swept box reject the elements that are
        nowhere near, cheaply; whatever survives goes to :func:`~compas_timber_sequencing.timber.collision.swept_box_hits_box`, which
        is an exact separating-axis test between the swept oriented box and the other
        element's oriented box.

        The narrow phase is not an optimization, it is the answer. Axis-aligned bounds
        around a diagonal brace enclose several times the brace's own volume, nearly all of
        it empty, and the beam whose extraction path crosses that empty corner is reported
        obstructed when nothing is in its way. Erring towards "obstructed" is the safe
        direction for a robot, but only where the obstruction is real: a false veto on the
        highest elements is what drives a bottom-up sequence to give up on them and start
        pulling posts out from under beams that are still standing.

        What remains approximate is the *element*, not the test. An element is taken as its
        oriented bounding box, so a beam is treated as solid to its full section and its
        processings are ignored. That still errs towards "obstructed".

        """
        element = self.elements_by_id[element_id]
        try:
            box = element.obb
            swept = swept_aabb(box, direction, distance)
        except Exception:  # geometry unavailable, e.g. an element without a blank
            return True

        near = distance * SWEEP_START_FRACTION
        neighbor_ids = set(_element_id(other) for joint in self._joints_for(element) for other in joint.elements)
        candidates = (set(active_ids) - neighbor_ids - {element_id}) | set(self._excluded_elements)

        for other_id in sorted(candidates, key=str):
            try:
                other_bounds = self._bounds(other_id)
            except Exception:
                continue
            if not bounds_overlap(swept, other_bounds):
                continue
            other = self.elements_by_id.get(other_id) or self._excluded_elements[other_id]
            try:
                other_box = other.obb
            except Exception:
                # No geometry to refine the broad-phase hit with, so keep the hit.
                return False
            if swept_box_hits_box(box, direction, near, distance, other_box):
                return False
        return True

    def build(self):
        """Build the sequencing input.

        Returns
        -------
        :class:`~compas_timber_sequencing.core.boundary.SequencingInput`

        """
        element_ids = sorted(self.elements_by_id, key=lambda i: str(i))
        base_z = {}
        centroid_z = {}
        length = {}
        for element_id in element_ids:
            centerline = self.elements_by_id[element_id].centerline
            base_z[element_id] = min(centerline.start.z, centerline.end.z)
            centroid_z[element_id] = centerline.midpoint.z
            length[element_id] = centerline.length

        return SequencingInput(
            element_ids=element_ids,
            neighbors=self._neighbors(),
            joint_members=self._joint_members(),
            base_z=base_z,
            centroid_z=centroid_z,
            length=length,
            constraints=self.constraints,
            path_is_clear=self.path_is_clear if self.use_geometry else None,
            groups=self._groups(),
            excluded=self.excluded,
        )

    def manual_set_from_model(self):
        """Read the persisted hand-placement set off the element attributes.

        Returns
        -------
        set

        """
        return read_manual_set(self.elements_by_id)

    def apply(self, result):
        """Write a sequence result back onto the model's elements.

        Everything goes into ``element.attributes``, which ``compas_timber`` serializes with
        the element, so the sequence survives a round trip through JSON. Read it back in
        order with :func:`assembly_order`.

        Parameters
        ----------
        result : :class:`~compas_timber_sequencing.core.result.SequenceResult`

        """
        write_result(self.elements_by_id, result)


def sequencing_input_from_model(model, use_geometry=True):
    """Flatten a timber model into a sequencing input.

    Parameters
    ----------
    model : :class:`~compas_timber.model.TimberModel`
    use_geometry : bool, optional

    Returns
    -------
    :class:`~compas_timber_sequencing.core.boundary.SequencingInput`

    """
    return TimberModelAdapter(model, use_geometry=use_geometry).build()


def generate_assembly_sequence(
    model,
    manual_set=None,
    pinned_order=None,
    strategy=None,
    width=None,
    distance=APPROACH_DISTANCE,
    use_geometry=True,
    apply_to_model=True,
):
    """Sequence a timber model and, by default, write the result onto it.

    Parameters
    ----------
    model : :class:`~compas_timber.model.TimberModel`
    manual_set : iterable, optional
        Element ids (``str(element.guid)``) to treat as hand-placed. Defaults to whatever
        is already persisted on the elements, so that an amended set survives a re-run.
    pinned_order : dict or list, optional
        Element id -> assembly index, or a list of element ids in assembly order.
    strategy : :class:`~compas_timber_sequencing.ranking.context.PreferenceStrategy`, optional
    width : int, optional
        Beam width.
    distance : float, optional
        Approach distance for the swept broad-phase check, in model units.
    use_geometry : bool, optional
        Whether to run the swept broad-phase check at all.
    apply_to_model : bool, optional
        Write the sequence onto element and graph attributes.

    Returns
    -------
    tuple
        ``(result, adapter)``. The adapter carries the id -> element mapping needed to turn
        ids back into elements.

    """
    adapter = TimberModelAdapter(model, use_geometry=use_geometry)
    sequencing_input = adapter.build()

    if manual_set is None:
        manual_set = adapter.manual_set_from_model()

    kwargs = {"manual_set": manual_set, "pinned_order": pinned_order, "strategy": strategy, "distance": distance}
    if width is not None:
        kwargs["width"] = width

    result = generate_sequence(sequencing_input, **kwargs)

    if apply_to_model:
        adapter.apply(result)

    return result, adapter
