"""Where a sequence is stored on the model: four keys in each element's ``attributes``.

``compas_timber`` serializes ``element.attributes`` with the element, so everything written here
survives a round trip through JSON. Nothing else in the model is touched.

"""

MANUAL_ATTRIBUTE = "requires_manual_assembly"
"""str: Element attribute holding the hand-placement flag.

Hand placement is a fact about the design, so it lives on the element and is serialized
with the model. Pinned order is a fact about a particular build and is supplied per run.
"""

SEQUENCE_ATTRIBUTE = "assembly_sequence"
"""str: Element attribute holding the element's index in the assembly order."""

VECTOR_ATTRIBUTE = "insertion_vector"
"""str: Element attribute holding the unit insertion vector, or ``None`` when hand-placed."""

STATE_ATTRIBUTE = "extraction_state"
"""str: Element attribute holding ``"roomy"``, ``"tight"`` or ``"locked"``."""


ATTRIBUTES = (SEQUENCE_ATTRIBUTE, VECTOR_ATTRIBUTE, MANUAL_ATTRIBUTE, STATE_ATTRIBUTE)
"""tuple of str: Every attribute this package writes."""


def clear_attributes(element):
    """Remove whatever an earlier sequence wrote on an element."""
    for key in ATTRIBUTES:
        element.attributes.pop(key, None)


def write_attributes(element, position, insertion_vector, by_hand, state):
    """Write one element's place in a sequence.

    Parameters
    ----------
    element : :class:`~compas_timber.elements.TimberElement`
    position : int
        Absolute position in the assembly order, from 0.
    insertion_vector : :class:`compas.geometry.Vector` or None
    by_hand : bool
    state : str

    """
    element.attributes[SEQUENCE_ATTRIBUTE] = position
    element.attributes[VECTOR_ATTRIBUTE] = insertion_vector
    element.attributes[MANUAL_ATTRIBUTE] = by_hand
    element.attributes[STATE_ATTRIBUTE] = state


def write_result(elements_by_id, result):
    """Write a sequence result onto the elements.

    Every element's earlier values are cleared first, so nothing from a previous run survives.
    After a run that got stuck, ``result.order`` holds only the elements the search did place --
    the last ones to be assembled -- so their positions start after the unplaced ones, and the
    unplaced ones get no position at all.

    Parameters
    ----------
    elements_by_id : dict
        Element id -> element, for every sequenceable element.
    result : :class:`~compas_timber_sequencing.core.result.SequenceResult`

    """
    for element in elements_by_id.values():
        clear_attributes(element)
    offset = len(elements_by_id) - len(result.order)
    vectors = result.insertion_vectors
    for index, element_id in enumerate(result.order):
        write_attributes(
            elements_by_id[element_id],
            offset + index,
            vectors[element_id],
            element_id in result.manual_set,
            result.extraction[element_id].state,
        )


def read_manual_set(elements_by_id):
    """The ids of the elements flagged as hand-placed.

    Parameters
    ----------
    elements_by_id : dict
        Element id -> element.

    Returns
    -------
    set

    """
    manual = set()
    for element_id, element in elements_by_id.items():
        if element.attributes.get(MANUAL_ATTRIBUTE):
            manual.add(element_id)
    return manual


def assembly_order(model):
    """The elements of a model that carry an assembly index, in assembly order.

    Reads what :func:`write_result` wrote. Elements without an index -- excluded from
    sequencing, or never sequenced -- are left out.

    Parameters
    ----------
    model : :class:`~compas_timber.model.TimberModel`

    Returns
    -------
    list of :class:`~compas_timber.elements.TimberElement`

    """
    sequenced = []
    for element in model.elements():
        index = element.attributes.get(SEQUENCE_ATTRIBUTE)
        if index is not None:
            sequenced.append((index, str(element.guid), element))
    return [element for _, _, element in sorted(sequenced, key=lambda item: (item[0], item[1]))]
