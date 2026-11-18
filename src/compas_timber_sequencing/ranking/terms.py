"""The named ranking terms that :class:`~compas_timber_sequencing.ranking.strategies.TermStrategy` and
:class:`~compas_timber_sequencing.ranking.strategies.WeightedStrategy` compose.

Each term is ``f(context, element_id, solution, ranges) -> float`` on 0..1, where **higher means
removed earlier in disassembly**, which is *later* in assembly. See :data:`TERMS`.

"""

from ..core.result import ROOMY


class _Ranges(object):
    """The min and max of each per-element quantity in one model.

    Terms are normalized against these so that a height in millimetres and a length in
    metres can appear in the same weighted sum without one drowning the other. The
    normalization is per-model, so a term value is only comparable within one run.

    """

    def __init__(self, sequencing_input):
        element_ids = sequencing_input.element_ids
        self.spans = {
            "base_z": _span(sequencing_input.base_z[i] for i in element_ids),
            "centroid_z": _span(sequencing_input.centroid_z[i] for i in element_ids),
            "length": _span(sequencing_input.length[i] for i in element_ids),
            "degree": _span(sequencing_input.degree(i) for i in element_ids),
        }

    def unit(self, name, value):
        """Map `value` onto 0..1 against the model's span for `name`.

        Parameters
        ----------
        name : str
        value : float

        Returns
        -------
        float

        """
        return _unit(value, self.spans[name])


def _span(values):
    values = [float(value) for value in values]
    if not values:
        return (0.0, 0.0)
    return (min(values), max(values))


def _unit(value, span):
    low, high = span
    if high <= low:
        # Every element carries the same value, so the term cannot separate anything.
        # Returning a constant makes it inert and lets the next term decide, rather than
        # dividing by zero or manufacturing an ordering out of nothing.
        return 0.5
    return (float(value) - low) / (high - low)


def _term_stable(context, element_id, solution, ranges):
    return 0.0 if element_id in context.disconnecting else 1.0


def _term_clear_above(context, element_id, solution, ranges):
    return 0.0 if context.input.above(element_id) & context.active_ids else 1.0


def _term_base_z(context, element_id, solution, ranges):
    return ranges.unit("base_z", context.input.base_z[element_id])


def _term_centroid_z(context, element_id, solution, ranges):
    return ranges.unit("centroid_z", context.input.centroid_z[element_id])


def _term_length(context, element_id, solution, ranges):
    return ranges.unit("length", context.input.length[element_id])


def _term_degree(context, element_id, solution, ranges):
    return ranges.unit("degree", context.input.degree(element_id))


def _term_roomy(context, element_id, solution, ranges):
    return 1.0 if solution.state == ROOMY else 0.0


def _term_margin(context, element_id, solution, ranges):
    if not solution.is_feasible:
        # A Locked result reaches ranking only for hand-placed elements, and its margin is
        # NaN by design. NaN in a sort key compares false against everything and would
        # silently scramble the order, so it is mapped to the worst finite value here.
        return 0.0
    margin = solution.margin
    if margin != margin:
        return 0.0
    return max(0.0, min(1.0, margin))


def _term_certain(context, element_id, solution, ranges):
    return 0.0 if solution.inferred_count else 1.0


def _term_subassembly(context, element_id, solution, ranges):
    if context.last_removed is None:
        return 0.0
    return 1.0 if context.subassemblies.get(element_id) == context.subassemblies.get(context.last_removed) else 0.0


def _term_chain(context, element_id, solution, ranges):
    if context.last_removed is None:
        return 0.0
    return 1.0 if context.last_removed in context.input.neighbors[element_id] else 0.0


def _term_locality(context, element_id, solution, ranges):
    if context.last_removed is None:
        return 0.5
    neighbor_ids = context.input.neighbors[element_id]
    if context.last_removed in neighbor_ids:
        return 1.0
    if neighbor_ids & context.input.neighbors[context.last_removed]:
        return 0.5
    return 0.0


def _term_freed(context, element_id, solution, ranges):
    neighbor_ids = context.input.neighbors[element_id]
    if not neighbor_ids:
        return 1.0
    return float(len(neighbor_ids - set(context.active_ids))) / float(len(neighbor_ids))


TERMS = {
    "base_z": _term_base_z,
    "centroid_z": _term_centroid_z,
    "certain": _term_certain,
    "chain": _term_chain,
    "clear_above": _term_clear_above,
    "degree": _term_degree,
    "freed": _term_freed,
    "length": _term_length,
    "locality": _term_locality,
    "margin": _term_margin,
    "roomy": _term_roomy,
    "stable": _term_stable,
    "subassembly": _term_subassembly,
}
"""dict: The named ranking terms, each ``f(context, element_id, solution, ranges) -> float``.

Every term returns a value on 0..1 where **higher means removed earlier in disassembly**,
which is *later* in assembly. Read every term below in that direction; the assembly
consequence is the mirror image.

===============  ========================================================================
``stable``       1.0 unless removing this element would strand something from the ground.
``clear_above``  1.0 unless a jointed neighbour whose base sits higher is still in place.
``base_z``       Height of the element's lowest point, normalized over the model.
``centroid_z``   Height of the element's centroid, normalized over the model.
``length``       Element length, normalized over the model.
``degree``       Number of jointed neighbours, normalized. Negate for peripheral-first.
``roomy``        1.0 for a comfortable clearance, 0.0 for a zero-clearance slot.
``margin``       The clearance angle's sine, a continuous version of ``roomy``.
``certain``      1.0 when no constraint behind this result came from a generic fallback.
``subassembly``  1.0 when the element is in the same cluster as the last one removed.
``chain``        1.0 when the element is jointed to the last one removed.
``locality``     1.0 if jointed to the last removed, 0.5 if two hops away, else 0.0.
``freed``        Fraction of this element's neighbours that are already gone.
===============  ========================================================================

Prefix a name with ``-`` to invert it: ``"-degree"`` prefers peripheral elements,
``"-length"`` prefers short ones.

"""


def _resolve_terms(names):
    """Turn term names, optionally ``-``-prefixed, into ``(sign, function)`` pairs."""
    resolved = []
    for name in names:
        sign = 1.0
        key = name
        if key.startswith("-"):
            sign = -1.0
            key = key[1:]
        if key not in TERMS:
            raise ValueError("Unknown ranking term {!r}. Known terms: {}.".format(name, ", ".join(sorted(TERMS))))
        resolved.append((sign, TERMS[key]))
    return resolved
