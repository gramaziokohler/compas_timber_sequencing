"""What a ranking strategy is given, what it must return, and how candidates are ranked.

A strategy scores one candidate removal at a time against a :class:`RankingContext`;
:func:`rank` orders the candidates by those scores. Hard constraints filter (in
:mod:`compas_timber_sequencing.core.search`); strategies only order what is already permitted.

"""

from ..core.boundary import sort_key


class RankingContext(object):
    """The state a strategy may score against.

    Parameters
    ----------
    sequencing_input : :class:`~compas_timber_sequencing.core.boundary.SequencingInput`
    active_ids : set
        Elements still in place, before this step's removal.
    last_removed : hashable or None
        The element removed at the previous step.
    subassemblies : dict
        Element id -> subassembly label.
    disconnecting : set
        Elements whose removal would strand part of the assembly from the ground, from
        :func:`~compas_timber_sequencing.core.blocking.disconnecting_elements`.
    step_index : int

    """

    def __init__(self, sequencing_input, active_ids, last_removed, subassemblies, disconnecting, step_index):
        self.input = sequencing_input
        self.active_ids = active_ids
        self.last_removed = last_removed
        self.subassemblies = subassemblies
        self.disconnecting = disconnecting
        self.step_index = step_index


class PreferenceStrategy(object):
    """Base class for ranking strategies."""

    name = "preference"

    def score(self, context, element_id, solution):
        """Score a candidate removal. Higher is better.

        Parameters
        ----------
        context : :class:`RankingContext`
        element_id : hashable
        solution : :class:`~compas_timber_sequencing.core.result.Solution` or :class:`~compas_timber_sequencing.core.result.Locked`
            The extraction result for this candidate in this state. Hand-placed elements
            may carry a ``Locked``.

        Returns
        -------
        tuple
            A comparable tuple. All candidates at a step must return tuples of the same
            shape.

        """
        raise NotImplementedError


def rank(context, candidates, strategy):
    """Order candidate removals, best first.

    Parameters
    ----------
    context : :class:`RankingContext`
    candidates : dict
        Element id -> extraction result.
    strategy : :class:`PreferenceStrategy`

    Returns
    -------
    list of tuple
        ``(score, element_id)`` pairs, best first. The element id is appended to the sort
        key so that equal scores resolve the same way on every run.

    """
    scored = [(strategy.score(context, element_id, result), element_id) for element_id, result in candidates.items()]
    scored.sort(key=lambda item: (item[0], sort_key(item[1])), reverse=True)
    return scored
