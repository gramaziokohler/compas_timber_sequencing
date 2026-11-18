"""Injectable ranking strategies.

Hard constraints filter; soft preferences rank. The feasibility filter in
:mod:`compas_timber_sequencing.core.search` decides *whether* an element may be removed at a step;
a strategy here decides only *which of the permitted ones* is preferred. Blending the two
into one comparator, as a single lexicographic tuple of booleans and continuous values,
means a constraint violation can be outvoted by an ordering accident.

Bottom-up gravity is the default strategy, not the only expressible one -- this package is
meant to hold several sequencing algorithms.

What is here
------------
Every strategy is stated in **disassembly** terms, because that is what the search does:
a higher score means *removed earlier*, which means *placed later* in the assembly order
the caller gets back. Each docstring names the assembly consequence out loud, since that
is the sentence a fabricator cares about.

============================  ==============================================================
:class:`GravityStrategy`      Bottom-up assembly. The default.
:class:`LayeredStrategy`      Bottom-up, course by course, with height quantized.
:class:`SubassemblyStrategy`  One cluster finished before the next is started.
:class:`ChainStrategy`        Stay where you are standing; minimize travel.
:class:`ClearanceStrategy`    Tight fits placed first, into the emptiest site.
:class:`SkeletonFirstStrategy` Long, well-connected members first; infill last.
:class:`RandomStrategy`       A deterministic shuffle, as a control.
:class:`TermStrategy`         Lexicographic over named terms, composed by the caller.
:class:`WeightedStrategy`     A weighted sum of named terms, for tuning.
:class:`HeuristicStrategy`    A caller-supplied scalar function of the element id.
============================  ==============================================================

The named terms the last two compose over are listed in :data:`TERMS`. There is no single
best strategy: which one wins is a property of the model, which is why they are injectable
and why :mod:`compas_timber_sequencing.ranking.compare` exists to score them side by side.

"""

import math
import zlib

from ..core.boundary import sort_key
from ..core.result import ROOMY
from .context import PreferenceStrategy
from .terms import TERMS
from .terms import _Ranges
from .terms import _resolve_terms
from .terms import _term_stable


class GravityStrategy(PreferenceStrategy):
    """Bottom-up assembly, which is top-down disassembly.

    The score, most significant term first:

    1. **Does not strand anything from the ground.** A topological proxy for stability,
       not a stability calculation, and the one term whose violation produces obvious
       nonsense -- so it leads.
    2. **Nothing jointed to it is still standing higher up**, from
       :meth:`~compas_timber_sequencing.core.boundary.SequencingInput.above`. Comparing heights
       *within a step* is not the same as sorting by height: at any step the tallest
       element left may be kinematically blocked, and a comparator that only sorts by
       ``base_z`` then reaches straight past it and takes a post out from under a beam
       that is still up there. Read forwards, that put the beam into the model before the
       post that carries it. This term is what forbids it: a candidate with a jointed
       neighbour still above it ranks below every candidate without one, whatever their
       heights.

       It is a preference and not a filter on purpose. When *every* remaining candidate
       is carrying something -- an interlock, a blocked extraction overhead -- the search
       needs a sequence more than it needs this rule, so the term goes quiet and the
       height terms below decide. That fallback is visible: ``support_inversions`` in
       :mod:`compas_timber_sequencing.ranking.compare` counts the times it fired.
    3. **base_z, then centroid_z**, both descending, as two separate lexicographic terms.
       Both carry real signal: centroid alone ranks a ground-standing post as "high",
       base alone cannot separate two beams that start at the same level. Summing them,
       as an earlier implementation did, produces a quantity with no physical meaning and
       no nameable unit -- and was then used simultaneously as a hard gate and a
       continuous sort key.
    4. **Roomy before tight.** Free clearance is strictly better than a zero-clearance
       slot, so spend the roomy extractions while there is still room.
    5. **Subassembly continuity** -- finish a cluster before starting another.
    6. **Chain continuity** -- prefer a neighbour of the last element removed.
    7. **Length**, then **low connectivity**, as tiebreaks. Long members and peripheral
       members come off first.

    """

    name = "gravity"

    def score(self, context, element_id, solution):
        sequencing_input = context.input

        stable = element_id not in context.disconnecting
        clear_above = not (sequencing_input.above(element_id) & context.active_ids)
        roomy = solution.state == ROOMY

        same_subassembly = False
        in_chain = False
        if context.last_removed is not None:
            same_subassembly = context.subassemblies.get(element_id) == context.subassemblies.get(context.last_removed)
            in_chain = context.last_removed in sequencing_input.neighbors[element_id]

        return (
            int(stable),
            int(clear_above),
            sequencing_input.base_z[element_id],
            sequencing_input.centroid_z[element_id],
            int(roomy),
            int(same_subassembly),
            int(in_chain),
            sequencing_input.length[element_id],
            -sequencing_input.degree(element_id),
        )


class HeuristicStrategy(PreferenceStrategy):
    """Rank by a caller-supplied scalar function of the element id.

    An escape hatch for callers that want to express one idea -- "tallest first",
    "longest first" -- without subclassing.

    Parameters
    ----------
    function : callable
        ``f(element_id) -> float``. Higher scores are removed earlier.
    name : str, optional

    """

    def __init__(self, function, name="heuristic"):
        self.function = function
        self.name = name

    def score(self, context, element_id, solution):
        return (float(self.function(element_id)),)


class TermStrategy(PreferenceStrategy):
    """Rank lexicographically by a named list of terms.

    The first term decides; the rest only break ties in the terms before them. This is the
    shape most of the shipped strategies take, and the one to reach for when the terms
    genuinely rank -- when no amount of the second term should ever outvote the first. Use
    :class:`WeightedStrategy` when they should trade against each other instead.

    Parameters
    ----------
    terms : iterable of str, optional
        Names from :data:`TERMS`, most significant first, each optionally ``-``-prefixed
        to invert it. Defaults to the subclass's :attr:`terms`.
    name : str, optional

    Examples
    --------
    >>> strategy = TermStrategy(["stable", "base_z", "-degree"], name="my-order")
    >>> strategy.name
    'my-order'

    """

    terms = ()
    name = "terms"

    def __init__(self, terms=None, name=None):
        if terms is not None:
            self.terms = tuple(terms)
        if name is not None:
            self.name = name
        if not self.terms:
            raise ValueError("A TermStrategy needs at least one term. Known terms: {}.".format(", ".join(sorted(TERMS))))
        self._resolved = _resolve_terms(self.terms)
        self._ranges = None
        self._ranges_input = None

    def __repr__(self):
        return "{}({!r})".format(type(self).__name__, list(self.terms))

    def ranges(self, sequencing_input):
        """The normalization spans for `sequencing_input`, computed once per model.

        Parameters
        ----------
        sequencing_input : :class:`~compas_timber_sequencing.core.boundary.SequencingInput`

        Returns
        -------
        :class:`_Ranges`

        """
        if self._ranges_input is not sequencing_input:
            self._ranges = _Ranges(sequencing_input)
            self._ranges_input = sequencing_input
        return self._ranges

    def score(self, context, element_id, solution):
        """Score a candidate removal. Higher is better.

        Parameters
        ----------
        context : :class:`RankingContext`
        element_id : hashable
        solution : :class:`~compas_timber_sequencing.core.result.Solution` or :class:`~compas_timber_sequencing.core.result.Locked`

        Returns
        -------
        tuple

        """
        ranges = self.ranges(context.input)
        return tuple(sign * function(context, element_id, solution, ranges) for sign, function in self._resolved)


class ClearanceStrategy(TermStrategy):
    """Spend the clearance while there is still clearance.

    Ranks by how much room an extraction has, ahead of height. In disassembly the roomiest
    elements come off first, which means **the tight fits are placed first in assembly**,
    into the emptiest site. This is :class:`GravityStrategy`'s fourth term promoted to the
    top.

    Worth trying when a model is full of slots and housings and the gravity ordering leaves
    the awkward fits until the site is crowded. Expect it to argue with gravity: it will
    take a low element off before a high one if the low one has more room, so ``stable``
    still leads to keep the result from floating.

    """

    name = "clearance"
    terms = ("stable", "roomy", "margin", "base_z", "centroid_z", "subassembly", "chain", "-degree")


class SkeletonFirstStrategy(TermStrategy):
    """Structure first, infill last.

    Peripheral and short elements are removed first in disassembly, so **the long,
    highly-connected members are placed first in assembly**: posts and sills go up as a
    frame, then studs, braces and blocking fill it in.

    This is how a carpenter describes the job, and it is the strategy most likely to
    disagree with :class:`GravityStrategy` in an interesting way -- a full-height post
    outranks the low blocking beside it. ``stable`` still leads, so nothing is asked to
    float, but expect more height inversions than gravity gives.

    It also leans harder than the others on what ``stable`` actually measures, which is
    connectivity to the ground and not stability: an element hanging off a member above it
    counts as connected. Check the order this one produces against a temporary-support plan
    before trusting it on site.

    """

    name = "skeleton"
    terms = ("stable", "-degree", "-length", "base_z", "roomy", "subassembly", "chain")


class SubassemblyStrategy(TermStrategy):
    """One cluster at a time.

    Cluster continuity outranks height, so a subassembly is finished before the next one is
    started. Within a cluster, the ordering is the gravity ordering.

    For prefabrication: it produces a sequence that can be cut at cluster boundaries into
    separately buildable panels or trusses. Declared groups win over inferred clusters in
    :func:`~compas_timber_sequencing.core.blocking.subassemblies`, so this is also the strategy that
    follows a user's own grouping most directly.

    """

    name = "subassembly"
    terms = ("stable", "subassembly", "chain", "base_z", "centroid_z", "roomy", "length", "-degree")


class ChainStrategy(TermStrategy):
    """Keep working where you are standing.

    Prefers an element jointed to the last one moved, then one two hops away, then one that
    is already mostly free of its neighbours. Height is only a tiebreak.

    A travel-minimizing order: it keeps the robot, or the crew, in one region of the model
    instead of crossing it every step. It has no notion of distance beyond the joint graph,
    since the sequencing boundary carries no positions other than heights -- adjacency is
    the proxy, and it is a good one for framing, a poor one for a model whose joints skip
    across the structure.

    """

    name = "chain"
    terms = ("stable", "locality", "freed", "base_z", "roomy", "length", "-degree")


class LayeredStrategy(TermStrategy):
    """Course by course.

    Height is quantized into bands, so elements within one course rank as equals and are
    ordered by cluster and chain continuity instead. Between courses the higher course is
    removed first, so **assembly runs bottom course to top course**, finishing each before
    starting the next.

    That is the difference from :class:`GravityStrategy`, which reads raw height and will
    hop back and forth across a course to chase a few millimetres of z. Give a `tolerance`
    in model units -- a storey height, a course depth -- or leave it to be derived by
    splitting the model's height into `bands` equal courses.

    Parameters
    ----------
    tolerance : float, optional
        Band height in model units. Defaults to None, meaning derive it from `bands`.
    bands : int, optional
        How many equal courses to split the model into when `tolerance` is None.
        Default is 4.
    terms : iterable of str, optional
        How to rank *within* a course.
    name : str, optional

    """

    name = "layered"
    terms = ("subassembly", "chain", "centroid_z", "roomy", "length", "-degree")

    def __init__(self, tolerance=None, bands=4, terms=None, name=None):
        super(LayeredStrategy, self).__init__(terms=terms, name=name)
        if tolerance is not None and float(tolerance) <= 0.0:
            raise ValueError("tolerance must be positive, got {!r}.".format(tolerance))
        if int(bands) < 1:
            raise ValueError("bands must be at least 1, got {!r}.".format(bands))
        self.tolerance = None if tolerance is None else float(tolerance)
        self.bands = int(bands)

    def __repr__(self):
        return "LayeredStrategy(tolerance={!r}, bands={!r})".format(self.tolerance, self.bands)

    def band(self, base_z, ranges):
        """Which course `base_z` falls in. Higher is further up.

        Parameters
        ----------
        base_z : float
        ranges : :class:`_Ranges`

        Returns
        -------
        float

        """
        low, high = ranges.spans["base_z"]
        if self.tolerance is not None:
            return float(int(math.floor((float(base_z) - low) / self.tolerance)))

        span = high - low
        if span <= 0.0:
            # A flat model: everything is one course and the term goes inert.
            return 0.0
        index = int(math.floor((float(base_z) - low) * self.bands / span))
        # The highest element sits exactly on the top edge and would otherwise land in a
        # course of its own, giving `bands + 1` courses of which the last holds one
        # element. Asking for four courses has to produce four.
        return float(min(index, self.bands - 1))

    def score(self, context, element_id, solution):
        """Score a candidate removal. Higher is better.

        Parameters
        ----------
        context : :class:`RankingContext`
        element_id : hashable
        solution : :class:`~compas_timber_sequencing.core.result.Solution` or :class:`~compas_timber_sequencing.core.result.Locked`

        Returns
        -------
        tuple

        """
        ranges = self.ranges(context.input)
        stable = _term_stable(context, element_id, solution, ranges)
        band = self.band(context.input.base_z[element_id], ranges)
        return (stable, band) + super(LayeredStrategy, self).score(context, element_id, solution)


class WeightedStrategy(PreferenceStrategy):
    """Trade the terms against each other with explicit weights.

    Where :class:`TermStrategy` lets no term outvote the one before it, this one sums them:
    ``sum(weight * term)``, collapsed to a single number. Because every term is normalized
    onto 0..1 against the model, the sum is dimensionless and the weights mean what they
    say -- which is the thing an unweighted ``base_z + centroid_z`` could not claim.

    It is still a blend, so give ``stable`` a weight large enough that nothing outvotes it
    unless you mean to let it. This is the strategy to sweep when tuning by hand.

    Parameters
    ----------
    weights : dict
        Term name -> weight. Names may be ``-``-prefixed and weights may be negative; both
        invert, so use one or the other.
    name : str, optional

    Examples
    --------
    >>> strategy = WeightedStrategy({"stable": 10.0, "base_z": 3.0, "roomy": 1.0})
    >>> sorted(strategy.weights)
    ['base_z', 'roomy', 'stable']

    """

    name = "weighted"

    def __init__(self, weights, name=None):
        if not weights:
            raise ValueError("A WeightedStrategy needs at least one weighted term. Known terms: {}.".format(", ".join(sorted(TERMS))))
        if name is not None:
            self.name = name
        self.weights = dict(weights)
        self._resolved = []
        for term_name in sorted(self.weights):
            sign, function = _resolve_terms([term_name])[0]
            self._resolved.append((sign * float(self.weights[term_name]), function))
        self._ranges = None
        self._ranges_input = None

    def __repr__(self):
        return "WeightedStrategy({!r})".format(self.weights)

    def score(self, context, element_id, solution):
        """Score a candidate removal. Higher is better.

        Parameters
        ----------
        context : :class:`RankingContext`
        element_id : hashable
        solution : :class:`~compas_timber_sequencing.core.result.Solution` or :class:`~compas_timber_sequencing.core.result.Locked`

        Returns
        -------
        tuple
            A one-tuple holding the weighted sum.

        """
        if self._ranges_input is not context.input:
            self._ranges = _Ranges(context.input)
            self._ranges_input = context.input
        total = 0.0
        for weight, function in self._resolved:
            total += weight * function(context, element_id, solution, self._ranges)
        return (total,)


class RandomStrategy(PreferenceStrategy):
    """A deterministic shuffle, as a control.

    Ranks by a hash of the element id, behind stability. It exists to answer the question
    that makes the others meaningful: **does this strategy actually beat an arbitrary
    order?** A heuristic that scores no better than this one on a model is not earning its
    place there.

    It is also the cheapest way to produce genuinely different alternative sequences --
    vary the seed -- when the override workflow wants something else to look at.

    Deterministic across processes and platforms: it hashes with ``crc32`` rather than
    Python's ``hash``, which is salted per process for strings.

    Parameters
    ----------
    seed : int or str, optional
    name : str, optional

    """

    name = "random"

    def __init__(self, seed=0, name=None):
        self.seed = seed
        if name is not None:
            self.name = name

    def __repr__(self):
        return "RandomStrategy(seed={!r})".format(self.seed)

    def score(self, context, element_id, solution):
        """Score a candidate removal. Higher is better.

        Parameters
        ----------
        context : :class:`RankingContext`
        element_id : hashable
        solution : :class:`~compas_timber_sequencing.core.result.Solution` or :class:`~compas_timber_sequencing.core.result.Locked`

        Returns
        -------
        tuple

        """
        stable = 0.0 if element_id in context.disconnecting else 1.0
        token = "{}:{}".format(self.seed, sort_key(element_id)).encode("utf-8")
        return (stable, (zlib.crc32(token) & 0xFFFFFFFF) / float(0xFFFFFFFF))


STRATEGIES = {
    "chain": ChainStrategy,
    "clearance": ClearanceStrategy,
    "gravity": GravityStrategy,
    "layered": LayeredStrategy,
    "random": RandomStrategy,
    "skeleton": SkeletonFirstStrategy,
    "subassembly": SubassemblyStrategy,
}
"""dict: The strategies that can be built with no arguments, by name.

:class:`WeightedStrategy`, :class:`TermStrategy` and :class:`HeuristicStrategy` are
deliberately absent: each needs the caller to say what it should rank by, so there is no
default worth registering.

"""


def make_strategy(name, **kwargs):
    """Build a named strategy from :data:`STRATEGIES`.

    Parameters
    ----------
    name : str
    **kwargs : dict
        Passed to the strategy's constructor.

    Returns
    -------
    :class:`PreferenceStrategy`

    Examples
    --------
    >>> make_strategy("clearance").name
    'clearance'

    """
    if name not in STRATEGIES:
        raise ValueError("Unknown strategy {!r}. Known strategies: {}.".format(name, ", ".join(sorted(STRATEGIES))))
    return STRATEGIES[name](**kwargs)
