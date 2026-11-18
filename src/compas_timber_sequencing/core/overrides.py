"""What a designer wants changed about a sequence, recorded as intent rather than positions.

The search proposes an order; :class:`SequenceOverrides` is how a person adjusts it. Each method
records what they *mean* -- "the bottom rail goes before the strut", "keep the first three steps as
they are", "the brace is placed by hand" -- keyed by element keys, so the overrides survive a rebuild
of the model and elements being added or removed. Relative precedences are the main tool for that
reason; absolute pins are kept for the rare "this must be step 1".

Overrides are plain data. They save to and load from JSON like any COMPAS data object, so they can be
kept next to the model, versioned, and handed between scripts -- but they are written by these
methods, not by editing the file.

Examples
--------
>>> overrides = SequenceOverrides()
>>> overrides.place_before("bottom rail", "strut").place_by_hand("brace")
SequenceOverrides(1 precedence, 0 pins, 1 by hand)
>>> overrides.precedences
[('bottom rail', 'strut')]

"""

from compas.data import Data


class SequenceOverrides(Data):
    """Adjustments to a sequence, applied as hard constraints by the search.

    Parameters
    ----------
    precedences : iterable of tuple, optional
        ``(a, b)`` pairs of element keys: `a` is assembled before `b`.
    pins : dict, optional
        Element key -> assembly position. Negative positions count from the end; -1 is last.
    by_hand : iterable, optional
        Element keys placed by hand: exempt from the robot's kinematic check, and given no
        insertion vector to follow.
    strategy : str, optional
        Name of the ranking strategy to use (see
        :data:`~compas_timber_sequencing.ranking.strategies.STRATEGIES`).

    Attributes
    ----------
    precedences : list of tuple
    pins : dict
    by_hand : set
    strategy : str or None

    Notes
    -----
    Nothing here is checked against a model until the overrides are used: an element key that
    no longer exists is reported in the plan's staleness report, and precedences that contradict
    each other raise when the search starts.

    """

    @property
    def __dtype__(self):
        # Saved as compas_timber_sequencing/SequenceOverrides, which the package root exports, so that loading
        # from JSON finds the class without the subpackage having to import it.
        return "compas_timber_sequencing/SequenceOverrides"

    @property
    def __data__(self):
        return {
            "precedences": [list(pair) for pair in self.precedences],
            "pins": dict(self.pins),
            "by_hand": sorted(self.by_hand),
            "strategy": self.strategy,
        }

    @classmethod
    def __from_data__(cls, data):
        return cls(
            precedences=[tuple(pair) for pair in data.get("precedences", [])],
            pins=data.get("pins", {}),
            by_hand=data.get("by_hand", []),
            strategy=data.get("strategy"),
        )

    def __init__(self, precedences=None, pins=None, by_hand=None, strategy=None, name=None):
        super(SequenceOverrides, self).__init__(name=name)
        self.precedences = []
        for first, then in precedences or ():
            self.place_before(first, then)
        self.pins = dict(pins or {})
        self.by_hand = set(by_hand or ())
        self.strategy = strategy

    def __repr__(self):
        return "SequenceOverrides({} precedence{}, {} pin{}, {} by hand)".format(
            len(self.precedences),
            "" if len(self.precedences) == 1 else "s",
            len(self.pins),
            "" if len(self.pins) == 1 else "s",
            len(self.by_hand),
        )

    def __len__(self):
        return len(self.precedences) + len(self.pins) + len(self.by_hand)

    # ------------------------------------------------------------------------------------
    # recording intent
    # ------------------------------------------------------------------------------------

    def place_before(self, first, then):
        """Assemble `first` before `then`.

        Parameters
        ----------
        first, then : str
            Element keys.

        Returns
        -------
        :class:`SequenceOverrides`
            This object, so calls can be chained.

        """
        pair = (first, then)
        if pair not in self.precedences:
            self.precedences.append(pair)
        return self

    def place_after(self, later, earlier):
        """Assemble `later` after `earlier`. The same as ``place_before(earlier, later)``.

        Returns
        -------
        :class:`SequenceOverrides`

        """
        return self.place_before(earlier, later)

    def keep_order(self, plan, start=0, until=None):
        """Keep a stretch of an existing sequence as it is.

        Records the order of ``plan.order[start:until]`` as a chain of precedences, so the search
        keeps those elements in that order while it is free to place everything else. Use it on
        the part of a plan you are happy with before adjusting the rest.

        Parameters
        ----------
        plan : :class:`~compas_timber_sequencing.timber.plan.SequencePlan` or list
            A plan, or anything else with an ``order`` attribute, or a list of keys in order.
        start : int, optional
        until : int, optional
            Exclusive end, like a slice. The default keeps everything from `start` on.

        Returns
        -------
        :class:`SequenceOverrides`

        """
        order = list(getattr(plan, "order", plan))[start:until]
        for first, then in zip(order, order[1:]):
            self.place_before(first, then)
        return self

    def pin(self, key, position):
        """Fix an element at an absolute assembly position.

        Prefer :meth:`place_before`: a pin stops meaning what it meant as soon as an element is
        added or removed before it.

        Parameters
        ----------
        key : str
        position : int
            0 is the first element placed; negative positions count from the end.

        Returns
        -------
        :class:`SequenceOverrides`

        """
        self.pins[key] = int(position)
        return self

    def place_by_hand(self, key):
        """Mark an element as placed by hand.

        Returns
        -------
        :class:`SequenceOverrides`

        """
        self.by_hand.add(key)
        return self

    def forget(self, key):
        """Drop everything said about one element: its precedences, its pin, its hand placement.

        Returns
        -------
        :class:`SequenceOverrides`

        """
        self.precedences = [pair for pair in self.precedences if key not in pair]
        self.pins.pop(key, None)
        self.by_hand.discard(key)
        return self

    def use_strategy(self, name):
        """Rank with a named strategy instead of the default.

        Returns
        -------
        :class:`SequenceOverrides`

        """
        self.strategy = name
        return self
