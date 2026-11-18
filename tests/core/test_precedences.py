"""SequenceOverrides, and precedences ("a before b") in the search, on hand-built inputs."""

import pytest
from compas.data import json_dumps
from compas.data import json_loads
from compas.geometry import Vector
from synthetic import stack
from synthetic import wall_panels

from compas_timber_sequencing import HalfSpace
from compas_timber_sequencing import SequenceOverrides
from compas_timber_sequencing import SequencingInput
from compas_timber_sequencing import SignedAxis
from compas_timber_sequencing import generate

# ---------------------------------------------------------------------------
# the overrides object
# ---------------------------------------------------------------------------


def test_methods_chain_and_record_intent():
    overrides = SequenceOverrides().place_before("a", "b").place_after("d", "c").pin("e", -1).place_by_hand("f").use_strategy("layered")
    assert overrides.precedences == [("a", "b"), ("c", "d")]
    assert overrides.pins == {"e": -1}
    assert overrides.by_hand == {"f"}
    assert overrides.strategy == "layered"
    assert len(overrides) == 4


def test_a_precedence_is_recorded_once():
    overrides = SequenceOverrides().place_before("a", "b").place_before("a", "b")
    assert overrides.precedences == [("a", "b")]


def test_keep_order_chains_a_stretch_of_an_order():
    overrides = SequenceOverrides().keep_order(["a", "b", "c", "d"], start=1, until=4)
    assert overrides.precedences == [("b", "c"), ("c", "d")]


def test_keep_order_accepts_anything_with_an_order():
    class Planned(object):
        order = ["a", "b", "c"]

    assert SequenceOverrides().keep_order(Planned()).precedences == [("a", "b"), ("b", "c")]


def test_forget_drops_everything_about_one_element():
    overrides = SequenceOverrides().place_before("a", "b").place_before("c", "d").pin("a", 0).place_by_hand("a")
    overrides.forget("a")
    assert overrides.precedences == [("c", "d")]
    assert overrides.pins == {}
    assert overrides.by_hand == set()


def test_overrides_survive_a_json_round_trip():
    overrides = SequenceOverrides().place_before("a", "b").pin("c", 2).place_by_hand("d").use_strategy("chain")
    restored = json_loads(json_dumps(overrides))
    assert isinstance(restored, SequenceOverrides)
    assert restored.__data__ == overrides.__data__


# ---------------------------------------------------------------------------
# precedences in the search
# ---------------------------------------------------------------------------


def test_a_precedence_reorders_otherwise_free_choices():
    data = wall_panels()
    default = generate(data).order
    studs = [key for key in default if key.startswith("stud_a")]
    first, second = studs[0], studs[1]
    assert default.index(first) < default.index(second)

    adjusted = generate(data, precedences=[(second, first)]).order
    assert adjusted.index(second) < adjusted.index(first)


def peg_in_box():
    """A peg in a closed box: the peg slides out upward, the box cannot move while the peg is in it.

    So the box must be assembled first. Not an intrinsic lock -- only one of the two blocks the
    other -- so nothing is placed by hand.
    """
    every_way = [HalfSpace(Vector(*axis)) for axis in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))]
    table = {
        ("peg", frozenset(["box"])): [SignedAxis(Vector(0, 0, 1))],
        ("box", frozenset(["peg"])): every_way,
    }
    return SequencingInput(
        element_ids=["peg", "box"],
        neighbors={"peg": {"box"}, "box": {"peg"}},
        base_z={"peg": 0.5, "box": 0.0},
        centroid_z={"peg": 1.0, "box": 0.5},
        length={"peg": 1.0, "box": 1.0},
        constraints=lambda element_id, active: table.get((element_id, frozenset(active)), []),
    )


def test_an_impossible_precedence_ends_in_a_stuck_report_that_says_why():
    assert generate(peg_in_box()).order == ["box", "peg"]

    result = generate(peg_in_box(), precedences=[("peg", "box")])
    assert not result.is_complete
    assert result.stuck.blockers["peg"] == "must be assembled before box, still in place"
    assert "box" in result.stuck.blockers


def test_contradicting_precedences_raise_and_name_the_cycle():
    with pytest.raises(ValueError, match="'bottom' before 'middle' before 'top' before 'bottom'"):
        generate(stack(), precedences=[("bottom", "middle"), ("middle", "top"), ("top", "bottom")])


def test_a_precedence_on_an_unknown_element_is_reported_as_stale():
    result = generate(stack(), precedences=[("ghost", "top")])
    assert result.is_complete
    assert [kind for kind, _, _ in result.staleness] == ["before"]


def test_a_negative_pin_counts_from_the_end():
    assert generate(stack(), pinned_order={"top": -1}).order[-1] == "top"


def test_a_pin_that_contradicts_a_precedence_is_a_pin_conflict():
    result = generate(stack(), pinned_order={"top": -1}, precedences=[("top", "middle")])
    assert not result.is_complete
