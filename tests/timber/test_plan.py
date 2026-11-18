"""SequencePlan: a sequence as data of its own, keyed by stable keys, kept apart from the model."""

import pytest
from compas.data import json_dumps
from compas.data import json_loads
from compas_timber.connections import TLapJoint
from frames import build_frame

from compas_timber_sequencing import SequenceOverrides
from compas_timber_sequencing import SequencePlan
from compas_timber_sequencing import apply_plan
from compas_timber_sequencing import assembly_order
from compas_timber_sequencing import diff
from compas_timber_sequencing import plan_sequence
from compas_timber_sequencing.timber.attributes import SEQUENCE_ATTRIBUTE


@pytest.fixture
def frame():
    model, _ = build_frame()
    return model


@pytest.fixture
def plan(frame):
    return plan_sequence(frame)


# ---------------------------------------------------------------------------
# making a plan
# ---------------------------------------------------------------------------


def test_planning_does_not_change_the_model(frame):
    plan_sequence(frame)
    assert all(SEQUENCE_ATTRIBUTE not in beam.attributes for beam in frame.beams)


def test_a_plan_is_keyed_by_name(plan):
    assert sorted(plan.order) == sorted(["left post", "brace", "strut", "right post", "bottom rail", "top rail"])
    assert plan.is_complete
    assert plan.position(plan.order[0]) == 0


def test_rebuilding_the_model_gives_the_same_plan(plan):
    # New guids, same design: the ids the search breaks ties with are the stable keys, so the
    # sequence of an unchanged design does not depend on the rebuild.
    rebuilt, _ = build_frame()
    again = plan_sequence(rebuilt)
    assert again.order == plan.order
    assert diff(plan, again).is_empty


def test_unnamed_beams_also_give_the_same_plan_after_a_rebuild():
    first, _ = build_frame(named=False)
    second, _ = build_frame(named=False)
    assert plan_sequence(first).order == plan_sequence(second).order


def test_a_plan_records_what_it_was_made_from(plan):
    assert plan.settings["strategy"] == "gravity"
    assert plan.settings["width"] == 5
    assert set(plan.fingerprints) == set(plan.keys)
    assert plan.version


def test_a_plan_survives_a_json_round_trip(plan):
    restored = json_loads(json_dumps(plan))
    assert isinstance(restored, SequencePlan)
    assert restored.order == plan.order
    assert restored.insertion_vector("brace") == plan.insertion_vector("brace")
    assert isinstance(restored.overrides, SequenceOverrides)
    assert diff(plan, restored).is_empty


def test_a_plan_saves_to_and_loads_from_a_file(plan, tmp_path):
    path = str(tmp_path / "plan.json")
    plan.to_json(path)
    assert SequencePlan.from_json(path).order == plan.order


# ---------------------------------------------------------------------------
# overrides
# ---------------------------------------------------------------------------


def test_a_precedence_is_honoured(frame, plan):
    assert plan.position("bottom rail") < plan.position("strut")
    adjusted = plan_sequence(frame, SequenceOverrides().place_before("strut", "bottom rail"))
    assert adjusted.is_complete
    assert adjusted.position("strut") < adjusted.position("bottom rail")


def test_keep_order_holds_the_start_of_a_plan(frame, plan):
    overrides = SequenceOverrides().keep_order(plan, until=3).place_before("top rail", "right post")
    adjusted = plan_sequence(frame, overrides)
    kept = plan.order[:3]
    assert [key for key in adjusted.order if key in kept] == kept
    assert adjusted.position("top rail") < adjusted.position("right post")


def test_hand_placement_is_honoured(frame):
    adjusted = plan_sequence(frame, SequenceOverrides().place_by_hand("brace"))
    assert adjusted.steps["brace"]["by_hand"] is True


def test_a_negative_pin_counts_from_the_end(frame):
    adjusted = plan_sequence(frame, SequenceOverrides().pin("left post", -1))
    assert adjusted.order[-1] == "left post"


def test_overrides_can_choose_the_strategy(frame):
    assert plan_sequence(frame, SequenceOverrides().use_strategy("clearance")).settings["strategy"] == "clearance"


def test_an_override_on_a_missing_element_is_reported_not_ignored(frame):
    adjusted = plan_sequence(frame, SequenceOverrides().place_before("ghost", "strut"))
    assert adjusted.is_complete
    assert [kind for kind, _, _ in adjusted.staleness] == ["before"]


def test_contradicting_precedences_raise(frame):
    overrides = SequenceOverrides().place_before("strut", "brace").place_before("brace", "strut")
    with pytest.raises(ValueError, match="contradict"):
        plan_sequence(frame, overrides)


def test_the_plan_keeps_a_copy_of_its_overrides(frame):
    overrides = SequenceOverrides().place_before("strut", "bottom rail")
    adjusted = plan_sequence(frame, overrides)
    overrides.place_by_hand("brace")
    assert adjusted.overrides.by_hand == set()


# ---------------------------------------------------------------------------
# a plan against a model
# ---------------------------------------------------------------------------


def test_a_plan_matches_a_rebuild_of_its_model(plan):
    rebuilt, _ = build_frame()
    assert plan.check(rebuilt).is_current


def test_check_finds_a_moved_beam(plan):
    moved, _ = build_frame(move=("brace", (0, 0, 50)))
    check = plan.check(moved)
    assert not check.is_current
    assert check.changed["brace"] == ["geometry"]


def test_check_finds_changed_joints(plan):
    changed, _ = build_frame(without_joints=[(TLapJoint, "strut", "brace")])
    check = plan.check(changed)
    assert "joints" in check.changed["strut"]
    assert "joints" in check.changed["brace"]


def test_check_finds_missing_and_new_beams(plan):
    smaller, _ = build_frame(without=["strut"])
    assert plan.check(smaller).missing == ["strut"]
    assert plan_sequence(smaller).check(build_frame()[0]).new == ["strut"]


def test_apply_plan_writes_a_rebuilt_model_by_key(plan):
    rebuilt, _ = build_frame()
    check = apply_plan(rebuilt, plan)
    assert check.is_current
    assert [beam.name for beam in assembly_order(rebuilt)] == plan.order


def test_apply_plan_can_refuse_a_model_that_changed(plan):
    moved, _ = build_frame(move=("brace", (0, 0, 50)))
    with pytest.raises(ValueError, match="does not match"):
        apply_plan(moved, plan, strict=True)


def test_apply_plan_clears_what_an_earlier_plan_wrote(plan):
    model, beams = build_frame()
    apply_plan(model, plan)
    smaller = SequencePlan(order=plan.order[1:], steps={key: plan.steps[key] for key in plan.order[1:]}, unplaced=plan.order[:1])
    apply_plan(model, smaller)
    assert SEQUENCE_ATTRIBUTE not in beams[plan.order[0]].attributes


def test_placed_elements_keep_absolute_positions_when_some_are_unplaced(plan):
    # A search that gets stuck places only the last elements; they keep their true positions.
    tail = plan.order[2:]
    partial = SequencePlan(order=tail, steps={key: plan.steps[key] for key in tail}, unplaced=plan.order[:2], stuck={"remaining": plan.order[:2], "reasons": {}})
    assert not partial.is_complete
    assert [partial.position(key) for key in tail] == [2, 3, 4, 5]
    assert partial.position(plan.order[0]) is None

    model, beams = build_frame()
    apply_plan(model, partial)
    assert [beams[key].attributes[SEQUENCE_ATTRIBUTE] for key in tail] == [2, 3, 4, 5]
    assert SEQUENCE_ATTRIBUTE not in beams[plan.order[0]].attributes
