"""diff(): what changed between two plans."""

from frames import build_frame

from compas_timber_sequencing import SequenceOverrides
from compas_timber_sequencing import diff
from compas_timber_sequencing import plan_sequence


def test_the_same_plan_has_no_differences():
    model, _ = build_frame()
    plan = plan_sequence(model)
    change = diff(plan, plan)
    assert change.is_empty
    assert not change
    assert change.describe() == "No change."


def test_a_reorder_shows_up_as_moves():
    model, _ = build_frame()
    before = plan_sequence(model)
    after = plan_sequence(model, SequenceOverrides().place_before("strut", "bottom rail"))
    change = diff(before, after)

    moved = {key: (old, new) for key, old, new in change.moved}
    assert moved["strut"][1] < moved["bottom rail"][1]
    assert change.overrides_changed
    assert "moved: strut" in change.describe()


def test_a_changed_insertion_direction_is_reported_with_its_angle():
    model, _ = build_frame()
    before = plan_sequence(model)
    after = plan_sequence(model, SequenceOverrides().place_before("strut", "bottom rail"))
    turned = dict(diff(before, after).turned)
    # Placing the strut first changes how the bottom rail can go in.
    assert turned["bottom rail"] > 1.0


def test_added_and_removed_elements():
    full, _ = build_frame()
    smaller, _ = build_frame(without=["strut"])
    change = diff(plan_sequence(full), plan_sequence(smaller))
    assert change.removed == ["strut"]
    assert diff(plan_sequence(smaller), plan_sequence(full)).added == ["strut"]


def test_a_design_change_is_reported_per_element():
    original, _ = build_frame()
    moved, _ = build_frame(move=("brace", (0, 0, 50)))
    change = diff(plan_sequence(original), plan_sequence(moved))
    assert change.geometry_changed == ["brace"]


def test_hand_placement_and_fit_changes_are_reported():
    model, _ = build_frame()
    before = plan_sequence(model)
    after = plan_sequence(model, SequenceOverrides().place_by_hand("strut"))
    assert ("strut", False, True) in diff(before, after).by_hand_changed
