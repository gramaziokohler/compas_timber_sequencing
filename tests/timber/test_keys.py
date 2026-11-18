"""Stable element keys: the same beam gets the same key across rebuilds."""

import pytest
from compas.geometry import Line
from compas.geometry import Point
from compas_timber.elements import Beam
from compas_timber.model import TimberModel
from frames import build_frame

from compas_timber_sequencing.timber.keys import element_keys
from compas_timber_sequencing.timber.keys import geometric_key


def _keys_by_name(model, beams):
    keys = element_keys(model)
    return {name: keys[str(beam.guid)] for name, beam in beams.items()}


def test_a_unique_name_is_the_key():
    model, beams = build_frame()
    assert _keys_by_name(model, beams)["brace"] == "brace"


def test_unnamed_beams_get_geometric_keys():
    model, beams = build_frame(named=False)
    keys = _keys_by_name(model, beams)
    assert all(key.startswith("beam-") for key in keys.values())
    assert len(set(keys.values())) == len(keys)


def test_keys_survive_a_rebuild_even_though_guids_do_not():
    first_model, first = build_frame(named=False)
    second_model, second = build_frame(named=False)
    assert str(first["brace"].guid) != str(second["brace"].guid)
    assert _keys_by_name(first_model, first) == _keys_by_name(second_model, second)


def test_a_reversed_centerline_gives_the_same_geometric_key():
    forward = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), 60, 120)
    backward = Beam.from_centerline(Line(Point(1000, 0, 0), Point(0, 0, 0)), 60, 120)
    assert geometric_key(forward) == geometric_key(backward)


def test_moving_an_unnamed_beam_changes_its_key_and_a_named_one_keeps_it():
    named, named_beams = build_frame(move=("brace", (0, 0, 50)))
    assert _keys_by_name(named, named_beams)["brace"] == "brace"

    original, original_beams = build_frame(named=False)
    moved, moved_beams = build_frame(named=False, move=("brace", (0, 0, 50)))
    assert _keys_by_name(original, original_beams)["brace"] != _keys_by_name(moved, moved_beams)["brace"]


def test_movement_within_the_tolerance_keeps_the_key():
    a = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), 60, 120)
    b = Beam.from_centerline(Line(Point(0.2, 0, 0), Point(1000.2, 0, 0)), 60, 120)
    assert geometric_key(a) == geometric_key(b)


def test_the_default_name_does_not_count_as_a_name():
    model = TimberModel()
    beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), 60, 120)
    model.add_element(beam)
    assert beam.name == "Beam"
    assert element_keys(model)[str(beam.guid)].startswith("beam-")


def test_duplicate_names_fall_back_to_geometry():
    model = TimberModel()
    first = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), 60, 120)
    second = Beam.from_centerline(Line(Point(0, 500, 0), Point(1000, 500, 0)), 60, 120)
    first.name = second.name = "joist"
    model.add_element(first)
    model.add_element(second)
    keys = element_keys(model)
    assert keys[str(first.guid)] != keys[str(second.guid)]
    assert all(key.startswith("beam-") for key in keys.values())


def test_two_unnamed_beams_in_the_same_place_cannot_be_told_apart():
    model = TimberModel()
    for _ in range(2):
        model.add_element(Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), 60, 120))
    with pytest.raises(ValueError, match="cannot be told apart"):
        element_keys(model)
