"""Model-level regression net for assembly sequencing.

The synthetic fixtures under ``tests/core`` come first and are where a
failure can be attributed to a cause. These tests exercise the adapter: a real model, real
joint classes, real geometry. When one of these fails on its own it will not tell you why
-- a hundred interacting decisions produced that order -- which is exactly why the
synthetic fixtures exist.

"""

import pytest
from compas.data import json_dumps
from compas.data import json_loads
from compas.geometry import Line
from compas.geometry import Plane
from compas.geometry import Point
from compas.geometry import Vector

from compas_timber_sequencing import HalfSpace
from compas_timber_sequencing import SignedAxis
from compas_timber.connections import TButtJoint
from compas_timber.connections import TTenonMortiseJoint
from compas_timber.elements import Beam
from compas_timber.model import TimberModel
from compas_timber_sequencing.timber import TimberModelAdapter
from compas_timber_sequencing.timber import assembly_order
from compas_timber_sequencing.timber import generate_assembly_sequence
from compas_timber_sequencing.timber import MANUAL_ATTRIBUTE
from compas_timber_sequencing.timber import SEQUENCE_ATTRIBUTE
from compas_timber_sequencing.timber import VECTOR_ATTRIBUTE
from compas_timber_sequencing.kinematics import kinematic_constraint
from compas_timber_sequencing.timber import constraints_from_joint_output

WIDTH = 100.0
HEIGHT = 100.0


@pytest.fixture
def portal():
    """Two posts and a top beam, T-butted together."""
    model = TimberModel()
    post_west = Beam.from_centerline(Line(Point(0, 0, 0), Point(0, 0, 2000)), WIDTH, HEIGHT)
    post_east = Beam.from_centerline(Line(Point(3000, 0, 0), Point(3000, 0, 2000)), WIDTH, HEIGHT)
    top = Beam.from_centerline(Line(Point(0, 0, 2000), Point(3000, 0, 2000)), WIDTH, HEIGHT)
    for beam in (post_west, post_east, top):
        model.add_element(beam)
    TButtJoint.create(model, top, post_west)
    TButtJoint.create(model, top, post_east)
    model.process_joinery()
    return model, post_west, post_east, top


def ids_to_beams(result, adapter):
    return [adapter.elements_by_id[element_id] for element_id in result.order]


# ---------------------------------------------------------------------------
# constraint conversion
# ---------------------------------------------------------------------------


def test_a_line_becomes_a_signed_axis_oriented_start_to_end():
    constraints = constraints_from_joint_output(Line(Point(0, 0, 0), Point(0, 0, 5)))
    assert len(constraints) == 1
    assert isinstance(constraints[0], SignedAxis)
    assert constraints[0].direction.z == pytest.approx(1.0)


def test_a_vector_becomes_a_half_space():
    constraints = constraints_from_joint_output(Vector(1, 0, 0))
    assert isinstance(constraints[0], HalfSpace)


def test_a_list_of_vectors_becomes_several_half_spaces():
    constraints = constraints_from_joint_output([Vector(1, 0, 0), Vector(0, 1, 0)])
    assert len(constraints) == 2
    assert all(isinstance(constraint, HalfSpace) for constraint in constraints)


def test_a_plane_is_rejected_rather_than_dropped():
    # Four docstrings used to promise a Plane return that no implementation produces. If one
    # ever appears it must fail loudly, not vanish and leave the element looking free.
    with pytest.raises(TypeError):
        constraints_from_joint_output(Plane(Point(0, 0, 0), Vector(0, 0, 1)))


def test_the_inferred_flag_is_carried_through_conversion():
    constraints = constraints_from_joint_output([Vector(1, 0, 0)], inferred=True)
    assert constraints[0].inferred is True


# ---------------------------------------------------------------------------
# mortise-tenon sign regression
# ---------------------------------------------------------------------------


def test_mortise_tenon_signs_are_opposite_for_the_two_members():
    # The joint class once defined get_kinematic_constraint twice with opposite signs. That class
    # of defect only ever surfaces as a sign flip in output, so pin it: main_beam takes
    # + axis, cross_beam takes - axis.
    main_beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), 80, 100)
    cross_beam = Beam.from_centerline(Line(Point(500, -300, 0), Point(500, 300, 0)), 80, 100)
    joint = TTenonMortiseJoint(main_beam=main_beam, cross_beam=cross_beam)

    axis = cross_beam.ref_sides[joint.cross_beam_ref_side_index].normal
    main_constraint = constraints_from_joint_output(kinematic_constraint(joint, main_beam))[0]
    cross_constraint = constraints_from_joint_output(kinematic_constraint(joint, cross_beam))[0]

    assert main_constraint.direction.dot(axis) == pytest.approx(1.0)
    assert cross_constraint.direction.dot(axis) == pytest.approx(-1.0)
    assert main_constraint.direction.dot(cross_constraint.direction) == pytest.approx(-1.0)


def test_mortise_tenon_is_a_strict_one_dof_constraint():
    main_beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), 80, 100)
    cross_beam = Beam.from_centerline(Line(Point(500, -300, 0), Point(500, 300, 0)), 80, 100)
    joint = TTenonMortiseJoint(main_beam=main_beam, cross_beam=cross_beam)
    assert isinstance(constraints_from_joint_output(kinematic_constraint(joint, main_beam))[0], SignedAxis)


# ---------------------------------------------------------------------------
# adapter
# ---------------------------------------------------------------------------


def test_the_adapter_flattens_a_model(portal):
    model, post_west, post_east, top = portal
    data = TimberModelAdapter(model).build()

    assert len(data.element_ids) == 3
    assert data.neighbors[str(top.guid)] == {str(post_west.guid), str(post_east.guid)}
    assert data.neighbors[str(post_west.guid)] == {str(top.guid)}
    assert len(data.joint_members) == 2


def test_the_adapter_measures_height_and_length_from_the_centerline(portal):
    model, post_west, _, top = portal
    data = TimberModelAdapter(model).build()

    assert data.base_z[str(post_west.guid)] == pytest.approx(0.0)
    assert data.centroid_z[str(post_west.guid)] == pytest.approx(1000.0)
    assert data.base_z[str(top.guid)] == pytest.approx(2000.0)
    assert data.length[str(top.guid)] == pytest.approx(3000.0)


def test_the_adapter_supplies_a_swept_check_by_default(portal):
    model, _, _, _ = portal
    assert TimberModelAdapter(model).build().has_geometry is True
    assert TimberModelAdapter(model, use_geometry=False).build().has_geometry is False


# ---------------------------------------------------------------------------
# the swept check
# ---------------------------------------------------------------------------


def test_the_swept_check_sees_an_element_standing_in_the_way():
    model = TimberModel()
    mover = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), WIDTH, HEIGHT)
    blocker = Beam.from_centerline(Line(Point(0, 0, 400), Point(1000, 0, 400)), WIDTH, HEIGHT)
    model.add_element(mover)
    model.add_element(blocker)

    adapter = TimberModelAdapter(model)
    active = set(str(element.guid) for element in (mover, blocker))
    assert adapter.path_is_clear(str(mover.guid), Vector(0, 0, 1), 1000.0, active) is False
    assert adapter.path_is_clear(str(mover.guid), Vector(0, 0, -1), 1000.0, active) is True


def test_the_swept_check_is_not_fooled_by_the_empty_corner_of_a_diagonal():
    # A brace running corner to corner fills a fraction of its axis-aligned bounds, and
    # the beam here passes through the empty part of them. An axis-aligned test alone
    # calls that a collision; in a braced frame that false veto lands on the elements
    # highest up and drives the sequence to pull posts out from under standing beams.
    model = TimberModel()
    beam = Beam.from_centerline(Line(Point(0, 0, 2000), Point(3000, 0, 2000)), WIDTH, HEIGHT)
    brace = Beam.from_centerline(Line(Point(0, 0, 1000), Point(0, -1000, 2000)), WIDTH, HEIGHT)
    model.add_element(beam)
    model.add_element(brace)

    adapter = TimberModelAdapter(model)
    active = set(str(element.guid) for element in (beam, brace))

    from compas_timber_sequencing.timber import _bounds_overlap
    from compas_timber_sequencing.timber import _swept_aabb

    swept = _swept_aabb(beam.obb, Vector(0, 0, 1), 100.0)
    assert _bounds_overlap(swept, adapter._bounds(str(brace.guid))) is True, "the broad phase should be the one that is fooled"
    assert adapter.path_is_clear(str(beam.guid), Vector(0, 0, 1), 100.0, active) is True


def test_the_swept_check_stops_short_of_something_further_than_the_approach():
    model = TimberModel()
    mover = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), WIDTH, HEIGHT)
    far = Beam.from_centerline(Line(Point(0, 0, 5000), Point(1000, 0, 5000)), WIDTH, HEIGHT)
    model.add_element(mover)
    model.add_element(far)

    adapter = TimberModelAdapter(model)
    active = set(str(element.guid) for element in (mover, far))
    assert adapter.path_is_clear(str(mover.guid), Vector(0, 0, 1), 100.0, active) is True
    assert adapter.path_is_clear(str(mover.guid), Vector(0, 0, 1), 6000.0, active) is False


def test_elements_that_are_not_beams_are_excluded_and_reported():
    from compas_model.elements import Element

    model = TimberModel()
    beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), WIDTH, HEIGHT)
    model.add_element(beam)
    other = Element()
    model.add_element(other)

    data = TimberModelAdapter(model).build()
    assert data.element_ids == [str(beam.guid)]
    assert str(other.guid) in data.excluded
    assert "beams only" in data.excluded[str(other.guid)]


# ---------------------------------------------------------------------------
# end to end
# ---------------------------------------------------------------------------


def test_a_real_model_sequences_end_to_end(portal):
    model, post_west, post_east, top = portal
    result, adapter = generate_assembly_sequence(model)

    assert result.is_complete
    assert result.stuck is None
    assert result.pin_conflict is None
    assert len(result.order) == 3

    ordered = ids_to_beams(result, adapter)
    assert ordered[-1] is top, "the posts must stand before the beam goes on top"
    assert set(ordered[:2]) == {post_west, post_east}


def test_every_sequenced_element_gets_a_unit_insertion_vector(portal):
    model, _, _, _ = portal
    result, _ = generate_assembly_sequence(model)
    for element_id in result.order:
        vector = result.insertion_vectors[element_id]
        assert vector is not None
        assert vector.length == pytest.approx(1.0)


def test_the_sequence_is_written_onto_the_model(portal):
    model, _, _, top = portal
    generate_assembly_sequence(model)

    assert top.attributes[SEQUENCE_ATTRIBUTE] == 2
    assert top.attributes[VECTOR_ATTRIBUTE] is not None
    assert top.attributes[MANUAL_ATTRIBUTE] is False
    assert assembly_order(model)[-1] is top


def test_assembly_order_lists_the_sequenced_elements_in_order(portal):
    model, _, _, _ = portal
    result, adapter = generate_assembly_sequence(model)
    assert assembly_order(model) == ids_to_beams(result, adapter)


def test_the_sequence_survives_a_json_round_trip(portal):
    model, _, _, _ = portal
    generate_assembly_sequence(model)
    before = [str(element.guid) for element in assembly_order(model)]

    restored = json_loads(json_dumps(model))
    assert [str(element.guid) for element in assembly_order(restored)] == before
    for element in assembly_order(restored):
        assert element.attributes[VECTOR_ATTRIBUTE] is not None
        assert MANUAL_ATTRIBUTE in element.attributes


def test_a_real_model_sequences_the_same_way_twice(portal):
    model, _, _, _ = portal
    first, _ = generate_assembly_sequence(model)
    second, _ = generate_assembly_sequence(model)
    assert first.order == second.order


def test_the_hand_placement_set_round_trips_through_element_attributes(portal):
    model, post_west, _, _ = portal
    # A fact about the design, so it is persisted on the element and picked up on re-run.
    post_west.attributes[MANUAL_ATTRIBUTE] = True

    result, _ = generate_assembly_sequence(model)
    assert str(post_west.guid) in result.manual_set
    assert post_west.attributes[MANUAL_ATTRIBUTE] is True


def test_an_explicit_manual_set_overrides_the_persisted_one(portal):
    model, post_west, _, _ = portal
    post_west.attributes[MANUAL_ATTRIBUTE] = True
    result, _ = generate_assembly_sequence(model, manual_set=set())
    assert str(post_west.guid) not in result.manual_set


def test_sequencing_without_geometry_still_works(portal):
    model, _, _, top = portal
    result, adapter = generate_assembly_sequence(model, use_geometry=False)
    assert result.is_complete
    assert ids_to_beams(result, adapter)[-1] is top


def test_a_pinned_order_is_honoured_on_a_real_model(portal):
    model, post_west, post_east, top = portal
    wanted = [str(post_east.guid), str(post_west.guid), str(top.guid)]
    result, _ = generate_assembly_sequence(model, pinned_order=wanted)
    assert result.is_complete
    assert result.order == wanted


def test_an_unjointed_beam_is_free_rather_than_locked():
    # Zero constraints means entirely free. It used to mean locked.
    model = TimberModel()
    beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), WIDTH, HEIGHT)
    model.add_element(beam)
    result, _ = generate_assembly_sequence(model)
    assert result.is_complete
    assert result.insertion_vectors[str(beam.guid)] is not None
