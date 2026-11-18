"""Tests for ``kinematic_constraint``.

The contract, stated in :func:`compas_timber_sequencing.kinematics.kinematic_constraint`:

* a :class:`~compas.geometry.Line` is a strict 1-DOF sliding axis, **signed**: the element
  may leave along ``start -> end`` only;
* a :class:`~compas.geometry.Vector` is the inward normal of a permitted half-space;
* a list of vectors is the intersection of those half-spaces.

The central check here is a physical invariant that every correct two-member joint satisfies
whatever its geometry: pulling A out along ``d`` while B stays put is the same relative motion
as pulling B out along ``-d`` while A stays put. A sign error on one member breaks it, and sign
errors are exactly the defect this code has had before.

None of these tests runs ``process_joinery``: the constraints are computed from the joint's
own planes and the beams' reference sides, not from the processed geometry.

"""

import itertools

import pytest
from compas.geometry import Line
from compas.geometry import Point
from compas.geometry import Vector

from compas_timber.connections import ISimpleScarf
from compas_timber.connections import LButtJoint
from compas_timber.connections import LFrenchRidgeLapJoint
from compas_timber.connections import LLapJoint
from compas_timber.connections import LMiterJoint
from compas_timber.connections import LTenonMortiseJoint
from compas_timber.connections import TBirdsmouthJoint
from compas_timber.connections import TButtJoint
from compas_timber.connections import TDovetailJoint
from compas_timber.connections import TLapJoint
from compas_timber.connections import TStepJoint
from compas_timber.connections import TTenonMortiseJoint
from compas_timber.connections import XLapJoint
from compas_timber.connections import XNotchJoint
from compas_timber.connections import YButtJoint
from compas_timber.elements import Beam
from compas_timber.model import TimberModel

from compas_timber_sequencing.kinematics import is_fallback
from compas_timber_sequencing.kinematics import kinematic_constraint

TOL = 1e-9


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def beam(start, end, width=80, height=100):
    return Beam.from_centerline(Line(Point(*start), Point(*end)), width=width, height=height)


def t_pair():
    """Main beam ending on the side of a cross beam."""
    return beam((0, -1000, 0), (0, 0, 0)), beam((-1000, 0, 0), (1000, 0, 0))


def l_pair():
    return beam((0, 0, 0), (1000, 0, 0)), beam((0, 0, 0), (0, 1000, 0))


def x_pair():
    return beam((-1000, 0, 0), (1000, 0, 0)), beam((0, -1000, 0), (0, 1000, 0))


def i_pair():
    return beam((-1000, 0, 0), (0, 0, 0)), beam((0, 0, 0), (1000, 0, 0))


def create(joint_type, make_pair, **kwargs):
    a, b = make_pair()
    model = TimberModel()
    model.add_element(a)
    model.add_element(b)
    joint = joint_type.create(model, a, b, **kwargs)
    return joint, a, b


def as_list(raw):
    return list(raw) if isinstance(raw, (list, tuple)) else [raw]


def margin(direction, normals):
    return min(normal.unitized().dot(direction) for normal in normals)


def free_direction(raw):
    """A direction satisfying every constraint in `raw`, or None.

    A Line admits exactly its own direction. For half-spaces the candidates are the normals,
    their sum and both signs of every pairwise cross product -- the extreme rays of the cone
    -- which is enough to find a feasible direction whenever one exists.

    """
    if isinstance(raw, Line):
        return raw.direction.unitized()
    normals = as_list(raw)
    candidates = [n.unitized() for n in normals]
    total = Vector(0, 0, 0)
    for normal in normals:
        total += normal.unitized()
    if total.length > TOL:
        candidates.append(total.unitized())
    for n1, n2 in itertools.combinations(normals, 2):
        cross = n1.cross(n2)
        if cross.length > 1e-6:
            candidates.extend([cross.unitized(), -cross.unitized()])
    best = max(candidates, key=lambda d: margin(d, normals))
    return best if margin(best, normals) >= -TOL else None


def allows(raw, direction):
    if isinstance(raw, Line):
        return raw.direction.unitized().dot(direction) >= 1.0 - 1e-6
    return margin(direction, as_list(raw)) >= -1e-6


# Every implemented two-member beam joint, in the configurations whose answers differ.
CASES = [
    pytest.param(TButtJoint, t_pair, {}, id="t_butt"),
    pytest.param(TButtJoint, t_pair, {"mill_depth": 10.0}, id="t_butt_milled"),
    pytest.param(LButtJoint, l_pair, {}, id="l_butt"),
    pytest.param(LButtJoint, l_pair, {"mill_depth": 10.0}, id="l_butt_milled"),
    pytest.param(LLapJoint, l_pair, {}, id="l_lap"),
    pytest.param(TLapJoint, t_pair, {}, id="t_lap"),
    pytest.param(XLapJoint, x_pair, {}, id="x_lap"),
    pytest.param(LMiterJoint, l_pair, {}, id="l_miter"),
    pytest.param(LFrenchRidgeLapJoint, l_pair, {}, id="l_french_ridge_lap"),
    pytest.param(TTenonMortiseJoint, t_pair, {}, id="t_tenon_mortise"),
    pytest.param(LTenonMortiseJoint, l_pair, {}, id="l_tenon_mortise"),
    pytest.param(TDovetailJoint, t_pair, {}, id="t_dovetail"),
    pytest.param(TStepJoint, t_pair, {}, id="t_step"),
    pytest.param(TBirdsmouthJoint, t_pair, {}, id="t_birdsmouth"),
    pytest.param(TBirdsmouthJoint, t_pair, {"mill_depth": 10.0}, id="t_birdsmouth_milled"),
    pytest.param(ISimpleScarf, i_pair, {}, id="i_simple_scarf"),
]


# ---------------------------------------------------------------------------
# the contract, for every implemented joint
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("joint_type, make_pair, kwargs", CASES)
def test_every_joint_overrides_the_generic_fallback(joint_type, make_pair, kwargs):
    assert not is_fallback(joint_type)


@pytest.mark.parametrize("joint_type, make_pair, kwargs", CASES)
def test_the_return_value_follows_the_contract(joint_type, make_pair, kwargs):
    joint, a, b = create(joint_type, make_pair, **kwargs)
    for element in (a, b):
        raw = kinematic_constraint(joint, element)
        if isinstance(raw, Line):
            assert raw.length > TOL
        else:
            vectors = as_list(raw)
            assert vectors, "an empty list would mean 'unconstrained', which no joint is"
            for vector in vectors:
                assert isinstance(vector, Vector)
                assert vector.length > TOL


@pytest.mark.parametrize("joint_type, make_pair, kwargs", CASES)
def test_each_member_can_be_extracted(joint_type, make_pair, kwargs):
    # A single joint on its own never locks either member; locks come from combining joints.
    joint, a, b = create(joint_type, make_pair, **kwargs)
    for element in (a, b):
        assert free_direction(kinematic_constraint(joint, element)) is not None


@pytest.mark.parametrize("joint_type, make_pair, kwargs", CASES)
def test_the_two_members_separate_in_opposite_directions(joint_type, make_pair, kwargs):
    joint, a, b = create(joint_type, make_pair, **kwargs)
    raw_a = kinematic_constraint(joint, a)
    raw_b = kinematic_constraint(joint, b)

    direction_a = free_direction(raw_a)
    direction_b = free_direction(raw_b)
    assert allows(raw_b, -direction_a), "pulling A out along d must mean B can leave along -d"
    assert allows(raw_a, -direction_b), "pulling B out along d must mean A can leave along -d"


@pytest.mark.parametrize("joint_type, make_pair, kwargs", CASES)
def test_an_element_outside_the_joint_is_rejected(joint_type, make_pair, kwargs):
    joint, _, _ = create(joint_type, make_pair, **kwargs)
    with pytest.raises(ValueError):
        kinematic_constraint(joint, beam((5000, 5000, 0), (6000, 5000, 0)))


# ---------------------------------------------------------------------------
# specific joints
# ---------------------------------------------------------------------------


def test_mortise_tenon_signs_are_opposite_for_the_two_members():
    # The class once defined get_kinematic_constraint twice with opposite signs. That class
    # of defect only ever surfaces as a sign flip in output, so pin it: main_beam takes
    # + axis, cross_beam takes - axis.
    main_beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), 80, 100)
    cross_beam = Beam.from_centerline(Line(Point(500, -300, 0), Point(500, 300, 0)), 80, 100)
    joint = TTenonMortiseJoint(main_beam=main_beam, cross_beam=cross_beam)

    axis = cross_beam.ref_sides[joint.cross_beam_ref_side_index].normal
    main_line = kinematic_constraint(joint, main_beam)
    cross_line = kinematic_constraint(joint, cross_beam)

    assert isinstance(main_line, Line)
    assert isinstance(cross_line, Line)
    assert main_line.direction.unitized().dot(axis) == pytest.approx(1.0)
    assert cross_line.direction.unitized().dot(axis) == pytest.approx(-1.0)


def test_t_butt_main_beam_leaves_away_from_the_cross_beam():
    joint, main_beam, cross_beam = create(TButtJoint, t_pair)
    direction = free_direction(kinematic_constraint(joint, main_beam))
    away = Vector.from_start_end(joint.location, main_beam.centerline.midpoint)
    assert direction.dot(away) > 0.0


def test_t_butt_with_a_pocket_only_slides_out_of_the_pocket():
    joint, main_beam, _ = create(TButtJoint, t_pair, mill_depth=10.0)
    raw = kinematic_constraint(joint, main_beam)
    # Sideways, along the cross beam, is now through the pocket walls.
    assert not allows(raw, Vector(1, 0, 0))
    assert not allows(raw, Vector(-1, 0, 0))
    assert allows(raw, Vector(0, -1, 0))


def test_birdsmouth_with_a_mill_depth_is_a_signed_axis_along_the_main_beam():
    joint, main_beam, cross_beam = create(TBirdsmouthJoint, t_pair, mill_depth=10.0)
    main_line = kinematic_constraint(joint, main_beam)
    cross_line = kinematic_constraint(joint, cross_beam)
    assert isinstance(main_line, Line)
    assert abs(main_line.direction.unitized().dot(main_beam.centerline.direction)) == pytest.approx(1.0)
    assert main_line.direction.unitized().dot(cross_line.direction.unitized()) == pytest.approx(-1.0)


def test_x_lap_is_a_signed_axis_perpendicular_to_both_beams():
    joint, a, b = create(XLapJoint, x_pair)
    line_a = kinematic_constraint(joint, a)
    line_b = kinematic_constraint(joint, b)
    assert isinstance(line_a, Line)
    assert line_a.direction.unitized().dot(a.centerline.direction) == pytest.approx(0.0, abs=1e-9)
    assert line_a.direction.unitized().dot(b.centerline.direction) == pytest.approx(0.0, abs=1e-9)
    assert line_a.direction.unitized().dot(line_b.direction.unitized()) == pytest.approx(-1.0)


# ---------------------------------------------------------------------------
# n-ary: the Y-butt
# ---------------------------------------------------------------------------


@pytest.fixture
def y_butt():
    main_beam = beam((0, 0, 0), (0, 0, 1000))
    cross_a = beam((0, 0, 0), (1000, 0, 0))
    cross_b = beam((0, 0, 0), (0, 1000, 0))
    model = TimberModel()
    for element in (main_beam, cross_a, cross_b):
        model.add_element(element)
    joint = YButtJoint.create(model, main_beam, cross_a, cross_b, mill_depth=10.0)
    return joint, main_beam, cross_a, cross_b


def test_y_butt_accepts_the_one_argument_call(y_butt):
    # The contract every other joint follows; the Y-butt used to require a second argument.
    joint, main_beam, cross_a, cross_b = y_butt
    for element in (main_beam, cross_a, cross_b):
        assert as_list(kinematic_constraint(joint, element))


def test_y_butt_defaults_to_every_other_member_being_in_place(y_butt):
    joint, main_beam, cross_a, cross_b = y_butt
    default = kinematic_constraint(joint, main_beam)
    explicit = kinematic_constraint(joint, main_beam, [cross_a, cross_b])
    assert [list(v) for v in default] == [list(v) for v in explicit]


def test_y_butt_constrains_only_against_members_already_in_place(y_butt):
    joint, main_beam, cross_a, _ = y_butt
    assert kinematic_constraint(joint, main_beam, []) == []
    assert len(kinematic_constraint(joint, main_beam, [cross_a])) < len(kinematic_constraint(joint, main_beam))


# ---------------------------------------------------------------------------
# the generic fallback
# ---------------------------------------------------------------------------


def test_a_joint_without_an_implementation_falls_back_to_a_half_space_between_the_members():
    def offset_pair():
        # Crossing, but not at the midpoints, so there is a beam-to-beam direction.
        return beam((-1000, 0, 0), (1000, 0, 0)), beam((300, -1000, 0), (300, 500, 0))

    joint, a, b = create(XNotchJoint, offset_pair)
    assert is_fallback(joint)

    raw = kinematic_constraint(joint, a)
    assert isinstance(raw, Vector)
    assert raw.length == pytest.approx(1.0)
    assert raw.dot(Vector.from_start_end(b.centerline.midpoint, a.centerline.midpoint)) > 0.0
    assert raw.dot(kinematic_constraint(joint, b)) == pytest.approx(-1.0)


def test_the_fallback_handles_members_crossing_at_their_midpoints():
    # The beam-to-beam vector is zero here; it used to be unitized and divide by zero.
    joint, a, b = create(XNotchJoint, x_pair)
    raw_a = kinematic_constraint(joint, a)
    raw_b = kinematic_constraint(joint, b)
    assert raw_a.length == pytest.approx(1.0)
    assert raw_a.dot(a.centerline.direction) == pytest.approx(0.0, abs=1e-9)
    assert raw_a.dot(b.centerline.direction) == pytest.approx(0.0, abs=1e-9)
    assert raw_a.dot(raw_b) == pytest.approx(-1.0)
