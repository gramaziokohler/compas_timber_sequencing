"""The swept-box collision test, on plain boxes: no model, no compas_timber.

The adapter-level tests in ``tests/timber/test_adapter.py`` check that the right elements are tested;
these check the geometry itself.
"""

import math

from compas.geometry import Box
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector

from compas_timber_sequencing.timber.collision import SWEEP_START_FRACTION
from compas_timber_sequencing.timber.collision import bounds_overlap
from compas_timber_sequencing.timber.collision import box_bounds
from compas_timber_sequencing.timber.collision import swept_aabb
from compas_timber_sequencing.timber.collision import swept_box_hits_box

UP = Vector(0, 0, 1)
DOWN = Vector(0, 0, -1)


def cube(x=0.0, y=0.0, z=0.0, size=1.0):
    return Box(size, size, size, frame=Frame(Point(x, y, z), Vector(1, 0, 0), Vector(0, 1, 0)))


def sweep_hits(moving, direction, distance, other):
    return swept_box_hits_box(moving, direction, distance * SWEEP_START_FRACTION, distance, other)


def test_a_box_beside_the_path_is_not_hit():
    assert not sweep_hits(cube(), UP, 10.0, cube(x=5.0))


def test_a_box_in_the_path_is_hit():
    assert sweep_hits(cube(), UP, 10.0, cube(z=5.0))


def test_a_box_beyond_the_approach_distance_is_not_hit():
    assert not sweep_hits(cube(), UP, 10.0, cube(z=50.0))


def test_the_sweep_starts_a_little_way_out():
    # Touching face to face at rest: moving away must not count as a collision.
    assert not sweep_hits(cube(), DOWN, 10.0, cube(z=1.0))


def test_the_exact_test_is_not_fooled_by_the_empty_corner_of_a_diagonal():
    # A long, thin member at 45 degrees: its axis-aligned bounds are mostly empty space.
    diagonal = Box(10.0, 0.2, 0.2, frame=Frame(Point(0, 0, 0), Vector(1, 1, 0), Vector(-1, 1, 0)))
    in_the_corner = cube(x=3.0, y=-3.0, size=0.5)

    assert bounds_overlap(swept_aabb(diagonal, UP, 1.0), box_bounds(in_the_corner)), "the broad phase should be fooled"
    assert not sweep_hits(diagonal, UP, 1.0, in_the_corner)


def test_the_exact_test_still_hits_a_box_on_the_diagonal():
    diagonal = Box(10.0, 0.2, 0.2, frame=Frame(Point(0, 0, 0), Vector(1, 1, 0), Vector(-1, 1, 0)))
    on_the_line = cube(x=3.0, y=3.0, z=0.5, size=0.5)
    assert sweep_hits(diagonal, UP, 1.0, on_the_line)


def test_boxes_already_overlapping_only_block_a_direction_that_goes_deeper():
    # Overlapping by 0.2 at rest, as unjointed beams in a reciprocal frame do.
    moving, other = cube(), cube(z=0.8)
    assert swept_box_hits_box(moving, UP, 0.025, 0.5, other), "towards the other box is deeper"
    assert not swept_box_hits_box(moving, DOWN, 0.025, 0.5, other), "away from it is a way out"


def test_sliding_along_a_shared_face_is_not_a_collision():
    # Overlapping slightly, sliding parallel to the contact: the depth does not change.
    moving, other = cube(), cube(z=0.9)
    assert not swept_box_hits_box(moving, Vector(1, 0, 0), 0.025, 0.5, other)


def test_swept_bounds_cover_the_start_and_the_end_of_the_sweep():
    low_x, low_y, low_z, high_x, high_y, high_z = swept_aabb(cube(), UP, 10.0)
    assert low_z == SWEEP_START_FRACTION * 10.0 - 0.5
    assert high_z == 10.0 + 0.5
    assert (low_x, high_x, low_y, high_y) == (-0.5, 0.5, -0.5, 0.5)


def test_bounds_that_only_touch_do_not_overlap():
    assert not bounds_overlap((0, 0, 0, 1, 1, 1), (1, 0, 0, 2, 1, 1))
    assert bounds_overlap((0, 0, 0, 1, 1, 1), (0.5, 0.5, 0.5, 2, 2, 2))


def test_a_rotated_box_bounds_are_its_corners_extent():
    rotated = Box(2.0, 2.0, 2.0, frame=Frame(Point(0, 0, 0), Vector(1, 1, 0), Vector(-1, 1, 0)))
    low_x, _, _, high_x, _, _ = box_bounds(rotated)
    assert math.isclose(high_x, math.sqrt(2.0), rel_tol=1e-9)
    assert math.isclose(low_x, -math.sqrt(2.0), rel_tol=1e-9)
