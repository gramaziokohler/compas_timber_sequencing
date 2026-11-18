"""Kinematics for :class:`compas_timber.connections.YButtJoint`."""

from itertools import combinations

from compas.geometry import Point
from compas.geometry import intersection_line_line
from compas_timber.connections import YButtJoint

from .base import register


def average_intersection_of_lines(compas_lines):
    """
    Finds a 3D point approximating the intersection of multiple lines by
    averaging the midpoints of the shortest segments between all line pairs.
    """
    if len(compas_lines) < 2:
        raise ValueError("At least two lines are required.")

    midpoints = []

    for line_a, line_b in combinations(compas_lines, 2):
        result = intersection_line_line(line_a, line_b)

        if result is None:
            continue

        pt_a, pt_b = result

        mid_x = (pt_a[0] + pt_b[0]) / 2.0
        mid_y = (pt_a[1] + pt_b[1]) / 2.0
        mid_z = (pt_a[2] + pt_b[2]) / 2.0

        midpoints.append([mid_x, mid_y, mid_z])

    if not midpoints:
        raise ValueError("No intersections found (all lines might be parallel).")

    num_points = len(midpoints)
    avg_x = sum(p[0] for p in midpoints) / num_points
    avg_y = sum(p[1] for p in midpoints) / num_points
    avg_z = sum(p[2] for p in midpoints) / num_points

    return Point(avg_x, avg_y, avg_z)


@register(YButtJoint)
def constraint(joint, moving_element, assembled_elements=None):
    """Calculates the escape constraint for the Y-Butt joint."""
    if moving_element not in joint.elements:
        raise ValueError("Element is not part of this joint.")
    if assembled_elements is None:
        # Conservative default: every other member of the joint is in place.
        assembled_elements = [element for element in joint.elements if element is not moving_element]

    approx_joint_location = average_intersection_of_lines([joint.main_beam.centerline, joint.cross_beam_a.centerline, joint.cross_beam_b.centerline])

    if moving_element == joint.main_beam:
        kc_vectors = []
        if joint.cross_beam_a in assembled_elements:
            kc_vectors.append(joint.cross_beam_a.ref_sides[joint.cross_beam_ref_side_index(joint.cross_beam_a)].normal)
            if joint.mill_depth:
                cross_beam_a_direction = joint.cross_beam_a.centerline.direction
                if joint.cross_beam_a.endpoint_closest_to_point(approx_joint_location)[0] == "start":
                    cross_beam_a_direction = -cross_beam_a_direction
                kc_vectors.append(cross_beam_a_direction)
        if joint.cross_beam_b in assembled_elements:
            kc_vectors.append(joint.cross_beam_b.ref_sides[joint.cross_beam_ref_side_index(joint.cross_beam_b)].normal)
            if joint.mill_depth:
                cross_beam_b_direction = joint.cross_beam_b.centerline.direction
                if joint.cross_beam_b.endpoint_closest_to_point(approx_joint_location)[0] == "start":
                    cross_beam_b_direction = -cross_beam_b_direction
                kc_vectors.append(cross_beam_b_direction)
        return kc_vectors

    elif moving_element == joint.cross_beam_a:
        kc_vectors = []
        if joint.main_beam in assembled_elements:
            kc_vectors.append(joint.cross_beam_a.ref_sides[joint.cross_beam_ref_side_index(joint.cross_beam_a)].normal * -1)
            if joint.mill_depth:
                cross_beam_a_direction = joint.cross_beam_a.centerline.direction
                if joint.cross_beam_a.endpoint_closest_to_point(approx_joint_location)[0] == "end":
                    cross_beam_a_direction = -cross_beam_a_direction
                kc_vectors.append(cross_beam_a_direction)
        if joint.cross_beam_b in assembled_elements:
            plane_a, plane_b = joint.get_miter_planes(*joint.cross_beams)
            kc_vectors.append(plane_b.normal)
        return kc_vectors

    elif moving_element == joint.cross_beam_b:
        kc_vectors = []
        if joint.main_beam in assembled_elements:
            kc_vectors.append(joint.cross_beam_b.ref_sides[joint.cross_beam_ref_side_index(joint.cross_beam_b)].normal * -1)
            if joint.mill_depth:
                cross_beam_b_direction = joint.cross_beam_b.centerline.direction
                if joint.cross_beam_b.endpoint_closest_to_point(approx_joint_location)[0] == "end":
                    cross_beam_b_direction = -cross_beam_b_direction
                kc_vectors.append(cross_beam_b_direction)
        if joint.cross_beam_a in assembled_elements:
            plane_a, plane_b = joint.get_miter_planes(*joint.cross_beams)
            kc_vectors.append(plane_a.normal)
        return kc_vectors
