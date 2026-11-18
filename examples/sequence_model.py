"""Sequence a timber model: assembly order, insertion vectors, and where the result is stored.

By default this builds the six-beam frame from ``compas_timber``'s own example
(``examples/model/0001_model.py`` on its main branch): two posts, a top and a bottom rail
joined with L-miters, a diagonal brace T-butted into the posts, and a strut T-lapped into the
brace and T-butted onto the bottom rail. The centerlines are in ``data/frame_lines.json``,
copied from ``compas_timber``'s ``examples/model/lines.json``.

Pass the path of any saved ``TimberModel`` JSON to sequence that instead, for example::

    python examples/sequence_model.py
    python examples/sequence_model.py tests/timber/data/model_test.json

What it shows, in order:

1. What each joint allows each of its members to do (the kinematics).
2. The assembly sequence: order, insertion vector, roomy/tight, clearance angle, by hand or not.
3. How the ranking strategies compare on this model.
4. Where the result is stored -- four keys in each element's ``attributes`` -- and that it
   survives a round trip through JSON.

``process_joinery()`` is not called: the kinematics read each joint's planes and reference sides,
not the processed geometry, so sequencing does not need the Brep backend.
"""

import os
import sys

from compas.data import json_dumps
from compas.data import json_load
from compas.data import json_loads
from compas.geometry import Line
from compas_timber.connections import LMiterJoint
from compas_timber.connections import TButtJoint
from compas_timber.connections import TLapJoint
from compas_timber.elements import Beam
from compas_timber.model import TimberModel

from compas_timber_sequencing import assembly_order
from compas_timber_sequencing import compare_strategies
from compas_timber_sequencing import describe_comparison
from compas_timber_sequencing import generate_assembly_sequence
from compas_timber_sequencing import sequencing_input_from_model
from compas_timber_sequencing.kinematics import is_fallback
from compas_timber_sequencing.kinematics import kinematic_constraint
from compas_timber_sequencing.timber import MANUAL_ATTRIBUTE
from compas_timber_sequencing.timber import SEQUENCE_ATTRIBUTE
from compas_timber_sequencing.timber import STATE_ATTRIBUTE
from compas_timber_sequencing.timber import VECTOR_ATTRIBUTE

HERE = os.path.dirname(__file__)
FRAME_LINES = os.path.join(HERE, "data", "frame_lines.json")

# Names for the six centerlines in frame_lines.json, in file order.
FRAME_NAMES = ["left post", "brace", "strut", "right post", "bottom rail", "top rail"]


def build_frame():
    """The frame from compas_timber's examples/model/0001_model.py, with named beams."""
    lines = json_load(FRAME_LINES)
    model = TimberModel()
    for name, line in zip(FRAME_NAMES, lines):
        beam = Beam.from_centerline(centerline=line, height=120, width=60)
        beam.name = name
        model.add_element(beam)
    left_post, brace, strut, right_post, bottom_rail, top_rail = model.beams

    # The frame: L-miters at the four corners.
    LMiterJoint.create(model, top_rail, right_post)
    LMiterJoint.create(model, right_post, bottom_rail)
    LMiterJoint.create(model, bottom_rail, left_post)
    LMiterJoint.create(model, left_post, top_rail)
    # The inner members.
    TLapJoint.create(model, strut, brace)
    TButtJoint.create(model, brace, left_post)
    TButtJoint.create(model, brace, right_post)
    TButtJoint.create(model, strut, bottom_rail)
    return model


def load_model(path):
    model = json_load(path)
    if not isinstance(model, TimberModel):
        raise SystemExit("{} does not contain a TimberModel".format(path))
    return model


def label(element):
    return element.name if element.name and element.name != type(element).__name__ else str(element.guid)[:8]


def describe_constraint(raw):
    if isinstance(raw, Line):
        d = raw.direction.unitized()
        return "slides out along ({:+.2f}, {:+.2f}, {:+.2f}) only".format(d.x, d.y, d.z)
    vectors = raw if isinstance(raw, (list, tuple)) else [raw]
    if not vectors:
        return "free"
    return "{} half-space(s): ".format(len(vectors)) + ", ".join("({:+.2f}, {:+.2f}, {:+.2f})".format(*v.unitized()) for v in vectors)


def show_kinematics(model):
    print("1. WHAT EACH JOINT ALLOWS (the other member held fixed)")
    for joint in model.joints:
        note = "  [fallback: no kinematics written for this joint type]" if is_fallback(joint) else ""
        print("  {}{}".format(type(joint).__name__, note))
        for element in joint.elements:
            print("    {:12s} {}".format(label(element), describe_constraint(kinematic_constraint(joint, element))))
    print()


def show_sequence(result, adapter):
    print("2. ASSEMBLY SEQUENCE")
    print("  {:>3}  {:12s} {:28s} {:6s} {:>9}  {}".format("#", "element", "insertion vector", "fit", "clearance", ""))
    for index, element_id in enumerate(result.order):
        element = adapter.elements_by_id[element_id]
        vector = result.insertion_vectors[element_id]
        extraction = result.extraction[element_id]
        vector_text = "-" if vector is None else "({:+.2f}, {:+.2f}, {:+.2f})".format(vector.x, vector.y, vector.z)
        angle = "-" if vector is None else "{:.1f} deg".format(extraction.angle_degrees)
        by_hand = "by hand" if element_id in result.manual_set else ""
        print("  {:>3}  {:12s} {:28s} {:6s} {:>9}  {}".format(index, label(element), vector_text, extraction.state, angle, by_hand))
    if not result.is_complete:
        print()
        print(result.stuck.describe() if result.stuck else repr(result.pin_conflict))
    for element_id, reason in sorted(result.excluded.items()):
        print("  excluded: {} ({})".format(element_id[:8], reason))
    print("  complete: {} | intrinsic locks: {} | constraints from the fallback: {}".format(result.is_complete, len(result.intrinsic_locks), result.inferred_total))
    print()


def show_strategies(model):
    print("3. STRATEGIES SIDE BY SIDE")
    reports = compare_strategies(sequencing_input_from_model(model))
    print(describe_comparison(reports))
    print()


def show_storage(model):
    print("4. WHERE THE RESULT IS STORED")
    first = assembly_order(model)[0]
    print("  element.attributes of '{}':".format(label(first)))
    for key in (SEQUENCE_ATTRIBUTE, VECTOR_ATTRIBUTE, MANUAL_ATTRIBUTE, STATE_ATTRIBUTE):
        print("    {:26s} {!r}".format(key, first.attributes.get(key)))

    restored = json_loads(json_dumps(model))
    before = [str(element.guid) for element in assembly_order(model)]
    after = [str(element.guid) for element in assembly_order(restored)]
    print("  after a JSON round trip, assembly_order() returns the same order: {}".format(before == after))
    print("  order: " + " -> ".join(label(element) for element in assembly_order(restored)))
    print()


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    model = load_model(argv[0]) if argv else build_frame()
    print("{} beams, {} joints\n".format(len(model.beams), len(list(model.joints))))

    show_kinematics(model)
    result, adapter = generate_assembly_sequence(model)  # also writes into element.attributes
    show_sequence(result, adapter)
    show_strategies(model)
    show_storage(model)
    return result


if __name__ == "__main__":
    main()
