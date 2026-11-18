"""Sequence a timber model, adjust the sequence, and see what changed.

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
2. The plan: order, insertion vector, roomy/tight, clearance angle, by hand or not.
3. How the ranking strategies compare on this model.
4. Adjusting the plan with overrides -- recorded as intent ("the strut before the bottom rail"),
   not as positions -- and the diff against the previous plan.
5. Saving the plan, rebuilding the model (new guids, as a Grasshopper recompute or a new script run
   would), checking that the plan still matches, and writing it onto the model for export.

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

from compas_timber_sequencing import SequenceOverrides
from compas_timber_sequencing import apply_plan
from compas_timber_sequencing import assembly_order
from compas_timber_sequencing import compare_strategies
from compas_timber_sequencing import describe_comparison
from compas_timber_sequencing import diff
from compas_timber_sequencing import plan_sequence
from compas_timber_sequencing import sequencing_input_from_model
from compas_timber_sequencing.kinematics import is_fallback
from compas_timber_sequencing.kinematics import kinematic_constraint
from compas_timber_sequencing.timber import MANUAL_ATTRIBUTE
from compas_timber_sequencing.timber import SEQUENCE_ATTRIBUTE
from compas_timber_sequencing.timber import STATE_ATTRIBUTE
from compas_timber_sequencing.timber import VECTOR_ATTRIBUTE

HERE = os.path.dirname(__file__)
FRAME_LINES = os.path.join(HERE, "data", "frame_lines.json")

# Names for the six centerlines in frame_lines.json, in file order. The names are the beams'
# keys: plans and overrides refer to beams by them, so they survive a rebuild of the model.
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


def show_plan(plan, title):
    print(title)
    print("  {:>3}  {:14s} {:24s} {:6s} {:>9}  {}".format("#", "element", "insertion vector", "fit", "clearance", ""))
    for key in plan.unplaced:
        print("  {:>3}  {:14s} not placed".format("-", key[:14]))
    for key in plan.order:
        step = plan.steps[key]
        vector = step["insertion_vector"]
        vector_text = "-" if vector is None else "({:+.2f}, {:+.2f}, {:+.2f})".format(vector[0], vector[1], vector[2])
        angle = "-" if step["clearance"] is None else "{:.1f} deg".format(step["clearance"])
        print("  {:>3}  {:14s} {:24s} {:6s} {:>9}  {}".format(plan.position(key), key[:14], vector_text, step["state"], angle, "by hand" if step["by_hand"] else ""))
    for kind, key, message in plan.staleness:
        print("  stale {}: {}".format(kind, message))
    print(
        "  complete: {} | intrinsic locks: {} | excluded: {} | constraints from the fallback: {}".format(
            plan.is_complete, len(plan.intrinsic_locks), len(plan.excluded), plan.inferred_total
        )
    )
    print()


def show_strategies(model):
    print("3. STRATEGIES SIDE BY SIDE")
    reports = compare_strategies(sequencing_input_from_model(model))
    print(describe_comparison(reports))
    print()


def adjust(model, plan):
    """Adjust the frame's plan the way a designer would: by stating intent."""
    print("4. ADJUSTING THE PLAN")
    overrides = SequenceOverrides()
    overrides.keep_order(plan, until=2)  # keep the first two steps in the order planned
    overrides.place_before("strut", "bottom rail")  # but the strut should go in before the bottom rail
    print("  overrides: {}".format(overrides.precedences))
    adjusted = plan_sequence(model, overrides)
    print("  what changed:")
    for line in diff(plan, adjusted).describe().splitlines():
        print("    " + line)
    print()
    show_plan(adjusted, "  ADJUSTED PLAN")
    return adjusted


def save_rebuild_apply(plan, rebuild):
    print("5. SAVE, REBUILD, APPLY")
    saved = json_dumps(plan)  # plan.to_json("plan.json") writes the same to a file
    restored = json_loads(saved)
    print("  the plan saves to {} characters of JSON and loads back unchanged: {}".format(len(saved), diff(plan, restored).is_empty))

    model = rebuild()  # every guid is new
    check = restored.check(model)
    print("  against a rebuilt model: {}".format(check.describe()))
    apply_plan(model, restored)

    first = assembly_order(model)[0]
    print("  written into element.attributes, e.g. on '{}':".format(label(first)))
    for key in (SEQUENCE_ATTRIBUTE, VECTOR_ATTRIBUTE, MANUAL_ATTRIBUTE, STATE_ATTRIBUTE):
        print("    {:26s} {!r}".format(key, first.attributes.get(key)))
    print("  order from the attributes: " + " -> ".join(label(element) for element in assembly_order(model)))
    print()
    return model


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    path = argv[0] if argv else None
    rebuild = (lambda: load_model(path)) if path else build_frame
    model = rebuild()
    print("{} beams, {} joints\n".format(len(model.beams), len(list(model.joints))))

    show_kinematics(model)
    plan = plan_sequence(model)  # the model itself is not changed
    show_plan(plan, "2. THE PLAN")
    show_strategies(model)
    if path is None:
        plan = adjust(model, plan)
    save_rebuild_apply(plan, rebuild)
    return plan


if __name__ == "__main__":
    main()
