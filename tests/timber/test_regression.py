"""Behavioural regression against the sequencer as it was before it left ``compas_timber``.

``data/archive_baseline.json`` was recorded from ``compas_timber`` at commit ``72b3a26b03``
(branch ``feature/joint_kinematics``, archived at tag ``archive/joint-kinematics``), where
this package still lived as ``assembly_sequencing`` inside the ``compas_timber`` wheel. It
holds the full result -- order, insertion vectors, extraction states, hand-placement sets,
locks -- of every registered strategy, with the swept check on and off, on two models.

Run against that old ``compas_timber``, this package reproduces all 28 results exactly.
Against current ``compas_timber`` two of them differ, for a reason recorded in
:data:`KNOWN_DIFFERENCES`; anything else changing is a regression.

"""

import os
import uuid

import pytest
from compas.data import json_load
from compas.geometry import Line
from compas.geometry import Point

from compas_timber.connections import TButtJoint
from compas_timber.elements import Beam
from compas_timber.model import TimberModel
from compas_timber_sequencing import STRATEGIES
from compas_timber_sequencing.timber import generate_assembly_sequence

HERE = os.path.dirname(__file__)

KNOWN_DIFFERENCES = {
    "data/model_test.json | clearance | geometry=False": (
        "compas_timber 3.0 computes beam frames differently in the last bit (<= 2.2e-16 on every "
        "constraint normal; all other inputs are bit-identical). The clearance strategy ranks on the "
        "unrounded margin ahead of height, so near-ties between equal margins flip."
    ),
    "data/model_test.json | clearance | geometry=True": "as above",
}


def portal():
    model = TimberModel()
    beams = [
        Beam.from_centerline(Line(Point(0, 0, 0), Point(0, 0, 2000)), 100.0, 100.0),
        Beam.from_centerline(Line(Point(3000, 0, 0), Point(3000, 0, 2000)), 100.0, 100.0),
        Beam.from_centerline(Line(Point(0, 0, 2000), Point(3000, 0, 2000)), 100.0, 100.0),
    ]
    # Fixed guids: ties between the two mirror-image posts are broken by id, and so is the
    # random strategy's shuffle; the baseline was recorded with these same ids.
    for index, beam in enumerate(beams):
        beam._guid = uuid.UUID(int=index + 1)
        model.add_element(beam)
    TButtJoint.create(model, beams[2], beams[0])
    TButtJoint.create(model, beams[2], beams[1])
    return model


MODELS = {
    "portal": portal,
    "data/model_test.json": lambda: json_load(os.path.join(HERE, "data", "model_test.json")),
}


def _vector(value):
    return None if value is None else [round(value[0], 6), round(value[1], 6), round(value[2], 6)]


def summarize(model, result):
    key = {str(element.guid): "e{}".format(index) for index, element in enumerate(model.elements())}
    vectors = result.insertion_vectors
    return {
        "complete": result.is_complete,
        "order": [key[i] for i in result.order],
        "vectors": {key[i]: _vector(vectors[i]) for i in sorted(vectors)},
        "states": {key[i]: result.extraction[i].state for i in sorted(result.extraction)},
        "manual": sorted(key[i] for i in result.manual_set),
        "proposed_manual": sorted(key[i] for i in result.proposed_manual_set),
        "intrinsic_locks": sorted(sorted(key[i] for i in cluster) for cluster in result.intrinsic_locks),
        "excluded": sorted(key.get(i, i) for i in result.excluded),
        "stuck_step": None if result.stuck is None else result.stuck.step_index,
    }


def _load_baseline():
    import json

    with open(os.path.join(HERE, "data", "archive_baseline.json")) as handle:
        return json.load(handle)


BASELINE = _load_baseline()


@pytest.mark.parametrize("label", sorted(BASELINE))
def test_the_sequence_matches_the_archived_implementation(label):
    model_name, strategy_name, geometry = [part.strip() for part in label.split("|")]
    if label in KNOWN_DIFFERENCES:
        pytest.xfail(KNOWN_DIFFERENCES[label])

    model = MODELS[model_name]()
    result, _ = generate_assembly_sequence(
        model,
        strategy=STRATEGIES[strategy_name](),
        use_geometry=geometry == "geometry=True",
        apply_to_model=False,
    )
    assert summarize(model, result) == BASELINE[label]
