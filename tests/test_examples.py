"""The examples run, and say what they claim to."""

import importlib.util
import os

EXAMPLES = os.path.join(os.path.dirname(__file__), "..", "examples")


def _load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(EXAMPLES, name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sequence_model_runs_on_the_compas_timber_example_frame(capsys):
    example = _load("sequence_model")
    result = example.main([])
    output = capsys.readouterr().out

    assert result.is_complete
    assert len(result.order) == 6
    assert result.inferred_total == 0, "every joint in the frame has kinematics"
    for heading in ("1. WHAT EACH JOINT ALLOWS", "2. THE PLAN", "3. STRATEGIES SIDE BY SIDE", "4. ADJUSTING THE PLAN", "5. SAVE, REBUILD, APPLY"):
        assert heading in output
    assert "loads back unchanged: True" in output
    assert "The plan matches the model." in output
    # The adjustment is honoured: the strut goes in before the bottom rail.
    assert result.position("strut") < result.position("bottom rail")


def test_the_example_frame_matches_compas_timbers_example():
    example = _load("sequence_model")
    model = example.build_frame()
    joint_types = sorted(type(joint).__name__ for joint in model.joints)
    assert joint_types == ["LMiterJoint"] * 4 + ["TButtJoint"] * 3 + ["TLapJoint"]
    assert [beam.name for beam in model.beams] == example.FRAME_NAMES
