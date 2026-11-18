"""The example frame (compas_timber's examples/model/0001_model.py), with variations for tests.

Every call builds a fresh model, so guids differ between calls -- which is the point: plans, keys and
diffs must not depend on them.
"""

import os

from compas.data import json_load
from compas.geometry import Translation
from compas.geometry import Vector
from compas_timber.connections import LMiterJoint
from compas_timber.connections import TButtJoint
from compas_timber.connections import TLapJoint
from compas_timber.elements import Beam
from compas_timber.model import TimberModel

LINES = os.path.join(os.path.dirname(__file__), "..", "examples", "data", "frame_lines.json")
NAMES = ["left post", "brace", "strut", "right post", "bottom rail", "top rail"]

JOINTS = [
    (LMiterJoint, "top rail", "right post"),
    (LMiterJoint, "right post", "bottom rail"),
    (LMiterJoint, "bottom rail", "left post"),
    (LMiterJoint, "left post", "top rail"),
    (TLapJoint, "strut", "brace"),
    (TButtJoint, "brace", "left post"),
    (TButtJoint, "brace", "right post"),
    (TButtJoint, "strut", "bottom rail"),
]


def build_frame(named=True, move=None, without=(), without_joints=()):
    """Build the frame.

    Parameters
    ----------
    named : bool, optional
        Give the beams their names; otherwise they keep the default and get geometric keys.
    move : tuple, optional
        ``(name, (dx, dy, dz))``: translate one beam's centerline before building it.
    without : iterable of str, optional
        Beams to leave out (and their joints with them).
    without_joints : iterable of tuple, optional
        ``(type, a, b)`` entries of :data:`JOINTS` to leave out.

    """
    lines = json_load(LINES)
    model = TimberModel()
    beams = {}
    for name, line in zip(NAMES, lines):
        if name in without:
            continue
        if move and move[0] == name:
            line = line.transformed(Translation.from_vector(Vector(*move[1])))
        beam = Beam.from_centerline(centerline=line, height=120, width=60)
        if named:
            beam.name = name
        model.add_element(beam)
        beams[name] = beam
    for joint_type, a, b in JOINTS:
        if a in beams and b in beams and (joint_type, a, b) not in set(without_joints):
            joint_type.create(model, beams[a], beams[b])
    return model, beams
