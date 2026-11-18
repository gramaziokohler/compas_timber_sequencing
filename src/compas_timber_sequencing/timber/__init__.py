"""Everything that knows a ``TimberModel``: the adapter that turns a model into a
:class:`~compas_timber_sequencing.core.boundary.SequencingInput` and writes the result back
(:mod:`~compas_timber_sequencing.timber.adapter`, :mod:`~compas_timber_sequencing.timber.attributes`), and the
swept-box collision test it uses (:mod:`~compas_timber_sequencing.timber.collision`, pure ``compas`` geometry).

"""

from .adapter import TimberModelAdapter
from .adapter import constraints_from_joint_output
from .adapter import generate_assembly_sequence
from .adapter import sequencing_input_from_model
from .attributes import MANUAL_ATTRIBUTE
from .attributes import SEQUENCE_ATTRIBUTE
from .attributes import STATE_ATTRIBUTE
from .attributes import VECTOR_ATTRIBUTE
from .attributes import assembly_order

__all__ = [
    "MANUAL_ATTRIBUTE",
    "SEQUENCE_ATTRIBUTE",
    "STATE_ATTRIBUTE",
    "TimberModelAdapter",
    "VECTOR_ATTRIBUTE",
    "assembly_order",
    "constraints_from_joint_output",
    "generate_assembly_sequence",
    "sequencing_input_from_model",
]
