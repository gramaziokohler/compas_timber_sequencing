"""Everything that knows a ``TimberModel``.

* :mod:`~compas_timber_sequencing.timber.keys` -- stable element keys that survive a rebuild.
* :mod:`~compas_timber_sequencing.timber.adapter` -- the model as a
  :class:`~compas_timber_sequencing.core.boundary.SequencingInput`.
* :mod:`~compas_timber_sequencing.timber.plan` -- :func:`plan_sequence`, the :class:`SequencePlan` it
  returns, and :func:`apply_plan` to write a plan onto a model.
* :mod:`~compas_timber_sequencing.timber.diff` -- what changed between two plans.
* :mod:`~compas_timber_sequencing.timber.attributes` -- the four element attributes a sequence is
  written to.
* :mod:`~compas_timber_sequencing.timber.collision` -- the swept-box collision test (pure ``compas``
  geometry).

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
from .diff import PlanDiff
from .diff import diff
from .keys import KEY_TOLERANCE
from .keys import element_keys
from .keys import geometric_key
from .plan import PlanCheck
from .plan import SequencePlan
from .plan import apply_plan
from .plan import fingerprints
from .plan import plan_sequence

__all__ = [
    "KEY_TOLERANCE",
    "PlanCheck",
    "PlanDiff",
    "SequencePlan",
    "apply_plan",
    "diff",
    "element_keys",
    "fingerprints",
    "geometric_key",
    "plan_sequence",
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
