"""Kinematic assembly sequencing.

Produces an assembly order for a set of jointed elements, a per-element unit insertion
vector for trajectory planning, and an explicit set of elements that must be placed by
hand.

Three consumers, in priority order:

1. **Robotic assembly** -- `compas_fab` consumes the unit insertion vector. The approach
   distance is fixed, short, and decided downstream.
2. **Visualization** -- display of order and direction.
3. **Human fabrication** -- the hand-placement set is a real fabrication instruction.

The tool must work acceptably on all designs and let the user override both the order and
the hand-placement set. Every design has exceptions; the job is to get close and make the
exceptions easy to express, not to be right unaided.

Packaging
---------
The algorithms depend on ``compas.geometry`` and the standard library only. No numpy, no
scipy: the solver is dot products and cross products. Python 3.9 syntax throughout, because
this must run under Rhino's CPython.

``compas_timber`` is a required dependency, but only two parts import it, and nothing else in
the package imports those two (``tests/test_layering.py`` enforces both):

* :mod:`compas_timber_sequencing.kinematics` -- how each ``compas_timber`` joint lets its
  members be pulled apart, computed from outside ``compas_timber``, looked up by the joint's
  exact class.
* :mod:`compas_timber_sequencing.timber` -- the adapter from a ``TimberModel`` to a
  :class:`~compas_timber_sequencing.core.boundary.SequencingInput` and back.

``compas_timber`` itself knows nothing about sequencing. Everything the algorithms are
permitted to know about a model passes through
:class:`~compas_timber_sequencing.core.boundary.SequencingInput`.

Known limitations, accepted
---------------------------
* **Cone feasibility is infinitesimal freedom.** ``min_i(n_i . d) > 0`` says the element
  can *begin* to move, not that it can travel far enough to clear the assembly. The
  swept broad-phase check mitigates this over the fixed approach distance; it does not
  eliminate the gap for deep mortises, scarfs or long engagements.
* **No clearance distance.** Maximum travel before collision is expensive and the
  approach distance is fixed and short downstream, so it is deliberately not computed.
* **The robot is not a point.** Gripper and arm collision with already-placed elements is
  `compas_fab`'s domain. A sequence certified here can still be unbuildable for reach
  reasons.
* **Monotone sequences only.** No assemble, disassemble, reassemble.

Examples
--------
>>> from compas.geometry import Vector
>>> from compas_timber_sequencing import HalfSpace, SequencingInput, generate
>>> ids = ["post", "beam"]
>>> constraints = {
...     ("post", frozenset(["beam"])): [HalfSpace(Vector(0, 0, -1))],
...     ("beam", frozenset(["post"])): [HalfSpace(Vector(0, 0, 1))],
... }
>>> data = SequencingInput(
...     element_ids=ids,
...     neighbors={"post": {"beam"}, "beam": {"post"}},
...     base_z={"post": 0.0, "beam": 1.0},
...     centroid_z={"post": 0.5, "beam": 1.0},
...     length={"post": 1.0, "beam": 2.0},
...     constraints=lambda i, active: constraints.get((i, frozenset(active)), []),
... )
>>> generate(data).order
['post', 'beam']

"""

from .core.blocking import build_blocking_graph
from .core.blocking import disconnecting_elements
from .core.blocking import extract
from .core.blocking import fully_blocked
from .core.blocking import ground_ids
from .core.blocking import intrinsic_locks
from .core.blocking import order_dependent_locks
from .core.blocking import strongly_connected_components
from .core.blocking import subassemblies
from .core.boundary import SequencingInput
from .core.boundary import sort_key
from .ranking.compare import HEADER
from .ranking.compare import StrategyReport
from .ranking.compare import compare_strategies
from .ranking.compare import describe_comparison
from .core.constraints import TOL
from .core.constraints import Constraint
from .core.constraints import HalfSpace
from .core.constraints import SignedAxis
from .core.constraints import validate_constraints
from .ranking.strategies import STRATEGIES
from .ranking.terms import TERMS
from .ranking.strategies import ChainStrategy
from .ranking.strategies import ClearanceStrategy
from .ranking.strategies import GravityStrategy
from .ranking.strategies import HeuristicStrategy
from .ranking.strategies import LayeredStrategy
from .ranking.context import PreferenceStrategy
from .ranking.strategies import RandomStrategy
from .ranking.context import RankingContext
from .ranking.strategies import SkeletonFirstStrategy
from .ranking.strategies import SubassemblyStrategy
from .ranking.strategies import TermStrategy
from .ranking.strategies import WeightedStrategy
from .ranking.strategies import make_strategy
from .core.result import LOCKED
from .core.result import ROOMY
from .core.result import TIGHT
from .core.result import Locked
from .core.result import PinConflict
from .core.result import SequenceResult
from .core.result import Solution
from .core.result import StalenessReport
from .core.result import StuckReport
from .core.search import DEFAULT_BEAM_WIDTH
from .core.search import beam_search
from .core.search import generate
from .core.solver import APPROACH_DISTANCE
from .core.solver import PARALLEL_TOL
from .core.solver import ROOMY_MARGIN
from .core.solver import candidate_directions
from .core.solver import classify
from .core.solver import rank_candidates
from .core.solver import solve
from .core.trace import get_tracer
from .core.trace import set_tracer
from .core.trace import trace
from .core.trace import tracing
from .timber.adapter import TimberModelAdapter
from .timber.adapter import generate_assembly_sequence
from .timber.adapter import sequencing_input_from_model
from .timber.attributes import assembly_order
from .core.overrides import SequenceOverrides
from .timber.diff import PlanDiff
from .timber.diff import diff
from .timber.keys import element_keys
from .timber.plan import PlanCheck
from .timber.plan import SequencePlan
from .timber.plan import apply_plan
from .timber.plan import plan_sequence

__version__ = "0.1.0"

__all__ = [
    "PlanCheck",
    "PlanDiff",
    "SequenceOverrides",
    "SequencePlan",
    "apply_plan",
    "diff",
    "element_keys",
    "plan_sequence",
    "sequencing_input_from_model",
    "generate_assembly_sequence",
    "assembly_order",
    "TimberModelAdapter",
    "APPROACH_DISTANCE",
    "ChainStrategy",
    "ClearanceStrategy",
    "Constraint",
    "DEFAULT_BEAM_WIDTH",
    "GravityStrategy",
    "HEADER",
    "HalfSpace",
    "HeuristicStrategy",
    "LOCKED",
    "LayeredStrategy",
    "Locked",
    "PARALLEL_TOL",
    "PinConflict",
    "PreferenceStrategy",
    "ROOMY",
    "ROOMY_MARGIN",
    "RandomStrategy",
    "RankingContext",
    "STRATEGIES",
    "SequenceResult",
    "SequencingInput",
    "SignedAxis",
    "SkeletonFirstStrategy",
    "Solution",
    "StalenessReport",
    "StrategyReport",
    "StuckReport",
    "SubassemblyStrategy",
    "TERMS",
    "TIGHT",
    "TOL",
    "TermStrategy",
    "WeightedStrategy",
    "beam_search",
    "build_blocking_graph",
    "candidate_directions",
    "classify",
    "compare_strategies",
    "describe_comparison",
    "disconnecting_elements",
    "extract",
    "fully_blocked",
    "generate",
    "get_tracer",
    "ground_ids",
    "intrinsic_locks",
    "make_strategy",
    "order_dependent_locks",
    "rank_candidates",
    "set_tracer",
    "solve",
    "sort_key",
    "strongly_connected_components",
    "subassemblies",
    "trace",
    "tracing",
    "validate_constraints",
]
