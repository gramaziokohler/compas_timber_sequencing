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
the package imports those two (``tests/core/test_layering.py`` enforces both):

* :mod:`compas_timber_sequencing.kinematics` -- how each ``compas_timber`` joint lets its
  members be pulled apart, computed from outside ``compas_timber``, looked up by the joint's
  exact class.
* :mod:`compas_timber_sequencing.timber` -- the adapter from a ``TimberModel`` to a
  :class:`~compas_timber_sequencing.boundary.SequencingInput` and back.

``compas_timber`` itself knows nothing about sequencing. Everything the algorithms are
permitted to know about a model passes through
:class:`~compas_timber_sequencing.boundary.SequencingInput`.

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

from .blocking import build_blocking_graph
from .blocking import disconnecting_elements
from .blocking import extract
from .blocking import fully_blocked
from .blocking import ground_ids
from .blocking import intrinsic_locks
from .blocking import order_dependent_locks
from .blocking import strongly_connected_components
from .blocking import subassemblies
from .boundary import SequencingInput
from .boundary import sort_key
from .compare import HEADER
from .compare import StrategyReport
from .compare import compare_strategies
from .compare import describe_comparison
from .constraints import TOL
from .constraints import Constraint
from .constraints import HalfSpace
from .constraints import SignedAxis
from .constraints import validate_constraints
from .preferences import STRATEGIES
from .preferences import TERMS
from .preferences import ChainStrategy
from .preferences import ClearanceStrategy
from .preferences import GravityStrategy
from .preferences import HeuristicStrategy
from .preferences import LayeredStrategy
from .preferences import PreferenceStrategy
from .preferences import RandomStrategy
from .preferences import RankingContext
from .preferences import SkeletonFirstStrategy
from .preferences import SubassemblyStrategy
from .preferences import TermStrategy
from .preferences import WeightedStrategy
from .preferences import make_strategy
from .result import LOCKED
from .result import ROOMY
from .result import TIGHT
from .result import Locked
from .result import PinConflict
from .result import SequenceResult
from .result import Solution
from .result import StalenessReport
from .result import StuckReport
from .search import DEFAULT_BEAM_WIDTH
from .search import beam_search
from .search import generate
from .solver import APPROACH_DISTANCE
from .solver import PARALLEL_TOL
from .solver import ROOMY_MARGIN
from .solver import candidate_directions
from .solver import classify
from .solver import rank_candidates
from .solver import solve
from .trace import get_tracer
from .trace import set_tracer
from .trace import trace
from .trace import tracing

__version__ = "0.1.0"

__all__ = [
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
