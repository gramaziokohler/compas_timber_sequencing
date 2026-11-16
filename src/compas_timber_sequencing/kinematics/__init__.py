"""Joint kinematics for ``compas_timber`` joints.

How each joint lets its members be pulled apart, computed from outside ``compas_timber`` by reading
the joint's planes, reference sides and parameters. :func:`kinematic_constraint` is the single entry
point; it dispatches on the joint's class.

Implemented for :class:`~compas_timber.connections.ButtJoint` and its T, L and Y variants, the L, T and
X laps, the L-French-ridge lap, the L-miter, :class:`~compas_timber.connections.MortiseTenonJoint`
(and through it the T and L tenon-mortise and the Oligina joints), the T-dovetail, the T-step joint,
the T-birdsmouth, the I-simple-scarf, and :class:`~compas_timber.connections.PlateJoint` (and through
it the plate and panel joints). Everything else gets the permissive fallback; see :func:`is_fallback`.

To support another joint, register it::

    from compas_timber_sequencing.kinematics import kinematic_constraint


    @kinematic_constraint.register(MyJoint)
    def _(joint, moving_element, assembled_elements=None): ...

Coupling to ``compas_timber``
-----------------------------
These functions read joint internals that ``compas_timber`` does not promise to keep stable.
Almost all of them are public attributes (``main_beam``, ``butt_plane``, ``cutting_plane_a``,
``mill_depth``, the ref-side indices, ...); two are private methods, ``ISimpleScarf._get_beam_side``
and ``TBirdsmouthJoint._get_cutting_planes``. A scheduled CI job runs the tests against
``compas_timber``'s main branch to catch a rename upstream early.

"""

from .base import is_fallback
from .base import kinematic_constraint

# Imported for their registrations.
from . import butt  # noqa: F401
from . import lap  # noqa: F401
from . import miter  # noqa: F401
from . import mortise_tenon  # noqa: F401
from . import plate  # noqa: F401
from . import scarf  # noqa: F401
from . import step  # noqa: F401

__all__ = [
    "is_fallback",
    "kinematic_constraint",
]
