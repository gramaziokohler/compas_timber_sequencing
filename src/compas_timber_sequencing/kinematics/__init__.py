"""Joint kinematics for ``compas_timber`` beam joints.

How each joint lets its members be pulled apart, computed from outside ``compas_timber`` by reading
the joint's planes, reference sides and parameters. :func:`kinematic_constraint` is the single entry
point.

Layout
------
One module per ``compas_timber.connections`` module, with the same name: the kinematics of
``TDovetailJoint`` (``compas_timber/connections/t_dovetail.py``) are in ``kinematics/t_dovetail.py``.
Each module defines a ``constraint(joint, moving_element, assembled_elements=None)`` function and
registers it for its joint class.

Registry
--------
Lookup is by the **exact** joint class (:func:`register`): a subclass of a registered joint is not
covered unless it is registered too. Kinematics are specific to each joint and are written and
checked by hand, so a joint nobody has looked at -- including a new subclass -- gets the permissive
fallback and is reported by :func:`is_fallback`, rather than silently inheriting an answer.
``TTenonMortiseJoint`` and ``LTenonMortiseJoint`` share the ``MortiseTenonJoint`` implementation and
are registered for it explicitly.

To support another joint::

    from compas_timber_sequencing.kinematics import register


    @register(MyJoint)
    def constraint(joint, moving_element, assembled_elements=None): ...

Coupling to ``compas_timber``
-----------------------------
These functions read joint internals that ``compas_timber`` does not promise to keep stable.
Almost all of them are public attributes (``main_beam``, ``butt_plane``, ``cutting_plane_a``,
``mill_depth``, the ref-side indices, ...); two are private methods, ``ISimpleScarf._get_beam_side``
and ``TBirdsmouthJoint._get_cutting_planes``. A scheduled CI job runs the tests against
``compas_timber``'s main branch to catch a rename upstream early.

"""

from .base import fallback_constraint
from .base import is_fallback
from .base import kinematic_constraint
from .base import register
from .base import registered_joint_types
from .base import unregister

# Every supported joint, one module each, imported for its registration.
from . import butt_joint  # noqa: F401  ButtJoint
from . import t_butt  # noqa: F401  TButtJoint
from . import l_butt  # noqa: F401  LButtJoint
from . import y_butt  # noqa: F401  YButtJoint
from . import l_lap  # noqa: F401  LLapJoint
from . import t_lap  # noqa: F401  TLapJoint
from . import x_lap  # noqa: F401  XLapJoint
from . import l_french_ridge_lap  # noqa: F401  LFrenchRidgeLapJoint
from . import l_miter  # noqa: F401  LMiterJoint
from . import mortise_tenon  # noqa: F401  MortiseTenonJoint
from . import t_tenon_mortise  # noqa: F401  TTenonMortiseJoint
from . import l_tenon_mortise  # noqa: F401  LTenonMortiseJoint
from . import t_dovetail  # noqa: F401  TDovetailJoint
from . import t_step_joint  # noqa: F401  TStepJoint
from . import t_birdsmouth  # noqa: F401  TBirdsmouthJoint
from . import i_simple_scarf  # noqa: F401  ISimpleScarf

__all__ = [
    "fallback_constraint",
    "is_fallback",
    "kinematic_constraint",
    "register",
    "registered_joint_types",
    "unregister",
]
