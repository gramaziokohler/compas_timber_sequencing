"""Kinematics for :class:`compas_timber.connections.TTenonMortiseJoint`.

It is held by its tenon exactly like the general mortise-tenon joint, so it uses that
implementation, registered here explicitly.
"""

from compas_timber.connections import TTenonMortiseJoint

from .base import register
from .mortise_tenon import constraint

register(TTenonMortiseJoint)(constraint)
