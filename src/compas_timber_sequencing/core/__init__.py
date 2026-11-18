"""The algorithms, on plain data: what the search may know, the solver, the blocking graph,
the beam search, and the result types.

Depends on ``compas`` only -- never on ``compas_timber``, :mod:`compas_timber_sequencing.kinematics` or
:mod:`compas_timber_sequencing.timber` (``tests/test_layering.py``). :mod:`~compas_timber_sequencing.core.search`
uses a :mod:`~compas_timber_sequencing.ranking` strategy to choose between feasible removals.

Import from the modules, or from the package root.

"""
