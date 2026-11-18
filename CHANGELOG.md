# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

* Initial release as a standalone package, moved out of `compas_timber` where it lived as `assembly_sequencing` (branch `feature/joint_kinematics`, archived at tag `archive/joint-kinematics`). The algorithms are unchanged: against the `compas_timber` version it was recorded with, every strategy reproduces the archived results exactly (`tests/timber/test_regression.py`).
* Beam search over disassembly states, cone-feasibility solver, blocking graph with intrinsic-lock detection, and ten ranking strategies (`gravity` by default), with `compare_strategies` to score them side by side.
* `compas_timber_sequencing.timber`: the `TimberModel` adapter (`TimberModelAdapter`, `generate_assembly_sequence`, `sequencing_input_from_model`), formerly `compas_timber.planning.sequencing`.
* `compas_timber_sequencing.timber.assembly_order(model)`: the sequenced elements in assembly order, read from the element attributes.
* `compas_timber_sequencing.kinematics`: the joint kinematics, formerly the `get_kinematic_constraint` methods on `compas_timber`'s joint classes. `kinematic_constraint(joint, moving_element, assembled_elements=None)` looks up an explicit registry by the exact joint class (`register`, `registered_joint_types`, `is_fallback`); 16 beam joint classes are registered, one module per `compas_timber.connections` module. A subclass is not covered by its parent's registration, and a test fails when a `compas_timber` beam joint is neither registered nor knowingly left out. `TOliGinaJoint` (project-specific) is left out on purpose. Against those methods it returns bit-identical results on every joint in the tests and in two real models (705 calls).
* `compas_timber >=3.0,<4` is a required dependency. Only `kinematics/` and `timber.py` import it; the algorithms and `collision.py` depend on `compas` only, which `tests/core/test_layering.py` enforces. `compas_timber` needs no changes.
* A nightly CI job runs the tests against `compas_timber`'s main branch, because the kinematics read joint internals.

### Changed

* The adapter writes the sequence into `element.attributes` only. It no longer writes to `TimberModel`'s private graph, and `compas_timber` no longer declares sequencing node attributes or a `TimberModel.assembly_sequence` property. Use `assembly_order(model)` instead.
* Compared with the archived joint methods: `YButtJoint`'s `assembled_elements` is optional, defaulting to every other member; the fallback no longer divides by zero when two members cross at their midpoints; a debug `print` in the T-step joint is gone.

### Removed

* The legacy `KinematicSequenceGenerator` and `InsertionSolver` were not carried over. They remain on the archived `compas_timber` branch.
