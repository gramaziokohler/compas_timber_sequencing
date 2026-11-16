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
* `compas_timber` is the optional `timber` extra; the algorithms depend on `compas` only.

### Changed

* The adapter writes the sequence into `element.attributes` only. It no longer writes to `TimberModel`'s private graph, and `compas_timber` no longer declares sequencing node attributes or a `TimberModel.assembly_sequence` property. Use `assembly_order(model)` instead.

### Removed

* The legacy `KinematicSequenceGenerator` and `InsertionSolver` were not carried over. They remain on the archived `compas_timber` branch.
