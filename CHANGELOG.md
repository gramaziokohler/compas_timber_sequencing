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
* `compas_timber >=3.0,<4` is a required dependency. Only `kinematics/` and `timber/` import it; `core/`, `ranking/` and `timber/collision.py` depend on `compas` only, which `tests/test_layering.py` enforces. `compas_timber` needs no changes.
* Package layout: `core/` (the algorithms on plain data), `ranking/` (`context`, `terms`, `strategies`, `compare` -- the former `preferences` module, split), `kinematics/` (one module per `compas_timber` joint module), `timber/` (`adapter`, `attributes`, `collision`). Everything public is re-exported from the package root, including `generate_assembly_sequence` and `assembly_order`. Tests mirror the layout.
* A nightly CI job runs the tests against `compas_timber`'s main branch, because the kinematics read joint internals.
* Stable element keys (`timber/keys.py`, `element_keys`): a beam's unique name, otherwise a hash of its rounded centerline and section. They survive a rebuild of the model, unlike guids, and are the ids the search runs on, so an unchanged design gives the same sequence on every rebuild.
* `plan_sequence(model, overrides)` returns a `SequencePlan`: the sequence as data of its own, keyed by stable keys, with absolute positions, per-element fit and insertion vector, the overrides and settings used, and a geometry and joints fingerprint of every beam. It saves to and loads from JSON. `plan.check(model)` reports beams missing, new or changed since the plan was made; `apply_plan(model, plan)` writes a plan onto a (possibly rebuilt) model for export.
* `SequenceOverrides`: adjustments recorded as intent -- `place_before` / `place_after`, `keep_order`, `place_by_hand`, `pin` (negative positions count from the end), `use_strategy`, `forget` -- applied by the search as hard constraints. Precedences are new in the search (`generate(..., precedences=...)`); contradicting ones raise and name the cycle, and a precedence the joints cannot honour ends in a stuck report saying so.
* `diff(old, new)`: what changed between two plans -- elements added, removed and moved, insertion vectors that turned, fit and hand-placement changes, beams whose geometry or joints changed.

### Changed

* Sequencing no longer changes the model unless asked: `generate_assembly_sequence(..., apply_to_model=False)` by default, and `plan_sequence` never does. Hand placement is an explicit input (`SequenceOverrides.place_by_hand`, or `manual_set`); flags left on elements by an earlier run are no longer read back.
* Writing a sequence onto the model clears earlier values first, and after a stuck run gives the placed elements their absolute positions (after the unplaced ones) instead of numbering them from 0.
* The adapter identifies elements by stable key by default; `key="guid"` keeps guid ids (the archive regression test uses it).
* The adapter writes the sequence into `element.attributes` only. It no longer writes to `TimberModel`'s private graph, and `compas_timber` no longer declares sequencing node attributes or a `TimberModel.assembly_sequence` property. Use `assembly_order(model)` instead.
* Compared with the archived joint methods: `YButtJoint`'s `assembled_elements` is optional, defaulting to every other member; the fallback no longer divides by zero when two members cross at their midpoints; a debug `print` in the T-step joint is gone.

### Removed

* The legacy `KinematicSequenceGenerator` and `InsertionSolver` were not carried over. They remain on the archived `compas_timber` branch.
