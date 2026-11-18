"""The package's layering: only the model-facing folders touch compas_timber.

``compas_timber`` is a required dependency, so nothing at install time keeps it out of the
algorithms any more. This test does: it reads every module's imports and fails when

* anything in ``core/`` or ``ranking/`` imports ``compas_timber``,
* anything in ``core/`` or ``ranking/`` imports ``kinematics/`` or ``timber/`` -- the algorithms
  must not reach back up into the layer that knows about models, or
* ``timber/collision.py`` imports ``compas_timber`` -- it is pure geometry on boxes.

The package root ``__init__`` is the public API and may re-export from every folder.

That keeps the solver, the search and the ranking testable on hand-built inputs, and keeps every
dependency on ``compas_timber`` internals in ``kinematics/`` and ``timber/``.
"""

import ast
import os

import pytest

import compas_timber_sequencing

PACKAGE = "compas_timber_sequencing"
ROOT = os.path.dirname(compas_timber_sequencing.__file__)

MODEL_FACING = ("compas_timber_sequencing.kinematics", "compas_timber_sequencing.timber")
"""Modules (and packages) allowed to import compas_timber."""


def _modules():
    for directory, _, files in os.walk(ROOT):
        for name in sorted(files):
            if not name.endswith(".py"):
                continue
            path = os.path.join(directory, name)
            relative = os.path.relpath(path, ROOT)[: -len(".py")].replace(os.sep, ".")
            module = PACKAGE if relative == "__init__" else "{}.{}".format(PACKAGE, relative[: -len(".__init__")] if relative.endswith(".__init__") else relative)
            yield module, path, name == "__init__.py"


def _imports(module, path, is_package):
    tree = ast.parse(open(path, encoding="utf-8").read())
    package = module if is_package else module.rsplit(".", 1)[0]
    found = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            found.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                base = package.split(".")
                base = base[: len(base) - (node.level - 1)]
                target = ".".join(base + ([node.module] if node.module else []))
            else:
                target = node.module
            found.add(target)
            # `from . import kinematics` names a submodule, not an attribute
            found.update("{}.{}".format(target, alias.name) for alias in node.names)
    return found


def _is_within(name, prefixes):
    return any(name == prefix or name.startswith(prefix + ".") for prefix in prefixes)


MODULES = list(_modules())
ALGORITHMS = ("compas_timber_sequencing.core", "compas_timber_sequencing.ranking")
CORE = [(module, path, is_package) for module, path, is_package in MODULES if _is_within(module, ALGORITHMS)]
PURE_GEOMETRY = [(module, path, is_package) for module, path, is_package in MODULES if module == "compas_timber_sequencing.timber.collision"]


def test_every_module_belongs_to_a_layer():
    # A module outside the four folders would escape every rule below.
    unplaced = [module for module, _, _ in MODULES if module != PACKAGE and not _is_within(module, ALGORITHMS + MODEL_FACING)]
    assert unplaced == []


def test_the_layering_test_sees_the_package():
    names = [module for module, _, _ in MODULES]
    assert "compas_timber_sequencing.core.solver" in names
    assert "compas_timber_sequencing.ranking.strategies" in names
    assert "compas_timber_sequencing.timber.collision" in names
    assert "compas_timber_sequencing.kinematics.t_dovetail" in names
    assert len(CORE) >= 10
    assert len(PURE_GEOMETRY) == 1


@pytest.mark.parametrize("module, path, is_package", PURE_GEOMETRY, ids=[module for module, _, _ in PURE_GEOMETRY])
def test_collision_is_pure_geometry(module, path, is_package):
    offending = sorted(name for name in _imports(module, path, is_package) if _is_within(name, ("compas_timber",)))
    assert offending == [], "{} imports {}; it should need compas only".format(module, offending)


@pytest.mark.parametrize("module, path, is_package", CORE, ids=[module for module, _, _ in CORE])
def test_core_modules_do_not_import_compas_timber(module, path, is_package):
    offending = sorted(name for name in _imports(module, path, is_package) if _is_within(name, ("compas_timber",)))
    assert offending == [], "{} imports {}; only {} may".format(module, offending, ", ".join(MODEL_FACING))


@pytest.mark.parametrize("module, path, is_package", CORE, ids=[module for module, _, _ in CORE])
def test_core_modules_do_not_import_the_model_facing_layer(module, path, is_package):
    offending = sorted(name for name in _imports(module, path, is_package) if _is_within(name, MODEL_FACING))
    assert offending == [], "{} imports {}, which is the model-facing layer".format(module, offending)
