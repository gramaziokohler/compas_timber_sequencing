"""Let the core be tested without compas_timber installed.

``compas_timber`` is the optional ``timber`` extra. Without it, the adapter module and the tests
that need a real timber model are not collected, instead of failing on import.
"""

import importlib.util

collect_ignore = []

if importlib.util.find_spec("compas_timber") is None:
    collect_ignore += [
        "src/compas_timber_sequencing/kinematics",
        "src/compas_timber_sequencing/timber.py",
        "tests/timber",
    ]
