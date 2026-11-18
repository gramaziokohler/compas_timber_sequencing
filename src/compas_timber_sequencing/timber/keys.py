"""Stable element keys: how a beam is recognised across rebuilds of the model.

A ``guid`` is not stable: building the same beam twice -- as a Grasshopper definition does on every
recompute, or a script does on every run -- gives two different guids. Everything that has to
survive a rebuild (a plan, its overrides, a diff between two plans) refers to elements by key
instead.

The key of an element is

* its **name**, when the name is set, is not just the class's default (``"Beam"``), and is unique in
  the model -- the most readable choice, and the one that survives moving the beam;
* otherwise, for a beam, a **geometric key**: a short hash of its centerline end points (in either
  order) and its cross-section, rounded to ``tolerance``. It survives a rebuild of the same
  geometry, but moving the beam by more than ``tolerance`` makes it a different key;
* otherwise -- an element that is not sequenced -- its type and guid. Those keys only label excluded
  elements in reports and are not expected to be stable.

The search breaks ties between equally ranked elements by id, so using stable keys as ids also makes
the sequence of an unchanged design the same on every run.

"""

import hashlib
from collections import Counter

KEY_TOLERANCE = 1.0
"""float: Default rounding for geometric keys and fingerprints, in model units.

One model unit is a millimetre in most timber models. A value that sits on a rounding boundary can
still round differently after a tiny change; name the beams where that matters.
"""


def _grid(value, tolerance):
    return int(round(float(value) / tolerance))


def _ends(line, tolerance):
    start = tuple(_grid(c, tolerance) for c in line.start)
    end = tuple(_grid(c, tolerance) for c in line.end)
    return tuple(sorted([start, end]))


def _digest(token):
    return hashlib.sha1(repr(token).encode("utf-8")).hexdigest()[:10]


def geometry_token(beam, tolerance=KEY_TOLERANCE):
    """The rounded geometry a geometric key and a geometry fingerprint are made from.

    Parameters
    ----------
    beam : :class:`compas_timber.elements.Beam`
    tolerance : float, optional

    Returns
    -------
    tuple
        ``(end points, (width, height))``, with the end points in a fixed order so that a
        reversed centerline gives the same token.

    """
    return (_ends(beam.centerline, tolerance), (_grid(beam.width, tolerance), _grid(beam.height, tolerance)))


def geometric_key(beam, tolerance=KEY_TOLERANCE):
    """A key derived from a beam's centerline and cross-section.

    Parameters
    ----------
    beam : :class:`compas_timber.elements.Beam`
    tolerance : float, optional

    Returns
    -------
    str
        ``"beam-"`` followed by ten hex digits.

    """
    return "beam-" + _digest(geometry_token(beam, tolerance))


def _has_own_name(element):
    name = getattr(element, "name", None)
    return bool(name) and name != type(element).__name__


def element_keys(model, tolerance=KEY_TOLERANCE):
    """The key of every element of a model.

    Parameters
    ----------
    model : :class:`compas_timber.model.TimberModel`
    tolerance : float, optional
        Rounding for geometric keys, in model units.

    Returns
    -------
    dict
        ``str(element.guid)`` -> key, for every element in the model.

    Raises
    ------
    ValueError
        If two sequenced elements end up with the same key -- two unnamed beams with the same
        centerline and section within ``tolerance``. Name one of them.

    """
    elements = list(model.elements())
    names = Counter(element.name for element in elements if _has_own_name(element))

    keys = {}
    for element in elements:
        if _has_own_name(element) and names[element.name] == 1:
            key = element.name
        elif getattr(element, "is_beam", False):
            key = geometric_key(element, tolerance)
        else:
            key = "{}-{}".format(type(element).__name__.lower(), str(element.guid)[:8])
        keys[str(element.guid)] = key

    seen = {}
    duplicates = []
    for guid, key in keys.items():
        if key in seen:
            duplicates.append(key)
        seen[key] = guid
    if duplicates:
        raise ValueError(
            "These elements cannot be told apart: {}. Two unnamed beams share a centerline and section within the tolerance of {}; give one of them a name.".format(
                ", ".join(sorted(set(duplicates))), tolerance
            )
        )
    return keys
