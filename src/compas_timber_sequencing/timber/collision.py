"""Swept-box collision: can an element slide a short distance without hitting another one?

Pure geometry on :class:`compas.geometry.Box` -- no model, no ``compas_timber``. The timber adapter
decides *which* elements to test against (everything still in place that is not a jointed
neighbour) and calls :func:`swept_box_hits_box` for each, after the cheap axis-aligned rejection in
:func:`swept_aabb` / :func:`bounds_overlap`.

Two phases, because the broad phase alone is wrong in the way that matters: axis-aligned bounds
around a diagonal member are mostly empty space, and a bounds-only test reports beams obstructed by
nothing.

* **Broad phase** -- :func:`swept_aabb` and :func:`bounds_overlap`.
* **Narrow phase** -- :func:`swept_box_hits_box`, an exact separating-axis test between the swept
  oriented box and a static oriented box, which also handles boxes that already overlap at rest.

What stays approximate is the element, not the test: an element is its oriented bounding box.

"""

SWEEP_START_FRACTION = 0.05


_DEGENERATE_AXIS = 1e-6
"""float: Below this a candidate separating axis from a cross product is discarded.

The cross of two unit vectors has the magnitude of the sine of the angle between them, so
this is about two thousandths of a degree.
"""


def swept_aabb(box, direction, distance):
    """Axis-aligned bounds of an oriented box swept along `direction`.

    The sweep starts a little way along the direction rather than at the box itself, so
    that elements already in contact at the start position do not veto every extraction.

    """
    corners = box.points
    near = distance * SWEEP_START_FRACTION
    xs = []
    ys = []
    zs = []
    for scale in (near, distance):
        for corner in corners:
            xs.append(corner[0] + direction.x * scale)
            ys.append(corner[1] + direction.y * scale)
            zs.append(corner[2] + direction.z * scale)
    return (min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))


def box_bounds(box):
    corners = box.points
    xs = [corner[0] for corner in corners]
    ys = [corner[1] for corner in corners]
    zs = [corner[2] for corner in corners]
    return (min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))


def bounds_overlap(a, b, tolerance=1e-6):
    return (
        a[0] <= b[3] - tolerance and b[0] <= a[3] - tolerance and a[1] <= b[4] - tolerance and b[1] <= a[4] - tolerance and a[2] <= b[5] - tolerance and b[2] <= a[5] - tolerance
    )


def _box_frame(box):
    """An oriented box as ``(centre, [axis, ...], [half_extent, ...])`` in world space."""
    frame = box.frame
    centre = (frame.point.x, frame.point.y, frame.point.z)
    axes = [(frame.xaxis.x, frame.xaxis.y, frame.xaxis.z), (frame.yaxis.x, frame.yaxis.y, frame.yaxis.z), (frame.zaxis.x, frame.zaxis.y, frame.zaxis.z)]
    extents = [box.xsize / 2.0, box.ysize / 2.0, box.zsize / 2.0]
    return centre, axes, extents


def _norm(vector):
    return (vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2) ** 0.5


def _unit(vector):
    length = _norm(vector)
    if length < 1e-12:
        return (0.0, 0.0, 0.0)
    return (vector[0] / length, vector[1] / length, vector[2] / length)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _radius(axes, extents, normal):
    """Half the width of an oriented box's shadow on `normal`."""
    return sum(abs(_dot(axis, normal)) * extent for axis, extent in zip(axes, extents))


def _separating_axes(axes_a, axes_b, sweep):
    """The finite axis set that decides overlap between a swept box and a static box.

    The two boxes' face normals plus the cross products of their edge directions, which is
    the standard box-versus-box set, extended with the cross products of the sweep
    direction against the stationary box's edges -- the axes the sweep adds, since the
    swept solid's extra edges all run along it.

    Near-parallel edges give a cross product that is all rounding error; those are dropped.
    Dropping an axis is the safe direction, because one axis fewer can only make the test
    report an overlap, and the face normals already cover what it stood for.

    """
    axes = list(axes_a) + list(axes_b)
    for axis_a in axes_a:
        for axis_b in axes_b:
            axes.append(_cross(axis_a, axis_b))
    for axis_b in axes_b:
        axes.append(_cross(sweep, axis_b))
    return [_unit(axis) for axis in axes if _norm(axis) >= _DEGENERATE_AXIS]


def _separation(offset, axes, axes_a, extents_a, axes_b, extents_b):
    """How far apart two oriented boxes are, along their worst-case separating axis.

    Positive is a gap, negative is interpenetration and its magnitude is the depth. This is
    the separating axis theorem read as a quantity rather than a yes or no: the boxes are
    disjoint exactly when some axis reports a gap, so the largest value over the axis set
    is the answer.

    Parameters
    ----------
    offset : tuple
        The second box's centre relative to the first's.
    axes : list
        The candidate separating axes, already unitized, from :func:`_separating_axes`.
    axes_a, extents_a, axes_b, extents_b : list
        The two boxes' frames, from :func:`_box_frame`.

    Returns
    -------
    float

    """
    worst = None
    for normal in axes:
        gap = abs(_dot(offset, normal)) - _radius(axes_a, extents_a, normal) - _radius(axes_b, extents_b, normal)
        if worst is None or gap > worst:
            worst = gap
    return 0.0 if worst is None else worst


def swept_box_hits_box(moving, direction, near, far, other, tolerance=1e-6):
    """Whether sweeping an oriented box along `direction` runs it into another oriented box.

    Two questions, because the boxes may already be touching before anything moves.

    **Already in contact.** Then the contact is a fact about the assembled model, not about
    the path, and the only thing worth asking is whether this direction makes it worse. The
    box is swept to `far` and the interpenetration is measured again: deeper means the
    direction drives the element further into something it is already up against, and that
    is an obstruction; the same or shallower means the direction is a way out, and vetoing
    it would have the element veto itself.

    That case is not an edge case in timber. Beams in a reciprocal frame lap and bear on
    one another, and the model records a joint only for the pairs that were explicitly
    connected -- in the frame this was written against, 52 unjointed pairs of beams
    overlapped at rest by a median of 39 mm on a 100 x 140 section. Testing those pairs as
    if the element started clear rejects *every* direction for *every* one of them, and the
    search stops with most of the model still standing. This is the same reasoning that
    already skips jointed neighbours, applied to the contact a joint does not happen to
    describe.

    **Starting clear.** Then the swept solid is the convex hull of the box at `near` and the
    same box at `far`, which is convex, so the separating axis theorem is exact: the two are
    disjoint if and only if one axis in :func:`_separating_axes` separates their
    projections.

    Parameters
    ----------
    moving : :class:`compas.geometry.Box`
    direction : :class:`compas.geometry.Vector`
        Unit sweep direction.
    near : float
        Where the sweep starts, as a distance from the box's own position.
    far : float
        Where the sweep ends.
    other : :class:`compas.geometry.Box`
    tolerance : float, optional
        A gap narrower than this counts as contact rather than clearance, and a change in
        depth smaller than this counts as sliding rather than as driving in deeper.

    Returns
    -------
    bool

    """
    centre_a, axes_a, extents_a = _box_frame(moving)
    centre_b, axes_b, extents_b = _box_frame(other)

    sweep = (direction.x, direction.y, direction.z)
    offset = tuple(centre_b[axis] - centre_a[axis] for axis in range(3))
    axes = _separating_axes(axes_a, axes_b, sweep)

    at_rest = _separation(offset, axes, axes_a, extents_a, axes_b, extents_b)
    if at_rest < 0.0:
        # Moving the box by `far` moves the other box by `-far` in its frame.
        shifted = tuple(offset[axis] - sweep[axis] * far for axis in range(3))
        at_end = _separation(shifted, axes, axes_a, extents_a, axes_b, extents_b)
        return at_end < at_rest - tolerance

    for normal in axes:
        # B's centre, relative to the swept solid's, measured along the axis.
        delta = _dot(offset, normal)
        travel = _dot(sweep, normal)
        reach_low = min(near * travel, far * travel)
        reach_high = max(near * travel, far * travel)
        span = _radius(axes_a, extents_a, normal) + _radius(axes_b, extents_b, normal)

        if max(delta - reach_high, reach_low - delta) > span - tolerance:
            return False

    return True
