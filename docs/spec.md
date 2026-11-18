# Assembly Sequencing via Kinematic Escape Constraints

How `compas_timber_sequencing` computes a robotic assembly order, a per-element insertion
vector, and an explicit set of elements that must be placed by hand, for a `compas_timber`
model.

The algorithms live in `compas_timber_sequencing`; the per-joint input is
`compas_timber_sequencing.kinematics.kinematic_constraint`, and the model adapter is
`compas_timber_sequencing.timber`. `compas_timber` itself is unchanged.

## At a glance

```
TimberModel ──► timber.py (adapter) ──► SequencingInput ──► search.generate() ──► SequenceResult ──► element.attributes
                    │                     (plain data +                │
                    └── kinematics/ ◄──── constraints callback)       ├── blocking.py    locked in every order? (once, up front)
                        (per-joint freedom)                           ├── solver.py      can this element move right now?
                                                                      └── preferences.py which movable one goes first?
```

1. The adapter reads the model (§10) and flattens it into a `SequencingInput`: ids, the joint
   graph, heights and lengths, plus two callbacks — the constraints on an element given what
   is still in place (§3), and a collision check along a direction (§4.4).
2. The blocking graph finds the elements that are locked in every order (§5). They are placed
   by hand.
3. A beam search takes the model apart one element at a time (§6). At each step every remaining
   element is solved (§4), the movable ones are ranked by a strategy (§7), and the best few
   partial orders are carried forward.
4. The disassembly order is reversed and the extraction vectors negated, and the result is
   written into each element's `attributes` (§11).

---

## 1. Purpose and consumers

Forward-planning an assembly sequence runs into kinematic dead ends, so the algorithm works
backwards: **disassembly planning**. It repeatedly pulls an element clear of whatever is
still in place, then reverses the order and negates the vectors.

Three consumers, in priority order:

1. **Robotic assembly** — `compas_fab` consumes the unit insertion vector. The approach
   distance is fixed, short, and decided downstream.
2. **Visualization** — Grasshopper display of order and direction.
3. **Human fabrication** — the hand-placement set is a real fabrication instruction.

The tool must work acceptably on all designs and let the user override both the order and
the hand-placement set. Every design has exceptions; the job is to get close and make the
exceptions easy to express, not to be right unaided.

---

## 2. Where the code lives

```
compas_timber_sequencing/src/compas_timber_sequencing/
  kinematics/     kinematic_constraint(joint, element): per-joint freedom, dispatched on the
                  joint's class (butt, lap, miter, mortise_tenon, step, scarf)
  boundary.py     SequencingInput — everything the algorithms may touch
  constraints.py  HalfSpace, SignedAxis, validation
  solver.py       pure: solve(constraints) -> Solution | Locked
  blocking.py     blocking graph, SCC, intrinsic locks, articulation points
  search.py       beam search over disassembly states, and generate()
  preferences.py  injectable ranking strategies; gravity is one of them
  compare.py      side-by-side scoring of strategies
  result.py       SequenceResult, StuckReport, StalenessReport
  trace.py        opt-in tracing
  timber.py       TimberModel -> SequencingInput adapter
```

The dependency runs one way: this package uses `compas_timber`, `compas_timber` never imports
this package. Everything except `kinematics/` and `timber.py` needs `compas` alone, and
`compas_timber >=3.0,<4` is the optional `timber` extra.

Dependencies are `compas.geometry` and the standard library — no numpy, no scipy. Python
3.9 syntax, because it must run under Rhino's CPython.

Both constraint *computation* (`kinematics/`) and *consumption* (everything else) live in this
package. The computation reads the joint's planes, reference sides and parameters from
outside `compas_timber`; §10 lists exactly what it touches.

---

## 3. The joint interface

`kinematic_constraint(joint, moving_element, assembled_elements=None)` returns the freedom
to pull `moving_element` out, assuming every other member of that joint is frozen. It is a
`functools.singledispatch` function: implementations are registered per joint class and found
along the class hierarchy, like a method. They read the joint's planes, reference sides and
parameters from outside `compas_timber`; two of those reads are private methods
(`ISimpleScarf._get_beam_side`, `TBirdsmouthJoint._get_cutting_planes`), which is why the nightly
CI job tests against `compas_timber`'s main branch.

`assembled_elements` matters only for joints with more than two members (the Y-butt), whose
constraint on one member depends on which others are still in place. It defaults to all of them.

| Return | Meaning |
|---|---|
| `Vector` | Inward normal of a permitted half-space: `n · d >= 0`. Butt joints, laps. |
| `list[Vector]` | Several half-spaces; the freedom is their intersection. |
| `Line` | A strict 1-DOF sliding axis. Mortise-tenon, dovetail, housed seat cut. |

What each registered joint returns, read off the implementations:

| Joint (module) | Returns |
|---|---|
| `ButtJoint`, `TButtJoint` (`butt`) | The butt-plane normal pointing away from the other beam; with `mill_depth`, also the two pocket walls. |
| `LButtJoint` (`butt`) | The butt-plane normal; with `mill_depth`, also the one pocket wall the beam bears against. |
| `YButtJoint` (`butt`) | Depends on which members are present (`assembled_elements`): ref-side normals of the present cross beams for the main beam, plus the beam direction with `mill_depth`; for a cross beam, the main beam's face and the miter plane of the other cross beam. An empty list when nothing is present. |
| `LLapJoint` (`lap`) | Three half-spaces: the two lap cutting planes, oriented for the moving beam, and the lap face. |
| `TLapJoint` (`lap`) | Four half-spaces: the cross beam's incident face and three faces of the main beam's lap, negated for the cross beam. |
| `XLapJoint` (`lap`) | A `Line` along the intersection of the two cutting planes; opposite directions for the two beams. |
| `LFrenchRidgeLapJoint` (`lap`) | Three half-spaces: the two lap cutting planes and the inclined ridge face. |
| `LMiterJoint` (`miter`) | One half-space: the other beam's cutting-plane normal. |
| `MortiseTenonJoint` (`mortise_tenon`) — and so `TTenonMortiseJoint`, `LTenonMortiseJoint`, `TOliGinaJoint` | A `Line` along the cross beam's reference-side normal: `+axis` for the main beam, `-axis` for the cross beam. |
| `TDovetailJoint` (`mortise_tenon`) | A `Line` along the dovetail groove (with `rotation` applied); opposite directions for the two beams. |
| `TStepJoint` (`step`) | Two to four half-spaces depending on `step_shape` (step, heel, tapered heel, double); with `tenon_mortise_height`, two opposed half-spaces that leave a slot. Negated for the cross beam. |
| `TBirdsmouthJoint` (`step`) | With `mill_depth`, a `Line` along the main beam (a housed seat); otherwise the two notch faces, negated for the cross beam. |
| `ISimpleScarf` (`scarf`) | Two half-spaces: the inclined scarf face and the beam's end direction. |
| anything else | The fallback, §3.3. |

### 3.1 The `Line` is signed

This is the most surprising part of the protocol, and it was previously written down
nowhere.

A `Line` means **`start` → `end` is the only permitted extraction direction.** Its reverse
pushes deeper into the joint.

- The solver tests `direction` and never `-direction`. If the one permitted direction
  pushes through material, the element is locked.
- Two anti-parallel axes are a genuine deadlock, not a sign mistake. `abs()` in that test
  would be a bug.
- Two axes that are merely *near*-parallel are also a lock: two exact 1-DOF constraints
  with nothing to give really are over-constrained. The threshold is `PARALLEL_TOL`, about
  0.26°.
- The `Line`'s origin is carried through to `SignedAxis.origin` for callers that want to
  draw it. **The solver never reads it.**

Worked example — `TTenonMortiseJoint`: `main_beam` gets `+axis`, `cross_beam` gets `-axis`.

### 3.2 Unknown types raise

Anything that is not a `Line`, a `Vector`, or a list of those raises a `TypeError` in the
adapter. Dropping an unrecognised constraint makes an element look *more* free than it is,
which is the dangerous direction to fail in.

### 3.3 The permissive base implementation

A joint with no registered implementation gets a single half-space pointing from the static
element's centerline midpoint to the moving element's (or, when the two midpoints coincide, the
axis perpendicular to both members). `is_fallback(joint)` tells the two apart. That almost always yields
a valid extraction, and a large fraction of the joint library still relies on it.

Constraints from the fallback are tagged **inferred**, counted per element as
`Solution.inferred_count`, and totalled as `SequenceResult.inferred_total`. A result that
depended on guesses says so. Once joint coverage is good, unimplemented joints should
return a locked constraint instead — loud and honest.

---

## 4. The solver

Pure geometry: `solve(constraints, path_check=None) -> Solution | Locked`. No model, no
joint classes, no Rhino. That is what makes it testable against hand-built constraint sets.

### 4.1 Three extraction states

| State | Condition | Meaning |
|---|---|---|
| **Roomy** | `margin >= ROOMY_MARGIN` | Comfortable clearance, safe for robotic extraction |
| **Tight** | `-TOL <= margin < ROOMY_MARGIN` | A real but zero-clearance fit — a slot. Feasible, risky |
| **Locked** | `margin < -TOL` | No direction satisfies all constraints |

`margin` is `min_i (n_i · d)`, the sine of the angle by which the chosen direction clears
its nearest constraint boundary. **Margin classifies; it never rejects.** A beam dropped
into a slot between two others has a margin of exactly zero, and that is a correct answer
about a genuinely tight fit. Report `Solution.angle_degrees`: "3.2°" tells a fabricator
something, "tight" does not.

A strict 1-DOF fit is a slot by definition, so it is never roomy however generous the
surrounding half-spaces are.

`None` is not part of the vocabulary. An **empty** constraint list means the element is
entirely free, not locked.

### 4.2 Cone feasibility

```
maximise   min_i (n_i · d)
subject to |d| = 1
```

**Argmax over the candidate set, not first hit.** The candidates are the extreme rays of
the polyhedral cone: each `n_i`, both signs of each normalized pairwise cross `n_i × n_j`,
and the normalized sum of all normals as an interior seed. Crosses against world X/Y/Z are
not included; they are noise.

One degenerate case is handled explicitly. When two normals are anti-parallel their cross
vanishes and the feasible set collapses into the plane they share — a linear subspace with
no extreme rays. An orthonormal basis of that plane is added, so a beam housed between two
parallel faces is reported as the tight fit it is rather than as a lock.

Exact ties in margin are settled by alignment with the interior seed, then world up, then
rounded components. Output is therefore deterministic and independent of the order the
constraints arrived in.

### 4.3 Named constants

| Name | Value | Meaning |
|---|---|---|
| `ROOMY_MARGIN` | `sin(5°) ≈ 0.087` | Boundary between roomy and tight |
| `PARALLEL_TOL` | `1e-5` | 1-DOF parallelism, as `dot > 1 - PARALLEL_TOL` (~0.26°) |
| `TOL` | `1e-9` | Numeric zero |
| `APPROACH_DISTANCE` | `100.0` | Swept-check distance, model units |

### 4.4 Swept broad-phase

Constraints derived from joints only see *jointed* neighbours. In a dense lattice a beam
can be kinematically free per its joints while its extraction path passes straight through
a beam it is not jointed to — and the solver would report a clean vector with a comfortable
margin.

After the cone solve, `path_is_clear(element, d, APPROACH_DISTANCE, active_ids)` sweeps the
element against everything still in place. Candidates are offered in descending order of
margin, and the first one whose path is clear wins; only when every feasible direction is
obstructed is the element locked. Without geometry (`use_geometry=False`) the check is skipped
and the joints alone decide.

The adapter's implementation (`TimberModelAdapter.path_is_clear`):

- **Who is tested.** Every element still in place except the moving element's jointed
  neighbours, whose interaction the joint constraints already describe exactly — a beam
  sliding along a joint face would otherwise collide with it by definition. Excluded elements
  (anything that is not a beam) are always tested, as obstacles.
- **What is swept.** The element's oriented bounding box (`element.obb`), moved from 5% of the
  approach distance to the full distance along `d`. Starting a little way out keeps elements
  already touching at the start from vetoing every direction.
- **Broad phase.** Axis-aligned bounds of the swept box against each candidate's bounds, which
  discards everything that is nowhere near.
- **Narrow phase.** An exact separating-axis test between the swept oriented box and the other
  element's oriented box: the two boxes' face normals, the cross products of their edges, and
  the cross products of the sweep direction with the other box's edges. This is not an
  optimization: bounds around a diagonal brace are mostly empty, and a broad phase alone
  reports beams obstructed by empty space.
- **Already in contact.** Unjointed beams can overlap at rest (in a reciprocal frame, many lap
  and bear without a recorded joint). For such a pair only a direction that drives the element
  *deeper* into the other counts as obstructed; a direction that keeps or reduces the overlap
  is a way out.
- **When geometry is missing.** If the moving element has no box, its path counts as clear; if
  another element has none, a broad-phase hit on it counts as an obstruction.

What remains approximate is the element, not the test: each element is its oriented bounding
box, solid to its full section with processings ignored. That errs towards "obstructed".

---

## 5. Intrinsic vs. order-dependent locks

These are not the same failure and never share an output flag.

- **Intrinsic** — locked in every order. Detected up front and reported before sequencing
  starts. This is the only thing that legitimately produces a hand-placement flag.
- **Order-dependent** — locked given what is still in place, extractable under a different
  order. This is a failure of the sequence, and the search routes around it. It must never
  reach a shop floor as "place by hand".

### 5.1 The blocking graph

`A -> B` means "B must be removed before A, whatever the order". The edge test is
**pairwise**: an edge exists when the joint between A and B locks A *on its own*, with
everything else already gone. That makes it an unconditional precedence.

The tempting stronger rule — draw an edge to every neighbour that participates in locking A
in the complete assembly — is unsound. Blocking is disjunctive: an element freed by removing
*either* B *or* C would get edges to both, turning an OR into an AND and inventing cycles
where a good order exists.

Strongly connected components of size > 1 are mutually blocking clusters: **intrinsic
locks**. A rafter housed on both sides of a wall plate (a beam) is the canonical case — cut the model down
to just those two and they are still stuck.

This is a reduced form of Wilson & Latombe's non-directional blocking graph (~1994). The
full sphere partition is not implemented; the SCC pass gives what is needed for a fraction
of the work, and is the road if subassembly-level sequencing is wanted later.

**Sound but not complete.** Every cluster reported really is locked in every order. An
interlock that only closes across three or more elements at once — each pair separable, the
trio not — has no pairwise edge to find and is not reported. Those surface as a
`StuckReport` from the search, which is the honest outcome: no order was *found*, rather
than a claim that none exists.

### 5.2 Subassemblies

User-declared model groups win. Otherwise the SCCs supply real, geometry-derived groups, and
an element in no cluster is its own subassembly. Labels derive from element ids, never
iteration order, so two runs on identical input give identical labels.

This replaces label propagation, which broke ties on `(count, guid)` — and since guids are
random, subassembly labels differed between runs on identical input, making the whole
ranking non-deterministic. Non-deterministic output on a fabrication tool is a correctness
defect.

### 5.3 N-ary joints

A member's constraint depends on which *other* members of its joint are still present, so
`SequencingInput.constraints` is a function of `(element_id, active_neighbor_ids)`, not a
per-joint lookup table. A two-element joint is the degenerate case. Ball-node and multibeam
joints contribute precedence like any other.

How far the timber adapter uses this today: it drops a joint once every other member is gone,
but while at least one remains it calls `kinematic_constraint(joint, element)` without
`assembled_elements`, so the Y-butt is evaluated as if all of its members were still in place.
That is the conservative reading — it can only report less freedom than there is.

---

## 6. Search

**Beam search**, width 5 by default, over disassembly states, memoized on
`frozenset(remaining)`. Reversed at the end for assembly; vectors negated for insertion.

Chosen over memoized DFS because the goal is *a good* sequence, not merely *a* sequence.
Beam search is deterministic, degrades gracefully instead of falling off a node-budget
cliff, has no "best partial found" ambiguity, and makes alternative sequences nearly free.

Before the search, the blocking graph (§5) supplies the intrinsic locks — added to the manual
set — and the subassembly labels.

Per step, for each partial disassembly carried forward:

1. **Ground and stability.** The grounded elements are those whose base is within
   `support_tolerance` (by default a thousandth of the model's height) of the lowest base still
   in place. One articulation-point pass then finds the elements whose removal would leave part
   of the remaining structure with no grounded member (`disconnecting_elements`).
2. **Feasibility.** Every remaining element is solved against the elements still in place.
   Results are cached on `(element, frozenset(remaining))`, so the same state is never solved
   twice across partials. Elements with a `Solution`, and manual-set members, are candidates;
   the rest are blockers, each with its reason.
3. **Pins.** An element pinned to another step is not a candidate here; at its own step it is
   the only candidate.
4. **Ranking.** The strategy (§7) ranks the candidates; the top *k* extensions of this partial
   are kept.

Then all extensions are pooled, partials that leave the same set of elements standing are
merged (keeping the better score), and the *k* best go forward.

**Hard constraints filter; soft preferences rank.** They are separate mechanisms, so a
constraint violation cannot be outvoted by an ordering accident.

**Pins win.** A pin claims one assembly position and forbids every other. If a pinned
position is infeasible, a `PinConflict` is returned and sequencing stops. A user's
fabrication plan is never silently reordered.

**Dead ends** produce a `StuckReport`: step index, remaining elements, and the blocking
reason per element. A structured result, not a log line into a void that Grasshopper does
not display.

---

## 7. Ranking

The preference function is injectable; `GravityStrategy` is the default, not the only thing
the package can express. Scores are stated in **disassembly** terms: higher means removed
earlier, which means placed *later*. `GravityStrategy` compares, most significant first:

1. **Does not strand anything from the ground** (§6, step 1). What it measures is
   connectivity to a lowest-level element — a topological proxy, not a stability calculation.
2. **Nothing jointed to it is still standing higher up** (`SequencingInput.above`: a jointed
   neighbour whose base sits more than `support_tolerance` higher). This keeps the search
   from reaching past a blocked high element and pulling a post out from under a beam still
   standing on it. A preference, not a filter: when every candidate carries something, the
   heights below decide.
3. **`base_z`, then `centroid_z`**, both descending, as two separate lexicographic terms.
   Both carry real signal: centroid alone ranks a ground-standing post as "high", base alone
   cannot separate two beams starting at the same level. *Summing* them produces a quantity
   with no physical meaning and no nameable unit.
4. **Roomy before tight.**
5. **Same subassembly as the element removed last.**
6. **Jointed to the element removed last.**
7. **Longer first**, then **fewer joints first**.

Element ids break exact ties, so the ranking never depends on dict iteration order.

### 7.1 The other strategies

Registered in `STRATEGIES` and built with `make_strategy(name)`. All but `gravity` and `random`
are `TermStrategy`s: a list of named terms from `TERMS`, compared lexicographically, each
normalized to 0..1 over the model, a `-` prefix inverting it.

| Name | Terms, most significant first | Assembly consequence |
|---|---|---|
| `gravity` | as above | Bottom-up. The default. |
| `layered` | `stable`, height band, then `subassembly`, `chain`, `centroid_z`, `roomy`, `length`, `-degree` | Course by course: height is quantized into bands (`tolerance` in model units, or the height split into `bands`, default 4), and each course is finished before the next. |
| `clearance` | `stable`, `roomy`, `margin`, `base_z`, `centroid_z`, `subassembly`, `chain`, `-degree` | Tight fits are placed first, into the emptiest site. |
| `skeleton` | `stable`, `-degree`, `-length`, `base_z`, `roomy`, `subassembly`, `chain` | Long, well-connected members first; infill last. |
| `subassembly` | `stable`, `subassembly`, `chain`, `base_z`, `centroid_z`, `roomy`, `length`, `-degree` | One cluster finished before the next is started. |
| `chain` | `stable`, `locality`, `freed`, `base_z`, `roomy`, `length`, `-degree` | Stay near the last element; minimize travel. |
| `random` | `stable`, then a `crc32` hash of the seeded element id | A deterministic shuffle, as a control: a strategy that does not beat it is not earning its place. |

For custom orders: `TermStrategy(["stable", "base_z", "-degree"])`, `WeightedStrategy` (a weighted
sum of terms, for tuning), or `HeuristicStrategy` (any scalar function of the element id).

`clearance` ranks on the raw `margin`, so near-ties between equal margins are sensitive to the
last bit of the joint geometry; see §9.

### 7.2 Comparing strategies

There is no strategy that is best everywhere, so `compare_strategies(input, names)` runs
`generate` once per strategy and `describe_comparison` tabulates the results without collapsing
them into one score: whether it completed, tight placements, mean and minimum clearance angle,
hand-placed count, jumps between subassemblies, the fraction of placements touching something
already placed, height inversions, and **support inversions** — jointed pairs where the higher
element is placed before the lower one, the measurement to read first.
`scripts/compare_strategies.py` runs this on a saved model.

**Beams only, stated out loud.** Every element that is not a beam is excluded from
sequencing and listed in `SequenceResult.excluded`. Giving them a height of
zero sinks them to the bottom of the ranking while looking like a real measurement.

Deliberately absent: any rule ordering `cross_beam` before `main_beam`. "Main" and "cross"
are naming conventions about which beam got cut, not statements about assembly order, and
such a rule would override the geometry the solver computes.

---

## 8. Overrides

`generate(input, manual_set, pinned_order)` is idempotent and re-runnable.

1. The solver proposes a manual set: intrinsic locks, plus tight fits as candidates
   (`SequenceResult.proposed_manual_set`).
2. The user amends it.
3. Re-run with the amended set as a given.

Elements in the manual set are exempt from the feasibility filter — a human can rotate, tilt
and spring a member into place, so robot kinematics do not apply to them.

**Persistence.** The manual set is a fact about the design, so it lives in the element
attribute `requires_manual_assembly` and is serialized with the model; the adapter reads it
back on the next run. Pinned order is a fact about a particular build and is supplied per
run by the caller.

**Staleness.** When the model changes under an override — a pinned element deleted, a
position out of range, two pins on one position, an override on an excluded element — a
`StalenessReport` lists every override that no longer applies. A fabrication instruction
someone typed is never dropped without saying so.

---

## 9. Known limitations, accepted

- **Cone feasibility is infinitesimal freedom.** `min_i(n_i · d) > 0` says the element can
  *begin* to move, not that it can travel far enough to clear the assembly. The swept check
  mitigates this over the fixed approach distance; it does not eliminate the gap for deep
  mortises, scarfs, or long engagements.
- **No clearance distance.** Maximum travel before collision is expensive and the approach
  distance is fixed and short downstream. Deliberately not computed.
- **The robot is not a point.** Gripper and arm collision with already-placed elements is
  `compas_fab`'s domain. A sequence certified here can still be unbuildable for reach
  reasons, and users will experience that as this tool being wrong.
- **Monotone sequences only.** No assemble → disassemble → reassemble.
- **Intrinsic-lock detection is sound but not complete** (§5.1).
- **Elements are their bounding boxes in the swept check** (§4.4): solid to the full section,
  processings ignored.
- **N-ary joints are read with all members present** while any one remains (§5.3).
- **The `clearance` strategy is sensitive to the last bit of the geometry** (§7.1). Moving
  from `compas_timber` 2.x to 3.0 changed the constraint normals by at most 2.2e-16 and flipped
  near-ties on one test model; every other input was bit-identical.
- **A stuck run writes misleading indices** (§11).

---

## 10. Working with compas_timber

The dependency runs one way: this package reads `compas_timber`, and `compas_timber` never
imports it. `compas_timber` needs no changes to be sequenced.

**What is read.**

| From | What | Where |
|---|---|---|
| `TimberModel` | `elements()`, `joints`, `get_joints_for_element(element)` | adapter |
| every element | `guid`, `is_beam`, `is_group_element`, `children` (groups), `attributes` | adapter |
| beams | `centerline` (base and centroid height, length), `obb` (swept check) | adapter |
| every joint | `elements`, `element_a`, `element_b`, `name` | adapter, fallback |
| each joint type | its own attributes: `main_beam`, `cross_beam`, `beam_a`/`beam_b`, `location`, `butt_plane`, `cutting_plane_a`/`cutting_plane_b`, `cutting_planes`, `get_miter_planes()`, the ref-side indices, `mill_depth`, `step_shape`, `step_depth`, `heel_depth`, `tenon_mortise_height`, `rotation`, `length`, `depth_ref_side`/`depth_opp_side`, … | `kinematics/` |
| two joint types, privately | `ISimpleScarf._get_beam_side()`, `TBirdsmouthJoint._get_cutting_planes()` | `kinematics/` |
| helpers | `compas_timber.fabrication.StepShapeType`, `compas_timber.connections.utilities.beam_ref_side_incidence` | `kinematics/` |

**What is written.** Only `element.attributes`, and only when `apply_to_model=True` (§11).

**How the coupling is contained.** None of the joint attributes above is a documented API, and
two are private. The `timber` extra pins `compas_timber >=3.0,<4`, so a major release cannot
break this silently, and `.github/workflows/nightly.yml` runs the whole suite against
`compas_timber`'s main branch every night, so a rename shows up within a day.

---

## 11. Usage

```python
from compas_timber_sequencing.timber import assembly_order
from compas_timber_sequencing.timber import generate_assembly_sequence

result, adapter = generate_assembly_sequence(model)

if not result.is_complete:
    print(result.stuck.describe())

for index, element_id in enumerate(result.order):
    beam = adapter.elements_by_id[element_id]
    vector = result.insertion_vectors[element_id]   # None => place by hand
    state = result.state(element_id)                # roomy | tight | locked
```

`generate_assembly_sequence(model, ...)` takes:

| Argument | Default | Meaning |
|---|---|---|
| `manual_set` | read from the elements | Element ids (`str(element.guid)`) to place by hand. |
| `pinned_order` | none | Element id → assembly index, or a list of ids in assembly order. |
| `strategy` | `GravityStrategy()` | Any strategy from §7, e.g. `make_strategy("clearance")`. |
| `width` | 5 | How many partial orders the search carries forward. |
| `distance` | 100.0 | Swept-check approach distance, in model units. |
| `use_geometry` | `True` | Run the swept check; `False` trusts the joints alone. |
| `apply_to_model` | `True` | Write the result into the element attributes. |

It returns `(result, adapter)`; `adapter.elements_by_id` turns ids back into elements and
`adapter.excluded` says what was left out and why.

**What is written.** For each element in `result.order`: `assembly_sequence` (its index),
`insertion_vector` (a unit `Vector`, or `None` when hand-placed), `requires_manual_assembly`
and `extraction_state` (`roomy`, `tight` or `locked`), all in `element.attributes`, which
`compas_timber` serializes with the element. `assembly_order(model)` reads them back as the
elements in assembly order. On the next run `requires_manual_assembly` is read back as the
manual set, so an amended set survives; pins are never stored.

**After a stuck run**, `result.order` holds only the elements that were removed before the
search stopped — the *last* ones to be assembled — and they are written with indices from 0.
Elements that were not reached keep whatever an earlier run wrote. Check `result.is_complete`
before relying on the attributes.

**Tracing.** The library never prints. `set_tracer(True)` prints each search step;
`set_tracer(lines.append)` collects them into a list, which is the reliable way to get them into
a Grasshopper panel; `set_tracer(None)` turns it off.

The original `KinematicSequenceGenerator` / `InsertionSolver` were not carried over; they
remain on the archived `compas_timber` branch (tag `archive/joint-kinematics`).

To sequence something that is not a timber model, build a
`compas_timber_sequencing.SequencingInput` directly and call
`compas_timber_sequencing.generate`; see `tests/core/synthetic.py`.
