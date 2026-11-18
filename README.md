# compas_timber_sequencing

[![build](https://github.com/gramaziokohler/compas_timber_sequencing/actions/workflows/build.yml/badge.svg)](https://github.com/gramaziokohler/compas_timber_sequencing/actions/workflows/build.yml)

Assembly sequencing for [COMPAS Timber](https://github.com/gramaziokohler/compas_timber) models.

Given a timber model, it produces:

* an **assembly order**,
* a unit **insertion vector** per element, for robotic trajectory planning (`compas_fab`), and
* an explicit **hand-placement set**: the elements no robot can insert, which is a real fabrication instruction.

It works backwards. It takes the finished model apart one element at a time, only removing an
element when the joints still holding it let it move, then reverses that order to get the assembly
sequence. What each joint allows is worked out in this package, in
`compas_timber_sequencing.kinematics`; `compas_timber` itself needs no changes. A beam search then
picks a good order among the feasible ones, ranked by an exchangeable strategy (bottom-up gravity by
default).

> **Status: alpha.** Works with released `compas_timber` 3.x.

## Install

```bash
pip install "compas_timber_sequencing @ git+https://github.com/gramaziokohler/compas_timber_sequencing.git"
```

Requires Python 3.9 or later (Rhino 8's CPython included), `compas >=2.15.1,<3` and
`compas_timber >=3.0,<4`, which are installed with it.

The algorithms themselves (`core/` and `ranking/`) do not use `compas_timber`. To sequence something
that is not a timber model, build a `SequencingInput` yourself and call `generate()`; see
`tests/synthetic.py` for examples.

## Use

Sequencing is a loop: plan, look, adjust, plan again. The model is often rebuilt in between -- by a
Grasshopper recompute or a new script run -- so plans and adjustments refer to beams by **key**: the
beam's name where it has a unique one, otherwise a fingerprint of its centerline and section. Name your
beams if you can; it makes everything below readable and lets a beam keep its key when it moves.

```python
from compas_timber_sequencing import SequenceOverrides, apply_plan, diff, plan_sequence

plan = plan_sequence(model)  # model: a compas_timber TimberModel; it is not changed
print(plan.describe())  # position, key, insertion vector, fit (roomy | tight | locked)

# Adjust by stating intent, not positions:
overrides = SequenceOverrides()
overrides.keep_order(plan, until=2)  # keep the first two steps in their planned order
overrides.place_before("strut", "bottom rail")  # but the strut goes in before the bottom rail
overrides.place_by_hand("brace")  # placed by a person, not the robot

adjusted = plan_sequence(model, overrides)
print(diff(plan, adjusted).describe())  # what moved, what turned, what got tighter

adjusted.to_json("plan.json")  # plans and overrides save and load like any COMPAS data
```

* **Overrides are hard constraints.** `place_before` / `place_after` (relative, survive beams being
  added or removed), `keep_order`, `place_by_hand`, `pin(key, position)` (absolute; negative counts from
  the end), `use_strategy(name)`, `forget(key)`. Contradictions raise and name the cycle; an order the
  joints cannot allow ends in a stuck report saying which precedence blocked.
* **Plans are data, kept apart from the model.** `plan.position(key)`, `plan.insertion_vector(key)`,
  `plan.state(key)`, `plan.is_complete`, `plan.stuck`, `plan.staleness` (overrides that no longer
  match anything), `plan.proposed_by_hand` (what the solver suggests placing by hand).
* **Plans and models are compared, not trusted.** `plan.check(model)` says which beams are missing, new,
  or changed in geometry or joints since the plan was made. `apply_plan(model, plan)` writes the plan
  into the element attributes for export, matching beams by key, so it works on a rebuilt model;
  `assembly_order(model)` reads it back.
* **Options of `plan_sequence`:** `strategy=make_strategy("clearance")` uses a different ranking
  (`compare_strategies` scores them side by side; `scripts/compare_strategies.py` runs that on a saved
  model); `use_geometry=False` skips the collision check and trusts the joints alone; `distance=` sets
  the approach distance (default 100 model units); `width=` sets how many partial orders the search
  keeps (default 5); `tolerance=` sets the rounding for geometric keys (default 1 model unit).
* **Tracing:** `set_tracer(True)` prints each search step, and `set_tracer(lines.append)` collects the
  steps into a list, for a Grasshopper panel.

Beams only. Any other element is excluded and reported in `plan.excluded`.

## Example

[`examples/sequence_model.py`](examples/sequence_model.py) sequences the six-beam frame from
`compas_timber`'s own `examples/model/0001_model.py` (L-miters, a T-lap and T-butts), or any saved
`TimberModel` JSON you pass it:

```bash
python examples/sequence_model.py
python examples/sequence_model.py tests/timber/data/model_test.json
```

It prints what each joint allows, the plan (order, insertion vector, fit, clearance angle), the
strategies side by side, an adjustment with overrides and its diff, and the plan saved, checked against a
rebuilt model and applied. `tests/test_examples.py`
runs it, so it stays working.

## Joint kinematics

`kinematic_constraint(joint, moving_element)` returns how a member may be pulled out of a joint
while the other members stay put:

* a `Line`: a strict 1-DOF axis, **signed**, so the member leaves along `start -> end` only;
* a `Vector`: the inward normal of a permitted half-space;
* a list of vectors: the intersection of those half-spaces.

It is implemented for the butt (T, L, Y), lap (L, T, X), L-French-ridge lap, L-miter, tenon-mortise
(T, L), T-dovetail, T-step, T-birdsmouth and I-simple-scarf joints. Any other joint gets a
permissive fallback, which `is_fallback(joint)` reports. Results that rely on
the fallback are counted in `SequenceResult.inferred_total`.

There is one module per `compas_timber.connections` module, with the same name: the kinematics of
`TDovetailJoint` (`compas_timber/connections/t_dovetail.py`) live in
`compas_timber_sequencing/kinematics/t_dovetail.py`. Each defines a `constraint` function and
registers it for its joint class.

The registry is explicit and matches the **exact** joint class. A subclass does not inherit its
parent's kinematics: kinematics are specific to each joint and are checked by hand, so a joint
nobody has looked at gets the flagged fallback rather than a borrowed answer. A subclass that really
behaves like its parent is registered for the same function, as `TTenonMortiseJoint` and
`LTenonMortiseJoint` are. A test lists every beam joint in `compas_timber` and fails when one is
neither implemented nor knowingly left out (`BallNodeJoint`, `CompositeJoint`, `XNotchJoint`,
`TOliGinaJoint`, and the abstract `LapJoint`), so a joint added upstream shows up on the nightly run.

To support a joint of your own, register it:

```python
from compas_timber_sequencing.kinematics import register


@register(MyJoint)
def constraint(joint, moving_element, assembled_elements=None): ...
```

These functions read joint attributes from outside `compas_timber`. Almost all are public, and two
are private methods (`ISimpleScarf._get_beam_side`, `TBirdsmouthJoint._get_cutting_planes`).
`compas_timber` is therefore pinned to `>=3.0,<4`, and a nightly CI job runs the tests against
`compas_timber`'s main branch.

## How it works

```
TimberModel ──► timber/adapter.py ──► SequencingInput ──► core/search.py ──► SequenceResult ──► element.attributes
                    │                   (plain data +              │                                  (timber/attributes.py)
                    └── kinematics/ ◄── constraints callback)     ├── core/blocking.py  locked in every order? (once, up front)
                        (per-joint freedom)                       ├── core/solver.py    can this element move right now?
                                                                  └── ranking/          which movable one goes first?
```

1. **Adapter** (`timber/adapter.py`). Reads the beams (every other element is excluded), their
   joints, centerline heights and lengths, and turns them into a `SequencingInput`: plain ids
   and numbers, plus two callbacks. One gives the constraints on an element from its joints that
   still have a member in place, via `kinematic_constraint`. The other sweeps the element's
   bounding box 100 model units along a direction and checks for collisions with the elements it
   is not jointed to.
2. **Locked in every order** (`core/blocking.py`). A pair is an edge when the joint alone locks one
   member. Cycles of such edges can never come apart, so they are placed by hand.
3. **Search** (`core/search.py`). Disassembly, carrying the 5 best partial orders. At each step every
   remaining element is solved against what is still in place (`core/solver.py`: the direction with
   the most clearance inside the cone its joints allow, which also has to pass the collision
   check). The movable ones are ranked by a strategy (`ranking/`), which by default keeps
   the structure connected to the ground, takes nothing out from under something jointed above
   it, and goes top-down.
4. **Result.** Reversed into an assembly order, with the extraction vectors negated into
   insertion vectors, a roomy/tight state and clearance angle per element, the hand-placement
   set, and structured reports for dead ends, unhonourable pins and stale overrides.

| Folder / module | Imports `compas_timber` | Role |
|---|---|---|
| `core/boundary.py` | no | `SequencingInput`, everything the algorithms may know |
| `core/constraints.py` | no | `HalfSpace`, `SignedAxis` |
| `core/solver.py` | no | One element, one state: `Solution` or `Locked` |
| `core/blocking.py` | no | Intrinsic locks, ground connectivity, subassemblies |
| `core/search.py` | no | Beam search; `generate()` |
| `core/result.py`, `core/trace.py` | no | Result types; opt-in tracing (`set_tracer`) |
| `core/overrides.py` | no | `SequenceOverrides`: precedences, pins, hand placement, strategy |
| `ranking/context.py` | no | What a strategy is given (`RankingContext`), the strategy base class, `rank()` |
| `ranking/terms.py` | no | The named ranking terms (`stable`, `clear_above`, `base_z`, ...) |
| `ranking/strategies.py` | no | `gravity`, `layered`, `clearance`, `skeleton`, `subassembly`, `chain`, `random`, and custom strategies |
| `ranking/compare.py` | no | Scores several strategies on one model, side by side |
| `kinematics/` | yes | Per-joint freedom: one module per `compas_timber` joint module, explicit registry by exact joint class |
| `timber/keys.py` | yes | Stable element keys: names, or a fingerprint of the geometry |
| `timber/plan.py` | yes | `plan_sequence()`, `SequencePlan`, `plan.check(model)`, `apply_plan()` |
| `timber/diff.py` | yes | `diff(old, new)`: what changed between two plans |
| `timber/adapter.py` | yes | `TimberModel` → `SequencingInput`; the lower-level `generate_assembly_sequence()` |
| `timber/attributes.py` | yes | Writes the result into `element.attributes`; `assembly_order()` reads it back |
| `timber/collision.py` | no | The swept-box collision test the adapter uses (pure `compas` geometry) |

`core/` and `ranking/` never import `compas_timber`, `kinematics/` or `timber/`; `tests/test_layering.py`
enforces it. Everything public can also be imported from the package root.

More:

* [`docs/spec.md`](docs/spec.md): the design in full, including what each joint returns (§3), the collision check (§4.4), the search (§6), the strategies (§7) and exactly what is read from `compas_timber` (§10).
* [`docs/explained.html`](docs/explained.html): an illustrated walkthrough.
* [`docs/plan.md`](docs/plan.md): the original implementation plan (historical).

## Working with compas_timber

`compas_timber` never imports this package and needs no changes. This package reads from it:

* **From the model:** `elements()`, `joints`, `get_joints_for_element()`.
* **From elements:** `guid`, `is_beam`, `is_group_element`, `children`, `centerline`, `obb`, `attributes`.
* **From joints:** `elements`, and each joint type's planes, reference-side indices and parameters (see [`docs/spec.md`](docs/spec.md) §10).

It writes only `element.attributes`, and only when you call `apply_plan`: `assembly_sequence`,
`insertion_vector`, `requires_manual_assembly` and `extraction_state`.

## Known issues

* **The `clearance` strategy is sensitive to the last digit of the geometry.** It ranks on the raw
  clearance value, so beams with equal clearances can swap places when the geometry changes by
  rounding error alone (this happened between `compas_timber` 2.x and 3.0 on one test model).
* **Joints with more than two members** (the Y-butt) are evaluated as if all members were in place,
  as long as any one of them is. That is conservative: it can only report less freedom than there is.

## Development

```bash
pip install -e ".[dev]"
pytest
ruff check . && ruff format --check .
```

The tests mirror the package. `tests/core` and `tests/ranking` exercise the algorithms on the
hand-built inputs in `tests/synthetic.py`. `tests/kinematics` checks every joint's constraints, and that
every beam joint in `compas_timber` is either implemented or knowingly left out. `tests/timber` runs the
adapter and the collision test on real models, and includes a regression test against the results the
sequencer produced before it moved out of `compas_timber`. `tests/test_layering.py` enforces which
folders may import `compas_timber`, and `tests/test_examples.py` runs the example.

CI runs the tests on Python 3.9 (Windows), 3.12 and 3.14 (Ubuntu) on every push, plus a nightly run
against `compas_timber`'s main branch (`.github/workflows/nightly.yml`) to catch upstream changes to
the joint internals that `kinematics/` reads.

## Origin

Developed inside `compas_timber` on branch `feature/joint_kinematics`, archived at tag
`archive/joint-kinematics`. There, the sequencer was the `assembly_sequencing` package, and the
joint kinematics were `get_kinematic_constraint` methods on the joint classes. Both moved here so
that `compas_timber` stays unchanged and the sequencer can evolve independently.

## License

MIT, see [LICENSE](LICENSE).
