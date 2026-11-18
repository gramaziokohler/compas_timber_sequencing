# compas_timber_sequencing

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
pip install "compas_timber_sequencing[timber] @ git+https://github.com/gramaziokohler/compas_timber_sequencing.git"
```

Without the `timber` extra you get the sequencing algorithms alone. They depend on `compas` only, and
you supply the constraints yourself through `SequencingInput`.

## Use

```python
from compas_timber_sequencing.timber import assembly_order
from compas_timber_sequencing.timber import generate_assembly_sequence

result, adapter = generate_assembly_sequence(model)

if not result.is_complete:
    print(result.stuck.describe())

for element_id in result.order:
    beam = adapter.elements_by_id[element_id]
    vector = result.insertion_vectors[element_id]  # None means place by hand
    state = result.state(element_id)  # roomy | tight | locked

# The sequence is also written into each element's attributes, and survives a JSON round trip:
ordered_beams = assembly_order(model)
```

Overrides:

* `manual_set=...` declares elements as hand-placed. It is stored on the elements and survives re-runs.
* `pinned_order=...` fixes positions for one particular build.
* `strategy=make_strategy("clearance")` uses a different ranking. `compare_strategies` scores them side by side, and `scripts/compare_strategies.py` runs that on a saved model.
* `use_geometry=False` skips the collision check and trusts the joints alone; `distance=` sets the approach distance (default 100 model units); `width=` sets how many partial orders the search keeps (default 5); `apply_to_model=False` leaves the model untouched.
* `set_tracer(True)` prints each search step, and `set_tracer(lines.append)` collects the steps into a list, for a Grasshopper panel.

Beams only. Any other element is excluded and reported in `result.excluded`.

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

Joints with more than two members (the Y-butt) are evaluated as if all members were in place, as
long as any one of them is.

## How it works

```
TimberModel ──► timber.py (adapter) ──► SequencingInput ──► search.generate() ──► SequenceResult ──► element.attributes
                    │                     (plain data +                │
                    └── kinematics/ ◄──── constraints callback)       ├── blocking.py    locked in every order?
                        (per-joint freedom)                           ├── solver.py      can this element move now?
                                                                      └── preferences.py which movable one first?
```

1. **Adapter** (`timber.py`). Reads the beams (every other element is excluded), their
   joints, centerline heights and lengths, and turns them into a `SequencingInput`: plain ids
   and numbers, plus two callbacks. One gives the constraints on an element from its joints that
   still have a member in place, via `kinematic_constraint`. The other sweeps the element's
   bounding box 100 model units along a direction and checks for collisions with the elements it
   is not jointed to.
2. **Locked in every order** (`blocking.py`). A pair is an edge when the joint alone locks one
   member. Cycles of such edges can never come apart, so they are placed by hand.
3. **Search** (`search.py`). Disassembly, carrying the 5 best partial orders. At each step every
   remaining element is solved against what is still in place (`solver.py`: the direction with
   the most clearance inside the cone its joints allow, which also has to pass the collision
   check). The movable ones are ranked by a strategy (`preferences.py`), which by default keeps
   the structure connected to the ground, takes nothing out from under something jointed above
   it, and goes top-down.
4. **Result.** Reversed into an assembly order, with the extraction vectors negated into
   insertion vectors, a roomy/tight state and clearance angle per element, the hand-placement
   set, and structured reports for dead ends, unhonourable pins and stale overrides.

| Module | Needs `compas_timber` | Role |
|---|---|---|
| `kinematics/` | yes | Per-joint freedom: one module per `compas_timber` joint module, explicit registry by exact joint class |
| `timber.py` | yes | Model ⇄ `SequencingInput`; writes the result into `element.attributes` |
| `boundary.py` | no | `SequencingInput`, everything the algorithms may know |
| `constraints.py` | no | `HalfSpace`, `SignedAxis` |
| `solver.py` | no | One element, one state: `Solution` or `Locked` |
| `blocking.py` | no | Intrinsic locks, ground connectivity, subassemblies |
| `search.py` | no | Beam search; `generate()` |
| `preferences.py` | no | Ranking strategies (`gravity`, `layered`, `clearance`, `skeleton`, `subassembly`, `chain`, `random`) |
| `compare.py` | no | Scores several strategies on one model, side by side |
| `result.py`, `trace.py` | no | Result types; opt-in tracing (`set_tracer`) |

More:

* [`docs/spec.md`](docs/spec.md): the design in full, including what each joint returns (§3), the collision check (§4.4), the search (§6), the strategies (§7) and exactly what is read from `compas_timber` (§10).
* [`docs/explained.html`](docs/explained.html): an illustrated walkthrough.
* [`docs/plan.md`](docs/plan.md): the original implementation plan (historical).

## Working with compas_timber

`compas_timber` never imports this package and needs no changes. This package reads from it:

* **From the model:** `elements()`, `joints`, `get_joints_for_element()`.
* **From elements:** `guid`, `is_beam`, `is_group_element`, `children`, `centerline`, `obb`, `attributes`.
* **From joints:** `elements`, and each joint type's planes, reference-side indices and parameters (see [`docs/spec.md`](docs/spec.md) §10).

It writes only `element.attributes`: `assembly_sequence`, `insertion_vector`,
`requires_manual_assembly` and `extraction_state`.

## Known issues

* **After a stuck run** (`result.is_complete` is `False`), the indices written into the element
  attributes are not absolute assembly positions; see [`docs/spec.md`](docs/spec.md) §11.

## Development

```bash
pip install -e ".[timber,dev]"
pytest
ruff check . && ruff format --check .
```

`tests/core` exercises the algorithms on hand-built synthetic inputs and needs only `compas`.
`tests/timber` covers the joint kinematics and runs the adapter on real `compas_timber` models. It
includes a regression test against the results the sequencer produced before it moved out of
`compas_timber`.

## Origin

Developed inside `compas_timber` on branch `feature/joint_kinematics`, archived at tag
`archive/joint-kinematics`. There, the sequencer was the `assembly_sequencing` package, and the
joint kinematics were `get_kinematic_constraint` methods on the joint classes. Both moved here so
that `compas_timber` stays unchanged and the sequencer can evolve independently.

## License

MIT, see [LICENSE](LICENSE).
