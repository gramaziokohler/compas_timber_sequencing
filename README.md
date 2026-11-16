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

Beams only for now. Plates, panels and fasteners are excluded and reported in `result.excluded`.

## Joint kinematics

`kinematic_constraint(joint, moving_element)` returns how a member may be pulled out of a joint
while the other members stay put:

* a `Line`: a strict 1-DOF axis, **signed**, so the member leaves along `start -> end` only;
* a `Vector`: the inward normal of a permitted half-space;
* a list of vectors: the intersection of those half-spaces.

It is implemented for the butt (T, L, Y), lap (L, T, X), L-French-ridge lap, L-miter, tenon-mortise
(T, L, Oligina), T-dovetail, T-step, T-birdsmouth, I-simple-scarf, and plate and panel joints. Any
other joint gets a permissive fallback, which `is_fallback(joint)` reports. Results that rely on
the fallback are counted in `SequenceResult.inferred_total`.

The lookup is by joint class, following inheritance like a method would. To support a joint of your
own, register it:

```python
from compas_timber_sequencing.kinematics import kinematic_constraint


@kinematic_constraint.register(MyJoint)
def _(joint, moving_element, assembled_elements=None): ...
```

These functions read joint attributes from outside `compas_timber`. Almost all are public, and two
are private methods (`ISimpleScarf._get_beam_side`, `TBirdsmouthJoint._get_cutting_planes`).
`compas_timber` is therefore pinned to `>=3.0,<4`, and a nightly CI job runs the tests against
`compas_timber`'s main branch.

## How it works

* [`docs/spec.md`](docs/spec.md): the design (constraint types, solver, blocking graph, search, ranking, overrides, known limitations).
* [`docs/explained.html`](docs/explained.html): an illustrated walkthrough.
* [`docs/plan.md`](docs/plan.md): the original implementation plan (historical).

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
