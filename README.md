# compas_timber_sequencing

Assembly sequencing for [COMPAS Timber](https://github.com/gramaziokohler/compas_timber) models.

Given a timber model, it produces:

* an **assembly order**,
* a unit **insertion vector** per element, for robotic trajectory planning (`compas_fab`), and
* an explicit **hand-placement set**: the elements no robot can insert, which is a real fabrication instruction.

It works backwards. It takes the finished model apart one element at a time, only removing an
element when the joints still holding it let it move, then reverses that order to get the assembly
sequence. What each joint allows comes from `compas_timber` itself, via
`Joint.get_kinematic_constraint`. A beam search then picks a good order among the feasible ones,
ranked by an exchangeable strategy (bottom-up gravity by default).

> **Status: alpha.** Needs `Joint.get_kinematic_constraint`, which is not yet on `compas_timber`'s
> main branch: see branch `feature/joint-kinematic-constraints` of `gramaziokohler/compas_timber`.
> Until that is released, the `timber` extra installs `compas_timber` from that branch.

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
`tests/timber` runs the adapter on real `compas_timber` models. It includes a regression test against
the results the sequencer produced before it moved out of `compas_timber`.

## Origin

Developed inside `compas_timber` as the `assembly_sequencing` package on branch
`feature/joint_kinematics`, archived at tag `archive/joint-kinematics`. It was moved here so the
sequencer can evolve independently, while `compas_timber` keeps only the per-joint kinematics.

## License

MIT, see [LICENSE](LICENSE).
