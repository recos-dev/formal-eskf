# INS code generation

This package emits four INS15 arithmetic kernel families from the project's
right-local error formulation. `ESKF_CODEGEN=1` (default) selects them in the
production INS core and known runtime observation paths. Set the option to `0`
to retain dense arithmetic. AHRS and arbitrary-H correction products remain dense;
INS covariance reset follows the selected option even after generic correction.

## Generation scope

| Family | Equation | Pinned C++ |
|---|---|---|
| Prediction covariance | F P Fᵀ + F_i Q_i F_iᵀ, including rotated process noise | [ins_covariance.hpp](../include/formal_eskf/eskf/generated/ins_covariance.hpp) |
| Innovation products | C = P Hᵀ; S = H C + V | [ins_innovation.hpp](../include/formal_eskf/eskf/generated/ins_innovation.hpp) |
| Joseph covariance | A P Aᵀ + K_eff V K_effᵀ, with checked A = I − K_eff H | [ins_joseph.hpp](../include/formal_eskf/eskf/generated/ins_joseph.hpp) |
| Covariance reset | G P Gᵀ, G = diag(I₆, J, I₆) | [ins_reset.hpp](../include/formal_eskf/eskf/generated/ins_reset.hpp) |

The [physical model](ins/model.py) exposes state/noise ordering, transition blocks,
noise injection, observation Jacobians and matrix equations before CSE or emission.
It retains the specified truncated error transition; it does not replace that
transition with the derivative of the full nominal integrator.

Existing requirements own behavior: [E-PCOV](../docs/spec/requirements/core/covariance-prediction.md),
[E-NOISE](../docs/spec/requirements/core/process-noise.md),
[E-OBS](../docs/spec/requirements/core/observations.md),
[E-CORRECT](../docs/spec/requirements/core/correction.md) and
[E-RESET](../docs/spec/requirements/core/covariance-reset.md).
Artifact availability does not approve requirements or establish target qualification.

### Observation layouts

Six built-in observation wrappers reduce to five H layouts. Accel and mag share
arithmetic structure, not their residuals, applicability or gain policies.

| Layout | Nonzero H columns in [delta_p, delta_v, delta_theta, delta_b_a, delta_b_g] | Dimension |
|---|---|---|
| Horizontal position | Select 0, 1 | 2 |
| Vertical position | Select 2 | 1 |
| Horizontal velocity | Select 3, 4 | 2 |
| Three-dimensional velocity | Select 3, 4, 5 | 3 |
| Body vector: accel or mag | hat(h) in columns 6–8 | 3 |

Cross products retain every state correlation. Innovation computes **H C first,
then adds V**, with no mirroring of S. Joseph accepts the existing checked A and
the final constrained/projected gain; it retains full correlated measurement
noise and factored products, without an optimal-gain assumption. Covariance
kernels compute the upper triangle and mirror it. Reset transforms all attitude
cross-covariances, not only the 3×3 diagonal block.

These are unchecked kernels. Algebraic equivalence requires the declared matrix
structure and symmetric P/V; prediction also requires diagonal Q_i. Real PSD/PD
results have additional positivity premises. Callers remain responsible for
validation, overflow, status precedence, failure atomicity and output publication.

### Operations kept in ordinary C++

- Nominal propagation, transition/noise block producers and their checked scaling.
- Short observation formulas and constant selectors; physical-model checks verify
  the right-local Jacobian conventions independently of scalar lowering.
- Cholesky solve, gain permissions/projection, delta_x = K_eff r and nominal injection.
- SO(3) branches and the exact/approximate J producer; reset generation consumes J.
- Innovation gates, covariance cleanup, lifecycle, buffers and platform scheduling.

Generic arbitrary-H correction products and AHRS remain dense. Select future
specializations by known model identity, never measurement dimension alone. Known
INS wrappers select an internal compile-time `CorrectionModel`; callers of the
generic API keep its default arbitrary-H model. Runtime admission and
correction select the same model without reusing results across changed priors.
There is no mutable selector, dimension-based model inference or runtime fallback.

## Select in an application

Use `make menuconfig` → `ESKF algorithms` → `Use generated INS matrix arithmetic`.
The option defaults on in Kconfig, direct CMake, header-only use and the packaged
PX4 profile. Keep it identical across translation units. With direct CMake:

```bash
cmake -S . -B build/dense -DESKF_CODEGEN=OFF -DBUILD_TESTING=ON
cmake --build build/dense --parallel 2
ctest --test-dir build/dense --output-on-failure
```

The same transition/noise/Jacobian producers, validation, gain constraints,
solve, injection and atomic publication surround both arithmetic paths. Generated
covariances mirror the computed upper triangle and are checked for finiteness
before publication. They skip the dense path's pairwise average, which would add
unnecessary rounding to an already symmetric result. Prediction/reset retain
symmetric-PSD input as a caller precondition; no acceptance is claimed for
asymmetric priors. Correction still checks exact input symmetry and retains full
correlated V.

This is a numerical implementation choice, not bitwise compatibility: reassociation,
mirroring and omitted structural zeros can change roundoff, overflow and decisions
at numerical boundaries. Failures of the selected arithmetic retain checked status
and unchanged outputs; dense failure equivalence for every IEEE input is not claimed.
Use the configured scalar/backend/compiler profile for numerical acceptance.

## Generate and reproduce

```bash
python3 -m pip install -r codegen/requirements.txt
python3 codegen/generate.py --pin
python3 codegen/products.py generate
python3 codegen/products.py derive
python3 codegen/products.py native-export
python3 codegen/products.py esbmc-export
python3 codegen/products.py lean-export
python3 -m unittest discover -s codegen/tests -v
```

`generate.py` owns prediction; `products.py` owns reset, innovation and Joseph.
Both pass generated C++ through `clang-format-14` with the repository style before
verification/publication. Do not hand-edit generated C++ or Lean. The fixed date
tag is `2026-11-18`, not the invocation date. Scratch records go to ignored
`output/codegen/`; manifests record sources and tools, not executed proof success.

Check pinned C++ without rewriting it:

```bash
python3 codegen/generate.py --pin --check
python3 codegen/products.py generate --check
python3 codegen/products.py esbmc-export --check
```

## Formal and numerical evidence

Apply the [resource limits](../docs/guides/verification.md#proof-execution-resources)
before heavy verification. Commands run sequentially; `--list` only inventories.

```bash
python3 codegen/generate.py --check --pin --lean formal/lean/FormalESKF/codegen
./scripts/verify_lean.sh --wfail
./scripts/verify_esbmc.sh codegen --list
./scripts/verify_esbmc.sh codegen
python3 codegen/check_controls.py
python3 codegen/products.py controls
```

| Evidence | Claim and boundary |
|---|---|
| Independent source/algebra checker | Parse actual formatted C++; check allowed syntax, complete assignments and exact identities against independently constructed matrix equations |
| [Lean prediction](../formal/lean/FormalESKF/codegen) and [products](../formal/lean/FormalESKF/codegen/products) | Exact scalar identities, matrix assembly, conditional PSD/PD and innovation/Joseph algebraic composition |
| ESBMC generated suites | INS covariance/reset/correction dispatch and combined source transactions, actual same-type kernel frames/mirroring and publication; requires the [shared producer dependencies](../docs/guides/verification.md#generated-covariance-prediction) |
| ESBMC `codegen` | Actual pinned kernels and production Matrix wrapper with fixed-array storage: binary32/64 memory/access safety, without arithmetic range restrictions |
| Native rational checker | Recorded finite numerical values agree with IEEE RNE simulation and propagated error bounds against independent exact references |
| Differential arithmetic fixtures | Checked statuses, aliases, failure outputs, constrained gains, gate boundaries and dense/generated differences for executed cases |
| Production integration tests | CTest `formal_eskf.codegen.*` checks actual model selection at the backend boundary, generic/AHRS paths and numerical agreement; normal core/runtime suites cover checked behavior |
| Negative controls | Real Lean and ESBMC must reject deliberately corrupted coefficients and out-of-bounds accesses; timeout is not a pass |

Lean exports parsed expressions, not a proof of Python or the compiler. There is
no machine-checked Lean-to-C++ refinement. ESBMC storage checks do not prove
Eigen/PX4 internals or floating-point accuracy. Finite-record certificates require
nearest ties-to-even, gradual underflow, no FMA contraction and no fast-math;
nonfinite/overflow records are classified separately, not certified as finite.
Signed zeros are numerically equal in these certificates; mirrored covariance
entries are additionally checked bitwise. This does not establish bitwise dense/generated
equivalence, a uniform deployment-domain error budget or floating-point PSD.

## Native evaluation

An x86 host with SSE is required to check FTZ/DAZ. Use a GCC/Clang C++20 compiler;
stack probes require GCC and Linux. Eigen is the default backend. PX4 headers
remain external: use `--backends eigen px4 --px4 /path/to/PX4-Autopilot`.

```bash
# Existing prediction package.
python3 codegen/evaluate.py native
python3 codegen/evaluate.py benchmark
python3 codegen/evaluate.py stack

# Reset, innovation and Joseph; float/double, both reset and tilt modes.
python3 codegen/products.py native
python3 codegen/products.py callers
python3 codegen/products.py conditioning
python3 codegen/products.py regressions
python3 codegen/products.py benchmark
python3 codegen/products.py stack
```

Products default to `-O2`; use `--optimization O3`, `--cxx`, `--cxx-flag`, `--eigen`,
`--reset-modes` and `--tilt-modes` to select profiles. Keep distinct profile sets
in distinct `--output` directories. Prediction's corresponding options are listed
by `evaluate.py --help`. Example additional product profiles:

```bash
python3 codegen/products.py native --cxx clang++-14 --output output/codegen/products-clang
python3 codegen/products.py native --optimization O3 --output output/codegen/products-o3
python3 codegen/products.py native --sanitize --optimization O1 --output output/codegen/products-san
python3 codegen/products.py callers --sanitize --optimization O1 --output output/codegen/products-san
```

`native` includes rational certification. `certify RECORDS.log` rechecks existing
product records; missing corpus entries or altered outputs fail. `callers` keeps
all existing checks in isolated core copies, with an explicit test-only model
selector for comparing arithmetic within one process. `conditioning` exercises
nonzero gains, rank-one PSD priors and ill-conditioned SPD innovations;
`regressions` runs existing injection/correction tests against those copies.
These differential fixtures do not test production dispatch. Run the normal CTest
suite with `ESKF_CODEGEN=ON` and `OFF` for actual integration coverage.

Host benchmarks alternate paired runs on one pinned CPU. They measure kernels
and full checked calls, including generated reset versus direct block multiplication.
Stack probes compile dense/generated paths separately and report compiler-known
paths plus observed written bytes; unresolved external frames remain unbounded.
Neither measurement is target WCET or a whole-program stack bound.

## Structure and integration boundary

- `ins/model.py`: readable physical equations, independent of scheduling.
- `ins/covariance.py`, `ins/products/schedule.py`: sparse staged CSE.
- `ins/prediction_cpp.py`, `ins/products/cpp.py`, `ins/emission.py`: formatted C++ emission.
- `ins/verify/`: independent reference equations, source, Lean and IEEE checkers.
- `tests/native/`: native fixtures and isolated caller/resource probes.
- `formal/cpp/esbmc/codegen*.{cpp,hpp}`: shared proof storage and memory harnesses.

The ESBMC runner explicitly selects `ESKF_CODEGEN=0` for existing core
producer/caller summaries except `pcov-generated`, `reset-generated`,
`correction-generated` and `step-generated`, which select `1` explicitly.
`codegen` checks emitted kernels directly. Accept generated source composition
only with the matching actual producers and complete execution evidence. A
uniform deployment numerical budget and target performance/stack qualification
remain separate obligations.
AHRS generation is deferred.

Logs and experiments stay in ignored output. Retain release evidence under the
[evidence policy](../docs/guides/verification.md#record-ownership-and-reuse), with
source/configuration/tool identity; old results do not certify changed artifacts.
Generation reports also count scalar operations in the parsed DAG before compiler
optimization; these counts are not CPU instruction counts or measured latency.
