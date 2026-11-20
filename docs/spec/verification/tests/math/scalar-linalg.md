# Scalar and Linear Algebra Tests

**Status:** Draft
**Scope:** [F-SCALAR](../../../requirements/math/scalar/operations.md),
[F-LINALG](../../../requirements/math/linalg/operations.md) and
[F-SOLVE](../../../requirements/math/linalg/spd-solve.md); public primitives,
shared derived operations and deployed backend adapters.
**Level / purpose:** Unit and backend-conformance regression; compile-time cases
are build checks, not runtime tests.
**Profiles:** [Shared profiles](../profiles.md). Both scalars, all supported
backends; no estimator macros. Controlled-backend cases are identified below.
**Setup:** Exact row-major fixtures, independent scalar/index calculations and
independent high-precision matrix reference. Fresh outputs with distinct entries;
restore controlled backend/scalar returns after each case. Use all shapes listed
in F-LINALG for construction/access/compatible operation instantiation; exercise
rectangular nonsymmetric products, not square identity matrices alone.
**Regression:** Scalar, linalg, adapter, compiler or dependency changes; select
downstream SO(3), core and runtime numerical consumers as applicable.

## Cases

| Case ID | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|
| TC-F-SCALAR-BASIC | [F-SCALAR](../../../requirements/math/scalar/operations.md#required-behavior) | `pi`, `is_finite`, `absolute`: ±0, finite signs, min normal/subnormal, max finite, ±Inf, NaN. | Classification and absolute magnitude per scalar contract; pi compared to an independently enclosed constant, not production pi. NaN payload not prescribed. Detects wrong format/classification. | Native: [test_scalar.cpp](../../../../../tests/math/test_scalar.cpp) |
| TC-F-SCALAR-SQRT | F-SCALAR, checked root | sqrt at ±0, 1/4, 4; negative normal/subnormal, ±Inf and NaN; output aliases the original scalar variable. | Exact roots for dyadic squares, success for -0, negative finite → domain_error, nonfinite → non_finite_input; failure preserves output. | Native: [test_scalar.cpp](../../../../../tests/math/test_scalar.cpp) |
| TC-F-SCALAR-SINCOS | F-SCALAR, checked pair | Angle 0, both signs, quadrant/interior values and a declared large-angle domain; seed unequal sine/cosine outputs; pass the previous sine or cosine field as the by-value angle. | Pair matches independent bounded trig reference; 0 has sine 0/cosine 1. Nonfinite input rejected without publishing either field. Detects sine/cosine swap and partial update. | Partial: [test_scalar.cpp](../../../../../tests/math/test_scalar.cpp); independent interval oracle on declared fixtures; uniform libm error acceptance remains open |
| TC-F-SCALAR-ATAN2 | F-SCALAR, atan2 | All four quadrants, signed axis values, all four (±0,±0) combinations, either argument nonfinite; output separate or aliases y/x. | Independent argument-order/quadrant oracle; all zero pairs domain_error; nonfinite input precedes origin handling. Exact axis zero where specified; nonzero angles need numerical bound. | Partial: [test_scalar.cpp](../../../../../tests/math/test_scalar.cpp); axis and diagonal interval fixtures; uniform libm error acceptance remains open |
| TC-F-SCALAR-RESULT | F-SCALAR, finite result/publication | Controlled math returns NaN/Inf separately from sqrt, sin, cos, atan2 for valid inputs; check forwarded arguments and outputs. | non_finite_result and unchanged output, including both SinCos fields. Control demonstrates caller checks, not deployed libm accuracy. | Native: [test_scalar.cpp](../../../../../tests/math/test_scalar.cpp) |
| TC-F-LINALG-CONSTRUCTION | [F-LINALG construction/types](../../../requirements/math/linalg/operations.md#required-operations) | Default/zero, square identity, nonrepeating row-major values; copy/move and assignments, then mutate source/destination. Instantiate every required shape. | Zero/identity/index values exact; returned/copied value owns its coefficients and survives source mutation. Do not require a moved-from value to retain its contents. | Native: [test_linalg.cpp](../../../../../tests/math/test_linalg.cpp) |
| TC-F-LINALG-ACCESS | F-LINALG, coefficient/block/segment access | Read const/mutable row-column and vector access; first/last/interior block and segment; set distinct subregion and full-object self assignment. Mutate extracted result. | Correct coefficient/offsets; outside written region unchanged, extraction owning. Single-index vector access agrees with (index,0). Detects storage layout or view leakage. | Native: [test_linalg.cpp](../../../../../tests/math/test_linalg.cpp) |
| TC-F-LINALG-SHAPES | F-LINALG, compile-time bounds | Compile valid required shapes; separately instantiate zero dimensions, rectangular identity, scalar-vector access on nonvector, mismatched products, out-of-range/overflowing block offsets and segments. | Valid shapes compile; prohibited operations fail compilation. No runtime out-of-bounds test or obligation to reject invalid indices. | Native: [test_compile_contracts.py](../../../../../tests/common/test_compile_contracts.py) |
| TC-F-LINALG-ELEMENTWISE | F-LINALG, basic arithmetic | Distinct positive/negative dyadic coefficients, zero, scalar factors 0, -1, 2, 1/2, division by nonzero powers of two; nested expressions and assignment over an operand. | Coefficients equal hand arithmetic, eager results own storage, inputs unchanged until explicit assignment. No promised Status from raw divide-by-zero/overflow. | Native: [test_linalg.cpp](../../../../../tests/math/test_linalg.cpp) |
| TC-F-LINALG-PRODUCT | F-LINALG, products/transpose | A=2x3 rows (1,2,3),(4,5,6), B=3x2 rows (1,-1),(2,0),(0,3); plus one-hot rows/columns at each inner index and 15x12/12x15 products. | AB=[[5,8],[14,14]], transpose swaps shape/index; each inner term contributes once. Distinct rectangular products catch accidental ABᵀ/column-major indexing. | Native: [test_linalg.cpp](../../../../../tests/math/test_linalg.cpp) |
| TC-F-LINALG-REDUCTIONS | F-LINALG, dot/magnitude/trace/diagonal | v=(3,4,0), w=(2,-1,5), signed/dyadic matrices with off-diagonal decoys; zero vector and large finite inputs. | dot=2, squared norm=25, norm=5; trace/diagonal use only correct diagonal entries. General sums use independent reference; raw overflow is not a checked rejection. | Partial: [test_linalg.cpp](../../../../../tests/math/test_linalg.cpp); exact fixture arithmetic and overflow classification; general reduction error bounds remain open |
| TC-F-LINALG-CROSS | F-LINALG, cross | e1×e2, reversed operands, parallel/zero, nonaxis integer vectors. | e3, -e3, zero and explicit determinant coefficients. Detects handedness and swapped axes independently of production hat. | Native: [test_linalg.cpp](../../../../../tests/math/test_linalg.cpp) |
| TC-F-LINALG-FINITE | F-LINALG, all_finite/max_abs | Put NaN and each infinity at every coefficient position; finite signed entries and all zeros; multiple nonfinite entries only for classification. | all_finite rejects any nonfinite; finite max_abs equals independently selected magnitude. max_abs must not hide nonfinite input with a finite result; NaN payload/order is not an accepted numerical oracle. | Partial: [test_linalg.cpp](../../../../../tests/math/test_linalg.cpp); finite classification and max_abs fixtures; detailed max_abs contract ownership remains open |
| TC-F-LINALG-SYMMETRY | F-LINALG, symmetrize/is_symmetric | Nonsymmetric dyadic A, ±0 mirrored entries; residual below/equal/above finite tolerance; negative/NaN/Inf tolerance; nonfinite A; large finite opposite entries causing residual overflow. | Exact fixture yields (A+Aᵀ)/2; inclusive finite residual criterion, invalid tolerance/data false. Do not assume symmetrize avoids intermediate overflow or always returns finite. | Partial: [test_linalg.cpp](../../../../../tests/math/test_linalg.cpp); source behavior candidate; detailed threshold and failure contract ownership remains open |
| TC-F-LINALG-SANDWICH | F-LINALG, sandwich | Rectangular signed/permutation transform and dense correlated symmetric P; also nonsymmetric general P. | Independent indexed sum T P Tᵀ, all entries; avoid using production multiplication on both sides of oracle. Detects lost transpose/cross terms or unjustified symmetry assumption. | Native: [test_linalg.cpp](../../../../../tests/math/test_linalg.cpp) |
| TC-F-LINALG-NORMALIZE | F-LINALG, normalize | v=(3,4,0), threshold 5, immediately above/below 5; zero v; invalid/nonfinite threshold, nonfinite component, norm overflow; disjoint/in-place outputs. Pair norm overflow with nonpositive finite threshold. | (3/5,4/5,0) on valid domain with bounded reference; threshold equality accepted. Source binding currently rejects nonfinite inputs first, then nonfinite norm, then unsafe threshold/norm, preserves failure output. The detailed precedence needs explicit contract ownership before acceptance. | Partial: [test_linalg.cpp](../../../../../tests/math/test_linalg.cpp); exact and rational fixtures; detailed precedence and general numerical envelope remain open |
| TC-F-LINALG-NORMALIZE-RESULT | F-LINALG, checked normalization | Controlled norm producer returns nonfinite or finite unsafe divisor; controlled division produces nonfinite candidate from otherwise valid inputs. | Status and atomic publication according to reviewed normalize binding; actual producer conformance remains a separate obligation. Detects missing postcheck/early write. | Partial: [test_linalg.cpp](../../../../../tests/math/test_linalg.cpp); controlled producer checks; detailed contract ownership remains open |
| TC-F-SOLVE-KNOWN | [F-SOLVE](../../../requirements/math/linalg/spd-solve.md#required-behavior) | S=[[4,2],[2,2]], B=[[8,2],[6,4]]; one and multiple RHS, zero RHS; construct larger well-conditioned S=L Lᵀ with dyadic positive diagonal L. | Known solution [[1,-1],[2,3]], independently computed residual for general sizes. Multiple RHS share one factorization and no inverse (inspection/passive observation); zero RHS does not bypass S validation. | Pending |
| TC-F-SOLVE-RIGHT | F-SOLVE, right solve | Nonsquare B, correlated SPD S; use independently chosen X and form B=X S in reference arithmetic. | Output solves X S=B, not S X=B; check all entries/residual. Detects transpose or argument-order error. | Pending |
| TC-F-SOLVE-DOMAIN | F-SOLVE, ordered checks | NaN/±Inf in each S/B position, finite one-ULP asymmetry, ±0 symmetric entries, zero/negative diagonal, singular and indefinite matrix with positive diagonal. Pair nonfinite B with asymmetric S. | non_finite_input before symmetry/factorization; asymmetry and failed LLT → not_positive_definite. No silent symmetrization; ±0 comparison allowed. Snapshot outputs unchanged. | Pending |
| TC-F-SOLVE-ALIASES | F-SOLVE, publication | Successful and failed left/right solves with output disjoint, alias B, and alias S when full dimensions match. | Same successful solution as disjoint run; failed call preserves the aliased input/output completely. Sentinels detect partial triangular-solve publication. | Pending |
| TC-F-SOLVE-RESULT | F-SOLVE, backend propagation | Controlled backend emits each failure status or success with a nonfinite solution; actual backend witnesses exercise finite input leading to nonfinite intermediates where reproducible. | Failure propagated, success with nonfinite candidate → non_finite_result, never publish failed candidate. Separate actual numeric failure evidence from controlled caller tests. | Pending |
| TC-F-SOLVE-CONDITIONING | F-SOLVE, numerical limits | SPD systems with dyadic eigenvalue ratios from near one to poorly conditioned; record actual input after rounding and a high-precision condition estimate. | No invented ill_conditioned cutoff. Measure normalized backward residual and forward error separately against a reviewed domain/budget; mathematical SPD alone does not mandate successful floating LLT. | Pending, numerical criteria |

## Source allocation

Paths are relative to `include/formal_eskf/`. This is public behavior allocation,
not a requirement for tests named after every internal helper.

| Source | API group → cases above |
|---|---|
| `scalar/math.hpp`, `scalar/backend/standard.hpp` | SinCos, pi/classification/absolute, checked sqrt/sin_cos/atan2 and StandardMath adapter → all F-SCALAR rows. |
| `linalg/matrix.hpp` | Matrix/Vector construction, assignment, coefficient access, block/segment, eager operators → CONSTRUCTION through PRODUCT. |
| `linalg/operations.hpp` | transpose/products/reductions, normalize, cross, finite/symmetry, sandwich → PRODUCT through NORMALIZE-RESULT. |
| `linalg/solve.hpp` | solve_spd/right_solve_spd → all F-SOLVE rows. |
| `linalg/backend/eigen.hpp`, `linalg/backend/px4_matrix.hpp`, `linalg/backend/cmsis_dsp.hpp` | Each primitive reached through the public interface → applicable F-LINALG/F-SOLVE rows on that adapter. |
| `linalg/backend/cholesky.hpp` | Factorization/forward/back substitution reached by the using adapters → KNOWN, DOMAIN, RESULT, CONDITIONING; source inspection checks one-factorization/no-inverse. |
| `linalg/linalg.hpp` | Umbrella inclusion → SHAPES valid consumer build; no independent numerical operation. |

## Gaps and exclusions

- F-LINALG names several derived operations without fully specifying their
  threshold/error priority/nonfinite behavior. NORMALIZE, NORMALIZE-RESULT, FINITE (max_abs) and
  SYMMETRY retain the observed source binding as a proposal, not an approved
  contract. Resolve the canonical clauses before accepting those details.
- General libm, reduction, normalization and solve accuracy budgets need bounded
  input/conditioning profiles. Exact fixtures and status tests can be implemented
  first; arbitrary epsilons or backend-to-backend agreement cannot close this gap.
- Out-of-range runtime indexing, arbitrary template dimensions beyond the
  supported set, external library internals and target timing are excluded.
  CMSIS dependency-specific dimension/padding constraints need adapter inspection
  and sanitizer execution; no access to padding is promised by the public API.

## Review

Pending owner review; all executable allocations remain Pending.
