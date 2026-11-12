## Summary

This specification defines the C++ language and runtime profile for the shared ESKF core within the boundary established by [Formal Verification Strategy](formal-verification-strategy.md). It owns source-language and portability constraints; [STYLE.md](../../STYLE.md) owns readability, naming and organization.

The language-subset rules apply to handwritten numerical-core and portable-runtime code. Backend and platform interfaces follow the boundaries below; their external implementation requirements do not expand the shared-code subset. Host tools and tests may use facilities outside the core profile, with the same readability principles.

## Language baseline

The shared core uses C++20 language mode with the restricted subset below.

The standard version alone does not authorize every C++20 feature. Each used construct must be supported by the selected verifier and all supported target compilers. Permission to use a feature is not evidence that a particular use has been proved.

## Design baseline

ArduPilot's actual implementation practice is the primary reference for the structure of estimator and runtime code. The reference is the concrete types and direct member-function sequence in [its EKF](https://github.com/ArduPilot/ardupilot/blob/546cf152a8c868acbac82dd326f9649da66d6ba1/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp#L613), with generic numeric utilities in [AP_Math](https://github.com/ArduPilot/ardupilot/blob/546cf152a8c868acbac82dd326f9649da66d6ba1/libraries/AP_Math/AP_Math.h#L24). These links pin the reviewed revision. Naming and formatting remain defined by STYLE.md and `.clang-format`.

Compare equivalent design patterns in the same layer, including their C++11 spellings. A technique used in ArduPilot's math or HAL utilities does not establish a precedent for using it in estimator control flow. Where the inspected comparable code does not use a pattern, our default is not to introduce it; the absence of an explicit ArduPilot prohibition is not justification. This is formal-eskf's chosen design rule, not a claim that ArduPilot formally bans the technique everywhere.

C++20 remains available within this subset. Newer syntax must make the permitted design clearer, rather than introduce another dispatch or abstraction framework. This reference does not import ArduPilot's allocation, global-state, virtual-interface or floating-point conventions into the shared core.

## Template boundaries

| Code responsibility | Required approach |
|---|---|
| Scalar, fixed-size math and backend operations | Basic templates for numeric types, backends and dimensions are permitted. Keep storage access and backend-specific machinery here. |
| Compile-time configuration | Select backend, precision and INS/AHRS configuration at a clear boundary. Resolve related state, error, covariance and dimension types together. |
| ESKF prediction, observation, correction and reset | Write direct numerical steps using the resolved types. Common mathematical helpers may remain dimension/backend templates; sensor-specific behavior uses named functions or explicit overloads. |
| Fusion lifecycle and portable runtime | Use ordinary state-holding classes/structs and direct calls. A simple template binding to the selected numerical configuration is permitted; it must not turn each transition into a generic policy-dispatch interface. |
| Deployment adapter | Bind the deployment configuration and convert inputs/outputs. Do not expose backend storage or propagate type-selection machinery through platform callbacks. |

For new or redesigned estimator/runtime code:

- Do not select INS/AHRS behavior with `std::is_same`, `std::conditional` or their modern aliases inside sensor/fusion operations. Resolve type relationships at the configuration boundary and use explicit overloads for different behaviors. Simple type-validity assertions at numeric/backend boundaries remain permitted.
- Do not independently pass `State`, `Error` and their dimensions through every helper when those choices describe one configuration. Independent mathematical dimensions, such as measurement size, remain valid template parameters. There is no arbitrary numeric limit on template parameters.
- Do not introduce generic callable/policy parameters merely to inject an optional estimator stage, or use `std::nullptr_t` as a policy type flag. Use a named operation and an explicit compile-time choice. Preserve existing requirements that disabled features incur no runtime dispatch and that gain constraints affect both state and covariance consistently.
- Keep long dependent-type spellings such as `typename Configuration::template NominalState<Linalg>` at type/configuration boundaries. Use resolved aliases in estimator interfaces and bodies; do not hide the relationship behind additional layers of traits or alias forwarding.
- Long argument lists and forwarding are not violations by themselves. Keep each input's purpose and each layer's interface/contract responsibility clear; do not add layers solely to combine and then redispatch already distinct INS/AHRS paths.

Preserve the reusable fixed-size linalg abstraction and independently callable numerical APIs. These rules do not require a global backend selection, a single precision, a particular class hierarchy, or duplication of shared ESKF mathematics. Extensions to these boundaries follow the specification-change rule below.

## Language subset

The examples illustrate syntax, not new API declarations or required class designs. Use the existing project types and abstractions. A permitted syntax feature must also satisfy the template boundaries above.

### Ordinary code

| Feature | Example | Rule |
|---|---|---|
| Fundamental and fixed-width types | `bool active;`, `std::uint64_t time_us;` | Make representation, units and valid ranges clear where they matter. |
| Plain structs | `struct Sample { vector3_type acceleration; };` | Use direct fields for data aggregates; do not add accessor wrappers without an invariant. |
| Ordinary classes and members | `class Estimator;` | Encapsulate state, ownership or invariants with a clear purpose. |
| Aggregate/value initialization and member defaults | `Sample sample{};`, `bool active = false;` | Initialize objects explicitly; do not rely on platform allocator zeroing. |
| Const and references | `state_type const & input`, `state_type & output` | Distinguish read-only borrowing from mutation; preserve documented aliasing. |
| Scoped enums | `enum class Mode { idle, active };` | Give states and choices distinct names and types. |
| Named functions and methods | `Status predict(imu_sample_type const & sample);` | Make the operation and its failure behavior explicit. |
| Type aliases | `using covariance_type = linalg::Matrix<Linalg, 15U, 15U>;` | Resolve dependent types near the configuration boundary; avoid chains of aliases without added meaning. |
| Simple constexpr constants/functions | `constexpr std::size_t dimension = 15U;` | Use for constants and straightforward bounded calculations, not a compile-time programming framework. |
| Compile-time assertions | `static_assert(Rows > 0U);` | Check dimensions and configuration invariants close to their definitions. |
| Ordinary branches and bounded loops | `for (std::size_t i = 0U; i < 3U; ++i)` | Use direct control flow with a known upper bound. |
| Fixed-size containers | `std::array<bool, 3U> enabled{};` | Keep capacity static and operations allocation-free. |
| Explicit numeric conversions | `static_cast<value_type>(time_us)` | Check range and precision requirements; a cast alone does not establish validity. |
| Result-use and exception annotations | `[[nodiscard]] Status update() noexcept;` | Mark results that must be handled; noexcept does not replace validation. |
| Explicit/defaulted/deleted special members | `explicit Estimator(parameters_type const &);`, `Estimator(Estimator const &) = delete;` | Use when conversion, construction or ownership semantics require them; omit redundant declarations. |

### Features with restricted use

| Feature | Example | Restriction |
|---|---|---|
| Local type deduction | `auto const R = rotation_matrix(q);` | Use when the type is apparent or the explicit spelling obscures the expression. Keep important public interface types explicit. |
| Basic class/function templates | `template <typename Linalg>` | Keep actual variation at the boundaries above; do not propagate independently selectable state/error/policy types through estimator operations. |
| Integer non-type parameters | `template <std::size_t Rows, std::size_t Columns>` | Use for fixed dimensions, capacities and explicit compile-time choices. |
| Ordinary overloads | `bool valid(ins_state_type const & state);`, `bool valid(ahrs_state_type const & state);` | Prefer a small, explicit overload set to detecting the same distinction again inside a generic dispatcher. |
| Mathematical operators | `F * P * transpose(F) + Q` | Keep familiar mathematical meaning, eager evaluation and bounded storage. |
| Simple compile-time branches | `if constexpr (UseApproximation)` | Limit to explicit configuration choices; do not use state-type detection to redispatch INS/AHRS inside estimator operations. |
| Short local lambdas | `auto square = [](value_type x) { return x * x; };` | Keep behavior and captures obvious and local; do not store escaping references or build a generic callback framework around them. |
| Non-owning pointers and nullptr | `bias_permissions_type const * permissions` | Prefer references for required borrowed objects. Document absence, lifetime and access bounds when a pointer is needed. |
| Conditional compilation | `#if ESKF_MAG_TILT` | Use explicit, documented build choices; preserve configuration-specific verification. |

An operator overload must have the same eager behavior as its named operation. Expression-template behavior is confined to a backend adapter.

Concepts/requires clauses, SFINAE overload machinery, variadic/forwarding frameworks and layered policy/traits dispatch are not part of the default subset for new shared code. Existing simple type traits or compile-time branches are not an instruction to expand such mechanisms. Before extending the subset, document the concrete need, simpler alternatives, same-layer ArduPilot comparison and compiler/verifier implications in a specification change. A newer language version or the lack of an upstream prohibition is insufficient justification.

Standard-library facilities are selected by their actual behavior and verification support. Existing fixed-size facilities such as `std::array` do not need custom replacements merely to avoid `std::`. Introduce additional dependencies only under the dependency boundary below.

### Readable failure handling

Keep the success/commit boundary explicit. For example, where the operation's contract permits this validation order:

```cpp
Status const status = compute_prediction(input, candidate);
if (!succeeded(status))
{
    return status;
}

output = candidate;
return Status::success;
```

This is a syntax example, not a change to existing APIs. For state/covariance transitions, all required outputs must succeed together before publication. Keep domain-specific result data where it distinguishes rejected, failed and fused operations.

## Disallowed core features

The shared ESKF core does not use:

~~~text
dynamic allocation
exceptions
RTTI or dynamic_cast
virtual dispatch
recursion
variable-length arrays
unbounded loops or loops without a compile-time upper bound
mutable global or thread-local state
threads, mutexes, atomics, or volatile device access
filesystem, logging, clocks, or operating-system APIs
compiler-specific vector intrinsics
ranges, coroutines, or format
~~~

Platform adapters may use OS facilities and framework-required interfaces outside the shared core. For example, a PX4 work-item override does not permit virtual dispatch inside the numerical core. Portable runtime decisions remain separate from device access, scheduling and publication as specified in [the architecture boundaries](eskf-software-architecture-and-verification-boundaries.md).

## Memory and execution

All matrix dimensions and maximum loop bounds are compile-time constants. Bounded loops may exit early; their maximum work must remain statically bounded.

Estimator state, covariance, and numerical workspace have statically bounded sizes. Large temporary storage is visible in a workspace definition or stack-usage report.

Every object is initialized before use. The implementation does not depend on:

~~~text
out-of-bounds access
invalid lifetime or dangling references
strict-aliasing violations
signed integer overflow
unsequenced side effects
unspecified evaluation order
reinterpretation of object representation
~~~

The core is deterministic for the same input and numerical profile.

## Error handling

Expected numerical failures use explicit return values:

~~~text
Result<T, Error>
Status
~~~

The core does not throw, abort, log, retry, or select a platform-specific fallback. Public transitions commit state only after all required operations succeed.

## Dependency boundary

The shared core may depend only on:

~~~text
project scalar-math operations
project fixed-size linalg abstraction
project SO(3)/UnitQuaternion layer
verifier-approved basic C++ headers
~~~

Eigen types and expressions do not appear in shared-core state, interfaces, or equations. They remain inside `EigenBackend`.

Standard mathematical functions are isolated behind the scalar-math boundary. New standard-library dependencies require checked allocation, lifetime, bounded-execution and verifier/target behavior; do not infer support from the C++ standard version alone.

## Configuration and reproducibility

Algorithm variants use explicit configuration choices, simple compile-time branches, separate functions or documented macros. Keep these choices at the relevant boundary instead of spreading type dispatch through the estimator. Preprocessor conditionals do not silently change state layout or ESKF equations.

Any conditional compilation that changes verified behavior creates a separate verification target.

A reproducible build record contains:

~~~text
compiler name and exact version
target triple and ABI
C++ standard mode
preprocessor definitions
optimization level
exception and RTTI settings
linker and runtime-library versions
reference to the selected numerical profile
~~~

Floating-point behavior and flags are owned by the scalar/IEEE-754 issue rather than repeated here.

## Adoption and review

Apply the subset to new code and changes in the current task. Existing code may predate these rules; record deviations as refactor candidates rather than expanding them or performing a blanket rewrite. A style deviation alone is not a correctness defect. Preserve public interfaces, behavior, failure/aliasing contracts and compile-time defaults unless their change is explicitly in scope.

For each change, check that:

- The design follows the same-layer reference and template boundaries, not merely syntax accepted by the compiler.
- Template parameters correspond to real variation and important types can be understood locally.
- Helpers have a mathematical, contract or ownership purpose beyond forwarding arguments.
- State ownership, validation order and successful publication remain explicit.
- Added language/library features meet the relevant target and verification constraints.

Template definitions may remain in headers. Moving them into `.cpp` with explicit instantiations changes the supported linking/extension model and requires an explicit design decision; it is not mandatory style cleanup.

## Trusted computing base

Source-level verification does not by itself verify the compiler, assembler, linker, runtime libraries, operating system, generated machine code, or processor.
