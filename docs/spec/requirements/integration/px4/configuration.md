# PX4-CONFIG: packaged build profile

**ID:** `PX4-CONFIG`

**Applies to:** Single-instance PX4 INS module; linked deployment profile bounds apply.

## Basis

[PX4 module architecture](../../../architecture/px4-module.md) identifies the pinned reference, project adaptations and scope.

## Required behavior

The deployment configuration is immutable during a module session. Kconfig
selections are validated on the host, translated into numeric compile-time
definitions in `DeploymentConfig.hpp`, and packaged with their selected `.config`
and hashes. `DeploymentProfile.hpp` binds them to the existing typed parameters;
the shared core/runtime do not parse configuration or depend on Kconfiglib.
Every module translation unit and its test fixture uses the same algorithm modes.

Preserve the tracked profile's defaults. Validate numeric representation, angular
domains, noise floor/cap relationships, duration units and known arithmetic
constraints before packaging. Unknown symbols, ignored dependency requests and
invalid numbers fail. These checks do not certify arbitrary tuning as flight-safe
or extend existing proof domains; retain the effective profile with each result.

WMM build exclusion prevents geographic lookup/adoption and retains the existing
measured-field and declination fallback. Ground-effect exclusion leaves ordinary
barometric innovation processing in place. Gravity exclusion retains tilt
alignment but omits gravity sampling/correction and overrides `FESKF_GRAV_EN`.
Bias-learning permissions affect correction permissions independently of bias
process noise. Optional GNSS checks do not disable mandatory fix, numerical or
timestamp validation. Runtime health, alignment, flight-state and source-readiness
conditions cannot be forged through configuration switches.

The existing `FESKF_*` startup bindings keep their validated restart semantics.
Required initialization sensors, INS/double/PX4 matrix, timing/storage envelopes,
frame conventions and numerical failure checks remain fixed unless separately
specified. Replay retains its own settings. The
[configuration inventory](../../../../guides/configuration.md#px4-deployment-menu)
owns the menu, units and detailed configuration boundaries.
