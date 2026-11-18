# R-REPLAY-MAGNETIC-MODEL: Replay geographic magnetic model

**ID:** `R-REPLAY-MAGNETIC-MODEL`

**Applies to:** PX4 replay host preparation/execution.

**Uses:** [R-MAGNETIC-REFERENCE](../../runtime/magnetic/reference.md).

## Required behavior

`replay_px4.py --magnetic-model PATH` optionally calls the host-built
`px4_magnetic_model` batch bridge. Build it by supplying an existing target through
`FORMAL_ESKF_PX4_WMM_TARGET`; that target must export the model functions and
`geo_mag_declination.h` include directory. The project neither bundles/downloads
PX4 nor includes its data tables. Without that target, the ordinary replay and
portable library have no new dependency. The embedded adapter should call its
host model directly; the subprocess is only offline preparation.

- The bridge consumes latitude/longitude degrees and returns declination and
  inclination degrees plus microtesla strength using the host's `get_mag_*` APIs.
  Invalid coordinates, malformed input, non-finite/nonpositive output and I/O
  failure terminate preparation; never silently fall back to a measured field.
  Preparation records the executable path/hash. Data-table epoch/version and
  altitude/time dependence are properties of the supplied model, not certified
  by a successful lookup. The tested reference is the pinned PX4 v1.16.2 table.
- Geographic health data is available regardless of `EKF2_DECL_TYPE` bit 0.
  The existing declination selector determines initial heading and
  `use_model_heading`; changes to source settings still require a new replay.
  With geographic heading enabled, both roles use the adopted model field.
  Otherwise preserve `R(q_seed) * mean(m_b_seed)` for observation, innovation,
  filtered heading consistency and heading resets. The adopted model supplies
  only strength/inclination checks. Transform each position's local-NED model
  vector through ECEF into the fixed replay NED frame before expressing its
  D/I/B. Never alter the model's declination to match the chosen heading.
- `magnetic_model.csv` contains one available seed row, then exactly one row per
  navigation event: timestamp, availability, D/I/B. Unavailable rows do not carry
  model evidence. Numerical/structural input errors and missing/mismatched/extra
  rows fail. Only a successful horizontal-position correction/reset may cache
  that event's available candidate; rejected quality/position and velocity-only
  success cannot replace it. At magnetic events, recent own horizontal aiding
  permits periodic checks of the cached candidate through `R-MAGNETIC-REFERENCE`.
- This replay uses accepted GNSS locations, whereas PX4 normally queries its
  own estimated global location. The fixed frame never rebases here; repeated
  checks of a retained candidate do not claim a fresh geographic measurement.
  The deployment adapter must supply its own valid position/origin policy.
- With geographic heading enabled, the model owns the observation-reference
  revision starting at one and forwards adopted `field_n` and revision to the
  lifecycle. Context must retain revision zero and its seed field, while
  device/calibration and flight flags may change. With health-only model use,
  context retains observation-reference ownership: apply its normal versioning
  rules and never forward a model revision as a heading-reference change.
  A health-only model change can reject a field but cannot itself request a
  heading reset.
  Existing source/reference reset policy remains authoritative.
- `magnetic_reference_updates.csv` records candidate source time, fusion time,
  adoption decision, model revision and unmodified field; the summary counts
  checks/adoptions and records `magnetic_model_for_heading`. Source metadata
  separately identifies the selected declination and observation field.
  `--no-mag` keeps the same initialization but does not advance the
  active model after the seed. Seed mode must retain its previous replay outputs.

Native replay tests cover accepted updates, rejected positions and unavailable
model data, health-only model updates and strength rejection with an independent
observation reference. This matches separation of health and declination-source
selection in PX4, not its learned earth-field/bias states or their WMM resets.
The geographic-versus-seed comparison changes initialization as well
as the reference; it is not a flight-accuracy score or an isolated correction
experiment. Do not relax field/gate thresholds or invent magnetic bias to make
an external reference agree with the measured field.
