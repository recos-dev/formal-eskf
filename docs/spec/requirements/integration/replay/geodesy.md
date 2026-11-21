# R-REPLAY-GEODESY: Replay geodetic positions and vector frames

**ID:** `R-REPLAY-GEODESY`

**Status:** Draft

**Applies to:** Host PX4-log and KITTI OXTS preparation, not the live PX4 module.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [F-GEO-ECEF](../../math/geodesy/geodetic-ecef.md),
[F-GEO-NED](../../math/geodesy/local-ned.md).

## Required behavior

Replay preparation shall use the shared C++ geodesy component, in binary64, for
geodetic positions and local-frame bases. One origin is selected by the existing
seed policy and retained for the whole replay; no automatic rebasing is added.
Python shall not maintain a second implementation of the ellipsoid/basis maps.

- PX4-log positions use `alt_ellipsoid`, converted from millimetres to metres.
  KITTI preserves its explicit demo assumption that OXTS altitude is WGS84
  ellipsoidal height. Record that assumption; successful conversion does not
  establish the dataset's vertical datum. Neither adapter silently substitutes
  MSL height or applies an inferred geoid offset.
- Preserve the existing event selection, timestamps, initial attitude and CSV
  schemas. Positions are fixed-origin NED metres. Rotate receiver/sample-local
  NED velocity into that same frame; KITTI's Up velocity changes sign before
  the rotation. Geographic magnetic vectors use the same sample-to-origin basis
  before recomputing declination, inclination and strength.
- Keep the GNSS quality monitor's raw receiver-local velocity and adjacent-fix
  spherical/MSL displacement under [R-REPLAY-GNSS-QUALITY](gnss-quality.md).
  Trajectory conversion shall not replace these quality inputs, discard bad
  fixes, or sanitize malformed velocity/accuracy values into valid measurements.
- Invalid geodetic input, including height outside F-GEO-ECEF's domain, or a
  failed conversion shall stop preparation. No silent Python fallback, clipping
  or partial batch acceptance is permitted. Nonfinite velocity/quality data
  retain their existing downstream rejection semantics.
- Bind coordinate preparation to the selected replay executable. Validate its
  exit status and one finite position/basis result per requested location.
  Record the coordinate-conversion implementation in source metadata. A batch
  subprocess is host preparation, not an embedded API or a real-time execution claim.

## Acceptance and verification

[test_replay_geodesy.py](../../../../../tests/replay/test_replay_geodesy.py) invokes
the actual C++ bridge; it does not substitute the geodesy computation.

| Check | Required observation |
|---|---|
| Axes and transport | Analytic equator/90-degree fixtures detect degrees/radians, NED signs, translation and omitted velocity rotation. |
| Invalid batch | Malformed, nonfinite and out-of-domain coordinates fail, including a bad row after a valid output row. |
| KITTI preparation | Real preparer uses the fixed origin, Up-to-Down conversion and selected navigation cadence; unavailable navigation remains skipped. |
| PX4-log preparation | Distinct ellipsoid/MSL fixtures preserve both trajectory and quality meanings, bad fixes, nonfinite raw velocity and exact row timestamps. |
| Invalid replay height | Real preparer fails before publishing input CSVs. |
| Geographic magnetic field | A controlled external field, passed through the real bridge and preparer, rotates consistently; unavailable model rows retain their markers. This does not test WMM accuracy. |

The CTest binding is conditional on replay examples being built and runs in the
existing CI's additional-test selection. Recorded-log comparisons additionally
check operation counts, timestamps and numerical differences with the prior
preparer. Such runs are regression evidence, not independent navigation truth
or a uniform error bound. Component numerical/source proof gaps remain those of
F-GEO-ECEF/F-GEO-NED; this caller binding has no Lean/ESBMC proof yet.
