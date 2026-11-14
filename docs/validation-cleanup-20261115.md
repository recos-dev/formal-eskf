# Defensive-check cleanup acceptance

Baseline: `9efbf7763f5477d9653f1e333011470b7a0c5480`. The fixed implementation
sequence is DC-01–DC-08 in [ROADMAP.md](../ROADMAP.md). Numbers below preserve
the review list; they are work items, not proof requirements or measured speedups.

| Item | Audit origin | Candidate | Disposition |
|---|---|---|---|
| 1 | G01 | Reuse finite input premises in symmetry checks | Pending |
| 2 | G01-C02 | Pairwise symmetry residual | Pending |
| 3 | G01-C01 | Forward gain solve output | Pending |
| 4 | G01-C03, G06-C8, G08-C7 | Same-call PHt/S reuse after gating | Pending |
| 5 | G01-C04 | Hoist bias permissions out of column loops | Pending |
| 6 | G02-C1 | Reuse validated prior quaternion | Pending |
| 7 | G02-C2 | AHRS nominal output forwarding | Pending |
| 8 | G02-021.1 | Acceleration finite check subsumption | Pending |
| 9 | G03-C1 | Diagonal-only process-noise result scan | Pending |
| 10 | G02-C3, G03-C2, G04-C1–C3 | Shared prediction validation and mathematics | Pending |
| 11 | G04-C4 | Scan only variable F_x blocks | Pending |
| 12 | G04-C5 | Reuse -R | Pending |
| 13 | G04-C6 | Forward covariance prediction output | Pending |
| 14 | G05-C1–C2 | Reuse injection/reset input premises | Pending |
| 15 | G05-C3 | Share Exp/Jr angle intermediates | Pending |
| 16 | G05-C6 | Forward covariance reset output | Pending |
| 17 | G05-C8 | Avoid copied-and-overwritten INS candidate | Pending |
| 18 | G06-C1 | Reuse runtime innovation premises | Pending |
| 19 | G06-C2 | Simplify innovation range comparisons | Pending |
| 20 | G07-C1, C5 | Construction-valid H and delayed construction | Pending |
| 21 | G07-C2 | Accel h/z finite check subsumption | Pending |
| 22 | G07-C3, G08-C4 | Finite vector nonzero checks | Pending |
| 23 | G08-C1 | Build runtime magnetic H from existing h | Pending |
| 24 | G08-C2–C3 | Share magnetic model preparation across gate/core | Pending |
| 25 | G08-C6 | Share magnetic prediction/projection rotation | Pending |
| 26 | G08-C5 | Magnetic h/u finite check subsumption | Pending |
| 27 | G09-C1 | Direct constant selector Jacobians | Pending |
| 28 | G09-C3 | Reuse height local H | Pending |

Three separate numerical proposals are sparse covariance propagation (G04-C9),
block covariance reset (G05-C9) and selector-specific PHt/S (G09-C8). Their
disposition belongs to DC-08; no numerical rewrite is presumed equivalent.

## Evidence and acceptance

Local commands, logs and baseline snapshots live in the ignored
`output/validation-cleanup-20261115/` directory. Each accepted batch records its
tests and limitations below. Existing mathematical Lean statements are unchanged
unless explicitly recorded. A retained candidate must explain the contract or
numerical reason; a missing proof is not a pass.

## Batch records

Implementation has not started. No new proof or performance claim is made by
this plan.
