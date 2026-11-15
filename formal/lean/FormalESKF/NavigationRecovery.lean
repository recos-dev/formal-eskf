/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/
import FormalESKF.Innovation
import Mathlib.Analysis.SpecialFunctions.Sqrt

/-! Exact-real vertical innovation clipping. The source checks IEEE arithmetic
and retains raw diagnostics; these theorems do not authorize re-gating a rounded
boundary or establish sensor/source permission. -/
namespace FormalESKF.ESKF
noncomputable section

def boundedInnovation (r bound : ℝ) : ℝ :=
  if r < -bound then -bound else if bound < r then bound else r

theorem boundedInnovation_bounds (r : ℝ) {bound : ℝ} (h : 0 ≤ bound) :
    -bound ≤ boundedInnovation r bound ∧ boundedInnovation r bound ≤ bound := by
  unfold boundedInnovation
  split_ifs <;> constructor <;> linarith

theorem boundedInnovation_unchanged {r bound : ℝ} (hl : -bound ≤ r) (hu : r ≤ bound) :
    boundedInnovation r bound = r := by
  simp [boundedInnovation, not_lt.mpr hl, not_lt.mpr hu]

theorem boundedInnovation_sign (r : ℝ) {bound : ℝ} (h : 0 ≤ bound) :
    (0 ≤ r → 0 ≤ boundedInnovation r bound) ∧ (r ≤ 0 → boundedInnovation r bound ≤ 0) := by
  unfold boundedInnovation
  split_ifs <;> constructor <;> intro <;> linarith

theorem boundedInnovation_idempotent (r : ℝ) {bound : ℝ} (h : 0 ≤ bound) :
    boundedInnovation (boundedInnovation r bound) bound = boundedInnovation r bound := by
  exact boundedInnovation_unchanged (boundedInnovation_bounds r h).1 (boundedInnovation_bounds r h).2

theorem boundedInnovation_gate (r : ℝ) {variance sigma : ℝ} (hv : 0 < variance) (hs : 0 < sigma) :
    innovationRatio (boundedInnovation r (sigma * Real.sqrt variance)) variance sigma ≤ 1 := by
  rw [innovationRatio_le_one_iff _ hv hs]
  have hb : 0 ≤ sigma * Real.sqrt variance := mul_nonneg hs.le (Real.sqrt_nonneg _)
  obtain ⟨hl, hu⟩ := boundedInnovation_bounds r hb
  have hproduct := mul_nonneg (sub_nonneg.mpr hu) (by linarith :
    0 ≤ sigma * Real.sqrt variance + boundedInnovation r (sigma * Real.sqrt variance))
  have hsquare : (sigma * Real.sqrt variance) * (sigma * Real.sqrt variance) = sigma * sigma * variance := by
    calc
      _ = (sigma * sigma) * (Real.sqrt variance * Real.sqrt variance) := by ring
      _ = _ := by rw [Real.mul_self_sqrt hv.le]
  nlinarith

def limitInnovationAxis {n : ℕ} (r : Fin n → ℝ) (axis : Fin n) (sigma variance : ℝ) : Fin n → ℝ :=
  Function.update r axis (boundedInnovation (r axis) (sigma * Real.sqrt variance))

theorem limitInnovationAxis_other {n : ℕ} (r : Fin n → ℝ) (axis j : Fin n) (sigma variance : ℝ)
    (h : j ≠ axis) : limitInnovationAxis r axis sigma variance j = r j := by
  simp [limitInnovationAxis, h]

theorem limitInnovationAxis_selected {n : ℕ} (r : Fin n → ℝ) (axis : Fin n) {sigma variance : ℝ}
    (hv : 0 < variance) (hs : 0 < sigma) :
    innovationRatio (limitInnovationAxis r axis sigma variance axis) variance sigma ≤ 1 := by
  simpa [limitInnovationAxis] using boundedInnovation_gate (r axis) hv hs

end
end FormalESKF.ESKF
