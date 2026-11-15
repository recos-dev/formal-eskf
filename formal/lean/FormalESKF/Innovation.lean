/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import Mathlib.Data.Real.Basic
import Mathlib.Tactic

/-! E-INNOV's exact-real interpretation. The implementation deliberately gates
the computed IEEE ratio, which can differ near a threshold or underflow.
These theorems do not identify the floating-point decision with the real one.
Status, operation order and atomic diagnostics are checked on C++ by ESBMC. -/

namespace FormalESKF.ESKF

noncomputable section

def innovationRatio (r variance sigma : ℝ) : ℝ :=
  (r * r) / ((sigma * sigma) * variance)

def innovationRejected {n : ℕ} (r variance : Fin n → ℝ) (sigma : ℝ) : Prop :=
  ∃ i, 1 < innovationRatio (r i) (variance i) sigma

theorem innovationRatio_nonneg (r : ℝ) {variance sigma : ℝ}
    (hv : 0 < variance) (hs : 0 < sigma) : 0 ≤ innovationRatio r variance sigma :=
  div_nonneg (mul_self_nonneg r) (le_of_lt (mul_pos (mul_pos hs hs) hv))

theorem innovationRatio_le_one_iff (r : ℝ) {variance sigma : ℝ}
    (hv : 0 < variance) (hs : 0 < sigma) :
    innovationRatio r variance sigma ≤ 1 ↔ r * r ≤ (sigma * sigma) * variance := by
  unfold innovationRatio
  rw [div_le_iff₀ (mul_pos (mul_pos hs hs) hv), one_mul]

theorem innovationRatio_boundary {r variance sigma : ℝ}
    (hv : 0 < variance) (hs : 0 < sigma)
    (h : r * r = (sigma * sigma) * variance) :
    innovationRatio r variance sigma = 1 := by
  unfold innovationRatio
  rw [h, div_self (ne_of_gt (mul_pos (mul_pos hs hs) hv))]

theorem innovationRatio_neg (r variance sigma : ℝ) :
    innovationRatio (-r) variance sigma = innovationRatio r variance sigma := by
  simp [innovationRatio]

theorem innovationRejected_iff {n : ℕ} (r variance : Fin n → ℝ) (sigma : ℝ) :
    ¬ innovationRejected r variance sigma ↔
      ∀ i, innovationRatio (r i) (variance i) sigma ≤ 1 := by
  simp [innovationRejected]

theorem innovationAccepted_boundary {n : ℕ} (r variance : Fin n → ℝ) (sigma : ℝ)
    (h : ∀ i, innovationRatio (r i) (variance i) sigma = 1) :
    ¬ innovationRejected r variance sigma := by
  rw [innovationRejected_iff]
  intro i
  exact le_of_eq (h i)

end

end FormalESKF.ESKF
