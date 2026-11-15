/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.Covariance
import FormalESKF.State

/-! Navigation reinitialization in `runtime/navigation.hpp`, distinct from
post-injection error-coordinate reset. Exact real covariance/state statements;
IEEE validation, aliasing, overflow and caller permission are separate. -/
namespace FormalESKF.ESKF
noncomputable section
open Matrix SO3

/-- Selected coordinates lose all cross-covariance and receive the supplied
variance. Other coefficients are retained exactly. -/
def navigationCovariance {n : ℕ} (selected : Fin n → Bool) (variance : Fin n → ℝ)
    (P : Covariance n) : Covariance n :=
  fun i j => if i = j then (if selected i then variance i else P i i)
    else if selected i || selected j then 0 else P i j

def navigationRetain {n : ℕ} (selected : Fin n → Bool) : Covariance n :=
  diagonal (fun i => if selected i then 0 else 1)

theorem navigationCovariance_decomposition {n : ℕ} (selected : Fin n → Bool)
    (variance : Fin n → ℝ) (P : Covariance n) :
    navigationCovariance selected variance P =
      sandwich (navigationRetain selected) P + diagonal (fun i => if selected i then variance i else 0) := by
  ext i j
  by_cases hi : selected i = true <;> by_cases hj : selected j = true <;> by_cases hij : i = j <;>
    simp_all [navigationCovariance, sandwich, navigationRetain, diagonal_mul, mul_diagonal]

theorem navigationCovariance_posSemidef {n : ℕ} (selected : Fin n → Bool)
    (variance : Fin n → ℝ) {P : Covariance n} (hP : P.PosSemidef)
    (hv : ∀ i, selected i = true → 0 ≤ variance i) :
    (navigationCovariance selected variance P).PosSemidef := by
  rw [navigationCovariance_decomposition]
  apply (sandwich_posSemidef _ hP).add
  apply Matrix.PosSemidef.diagonal
  intro i
  change 0 ≤ if selected i then variance i else 0
  split_ifs with hi
  · exact hv i hi
  · exact le_rfl

theorem navigationCovariance_untouched {n : ℕ} (selected : Fin n → Bool)
    (variance : Fin n → ℝ) (P : Covariance n) (i j : Fin n)
    (hi : selected i = false) (hj : selected j = false) :
    navigationCovariance selected variance P i j = P i j := by
  by_cases h : i = j <;> simp_all [navigationCovariance]

theorem navigationCovariance_decoupled {n : ℕ} (selected : Fin n → Bool)
    (variance : Fin n → ℝ) (P : Covariance n) (i j : Fin n)
    (h : i ≠ j) (hs : selected i = true ∨ selected j = true) :
    navigationCovariance selected variance P i j = 0 := by
  rcases hs with hi | hj <;> simp [navigationCovariance, *]

/-- Exact decimal floor; C++ uses its scalar-format encoding of 0.01 squared. -/
def navigationVariance (variance : ℝ) : ℝ := max variance ((1 / 100 : ℝ) ^ 2)

theorem navigationVariance_positive (variance : ℝ) : 0 < navigationVariance variance := by
  exact lt_of_lt_of_le (by norm_num) (le_max_right _ _)

def resetHorizontalPositionCovariance (P : Covariance 15) (variance : Fin 2 → ℝ) : Covariance 15 :=
  navigationCovariance (fun i => decide (i.val < 2)) (fun _ => navigationVariance (variance 0 + variance 1)) P

def resetVerticalPositionCovariance (P : Covariance 15) (variance : ℝ) : Covariance 15 :=
  navigationCovariance (fun i => decide (i.val = 2)) (fun _ => navigationVariance variance) P

def resetVerticalVelocityCovariance (P : Covariance 15) (variance : ℝ) : Covariance 15 :=
  navigationCovariance (fun i => decide (i.val = 2 ∨ i.val = 5))
    (fun i => if i.val = 2 then P i i else navigationVariance variance) P

def resetVelocityCovariance (P : Covariance 15) (variance : Fin 3 → ℝ) : Covariance 15 :=
  navigationCovariance (fun i => decide (i.val < 6))
    (fun i => if i.val < 3 then P i i else navigationVariance (if i.val = 3 then variance 0 else if i.val = 4 then variance 1 else variance 2)) P

theorem resetHorizontalPositionCovariance_posSemidef {P : Covariance 15} (hP : P.PosSemidef)
    (variance : Fin 2 → ℝ) : (resetHorizontalPositionCovariance P variance).PosSemidef := by
  exact navigationCovariance_posSemidef _ _ hP (fun _ _ => (navigationVariance_positive _).le)

theorem resetVerticalPositionCovariance_posSemidef {P : Covariance 15} (hP : P.PosSemidef)
    (variance : ℝ) : (resetVerticalPositionCovariance P variance).PosSemidef := by
  exact navigationCovariance_posSemidef _ _ hP (fun _ _ => (navigationVariance_positive _).le)

theorem resetVerticalVelocityCovariance_posSemidef {P : Covariance 15} (hP : P.PosSemidef)
    (variance : ℝ) : (resetVerticalVelocityCovariance P variance).PosSemidef := by
  apply navigationCovariance_posSemidef _ _ hP
  intro i _
  split_ifs <;> first | exact hP.diag_nonneg | exact (navigationVariance_positive _).le

theorem resetVelocityCovariance_posSemidef {P : Covariance 15} (hP : P.PosSemidef)
    (variance : Fin 3 → ℝ) : (resetVelocityCovariance P variance).PosSemidef := by
  apply navigationCovariance_posSemidef _ _ hP
  intro i _
  split_ifs <;> first | exact hP.diag_nonneg | exact (navigationVariance_positive _).le

/-- Down velocity recovery retains the Down position variance. -/
theorem resetVerticalVelocityCovariance_position (P : Covariance 15) (variance : ℝ) :
    resetVerticalVelocityCovariance P variance 2 2 = P 2 2 := by
  simp [resetVerticalVelocityCovariance, navigationCovariance]

/-- Full velocity recovery retains each position variance but removes its correlations. -/
theorem resetVelocityCovariance_position (P : Covariance 15) (variance : Fin 3 → ℝ) (i : Fin 3) :
    resetVelocityCovariance P variance (Fin.castAdd 12 i) (Fin.castAdd 12 i) =
      P (Fin.castAdd 12 i) (Fin.castAdd 12 i) := by
  simp [resetVelocityCovariance, navigationCovariance, i.isLt]

theorem resetHorizontalPositionCovariance_variance (P : Covariance 15) (variance : Fin 2 → ℝ) (i : Fin 2) :
    resetHorizontalPositionCovariance P variance (Fin.castAdd 13 i) (Fin.castAdd 13 i) =
      navigationVariance (variance 0 + variance 1) := by
  simp [resetHorizontalPositionCovariance, navigationCovariance, i.isLt]

theorem resetVerticalPositionCovariance_variance (P : Covariance 15) (variance : ℝ) :
    resetVerticalPositionCovariance P variance 2 2 = navigationVariance variance := by
  simp [resetVerticalPositionCovariance, navigationCovariance]

theorem resetVerticalVelocityCovariance_variance (P : Covariance 15) (variance : ℝ) :
    resetVerticalVelocityCovariance P variance 5 5 = navigationVariance variance := by
  simp [resetVerticalVelocityCovariance, navigationCovariance]

theorem resetVelocityCovariance_variance (P : Covariance 15) (variance : Fin 3 → ℝ) (i : Fin 3) :
    resetVelocityCovariance P variance ⟨i.val + 3, by omega⟩ ⟨i.val + 3, by omega⟩ = navigationVariance (variance i) := by
  fin_cases i <;> simp [resetVelocityCovariance, navigationCovariance]

def resetHorizontalPositionState (s : InsState) (position : Fin 2 → ℝ) : InsState :=
  { s with p_n := ![position 0, position 1, s.p_n 2] }
def resetVerticalPositionState (s : InsState) (position : ℝ) : InsState :=
  { s with p_n := ![s.p_n 0, s.p_n 1, position] }
def resetVerticalVelocityState (s : InsState) (velocity : ℝ) : InsState :=
  { s with v_n := ![s.v_n 0, s.v_n 1, velocity] }
def resetVelocityState (s : InsState) (velocity : Vector3 ℝ) : InsState :=
  { s with v_n := velocity }

theorem navigationState_preserves_attitude_biases (s : InsState) (p : Fin 2 → ℝ) (d w : ℝ) (v : Vector3 ℝ) :
    ∀ result ∈ [resetHorizontalPositionState s p, resetVerticalPositionState s d,
                resetVerticalVelocityState s w, resetVelocityState s v],
      result.q_nb = s.q_nb ∧ result.b_a = s.b_a ∧ result.b_g = s.b_g := by
  intro result hr
  simp only [List.mem_cons, List.not_mem_nil, or_false] at hr
  rcases hr with rfl | rfl | rfl | rfl <;> exact ⟨rfl, rfl, rfl⟩

end
end FormalESKF.ESKF
