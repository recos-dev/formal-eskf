/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.Covariance

/-!
# Exact meaning of the Cholesky substitution recurrences

These are real-arithmetic loop-result contracts, not an IEEE residual bound
or a proof that floating-point LLT succeeds for every mathematically SPD input.
The factorization relation and nonzero diagonal are explicit premises.
-/

namespace FormalESKF.ESKF

noncomputable section
open Matrix

def choleskyPrefix {n : ℕ} (L : Covariance n) (i j : Fin n) : ℝ :=
  ∑ k, if k < j then L i k * L j k else 0

theorem cholesky_product_entry {n : ℕ} {L : Covariance n}
    (hLower : ∀ i k, i < k → L i k = 0) (i j : Fin n) :
    (L * Lᵀ) i j = choleskyPrefix L i j + L i j * L j j := by
  change (∑ k, L i k * L j k) = _
  calc
    _ = ∑ k, ((if k < j then L i k * L j k else 0) +
        (if k = j then L i j * L j j else 0)) := by
      apply Finset.sum_congr rfl
      intro k _
      rcases lt_trichotomy k j with h | rfl | h
      · simp [h, ne_of_lt h]
      · simp
      · simp [not_lt_of_gt h, ne_of_gt h, hLower j k h]
    _ = _ := by simp [Finset.sum_add_distrib, choleskyPrefix]

/-- The diagonal-square and off-diagonal division recurrences reconstruct
the entire symmetric system, not only its diagonal. Here L is the zero-extended
lower triangle; a C++ workspace's unused upper entries need not be zero. -/
theorem choleskyRecurrences_factor {n : ℕ} {S L : Covariance n}
    (hSymm : ∀ i j, S i j = S j i)
    (hLower : ∀ i k, i < k → L i k = 0) (hdiag : ∀ i, L i i ≠ 0)
    (hsquare : ∀ j, L j j ^ 2 = S j j - choleskyPrefix L j j)
    (hoff : ∀ i j, j < i → L i j = (S i j - choleskyPrefix L i j) / L j j) :
    S = L * Lᵀ := by
  have hlower : ∀ i j, j ≤ i → (L * Lᵀ) i j = S i j := by
    intro i j hij
    rw [cholesky_product_entry hLower]
    rcases lt_or_eq_of_le hij with h | rfl
    · have hentry := (eq_div_iff (hdiag j)).mp (hoff i j h)
      linarith
    · nlinarith [hsquare j]
  ext i j
  by_cases h : j ≤ i
  · exact (hlower i j h).symm
  · have hcomm : (L * Lᵀ) i j = (L * Lᵀ) j i := by
      simp only [Matrix.mul_apply, Matrix.transpose_apply]
      apply Finset.sum_congr rfl
      intro k _
      exact mul_comm _ _
    rw [hcomm, hlower j i (le_of_lt (lt_of_not_ge h))]
    exact hSymm i j

theorem cholesky_positive_pivot {x : ℝ} (hx : 0 < x) :
    Real.sqrt x ≠ 0 ∧ (Real.sqrt x) ^ 2 = x :=
  ⟨ne_of_gt (Real.sqrt_pos.mpr hx), Real.sq_sqrt hx.le⟩

def forwardSubstitutionStep {n r : ℕ} (L : Covariance n)
    (B Y : Matrix (Fin n) (Fin r) ℝ) (i : Fin n) (j : Fin r) : ℝ :=
  (B i j - ∑ k, if k < i then L i k * Y k j else 0) / L i i

def backwardSubstitutionStep {n r : ℕ} (L : Covariance n)
    (Y X : Matrix (Fin n) (Fin r) ℝ) (i : Fin n) (j : Fin r) : ℝ :=
  (Y i j - ∑ k, if i < k then L k i * X k j else 0) / L i i

theorem forwardSubstitution_equation {n r : ℕ} {L : Covariance n}
    (hLower : ∀ i k, i < k → L i k = 0) (hdiag : ∀ i, L i i ≠ 0)
    {B Y : Matrix (Fin n) (Fin r) ℝ}
    (hstep : ∀ i j, Y i j = forwardSubstitutionStep L B Y i j) : L * Y = B := by
  ext i j
  have hsum : ∑ k, L i k * Y k j =
      (∑ k, if k < i then L i k * Y k j else 0) + L i i * Y i j := by
    calc
      _ = ∑ k, ((if k < i then L i k * Y k j else 0) +
          (if k = i then L i i * Y i j else 0)) := by
        apply Finset.sum_congr rfl
        intro k _
        rcases lt_trichotomy k i with h | rfl | h
        · simp [h, ne_of_lt h]
        · simp
        · simp [not_lt_of_gt h, ne_of_gt h, hLower i k h]
      _ = _ := by simp [Finset.sum_add_distrib]
  have h := (eq_div_iff (hdiag i)).mp (hstep i j)
  change Y i j * L i i = _ at h
  rw [Matrix.mul_apply, hsum]
  linarith [mul_comm (Y i j) (L i i)]

theorem backwardSubstitution_equation {n r : ℕ} {L : Covariance n}
    (hLower : ∀ i k, i < k → L i k = 0) (hdiag : ∀ i, L i i ≠ 0)
    {Y X : Matrix (Fin n) (Fin r) ℝ}
    (hstep : ∀ i j, X i j = backwardSubstitutionStep L Y X i j) : Lᵀ * X = Y := by
  ext i j
  have hsum : ∑ k, L k i * X k j =
      (∑ k, if i < k then L k i * X k j else 0) + L i i * X i j := by
    calc
      _ = ∑ k, ((if i < k then L k i * X k j else 0) +
          (if k = i then L i i * X i j else 0)) := by
        apply Finset.sum_congr rfl
        intro k _
        rcases lt_trichotomy k i with h | rfl | h
        · simp [not_lt_of_gt h, ne_of_lt h, hLower k i h]
        · simp
        · simp [h, ne_of_gt h]
      _ = _ := by simp [Finset.sum_add_distrib]
  have h := (eq_div_iff (hdiag i)).mp (hstep i j)
  change X i j * L i i = _ at h
  change (∑ k, L k i * X k j) = Y i j
  rw [hsum]
  linarith [mul_comm (X i j) (L i i)]

theorem choleskySubstitutions_solve {n r : ℕ} {S L : Covariance n}
    (hfactor : S = L * Lᵀ) (hLower : ∀ i k, i < k → L i k = 0)
    (hdiag : ∀ i, L i i ≠ 0) {B Y X : Matrix (Fin n) (Fin r) ℝ}
    (hforward : ∀ i j, Y i j = forwardSubstitutionStep L B Y i j)
    (hbackward : ∀ i j, X i j = backwardSubstitutionStep L Y X i j) : S * X = B := by
  rw [hfactor, Matrix.mul_assoc, backwardSubstitution_equation hLower hdiag hbackward]
  exact forwardSubstitution_equation hLower hdiag hforward

theorem choleskySubstitutions_rightSolve {n r : ℕ} {S L : Covariance n}
    (hS : S.PosDef) (hfactor : S = L * Lᵀ)
    (hLower : ∀ i k, i < k → L i k = 0) (hdiag : ∀ i, L i i ≠ 0)
    {B : Matrix (Fin r) (Fin n) ℝ} {Y X : Matrix (Fin n) (Fin r) ℝ}
    (hforward : ∀ i j, Y i j = forwardSubstitutionStep L Bᵀ Y i j)
    (hbackward : ∀ i j, X i j = backwardSubstitutionStep L Y X i j) :
    Xᵀ = rightSolve B S := by
  apply rightSolve_unique hS
  have h := congrArg Matrix.transpose
    (choleskySubstitutions_solve hfactor hLower hdiag hforward hbackward)
  simpa [Matrix.transpose_mul, posSemidef_symmetric hS.posSemidef] using h

end
end FormalESKF.ESKF
