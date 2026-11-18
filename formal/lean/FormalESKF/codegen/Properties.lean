/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.codegen.Entries

namespace Codegen
noncomputable section
open Matrix FormalESKF.ESKF

theorem upper_symmetric (P : M15) : (upper P)ᵀ = upper P := by
  ext i j
  by_cases hij : i ≤ j
  · by_cases hji : j ≤ i
    · have h : i = j := le_antisymm hij hji
      subst j
      rfl
    · simp [upper, hij, hji]
  · have hji : j ≤ i := le_of_lt (lt_of_not_ge hij)
    simp [upper, hij, hji]

theorem upper_eq_self {P : M15} (hP : Pᵀ = P) : upper P = P := by
  ext i j
  dsimp [upper]
  split_ifs
  · rfl
  · exact congrArg (fun M : M15 => M i j) hP

theorem reference_symmetric (dt : ℝ) (A B T R : M3) (P : M15) (Q : Fin 12 → ℝ) :
    (reference dt A B T R P Q)ᵀ = reference dt A B T R P Q := by
  change (sandwich (transition dt A B T) (upper P) +
    sandwich (injection R) (Matrix.diagonal Q))ᵀ = _
  rw [Matrix.transpose_add, sandwich_symmetric _ (upper_symmetric P),
    sandwich_symmetric _ (Matrix.diagonal_transpose _)]
  rfl

/-- Full real identity, for every symmetric prior and arbitrary block values. -/
theorem generatedMatrix_eq_covariance (dt : ℝ) (A B T R : M3) {P : M15} (Q : Fin 12 → ℝ)
    (hP : Pᵀ = P) :
    generatedMatrix dt A B T R P Q =
      transition dt A B T * P * (transition dt A B T)ᵀ +
      injection R * Matrix.diagonal Q * (injection R)ᵀ := by
  rw [generatedMatrix_eq_reference _ _ _ _ _ _ _ (reference_symmetric _ _ _ _ _ _ _)]
  simp only [reference, transported, noise, upper_eq_self hP]

/-- Mirroring gives symmetry even when the unused lower input triangle disagrees. -/
theorem generatedMatrix_symmetric (dt : ℝ) (A B T R : M3) (P : M15) (Q : Fin 12 → ℝ) :
    (generatedMatrix dt A B T R P Q)ᵀ = generatedMatrix dt A B T R P Q := by
  rw [generatedMatrix_eq_reference _ _ _ _ _ _ _ (reference_symmetric _ _ _ _ _ _ _)]
  exact reference_symmetric _ _ _ _ _ _ _

/-- Exact real PSD only. Floating-point mirroring does not establish PSD. -/
theorem generatedMatrix_posSemidef (dt : ℝ) (A B T R : M3) {P : M15} (Q : Fin 12 → ℝ)
    (hP : P.PosSemidef) (hQ : ∀ i, 0 ≤ Q i) :
    (generatedMatrix dt A B T R P Q).PosSemidef := by
  rw [generatedMatrix_eq_covariance _ _ _ _ _ _ (posSemidef_symmetric hP)]
  exact (sandwich_posSemidef _ hP).add
    (sandwich_posSemidef _ (Matrix.PosSemidef.diagonal hQ))

/-- The dense off-diagonal average is redundant over exact symmetric values. -/
theorem generatedMatrix_eq_finished_reference (dt : ℝ) (A B T R : M3) (P : M15) (Q : Fin 12 → ℝ) :
    generatedMatrix dt A B T R P Q = finishCovariance (reference dt A B T R P Q) := by
  have hs := reference_symmetric dt A B T R P Q
  exact (generatedMatrix_eq_reference dt A B T R P Q hs).trans
    (finishCovariance_of_symmetric hs).symm

#print axioms generatedMatrix_eq_covariance
#print axioms generatedMatrix_symmetric
#print axioms generatedMatrix_posSemidef
end
end Codegen
