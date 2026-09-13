/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import Mathlib.LinearAlgebra.Matrix.PosDef
import Mathlib.Algebra.Order.Star.Real
import Mathlib.Tactic.Abel

/-!
# Exact covariance and correction algebra

Dimension-independent real arithmetic for `covariance_prediction.hpp`,
`injection.hpp`, and `correction.hpp`. PSD/PD are mathematical input premises,
not conclusions of the C++ diagonal checks. No IEEE-754 PSD guarantee or
correctness of the executable Cholesky factorization is asserted here.
-/

namespace FormalESKF.ESKF

noncomputable section

open Matrix

abbrev Covariance (n : ℕ) := Matrix (Fin n) (Fin n) ℝ

def sandwich {m n : ℕ} (A : Matrix (Fin m) (Fin n) ℝ) (P : Covariance n) :
    Covariance m := A * P * Aᵀ

theorem sandwich_posSemidef {m n : ℕ} (A : Matrix (Fin m) (Fin n) ℝ)
    {P : Covariance n} (hP : P.PosSemidef) : (sandwich A P).PosSemidef := by
  simpa [sandwich] using hP.mul_mul_conjTranspose_same A

theorem posSemidef_symmetric {n : ℕ} {P : Covariance n} (hP : P.PosSemidef) :
    Pᵀ = P := by
  exact hP.isHermitian

theorem sandwich_symmetric {m n : ℕ} (A : Matrix (Fin m) (Fin n) ℝ)
    {P : Covariance n} (hP : Pᵀ = P) : (sandwich A P)ᵀ = sandwich A P := by
  simp [sandwich, Matrix.transpose_mul, hP, Matrix.mul_assoc]

/-- Off-diagonal cleanup in `covariance.hpp`, without changing the diagonal. -/
def finishCovariance {n : ℕ} (P : Covariance n) : Covariance n :=
  fun i j => if i = j then P i j else (1 / 2) * P i j + (1 / 2) * P j i

theorem finishCovariance_symmetric {n : ℕ} (P : Covariance n) :
    (finishCovariance P)ᵀ = finishCovariance P := by
  ext i j
  by_cases h : i = j <;> simp [finishCovariance, h, eq_comm, add_comm]

theorem finishCovariance_of_symmetric {n : ℕ} {P : Covariance n} (hP : Pᵀ = P) :
    finishCovariance P = P := by
  ext i j
  have h : P j i = P i j := congrArg (fun M : Covariance n => M i j) hP
  simp only [finishCovariance, h]
  split_ifs <;> ring

/-- Cleanup is redundant over exact symmetric inputs, not a PSD repair. -/
theorem finishCovariance_of_posSemidef {n : ℕ} {P : Covariance n} (hP : P.PosSemidef) :
    finishCovariance P = P := finishCovariance_of_symmetric (posSemidef_symmetric hP)

def propagateCovariance {n k : ℕ} (F : Covariance n) (P : Covariance n)
    (B : Matrix (Fin n) (Fin k) ℝ) (Q : Covariance k) : Covariance n :=
  sandwich F P + sandwich B Q

theorem propagateCovariance_posSemidef {n k : ℕ} (F : Covariance n)
    (B : Matrix (Fin n) (Fin k) ℝ) {P : Covariance n} {Q : Covariance k}
    (hP : P.PosSemidef) (hQ : Q.PosSemidef) :
    (propagateCovariance F P B Q).PosSemidef :=
  (sandwich_posSemidef F hP).add (sandwich_posSemidef B hQ)

def innovationCovariance {n m : ℕ} (P : Covariance n)
    (H : Matrix (Fin m) (Fin n) ℝ) (V : Covariance m) : Covariance m :=
  H * (P * Hᵀ) + V

theorem innovationCovariance_posDef {n m : ℕ} {P : Covariance n}
    (H : Matrix (Fin m) (Fin n) ℝ) {V : Covariance m}
    (hP : P.PosSemidef) (hV : V.PosDef) :
    (innovationCovariance P H V).PosDef := by
  simpa [innovationCovariance, sandwich, Matrix.mul_assoc] using
    Matrix.PosDef.posSemidef_add (sandwich_posSemidef H hP) hV

/-- Mathematical characterization of the right solve, not an implementation
of matrix inversion. The C++ backend solves X*S=B without forming an inverse. -/
def rightSolve {n m : ℕ} (B : Matrix (Fin n) (Fin m) ℝ) (S : Covariance m) :=
  B * S⁻¹

theorem rightSolve_equation {n m : ℕ} (B : Matrix (Fin n) (Fin m) ℝ)
    {S : Covariance m} (hS : S.PosDef) : rightSolve B S * S = B := by
  exact Matrix.nonsing_inv_mul_cancel_right S B
    ((Matrix.isUnit_iff_isUnit_det S).mp hS.isUnit)

theorem rightSolve_unique {n m : ℕ} {B X : Matrix (Fin n) (Fin m) ℝ}
    {S : Covariance m} (hS : S.PosDef) (hX : X * S = B) : X = rightSolve B S := by
  have h := congrArg (fun M : Matrix (Fin n) (Fin m) ℝ => M * S⁻¹) hX
  simpa [rightSolve, Matrix.mul_assoc,
    Matrix.mul_nonsing_inv S ((Matrix.isUnit_iff_isUnit_det S).mp hS.isUnit)] using h

/-- The linalg wrapper can implement the right solve by transposing the
right-hand side and invoking an ordinary SPD left solve. -/
theorem rightSolve_transposed_equation {n m : ℕ} (B : Matrix (Fin n) (Fin m) ℝ)
    {S : Covariance m} (hS : S.PosDef) : S * (rightSolve B S)ᵀ = Bᵀ := by
  have h := congrArg Matrix.transpose (rightSolve_equation B hS)
  simpa [Matrix.transpose_mul, posSemidef_symmetric hS.posSemidef] using h

def kalmanGain {n m : ℕ} (P : Covariance n) (H : Matrix (Fin m) (Fin n) ℝ)
    (V : Covariance m) : Matrix (Fin n) (Fin m) ℝ :=
  rightSolve (P * Hᵀ) (innovationCovariance P H V)

theorem kalmanGain_equation {n m : ℕ} {P : Covariance n}
    (H : Matrix (Fin m) (Fin n) ℝ) {V : Covariance m}
    (hP : P.PosSemidef) (hV : V.PosDef) :
    kalmanGain P H V * innovationCovariance P H V = P * Hᵀ :=
  rightSolve_equation _ (innovationCovariance_posDef H hP hV)

def josephCovariance {n m : ℕ} (P : Covariance n)
    (H : Matrix (Fin m) (Fin n) ℝ) (V : Covariance m)
    (K : Matrix (Fin n) (Fin m) ℝ) : Covariance n :=
  sandwich (1 - K * H) P + sandwich K V

/-- Joseph PSD preservation does not require an optimal gain. -/
theorem josephCovariance_posSemidef {n m : ℕ} {P : Covariance n}
    (H : Matrix (Fin m) (Fin n) ℝ) {V : Covariance m}
    (K : Matrix (Fin n) (Fin m) ℝ) (hP : P.PosSemidef) (hV : V.PosSemidef) :
    (josephCovariance P H V K).PosSemidef :=
  (sandwich_posSemidef _ hP).add (sandwich_posSemidef _ hV)

theorem josephCovariance_expansion {n m : ℕ} (P : Covariance n)
    (H : Matrix (Fin m) (Fin n) ℝ) (V : Covariance m)
    (K : Matrix (Fin n) (Fin m) ℝ) :
    josephCovariance P H V K =
      P - K * H * P - P * Hᵀ * Kᵀ + K * innovationCovariance P H V * Kᵀ := by
  simp only [josephCovariance, sandwich, innovationCovariance, transpose_sub,
    transpose_one, transpose_mul, sub_mul, mul_sub, mul_one, one_mul,
    Matrix.mul_add, Matrix.add_mul, Matrix.mul_assoc]
  abel

/-- The simplified covariance equation follows from the right-solve equation.
It is an exact identity, not a recommendation to replace Joseph in C++. -/
theorem josephCovariance_eq_simplified {n m : ℕ} (P : Covariance n)
    (H : Matrix (Fin m) (Fin n) ℝ) (V : Covariance m)
    (K : Matrix (Fin n) (Fin m) ℝ)
    (hK : K * innovationCovariance P H V = P * Hᵀ) :
    josephCovariance P H V K = (1 - K * H) * P := by
  rw [josephCovariance_expansion, hK, sub_mul, one_mul]
  abel

theorem kalmanGain_joseph_eq_simplified {n m : ℕ} {P : Covariance n}
    (H : Matrix (Fin m) (Fin n) ℝ) {V : Covariance m}
    (hP : P.PosSemidef) (hV : V.PosDef) :
    josephCovariance P H V (kalmanGain P H V) = (1 - kalmanGain P H V * H) * P :=
  josephCovariance_eq_simplified P H V _ (kalmanGain_equation H hP hV)

/-- Information reduces covariance in the SAME pre-reset coordinates.
Comparing this directly with a covariance after nonlinear-coordinate reset
would compare different coordinate systems. -/
theorem kalmanGain_covariance_reduction {n m : ℕ} {P : Covariance n}
    (H : Matrix (Fin m) (Fin n) ℝ) {V : Covariance m}
    (hP : P.PosSemidef) (hV : V.PosDef) :
    P - josephCovariance P H V (kalmanGain P H V) =
      sandwich (P * Hᵀ) (innovationCovariance P H V)⁻¹ := by
  rw [kalmanGain_joseph_eq_simplified H hP hV]
  simp [sandwich, kalmanGain, rightSolve, transpose_mul, posSemidef_symmetric hP,
    Matrix.sub_mul, Matrix.mul_assoc]

theorem kalmanGain_covariance_reduction_posSemidef {n m : ℕ} {P : Covariance n}
    (H : Matrix (Fin m) (Fin n) ℝ) {V : Covariance m}
    (hP : P.PosSemidef) (hV : V.PosDef) :
    (P - josephCovariance P H V (kalmanGain P H V)).PosSemidef := by
  rw [kalmanGain_covariance_reduction H hP hV]
  exact sandwich_posSemidef _ (innovationCovariance_posDef H hP hV).inv.posSemidef

end

end FormalESKF.ESKF
