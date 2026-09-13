/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.RotationVector

/-!
# Right Jacobian formulas

Joan Sola, "Quaternion kinematics for the error-state Kalman filter", (183).
The exact Jacobian and the production small-angle polynomial are distinct.
`RotationDifferential` establishes the exact map's differential meaning.
-/

namespace FormalESKF.SO3

noncomputable section

open Matrix

def outer (v w : Vector3 ℝ) : Matrix3 ℝ := fun i j => v i * w j

@[simp] theorem hat_zero : hat (0 : Vector3 ℝ) = 0 := by
  ext i j
  fin_cases i <;> fin_cases j <;> simp [hat]

theorem hat_squared (v : Vector3 ℝ) :
    hat v * hat v = outer v v - vectorNormSquared v • (1 : Matrix3 ℝ) := by
  ext i j
  fin_cases i <;> fin_cases j <;>
    simp [hat, outer, vectorNormSquared, Matrix.mul_apply, dotProduct, Fin.sum_univ_succ] <;> ring

theorem hat_self (v : Vector3 ℝ) : hat v *ᵥ v = 0 := by
  rw [hat_mulVec]
  funext i
  fin_cases i <;> simp [cross] <;> ring

theorem cross_antisymm (v w : Vector3 ℝ) : cross v w = -cross w v := by
  funext i
  fin_cases i <;> simp [cross] <;> ring

theorem hat_scale (a : ℝ) (v : Vector3 ℝ) : hat (vectorScale a v) = a • hat v := by
  ext i j
  fin_cases i <;> fin_cases j <;> simp [hat, vectorScale]

def rightJacobian (phi : Vector3 ℝ) : Matrix3 ℝ :=
  let theta := vectorNorm phi
  if theta = 0 then 1
  else 1 - ((1 - Real.cos theta) / theta ^ 2) • hat phi +
    ((theta - Real.sin theta) / theta ^ 3) • (hat phi * hat phi)

def rightJacobianFirstOrder (phi : Vector3 ℝ) : Matrix3 ℝ :=
  1 - hat (vectorScale (1 / 2) phi)

def rightJacobianTaylorA (s : ℝ) : ℝ :=
  1 / 2 - s * (1 / 24 - s * (1 / 720 - s / 40320))

def rightJacobianTaylorB (s : ℝ) : ℝ :=
  1 / 6 - s * (1 / 120 - s * (1 / 5040 - s / 362880))

def rightJacobianTaylor (phi : Vector3 ℝ) : Matrix3 ℝ :=
  let s := vectorNormSquared phi
  let a := rightJacobianTaylorA s
  let b := rightJacobianTaylorB s
  (1 - s * b) • 1 + b • outer phi phi - a • hat phi

@[simp] theorem rightJacobian_zero : rightJacobian 0 = 1 := by simp [rightJacobian]

@[simp] theorem rightJacobianFirstOrder_zero : rightJacobianFirstOrder 0 = 1 := by
  simp [rightJacobianFirstOrder]

theorem rightJacobianTaylor_polynomial (phi : Vector3 ℝ) :
    rightJacobianTaylor phi = 1 - rightJacobianTaylorA (vectorNormSquared phi) • hat phi +
      rightJacobianTaylorB (vectorNormSquared phi) • (hat phi * hat phi) := by
  rw [hat_squared]
  ext i j
  simp [rightJacobianTaylor, Matrix.sub_apply, Matrix.add_apply, Matrix.smul_apply]
  ring

@[simp] theorem rightJacobianTaylor_zero : rightJacobianTaylor 0 = 1 := by
  rw [rightJacobianTaylor_polynomial]
  simp

theorem rightJacobian_axis {phi : Vector3 ℝ} (hn : vectorNorm phi ≠ 0) :
    let theta := vectorNorm phi
    let u := vectorScale (1 / theta) phi
    rightJacobian phi = (Real.sin theta / theta) • 1 +
      (1 - Real.sin theta / theta) • outer u u -
      ((1 - Real.cos theta) / theta) • hat u := by
  dsimp
  rw [rightJacobian, if_neg hn, hat_squared, hat_scale]
  have hs := vectorNorm_squared phi
  ext i j
  simp [outer, vectorScale, Matrix.sub_apply, Matrix.add_apply, Matrix.smul_apply]
  rw [← hs]
  field_simp
  ring

/-- The non-small production branch uses half angles to evaluate the same
exact axis formula; rounding and transcendental accuracy remain separate. -/
theorem rightJacobian_halfAngle {phi : Vector3 ℝ} (hn : vectorNorm phi ≠ 0) :
    let theta := vectorNorm phi
    let u := vectorScale (1 / theta) phi
    rightJacobian phi = (2 * Real.sin (theta / 2) * Real.cos (theta / 2) / theta) • 1 +
      (1 - 2 * Real.sin (theta / 2) * Real.cos (theta / 2) / theta) • outer u u -
      (2 * Real.sin (theta / 2) ^ 2 / theta) • hat u := by
  rw [rightJacobian_axis hn]
  have ht : vectorNorm phi = 2 * (vectorNorm phi / 2) := by ring
  have hs : Real.sin (vectorNorm phi) =
      2 * Real.sin (vectorNorm phi / 2) * Real.cos (vectorNorm phi / 2) := by
    conv_lhs => rw [ht, Real.sin_two_mul]
  have hc : 1 - Real.cos (vectorNorm phi) = 2 * Real.sin (vectorNorm phi / 2) ^ 2 := by
    conv_lhs => rw [ht, Real.cos_two_mul]
    nlinarith [Real.sin_sq_add_cos_sq (vectorNorm phi / 2)]
  dsimp
  rw [hs, hc]

theorem rightJacobian_preserves_axis (phi : Vector3 ℝ) : rightJacobian phi *ᵥ phi = phi := by
  dsimp only [rightJacobian]
  split_ifs
  · simp
  · simp [Matrix.add_mulVec, Matrix.sub_mulVec, Matrix.smul_mulVec,
      ← Matrix.mulVec_mulVec, hat_self]

end

end FormalESKF.SO3
