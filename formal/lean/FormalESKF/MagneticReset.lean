/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/
import FormalESKF.MagneticGain
import FormalESKF.MagneticHeading
import FormalESKF.NavigationReset
import Mathlib.Data.Matrix.Block

/-! Exact navigation-Down heading reset and covariance congruence. Attitude is
placed first in the block model; INS uses its explicit attitude offset in C++.
These statements do not prove IEEE PSD preservation or target libm accuracy. -/
namespace FormalESKF.ESKF
noncomputable section
open Matrix SO3 Quaternion

def magneticResetAttitude (q : Quaternion ℝ) (angle : ℝ) : Quaternion ℝ :=
  hamiltonMul (quaternionExp (vectorScale angle navigationDown)) q

theorem magneticResetAttitude_unit {q : Quaternion ℝ} (hq : HasUnitNorm q) (angle : ℝ) :
    HasUnitNorm (magneticResetAttitude q angle) :=
  (quaternionExp_hasUnitNorm _).hamiltonMul hq

theorem magneticResetAttitude_tilt (q : Quaternion ℝ) (angle : ℝ) :
    bodyDown (magneticResetAttitude q angle) = bodyDown q := by
  simp only [bodyDown, magneticResetAttitude, inverseRotate, rotationMatrix_hamiltonMul,
    transpose_mul]
  rw [← Matrix.mulVec_mulVec]
  change (rotationMatrix q)ᵀ *ᵥ
    (inverseRotate (quaternionExp (vectorScale angle navigationDown)) navigationDown) = _
  rw [inverseRotate_exp_parallel]

theorem magneticResetAttitude_normalized {q : Quaternion ℝ} (hq : HasUnitNorm q) (angle : ℝ) :
    Quaternion.normalize (magneticResetAttitude q angle) = magneticResetAttitude q angle := by
  have h : normSquared (magneticResetAttitude q angle) = 1 := magneticResetAttitude_unit hq angle
  rw [Quaternion.normalize, h]
  ext <;> simp [Quaternion.scale]

theorem quaternionExp_down (angle : ℝ) :
    quaternionExp (vectorScale angle navigationDown) =
      fromScalarVector (Real.cos (angle / 2)) (vectorScale (Real.sin (angle / 2)) navigationDown) := by
  have hn : vectorNorm navigationDown = 1 := by
    norm_num [vectorNorm, vectorNormSquared, navigationDown, dotProduct, Fin.sum_univ_succ]
  rw [← constantRateIncrement_eq_exp_signed, constantRateIncrement, hn]
  simp

def navigationYawMatrix (angle : ℝ) : Matrix3 ℝ :=
  !![Real.cos angle, -Real.sin angle, 0; Real.sin angle, Real.cos angle, 0; 0, 0, 1]

theorem rotationMatrix_exp_down (angle : ℝ) :
    rotationMatrix (quaternionExp (vectorScale angle navigationDown)) = navigationYawMatrix angle := by
  rw [quaternionExp_down]
  have hs := Real.sin_two_mul (angle / 2)
  have hc := Real.cos_two_mul' (angle / 2)
  have ht : 2 * (angle / 2) = angle := by ring
  rw [ht] at hs hc
  have hu := Real.sin_sq_add_cos_sq (angle / 2)
  ext i j
  fin_cases i <;> fin_cases j <;>
    simp [rotationMatrix, fromScalarVector, vectorScale, navigationDown, navigationYawMatrix] <;>
    nlinarith [hs, hc, hu]

def horizontalPolar (magnitude heading down : ℝ) : Vector3 ℝ :=
  ![magnitude * Real.cos heading, magnitude * Real.sin heading, down]

theorem navigationYawMatrix_polar (angle magnitude heading down : ℝ) :
    navigationYawMatrix angle *ᵥ horizontalPolar magnitude heading down =
      horizontalPolar magnitude (angle + heading) down := by
  ext i
  fin_cases i <;>
    simp [navigationYawMatrix, horizontalPolar, Matrix.mulVec, dotProduct, Fin.sum_univ_succ,
      Real.cos_add, Real.sin_add] <;> ring

theorem magneticHeadingWrap_trig (angle : ℝ) :
    Real.sin (magneticHeadingWrap angle) = Real.sin angle ∧
      Real.cos (magneticHeadingWrap angle) = Real.cos angle := by
  unfold magneticHeadingWrap
  split_ifs <;> simp

/-- A positive horizontal magnitude gives the reference heading without changing
Down or magnitude. Sensor truth and atan2 correspondence remain caller premises. -/
theorem magneticResetAttitude_alignment (q : Quaternion ℝ) (body : Vector3 ℝ)
    (magnitude measured reference down : ℝ)
    (h : rotate q body = horizontalPolar magnitude measured down) :
    rotate (magneticResetAttitude q (magneticHeadingWrap (reference - measured))) body =
      horizontalPolar magnitude reference down := by
  rw [magneticResetAttitude, rotate_hamiltonMul, h]
  simp only [rotate, rotationMatrix_exp_down]
  have hm : navigationYawMatrix (magneticHeadingWrap (reference - measured)) =
      navigationYawMatrix (reference - measured) := by
    simp [navigationYawMatrix, (magneticHeadingWrap_trig (reference - measured)).1,
      (magneticHeadingWrap_trig (reference - measured)).2]
  rw [hm, navigationYawMatrix_polar, sub_add_cancel]

def magneticResetTransform (before after : Matrix3 ℝ) : Matrix3 ℝ :=
  afterᵀ * (1 - axisProjector navigationDown) * before

def magneticResetAxis (after : Matrix3 ℝ) : Vector3 ℝ := afterᵀ *ᵥ navigationDown

theorem magneticTilt_removes_down :
    (1 - axisProjector navigationDown) *ᵥ navigationDown = 0 := by
  have h : dotProduct navigationDown navigationDown = 1 := by
    norm_num [navigationDown, dotProduct, Fin.sum_univ_succ]
  rw [Matrix.sub_mulVec, Matrix.one_mulVec, axisProjector_action, h]
  ext i
  simp

theorem magneticResetTransform_removes_yaw {q : Quaternion ℝ} (hq : HasUnitNorm q)
    (after : Matrix3 ℝ) :
    magneticResetTransform (rotationMatrix q) after *ᵥ bodyDown q = 0 := by
  have h : rotationMatrix q *ᵥ bodyDown q = navigationDown := by
    simp [bodyDown, inverseRotate, Matrix.mulVec_mulVec, rotationMatrix_mul_transpose_of_unit hq]
  rw [magneticResetTransform, ← Matrix.mulVec_mulVec, h, ← Matrix.mulVec_mulVec,
    magneticTilt_removes_down, Matrix.mulVec_zero]

theorem magneticResetAxis_unit {q : Quaternion ℝ} (hq : HasUnitNorm q) :
    vectorNormSquared (magneticResetAxis (rotationMatrix q)) = 1 := by
  exact bodyDown_unit hq

def magneticResetBlockTransform (n : ℕ) (A : Matrix3 ℝ) :
    Matrix (Fin 3 ⊕ Fin n) (Fin 3 ⊕ Fin n) ℝ :=
  fromBlocks A 0 0 1

def magneticResetBlockAxis (n : ℕ) (u : Vector3 ℝ) : (Fin 3 ⊕ Fin n) → ℝ :=
  Sum.elim u (fun _ => 0)

def magneticResetCovariance {n : ℕ} (A : Matrix3 ℝ) (u : Vector3 ℝ) (variance : ℝ)
    (P : Matrix (Fin 3 ⊕ Fin n) (Fin 3 ⊕ Fin n) ℝ) :=
  magneticResetBlockTransform n A * P * (magneticResetBlockTransform n A)ᵀ +
    navigationVariance variance • vecMulVec (magneticResetBlockAxis n u) (magneticResetBlockAxis n u)

theorem magneticResetCovariance_posSemidef {n : ℕ} (A : Matrix3 ℝ) (u : Vector3 ℝ)
    (variance : ℝ) {P : Matrix (Fin 3 ⊕ Fin n) (Fin 3 ⊕ Fin n) ℝ} (hP : P.PosSemidef) :
    (magneticResetCovariance A u variance P).PosSemidef := by
  apply Matrix.PosSemidef.add
  · simpa using hP.mul_mul_conjTranspose_same (magneticResetBlockTransform n A)
  · have h : (vecMulVec (magneticResetBlockAxis n u) (magneticResetBlockAxis n u)).PosSemidef := by
      simpa using posSemidef_vecMulVec_self_star (magneticResetBlockAxis n u)
    exact h.smul (navigationVariance_positive variance).le

theorem magneticResetCovariance_symmetric {n : ℕ} (A : Matrix3 ℝ) (u : Vector3 ℝ)
    (variance : ℝ) {P : Matrix (Fin 3 ⊕ Fin n) (Fin 3 ⊕ Fin n) ℝ} (hP : P.PosSemidef) :
    (magneticResetCovariance A u variance P)ᵀ = magneticResetCovariance A u variance P :=
  (magneticResetCovariance_posSemidef A u variance hP).isHermitian

theorem magneticResetCovariance_blocks {n : ℕ} (A Ptt : Matrix3 ℝ)
    (Pto : Matrix (Fin 3) (Fin n) ℝ) (Pot : Matrix (Fin n) (Fin 3) ℝ)
    (Poo : Covariance n) (u : Vector3 ℝ) (variance : ℝ) :
    magneticResetCovariance A u variance (fromBlocks Ptt Pto Pot Poo) =
      fromBlocks (A * Ptt * Aᵀ + navigationVariance variance • axisProjector u)
        (A * Pto) (Pot * Aᵀ) Poo := by
  have hn : navigationVariance variance •
      vecMulVec (magneticResetBlockAxis n u) (magneticResetBlockAxis n u) =
      fromBlocks (navigationVariance variance • axisProjector u)
        (0 : Matrix (Fin 3) (Fin n) ℝ) 0 0 := by
    ext (i | i) (j | j) <;>
      simp [magneticResetBlockAxis, fromBlocks, vecMulVec, axisProjector]
  rw [magneticResetCovariance, hn]
  simp [magneticResetBlockTransform, fromBlocks_transpose, fromBlocks_multiply, fromBlocks_add]

theorem magneticResetTransform_identity :
    magneticResetTransform 1 1 = 1 - axisProjector navigationDown := by
  simp [magneticResetTransform]

end
end FormalESKF.ESKF
