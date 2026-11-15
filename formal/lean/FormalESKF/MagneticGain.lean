/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.ConstrainedGain

/-! E-MAG-TILT: project attitude gain along navigation Down in body coordinates.
The exact right-multiplicative Exp update preserves this body Down vector.
This is not a bound on floating-point tilt error or magnetic observability. -/

namespace FormalESKF.ESKF
noncomputable section
open SO3 Quaternion Matrix

def navigationDown : Vector3 ℝ := ![0, 0, 1]
def bodyDown (q : Quaternion ℝ) : Vector3 ℝ := inverseRotate q navigationDown
def axisProjector (u : Vector3 ℝ) : Matrix3 ℝ := fun i j => u i * u j

theorem bodyDown_unit {q : Quaternion ℝ} (hq : HasUnitNorm q) :
    vectorNormSquared (bodyDown q) = 1 := by
  rw [bodyDown, inverseRotate_eq_rotate_conjugate, vectorNormSquared_rotate_of_unit hq.conjugate]
  norm_num [navigationDown, vectorNormSquared, dotProduct, Fin.sum_univ_succ]

theorem axisProjector_symmetric (u : Vector3 ℝ) : (axisProjector u)ᵀ = axisProjector u := by
  ext i j
  exact mul_comm (u j) (u i)

theorem axisProjector_action (u v : Vector3 ℝ) :
    axisProjector u *ᵥ v = vectorScale (dotProduct u v) u := by
  ext i
  simp [Matrix.mulVec, dotProduct, axisProjector, vectorScale, Fin.sum_univ_succ]
  ring

theorem axisProjector_idempotent {u : Vector3 ℝ} (hu : vectorNormSquared u = 1) :
    axisProjector u * axisProjector u = axisProjector u := by
  ext i j
  change (∑ k, (u i * u k) * (u k * u j)) = u i * u j
  calc
    _ = (u i * u j) * ∑ k, u k * u k := by
      rw [Finset.mul_sum]
      apply Finset.sum_congr rfl
      intro k _
      ring
    _ = u i * u j := by
      change (u i * u j) * vectorNormSquared u = _
      rw [hu, mul_one]

theorem axisProjector_orthogonal {u v : Vector3 ℝ} (h : dotProduct u v = 0) :
    axisProjector u *ᵥ v = 0 := by
  rw [axisProjector_action, h]
  ext i
  simp [vectorScale]

def projectedAttitudeGain {m : ℕ} (u : Vector3 ℝ) (K : Matrix (Fin 3) (Fin m) ℝ) :=
  axisProjector u * K

theorem projectedAttitudeGain_action {m : ℕ} (u : Vector3 ℝ)
    (K : Matrix (Fin 3) (Fin m) ℝ) (r : Fin m → ℝ) :
    projectedAttitudeGain u K *ᵥ r = vectorScale (dotProduct u (K *ᵥ r)) u := by
  simpa [projectedAttitudeGain, Matrix.mulVec_mulVec] using axisProjector_action u (K *ᵥ r)

private theorem rotate_parallel (c k : ℝ) (u : Vector3 ℝ) :
    rotate (fromScalarVector c (vectorScale k u)) u =
      vectorScale (normSquared (fromScalarVector c (vectorScale k u))) u := by
  ext i
  fin_cases i <;>
    simp [rotate, rotationMatrix, fromScalarVector, vectorScale, normSquared,
      Matrix.mulVec, dotProduct, Fin.sum_univ_succ] <;> ring

private theorem quaternionExp_parallel (a : ℝ) (u : Vector3 ℝ) :
    ∃ c k, quaternionExp (vectorScale a u) = fromScalarVector c (vectorScale k u) := by
  by_cases h : vectorNorm (vectorScale a u) = 0
  · refine ⟨1, 0, ?_⟩
    simp [quaternionExp, h, identity, fromScalarVector, vectorScale]
  · refine ⟨Real.cos (vectorNorm (vectorScale a u) / 2),
      (Real.sin (vectorNorm (vectorScale a u) / 2) / vectorNorm (vectorScale a u)) * a, ?_⟩
    ext <;> simp [quaternionExp, h, fromScalarVector, vectorScale, mul_assoc]

theorem rotate_exp_parallel (a : ℝ) (u : Vector3 ℝ) :
    rotate (quaternionExp (vectorScale a u)) u = u := by
  obtain ⟨c, k, h⟩ := quaternionExp_parallel a u
  have hn : normSquared (fromScalarVector c (vectorScale k u)) = 1 := by
    rw [← h]
    exact quaternionExp_hasUnitNorm _
  rw [h, rotate_parallel, hn]
  ext i
  simp [vectorScale]

theorem inverseRotate_exp_parallel (a : ℝ) (u : Vector3 ℝ) :
    inverseRotate (quaternionExp (vectorScale a u)) u = u := by
  have hn : -(vectorScale a u) = vectorScale (-a) u := by
    ext i
    simp [vectorScale]
  rw [inverseRotate_eq_rotate_conjugate, ← quaternionExp_neg, hn]
  exact rotate_exp_parallel (-a) u

/-- Any unit quaternion with vector part parallel to the axis fixes that axis.
This also applies to a normalized Taylor candidate; its angle need not equal Exp. -/
theorem inverseRotate_parallel_unit (c k : ℝ) (u : Vector3 ℝ)
    (h : HasUnitNorm (fromScalarVector c (vectorScale k u))) :
    inverseRotate (fromScalarVector c (vectorScale k u)) u = u := by
  rw [inverseRotate_eq_rotate_conjugate]
  have hc : conjugate (fromScalarVector c (vectorScale k u)) =
      fromScalarVector c (vectorScale (-k) u) := by
    ext <;> simp [conjugate, fromScalarVector, vectorScale]
  rw [hc, rotate_parallel]
  have hn : normSquared (fromScalarVector c (vectorScale (-k) u)) = 1 := by
    rw [← hc, normSquared_conjugate]
    exact h
  rw [hn]
  ext i
  simp [vectorScale]

theorem normalize_parallel_fixes_axis (c k : ℝ) (u : Vector3 ℝ)
    (h : 0 < normSquared (fromScalarVector c (vectorScale k u))) :
    inverseRotate (normalize (fromScalarVector c (vectorScale k u))) u = u := by
  let a := 1 / Real.sqrt (normSquared (fromScalarVector c (vectorScale k u)))
  have he : normalize (fromScalarVector c (vectorScale k u)) =
      fromScalarVector (a * c) (vectorScale (a * k) u) := by
    ext <;> simp [Quaternion.normalize, Quaternion.scale, fromScalarVector, vectorScale, a, mul_assoc]
  have hu := hasUnitNorm_normalize h
  rw [he] at hu ⊢
  exact inverseRotate_parallel_unit _ _ _ hu

theorem magnetic_injection_preserves_tilt {m : ℕ} (q : Quaternion ℝ)
    (K : Matrix (Fin 3) (Fin m) ℝ) (r : Fin m → ℝ) :
    bodyDown (localAttitude q (projectedAttitudeGain (bodyDown q) K *ᵥ r)) = bodyDown q := by
  rw [projectedAttitudeGain_action]
  change inverseRotate (hamiltonMul q (quaternionExp _)) navigationDown = _
  have hc (p d : Quaternion ℝ) (v : Vector3 ℝ) :
      inverseRotate (hamiltonMul p d) v = inverseRotate d (inverseRotate p v) := by
    simp [inverseRotate, rotationMatrix_hamiltonMul, transpose_mul, Matrix.mulVec_mulVec]
  rw [hc]
  exact inverseRotate_exp_parallel _ (bodyDown q)

/-- Exact counterpart of normalizing both the increment and the Hamilton product.
Any nonzero parallel-axis candidate works, including the Taylor branch. -/
theorem normalized_magnetic_injection_preserves_tilt {q : Quaternion ℝ}
    (hq : HasUnitNorm q) (c k : ℝ)
    (h : 0 < normSquared (fromScalarVector c (vectorScale k (bodyDown q)))) :
    bodyDown (Quaternion.normalize (hamiltonMul q
      (Quaternion.normalize (fromScalarVector c (vectorScale k (bodyDown q)))))) = bodyDown q := by
  have hu := hq.hamiltonMul (hasUnitNorm_normalize h)
  have he : Quaternion.normalize (hamiltonMul q
      (Quaternion.normalize (fromScalarVector c (vectorScale k (bodyDown q))))) =
      hamiltonMul q (Quaternion.normalize (fromScalarVector c (vectorScale k (bodyDown q)))) := by
    have hn : normSquared (hamiltonMul q
        (Quaternion.normalize (fromScalarVector c (vectorScale k (bodyDown q))))) = 1 := hu
    rw [Quaternion.normalize, hn]
    ext <;> simp [Quaternion.scale]
  rw [he]
  change inverseRotate (hamiltonMul q _) navigationDown = _
  have hc (p d : Quaternion ℝ) (v : Vector3 ℝ) :
      inverseRotate (hamiltonMul p d) v = inverseRotate d (inverseRotate p v) := by
    simp [inverseRotate, rotationMatrix_hamiltonMul, transpose_mul, Matrix.mulVec_mulVec]
  rw [hc]
  exact normalize_parallel_fixes_axis c k (bodyDown q) h

end
end FormalESKF.ESKF
