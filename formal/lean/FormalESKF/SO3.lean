/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.Quaternion
import Mathlib.Data.Real.Basic
import Mathlib.LinearAlgebra.Matrix.Determinant.Basic
import Mathlib.LinearAlgebra.Matrix.Notation
import Mathlib.LinearAlgebra.UnitaryGroup
import Mathlib.Tactic.FinCases
import Mathlib.Tactic.Ring

/-!
# SO(3) semantics

Exact quaternion-to-rotation-matrix semantics using scalar-first coefficients
and the active, column-vector convention used by the C++ implementation.
-/

namespace FormalESKF

namespace SO3

open Matrix

abbrev Matrix3 (R : Type*) := Matrix (Fin 3) (Fin 3) R
abbrev Vector3 (R : Type*) := Fin 3 → R

/-- The Euclidean squared norm, expressed without a square root. -/
def vectorNormSquared {R : Type*} [CommRing R] (v : Vector3 R) : R :=
  dotProduct v v

/-- `Q-ROT-MATRIX`: the scalar-first quaternion rotation matrix. -/
def rotationMatrix {R : Type*} [CommRing R] (q : Quaternion R) : Matrix3 R :=
  !![
    q.q0 * q.q0 + q.q1 * q.q1 - q.q2 * q.q2 - q.q3 * q.q3,
      2 * (q.q1 * q.q2 - q.q0 * q.q3),
      2 * (q.q1 * q.q3 + q.q0 * q.q2);
    2 * (q.q1 * q.q2 + q.q0 * q.q3),
      q.q0 * q.q0 - q.q1 * q.q1 + q.q2 * q.q2 - q.q3 * q.q3,
      2 * (q.q2 * q.q3 - q.q0 * q.q1);
    2 * (q.q1 * q.q3 - q.q0 * q.q2),
      2 * (q.q2 * q.q3 + q.q0 * q.q1),
      q.q0 * q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3 * q.q3
  ]

/-- Active rotation of a column vector. -/
def rotate {R : Type*} [CommRing R] (q : Quaternion R) (v : Vector3 R) : Vector3 R :=
  rotationMatrix q *ᵥ v

/-- Rotate by the transpose of `R(q)`. -/
def inverseRotate {R : Type*} [CommRing R]
    (q : Quaternion R) (v : Vector3 R) : Vector3 R :=
  (rotationMatrix q)ᵀ *ᵥ v

/-- `SO3-HAT`: cross-product matrix for the project's sign convention. -/
def hat {R : Type*} [CommRing R] (a : Vector3 R) : Matrix3 R :=
  !![
    0, -a 2, a 1;
    a 2, 0, -a 0;
    -a 1, a 0, 0
  ]

/-- Three-dimensional vector cross product. -/
def cross {R : Type*} [CommRing R] (a b : Vector3 R) : Vector3 R :=
  ![
    a 1 * b 2 - a 2 * b 1,
    a 2 * b 0 - a 0 * b 2,
    a 0 * b 1 - a 1 * b 0
  ]

@[simp]
theorem rotationMatrix_identity {R : Type*} [CommRing R] :
    rotationMatrix (Quaternion.identity : Quaternion R) = 1 := by
  ext i j
  fin_cases i <;> fin_cases j <;> simp [rotationMatrix, Quaternion.identity]

/-- `Q-SIGN`: `q` and `-q` produce the same rotation matrix. -/
theorem rotationMatrix_neg {R : Type*} [CommRing R] (q : Quaternion R) :
    rotationMatrix (Quaternion.neg q) = rotationMatrix q := by
  ext i j
  fin_cases i <;> fin_cases j <;>
    simp [rotationMatrix, Quaternion.neg]

/-- Quaternion conjugation corresponds to matrix transposition. -/
theorem rotationMatrix_conjugate {R : Type*} [CommRing R] (q : Quaternion R) :
    rotationMatrix (Quaternion.conjugate q) = (rotationMatrix q)ᵀ := by
  ext i j
  fin_cases i <;> fin_cases j <;>
    simp [rotationMatrix, Quaternion.conjugate, Matrix.transpose] <;> ring

/-- `Q-ROT-COMPOSE`: Hamilton multiplication agrees with matrix composition. -/
theorem rotationMatrix_hamiltonMul {R : Type*} [CommRing R]
    (a b : Quaternion R) :
    rotationMatrix (Quaternion.hamiltonMul a b) = rotationMatrix a * rotationMatrix b := by
  ext i j
  fin_cases i <;> fin_cases j <;>
    simp [rotationMatrix, Quaternion.hamiltonMul, Matrix.mul_apply,
      Fin.sum_univ_succ] <;> ring

/-- Rotation composition applies the rightmost quaternion first. -/
theorem rotate_hamiltonMul {R : Type*} [CommRing R]
    (a b : Quaternion R) (v : Vector3 R) :
    rotate (Quaternion.hamiltonMul a b) v = rotate a (rotate b v) := by
  simp [rotate, rotationMatrix_hamiltonMul]

/-- `q` and `-q` act identically on vectors. -/
theorem rotate_neg {R : Type*} [CommRing R] (q : Quaternion R) (v : Vector3 R) :
    rotate (Quaternion.neg q) v = rotate q v := by
  rw [rotate, rotationMatrix_neg, rotate]

/-- Inverse rotation is rotation by the conjugate quaternion. -/
theorem inverseRotate_eq_rotate_conjugate {R : Type*} [CommRing R]
    (q : Quaternion R) (v : Vector3 R) :
    inverseRotate q v = rotate (Quaternion.conjugate q) v := by
  rw [inverseRotate, rotate, rotationMatrix_conjugate]

/-- The determinant is the cube of the quaternion squared norm. -/
theorem det_rotationMatrix {R : Type*} [CommRing R] (q : Quaternion R) :
    (rotationMatrix q).det = Quaternion.normSquared q ^ 3 := by
  simp [rotationMatrix, Matrix.det_fin_three, Quaternion.normSquared]
  ring

/-- A unit quaternion's rotation matrix has determinant one. -/
theorem det_rotationMatrix_of_unit {R : Type*} [CommRing R] {q : Quaternion R}
    (hq : Quaternion.HasUnitNorm q) : (rotationMatrix q).det = 1 := by
  rw [det_rotationMatrix, hq]
  simp

/-- A unit quaternion's rotation matrix is orthogonal. -/
theorem rotationMatrix_mul_transpose_of_unit {R : Type*} [CommRing R]
    {q : Quaternion R}
    (hq : Quaternion.HasUnitNorm q) :
    rotationMatrix q * (rotationMatrix q)ᵀ = 1 := by
  rw [← rotationMatrix_conjugate, ← rotationMatrix_hamiltonMul,
    Quaternion.hamiltonMul_conjugate_of_unit hq, rotationMatrix_identity]

/-- Orthogonality also holds in transpose-times-matrix order. -/
theorem rotationMatrix_transpose_mul_of_unit {R : Type*} [CommRing R]
    {q : Quaternion R}
    (hq : Quaternion.HasUnitNorm q) :
    (rotationMatrix q)ᵀ * rotationMatrix q = 1 := by
  rw [← rotationMatrix_conjugate, ← rotationMatrix_hamiltonMul,
    Quaternion.conjugate_hamiltonMul_of_unit hq, rotationMatrix_identity]

/-- A unit-quaternion inverse rotation recovers the original vector. -/
theorem inverseRotate_rotate_of_unit {q : Quaternion ℝ}
    (hq : Quaternion.HasUnitNorm q) (v : Vector3 ℝ) :
    inverseRotate q (rotate q v) = v := by
  simp [inverseRotate, rotate, Matrix.mulVec_mulVec,
    rotationMatrix_transpose_mul_of_unit hq]

/-- Rotation scales squared norm by the square of the quaternion squared norm. -/
theorem vectorNormSquared_rotate {R : Type*} [CommRing R]
    (q : Quaternion R) (v : Vector3 R) :
    vectorNormSquared (rotate q v) =
      Quaternion.normSquared q ^ 2 * vectorNormSquared v := by
  simp [vectorNormSquared, rotate, rotationMatrix, Matrix.mulVec,
    dotProduct, Fin.sum_univ_succ]
  unfold Quaternion.normSquared
  ring

/-- `Q-ROT-NORM`: a unit-quaternion rotation preserves vector length squared. -/
theorem vectorNormSquared_rotate_of_unit {R : Type*} [CommRing R]
    {q : Quaternion R} (hq : Quaternion.HasUnitNorm q) (v : Vector3 R) :
    vectorNormSquared (rotate q v) = vectorNormSquared v := by
  rw [vectorNormSquared_rotate, hq]
  simp

/-- The hat matrix implements the three-dimensional cross product. -/
theorem hat_mulVec {R : Type*} [CommRing R] (a b : Vector3 R) :
    hat a *ᵥ b = cross a b := by
  funext i
  fin_cases i <;>
    simp [hat, cross, Matrix.mulVec, dotProduct, Fin.sum_univ_succ] <;> ring

/-- The hat matrix is skew-symmetric. -/
theorem transpose_hat {R : Type*} [CommRing R] (a : Vector3 R) :
    (hat a)ᵀ = -hat a := by
  ext i j
  fin_cases i <;> fin_cases j <;>
    simp [hat, Matrix.transpose]

/-- A unit quaternion maps into Mathlib's special orthogonal group `SO(3)`. -/
theorem rotationMatrix_mem_specialOrthogonalGroup {q : Quaternion ℝ}
    (hq : Quaternion.HasUnitNorm q) :
    rotationMatrix q ∈ Matrix.specialOrthogonalGroup (Fin 3) ℝ := by
  rw [Matrix.mem_specialOrthogonalGroup_iff, Matrix.mem_orthogonalGroup_iff]
  exact ⟨rotationMatrix_mul_transpose_of_unit hq, det_rotationMatrix_of_unit hq⟩

end SO3

end FormalESKF
