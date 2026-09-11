/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import Mathlib.Tactic.Ring
import Mathlib.Analysis.SpecialFunctions.Sqrt

/-!
# Quaternion semantics

Exact scalar-first quaternion definitions corresponding coefficient-for-
coefficient to `formal_eskf::so3::UnitQuaternion`.

These theorems are over a commutative ring. They do not claim that the same
equalities hold exactly for IEEE-754 evaluation.
-/

namespace FormalESKF

/-- A scalar-first quaternion `[q0, q1, q2, q3]`. -/
@[ext]
structure Quaternion (R : Type*) where
  q0 : R
  q1 : R
  q2 : R
  q3 : R
deriving DecidableEq

namespace Quaternion

variable {R : Type*}

/-- `Q-IDENTITY`: the multiplicative identity `[1, 0, 0, 0]`. -/
def identity [Zero R] [One R] : Quaternion R :=
  { q0 := 1, q1 := 0, q2 := 0, q3 := 0 }

/-- Embed a scalar as `[a, 0, 0, 0]`. -/
def ofScalar [Zero R] (a : R) : Quaternion R :=
  { q0 := a, q1 := 0, q2 := 0, q3 := 0 }

/-- Coefficient-wise negation. -/
def neg [Neg R] (q : Quaternion R) : Quaternion R :=
  { q0 := -q.q0, q1 := -q.q1, q2 := -q.q2, q3 := -q.q3 }

/-- Multiply every quaternion coefficient by the same scalar. -/
def scale [Mul R] (a : R) (q : Quaternion R) : Quaternion R :=
  { q0 := a * q.q0, q1 := a * q.q1, q2 := a * q.q2, q3 := a * q.q3 }

/-- `Q-MUL`: scalar-first Hamilton multiplication. -/
def hamiltonMul [Mul R] [Sub R] [Add R] (a b : Quaternion R) : Quaternion R :=
  {
    q0 := a.q0 * b.q0 - a.q1 * b.q1 - a.q2 * b.q2 - a.q3 * b.q3
    q1 := a.q0 * b.q1 + a.q1 * b.q0 + a.q2 * b.q3 - a.q3 * b.q2
    q2 := a.q0 * b.q2 - a.q1 * b.q3 + a.q2 * b.q0 + a.q3 * b.q1
    q3 := a.q0 * b.q3 + a.q1 * b.q2 - a.q2 * b.q1 + a.q3 * b.q0
  }

/-- `Q-INVERSE`: conjugation, which is the inverse of a unit quaternion. -/
def conjugate [Neg R] (q : Quaternion R) : Quaternion R :=
  { q0 := q.q0, q1 := -q.q1, q2 := -q.q2, q3 := -q.q3 }

/-- The polynomial squared norm `q0^2 + q1^2 + q2^2 + q3^2`. -/
def normSquared [Add R] [Mul R] (q : Quaternion R) : R :=
  q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3

/-- Exact unit-norm predicate. -/
def HasUnitNorm [One R] [Add R] [Mul R] (q : Quaternion R) : Prop :=
  normSquared q = 1

/-- `Q-NORMALIZE`: exact normalization of a nonzero real quaternion. -/
noncomputable def normalize (q : Quaternion ℝ) : Quaternion ℝ :=
  scale (1 / √(normSquared q)) q

@[simp]
theorem neg_neg [AddGroup R] (q : Quaternion R) : neg (neg q) = q := by
  ext <;> simp [neg]

@[simp]
theorem conjugate_conjugate [AddGroup R] (q : Quaternion R) : conjugate (conjugate q) = q := by
  ext <;> simp [conjugate]

@[simp]
theorem identity_hamiltonMul [CommRing R] (q : Quaternion R) :
    hamiltonMul identity q = q := by
  ext <;> simp [hamiltonMul, identity]

@[simp]
theorem hamiltonMul_identity [CommRing R] (q : Quaternion R) :
    hamiltonMul q identity = q := by
  ext <;> simp [hamiltonMul, identity]

/-- `Q-MUL-ASSOC`: Hamilton multiplication is associative in the exact model. -/
theorem hamiltonMul_assoc [CommRing R] (a b c : Quaternion R) :
    hamiltonMul (hamiltonMul a b) c = hamiltonMul a (hamiltonMul b c) := by
  ext <;> simp [hamiltonMul] <;> ring

/-- Conjugation reverses Hamilton-product order. -/
theorem conjugate_hamiltonMul [CommRing R] (a b : Quaternion R) :
    conjugate (hamiltonMul a b) = hamiltonMul (conjugate b) (conjugate a) := by
  ext <;> simp [conjugate, hamiltonMul] <;> ring

@[simp]
theorem normSquared_identity [CommRing R] :
    normSquared (identity : Quaternion R) = 1 := by
  simp [normSquared, identity]

@[simp]
theorem normSquared_neg [CommRing R] (q : Quaternion R) :
    normSquared (neg q) = normSquared q := by
  simp [normSquared, neg]

@[simp]
theorem normSquared_conjugate [CommRing R] (q : Quaternion R) :
    normSquared (conjugate q) = normSquared q := by
  simp [normSquared, conjugate]

theorem normSquared_scale [CommRing R] (a : R) (q : Quaternion R) :
    normSquared (scale a q) = a * a * normSquared q := by
  simp [normSquared, scale]
  ring

/-- Exact normalization divides each coefficient by the Euclidean norm. -/
theorem normalize_coefficients (q : Quaternion ℝ) :
    (normalize q).q0 = q.q0 / √(normSquared q) ∧
    (normalize q).q1 = q.q1 / √(normSquared q) ∧
    (normalize q).q2 = q.q2 / √(normSquared q) ∧
    (normalize q).q3 = q.q3 / √(normSquared q) := by
  simp [normalize, scale, div_eq_mul_inv, mul_comm]

/-- Exact normalization produces unit norm whenever the input norm is nonzero. -/
theorem hasUnitNorm_normalize {q : Quaternion ℝ} (hq : 0 < normSquared q) :
    HasUnitNorm (normalize q) := by
  rw [HasUnitNorm, normalize, normSquared_scale]
  have hsqrt : √(normSquared q) ≠ 0 := ne_of_gt (Real.sqrt_pos.2 hq)
  field_simp [hsqrt]
  exact (Real.sq_sqrt hq.le).symm

/-- Multiplying by the conjugate yields the squared norm as a scalar quaternion. -/
theorem hamiltonMul_conjugate [CommRing R] (q : Quaternion R) :
    hamiltonMul q (conjugate q) = ofScalar (normSquared q) := by
  ext <;> simp [hamiltonMul, conjugate, ofScalar, normSquared] <;> ring

/-- Multiplying the conjugate from the left gives the same scalar quaternion. -/
theorem conjugate_hamiltonMul_self [CommRing R] (q : Quaternion R) :
    hamiltonMul (conjugate q) q = ofScalar (normSquared q) := by
  ext <;> simp [hamiltonMul, conjugate, ofScalar, normSquared] <;> ring

/-- `Q-NORM-MUL`: the exact squared norm is multiplicative. -/
theorem normSquared_hamiltonMul [CommRing R] (a b : Quaternion R) :
    normSquared (hamiltonMul a b) = normSquared a * normSquared b := by
  simp [normSquared, hamiltonMul]
  ring

@[simp]
theorem hasUnitNorm_identity [CommRing R] :
    HasUnitNorm (identity : Quaternion R) := by
  simp [HasUnitNorm]

/-- Hamilton multiplication is closed on exact unit quaternions. -/
theorem HasUnitNorm.hamiltonMul [CommRing R] {a b : Quaternion R}
    (ha : HasUnitNorm a) (hb : HasUnitNorm b) : HasUnitNorm (hamiltonMul a b) := by
  rw [HasUnitNorm, normSquared_hamiltonMul, ha, hb, one_mul]

@[simp]
theorem HasUnitNorm.conjugate [CommRing R] {q : Quaternion R}
    (hq : HasUnitNorm q) : HasUnitNorm (conjugate q) := by
  rw [HasUnitNorm, normSquared_conjugate, hq]

/-- A unit quaternion multiplied by its conjugate gives identity. -/
theorem hamiltonMul_conjugate_of_unit [CommRing R] {q : Quaternion R}
    (hq : HasUnitNorm q) : hamiltonMul q (conjugate q) = identity := by
  rw [hamiltonMul_conjugate, hq]
  rfl

/-- A unit quaternion's conjugate is also its left inverse. -/
theorem conjugate_hamiltonMul_of_unit [CommRing R] {q : Quaternion R}
    (hq : HasUnitNorm q) : hamiltonMul (conjugate q) q = identity := by
  rw [conjugate_hamiltonMul_self, hq]
  rfl

end Quaternion

end FormalESKF
