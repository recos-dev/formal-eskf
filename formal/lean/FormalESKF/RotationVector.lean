/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.SO3
import Mathlib.Analysis.Calculus.Taylor
import Mathlib.Analysis.SpecialFunctions.Trigonometric.ArctanDeriv
import Mathlib.Analysis.SpecialFunctions.Trigonometric.Arctan
import Mathlib.Analysis.SpecialFunctions.Trigonometric.Deriv
import Mathlib.Analysis.SpecialFunctions.Trigonometric.Inverse
import Mathlib.LinearAlgebra.Matrix.DotProduct
import Mathlib.Tactic.FieldSimp

/-!
# Rotation-vector semantics

Exact real-number definitions for the quaternion exponential, principal
quaternion logarithm, and the polynomial used by the C++ small-angle branch.
The polynomial is modeled separately from the exact exponential so that an
approximation is never silently treated as an identity.
-/

namespace FormalESKF

namespace SO3

open Filter

/-- Scale a three-dimensional vector. -/
def vectorScale (a : ℝ) (v : Vector3 ℝ) : Vector3 ℝ :=
  fun i => a * v i

/-- The Euclidean norm of a three-dimensional real vector. -/
noncomputable def vectorNorm (v : Vector3 ℝ) : ℝ :=
  √(vectorNormSquared v)

/-- Scale every coefficient of a real three-by-three matrix. -/
def matrixScale (a : ℝ) (matrix : Matrix3 ℝ) : Matrix3 ℝ :=
  fun i j => a * matrix i j

/-- The vector part `[q1, q2, q3]` of a scalar-first quaternion. -/
def vectorPart (q : Quaternion ℝ) : Vector3 ℝ :=
  ![q.q1, q.q2, q.q3]

/-- Construct a scalar-first quaternion from a scalar and vector part. -/
def fromScalarVector (q0 : ℝ) (qv : Vector3 ℝ) : Quaternion ℝ :=
  { q0 := q0, q1 := qv 0, q2 := qv 1, q3 := qv 2 }

/-- `SO3-EXP`: exact quaternion exponential of a rotation vector. -/
noncomputable def quaternionExp (phi : Vector3 ℝ) : Quaternion ℝ :=
  let theta := vectorNorm phi
  if theta = 0 then
    Quaternion.identity
  else
    fromScalarVector (Real.cos (theta / 2))
      (vectorScale (Real.sin (theta / 2) / theta) phi)

/-- `SO3-EXP`: Rodrigues' exact rotation-matrix exponential. -/
noncomputable def rotationMatrixExp (phi : Vector3 ℝ) : Matrix3 ℝ :=
  let theta := vectorNorm phi
  if theta = 0 then
    1
  else
    1 + matrixScale (Real.sin theta / theta) (hat phi) +
      matrixScale ((1 - Real.cos theta) / theta ^ 2) (hat phi * hat phi)

/-- Candidate evaluated by the C++ Taylor branch before checked normalization. -/
noncomputable def quaternionExpTaylorCandidate (phi : Vector3 ℝ) : Quaternion ℝ :=
  let thetaSquared := vectorNormSquared phi
  let thetaFourth := thetaSquared * thetaSquared
  fromScalarVector
    (1 - thetaSquared / 8 + thetaFourth / 384)
    (vectorScale (1 / 2 - thetaSquared / 48 + thetaFourth / 3840) phi)

/-- Scale evaluated by the C++ principal-Log small-vector branch. -/
noncomputable def quaternionLogTaylorScale (q0 qvSquaredNorm : ℝ) : ℝ :=
  2 / q0 * (1 - qvSquaredNorm / (3 * q0 ^ 2))

/-- Select the representative used by the C++ principal-log implementation. -/
noncomputable def principalRepresentative (q : Quaternion ℝ) : Quaternion ℝ :=
  if q.q0 < 0 then Quaternion.neg q else q

/-- Candidate evaluated by the C++ principal-Log small-vector branch. -/
noncomputable def quaternionLogTaylorCandidate (q : Quaternion ℝ) : Vector3 ℝ :=
  let representative := principalRepresentative q
  let qv := vectorPart representative
  vectorScale
    (quaternionLogTaylorScale representative.q0 (vectorNormSquared qv)) qv

/-- `atan2(y, x)` specialized to the closed first quadrant used after sign selection. -/
noncomputable def firstQuadrantAtan2 (y x : ℝ) : ℝ :=
  if x = 0 then Real.pi / 2 else Real.arctan (y / x)

/-- `SO3-LOG`: principal quaternion logarithm for a unit quaternion. -/
noncomputable def quaternionLog (q : Quaternion ℝ) : Vector3 ℝ :=
  let representative := principalRepresentative q
  let qv := vectorPart representative
  let qvNorm := vectorNorm qv
  if qvNorm = 0 then
    0
  else
    vectorScale (2 * firstQuadrantAtan2 qvNorm representative.q0 / qvNorm) qv

@[simp]
theorem vectorScale_zero (a : ℝ) : vectorScale a (0 : Vector3 ℝ) = 0 := by
  funext i
  simp [vectorScale]

@[simp]
theorem vectorNormSquared_zero : vectorNormSquared (0 : Vector3 ℝ) = 0 := by
  simp [vectorNormSquared, dotProduct]

@[simp]
theorem vectorNorm_zero : vectorNorm (0 : Vector3 ℝ) = 0 := by
  simp [vectorNorm]

theorem vectorNormSquared_nonnegative (v : Vector3 ℝ) :
    0 ≤ vectorNormSquared v := by
  have h : vectorNormSquared v =
      v 0 * v 0 + v 1 * v 1 + v 2 * v 2 := by
    simp [vectorNormSquared, dotProduct, Fin.sum_univ_succ]
    ring
  rw [h]
  nlinarith [sq_nonneg (v 0), sq_nonneg (v 1), sq_nonneg (v 2)]

theorem vectorNorm_nonnegative (v : Vector3 ℝ) : 0 ≤ vectorNorm v := by
  exact Real.sqrt_nonneg _

theorem vectorNorm_squared (v : Vector3 ℝ) :
    vectorNorm v ^ 2 = vectorNormSquared v := by
  exact Real.sq_sqrt (vectorNormSquared_nonnegative v)

theorem vectorNorm_eq_zero_iff (v : Vector3 ℝ) : vectorNorm v = 0 ↔ v = 0 := by
  rw [vectorNorm, Real.sqrt_eq_zero (vectorNormSquared_nonnegative v)]
  exact dotProduct_self_eq_zero

theorem vectorNormSquared_vectorScale (a : ℝ) (v : Vector3 ℝ) :
    vectorNormSquared (vectorScale a v) = a ^ 2 * vectorNormSquared v := by
  simp [vectorNormSquared, vectorScale, dotProduct, Fin.sum_univ_succ]
  ring

theorem vectorNorm_vectorScale_of_nonnegative {a : ℝ} (ha : 0 ≤ a)
    (v : Vector3 ℝ) :
    vectorNorm (vectorScale a v) = a * vectorNorm v := by
  rw [vectorNorm, vectorNormSquared_vectorScale, vectorNorm]
  rw [Real.sqrt_mul (sq_nonneg a), Real.sqrt_sq ha]

@[simp]
theorem vectorScale_one (v : Vector3 ℝ) : vectorScale 1 v = v := by
  funext i
  simp [vectorScale]

theorem vectorScale_vectorScale (a b : ℝ) (v : Vector3 ℝ) :
    vectorScale a (vectorScale b v) = vectorScale (a * b) v := by
  funext i
  simp [vectorScale, mul_assoc]

theorem normSquared_fromScalarVector (q0 : ℝ) (qv : Vector3 ℝ) :
    Quaternion.normSquared (fromScalarVector q0 qv) =
      q0 ^ 2 + vectorNormSquared qv := by
  simp [Quaternion.normSquared, fromScalarVector, vectorNormSquared,
    dotProduct, Fin.sum_univ_succ]
  ring

@[simp]
theorem vectorPart_fromScalarVector (q0 : ℝ) (qv : Vector3 ℝ) :
    vectorPart (fromScalarVector q0 qv) = qv := by
  funext i
  fin_cases i <;> simp [vectorPart, fromScalarVector]

@[simp]
theorem vectorPart_identity :
    vectorPart (Quaternion.identity : Quaternion ℝ) = 0 := by
  funext i
  fin_cases i <;> simp [vectorPart, Quaternion.identity]

@[simp]
theorem quaternionExp_zero : quaternionExp (0 : Vector3 ℝ) = Quaternion.identity := by
  simp [quaternionExp]

@[simp]
theorem rotationMatrixExp_zero : rotationMatrixExp (0 : Vector3 ℝ) = 1 := by
  simp [rotationMatrixExp]

/-- Away from zero, the exact exponential has the stated closed-form coefficients. -/
theorem quaternionExp_of_vectorNorm_ne_zero {phi : Vector3 ℝ}
    (h : vectorNorm phi ≠ 0) :
    quaternionExp phi =
      fromScalarVector (Real.cos (vectorNorm phi / 2))
        (vectorScale (Real.sin (vectorNorm phi / 2) / vectorNorm phi) phi) := by
  simp [quaternionExp, h]

/-- The exact quaternion exponential has unit norm. -/
theorem quaternionExp_hasUnitNorm (phi : Vector3 ℝ) :
    Quaternion.HasUnitNorm (quaternionExp phi) := by
  by_cases htheta : vectorNorm phi = 0
  · simp [quaternionExp, htheta, Quaternion.HasUnitNorm]
  · rw [quaternionExp_of_vectorNorm_ne_zero htheta]
    rw [Quaternion.HasUnitNorm, normSquared_fromScalarVector,
      vectorNormSquared_vectorScale]
    have hthetaSquared := vectorNorm_squared phi
    field_simp [htheta]
    nlinarith [Real.sin_sq_add_cos_sq (vectorNorm phi / 2)]

/-- Quaternion Exp followed by `R(q)` is exactly Rodrigues' matrix Exp. -/
theorem rotationMatrix_quaternionExp (phi : Vector3 ℝ) :
    rotationMatrix (quaternionExp phi) = rotationMatrixExp phi := by
  by_cases hzero : vectorNorm phi = 0
  · have hphi : phi = 0 := (vectorNorm_eq_zero_iff phi).mp hzero
    subst phi
    simp
  · let theta := vectorNorm phi
    let sineScale := Real.sin (theta / 2) / theta
    let cosineHalf := Real.cos (theta / 2)
    have htheta : theta ≠ 0 := hzero
    have hthetaSquared : theta ^ 2 =
        phi 0 * phi 0 + phi 1 * phi 1 + phi 2 * phi 2 := by
      rw [show theta = vectorNorm phi by rfl, vectorNorm_squared]
      simp [vectorNormSquared, dotProduct, Fin.sum_univ_succ]
      ring
    have hsinDouble : Real.sin theta =
        2 * Real.sin (theta / 2) * Real.cos (theta / 2) := by
      conv_lhs => rw [show theta = 2 * (theta / 2) by ring]
      rw [Real.sin_two_mul]
    have hcosDouble : 1 - Real.cos theta =
        2 * Real.sin (theta / 2) ^ 2 := by
      conv_lhs => rw [show theta = 2 * (theta / 2) by ring]
      rw [Real.cos_two_mul_eq_one_sub]
      ring
    have hsinScale : Real.sin theta / theta =
        2 * cosineHalf * sineScale := by
      rw [hsinDouble]
      simp only [cosineHalf, sineScale]
      field_simp [htheta]
    have hcosScale : (1 - Real.cos theta) / theta ^ 2 =
        2 * sineScale ^ 2 := by
      rw [hcosDouble]
      simp only [sineScale]
      field_simp [htheta]
    have hunit : cosineHalf ^ 2 + sineScale ^ 2 *
        (phi 0 * phi 0 + phi 1 * phi 1 + phi 2 * phi 2) = 1 := by
      rw [← hthetaSquared]
      have hcancel : sineScale ^ 2 * theta ^ 2 = Real.sin (theta / 2) ^ 2 := by
        simp only [sineScale]
        field_simp [htheta]
      rw [hcancel]
      exact Real.cos_sq_add_sin_sq (theta / 2)
    rw [quaternionExp_of_vectorNorm_ne_zero hzero]
    change rotationMatrix (fromScalarVector cosineHalf (vectorScale sineScale phi)) =
      rotationMatrixExp phi
    simp only [rotationMatrixExp, hzero, if_false]
    rw [hsinScale, hcosScale]
    ext i j
    fin_cases i <;> fin_cases j <;>
      simp [rotationMatrix, fromScalarVector, vectorScale, matrixScale, hat]
    all_goals ring_nf at hunit ⊢
    all_goals nlinarith [hunit]

@[simp]
theorem quaternionExpTaylorCandidate_zero :
    quaternionExpTaylorCandidate (0 : Vector3 ℝ) = Quaternion.identity := by
  ext <;>
    simp [quaternionExpTaylorCandidate, fromScalarVector, vectorScale,
      Quaternion.identity]

/-- The scalar Taylor branch retains the exact fourth-order remainder shown here. -/
theorem expTaylor_scalar_remainder (thetaSquared : ℝ) :
    (1 - thetaSquared / 8 + thetaSquared * thetaSquared / 384) -
        (1 - thetaSquared / 8) =
      thetaSquared * thetaSquared / 384 := by
  ring

/-- The vector scale retains the exact fourth-order remainder shown here. -/
theorem expTaylor_vector_scale_remainder (thetaSquared : ℝ) :
    (1 / 2 - thetaSquared / 48 + thetaSquared * thetaSquared / 3840) -
        (1 / 2 - thetaSquared / 48) =
      thetaSquared * thetaSquared / 3840 := by
  ring

/-- Analytic remainder for the scalar cosine polynomial used by quaternion Exp. -/
theorem cos_fourth_order_remainder :
    (fun x : ℝ => Real.cos x - (1 - x ^ 2 / 2 + x ^ 4 / 24))
      =o[nhds 0] (fun x => x ^ 4) := by
  have hpoly : (fun x : ℝ => taylorWithinEval Real.cos 4 Set.univ 0 x) =
      fun x => 1 - x ^ 2 / 2 + x ^ 4 / 24 := by
    funext x
    simp [taylorWithinEval_succ]
    ring
  simpa only [hpoly, sub_zero] using
    (taylor_isLittleO_univ (f := Real.cos) (x₀ := 0) (n := 4)
      Real.contDiff_cos)

/-- Analytic remainder for the vector-scale sine polynomial used by quaternion Exp. -/
theorem sin_fifth_order_remainder :
    (fun x : ℝ => Real.sin x - (x - x ^ 3 / 6 + x ^ 5 / 120))
      =o[nhds 0] (fun x => x ^ 5) := by
  have hpoly : (fun x : ℝ => taylorWithinEval Real.sin 5 Set.univ 0 x) =
      fun x => x - x ^ 3 / 6 + x ^ 5 / 120 := by
    funext x
    simp [taylorWithinEval_succ]
    ring
  simpa only [hpoly, sub_zero] using
    (taylor_isLittleO_univ (f := Real.sin) (x₀ := 0) (n := 5)
      Real.contDiff_sin)

theorem deriv_one_div_one_add_sq :
    deriv (fun x : ℝ => 1 / (1 + x ^ 2)) =
      fun x => -(2 * x) / (1 + x ^ 2) ^ 2 := by
  funext x
  have hbase : HasDerivAt (fun y : ℝ => 1 + y ^ 2) (2 * x) x := by
    have h := (hasDerivAt_const x (1 : ℝ)).add ((hasDerivAt_id x).pow 2)
    change HasDerivAt (fun y : ℝ => 1 + y ^ 2) (0 + 2 * x ^ (2 - 1) * 1) x at h
    convert h using 1 <;> ring
  simpa only [one_div] using (hbase.fun_inv (by positivity)).deriv

theorem deriv_neg_two_mul_div_one_add_sq_sq :
    deriv (fun x : ℝ => -(2 * x) / (1 + x ^ 2) ^ 2) =
      fun x => (6 * x ^ 2 - 2) / (1 + x ^ 2) ^ 3 := by
  funext x
  have hnum : HasDerivAt (fun y : ℝ => -(2 * y)) (-2) x := by
    have h := (hasDerivAt_id x).const_mul (-2)
    change HasDerivAt (fun y : ℝ => -2 * y) (-2 * 1) x at h
    convert h using 1 <;> ring
  have hbase : HasDerivAt (fun y : ℝ => 1 + y ^ 2) (2 * x) x := by
    have h := (hasDerivAt_const x (1 : ℝ)).add ((hasDerivAt_id x).pow 2)
    change HasDerivAt (fun y : ℝ => 1 + y ^ 2) (0 + 2 * x ^ (2 - 1) * 1) x at h
    convert h using 1 <;> ring
  have hdenominator : (1 + x ^ 2) ^ 2 ≠ 0 := by positivity
  have hquotient := hnum.div (hbase.pow 2) hdenominator
  change HasDerivAt (fun y : ℝ => -(2 * y) / (1 + y ^ 2) ^ 2)
    ((-2 * (1 + x ^ 2) ^ 2 - (-(2 * x)) *
      (2 * (1 + x ^ 2) ^ (2 - 1) * (2 * x))) / ((1 + x ^ 2) ^ 2) ^ 2) x at hquotient
  rw [hquotient.deriv]
  field_simp [show 1 + x ^ 2 ≠ 0 by positivity]
  ring

theorem deriv_six_sq_sub_two_div_one_add_sq_cube :
    deriv (fun x : ℝ => (6 * x ^ 2 - 2) / (1 + x ^ 2) ^ 3) =
      fun x => 24 * x * (1 - x ^ 2) / (1 + x ^ 2) ^ 4 := by
  funext x
  have hnum : HasDerivAt (fun y : ℝ => 6 * y ^ 2 - 2) (12 * x) x := by
    have h := ((hasDerivAt_id x).pow 2).const_mul 6 |>.sub_const 2
    simp only [Pi.pow_apply, id_eq] at h
    ring_nf at h ⊢
    exact h
  have hbase : HasDerivAt (fun y : ℝ => 1 + y ^ 2) (2 * x) x := by
    have h := (hasDerivAt_const x (1 : ℝ)).add ((hasDerivAt_id x).pow 2)
    change HasDerivAt (fun y : ℝ => 1 + y ^ 2) (0 + 2 * x ^ (2 - 1) * 1) x at h
    convert h using 1 <;> ring
  have hdenominator : (1 + x ^ 2) ^ 3 ≠ 0 := by positivity
  have hquotient := hnum.div (hbase.pow 3) hdenominator
  change HasDerivAt (fun y : ℝ => (6 * y ^ 2 - 2) / (1 + y ^ 2) ^ 3)
    ((12 * x * (1 + x ^ 2) ^ 3 - (6 * x ^ 2 - 2) *
      (3 * (1 + x ^ 2) ^ (3 - 1) * (2 * x))) / ((1 + x ^ 2) ^ 3) ^ 2) x at hquotient
  rw [hquotient.deriv]
  field_simp [show 1 + x ^ 2 ≠ 0 by positivity]
  ring

theorem arctan_iteratedDeriv_two_at_zero :
    iteratedDeriv 2 Real.arctan 0 = 0 := by
  simp [iteratedDeriv_succ]

theorem arctan_iteratedDeriv_three_at_zero :
    iteratedDeriv 3 Real.arctan 0 = -2 := by
  have hsecond : HasDerivAt
      (fun x : ℝ => -(2 * x) / (1 + x ^ 2) ^ 2) (-2) 0 := by
    have hnum : HasDerivAt (fun x : ℝ => -(2 * x)) (-2) 0 := by
      have h := (hasDerivAt_id (0 : ℝ)).const_mul (-2)
      change HasDerivAt (fun y : ℝ => -2 * y) (-2 * 1) 0 at h
      convert h using 1 <;> ring
    have hbase : HasDerivAt (fun x : ℝ => 1 + x ^ 2) 0 0 := by
      have h := (hasDerivAt_const (0 : ℝ) (1 : ℝ)).add
        ((hasDerivAt_id (0 : ℝ)).pow 2)
      change HasDerivAt (fun y : ℝ => 1 + y ^ 2)
        (0 + 2 * 0 ^ (2 - 1) * 1) 0 at h
      norm_num at h ⊢
      exact h
    have h := hnum.div (hbase.pow 2) (by norm_num)
    change HasDerivAt (fun x : ℝ => -(2 * x) / (1 + x ^ 2) ^ 2)
      ((-2 * (1 + 0 ^ 2) ^ 2 - (-(2 * 0)) *
        (2 * (1 + 0 ^ 2) ^ (2 - 1) * 0)) / ((1 + 0 ^ 2) ^ 2) ^ 2) 0 at h
    norm_num at h ⊢
    exact h
  rw [show iteratedDeriv 3 Real.arctan =
    deriv (deriv (deriv Real.arctan)) by simp [iteratedDeriv_succ]]
  rw [Real.deriv_arctan, deriv_one_div_one_add_sq]
  exact hsecond.deriv

theorem arctan_iteratedDeriv_four_at_zero :
    iteratedDeriv 4 Real.arctan 0 = 0 := by
  rw [show iteratedDeriv 4 Real.arctan =
    deriv (deriv (deriv (deriv Real.arctan))) by simp [iteratedDeriv_succ]]
  rw [Real.deriv_arctan, deriv_one_div_one_add_sq,
    deriv_neg_two_mul_div_one_add_sq_sq,
    deriv_six_sq_sub_two_div_one_add_sq_cube]
  norm_num

theorem arctan_iteratedDeriv_five_at_zero :
    iteratedDeriv 5 Real.arctan 0 = 24 := by
  have hnum : HasDerivAt (fun x : ℝ => 24 * x * (1 - x ^ 2)) 24 0 := by
    have hlinear : HasDerivAt (fun x : ℝ => 24 * x) 24 0 := by
      simpa using (hasDerivAt_id (0 : ℝ)).const_mul 24
    have hquadratic : HasDerivAt (fun x : ℝ => 1 - x ^ 2) 0 0 := by
      have h := (hasDerivAt_const (0 : ℝ) 1).sub
        ((hasDerivAt_id (0 : ℝ)).pow 2)
      change HasDerivAt (fun x : ℝ => 1 - x ^ 2) (0 - 2 * 0 ^ (2 - 1) * 1) 0 at h
      norm_num at h ⊢
      exact h
    have h := hlinear.mul hquadratic
    change HasDerivAt (fun x : ℝ => 24 * x * (1 - x ^ 2))
      (24 * (1 - 0 ^ 2) + (24 * 0) * 0) 0 at h
    norm_num at h ⊢
    exact h
  have hbase : HasDerivAt (fun x : ℝ => 1 + x ^ 2) 0 0 := by
    have h := (hasDerivAt_const (0 : ℝ) (1 : ℝ)).add
      ((hasDerivAt_id (0 : ℝ)).pow 2)
    change HasDerivAt (fun x : ℝ => 1 + x ^ 2)
      (0 + 2 * 0 ^ (2 - 1) * 1) 0 at h
    norm_num at h ⊢
    exact h
  have hquotient := hnum.div (hbase.pow 4) (by norm_num)
  change HasDerivAt (fun x : ℝ => 24 * x * (1 - x ^ 2) / (1 + x ^ 2) ^ 4)
    ((24 * (1 + 0 ^ 2) ^ 4 - (24 * 0 * (1 - 0 ^ 2)) *
      (4 * (1 + 0 ^ 2) ^ (4 - 1) * 0)) / ((1 + 0 ^ 2) ^ 4) ^ 2) 0 at hquotient
  norm_num at hquotient
  rw [show iteratedDeriv 5 Real.arctan =
    deriv (deriv (deriv (deriv (deriv Real.arctan)))) by simp [iteratedDeriv_succ]]
  rw [Real.deriv_arctan, deriv_one_div_one_add_sq,
    deriv_neg_two_mul_div_one_add_sq_sq,
    deriv_six_sq_sub_two_div_one_add_sq_cube]
  exact hquotient.deriv

/-- Analytic remainder behind the principal-Log small-vector scale. -/
theorem arctan_third_order_remainder :
    (fun x : ℝ => Real.arctan x - (x - x ^ 3 / 3))
      =o[nhds 0] (fun x => x ^ 3) := by
  have hpoly : (fun x : ℝ =>
      taylorWithinEval Real.arctan 3 Set.univ 0 x) =
      fun x => x - x ^ 3 / 3 := by
    funext x
    simp [taylorWithinEval_succ, arctan_iteratedDeriv_two_at_zero,
      arctan_iteratedDeriv_three_at_zero]
    ring
  simpa only [hpoly, sub_zero] using
    (taylor_isLittleO_univ (f := Real.arctan) (x₀ := 0) (n := 3)
      Real.contDiff_arctan)

/-- Fifth-order remainder supporting the `O(r⁴)` principal-Log scale term. -/
theorem arctan_fifth_order_remainder :
    (fun x : ℝ => Real.arctan x - (x - x ^ 3 / 3 + x ^ 5 / 5))
      =o[nhds 0] (fun x => x ^ 5) := by
  have hpoly : (fun x : ℝ =>
      taylorWithinEval Real.arctan 5 Set.univ 0 x) =
      fun x => x - x ^ 3 / 3 + x ^ 5 / 5 := by
    funext x
    simp [taylorWithinEval_succ, arctan_iteratedDeriv_two_at_zero,
      arctan_iteratedDeriv_three_at_zero, arctan_iteratedDeriv_four_at_zero,
      arctan_iteratedDeriv_five_at_zero]
    ring
  simpa only [hpoly, sub_zero] using
    (taylor_isLittleO_univ (f := Real.arctan) (x₀ := 0) (n := 5)
      Real.contDiff_arctan)

@[simp]
theorem principalRepresentative_identity :
    principalRepresentative (Quaternion.identity : Quaternion ℝ) = Quaternion.identity := by
  norm_num [principalRepresentative, Quaternion.identity]

@[simp]
theorem quaternionLogTaylorCandidate_identity :
    quaternionLogTaylorCandidate (Quaternion.identity : Quaternion ℝ) = 0 := by
  simp [quaternionLogTaylorCandidate, quaternionLogTaylorScale]

/-- Sign selection always produces a representative with nonnegative scalar part. -/
theorem principalRepresentative_q0_nonnegative (q : Quaternion ℝ) :
    0 ≤ (principalRepresentative q).q0 := by
  by_cases h : q.q0 < 0
  · simp [principalRepresentative, h, Quaternion.neg, h.le]
  · simp [principalRepresentative, h, le_of_not_gt h]

theorem principalRepresentative_of_q0_nonnegative {q : Quaternion ℝ}
    (h : 0 ≤ q.q0) :
    principalRepresentative q = q := by
  simp [principalRepresentative, not_lt_of_ge h]

theorem firstQuadrantAtan2_nonnegative {y x : ℝ} (hy : 0 ≤ y)
    (hx : 0 ≤ x) :
    0 ≤ firstQuadrantAtan2 y x := by
  by_cases hzero : x = 0
  · rw [firstQuadrantAtan2, if_pos hzero]
    positivity
  · have hxpos : 0 < x := lt_of_le_of_ne hx (Ne.symm hzero)
    simp [firstQuadrantAtan2, hzero, Real.arctan_nonneg, div_nonneg hy hxpos.le]

theorem firstQuadrantAtan2_le_pi_div_two {y x : ℝ} (_hy : 0 ≤ y)
    (_hx : 0 ≤ x) :
    firstQuadrantAtan2 y x ≤ Real.pi / 2 := by
  by_cases hzero : x = 0
  · simp [firstQuadrantAtan2, hzero]
  · simp only [firstQuadrantAtan2, hzero, if_false]
    exact (Real.arctan_lt_pi_div_two (y / x)).le

theorem firstQuadrantAtan2_sin_cos {x : ℝ} (hx0 : 0 ≤ x)
    (hxpi : x < Real.pi / 2) :
    firstQuadrantAtan2 (Real.sin x) (Real.cos x) = x := by
  have hcos : 0 < Real.cos x :=
    Real.cos_pos_of_mem_Ioo ⟨by nlinarith [Real.pi_pos], hxpi⟩
  rw [firstQuadrantAtan2, if_neg hcos.ne']
  rw [← Real.tan_eq_sin_div_cos]
  exact Real.arctan_tan (by nlinarith [Real.pi_pos]) hxpi

/-- Away from the pi boundary, `q` and `-q` select the same representative. -/
theorem principalRepresentative_neg_of_q0_ne_zero (q : Quaternion ℝ)
    (h : q.q0 ≠ 0) :
    principalRepresentative (Quaternion.neg q) = principalRepresentative q := by
  by_cases hq : q.q0 < 0
  · have hnq : ¬ (-q.q0 < 0) := by linarith
    simp [principalRepresentative, Quaternion.neg, hq, hnq]
  · have hqpos : 0 < q.q0 := lt_of_le_of_ne (le_of_not_gt hq) (Ne.symm h)
    have hnq : -q.q0 < 0 := neg_lt_zero.mpr hqpos
    simp [principalRepresentative, Quaternion.neg, hq, hnq]

@[simp]
theorem quaternionLog_identity :
    quaternionLog (Quaternion.identity : Quaternion ℝ) = 0 := by
  simp [quaternionLog]

/-- Away from the pi boundary, the principal logarithm is sign invariant. -/
theorem quaternionLog_neg_of_q0_ne_zero (q : Quaternion ℝ) (h : q.q0 ≠ 0) :
    quaternionLog (Quaternion.neg q) = quaternionLog q := by
  simp only [quaternionLog, principalRepresentative_neg_of_q0_ne_zero q h]

/-- The selected logarithm always lies in the closed principal ball. -/
theorem quaternionLog_norm_le_pi (q : Quaternion ℝ) :
    vectorNorm (quaternionLog q) ≤ Real.pi := by
  let representative := principalRepresentative q
  let qv := vectorPart representative
  let qvNorm := vectorNorm qv
  have hqvNorm : 0 ≤ qvNorm := vectorNorm_nonnegative qv
  have hq0 : 0 ≤ representative.q0 := principalRepresentative_q0_nonnegative q
  by_cases hzero : qvNorm = 0
  · simp [quaternionLog, representative, qv, qvNorm, hzero, Real.pi_pos.le]
  · have hqvNormPos : 0 < qvNorm := lt_of_le_of_ne hqvNorm (Ne.symm hzero)
    have hangleNonnegative : 0 ≤ firstQuadrantAtan2 qvNorm representative.q0 :=
      firstQuadrantAtan2_nonnegative hqvNorm hq0
    have hscale : 0 ≤ 2 * firstQuadrantAtan2 qvNorm representative.q0 / qvNorm :=
      div_nonneg (mul_nonneg (by norm_num) hangleNonnegative) hqvNorm
    rw [quaternionLog]
    simp only [representative, qv, qvNorm, hzero, if_false]
    rw [vectorNorm_vectorScale_of_nonnegative hscale]
    have hcancel :
        (2 * firstQuadrantAtan2 qvNorm representative.q0 / qvNorm) * qvNorm =
          2 * firstQuadrantAtan2 qvNorm representative.q0 := by
      field_simp
    rw [hcancel]
    nlinarith [firstQuadrantAtan2_le_pi_div_two hqvNorm hq0]

/-- Exact local inverse law on the open principal ball. -/
theorem quaternionLog_quaternionExp {phi : Vector3 ℝ}
    (hpositive : 0 < vectorNorm phi) (hprincipal : vectorNorm phi < Real.pi) :
    quaternionLog (quaternionExp phi) = phi := by
  let theta := vectorNorm phi
  have htheta : 0 < theta := hpositive
  have hthetaNe : theta ≠ 0 := htheta.ne'
  have hhalf0 : 0 ≤ theta / 2 := by positivity
  have hhalfPi : theta / 2 < Real.pi / 2 := by linarith
  have hhalfLtPi : theta / 2 < Real.pi := by nlinarith [Real.pi_pos]
  have hsin : 0 < Real.sin (theta / 2) :=
    Real.sin_pos_of_pos_of_lt_pi (by positivity) hhalfLtPi
  have hcos : 0 < Real.cos (theta / 2) :=
    Real.cos_pos_of_mem_Ioo ⟨by nlinarith [Real.pi_pos], hhalfPi⟩
  have hscale : 0 ≤ Real.sin (theta / 2) / theta :=
    div_nonneg hsin.le htheta.le
  have hqvNorm :
      vectorNorm (vectorScale (Real.sin (theta / 2) / theta) phi) =
        Real.sin (theta / 2) := by
    rw [vectorNorm_vectorScale_of_nonnegative hscale]
    change (Real.sin (theta / 2) / theta) * theta = Real.sin (theta / 2)
    field_simp
  rw [quaternionExp_of_vectorNorm_ne_zero hthetaNe]
  change quaternionLog
      (fromScalarVector (Real.cos (theta / 2))
        (vectorScale (Real.sin (theta / 2) / theta) phi)) = phi
  rw [quaternionLog]
  rw [principalRepresentative_of_q0_nonnegative hcos.le]
  simp only [vectorPart_fromScalarVector, hqvNorm, hsin.ne', if_false]
  simp only [fromScalarVector]
  rw [firstQuadrantAtan2_sin_cos hhalf0 hhalfPi]
  rw [vectorScale_vectorScale]
  have hcoefficient :
      (2 * (theta / 2) / Real.sin (theta / 2)) *
          (Real.sin (theta / 2) / theta) = 1 := by
    field_simp [hthetaNe, hsin.ne']
  rw [hcoefficient, vectorScale_one]

/-- Exact local inverse law, including the zero rotation. -/
theorem quaternionLog_quaternionExp_of_norm_lt_pi {phi : Vector3 ℝ}
    (hprincipal : vectorNorm phi < Real.pi) :
    quaternionLog (quaternionExp phi) = phi := by
  rcases (vectorNorm_nonnegative phi).eq_or_lt with hzero | hpositive
  · have hphi : phi = 0 := (vectorNorm_eq_zero_iff phi).mp hzero.symm
    subst phi
    simp
  · exact quaternionLog_quaternionExp hpositive hprincipal

/-- At zero vector norm the exact exponential and Taylor branch agree. -/
theorem exp_zero_branches_agree :
    quaternionExp (0 : Vector3 ℝ) = quaternionExpTaylorCandidate 0 := by
  simp

end SO3

end FormalESKF
