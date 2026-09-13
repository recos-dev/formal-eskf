/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.Prediction
import FormalESKF.RightJacobian
import Mathlib.Tactic.LinearCombination

/-! Exact directional derivatives in every direction, including the origin.
These concern smooth real maps, not derivatives of floating-point programs. -/

namespace FormalESKF.ESKF

noncomputable section

open SO3 Quaternion Matrix

def HasVectorDerivAt (f : ℝ → Vector3 ℝ) (d : Vector3 ℝ) (t : ℝ) : Prop :=
  ∀ i, HasDerivAt (fun x => f x i) (d i) t

theorem vectorNorm_neg (v : Vector3 ℝ) : vectorNorm (-v) = vectorNorm v := by
  simp [vectorNorm, vectorNormSquared, dotProduct, Fin.sum_univ_succ]

theorem quaternionExp_neg (v : Vector3 ℝ) : quaternionExp (-v) = conjugate (quaternionExp v) := by
  dsimp only [quaternionExp]
  rw [vectorNorm_neg]
  split_ifs <;> ext <;> simp [conjugate, identity, fromScalarVector, vectorScale]

/-- Signed-time equality is needed for a two-sided derivative at zero. -/
theorem constantRateIncrement_eq_exp_signed (w : Vector3 ℝ) (t : ℝ) :
    constantRateIncrement w t = quaternionExp (vectorScale t w) := by
  by_cases ht : 0 ≤ t
  · exact constantRateIncrement_eq_exp w ht
  · have hv : vectorScale t w = -(vectorScale (-t) w) := by funext i; simp [vectorScale]
    rw [hv, quaternionExp_neg, ← constantRateIncrement_eq_exp w (by linarith : 0 ≤ -t)]
    dsimp only [constantRateIncrement]
    split_ifs <;> ext <;>
      simp [conjugate, identity, fromScalarVector, vectorScale, mul_neg, neg_div]

@[simp] theorem constantRateIncrement_zero (w : Vector3 ℝ) : constantRateIncrement w 0 = identity := by
  rw [constantRateIncrement_eq_exp_signed]
  have hz : vectorScale 0 w = 0 := by funext i; simp [vectorScale]
  simp [hz]

theorem quaternionExp_ray_hasDerivAt_zero (v : Vector3 ℝ) :
    HasQuaternionDerivAt (fun t => quaternionExp (vectorScale t v))
      (fromScalarVector 0 (vectorScale (1 / 2) v)) 0 := by
  have h := constantRateIncrement_kinematics v 0
  simpa [constantRateIncrement_eq_exp_signed, HasQuaternionDerivAt,
    show vectorScale 0 v = 0 by funext i; simp [vectorScale], scale, fromScalarVector,
    vectorScale] using h

theorem HasVectorDerivAt.add {f g : ℝ → Vector3 ℝ} {df dg : Vector3 ℝ} {t : ℝ}
    (hf : HasVectorDerivAt f df t) (hg : HasVectorDerivAt g dg t) :
    HasVectorDerivAt (fun x => f x + g x) (df + dg) t := fun i => (hf i).add (hg i)

theorem HasVectorDerivAt.sub {f g : ℝ → Vector3 ℝ} {df dg : Vector3 ℝ} {t : ℝ}
    (hf : HasVectorDerivAt f df t) (hg : HasVectorDerivAt g dg t) :
    HasVectorDerivAt (fun x => f x - g x) (df - dg) t := fun i => (hf i).sub (hg i)

theorem HasVectorDerivAt.neg {f : ℝ → Vector3 ℝ} {df : Vector3 ℝ} {t : ℝ}
    (hf : HasVectorDerivAt f df t) : HasVectorDerivAt (fun x => -f x) (-df) t :=
  fun i => (hf i).neg

theorem HasVectorDerivAt.const_matrix {f : ℝ → Vector3 ℝ} {df : Vector3 ℝ} {t : ℝ}
    (hf : HasVectorDerivAt f df t) (A : Matrix3 ℝ) :
    HasVectorDerivAt (fun x => A *ᵥ f x) (A *ᵥ df) t := by
  intro i
  convert! ((hf 0).const_mul (A i 0)).add
      (((hf 1).const_mul (A i 1)).add ((hf 2).const_mul (A i 2))) using 1 <;>
    simp [Matrix.mulVec, dotProduct, Fin.sum_univ_succ]
  rfl

theorem HasVectorDerivAt.cross {f g : ℝ → Vector3 ℝ} {df dg : Vector3 ℝ} {t : ℝ}
    (hf : HasVectorDerivAt f df t) (hg : HasVectorDerivAt g dg t) :
    HasVectorDerivAt (fun x => SO3.cross (f x) (g x))
      (SO3.cross df (g t) + SO3.cross (f t) dg) t := by
  intro i
  fin_cases i
  · exact (((hf 1).mul (hg 2)).sub ((hf 2).mul (hg 1))).congr_deriv (by simp [SO3.cross]; ring)
  · exact (((hf 2).mul (hg 0)).sub ((hf 0).mul (hg 2))).congr_deriv (by simp [SO3.cross]; ring)
  · exact (((hf 0).mul (hg 1)).sub ((hf 1).mul (hg 0))).congr_deriv (by simp [SO3.cross]; ring)

theorem vectorLine_hasDerivAt (a v : Vector3 ℝ) (t : ℝ) :
    HasVectorDerivAt (fun x => a + vectorScale x v) v t := by
  intro i
  exact ((hasDerivAt_const t (a i)).add ((hasDerivAt_id t).mul_const (v i))).congr_deriv
    (by simp)

theorem vectorNorm_line_hasDerivAt {a : Vector3 ℝ} (ha : vectorNorm a ≠ 0) (v : Vector3 ℝ) :
    HasDerivAt (fun t => vectorNorm (a + vectorScale t v)) ((a ⬝ᵥ v) / vectorNorm a) 0 := by
  have hl := vectorLine_hasDerivAt a v 0
  have hs : HasDerivAt (fun t => vectorNormSquared (a + vectorScale t v)) (2 * (a ⬝ᵥ v)) 0 := by
    convert! ((hl 0).mul (hl 0)).add (((hl 1).mul (hl 1)).add ((hl 2).mul (hl 2))) using 1
    · funext t
      simp [vectorNormSquared, dotProduct, Fin.sum_univ_succ]
    · simp [vectorScale, dotProduct, Fin.sum_univ_succ]
      ring
  have hn : vectorNormSquared (a + vectorScale 0 v) ≠ 0 := by
    have hz : vectorScale 0 v = 0 := by funext i; simp [vectorScale]
    simpa [hz, ← vectorNorm_squared] using pow_ne_zero 2 ha
  exact (hs.sqrt hn).congr_deriv (by
    have hz : a + vectorScale 0 v = a := by ext i; simp [vectorScale]
    rw [hz]
    change 2 * (a ⬝ᵥ v) / (2 * vectorNorm a) = _
    ring)

/-- Coefficient derivative away from zero, prior to identifying its tangent
with the right Jacobian. -/
def expCoefficientDerivative (a v : Vector3 ℝ) : Quaternion ℝ :=
  let theta := vectorNorm a
  let d := (a ⬝ᵥ v) / theta
  fromScalarVector (-Real.sin (theta / 2) * (d / 2))
    (fun i => (Real.cos (theta / 2) * (d / 2) * theta - Real.sin (theta / 2) * d) /
      theta ^ 2 * a i + Real.sin (theta / 2) / theta * v i)

theorem quaternionExp_line_hasDerivAt_of_ne_zero {a : Vector3 ℝ}
    (ha : vectorNorm a ≠ 0) (v : Vector3 ℝ) :
    HasQuaternionDerivAt (fun t => quaternionExp (a + vectorScale t v))
      (expCoefficientDerivative a v) 0 := by
  have hl := vectorLine_hasDerivAt a v 0
  have hn := vectorNorm_line_hasDerivAt ha v
  have hz : a + vectorScale 0 v = a := by ext i; simp [vectorScale]
  have he : ∀ᶠ t in nhds (0 : ℝ), vectorNorm (a + vectorScale t v) ≠ 0 :=
    hn.continuousAt.eventually_ne (by simpa [hz] using ha)
  have hcos := (hn.div_const 2).cos
  have hscale := ((hn.div_const 2).sin).div hn (by simpa [hz] using ha)
  have h0 : HasDerivAt (fun t => (quaternionExp (a + vectorScale t v)).q0)
      (expCoefficientDerivative a v).q0 0 := by
    apply (hcos.congr_deriv (by simp [hz, expCoefficientDerivative, fromScalarVector])).congr_of_eventuallyEq
    filter_upwards [he] with t ht
    simp [quaternionExp, ht, fromScalarVector]
  have hv (i : Fin 3) : HasDerivAt (fun t => (vectorPart (quaternionExp (a + vectorScale t v))) i)
      (vectorPart (expCoefficientDerivative a v) i) 0 := by
    have hvi := (hscale.mul (hl i)).congr_deriv
      (g' := vectorPart (expCoefficientDerivative a v) i)
      (by simp [hz, expCoefficientDerivative, vectorPart_fromScalarVector, vectorScale])
    apply hvi.congr_of_eventuallyEq
    filter_upwards [he] with t ht
    simp [quaternionExp, ht, vectorPart_fromScalarVector, vectorScale]
  exact ⟨h0, hv 0, hv 1, hv 2⟩

theorem expCoefficientDerivative_right_trivialized {a : Vector3 ℝ}
    (ha : vectorNorm a ≠ 0) (v : Vector3 ℝ) :
    hamiltonMul (conjugate (quaternionExp a)) (expCoefficientDerivative a v) =
      fromScalarVector 0 (vectorScale (1 / 2) (rightJacobian a *ᵥ v)) := by
  rw [quaternionExp_of_vectorNorm_ne_zero ha, rightJacobian_halfAngle ha]
  have hs := Real.sin_sq_add_cos_sq (vectorNorm a / 2)
  have hn := vectorNorm_squared a
  simp [vectorNormSquared, dotProduct, Fin.sum_univ_succ] at hn
  ext <;> simp [hamiltonMul, conjugate, expCoefficientDerivative, fromScalarVector,
    vectorScale, Matrix.mulVec, dotProduct, Fin.sum_univ_succ, outer, hat, Matrix.one_apply] <;>
    field_simp
  · linear_combination
      -(vectorNorm a * Real.sin (vectorNorm a / 2) * Real.cos (vectorNorm a / 2) -
        2 * Real.sin (vectorNorm a / 2) ^ 2) *
        (a 0 * v 0 + a 1 * v 1 + a 2 * v 2) * hn
  · linear_combination vectorNorm a ^ 2 * a 0 * (a 0 * v 0 + a 1 * v 1 + a 2 * v 2) * hs
  · linear_combination vectorNorm a ^ 2 * a 1 * (a 0 * v 0 + a 1 * v 1 + a 2 * v 2) * hs
  · linear_combination vectorNorm a ^ 2 * a 2 * (a 0 * v 0 + a 1 * v 1 + a 2 * v 2) * hs

/-- Right-trivialized differential at ANY injected correction a. The curve
starts at identity, and its tangent coordinates are J_r(a)*v. This gives the
quaternion directional derivative; `ResetDifferential` proves the full
Fréchet derivative after applying the local principal Log. -/
theorem resetAttitude_hasDerivAt (a v : Vector3 ℝ) :
    HasQuaternionDerivAt
      (fun t => hamiltonMul (conjugate (quaternionExp a)) (quaternionExp (a + vectorScale t v)))
      (fromScalarVector 0 (vectorScale (1 / 2) (rightJacobian a *ᵥ v))) 0 := by
  by_cases ha : vectorNorm a = 0
  · have haz := (vectorNorm_eq_zero_iff a).mp ha
    subst a
    have hc : conjugate (identity : Quaternion ℝ) = identity := by simp [conjugate, identity]
    simpa [hc] using quaternionExp_ray_hasDerivAt_zero v
  · have h := (quaternionExp_line_hasDerivAt_of_ne_zero ha v).hamilton_left
      (conjugate (quaternionExp a))
    rw [expCoefficientDerivative_right_trivialized ha v] at h
    exact h

theorem resetAttitude_zero (a : Vector3 ℝ) :
    hamiltonMul (conjugate (quaternionExp a)) (quaternionExp (a + vectorScale 0 a)) = identity := by
  have hz : vectorScale 0 a = 0 := by ext i; simp [vectorScale]
  simp [hz, conjugate_hamiltonMul_of_unit (quaternionExp_hasUnitNorm a)]

/-- The finite correction a is not linearized here: J_r(a) is the exact
differential. Covariance propagation still uses only this first derivative. -/
theorem resetAttitude_tangent_matches_exp (a v : Vector3 ℝ) :
    HasQuaternionDerivAt (fun t => quaternionExp (vectorScale t (rightJacobian a *ᵥ v)))
      (fromScalarVector 0 (vectorScale (1 / 2) (rightJacobian a *ᵥ v))) 0 :=
  quaternionExp_ray_hasDerivAt_zero _

theorem HasQuaternionDerivAt.hamilton_right {f : ℝ → Quaternion ℝ} {d : Quaternion ℝ} {t : ℝ}
    (h : HasQuaternionDerivAt f d t) (q : Quaternion ℝ) :
    HasQuaternionDerivAt (fun x => hamiltonMul (f x) q) (hamiltonMul d q) t := by
  rcases h with ⟨h0, h1, h2, h3⟩
  exact ⟨(((h0.mul_const q.q0).sub (h1.mul_const q.q1)).sub (h2.mul_const q.q2)).sub (h3.mul_const q.q3),
    (((h0.mul_const q.q1).add (h1.mul_const q.q0)).add (h2.mul_const q.q3)).sub (h3.mul_const q.q2),
    (((h0.mul_const q.q2).sub (h1.mul_const q.q3)).add (h2.mul_const q.q0)).add (h3.mul_const q.q1),
    (((h0.mul_const q.q3).add (h1.mul_const q.q2)).sub (h2.mul_const q.q1)).add (h3.mul_const q.q0)⟩

theorem quaternionConjugation_pure (q : Quaternion ℝ) (v : Vector3 ℝ) :
    hamiltonMul (conjugate q) (hamiltonMul (fromScalarVector 0 v) q) =
      fromScalarVector 0 (inverseRotate q v) := by
  ext <;> simp [hamiltonMul, conjugate, fromScalarVector, inverseRotate,
    rotationMatrix, Matrix.mulVec, dotProduct, Fin.sum_univ_succ] <;> ring

/-- Frozen-rate attitude error is transported by the inverse increment,
which explains the transpose in covariance prediction's Exp branch. -/
theorem conjugatedIncrement_hasDerivAt (q : Quaternion ℝ) (v : Vector3 ℝ) :
    HasQuaternionDerivAt
      (fun t => hamiltonMul (conjugate q) (hamiltonMul (quaternionExp (vectorScale t v)) q))
      (fromScalarVector 0 (vectorScale (1 / 2) (inverseRotate q v))) 0 := by
  have h := ((quaternionExp_ray_hasDerivAt_zero v).hamilton_right q).hamilton_left (conjugate q)
  rw [quaternionConjugation_pure] at h
  have he : inverseRotate q (vectorScale (1 / 2) v) = vectorScale (1 / 2) (inverseRotate q v) := by
    change (rotationMatrix q)ᵀ *ᵥ ((1 / 2 : ℝ) • v) = (1 / 2 : ℝ) • ((rotationMatrix q)ᵀ *ᵥ v)
    exact Matrix.mulVec_smul _ _ _
  rw [he] at h
  exact h

theorem rotationMatrix_hasDerivAt_identity {q : ℝ → Quaternion ℝ} (v : Vector3 ℝ)
    (hq : q 0 = identity)
    (hd : HasQuaternionDerivAt q (fromScalarVector 0 (vectorScale (1 / 2) v)) 0)
    (i j : Fin 3) : HasDerivAt (fun t => rotationMatrix (q t) i j) (hat v i j) 0 := by
  rcases hd with ⟨h0, h1, h2, h3⟩
  fin_cases i <;> fin_cases j
  · convert! (((h0.mul h0).add (h1.mul h1)).sub (h2.mul h2)).sub (h3.mul h3) using 1
    simp [hq, identity, fromScalarVector, vectorScale, hat]
  · convert! ((h1.mul h2).sub (h0.mul h3)).const_mul 2 using 1
    simp [hq, identity, fromScalarVector, vectorScale, hat]
  · convert! ((h1.mul h3).add (h0.mul h2)).const_mul 2 using 1
    simp [hq, identity, fromScalarVector, vectorScale, hat]
  · convert! ((h1.mul h2).add (h0.mul h3)).const_mul 2 using 1
    simp [hq, identity, fromScalarVector, vectorScale, hat]
  · convert! (((h0.mul h0).sub (h1.mul h1)).add (h2.mul h2)).sub (h3.mul h3) using 1
    simp [hq, identity, fromScalarVector, vectorScale, hat]
  · convert! ((h2.mul h3).sub (h0.mul h1)).const_mul 2 using 1
    simp [hq, identity, fromScalarVector, vectorScale, hat]
  · convert! ((h1.mul h3).sub (h0.mul h2)).const_mul 2 using 1
    simp [hq, identity, fromScalarVector, vectorScale, hat]
  · convert! ((h2.mul h3).add (h0.mul h1)).const_mul 2 using 1
    simp [hq, identity, fromScalarVector, vectorScale, hat]
  · convert! (((h0.mul h0).sub (h1.mul h1)).sub (h2.mul h2)).add (h3.mul h3) using 1
    simp [hq, identity, fromScalarVector, vectorScale, hat]

theorem matrixVector_hasDerivAt {A : ℝ → Matrix3 ℝ} {dA : Matrix3 ℝ}
    {v : ℝ → Vector3 ℝ} {dv : Vector3 ℝ} {t : ℝ}
    (hA : ∀ i j, HasDerivAt (fun x => A x i j) (dA i j) t) (hv : HasVectorDerivAt v dv t) :
    HasVectorDerivAt (fun x => A x *ᵥ v x) (dA *ᵥ v t + A t *ᵥ dv) t := by
  intro i
  convert! ((hA i 0).mul (hv 0)).add (((hA i 1).mul (hv 1)).add ((hA i 2).mul (hv 2))) using 1
  · funext x
    simp [Matrix.mulVec, dotProduct, Fin.sum_univ_succ]
  · simp [Matrix.mulVec, dotProduct, Fin.sum_univ_succ]
    ring

/-- Local inverse action has +hat(h), not -hat(h), as the observation Jacobian. -/
theorem inverseRotate_exp_hasDerivAt {v : ℝ → Vector3 ℝ} {dv : Vector3 ℝ}
    (hv : HasVectorDerivAt v dv 0) (d : Vector3 ℝ) :
    HasVectorDerivAt (fun t => inverseRotate (quaternionExp (vectorScale t d)) (v t))
      (hat (v 0) *ᵥ d + dv) 0 := by
  have hz : quaternionExp (vectorScale 0 d) = identity := by
    rw [← constantRateIncrement_eq_exp_signed, constantRateIncrement_zero]
  have hR := rotationMatrix_hasDerivAt_identity d hz (quaternionExp_ray_hasDerivAt_zero d)
  have h := matrixVector_hasDerivAt (A := fun t => (rotationMatrix (quaternionExp (vectorScale t d)))ᵀ)
    (dA := (hat d)ᵀ) (fun i j => hR j i) hv
  have he : (hat d)ᵀ *ᵥ v 0 = hat (v 0) *ᵥ d := by
    rw [transpose_hat, Matrix.neg_mulVec, hat_mulVec, hat_mulVec, cross_antisymm]
    simp
  simpa only [inverseRotate, hz, rotationMatrix_identity, transpose_one, one_mulVec, he] using! h

end

end FormalESKF.ESKF
