/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.RightJacobian
import Mathlib.Analysis.Calculus.Deriv.Slope

/-! Removable singularities and first-order behavior of Sola (183)/(294).
Asymptotic real remainders are not uniform IEEE-754 error bounds. -/

namespace FormalESKF.SO3

noncomputable section

open Filter Matrix
open scoped Topology

def rightJacobianA (t : ℝ) : ℝ := if t = 0 then 1 / 2 else (1 - Real.cos t) / t ^ 2
def rightJacobianB (t : ℝ) : ℝ := if t = 0 then 1 / 6 else (t - Real.sin t) / t ^ 3

theorem rightJacobianA_tendsto_zero : Tendsto rightJacobianA (nhds 0) (nhds (1 / 2)) := by
  have hp : (fun x : ℝ => taylorWithinEval Real.cos 2 Set.univ 0 x) =
      fun x => 1 - x ^ 2 / 2 := by
    funext x
    simp [taylorWithinEval_succ]
    ring
  have h := (taylor_isLittleO_univ (f := Real.cos) (x₀ := 0) (n := 2)
    Real.contDiff_cos).tendsto_div_nhds_zero
  simp only [hp, sub_zero] at h
  have he (x : ℝ) : rightJacobianA x = 1 / 2 - (Real.cos x - (1 - x ^ 2 / 2)) / x ^ 2 := by
    by_cases hx : x = 0
    · simp [rightJacobianA, hx]
    · simp only [rightJacobianA, if_neg hx]
      field_simp
      ring
  simpa only [sub_zero, ← he] using (tendsto_const_nhds (x := (1 / 2 : ℝ))).sub h

theorem rightJacobianB_tendsto_zero : Tendsto rightJacobianB (nhds 0) (nhds (1 / 6)) := by
  have hp : (fun x : ℝ => taylorWithinEval Real.sin 3 Set.univ 0 x) =
      fun x => x - x ^ 3 / 6 := by
    funext x
    simp [taylorWithinEval_succ]
    ring
  have h := (taylor_isLittleO_univ (f := Real.sin) (x₀ := 0) (n := 3)
    Real.contDiff_sin).tendsto_div_nhds_zero
  simp only [hp, sub_zero] at h
  have he (x : ℝ) : rightJacobianB x = 1 / 6 - (Real.sin x - (x - x ^ 3 / 6)) / x ^ 3 := by
    by_cases hx : x = 0
    · simp [rightJacobianB, hx]
    · simp only [rightJacobianB, if_neg hx]
      field_simp
      ring
  simpa only [sub_zero, ← he] using (tendsto_const_nhds (x := (1 / 6 : ℝ))).sub h

theorem rightJacobian_regularized (a : Vector3 ℝ) :
    rightJacobian a = 1 - rightJacobianA (vectorNorm a) • hat a +
      rightJacobianB (vectorNorm a) • (hat a * hat a) := by
  by_cases h : vectorNorm a = 0
  · have ha := (vectorNorm_eq_zero_iff a).mp h
    simp [ha]
  · simp [rightJacobian, rightJacobianA, rightJacobianB, h]

theorem vectorNorm_ray_tendsto_zero (v : Vector3 ℝ) :
    Tendsto (fun t => vectorNorm (vectorScale t v)) (nhds 0) (nhds 0) := by
  have hc : Continuous (fun t => vectorNorm (vectorScale t v)) := by
    simp only [vectorNorm, vectorNormSquared, vectorScale, dotProduct, Fin.sum_univ_succ]
    fun_prop
  simpa [vectorScale, vectorNorm, vectorNormSquared, dotProduct, Fin.sum_univ_succ] using hc.tendsto 0

/-- Every matrix entry has derivative -hat(v)/2 along a=t*v at t=0.
Thus the approximate reset has the correct first-order sign and factor. -/
theorem rightJacobian_hasDerivAt_zero (v : Vector3 ℝ) (i j : Fin 3) :
    HasDerivAt (fun t => rightJacobian (vectorScale t v) i j) (-(hat v i j) / 2) 0 := by
  have hn := vectorNorm_ray_tendsto_zero v
  have hA := rightJacobianA_tendsto_zero.comp hn
  have hB := rightJacobianB_tendsto_zero.comp hn
  have hlim := ((hA.mul_const (hat v i j)).neg).add
    ((tendsto_id.mul hB).mul_const ((hat v * hat v) i j))
  apply hasDerivAt_iff_tendsto_slope.mpr
  have he : slope (fun t => rightJacobian (vectorScale t v) i j) 0 =ᶠ[nhdsWithin 0 {0}ᶜ]
      (fun t => -(rightJacobianA (vectorNorm (vectorScale t v)) * hat v i j) +
        t * rightJacobianB (vectorNorm (vectorScale t v)) * (hat v * hat v) i j) := by
    filter_upwards [self_mem_nhdsWithin] with t ht
    have ht0 : t ≠ 0 := ht
    have hz : vectorScale 0 v = 0 := by ext k; simp [vectorScale]
    simp [slope, hz, rightJacobian_regularized, hat_scale,
      Matrix.sub_apply, Matrix.add_apply, Matrix.smul_apply]
    field_simp
    ring
  apply Tendsto.congr' he.symm
  convert! hlim.mono_left nhdsWithin_le_nhds using 1
  simp
  ring

theorem rightJacobianFirstOrder_hasDerivAt_zero (v : Vector3 ℝ) (i j : Fin 3) :
    HasDerivAt (fun t => rightJacobianFirstOrder (vectorScale t v) i j) (-(hat v i j) / 2) 0 := by
  have he : (fun t => rightJacobianFirstOrder (vectorScale t v) i j) =
      fun t => (1 : Matrix3 ℝ) i j - (1 / 2) * t * hat v i j := by
    funext t
    simp [rightJacobianFirstOrder, hat_scale]
    ring
  rw [he]
  exact ((hasDerivAt_const 0 ((1 : Matrix3 ℝ) i j)).sub
    (((hasDerivAt_id 0).const_mul (1 / 2)).mul_const (hat v i j))).congr_deriv (by ring)

/-- The small-angle coefficient A polynomial in `right_jacobian.hpp` has
remainder o(theta^6). This is an asymptotic statement, not a bound over the
whole executable Taylor branch or a bound on library sin/cos rounding. -/
theorem rightJacobianTaylorA_remainder :
    (fun t => rightJacobianA t - rightJacobianTaylorA (t ^ 2)) =o[nhds 0] (fun t => t ^ 6) := by
  have hp : (fun x : ℝ => taylorWithinEval Real.cos 8 Set.univ 0 x) =
      fun x => 1 - x ^ 2 / 2 + x ^ 4 / 24 - x ^ 6 / 720 + x ^ 8 / 40320 := by
    funext x
    simp [taylorWithinEval_succ]
    ring
  have h := (taylor_isLittleO_univ (f := Real.cos) (x₀ := 0) (n := 8)
    Real.contDiff_cos).tendsto_div_nhds_zero
  simp only [hp, sub_zero] at h
  apply (Asymptotics.isLittleO_iff_tendsto (fun t ht => by
    have hz : t = 0 := (pow_eq_zero_iff (by decide : 6 ≠ 0)).mp ht
    simp [hz, rightJacobianA, rightJacobianTaylorA])).mpr
  have he (t : ℝ) : (rightJacobianA t - rightJacobianTaylorA (t ^ 2)) / t ^ 6 =
      -((Real.cos t - (1 - t ^ 2 / 2 + t ^ 4 / 24 - t ^ 6 / 720 + t ^ 8 / 40320)) / t ^ 8) := by
    by_cases ht : t = 0
    · simp [ht]
    · simp only [rightJacobianA, if_neg ht, rightJacobianTaylorA]
      field_simp
      ring
  simpa only [neg_zero, he] using h.neg

theorem rightJacobianTaylorB_remainder :
    (fun t => rightJacobianB t - rightJacobianTaylorB (t ^ 2)) =o[nhds 0] (fun t => t ^ 6) := by
  have hp : (fun x : ℝ => taylorWithinEval Real.sin 9 Set.univ 0 x) =
      fun x => x - x ^ 3 / 6 + x ^ 5 / 120 - x ^ 7 / 5040 + x ^ 9 / 362880 := by
    funext x
    simp [taylorWithinEval_succ]
    ring
  have h := (taylor_isLittleO_univ (f := Real.sin) (x₀ := 0) (n := 9)
    Real.contDiff_sin).tendsto_div_nhds_zero
  simp only [hp, sub_zero] at h
  apply (Asymptotics.isLittleO_iff_tendsto (fun t ht => by
    have hz : t = 0 := (pow_eq_zero_iff (by decide : 6 ≠ 0)).mp ht
    simp [hz, rightJacobianB, rightJacobianTaylorB])).mpr
  have he (t : ℝ) : (rightJacobianB t - rightJacobianTaylorB (t ^ 2)) / t ^ 6 =
      -((Real.sin t - (t - t ^ 3 / 6 + t ^ 5 / 120 - t ^ 7 / 5040 + t ^ 9 / 362880)) / t ^ 9) := by
    by_cases ht : t = 0
    · simp [ht]
    · simp only [rightJacobianB, if_neg ht, rightJacobianTaylorB]
      field_simp
      ring
  simpa only [neg_zero, he] using h.neg

end

end FormalESKF.SO3
