/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.RotationDifferential
import Mathlib.Analysis.Calculus.DSlope
import Mathlib.Analysis.Calculus.LineDeriv.Basic
import Mathlib.Analysis.Normed.Module.FiniteDimension

/-!
# Reset in local rotation-vector coordinates

Fréchet differentiation of Log(Exp(-a) * Exp(a + epsilon)) at epsilon=0.
The principal Log is used only near identity, not assumed globally smooth.
These are exact-real maps; numerical approximation and checked-API failures
remain separate obligations.
-/

namespace FormalESKF.ESKF

noncomputable section

open SO3 Quaternion Matrix Filter Asymptotics
open scoped Topology

private theorem vectorNorm_continuous : Continuous (vectorNorm : Vector3 ℝ → ℝ) := by
  change Continuous (fun v : Vector3 ℝ => √(vectorNormSquared v))
  simp only [vectorNormSquared, dotProduct, Fin.sum_univ_succ]
  fun_prop

/-- Compare the Euclidean norm used by Exp with the Pi-space norm used by
Mathlib's Fréchet derivative. Only a uniform bound is needed here. -/
private theorem vectorNorm_isBigO : (vectorNorm : Vector3 ℝ → ℝ) =O[nhds 0] id := by
  apply isBigO_of_le' (c := 3)
  intro v
  have h (i : Fin 3) : (v i) ^ 2 ≤ ‖v‖ ^ 2 := by
    have hi := norm_le_pi_norm v i
    simpa [Real.norm_eq_abs, sq_abs] using pow_le_pow_left₀ (norm_nonneg (v i)) hi 2
  have hs := vectorNorm_squared v
  simp [vectorNormSquared, dotProduct, Fin.sum_univ_succ] at hs
  rw [Real.norm_eq_abs, abs_of_nonneg (vectorNorm_nonnegative v)]
  change vectorNorm v ≤ 3 * ‖v‖
  nlinarith [h 0, h 1, h 2, vectorNorm_nonnegative v, norm_nonneg v]

/-- A continuous coefficient multiplying a vanishing differentiable vector
needs no derivative of the coefficient. This handles radial singularities. -/
private theorem continuous_smul_hasFDerivAt_zero
    {E F : Type*} [NormedAddCommGroup E] [NormedSpace ℝ E]
    [NormedAddCommGroup F] [NormedSpace ℝ F]
    {c : E → ℝ} {f : E → F} {L : E →L[ℝ] F} {x : E}
    (hc : ContinuousAt c x) (hf : HasFDerivAt f L x) (hz : f x = 0) :
    HasFDerivAt (fun y => c y • f y) (c x • L) x := by
  have hc0 : (fun y => c y - c x) =o[nhds x] (fun _ => (1 : ℝ)) :=
    (isLittleO_one_iff ℝ).mpr (by
      simpa using hc.tendsto.sub (tendsto_const_nhds (x := c x)))
  have hb : f =O[nhds x] (fun y => y - x) := by simpa [hz] using hf.isBigO_sub
  have hp : (fun y => (c y - c x) • f y) =o[nhds x] (fun y => y - x) := by
    simpa using hc0.smul_isBigO hb
  have h := hp.add (hf.isLittleO.const_smul_left (c x))
  apply HasFDerivAt.of_isLittleO
  convert! h using 1
  funext y
  simp [hz, sub_smul, smul_sub]

private theorem radial_zero_hasFDerivAt {f : ℝ → ℝ} (hf : HasDerivAt f 0 0) :
    HasFDerivAt (fun v : Vector3 ℝ => f (vectorNorm v)) (0 : Vector3 ℝ →L[ℝ] ℝ) 0 := by
  have h := hf.isLittleO.comp_tendsto
    (show Tendsto vectorNorm (nhds (0 : Vector3 ℝ)) (nhds 0) by
      simpa using vectorNorm_continuous.continuousAt.tendsto (x := (0 : Vector3 ℝ)))
  have hr : (fun v : Vector3 ℝ => f (vectorNorm v) - f 0) =o[nhds 0] vectorNorm := by
    simpa [Function.comp_def] using! h
  apply HasFDerivAt.of_isLittleO
  simpa using! hr.trans_isBigO vectorNorm_isBigO

private def expScale (t : ℝ) : ℝ := dslope (fun x : ℝ => Real.sin (x / 2)) 0 t

private theorem sin_half_hasDerivAt_zero : HasDerivAt (fun x : ℝ => Real.sin (x / 2)) (1 / 2) 0 := by
  simpa using ((hasDerivAt_id (0 : ℝ)).div_const 2).sin

private theorem quaternionExp_regularized (a : Vector3 ℝ) :
    quaternionExp a = fromScalarVector (Real.cos (vectorNorm a / 2))
      (vectorScale (expScale (vectorNorm a)) a) := by
  by_cases h : vectorNorm a = 0
  · have ha := (vectorNorm_eq_zero_iff a).mp h
    simp [ha, fromScalarVector, identity]
  · rw [quaternionExp_of_vectorNorm_ne_zero h]
    simp [expScale, dslope_of_ne _ h, slope, div_eq_mul_inv, mul_comm]

private theorem vectorNorm_differentiableAt {a : Vector3 ℝ} (ha : vectorNorm a ≠ 0) :
    DifferentiableAt ℝ vectorNorm a := by
  have hs : vectorNormSquared a ≠ 0 := by
    intro h
    exact ha (by simp [vectorNorm, h])
  have hd : DifferentiableAt ℝ (fun v : Vector3 ℝ => vectorNormSquared v) a := by
    simp only [vectorNormSquared, dotProduct, Fin.sum_univ_succ]
    fun_prop
  simpa [vectorNorm] using! hd.sqrt hs

private theorem exp_scalar_differentiableAt (a : Vector3 ℝ) :
    DifferentiableAt ℝ (fun v => (quaternionExp v).q0) a := by
  simp only [quaternionExp_regularized, fromScalarVector]
  by_cases h : vectorNorm a = 0
  · have ha := (vectorNorm_eq_zero_iff a).mp h
    subst a
    apply (radial_zero_hasFDerivAt (f := fun t => Real.cos (t / 2)) _).differentiableAt
    simpa using ((hasDerivAt_id (0 : ℝ)).div_const 2).cos
  · simpa [div_eq_mul_inv] using! ((vectorNorm_differentiableAt h).mul_const (2 : ℝ)⁻¹).cos

private theorem exp_vector_differentiableAt (a : Vector3 ℝ) :
    DifferentiableAt ℝ (fun v => vectorPart (quaternionExp v)) a := by
  simp only [quaternionExp_regularized, vectorPart_fromScalarVector]
  change DifferentiableAt ℝ (fun v : Vector3 ℝ => expScale (vectorNorm v) • v) a
  by_cases h : vectorNorm a = 0
  · have ha := (vectorNorm_eq_zero_iff a).mp h
    subst a
    have hc : ContinuousAt expScale 0 := continuousAt_dslope_same.mpr
      sin_half_hasDerivAt_zero.differentiableAt
    have hcn : ContinuousAt (fun v : Vector3 ℝ => expScale (vectorNorm v)) 0 := by
      exact hc.comp_of_eq vectorNorm_continuous.continuousAt vectorNorm_zero
    exact (continuous_smul_hasFDerivAt_zero hcn (hasFDerivAt_id (0 : Vector3 ℝ)) rfl).differentiableAt
  · have hd : DifferentiableAt ℝ expScale (vectorNorm a) :=
      differentiableAt_dslope_of_ne h |>.mpr (by fun_prop)
    exact (hd.comp a (vectorNorm_differentiableAt h)).smul differentiableAt_id

/-- Relative quaternion after injecting a fixed correction a, parameterized
by the remaining additive error epsilon in the old rotation-vector chart. -/
def resetQuaternion (a epsilon : Vector3 ℝ) : Quaternion ℝ :=
  hamiltonMul (conjugate (quaternionExp a)) (quaternionExp (a + epsilon))

@[simp] theorem resetQuaternion_zero (a : Vector3 ℝ) : resetQuaternion a 0 = identity := by
  simp [resetQuaternion, conjugate_hamiltonMul_of_unit (quaternionExp_hasUnitNorm a)]

private theorem resetQuaternion_differentiableAt (a : Vector3 ℝ) :
    DifferentiableAt ℝ (fun e => (resetQuaternion a e).q0) 0 ∧
      DifferentiableAt ℝ (fun e => vectorPart (resetQuaternion a e)) 0 := by
  have hl : DifferentiableAt ℝ (fun e : Vector3 ℝ => a + e) 0 := by fun_prop
  have h0 : DifferentiableAt ℝ (fun e => (quaternionExp (a + e)).q0) 0 := by
    simpa using! (exp_scalar_differentiableAt (a + 0)).comp (0 : Vector3 ℝ) hl
  have hv : DifferentiableAt ℝ (fun e => vectorPart (quaternionExp (a + e))) 0 := by
    simpa using! (exp_vector_differentiableAt (a + 0)).comp (0 : Vector3 ℝ) hl
  have h1 : DifferentiableAt ℝ (fun e => (quaternionExp (a + e)).q1) 0 := by
    simpa [vectorPart] using! differentiableAt_pi.mp hv 0
  have h2 : DifferentiableAt ℝ (fun e => (quaternionExp (a + e)).q2) 0 := by
    simpa [vectorPart] using! differentiableAt_pi.mp hv 1
  have h3 : DifferentiableAt ℝ (fun e => (quaternionExp (a + e)).q3) 0 := by
    simpa [vectorPart] using! differentiableAt_pi.mp hv 2
  constructor
  · simp only [resetQuaternion, hamiltonMul, conjugate]
    fun_prop
  · apply differentiableAt_pi.mpr
    intro i
    fin_cases i <;> simp [vectorPart, resetQuaternion, hamiltonMul, conjugate] <;> fun_prop

/-- Matrix action as a continuous linear map for the Fréchet statement. -/
def rightJacobianLinearMap (a : Vector3 ℝ) : Vector3 ℝ →L[ℝ] Vector3 ℝ :=
  (Matrix.toLin' (rightJacobian a)).toContinuousLinearMap

@[simp] theorem rightJacobianLinearMap_apply (a v : Vector3 ℝ) :
    rightJacobianLinearMap a v = rightJacobian a *ᵥ v := rfl

theorem resetQuaternion_vector_hasFDerivAt (a : Vector3 ℝ) :
    HasFDerivAt (fun e => vectorPart (resetQuaternion a e))
      ((1 / 2 : ℝ) • rightJacobianLinearMap a) 0 := by
  have hd := (resetQuaternion_differentiableAt a).2.hasFDerivAt
  apply hd.congr_fderiv
  ext v i
  have h := (hasFDerivAt_pi'.mp hd i).hasLineDerivAt v
  have hr := resetAttitude_hasDerivAt a v
  have hh : HasDerivAt (fun t => vectorPart (resetQuaternion a (vectorScale t v)) i)
      ((fderiv ℝ (fun e => vectorPart (resetQuaternion a e)) 0) v i) 0 := by
    simpa [HasLineDerivAt, vectorScale] using! h
  apply hh.unique
  fin_cases i
  · simpa [resetQuaternion, vectorPart, fromScalarVector, vectorScale] using! hr.2.1
  · simpa [resetQuaternion, vectorPart, fromScalarVector, vectorScale] using! hr.2.2.1
  · simpa [resetQuaternion, vectorPart, fromScalarVector, vectorScale] using! hr.2.2.2

private def logScale (q : Quaternion ℝ) : ℝ :=
  2 / q.q0 * dslope Real.arctan 0 (vectorNorm (vectorPart q) / q.q0)

/-- A regularized expression for the EXISTING principal Log on its positive
scalar chart. In particular it agrees at zero vector part. -/
private theorem quaternionLog_regularized {q : Quaternion ℝ} (hq : 0 < q.q0) :
    quaternionLog q = logScale q • vectorPart q := by
  have hp : principalRepresentative q = q := by
    simp [principalRepresentative, not_lt.mpr hq.le]
  have hq0 := ne_of_gt hq
  simp only [quaternionLog, hp]
  by_cases hn : vectorNorm (vectorPart q) = 0
  · have hv := (vectorNorm_eq_zero_iff _).mp hn
    simp [hv]
  · have hd := div_ne_zero hn hq0
    simp only [if_neg hn, firstQuadrantAtan2, if_neg hq0, logScale, dslope_of_ne _ hd]
    ext i
    simp [vectorScale, slope]
    field_simp
    simp

private theorem logScale_reset_continuousAt (a : Vector3 ℝ) :
    ContinuousAt (fun e => logScale (resetQuaternion a e)) 0 := by
  have h0 := (resetQuaternion_differentiableAt a).1.continuousAt
  have hv := (resetQuaternion_vector_hasFDerivAt a).continuousAt
  have hn := vectorNorm_continuous.continuousAt.comp hv
  have hr : ContinuousAt
      (fun e => vectorNorm (vectorPart (resetQuaternion a e)) / (resetQuaternion a e).q0) 0 :=
    hn.div h0 (by simp [identity])
  have hd : ContinuousAt (dslope Real.arctan 0) 0 :=
    continuousAt_dslope_same.mpr (Real.differentiable_arctan 0)
  have hc := hd.comp_of_eq hr (by
    simp only [resetQuaternion_zero, vectorPart_identity, vectorNorm_zero, zero_div])
  exact (continuousAt_const.div h0 (by simp [identity])).mul hc

private theorem logScale_identity : logScale identity = 2 := by
  rw [logScale, vectorPart_identity, vectorNorm_zero]
  simp [identity, dslope_same, Real.deriv_arctan]

/-- Post-injection error in the principal local rotation-vector chart. -/
def resetRotationVector (a epsilon : Vector3 ℝ) : Vector3 ℝ :=
  quaternionLog (resetQuaternion a epsilon)

theorem resetRotationVector_eq (a epsilon : Vector3 ℝ) :
    resetRotationVector a epsilon =
      quaternionLog (hamiltonMul (quaternionExp (-a)) (quaternionExp (a + epsilon))) := by
  rw [quaternionExp_neg]
  rfl

@[simp] theorem resetRotationVector_zero (a : Vector3 ℝ) : resetRotationVector a 0 = 0 := by
  simp [resetRotationVector, quaternionLog_identity]

/-- Sola (182)--(183): the actual local Log-coordinate reset has Fréchet
derivative J_r(a), for every finite real correction a, including a=0.
No global smoothness of principal Log and no directional-only premise is used. -/
theorem resetRotationVector_hasFDerivAt (a : Vector3 ℝ) :
    HasFDerivAt (resetRotationVector a) (rightJacobianLinearMap a) 0 := by
  have hd := continuous_smul_hasFDerivAt_zero (logScale_reset_continuousAt a)
    (resetQuaternion_vector_hasFDerivAt a) (by simp)
  have hm : HasFDerivAt
      (fun e => logScale (resetQuaternion a e) • vectorPart (resetQuaternion a e))
      (rightJacobianLinearMap a) 0 := by
    simpa [logScale_identity, smul_smul] using! hd
  have hp : ∀ᶠ e in nhds (0 : Vector3 ℝ), 0 < (resetQuaternion a e).q0 :=
    (resetQuaternion_differentiableAt a).1.continuousAt.eventually
      (eventually_gt_nhds (by simp [identity] : (0 : ℝ) < (resetQuaternion a 0).q0))
  apply hm.congr_of_eventuallyEq
  filter_upwards [hp] with e he
  exact quaternionLog_regularized he

/-- A multivariate little-o remainder, not merely agreement along fixed rays.
This concerns the ideal reset, not a finite-range numerical error bound. -/
theorem resetRotationVector_remainder (a : Vector3 ℝ) :
    (fun e => resetRotationVector a e - rightJacobian a *ᵥ e) =o[nhds 0] id := by
  simpa using! (resetRotationVector_hasFDerivAt a).isLittleO

end

end FormalESKF.ESKF
