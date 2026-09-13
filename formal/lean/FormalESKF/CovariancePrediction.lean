/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.Prediction
import FormalESKF.ProcessNoise
import FormalESKF.RightJacobian
import FormalESKF.RotationDifferential

/-!
# ESKF covariance prediction

The selected truncated transition in `covariance_prediction.hpp`, with OLD
attitude and biases. Even in Exp mode it omits higher-order position and bias
couplings; it is not the full derivative of the constant-acceleration nominal
integrator. The Euler mode below is the separately selected first-order error
transition, not the exact derivative of normalized quaternion Euler.
-/

namespace FormalESKF.ESKF

noncomputable section

open SO3 Matrix

def attitudeTransition (mode : AttitudeMode) (omega : Vector3 ℝ) (dt : ℝ) : Matrix3 ℝ :=
  match mode with
  | .exponential => (rotationMatrix (quaternionExp (vectorScale dt omega)))ᵀ
  | .normalizedEuler => 1 - hat (vectorScale dt omega)

/-- With the same fixed body rate on nominal and perturbed trajectories,
the local attitude-error differential is precisely the Exp F_theta block.
This does not claim exact discretization of the separate bias coupling. -/
theorem attitudeTransition_exponential_differential {q : Quaternion ℝ}
    (hq : Quaternion.HasUnitNorm q) (w v : Vector3 ℝ) (dt : ℝ) :
    HasQuaternionDerivAt
      (fun t => Quaternion.hamiltonMul (Quaternion.conjugate (predictAttitude .exponential q w dt))
        (predictAttitude .exponential (localAttitude q (vectorScale t v)) w dt))
      (fromScalarVector 0 (vectorScale (1 / 2) (attitudeTransition .exponential w dt *ᵥ v))) 0 := by
  have he (t : ℝ) :
      Quaternion.hamiltonMul (Quaternion.conjugate (predictAttitude .exponential q w dt))
        (predictAttitude .exponential (localAttitude q (vectorScale t v)) w dt) =
      Quaternion.hamiltonMul (Quaternion.conjugate (quaternionExp (vectorScale dt w)))
        (Quaternion.hamiltonMul (quaternionExp (vectorScale t v)) (quaternionExp (vectorScale dt w))) := by
    simp only [predictAttitude, localAttitude, Quaternion.conjugate_hamiltonMul,
      Quaternion.hamiltonMul_assoc]
    rw [← Quaternion.hamiltonMul_assoc (Quaternion.conjugate q) q,
      Quaternion.conjugate_hamiltonMul_of_unit hq, Quaternion.identity_hamiltonMul]
  simpa only [he, attitudeTransition, inverseRotate] using!
    conjugatedIncrement_hasDerivAt (quaternionExp (vectorScale dt w)) v

def insTransitionBlocks (mode : AttitudeMode) (s : InsState)
    (f w : Vector3 ℝ) (dt : ℝ) : Fin 5 → Fin 5 → Matrix3 ℝ :=
  let R := rotationMatrix s.q_nb
  ![![1, dt • 1, 0, 0, 0],
    ![0, 1, dt • (-R * hat (f - s.b_a)), dt • -R, 0],
    ![0, 0, attitudeTransition mode (w - s.b_g) dt, 0, -(dt • 1)],
    ![0, 0, 0, 1, 0],
    ![0, 0, 0, 0, 1]]

def insTransition (mode : AttitudeMode) (s : InsState) (f w : Vector3 ℝ) (dt : ℝ) :
    Covariance 15 := blocks (insTransitionBlocks mode s f w dt)

def linearizedInsError (mode : AttitudeMode) (s : InsState) (f w : Vector3 ℝ) (dt : ℝ)
    (e : InsError) : InsError :=
  let R := rotationMatrix s.q_nb
  ⟨e.delta_p_n + dt • e.delta_v_n,
    e.delta_v_n - dt • ((R * hat (f - s.b_a)) *ᵥ e.delta_theta_b) - dt • (R *ᵥ e.delta_b_a),
    attitudeTransition mode (w - s.b_g) dt *ᵥ e.delta_theta_b - dt • e.delta_b_g,
    e.delta_b_a, e.delta_b_g⟩

/-- Multiplying the flat 15-coordinate matrix has exactly the intended five
error-block equations, including bias signs and the old-state rotation. -/
theorem insTransition_action (mode : AttitudeMode) (s : InsState) (f w : Vector3 ℝ)
    (dt : ℝ) (e : InsError) :
    insTransition mode s f w dt *ᵥ packInsError e = packInsError (linearizedInsError mode s f w dt e) := by
  rw [insTransition, packInsError_eq_blockVector, blocks_mulVec, packInsError_eq_blockVector]
  apply congrArg blockVector
  funext b
  fin_cases b <;> simp [insTransitionBlocks, linearizedInsError, Fin.sum_univ_succ,
    Matrix.smul_mulVec, Matrix.neg_mulVec]
  all_goals abel

def insNoiseInjectionBlocks (q : Quaternion ℝ) : Fin 5 → Fin 4 → Matrix3 ℝ :=
  ![![0, 0, 0, 0], ![-rotationMatrix q, 0, 0, 0],
    ![0, -1, 0, 0], ![0, 0, 1, 0], ![0, 0, 0, 1]]

def insNoiseInjection (q : Quaternion ℝ) : Matrix (Fin 15) (Fin 12) ℝ :=
  blocks (insNoiseInjectionBlocks q)

def injectedNoiseBlocks (q : Quaternion ℝ) (noise : InsNoise) (dt : ℝ) :
    Fin 5 → Matrix3 ℝ :=
  ![0, sandwich (rotationMatrix q) (measurementImpulse noise.specific_force_variance dt),
    measurementImpulse noise.angular_rate_variance dt,
    biasImpulse noise.accelerometer_bias_random_walk_variance_density dt,
    biasImpulse noise.gyroscope_bias_random_walk_variance_density dt]

/-- The production four diagonal-block additions equal F_i Q_i F_i^T.
This includes the anisotropic accelerometer rotation and the negative signs
in F_i; no isotropy premise is used. -/
theorem insNoiseInjection_expansion (q : Quaternion ℝ) (noise : InsNoise) (dt : ℝ) :
    sandwich (insNoiseInjection q) (insProcessNoise noise dt) =
      blocks (fun b c : Fin 5 => if b = c then injectedNoiseBlocks q noise dt b else 0) := by
  rw [sandwich, insNoiseInjection, insProcessNoise_blocks, blocks_transpose,
    blocks_mul, blocks_mul]
  apply congrArg blocks
  funext b c
  fin_cases b <;> fin_cases c <;>
    simp [insNoiseInjectionBlocks, injectedNoiseBlocks, noiseBlocks,
      Fin.sum_univ_succ, sandwich, Matrix.mul_assoc]

theorem isotropicNoise_rotation {q : Quaternion ℝ} (hq : Quaternion.HasUnitNorm q) (v : ℝ) :
    sandwich (rotationMatrix q) (v • (1 : Matrix3 ℝ)) = v • (1 : Matrix3 ℝ) := by
  simp [sandwich, rotationMatrix_mul_transpose_of_unit hq]

def predictInsCovariance (mode : AttitudeMode) (s : InsState) (P : Covariance 15)
    (f w : Vector3 ℝ) (noise : InsNoise) (dt : ℝ) : Covariance 15 :=
  propagateCovariance (insTransition mode s f w dt) P (insNoiseInjection s.q_nb)
    (insProcessNoise noise dt)

def predictAhrsCovariance (mode : AttitudeMode) (P : Covariance 3)
    (w gyroVariance : Vector3 ℝ) (dt : ℝ) : Covariance 3 :=
  sandwich (attitudeTransition mode w dt) P + measurementImpulse gyroVariance dt

theorem predictInsCovariance_posSemidef (mode : AttitudeMode) (s : InsState)
    {P : Covariance 15} (hP : P.PosSemidef) (f w : Vector3 ℝ)
    {noise : InsNoise} (hn : ValidInsNoise noise) {dt : ℝ} (ht : 0 ≤ dt) :
    (predictInsCovariance mode s P f w noise dt).PosSemidef :=
  propagateCovariance_posSemidef _ _ hP (insProcessNoise_posSemidef hn ht)

theorem predictAhrsCovariance_posSemidef (mode : AttitudeMode)
    {P : Covariance 3} (hP : P.PosSemidef) (w : Vector3 ℝ)
    {gyroVariance : Vector3 ℝ} (hn : ∀ i, 0 ≤ gyroVariance i) (dt : ℝ) :
    (predictAhrsCovariance mode P w gyroVariance dt).PosSemidef :=
  (sandwich_posSemidef _ hP).add (measurementImpulse_posSemidef hn dt)

theorem attitudeTransition_zero_step (mode : AttitudeMode) (w : Vector3 ℝ) :
    attitudeTransition mode w 0 = 1 := by
  have hz : vectorScale 0 w = 0 := by funext i; simp [vectorScale]
  cases mode <;> simp [attitudeTransition, hz]

/-- The two covariance attitude modes have the same first derivative in dt.
Their finite-step matrices need not agree. -/
theorem attitudeTransition_hasDerivAt_zero (mode : AttitudeMode) (w : Vector3 ℝ) (i j : Fin 3) :
    HasDerivAt (fun dt => attitudeTransition mode w dt i j) (-(hat w i j)) 0 := by
  cases mode with
  | exponential =>
    have hz : quaternionExp (vectorScale 0 w) = Quaternion.identity := by
      rw [← constantRateIncrement_eq_exp_signed, constantRateIncrement_zero]
    have h := rotationMatrix_hasDerivAt_identity w hz (quaternionExp_ray_hasDerivAt_zero w) j i
    have ht : hat w j i = -(hat w i j) := congrArg (fun M : Matrix3 ℝ => M i j) (transpose_hat w)
    simpa only [attitudeTransition, transpose_apply, ht] using h
  | normalizedEuler =>
    have he : (fun dt => attitudeTransition .normalizedEuler w dt i j) =
        fun dt => (1 : Matrix3 ℝ) i j - dt * hat w i j := by
      funext dt
      simp [attitudeTransition, hat_scale]
    rw [he]
    exact ((hasDerivAt_const 0 ((1 : Matrix3 ℝ) i j)).sub
      ((hasDerivAt_id 0).mul_const (hat w i j))).congr_deriv (by simp)

theorem insTransition_zero_step (mode : AttitudeMode) (s : InsState) (f w : Vector3 ℝ) :
    insTransition mode s f w 0 = 1 := by
  rw [insTransition, ← blocks_one 5]
  apply congrArg blocks
  funext b c
  fin_cases b <;> fin_cases c <;>
    simp [insTransitionBlocks, attitudeTransition_zero_step]

/-- Zero time is a mathematical limit; the public API requires positive dt. -/
theorem predictInsCovariance_zero_step (mode : AttitudeMode) (s : InsState)
    (P : Covariance 15) (f w : Vector3 ℝ) (noise : InsNoise) :
    predictInsCovariance mode s P f w noise 0 = P := by
  simp [predictInsCovariance, propagateCovariance, sandwich, insTransition_zero_step,
    insProcessNoise_zero_step]

theorem predictAhrsCovariance_zero_step (mode : AttitudeMode) (P : Covariance 3)
    (w gyroVariance : Vector3 ℝ) : predictAhrsCovariance mode P w gyroVariance 0 = P := by
  simp [predictAhrsCovariance, sandwich, attitudeTransition_zero_step,
    measurementImpulse_zero_step]

end

end FormalESKF.ESKF
