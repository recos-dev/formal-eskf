/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.State

/-!
# Nominal ESKF prediction

Exact real models of `prediction.hpp`. INS freezes navigation acceleration
over each step and subtracts the OLD body-frame biases. Exp and normalized
Euler are distinct attitude models. C++ Exp also has a Taylor branch and
rounding: these exact theorems do not remove that numerical boundary.
-/

namespace FormalESKF.ESKF

noncomputable section

open SO3 Quaternion Matrix

inductive AttitudeMode where
  | exponential
  | normalizedEuler

/-- Unnormalized Euler step: q + dt/2 * q * [0, omega]. -/
def eulerCandidate (q : Quaternion ℝ) (theta : Vector3 ℝ) : Quaternion ℝ :=
  hamiltonMul q (fromScalarVector 1 (vectorScale (1 / 2) theta))

noncomputable def predictAttitude (mode : AttitudeMode) (q : Quaternion ℝ)
    (omega : Vector3 ℝ) (dt : ℝ) : Quaternion ℝ :=
  match mode with
  | .exponential => hamiltonMul q (quaternionExp (vectorScale dt omega))
  | .normalizedEuler => normalize (eulerCandidate q (vectorScale dt omega))

theorem eulerCandidate_coefficients (q : Quaternion ℝ) (theta : Vector3 ℝ) :
    eulerCandidate q theta =
      ⟨q.q0 - q.q1 * (theta 0 / 2) - q.q2 * (theta 1 / 2) - q.q3 * (theta 2 / 2),
       q.q1 + q.q0 * (theta 0 / 2) + q.q2 * (theta 2 / 2) - q.q3 * (theta 1 / 2),
       q.q2 + q.q0 * (theta 1 / 2) + q.q3 * (theta 0 / 2) - q.q1 * (theta 2 / 2),
       q.q3 + q.q0 * (theta 2 / 2) + q.q1 * (theta 1 / 2) - q.q2 * (theta 0 / 2)⟩ := by
  ext <;> simp [eulerCandidate, hamiltonMul, fromScalarVector, vectorScale] <;> ring

theorem eulerCandidate_is_euler (q : Quaternion ℝ) (omega : Vector3 ℝ) (dt : ℝ) :
    let qdot := scale (1 / 2) (hamiltonMul q (fromScalarVector 0 omega))
    eulerCandidate q (vectorScale dt omega) =
      ⟨q.q0 + dt * qdot.q0, q.q1 + dt * qdot.q1,
       q.q2 + dt * qdot.q2, q.q3 + dt * qdot.q3⟩ := by
  ext <;> simp [eulerCandidate, hamiltonMul, fromScalarVector, vectorScale, scale] <;> ring

theorem eulerCandidate_normSquared (q : Quaternion ℝ) (theta : Vector3 ℝ) :
    normSquared (eulerCandidate q theta) =
      normSquared q * (1 + vectorNormSquared theta / 4) := by
  rw [eulerCandidate, normSquared_hamiltonMul, normSquared_fromScalarVector,
    vectorNormSquared_vectorScale]
  ring

theorem eulerCandidate_normSquared_pos {q : Quaternion ℝ} (hq : HasUnitNorm q)
    (theta : Vector3 ℝ) : 0 < normSquared (eulerCandidate q theta) := by
  rw [eulerCandidate_normSquared, hq, one_mul]
  have := vectorNormSquared_nonnegative theta
  linarith

theorem normalize_of_unit {q : Quaternion ℝ} (hq : HasUnitNorm q) : normalize q = q := by
  change normSquared q = 1 at hq
  ext <;> simp [Quaternion.normalize, hq, scale]

theorem predictAttitude_hasUnitNorm (mode : AttitudeMode) {q : Quaternion ℝ}
    (hq : HasUnitNorm q) (omega : Vector3 ℝ) (dt : ℝ) :
    HasUnitNorm (predictAttitude mode q omega dt) := by
  cases mode with
  | exponential => exact hq.hamiltonMul (quaternionExp_hasUnitNorm _)
  | normalizedEuler => exact hasUnitNorm_normalize (eulerCandidate_normSquared_pos hq _)

/-- In exact arithmetic, the implementation's two checked normalizations
are redundant for a unit initial attitude and the exact Exp increment.
This is not a claim that floating-point normalization is redundant. -/
theorem normalized_exponential_prediction_eq {q : Quaternion ℝ} (hq : HasUnitNorm q)
    (omega : Vector3 ℝ) (dt : ℝ) :
    normalize (hamiltonMul q (normalize (quaternionExp (vectorScale dt omega)))) =
      predictAttitude .exponential q omega dt := by
  rw [normalize_of_unit (quaternionExp_hasUnitNorm _)]
  exact normalize_of_unit (hq.hamiltonMul (quaternionExp_hasUnitNorm _))

theorem predictAttitude_zero_rate (mode : AttitudeMode) {q : Quaternion ℝ}
    (hq : HasUnitNorm q) (dt : ℝ) : predictAttitude mode q 0 dt = q := by
  cases mode <;> simp [predictAttitude, eulerCandidate, fromScalarVector, vectorScale,
    show (⟨1, 0, 0, 0⟩ : Quaternion ℝ) = identity by rfl, normalize_of_unit hq]

theorem predictAttitude_zero_step (mode : AttitudeMode) {q : Quaternion ℝ}
    (hq : HasUnitNorm q) (omega : Vector3 ℝ) : predictAttitude mode q omega 0 = q := by
  have h : vectorScale 0 omega = 0 := by funext i; simp [vectorScale]
  cases mode <;> simp [predictAttitude, h, eulerCandidate, fromScalarVector,
    show (⟨1, 0, 0, 0⟩ : Quaternion ℝ) = identity by rfl, normalize_of_unit hq]

theorem predictAttitude_exponential_action (q : Quaternion ℝ) (omega v_b : Vector3 ℝ) (dt : ℝ) :
    rotate (predictAttitude .exponential q omega dt) v_b =
      rotate q (rotationMatrixExp (vectorScale dt omega) *ᵥ v_b) := by
  change rotate (hamiltonMul q (quaternionExp (vectorScale dt omega))) v_b = _
  rw [rotate_hamiltonMul]
  congr 1
  exact congrArg (fun m => m *ᵥ v_b) (rotationMatrix_quaternionExp _)

/-- Constant body-rate solution, written without `abs dt` so derivatives are
defined also at dt=0. For nonnegative time this equals Exp(dt*omega). -/
def constantRateIncrement (omega : Vector3 ℝ) (dt : ℝ) : Quaternion ℝ :=
  let w := vectorNorm omega
  if w = 0 then identity else
    fromScalarVector (Real.cos (w * dt / 2))
      (vectorScale (Real.sin (w * dt / 2) / w) omega)

theorem constantRateIncrement_eq_exp (omega : Vector3 ℝ) {dt : ℝ} (ht : 0 ≤ dt) :
    constantRateIncrement omega dt = quaternionExp (vectorScale dt omega) := by
  by_cases hw : vectorNorm omega = 0
  · have ho := (vectorNorm_eq_zero_iff omega).mp hw
    subst omega
    simp [constantRateIncrement]
  · by_cases hz : dt = 0
    · subst dt
      have hs : vectorScale 0 omega = 0 := by funext i; simp [vectorScale]
      simp [constantRateIncrement, hw, hs, fromScalarVector, identity]
    · have hn : vectorNorm (vectorScale dt omega) ≠ 0 := by
        rw [vectorNorm_vectorScale_of_nonnegative ht]
        exact mul_ne_zero hz hw
      rw [quaternionExp_of_vectorNorm_ne_zero hn]
      ext <;> simp [constantRateIncrement, hw, fromScalarVector, vectorScale,
        vectorNorm_vectorScale_of_nonnegative ht, mul_comm dt (vectorNorm omega)]
      all_goals field_simp

/-- Componentwise quaternion derivative, using four stored coefficients. -/
def HasQuaternionDerivAt (f : ℝ → Quaternion ℝ) (d : Quaternion ℝ) (t : ℝ) : Prop :=
  HasDerivAt (fun x => (f x).q0) d.q0 t ∧ HasDerivAt (fun x => (f x).q1) d.q1 t ∧
  HasDerivAt (fun x => (f x).q2) d.q2 t ∧ HasDerivAt (fun x => (f x).q3) d.q3 t

/-- The closed form satisfies qdot = (q * [0,omega])/2, not left multiplication. -/
theorem constantRateIncrement_kinematics (omega : Vector3 ℝ) (t : ℝ) :
    HasQuaternionDerivAt (constantRateIncrement omega)
      (scale (1 / 2) (hamiltonMul (constantRateIncrement omega t) (fromScalarVector 0 omega))) t := by
  by_cases hw : vectorNorm omega = 0
  · have ho := (vectorNorm_eq_zero_iff omega).mp hw
    subst omega
    simp only [HasQuaternionDerivAt, constantRateIncrement, vectorNorm_zero,
      hamiltonMul, fromScalarVector, identity, scale, Pi.zero_apply, mul_zero,
      zero_mul, sub_zero, add_zero]
    exact ⟨hasDerivAt_const _ _, hasDerivAt_const _ _, hasDerivAt_const _ _, hasDerivAt_const _ _⟩
  · have hn := vectorNorm_squared omega
    have hn' : vectorNorm omega ^ 2 = omega 0 * omega 0 + omega 1 * omega 1 + omega 2 * omega 2 := by
      simpa [vectorNormSquared, dotProduct, Fin.sum_univ_succ, add_assoc] using hn
    have ha : HasDerivAt (fun x : ℝ => vectorNorm omega * x / 2) (vectorNorm omega / 2) t := by
      simpa only [mul_one, id_eq] using! ((hasDerivAt_id t).const_mul (vectorNorm omega)).div_const 2
    have hc := ha.cos
    have hs (i : Fin 3) := (ha.sin.div_const (vectorNorm omega)).mul_const (omega i)
    simp only [HasQuaternionDerivAt, constantRateIncrement, hw, if_false, fromScalarVector,
      vectorScale, hamiltonMul, scale]
    refine ⟨hc.congr_deriv ?_, (hs 0).congr_deriv ?_, (hs 1).congr_deriv ?_, (hs 2).congr_deriv ?_⟩
    all_goals field_simp
    · rw [hn']
      ring
    all_goals ring

/-- A fixed initial attitude is a linear map on quaternion coefficients. -/
theorem HasQuaternionDerivAt.hamilton_left {f : ℝ → Quaternion ℝ} {d : Quaternion ℝ} {t : ℝ}
    (h : HasQuaternionDerivAt f d t) (q : Quaternion ℝ) :
    HasQuaternionDerivAt (fun x => hamiltonMul q (f x)) (hamiltonMul q d) t := by
  rcases h with ⟨h0, h1, h2, h3⟩
  exact ⟨(((h0.const_mul q.q0).sub (h1.const_mul q.q1)).sub (h2.const_mul q.q2)).sub (h3.const_mul q.q3),
    (((h1.const_mul q.q0).add (h0.const_mul q.q1)).add (h3.const_mul q.q2)).sub (h2.const_mul q.q3),
    (((h2.const_mul q.q0).sub (h3.const_mul q.q1)).add (h0.const_mul q.q2)).add (h1.const_mul q.q3),
    (((h3.const_mul q.q0).add (h2.const_mul q.q1)).sub (h1.const_mul q.q2)).add (h0.const_mul q.q3)⟩

/-- For an arbitrary initial q, the Exp trajectory solves constant body-rate
kinematics. Combine with constantRateIncrement_eq_exp for the prediction map. -/
theorem constantRate_attitude_kinematics (q : Quaternion ℝ) (omega : Vector3 ℝ) (t : ℝ) :
    HasQuaternionDerivAt (fun x => hamiltonMul q (constantRateIncrement omega x))
      (scale (1 / 2) (hamiltonMul (hamiltonMul q (constantRateIncrement omega t))
        (fromScalarVector 0 omega))) t := by
  have h := (constantRateIncrement_kinematics omega t).hamilton_left q
  have heq : hamiltonMul q (scale (1 / 2)
      (hamiltonMul (constantRateIncrement omega t) (fromScalarVector 0 omega))) =
      scale (1 / 2) (hamiltonMul (hamiltonMul q (constantRateIncrement omega t))
        (fromScalarVector 0 omega)) := by
    rw [hamiltonMul_assoc]
    ext <;> simp [hamiltonMul, scale] <;> ring
  rw [heq] at h
  exact h

/-- Frozen navigation acceleration, evaluated from the old state. -/
def navigationAcceleration (s : InsState) (specific_force_b gravity_n : Vector3 ℝ) : Vector3 ℝ :=
  rotate s.q_nb (specific_force_b - s.b_a) + gravity_n

def positionStep (p v a : Vector3 ℝ) (dt : ℝ) : Vector3 ℝ :=
  fun i => p i + v i * dt + a i * ((1 / 2) * dt * dt)

def velocityStep (v a : Vector3 ℝ) (dt : ℝ) : Vector3 ℝ :=
  fun i => v i + a i * dt

noncomputable def predictIns (mode : AttitudeMode) (s : InsState)
    (specific_force_b angular_rate_b gravity_n : Vector3 ℝ) (dt : ℝ) : InsState :=
  let a := navigationAcceleration s specific_force_b gravity_n
  { s with
    p_n := positionStep s.p_n s.v_n a dt
    v_n := velocityStep s.v_n a dt
    q_nb := predictAttitude mode s.q_nb (angular_rate_b - s.b_g) dt }

noncomputable def predictAhrs (mode : AttitudeMode) (s : AhrsState)
    (angular_rate_b : Vector3 ℝ) (dt : ℝ) : AhrsState :=
  ⟨predictAttitude mode s.q_nb angular_rate_b dt⟩

/-- The position curve has the selected velocity as its derivative. -/
theorem positionStep_hasDerivAt (p v a : Vector3 ℝ) (t : ℝ) (i : Fin 3) :
    HasDerivAt (fun dt => positionStep p v a dt i) (velocityStep v a t i) t := by
  have h := ((hasDerivAt_const t (p i)).add ((hasDerivAt_id t).const_mul (v i))).add
    ((((hasDerivAt_const t (1 / 2 : ℝ)).mul (hasDerivAt_id t)).mul
      (hasDerivAt_id t)).const_mul (a i))
  exact h.congr_deriv (by dsimp [velocityStep]; ring)

theorem velocityStep_hasDerivAt (v a : Vector3 ℝ) (t : ℝ) (i : Fin 3) :
    HasDerivAt (fun dt => velocityStep v a dt i) (a i) t := by
  have h := (hasDerivAt_const t (v i)).add ((hasDerivAt_id t).const_mul (a i))
  exact h.congr_deriv (by ring)

/-- Splitting a step is exact for a fixed navigation acceleration. This does
not assert that recomputing acceleration after rotating gives the same step. -/
theorem constantAcceleration_compose (p v a : Vector3 ℝ) (t u : ℝ) :
    positionStep (positionStep p v a t) (velocityStep v a t) a u = positionStep p v a (t + u) ∧
    velocityStep (velocityStep v a t) a u = velocityStep v a (t + u) := by
  constructor <;> funext i <;> simp [positionStep, velocityStep] <;> ring

/-- Semi-implicit position using the NEW velocity adds an unwanted a*dt^2. -/
theorem newVelocity_position_difference (p v a : Vector3 ℝ) (dt : ℝ) :
    positionStep p (velocityStep v a dt) a dt - positionStep p v a dt =
      vectorScale (dt ^ 2) a := by
  funext i
  simp [positionStep, velocityStep, vectorScale]
  ring

theorem predictIns_preserves_biases (mode : AttitudeMode) (s : InsState)
    (f w g : Vector3 ℝ) (dt : ℝ) :
    (predictIns mode s f w g dt).b_a = s.b_a ∧ (predictIns mode s f w g dt).b_g = s.b_g :=
  ⟨rfl, rfl⟩

/-- Applying the same calibration shift to a measurement and its bias leaves
the predicted p/v/q unchanged; this detects the subtraction convention. -/
theorem predictIns_bias_compensation (mode : AttitudeMode) (s : InsState)
    (f w g shift_a shift_g : Vector3 ℝ) (dt : ℝ) :
    let shifted := predictIns mode { s with b_a := s.b_a + shift_a, b_g := s.b_g + shift_g }
      (f + shift_a) (w + shift_g) g dt
    let original := predictIns mode s f w g dt
    shifted.p_n = original.p_n ∧ shifted.v_n = original.v_n ∧ shifted.q_nb = original.q_nb := by
  simp [predictIns, navigationAcceleration, add_sub_add_right_eq_sub]

theorem predictIns_hasUnitNorm (mode : AttitudeMode) (s : InsState)
    (hq : HasUnitNorm s.q_nb) (f w g : Vector3 ℝ) (dt : ℝ) :
    HasUnitNorm (predictIns mode s f w g dt).q_nb :=
  predictAttitude_hasUnitNorm mode hq _ _

theorem predictAhrs_hasUnitNorm (mode : AttitudeMode) (s : AhrsState)
    (hq : HasUnitNorm s.q_nb) (w : Vector3 ℝ) (dt : ℝ) :
    HasUnitNorm (predictAhrs mode s w dt).q_nb :=
  predictAttitude_hasUnitNorm mode hq _ _

/-- At rest, the accelerometer measures negative rotated gravity plus bias.
This fixes the gravity sign for every unit attitude, not only level flight. -/
theorem navigationAcceleration_stationary (s : InsState) (hq : HasUnitNorm s.q_nb)
    (g : Vector3 ℝ) : navigationAcceleration s (s.b_a - inverseRotate s.q_nb g) g = 0 := by
  have hb : s.b_a - inverseRotate s.q_nb g - s.b_a = -inverseRotate s.q_nb g := by
    funext i
    simp only [Pi.sub_apply, Pi.neg_apply]
    ring
  have hr : rotate s.q_nb (inverseRotate s.q_nb g) = g := by
    simp [rotate, inverseRotate, Matrix.mulVec_mulVec, rotationMatrix_mul_transpose_of_unit hq]
  rw [navigationAcceleration, hb, rotate, Matrix.mulVec_neg]
  change -rotate s.q_nb (inverseRotate s.q_nb g) + g = 0
  rw [hr]
  simp

theorem predictIns_stationary (mode : AttitudeMode) (s : InsState)
    (hq : HasUnitNorm s.q_nb) (hv : s.v_n = 0) (g : Vector3 ℝ) (dt : ℝ) :
    predictIns mode s (s.b_a - inverseRotate s.q_nb g) s.b_g g dt = s := by
  have ha := navigationAcceleration_stationary s hq g
  apply InsState.ext
  · funext i
    simp [predictIns, ha, positionStep, hv]
  · funext i
    simp [predictIns, ha, velocityStep]
  · simpa [predictIns] using predictAttitude_zero_rate mode hq dt
  · rfl
  · rfl

end

end FormalESKF.ESKF
