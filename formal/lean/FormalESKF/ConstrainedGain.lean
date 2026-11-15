/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.Correction

/-! E-BIAS-UPDATE: zeroing disallowed gain rows before BOTH the correction
vector and Joseph update. Exact real algebra, not an IEEE PSD/error bound.
The bias mask acts on correction only; it does not modify prediction noise. -/

namespace FormalESKF.ESKF

noncomputable section
open Matrix

/-- Six body-axis permissions; position, velocity and attitude remain enabled. -/
def insBiasAllowed (accelerometer gyroscope : Fin 3 → Bool) : Fin 15 → Bool :=
  ![true, true, true, true, true, true, true, true, true,
    accelerometer 0, accelerometer 1, accelerometer 2,
    gyroscope 0, gyroscope 1, gyroscope 2]

def constrainGain {n m : ℕ} (allowed : Fin n → Bool) (K : Matrix (Fin n) (Fin m) ℝ) :
    Matrix (Fin n) (Fin m) ℝ :=
  fun i j => if allowed i then K i j else 0

theorem constrainGain_enabled {n m : ℕ} (allowed : Fin n → Bool)
    (K : Matrix (Fin n) (Fin m) ℝ) {i : Fin n} (h : allowed i = true) :
    ∀ j, constrainGain allowed K i j = K i j := by
  intro j
  simp [constrainGain, h]

theorem constrainGain_disabled {n m : ℕ} (allowed : Fin n → Bool)
    (K : Matrix (Fin n) (Fin m) ℝ) {i : Fin n} (h : allowed i = false) :
    ∀ j, constrainGain allowed K i j = 0 := by
  intro j
  simp [constrainGain, h]

theorem constrainGain_default {n m : ℕ} (K : Matrix (Fin n) (Fin m) ℝ) :
    constrainGain (fun _ => true) K = K := by
  ext i j
  simp [constrainGain]

theorem constrainGain_disabled_delta {n m : ℕ} (allowed : Fin n → Bool)
    (K : Matrix (Fin n) (Fin m) ℝ) (r : Fin m → ℝ)
    {i : Fin n} (h : allowed i = false) : (constrainGain allowed K *ᵥ r) i = 0 := by
  simp [Matrix.mulVec, dotProduct, constrainGain, h]

/-- Disabling a correction row retains its prior variance; it does not erase uncertainty. -/
theorem constrainGain_disabled_variance {n m : ℕ} (allowed : Fin n → Bool)
    (P : Covariance n) (H : Matrix (Fin m) (Fin n) ℝ) (V : Covariance m)
    (K : Matrix (Fin n) (Fin m) ℝ) {i : Fin n} (h : allowed i = false) :
    josephCovariance P H V (constrainGain allowed K) i i = P i i := by
  rw [josephCovariance_expansion]
  simp [Matrix.mul_apply, constrainGain, h]

def correctInsConstrained {m : ℕ} (mode : ResetMode) (s : InsState) (P : Covariance 15)
    (r : Fin m → ℝ) (H : Matrix (Fin m) (Fin 15) ℝ) (V : Covariance m)
    (accelerometer gyroscope : Fin 3 → Bool) :=
  let K := constrainGain (insBiasAllowed accelerometer gyroscope) (kalmanGain P H V)
  injectAndResetIns mode s (josephCovariance P H V K) (unpackInsError (K *ᵥ r))

theorem correctInsConstrained_default {m : ℕ} (mode : ResetMode) (s : InsState)
    (P : Covariance 15) (r : Fin m → ℝ) (H : Matrix (Fin m) (Fin 15) ℝ) (V : Covariance m) :
    correctInsConstrained mode s P r H V (fun _ => true) (fun _ => true) =
      correctIns mode s P r H V := by
  have h : insBiasAllowed (fun _ => true) (fun _ => true) = fun _ => true := by
    funext i
    fin_cases i <;> rfl
  simp [correctInsConstrained, h, constrainGain_default, correctIns, correctionVector]

theorem correctInsConstrained_invariants {m : ℕ} (mode : ResetMode) {s : InsState}
    (hq : Quaternion.HasUnitNorm s.q_nb) {P : Covariance 15} (hP : P.PosSemidef)
    (r : Fin m → ℝ) (H : Matrix (Fin m) (Fin 15) ℝ) {V : Covariance m} (hV : V.PosDef)
    (accelerometer gyroscope : Fin 3 → Bool) :
    let result := correctInsConstrained mode s P r H V accelerometer gyroscope
    Quaternion.HasUnitNorm result.1.q_nb ∧ result.2.1.PosSemidef ∧ result.2.2 = zeroInsError :=
  injectAndResetIns_invariants mode hq (josephCovariance_posSemidef H _ hP hV.posSemidef) _

theorem correctInsConstrained_accelerometer {m : ℕ} (mode : ResetMode) (s : InsState)
    (P : Covariance 15) (r : Fin m → ℝ) (H : Matrix (Fin m) (Fin 15) ℝ) (V : Covariance m)
    (accelerometer gyroscope : Fin 3 → Bool) (i : Fin 3) (h : accelerometer i = false) :
    (correctInsConstrained mode s P r H V accelerometer gyroscope).1.b_a i = s.b_a i := by
  fin_cases i <;>
    simp_all [correctInsConstrained, injectAndResetIns, injectIns, unpackInsError,
      Matrix.mulVec, dotProduct, constrainGain, insBiasAllowed, Matrix.vecHead, Matrix.vecTail]

theorem correctInsConstrained_gyroscope {m : ℕ} (mode : ResetMode) (s : InsState)
    (P : Covariance 15) (r : Fin m → ℝ) (H : Matrix (Fin m) (Fin 15) ℝ) (V : Covariance m)
    (accelerometer gyroscope : Fin 3 → Bool) (i : Fin 3) (h : gyroscope i = false) :
    (correctInsConstrained mode s P r H V accelerometer gyroscope).1.b_g i = s.b_g i := by
  fin_cases i <;>
    simp_all [correctInsConstrained, injectAndResetIns, injectIns, unpackInsError,
      Matrix.mulVec, dotProduct, constrainGain, insBiasAllowed, Matrix.vecHead, Matrix.vecTail]

end
end FormalESKF.ESKF
