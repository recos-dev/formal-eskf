/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.Injection

/-!
# Linearized correction followed by injection/reset

Exact equations of `correction.hpp`: Sola (274)--(275) and Joseph form.
The caller supplies r=y-h, H, SPD effective noise V and a zero prior error
mean. The statistical interpretation assumes no prior-error/noise correlation;
this file proves the algebra and invariants, not that sensor data satisfy
those physical/statistical premises. Inverse notation characterizes the
unique right solve; C++ still uses Cholesky, without constructing an inverse.
-/

namespace FormalESKF.ESKF

noncomputable section

open SO3 Quaternion Matrix

def correctionVector {n m : ℕ} (P : Covariance n) (H : Matrix (Fin m) (Fin n) ℝ)
    (V : Covariance m) (r : Fin m → ℝ) : Fin n → ℝ := kalmanGain P H V *ᵥ r

def correctIns {m : ℕ} (mode : ResetMode) (s : InsState) (P : Covariance 15)
    (r : Fin m → ℝ) (H : Matrix (Fin m) (Fin 15) ℝ) (V : Covariance m) :=
  injectAndResetIns mode s (josephCovariance P H V (kalmanGain P H V))
    (unpackInsError (correctionVector P H V r))

def correctAhrs {m : ℕ} (mode : ResetMode) (s : AhrsState) (P : Covariance 3)
    (r : Fin m → ℝ) (H : Matrix (Fin m) (Fin 3) ℝ) (V : Covariance m) :=
  injectAndResetAhrs mode s (josephCovariance P H V (kalmanGain P H V))
    (unpackAhrsError (correctionVector P H V r))

theorem correctionVector_zero_residual {n m : ℕ} (P : Covariance n)
    (H : Matrix (Fin m) (Fin n) ℝ) (V : Covariance m) : correctionVector P H V 0 = 0 := by
  simp [correctionVector]

theorem correctIns_invariants {m : ℕ} (mode : ResetMode) {s : InsState}
    (hq : HasUnitNorm s.q_nb) {P : Covariance 15} (hP : P.PosSemidef)
    (r : Fin m → ℝ) (H : Matrix (Fin m) (Fin 15) ℝ) {V : Covariance m} (hV : V.PosDef) :
    let result := correctIns mode s P r H V
    HasUnitNorm result.1.q_nb ∧ result.2.1.PosSemidef ∧ result.2.2 = zeroInsError :=
  injectAndResetIns_invariants mode hq (josephCovariance_posSemidef H _ hP hV.posSemidef) _

theorem correctAhrs_invariants {m : ℕ} (mode : ResetMode) {s : AhrsState}
    (hq : HasUnitNorm s.q_nb) {P : Covariance 3} (hP : P.PosSemidef)
    (r : Fin m → ℝ) (H : Matrix (Fin m) (Fin 3) ℝ) {V : Covariance m} (hV : V.PosDef) :
    let result := correctAhrs mode s P r H V
    HasUnitNorm result.1.q_nb ∧ result.2.1.PosSemidef ∧ result.2.2 = zeroAhrsError :=
  injectAndResetAhrs_invariants mode hq (josephCovariance_posSemidef H _ hP hV.posSemidef) _

/-- Zero innovation preserves the nominal state, but measurement information
still changes covariance. Reset is identity only because the correction is zero.
This is the ideal model: correspondence to normalized C++ injection requires
a unit prior, as specified by `normalized_localAttitude_eq`. -/
theorem correctIns_zero_residual {m : ℕ} (mode : ResetMode) (s : InsState) (P : Covariance 15)
    (H : Matrix (Fin m) (Fin 15) ℝ) (V : Covariance m) :
    correctIns mode s P 0 H V =
      (s, josephCovariance P H V (kalmanGain P H V), zeroInsError) := by
  simp [correctIns, correctionVector_zero_residual, unpackInsError_zero,
    injectAndResetIns, injectIns_zero, resetInsCovariance_zero]

theorem correctAhrs_zero_residual {m : ℕ} (mode : ResetMode) (s : AhrsState) (P : Covariance 3)
    (H : Matrix (Fin m) (Fin 3) ℝ) (V : Covariance m) :
    correctAhrs mode s P 0 H V =
      (s, josephCovariance P H V (kalmanGain P H V), zeroAhrsError) := by
  simp [correctAhrs, correctionVector_zero_residual, unpackAhrsError_zero,
    injectAndResetAhrs, injectAhrs_zero, resetAhrsCovariance_zero]

theorem correction_unpack_ins {m : ℕ} (P : Covariance 15)
    (H : Matrix (Fin m) (Fin 15) ℝ) (V : Covariance m) (r : Fin m → ℝ) :
    packInsError (unpackInsError (correctionVector P H V r)) = kalmanGain P H V *ᵥ r :=
  pack_unpack_insError _

theorem correction_unpack_ahrs {m : ℕ} (P : Covariance 3)
    (H : Matrix (Fin m) (Fin 3) ℝ) (V : Covariance m) (r : Fin m → ℝ) :
    packAhrsError (unpackAhrsError (correctionVector P H V r)) = kalmanGain P H V *ᵥ r := rfl

end

end FormalESKF.ESKF
