/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.Blocks
import FormalESKF.ResetDifferential

/-!
# Nominal injection and covariance reset

Ideal real model of `injection.hpp`, corresponding to its normalized attitude
update for a unit prior and an exact Exp increment. The normalization bridge
below makes this domain explicit: arbitrary non-unit priors are not preserved
by the C++ zero-error update. Neither the executable Taylor branch nor IEEE-754
normalization is treated as exact Exp. Nominal injection always uses Exp,
independent of either macro. The reset uses Sola (183) by default or the
separately modeled first-order (294). G P G^T is linearized covariance
re-expression, not an exact nonlinear transformation of a probability
distribution. Only the estimated error MEAN is zeroed, not covariance or
the unknown true estimation error.
-/

namespace FormalESKF.ESKF

noncomputable section

open SO3 Quaternion Matrix

inductive ResetMode where
  | rightJacobian
  | firstOrder

def resetAttitudeMatrix (mode : ResetMode) (a : Vector3 ℝ) : Matrix3 ℝ :=
  match mode with
  | .rightJacobian => rightJacobian a
  | .firstOrder => rightJacobianFirstOrder a

/-- Connect the default covariance-reset block to the derivative of the
actual post-injection rotation-vector error, not just quaternion tangents. -/
theorem resetAttitudeMatrix_hasFDerivAt (a : Vector3 ℝ) :
    HasFDerivAt (resetRotationVector a)
      (Matrix.toLin' (resetAttitudeMatrix .rightJacobian a)).toContinuousLinearMap 0 :=
  resetRotationVector_hasFDerivAt a

def injectIns (s : InsState) (e : InsError) : InsState :=
  ⟨s.p_n + e.delta_p_n, s.v_n + e.delta_v_n, localAttitude s.q_nb e.delta_theta_b,
    s.b_a + e.delta_b_a, s.b_g + e.delta_b_g⟩

def injectAhrs (s : AhrsState) (e : AhrsError) : AhrsState :=
  ⟨localAttitude s.q_nb e.delta_theta_b⟩

/-- The two implementation normalizations are redundant only for a unit prior
and the exact-real Exp increment. This is not a floating-point identity. -/
theorem normalized_localAttitude_eq {q : Quaternion ℝ} (hq : HasUnitNorm q) (a : Vector3 ℝ) :
    normalize (hamiltonMul q (normalize (quaternionExp a))) = localAttitude q a := by
  rw [normalize_of_unit (quaternionExp_hasUnitNorm a)]
  exact normalize_of_unit (localAttitude_hasUnitNorm hq a)

theorem injectIns_normalized_attitude_eq {s : InsState} (hq : HasUnitNorm s.q_nb) (e : InsError) :
    normalize (hamiltonMul s.q_nb (normalize (quaternionExp e.delta_theta_b))) =
      (injectIns s e).q_nb := normalized_localAttitude_eq hq _

theorem injectAhrs_normalized_attitude_eq {s : AhrsState} (hq : HasUnitNorm s.q_nb) (e : AhrsError) :
    normalize (hamiltonMul s.q_nb (normalize (quaternionExp e.delta_theta_b))) =
      (injectAhrs s e).q_nb := normalized_localAttitude_eq hq _

/-- At zero error the normalized real expression returns normalize(q), not
arbitrary q. A successful checked call additionally needs a nonzero norm. -/
theorem normalized_localAttitude_zero (q : Quaternion ℝ) :
    normalize (hamiltonMul q (normalize (quaternionExp 0))) = normalize q := by
  rw [quaternionExp_zero, normalize_of_unit hasUnitNorm_identity, hamiltonMul_identity]

/-- Zero-error identity for the ideal model. Its correspondence to normalized
C++ injection requires the unit-prior premise of the bridge above. -/
theorem injectIns_zero (s : InsState) : injectIns s zeroInsError = s := by
  cases s
  simp [injectIns, zeroInsError, localAttitude_zero]

theorem injectAhrs_zero (s : AhrsState) : injectAhrs s zeroAhrsError = s := by
  cases s
  simp [injectAhrs, zeroAhrsError, localAttitude_zero]

theorem injectIns_hasUnitNorm {s : InsState} (hq : HasUnitNorm s.q_nb) (e : InsError) :
    HasUnitNorm (injectIns s e).q_nb := localAttitude_hasUnitNorm hq _

theorem injectAhrs_hasUnitNorm {s : AhrsState} (hq : HasUnitNorm s.q_nb) (e : AhrsError) :
    HasUnitNorm (injectAhrs s e).q_nb := localAttitude_hasUnitNorm hq _

def insResetBlocks (mode : ResetMode) (a : Vector3 ℝ) : Fin 5 → Matrix3 ℝ :=
  ![1, 1, resetAttitudeMatrix mode a, 1, 1]

def insResetMatrix (mode : ResetMode) (a : Vector3 ℝ) : Covariance 15 :=
  blockDiagonal (insResetBlocks mode a)

def resetInsCovariance (mode : ResetMode) (e : InsError) (P : Covariance 15) : Covariance 15 :=
  sandwich (insResetMatrix mode e.delta_theta_b) P

def resetAhrsCovariance (mode : ResetMode) (e : AhrsError) (P : Covariance 3) : Covariance 3 :=
  sandwich (resetAttitudeMatrix mode e.delta_theta_b) P

theorem resetInsCovariance_posSemidef (mode : ResetMode) (e : InsError)
    {P : Covariance 15} (hP : P.PosSemidef) : (resetInsCovariance mode e P).PosSemidef :=
  sandwich_posSemidef _ hP

theorem resetAhrsCovariance_posSemidef (mode : ResetMode) (e : AhrsError)
    {P : Covariance 3} (hP : P.PosSemidef) : (resetAhrsCovariance mode e P).PosSemidef :=
  sandwich_posSemidef _ hP

theorem resetInsCovariance_blocks (mode : ResetMode) (e : InsError) (P : Covariance 15)
    (b c : Fin 5) :
    matrixBlocks (n := 5) (m := 5) (resetInsCovariance mode e P) b c =
      insResetBlocks mode e.delta_theta_b b * matrixBlocks P b c *
        (insResetBlocks mode e.delta_theta_b c)ᵀ := by
  unfold resetInsCovariance insResetMatrix
  rw [blockDiagonal_sandwich]
  ext i j
  exact blocks_apply _ b c i j

/-- In particular, attitude/velocity cross-covariance is not left unchanged. -/
theorem resetInsCovariance_attitude_velocity (mode : ResetMode) (e : InsError) (P : Covariance 15) :
    matrixBlocks (n := 5) (m := 5) (resetInsCovariance mode e P) 2 1 =
      resetAttitudeMatrix mode e.delta_theta_b * matrixBlocks (n := 5) (m := 5) P 2 1 := by
  rw [resetInsCovariance_blocks]
  simp [insResetBlocks]

theorem resetAttitudeMatrix_zero (mode : ResetMode) : resetAttitudeMatrix mode 0 = 1 := by
  cases mode <;> simp [resetAttitudeMatrix]

theorem insResetMatrix_zero (mode : ResetMode) : insResetMatrix mode 0 = 1 := by
  have h : insResetBlocks mode 0 = fun _ => (1 : Matrix3 ℝ) := by
    funext b
    fin_cases b <;> simp [insResetBlocks, resetAttitudeMatrix_zero]
  rw [insResetMatrix, blockDiagonal, h, blocks_one]

theorem resetInsCovariance_zero (mode : ResetMode) (P : Covariance 15) :
    resetInsCovariance mode zeroInsError P = P := by
  simp [resetInsCovariance, zeroInsError, insResetMatrix_zero, sandwich]

theorem resetAhrsCovariance_zero (mode : ResetMode) (P : Covariance 3) :
    resetAhrsCovariance mode zeroAhrsError P = P := by
  simp [resetAhrsCovariance, zeroAhrsError, resetAttitudeMatrix_zero, sandwich]

def injectAndResetIns (mode : ResetMode) (s : InsState) (P : Covariance 15) (e : InsError) :=
  (injectIns s e, resetInsCovariance mode e P, zeroInsError)

def injectAndResetAhrs (mode : ResetMode) (s : AhrsState) (P : Covariance 3) (e : AhrsError) :=
  (injectAhrs s e, resetAhrsCovariance mode e P, zeroAhrsError)

theorem injectAndResetIns_invariants (mode : ResetMode) {s : InsState} (hq : HasUnitNorm s.q_nb)
    {P : Covariance 15} (hP : P.PosSemidef) (e : InsError) :
    let result := injectAndResetIns mode s P e
    HasUnitNorm result.1.q_nb ∧ result.2.1.PosSemidef ∧ result.2.2 = zeroInsError :=
  ⟨injectIns_hasUnitNorm hq e, resetInsCovariance_posSemidef mode e hP, rfl⟩

theorem injectAndResetAhrs_invariants (mode : ResetMode) {s : AhrsState} (hq : HasUnitNorm s.q_nb)
    {P : Covariance 3} (hP : P.PosSemidef) (e : AhrsError) :
    let result := injectAndResetAhrs mode s P e
    HasUnitNorm result.1.q_nb ∧ result.2.1.PosSemidef ∧ result.2.2 = zeroAhrsError :=
  ⟨injectAhrs_hasUnitNorm hq e, resetAhrsCovariance_posSemidef mode e hP, rfl⟩

end

end FormalESKF.ESKF
