/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/
import FormalESKF.Covariance
import Mathlib.Tactic.Ring
import Mathlib.Tactic.Linarith

/-! Real NED tilt-variance projection. A covariance's PSD property and the
body-to-NED rotation remain premises; this is not an attitude-accuracy proof. -/
namespace FormalESKF.ESKF
noncomputable section

def tiltVariance (R P : Covariance 3) : ℝ := (sandwich R P) 0 0 + (sandwich R P) 1 1

theorem tiltVariance_nonneg (R : Covariance 3) {P : Covariance 3} (hP : P.PosSemidef) :
    0 ≤ tiltVariance R P := by
  exact add_nonneg (sandwich_posSemidef R hP).diag_nonneg (sandwich_posSemidef R hP).diag_nonneg

theorem tiltVariance_trace (R P : Covariance 3) :
    tiltVariance R P + (sandwich R P) 2 2 = Matrix.trace (sandwich R P) := by
  simp [tiltVariance, Matrix.trace, Matrix.diag, Fin.sum_univ_succ]
  ring

theorem tiltVariance_le_trace (R : Covariance 3) {P : Covariance 3} (hP : P.PosSemidef) :
    tiltVariance R P ≤ Matrix.trace (sandwich R P) := by
  have h := (sandwich_posSemidef R hP).diag_nonneg (i := (2 : Fin 3))
  rw [← tiltVariance_trace]
  linarith

theorem tiltVariance_identity (P : Covariance 3) :
    tiltVariance 1 P = P 0 0 + P 1 1 := by
  simp [tiltVariance, sandwich]

end
end FormalESKF.ESKF
