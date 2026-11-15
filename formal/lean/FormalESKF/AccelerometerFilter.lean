/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/
import Mathlib.Data.Real.Basic
import Mathlib.Tactic.Linarith
import Mathlib.Tactic.Ring

/-! Component-wise real model of the fixed-coefficient specific-force filter.
It does not identify gravity, prove rest, or bound IEEE rounding error. -/
namespace FormalESKF.ESKF
noncomputable section

def forceLowPass (alpha prior sample : ℝ) : ℝ := prior + alpha * (sample - prior)

theorem forceLowPass_affine (alpha prior sample : ℝ) :
    forceLowPass alpha prior sample = (1 - alpha) * prior + alpha * sample := by
  unfold forceLowPass
  ring

theorem forceLowPass_constant (alpha sample : ℝ) :
    forceLowPass alpha sample sample = sample := by simp [forceLowPass]

theorem forceLowPass_error (alpha prior sample : ℝ) :
    forceLowPass alpha prior sample - sample = (1 - alpha) * (prior - sample) := by
  unfold forceLowPass
  ring

theorem forceLowPass_bounds {alpha prior sample lower upper : ℝ}
    (ha : 0 ≤ alpha) (ha' : alpha ≤ 1)
    (hp : lower ≤ prior) (hp' : prior ≤ upper)
    (hs : lower ≤ sample) (hs' : sample ≤ upper) :
    lower ≤ forceLowPass alpha prior sample ∧ forceLowPass alpha prior sample ≤ upper := by
  rw [forceLowPass_affine]
  have hw : 0 ≤ 1 - alpha := by linarith
  constructor
  · nlinarith [mul_nonneg hw (sub_nonneg.mpr hp), mul_nonneg ha (sub_nonneg.mpr hs)]
  · nlinarith [mul_nonneg hw (sub_nonneg.mpr hp'), mul_nonneg ha (sub_nonneg.mpr hs')]

theorem forceLowPass_residual (prior sample : ℝ) {alpha : ℝ}
    (ha : 0 ≤ alpha) (ha' : alpha ≤ 1) :
    |forceLowPass alpha prior sample - sample| ≤ |prior - sample| := by
  rw [forceLowPass_error, abs_mul, abs_of_nonneg (by linarith : 0 ≤ 1 - alpha)]
  nlinarith [abs_nonneg (prior - sample)]

def forceFilterRepeated (alpha initial sample : ℝ) : ℕ → ℝ
  | 0 => initial
  | n + 1 => forceLowPass alpha (forceFilterRepeated alpha initial sample n) sample

theorem forceFilterRepeated_error (alpha initial sample : ℝ) (n : ℕ) :
    forceFilterRepeated alpha initial sample n - sample = (1 - alpha)^n * (initial - sample) := by
  induction n with
  | zero => simp [forceFilterRepeated]
  | succ n ih =>
    rw [forceFilterRepeated, forceLowPass_error, ih, pow_succ]
    ring

end
end FormalESKF.ESKF
