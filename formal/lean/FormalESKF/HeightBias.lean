/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/
import Mathlib.Data.Real.Basic
import Mathlib.Tactic.FieldSimp
import Mathlib.Tactic.Linarith
import Mathlib.Tactic.Ring

/-! Real scalar offset equations in HeightAiding. These are conditional model
properties, not floating-point error bounds or independent sensor truth. -/
namespace FormalESKF.ESKF
noncomputable section

def heightBiasGain (p r : ℝ) : ℝ := p / (p + r)
def heightBiasVariance (p r : ℝ) : ℝ := (1 - heightBiasGain p r) * p

theorem heightBiasGain_bounds {p r : ℝ} (hp : 0 ≤ p) (hr : 0 < r) :
    0 ≤ heightBiasGain p r ∧ heightBiasGain p r ≤ 1 := by
  have hd : 0 < p + r := by linarith
  exact ⟨div_nonneg hp hd.le, (div_le_one hd).mpr (by linarith)⟩

theorem heightBiasVariance_formula {p r : ℝ} (h : p + r ≠ 0) :
    heightBiasVariance p r = p * r / (p + r) := by
  unfold heightBiasVariance heightBiasGain
  field_simp
  ring

theorem heightBiasVariance_bounds {p r : ℝ} (hp : 0 ≤ p) (hr : 0 < r) :
    0 ≤ heightBiasVariance p r ∧ heightBiasVariance p r ≤ p := by
  obtain ⟨hl,hu⟩ := heightBiasGain_bounds hp hr
  unfold heightBiasVariance
  constructor
  · exact mul_nonneg (by linarith) hp
  · nlinarith

theorem heightBiasVariance_joseph {p r : ℝ} (h : p + r ≠ 0) :
    (1 - heightBiasGain p r)^2 * p + (heightBiasGain p r)^2 * r = heightBiasVariance p r := by
  unfold heightBiasGain heightBiasVariance
  unfold heightBiasGain
  field_simp
  ring

def heightBiasUpdate (b measurement p r : ℝ) : ℝ := b + heightBiasGain p r * (measurement - b)

theorem heightBiasUpdate_convex (b measurement p r : ℝ) :
    heightBiasUpdate b measurement p r =
      (1 - heightBiasGain p r) * b + heightBiasGain p r * measurement := by
  unfold heightBiasUpdate
  ring

theorem heightBiasUpdate_reduces_residual (b measurement : ℝ) {p r : ℝ}
    (hp : 0 ≤ p) (hr : 0 < r) :
    |measurement - heightBiasUpdate b measurement p r| ≤ |measurement - b| := by
  obtain ⟨hl,hu⟩ := heightBiasGain_bounds hp hr
  have h : measurement - heightBiasUpdate b measurement p r =
      (1 - heightBiasGain p r) * (measurement - b) := by unfold heightBiasUpdate; ring
  rw [h, abs_mul, abs_of_nonneg (by linarith : 0 ≤ 1 - heightBiasGain p r)]
  nlinarith [abs_nonneg (measurement - b)]

theorem heightOffset_reset_innovation (origin measurement bias position delta : ℝ) :
    (origin - (measurement - (bias + delta))) - (position + delta) =
      (origin - (measurement - bias)) - position := by ring

end
end FormalESKF.ESKF
