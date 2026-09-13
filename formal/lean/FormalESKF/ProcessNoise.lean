/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.Blocks

/-!
# Discrete process-noise covariance

Exact model of `process_noise.hpp`: measurement variances use dt²; independent
bias driving-noise densities use dt. This is the selected impulse-noise model,
not an exact continuous-time joint discretization. Source correlations are
absent by assumption. Navigation-frame rotation belongs to noise injection.
-/

namespace FormalESKF.ESKF

open SO3 Matrix

structure InsNoise where
  specific_force_variance : Vector3 ℝ
  angular_rate_variance : Vector3 ℝ
  accelerometer_bias_random_walk_variance_density : Vector3 ℝ
  gyroscope_bias_random_walk_variance_density : Vector3 ℝ

def noiseGroups (noise : InsNoise) : Fin 4 → Vector3 ℝ :=
  ![noise.specific_force_variance, noise.angular_rate_variance,
    noise.accelerometer_bias_random_walk_variance_density,
    noise.gyroscope_bias_random_walk_variance_density]

def ValidInsNoise (noise : InsNoise) : Prop := ∀ b i, 0 ≤ noiseGroups noise b i

def measurementImpulse (variance : Vector3 ℝ) (dt : ℝ) : Matrix3 ℝ :=
  diagonal (fun i => (variance i * dt) * dt)

def biasImpulse (density : Vector3 ℝ) (dt : ℝ) : Matrix3 ℝ :=
  diagonal (fun i => density i * dt)

def noiseBlocks (noise : InsNoise) (dt : ℝ) : Fin 4 → Matrix3 ℝ :=
  ![measurementImpulse noise.specific_force_variance dt,
    measurementImpulse noise.angular_rate_variance dt,
    biasImpulse noise.accelerometer_bias_random_walk_variance_density dt,
    biasImpulse noise.gyroscope_bias_random_walk_variance_density dt]

def insProcessNoise (noise : InsNoise) (dt : ℝ) : Covariance 12 :=
  diagonal (fun i =>
    let b := ((finProdFinEquiv (m := 4) (n := 3)).symm i).1
    let a := ((finProdFinEquiv (m := 4) (n := 3)).symm i).2
    if b.val < 2 then (noiseGroups noise b a * dt) * dt
    else noiseGroups noise b a * dt)

theorem measurementImpulse_scaling (v : Vector3 ℝ) (dt : ℝ) :
    measurementImpulse v dt = dt ^ 2 • diagonal v := by
  ext i j
  by_cases hij : i = j <;> simp [measurementImpulse, hij]
  ring

theorem biasImpulse_scaling (v : Vector3 ℝ) (dt : ℝ) :
    biasImpulse v dt = dt • diagonal v := by
  ext i j
  by_cases hij : i = j <;> simp [biasImpulse, hij]
  ring

theorem measurementImpulse_posSemidef {v : Vector3 ℝ} (hv : ∀ i, 0 ≤ v i) (dt : ℝ) :
    (measurementImpulse v dt).PosSemidef := by
  rw [measurementImpulse_scaling]
  exact (Matrix.PosSemidef.diagonal hv).smul (sq_nonneg dt)

theorem biasImpulse_posSemidef {v : Vector3 ℝ} (hv : ∀ i, 0 ≤ v i)
    {dt : ℝ} (ht : 0 ≤ dt) : (biasImpulse v dt).PosSemidef := by
  rw [biasImpulse_scaling]
  exact (Matrix.PosSemidef.diagonal hv).smul ht

theorem insProcessNoise_posSemidef {noise : InsNoise} (hn : ValidInsNoise noise)
    {dt : ℝ} (ht : 0 ≤ dt) : (insProcessNoise noise dt).PosSemidef := by
  apply Matrix.PosSemidef.diagonal
  intro i
  dsimp
  split_ifs
  · exact mul_nonneg (mul_nonneg (hn _ _) ht) ht
  · exact mul_nonneg (hn _ _) ht

/-- The twelve source coordinates have precisely the four C++ blocks. -/
theorem insProcessNoise_blocks (noise : InsNoise) (dt : ℝ) :
    insProcessNoise noise dt =
      blocks (fun b c : Fin 4 => if b = c then noiseBlocks noise dt b else 0) := by
  apply blocks_ext (n := 4) (m := 4)
  intro b c i j
  rw [blocks_apply]
  simp only [insProcessNoise, diagonal_apply, blockIndex, Equiv.apply_eq_iff_eq,
    Prod.mk.injEq, Equiv.symm_apply_apply]
  fin_cases b <;> fin_cases c <;> fin_cases i <;> fin_cases j <;>
    simp [noiseGroups, noiseBlocks, measurementImpulse, biasImpulse]

theorem measurementImpulse_zero_step (v : Vector3 ℝ) : measurementImpulse v 0 = 0 := by
  simp [measurementImpulse]

theorem biasImpulse_zero_step (v : Vector3 ℝ) : biasImpulse v 0 = 0 := by
  simp [biasImpulse]

theorem insProcessNoise_zero_step (noise : InsNoise) : insProcessNoise noise 0 = 0 := by
  simp [insProcessNoise]

theorem insProcessNoise_zero (dt : ℝ) : insProcessNoise ⟨0, 0, 0, 0⟩ dt = 0 := by
  have hz : noiseGroups ⟨0, 0, 0, 0⟩ = 0 := by funext b; fin_cases b <;> rfl
  simp [insProcessNoise, hz]

end FormalESKF.ESKF
