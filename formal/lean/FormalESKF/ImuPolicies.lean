/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.CovariancePrediction

/-! Exact noise-policy consequences, independent of detector calibration,
caller scheduling and IEEE covariance error. C++ failure/alias frames are
checked separately by the actual portable selector/limit harnesses. -/
namespace FormalESKF.ESKF
noncomputable section
open Matrix SO3

def selectImuNoise (noise : InsNoise) (faulted : Fin 3 → Bool) (fault : ℝ) : InsNoise :=
  { noise with specific_force_variance := fun i => if faulted i then fault else noise.specific_force_variance i }

theorem selectImuNoise_valid {noise : InsNoise} (hn : ValidInsNoise noise)
    (faulted : Fin 3 → Bool) {fault : ℝ} (hf : 0 ≤ fault) :
    ValidInsNoise (selectImuNoise noise faulted fault) := by
  intro b i
  fin_cases b
  · change 0 ≤ (if faulted i then fault else noise.specific_force_variance i)
    split_ifs
    · exact hf
    · exact hn 0 i
  · exact hn 1 i
  · exact hn 2 i
  · exact hn 3 i

def inhibitIntensity (prior measured density : ℝ) : ℝ :=
  if measured ≤ prior then 0 else density

theorem inhibitIntensity_nonnegative (prior measured : ℝ) {density : ℝ} (hd : 0 ≤ density) :
    0 ≤ inhibitIntensity prior measured density := by
  unfold inhibitIntensity
  split_ifs
  · exact le_rfl
  · exact hd

theorem inhibitIntensity_threshold (variance density : ℝ) :
    inhibitIntensity variance variance density = 0 := by simp [inhibitIntensity]

def inhibitBiasNoise (P : Covariance 15) (noise : InsNoise) : InsNoise :=
  { noise with
    accelerometer_bias_random_walk_variance_density := fun i =>
      inhibitIntensity (P (blockIndex (n := 5) 3 i) (blockIndex (n := 5) 3 i)) (noise.specific_force_variance i)
        (noise.accelerometer_bias_random_walk_variance_density i)
    gyroscope_bias_random_walk_variance_density := fun i =>
      inhibitIntensity (P (blockIndex (n := 5) 4 i) (blockIndex (n := 5) 4 i)) (noise.angular_rate_variance i)
        (noise.gyroscope_bias_random_walk_variance_density i) }

theorem inhibitBiasNoise_valid (P : Covariance 15) {noise : InsNoise} (hn : ValidInsNoise noise) :
    ValidInsNoise (inhibitBiasNoise P noise) := by
  intro b i
  fin_cases b
  · exact hn 0 i
  · exact hn 1 i
  · exact inhibitIntensity_nonnegative _ _ (hn 2 i)
  · exact inhibitIntensity_nonnegative _ _ (hn 3 i)

/-- Bias transition rows are identity: cross-covariances do not change the
prior bias variances before adding this step's bias driving noise. -/
theorem insTransition_bias_variance (mode : AttitudeMode) (s : InsState)
    (f w : Vector3 ℝ) (dt : ℝ) (P : Covariance 15) (b : Fin 5)
    (hb : b = 3 ∨ b = 4) (i : Fin 3) :
    sandwich (insTransition mode s f w dt) P (blockIndex b i) (blockIndex b i) =
      P (blockIndex b i) (blockIndex b i) := by
  conv_lhs => rw [← blocks_matrixBlocks (n := 5) (m := 5) P]
  unfold sandwich insTransition
  rw [blocks_transpose (n := 5) (m := 5),
    blocks_mul (n := 5) (m := 5) (k := 5), blocks_mul (n := 5) (m := 5) (k := 5),
    blocks_apply]
  rcases hb with rfl | rfl <;> simp [insTransitionBlocks, Fin.sum_univ_succ, matrixBlocks]

theorem selected_prediction_posSemidef (mode : AttitudeMode) (s : InsState)
    {P : Covariance 15} (hP : P.PosSemidef) (f w : Vector3 ℝ)
    {noise : InsNoise} (hn : ValidInsNoise noise) (faulted : Fin 3 → Bool)
    {fault dt : ℝ} (hf : 0 ≤ fault) (ht : 0 ≤ dt) :
    (predictInsCovariance mode s P f w (inhibitBiasNoise P (selectImuNoise noise faulted fault)) dt).PosSemidef :=
  predictInsCovariance_posSemidef mode s hP f w (inhibitBiasNoise_valid P (selectImuNoise_valid hn faulted hf)) ht

end
end FormalESKF.ESKF
