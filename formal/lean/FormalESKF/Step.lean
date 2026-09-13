/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.CovariancePrediction
import FormalESKF.Observations

/-!
# Composition of ESKF steps

Pure successful-result semantics of the combined prediction API, followed
by the separate correction API. Both predictions consume the SAME prior.
The multi-step invariant is mathematical unit norm and PSD; finite arithmetic,
validation, exceptions, aliasing and atomic output publication remain C++
verification obligations. No scheduler, sensor ordering or new runtime API is
introduced by this model.
-/

namespace FormalESKF.ESKF

noncomputable section

open SO3 Quaternion Matrix

abbrev InsEstimate := InsState × Covariance 15
abbrev AhrsEstimate := AhrsState × Covariance 3

def ValidInsEstimate (x : InsEstimate) : Prop := HasUnitNorm x.1.q_nb ∧ x.2.PosSemidef
def ValidAhrsEstimate (x : AhrsEstimate) : Prop := HasUnitNorm x.1.q_nb ∧ x.2.PosSemidef

def predictInsStep (mode : AttitudeMode) (x : InsEstimate) (f w g : Vector3 ℝ)
    (noise : InsNoise) (dt : ℝ) : InsEstimate :=
  (predictIns mode x.1 f w g dt, predictInsCovariance mode x.1 x.2 f w noise dt)

def predictAhrsStep (mode : AttitudeMode) (x : AhrsEstimate) (w gyroVariance : Vector3 ℝ)
    (dt : ℝ) : AhrsEstimate :=
  (predictAhrs mode x.1 w dt, predictAhrsCovariance mode x.2 w gyroVariance dt)

def correctInsStep {m : ℕ} (mode : ResetMode) (x : InsEstimate) (r : Fin m → ℝ)
    (H : Matrix (Fin m) (Fin 15) ℝ) (V : Covariance m) : InsEstimate :=
  let result := correctIns mode x.1 x.2 r H V
  (result.1, result.2.1)

def correctAhrsStep {m : ℕ} (mode : ResetMode) (x : AhrsEstimate) (r : Fin m → ℝ)
    (H : Matrix (Fin m) (Fin 3) ℝ) (V : Covariance m) : AhrsEstimate :=
  let result := correctAhrs mode x.1 x.2 r H V
  (result.1, result.2.1)

theorem predictInsStep_valid (mode : AttitudeMode) {x : InsEstimate} (hx : ValidInsEstimate x)
    (f w g : Vector3 ℝ) {noise : InsNoise} (hn : ValidInsNoise noise) {dt : ℝ} (ht : 0 ≤ dt) :
    ValidInsEstimate (predictInsStep mode x f w g noise dt) :=
  ⟨predictIns_hasUnitNorm mode x.1 hx.1 f w g dt,
    predictInsCovariance_posSemidef mode x.1 hx.2 f w hn ht⟩

theorem predictAhrsStep_valid (mode : AttitudeMode) {x : AhrsEstimate} (hx : ValidAhrsEstimate x)
    (w : Vector3 ℝ) {gyroVariance : Vector3 ℝ} (hn : ∀ i, 0 ≤ gyroVariance i) (dt : ℝ) :
    ValidAhrsEstimate (predictAhrsStep mode x w gyroVariance dt) :=
  ⟨predictAhrs_hasUnitNorm mode x.1 hx.1 w dt,
    predictAhrsCovariance_posSemidef mode hx.2 w hn dt⟩

theorem correctInsStep_valid {m : ℕ} (mode : ResetMode) {x : InsEstimate} (hx : ValidInsEstimate x)
    (r : Fin m → ℝ) (H : Matrix (Fin m) (Fin 15) ℝ) {V : Covariance m} (hV : V.PosDef) :
    ValidInsEstimate (correctInsStep mode x r H V) := by
  have h := correctIns_invariants mode hx.1 hx.2 r H hV
  exact ⟨h.1, h.2.1⟩

theorem correctAhrsStep_valid {m : ℕ} (mode : ResetMode) {x : AhrsEstimate} (hx : ValidAhrsEstimate x)
    (r : Fin m → ℝ) (H : Matrix (Fin m) (Fin 3) ℝ) {V : Covariance m} (hV : V.PosDef) :
    ValidAhrsEstimate (correctAhrsStep mode x r H V) := by
  have h := correctAhrs_invariants mode hx.1 hx.2 r H hV
  exact ⟨h.1, h.2.1⟩

/-- A sensor can evaluate its model at the predicted state; the result is
still covered by the generic correction proof, with the full supplied V. -/
theorem predict_correct_ins_valid {m : ℕ} (predictionMode : AttitudeMode) (resetMode : ResetMode)
    {x : InsEstimate} (hx : ValidInsEstimate x) (f w g : Vector3 ℝ)
    {noise : InsNoise} (hn : ValidInsNoise noise) {dt : ℝ} (ht : 0 ≤ dt)
    (r : InsState → Fin m → ℝ) (H : InsState → Matrix (Fin m) (Fin 15) ℝ)
    {V : Covariance m} (hV : V.PosDef) :
    let prior := predictInsStep predictionMode x f w g noise dt
    ValidInsEstimate (correctInsStep resetMode prior (r prior.1) (H prior.1) V) := by
  exact correctInsStep_valid resetMode (predictInsStep_valid predictionMode hx f w g hn ht) _ _ hV

theorem predict_correct_ahrs_valid {m : ℕ} (predictionMode : AttitudeMode) (resetMode : ResetMode)
    {x : AhrsEstimate} (hx : ValidAhrsEstimate x) (w : Vector3 ℝ)
    {gyroVariance : Vector3 ℝ} (hn : ∀ i, 0 ≤ gyroVariance i) (dt : ℝ)
    (r : AhrsState → Fin m → ℝ) (H : AhrsState → Matrix (Fin m) (Fin 3) ℝ)
    {V : Covariance m} (hV : V.PosDef) :
    let prior := predictAhrsStep predictionMode x w gyroVariance dt
    ValidAhrsEstimate (correctAhrsStep resetMode prior (r prior.1) (H prior.1) V) := by
  exact correctAhrsStep_valid resetMode (predictAhrsStep_valid predictionMode hx w hn dt) _ _ hV

/-- Induction over any finite sequence of invariant-preserving steps. The
premise is discharged by the prediction/correction theorems above, not by
assuming that a whole estimator execution is correct. -/
theorem finite_steps_preserve {α : Type*} (valid : α → Prop) (steps : List (α → α))
    (hsteps : ∀ step ∈ steps, ∀ x, valid x → valid (step x)) {x : α} (hx : valid x) :
    valid (steps.foldl (fun state step => step state) x) := by
  induction steps generalizing x with
  | nil => exact hx
  | cons step rest ih =>
    apply ih
    · intro next hn
      exact hsteps next (by simp [hn])
    · exact hsteps step (by simp) x hx

end

end FormalESKF.ESKF
