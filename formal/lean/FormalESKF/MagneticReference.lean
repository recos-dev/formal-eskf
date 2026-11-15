/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/
import FormalESKF.SO3
import Mathlib.Analysis.SpecialFunctions.Trigonometric.Basic
import Mathlib.Tactic.Linarith

/-! Declination/inclination/strength to a NED field. These exact identities
say nothing about the accuracy or geographic validity of an external model. -/
namespace FormalESKF.ESKF
noncomputable section

def magneticReferenceField (declination inclination strength : ℝ) : SO3.Vector3 ℝ :=
  ![strength * Real.cos inclination * Real.cos declination,
    strength * Real.cos inclination * Real.sin declination,
    strength * Real.sin inclination]

theorem magneticReferenceField_normSquared (d i b : ℝ) :
    SO3.vectorNormSquared (magneticReferenceField d i b) = b^2 := by
  simp [SO3.vectorNormSquared, dotProduct, magneticReferenceField, Fin.sum_univ_succ]
  calc
    _ = b^2 * ((Real.cos i)^2 * ((Real.sin d)^2 + (Real.cos d)^2) + (Real.sin i)^2) := by ring
    _ = b^2 := by rw [Real.sin_sq_add_cos_sq]; nlinarith [Real.sin_sq_add_cos_sq i]

theorem magneticReferenceField_north (b : ℝ) :
    magneticReferenceField 0 0 b = ![b, 0, 0] := by simp [magneticReferenceField]

theorem magneticReferenceField_down (d i b : ℝ) :
    magneticReferenceField d i b 2 = b * Real.sin i := by simp [magneticReferenceField]

theorem magneticReferenceField_horizontal (d i b : ℝ) :
    (magneticReferenceField d i b 0)^2 + (magneticReferenceField d i b 1)^2 =
      (b * Real.cos i)^2 := by
  simp only [magneticReferenceField, Matrix.cons_val_zero, Matrix.cons_val_one]
  calc
    _ = (b * Real.cos i)^2 * ((Real.sin d)^2 + (Real.cos d)^2) := by ring
    _ = (b * Real.cos i)^2 := by rw [Real.sin_sq_add_cos_sq, mul_one]

end
end FormalESKF.ESKF
