/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/
import Mathlib.Analysis.SpecialFunctions.Trigonometric.Basic
import Mathlib.Tactic.Linarith
import Mathlib.Tactic.SplitIfs

/-! Single-wrap magnetic heading residual. Principal-range conclusions require
principal input angles; finite C++ library output alone is not that premise. -/
namespace FormalESKF.ESKF
noncomputable section

def magneticHeadingWrap (x : ℝ) : ℝ :=
  if Real.pi < x then x - 2 * Real.pi else if x < -Real.pi then x + 2 * Real.pi else x

theorem magneticHeadingWrap_principal {x : ℝ}
    (hl : -2 * Real.pi ≤ x) (hu : x ≤ 2 * Real.pi) :
    -Real.pi ≤ magneticHeadingWrap x ∧ magneticHeadingWrap x ≤ Real.pi := by
  unfold magneticHeadingWrap
  split_ifs <;> constructor <;> linarith [Real.pi_pos]

theorem magneticHeadingWrap_unchanged {x : ℝ} (hl : -Real.pi ≤ x) (hu : x ≤ Real.pi) :
    magneticHeadingWrap x = x := by
  simp [magneticHeadingWrap, not_lt.mpr hu, not_lt.mpr hl]

theorem magneticHeadingWrap_difference {measured reference : ℝ}
    (hm : -Real.pi ≤ measured ∧ measured ≤ Real.pi)
    (hr : -Real.pi ≤ reference ∧ reference ≤ Real.pi) :
    -Real.pi ≤ magneticHeadingWrap (measured-reference) ∧
      magneticHeadingWrap (measured-reference) ≤ Real.pi := by
  apply magneticHeadingWrap_principal <;> linarith [hm.1, hm.2, hr.1, hr.2]

theorem magneticHeadingWrap_equal (heading : ℝ) : magneticHeadingWrap (heading-heading) = 0 := by
  rw [sub_self]
  exact magneticHeadingWrap_unchanged (by linarith [Real.pi_pos]) Real.pi_pos.le

end
end FormalESKF.ESKF
