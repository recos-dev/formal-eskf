/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/
import FormalESKF.AccelerometerFilter
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

/-- The variable-step acceleration LPF is a convex combination for positive dt. -/
theorem magneticAccelerationWeight {dt : ℝ} (hd : 0 < dt) :
    0 < dt / (1 + dt) ∧ dt / (1 + dt) < 1 := by
  have hp : 0 < 1 + dt := by linarith
  exact ⟨div_pos hd hp, (div_lt_one hp).2 (by linarith)⟩

/-- Exact-real bound; the separate IEEE invariant uses a conservative 2*pi bound. -/
theorem magneticHeadingFilter_bounds {prior sample : ℝ}
    (hp : -Real.pi ≤ prior ∧ prior ≤ Real.pi)
    (hs : -Real.pi ≤ sample ∧ sample ≤ Real.pi) :
    -Real.pi ≤ forceLowPass (1 / 10) prior sample ∧
      forceLowPass (1 / 10) prior sample ≤ Real.pi := by
  exact forceLowPass_bounds (by norm_num) (by norm_num) hp.1 hp.2 hs.1 hs.2

/-- Numerical consistency latch, not a proof of physical observability or correct yaw. -/
def magneticHeadingLatch (prior observable : Bool) (filtered noise : ℝ) : Bool :=
  if noise ≤ |filtered| then false else if observable then true else prior

theorem magneticHeadingLatch_reject {prior observable : Bool} {filtered noise : ℝ}
    (h : noise ≤ |filtered|) : magneticHeadingLatch prior observable filtered noise = false := by
  simp [magneticHeadingLatch, h]

theorem magneticHeadingLatch_hold {prior : Bool} {filtered noise : ℝ}
    (h : |filtered| < noise) : magneticHeadingLatch prior false filtered noise = prior := by
  simp [magneticHeadingLatch, not_le.mpr h]

theorem magneticHeadingLatch_acquire {prior : Bool} {filtered noise : ℝ}
    (h : |filtered| < noise) : magneticHeadingLatch prior true filtered noise = true := by
  simp [magneticHeadingLatch, not_le.mpr h]

end
end FormalESKF.ESKF
