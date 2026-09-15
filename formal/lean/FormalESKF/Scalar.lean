/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import Mathlib.Analysis.Real.Pi.Bounds
import Mathlib.Analysis.SpecialFunctions.Complex.Arg

/-! Scalar contracts have two separate models: abstract checked publication,
with explicit finite/domain predicates, and exact real elementary functions.
Neither model represents IEEE arithmetic or proves the accuracy of libm.
ESBMC separately checks the predicates and production wrapper control flow.
NaN payloads, exception flags and libm side effects are outside this model. -/

namespace FormalESKF.Scalar

noncomputable section

inductive Status where
  | success | nonFiniteInput | domainError | nonFiniteResult
  deriving DecidableEq

def checked {α : Type*} (inputFinite domainValid resultFinite : Bool) (old candidate : α) :
    Status × α :=
  if !inputFinite then (.nonFiniteInput, old)
  else if !domainValid then (.domainError, old)
  else if !resultFinite then (.nonFiniteResult, old)
  else (.success, candidate)

theorem checked_success_iff {α : Type*} (i d f : Bool) (old candidate : α) :
    (checked i d f old candidate).1 = .success ↔ i = true ∧ d = true ∧ f = true := by
  cases i <;> cases d <;> cases f <;> simp [checked]

theorem checked_failure_preserves {α : Type*} (i d f : Bool) (old candidate : α)
    (h : (checked i d f old candidate).1 ≠ .success) :
    (checked i d f old candidate).2 = old := by
  cases i <;> cases d <;> cases f <;> simp_all [checked]

theorem checked_success_publishes {α : Type*} (i d f : Bool) (old candidate : α)
    (h : (checked i d f old candidate).1 = .success) :
    (checked i d f old candidate).2 = candidate := by
  cases i <;> cases d <;> cases f <;> simp_all [checked]

theorem checked_error_precedence {α : Type*} (d f : Bool) (old candidate : α) :
    checked false d f old candidate = (.nonFiniteInput, old) ∧
    checked true false f old candidate = (.domainError, old) ∧
    checked true true false old candidate = (.nonFiniteResult, old) := by simp [checked]

-- These predicates are abstract, not Real predicates pretending to cover NaN.
def checkedSqrt {α : Type*} (finite negative : α → Bool) (x old root : α) :=
  checked (finite x) (!negative x) (finite root) old root

def checkedSinCos {α : Type*} (finite : α → Bool) (x : α) (old candidate : α × α) :=
  checked (finite x) true (finite candidate.1 && finite candidate.2) old candidate

def checkedAtan2 {α : Type*} (finite zero : α → Bool) (y x old angle : α) :=
  checked (finite y && finite x) (!(zero y && zero x)) (finite angle) old angle

theorem checkedSqrt_success {α : Type*} (finite negative : α → Bool) (x old root : α) :
    (checkedSqrt finite negative x old root).1 = .success ↔
    finite x = true ∧ negative x = false ∧ finite root = true := by
  simp [checkedSqrt, checked_success_iff]

theorem checkedSinCos_success {α : Type*} (finite : α → Bool) (x : α) (old candidate : α × α) :
    (checkedSinCos finite x old candidate).1 = .success ↔
    finite x = true ∧ finite candidate.1 = true ∧ finite candidate.2 = true := by
  simp [checkedSinCos, checked_success_iff]

theorem checkedAtan2_success {α : Type*} (finite zero : α → Bool) (y x old angle : α) :
    (checkedAtan2 finite zero y x old angle).1 = .success ↔
    (finite y = true ∧ finite x = true) ∧
    (zero y = false ∨ zero x = false) ∧ finite angle = true := by
  simp [checkedAtan2, checked_success_iff]

theorem sqrt_spec (x : ℝ) (h : 0 ≤ x) : 0 ≤ Real.sqrt x ∧ (Real.sqrt x) ^ 2 = x :=
  ⟨Real.sqrt_nonneg x, Real.sq_sqrt h⟩

def sinCos (x : ℝ) : ℝ × ℝ := (Real.sin x, Real.cos x)

theorem sinCos_unit (x : ℝ) : (sinCos x).1 ^ 2 + (sinCos x).2 ^ 2 = 1 :=
  Real.sin_sq_add_cos_sq x

theorem sinCos_zero : sinCos 0 = (0, 1) := by simp [sinCos]

-- Exact Arg selects (-pi, pi]; IEEE atan2 can also return -pi on the negative
-- axis with a negative signed zero. Real numbers do not distinguish those zeros.
-- The checked API rejects the origin even though Complex.arg is total there.
def atan2 (y x : ℝ) : ℝ := Complex.arg ⟨x, y⟩

theorem atan2_range (y x : ℝ) : -Real.pi < atan2 y x ∧ atan2 y x ≤ Real.pi :=
  Complex.arg_mem_Ioc ⟨x, y⟩

theorem atan2_direction (y x : ℝ) (h : (⟨x, y⟩ : ℂ) ≠ 0) :
    Real.sin (atan2 y x) = y / ‖(⟨x, y⟩ : ℂ)‖ ∧
    Real.cos (atan2 y x) = x / ‖(⟨x, y⟩ : ℂ)‖ :=
  ⟨Complex.sin_arg _, Complex.cos_arg h⟩

theorem atan2_negative_axis (x : ℝ) (h : x < 0) : atan2 0 x = Real.pi := by
  exact Complex.arg_eq_pi_iff.mpr ⟨h, rfl⟩

-- Exact rationals represented by 0x40490fdb / 0x400921fb54442d18. The encoding
-- correspondence is reviewed against ESBMC/native bit checks, not a Lean IEEE
-- refinement. These bounds qualify the constant, not transcendental results.
def pi32 : ℝ := 13176795 / 4194304
def pi64 : ℝ := 884279719003555 / 281474976710656

theorem pi32_half_ulp : |pi32 - Real.pi| < 1 / 8388608 := by
  rw [abs_lt]
  have hlo := Real.pi_gt_d20
  have hhi := Real.pi_lt_d20
  norm_num [pi32] at *
  constructor <;> linarith

theorem pi64_half_ulp : |pi64 - Real.pi| < 1 / 4503599627370496 := by
  rw [abs_lt]
  have hlo := Real.pi_gt_d20
  have hhi := Real.pi_lt_d20
  norm_num [pi64] at *
  constructor <;> linarith

end

end FormalESKF.Scalar
