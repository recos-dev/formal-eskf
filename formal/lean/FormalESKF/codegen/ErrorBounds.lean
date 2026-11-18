/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import Mathlib.Data.Real.Basic
import Mathlib.Tactic.Linarith
import Mathlib.Tactic.Ring

/-!
Forward-error propagation over real values, conditional on the local rounding
inequality. This does not axiomatize or prove a particular IEEE implementation.
The executable rational checker supplies per-operation finite/RNE witnesses.
-/
namespace Codegen

theorem addition_input_error {a b x y ea eb : ℝ}
    (ha : |x-a| ≤ ea) (hb : |y-b| ≤ eb) :
    |(x+y)-(a+b)| ≤ ea+eb := by
  calc
    |(x+y)-(a+b)| = |(x-a)+(y-b)| := by congr 1; ring
    _ ≤ |x-a|+|y-b| := abs_add_le _ _
    _ ≤ ea+eb := add_le_add ha hb

theorem multiplication_input_error {a b x y ea eb : ℝ}
    (ha : |x-a| ≤ ea) (hb : |y-b| ≤ eb) :
    |x*y-a*b| ≤ |a| *eb+|b| *ea+ea*eb := by
  have hea : 0 ≤ ea := (abs_nonneg _).trans ha
  calc
    |x*y-a*b| = |a*(y-b)+b*(x-a)+(x-a)*(y-b)| := by congr 1; ring
    _ ≤ |a*(y-b)+b*(x-a)|+|(x-a)*(y-b)| := abs_add_le _ _
    _ ≤ |a*(y-b)|+|b*(x-a)|+|(x-a)*(y-b)| :=
      add_le_add (abs_add_le _ _) (le_refl _)
    _ = |a| *|y-b|+|b| *|x-a|+|x-a| *|y-b| := by simp only [abs_mul]
    _ ≤ |a| *eb+|b| *ea+ea*eb :=
      add_le_add
        (add_le_add (mul_le_mul_of_nonneg_left hb (abs_nonneg a))
          (mul_le_mul_of_nonneg_left ha (abs_nonneg b)))
        (mul_le_mul ha hb (abs_nonneg _) hea)

theorem local_rounding_error {v w z d u eta : ℝ}
    (hu : 0 ≤ u) (hd : |w-v| ≤ d)
    (hr : |z-w| ≤ u*|w|+eta/2) :
    |z-v| ≤ (1+u)*d+u*|v|+eta/2 := by
  have hw : |w| ≤ |v|+d := by
    calc
      |w| = |v+(w-v)| := by congr 1; ring
      _ ≤ |v|+|w-v| := abs_add_le _ _
      _ ≤ |v|+d := add_le_add (le_refl _) hd
  calc
    |z-v| = |(z-w)+(w-v)| := by congr 1; ring
    _ ≤ |z-w|+|w-v| := abs_add_le _ _
    _ ≤ (u*|w|+eta/2)+d := add_le_add hr hd
    _ ≤ (u*(|v|+d)+eta/2)+d := by
      exact add_le_add (add_le_add (mul_le_mul_of_nonneg_left hw hu) (le_refl _)) (le_refl _)
    _ = (1+u)*d+u*|v|+eta/2 := by ring

theorem addition_rounding_bound {a b x y z ea eb u eta : ℝ}
    (hu : 0 ≤ u) (ha : |x-a| ≤ ea) (hb : |y-b| ≤ eb)
    (hr : |z-(x+y)| ≤ u*|x+y|+eta/2) :
    |z-(a+b)| ≤ (1+u)*(ea+eb)+u*|a+b|+eta/2 :=
  local_rounding_error hu (addition_input_error ha hb) hr

theorem multiplication_rounding_bound {a b x y z ea eb u eta : ℝ}
    (hu : 0 ≤ u) (ha : |x-a| ≤ ea) (hb : |y-b| ≤ eb)
    (hr : |z-x*y| ≤ u*|x*y|+eta/2) :
    |z-a*b| ≤ (1+u)*(|a| *eb+|b| *ea+ea*eb)+u*|a*b|+eta/2 :=
  local_rounding_error hu (multiplication_input_error ha hb) hr

theorem upward_bound_preserves_acceptance {actual expected bound roundedBound : ℝ}
    (h : |actual-expected| ≤ bound) (hup : bound ≤ roundedBound) :
    |actual-expected| ≤ roundedBound := h.trans hup

end Codegen

namespace Codegen

theorem negation_input_error {a x e : ℝ} (h : |x-a| ≤ e) : |-x-(-a)| ≤ e := by
  have heq : -x-(-a) = -(x-a) := by ring
  rw [heq, abs_neg]
  exact h

theorem subtraction_input_error {a b x y ea eb : ℝ}
    (ha : |x-a| ≤ ea) (hb : |y-b| ≤ eb) :
    |(x-y)-(a-b)| ≤ ea+eb := by
  simpa [sub_eq_add_neg] using addition_input_error ha (negation_input_error hb)

theorem subtraction_rounding_bound {a b x y z ea eb u eta : ℝ}
    (hu : 0 ≤ u) (ha : |x-a| ≤ ea) (hb : |y-b| ≤ eb)
    (hr : |z-(x-y)| ≤ u*|x-y|+eta/2) :
    |z-(a-b)| ≤ (1+u)*(ea+eb)+u*|a-b|+eta/2 :=
  local_rounding_error hu (subtraction_input_error ha hb) hr

#print axioms addition_rounding_bound
#print axioms multiplication_rounding_bound
#print axioms subtraction_rounding_bound
end Codegen

namespace Codegen

/-- Arithmetic grammar of the emitted scalar expressions, with references inlined. -/
inductive ArithmeticExpr where
  | input : ℕ → ArithmeticExpr
  | constant : ℝ → ArithmeticExpr
  | neg : ArithmeticExpr → ArithmeticExpr
  | add : ArithmeticExpr → ArithmeticExpr → ArithmeticExpr
  | sub : ArithmeticExpr → ArithmeticExpr → ArithmeticExpr
  | mul : ArithmeticExpr → ArithmeticExpr → ArithmeticExpr

namespace ArithmeticExpr
noncomputable section

def exactValue (inputs : ℕ → ℝ) : ArithmeticExpr → ℝ
  | input i => inputs i
  | constant c => c
  | neg a => -exactValue inputs a
  | add a b => exactValue inputs a + exactValue inputs b
  | sub a b => exactValue inputs a - exactValue inputs b
  | mul a b => exactValue inputs a * exactValue inputs b

def roundedValue (inputs : ℕ → ℝ) (rnd : ℝ → ℝ) : ArithmeticExpr → ℝ
  | input i => inputs i
  | constant c => c
  | neg a => -roundedValue inputs rnd a
  | add a b => rnd (roundedValue inputs rnd a + roundedValue inputs rnd b)
  | sub a b => rnd (roundedValue inputs rnd a - roundedValue inputs rnd b)
  | mul a b => rnd (roundedValue inputs rnd a * roundedValue inputs rnd b)

def bound (inputs : ℕ → ℝ) (up : ℝ → ℝ) (u eta : ℝ) : ArithmeticExpr → ℝ
  | input _ => 0
  | constant _ => 0
  | neg a => bound inputs up u eta a
  | add a b => up ((1+u)*(bound inputs up u eta a + bound inputs up u eta b) +
      u*|exactValue inputs a + exactValue inputs b| + eta/2)
  | sub a b => up ((1+u)*(bound inputs up u eta a + bound inputs up u eta b) +
      u*|exactValue inputs a - exactValue inputs b| + eta/2)
  | mul a b => up ((1+u)*(|exactValue inputs a| *bound inputs up u eta b +
      |exactValue inputs b| *bound inputs up u eta a +
      bound inputs up u eta a * bound inputs up u eta b) +
      u*|exactValue inputs a * exactValue inputs b| + eta/2)

/-- Only visited operations need a local rounding witness; no global no-overflow axiom. -/
def roundingValid (inputs : ℕ → ℝ) (rnd : ℝ → ℝ) (u eta : ℝ) : ArithmeticExpr → Prop
  | input _ => True
  | constant _ => True
  | neg a => roundingValid inputs rnd u eta a
  | add a b => roundingValid inputs rnd u eta a ∧ roundingValid inputs rnd u eta b ∧
      |rnd (roundedValue inputs rnd a + roundedValue inputs rnd b) -
        (roundedValue inputs rnd a + roundedValue inputs rnd b)| ≤
        u*|roundedValue inputs rnd a + roundedValue inputs rnd b| + eta/2
  | sub a b => roundingValid inputs rnd u eta a ∧ roundingValid inputs rnd u eta b ∧
      |rnd (roundedValue inputs rnd a - roundedValue inputs rnd b) -
        (roundedValue inputs rnd a - roundedValue inputs rnd b)| ≤
        u*|roundedValue inputs rnd a - roundedValue inputs rnd b| + eta/2
  | mul a b => roundingValid inputs rnd u eta a ∧ roundingValid inputs rnd u eta b ∧
      |rnd (roundedValue inputs rnd a * roundedValue inputs rnd b) -
        roundedValue inputs rnd a * roundedValue inputs rnd b| ≤
        u*|roundedValue inputs rnd a * roundedValue inputs rnd b| + eta/2

/-- The local rules imply the whole expression bound, including upward rounding of bounds. -/
theorem error_bound (e : ArithmeticExpr) (inputs : ℕ → ℝ) (rnd up : ℝ → ℝ) (u eta : ℝ)
    (hu : 0 ≤ u) (hup : ∀ x, x ≤ up x) (hr : roundingValid inputs rnd u eta e) :
    |roundedValue inputs rnd e - exactValue inputs e| ≤ bound inputs up u eta e := by
  induction e with
  | input i => simp [roundedValue, exactValue, bound]
  | constant c => simp [roundedValue, exactValue, bound]
  | neg a ih => exact negation_input_error (ih hr)
  | add a b ia ib =>
      exact (addition_rounding_bound hu (ia hr.1) (ib hr.2.1) hr.2.2).trans (hup _)
  | sub a b ia ib =>
      exact (subtraction_rounding_bound hu (ia hr.1) (ib hr.2.1) hr.2.2).trans (hup _)
  | mul a b ia ib =>
      exact (multiplication_rounding_bound hu (ia hr.1) (ib hr.2.1) hr.2.2).trans (hup _)

#print axioms error_bound
end
end ArithmeticExpr
end Codegen
