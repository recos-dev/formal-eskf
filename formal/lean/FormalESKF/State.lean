/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.RotationVector

/-!
# ESKF state semantics

Logical coordinates, not C++ object byte layout. Navigation vectors are NED;
`q_nb` is scalar-first and maps body to navigation. Attitude error has three
local, right-multiplicative coordinates, although the nominal quaternion has
four coefficients. Exact unit norm is a premise, not a property of arbitrary
stored C++ floating-point coefficients.
-/

namespace FormalESKF.ESKF

open SO3

@[ext] structure InsState where
  p_n : Vector3 ℝ
  v_n : Vector3 ℝ
  q_nb : Quaternion ℝ
  b_a : Vector3 ℝ
  b_g : Vector3 ℝ

@[ext] structure AhrsState where
  q_nb : Quaternion ℝ

@[ext] structure InsError where
  delta_p_n : Vector3 ℝ
  delta_v_n : Vector3 ℝ
  delta_theta_b : Vector3 ℝ
  delta_b_a : Vector3 ℝ
  delta_b_g : Vector3 ℝ

@[ext] structure AhrsError where
  delta_theta_b : Vector3 ℝ

/-- Logical nominal order `[p_n, v_n, q0, q1, q2, q3, b_a, b_g]`. -/
def packInsState (s : InsState) : Fin 16 → ℝ :=
  ![s.p_n 0, s.p_n 1, s.p_n 2, s.v_n 0, s.v_n 1, s.v_n 2,
    s.q_nb.q0, s.q_nb.q1, s.q_nb.q2, s.q_nb.q3,
    s.b_a 0, s.b_a 1, s.b_a 2, s.b_g 0, s.b_g 1, s.b_g 2]

def unpackInsState (x : Fin 16 → ℝ) : InsState :=
  ⟨![x 0, x 1, x 2], ![x 3, x 4, x 5], ⟨x 6, x 7, x 8, x 9⟩,
    ![x 10, x 11, x 12], ![x 13, x 14, x 15]⟩

def packAhrsState (s : AhrsState) : Fin 4 → ℝ :=
  ![s.q_nb.q0, s.q_nb.q1, s.q_nb.q2, s.q_nb.q3]

def unpackAhrsState (x : Fin 4 → ℝ) : AhrsState :=
  ⟨⟨x 0, x 1, x 2, x 3⟩⟩

/-- The five three-coordinate INS error blocks; no fourth attitude error. -/
def packInsError (e : InsError) : Fin 15 → ℝ :=
  ![e.delta_p_n 0, e.delta_p_n 1, e.delta_p_n 2,
    e.delta_v_n 0, e.delta_v_n 1, e.delta_v_n 2,
    e.delta_theta_b 0, e.delta_theta_b 1, e.delta_theta_b 2,
    e.delta_b_a 0, e.delta_b_a 1, e.delta_b_a 2,
    e.delta_b_g 0, e.delta_b_g 1, e.delta_b_g 2]

/-- Corresponds to production `detail::unpack_correction` offsets 0,3,6,9,12. -/
def unpackInsError (x : Fin 15 → ℝ) : InsError :=
  ⟨![x 0, x 1, x 2], ![x 3, x 4, x 5], ![x 6, x 7, x 8],
    ![x 9, x 10, x 11], ![x 12, x 13, x 14]⟩

def packAhrsError (e : AhrsError) : Fin 3 → ℝ := e.delta_theta_b
def unpackAhrsError (x : Fin 3 → ℝ) : AhrsError := ⟨x⟩

theorem unpack_pack_insState (s : InsState) : unpackInsState (packInsState s) = s := by
  ext i <;> first | rfl | (fin_cases i <;> rfl)

theorem pack_unpack_insState (x : Fin 16 → ℝ) : packInsState (unpackInsState x) = x := by
  funext i
  fin_cases i <;> simp [unpackInsState, packInsState]

theorem unpack_pack_ahrsState (s : AhrsState) : unpackAhrsState (packAhrsState s) = s := by
  ext <;> rfl

theorem pack_unpack_ahrsState (x : Fin 4 → ℝ) : packAhrsState (unpackAhrsState x) = x := by
  funext i
  fin_cases i <;> rfl

theorem unpack_pack_insError (e : InsError) : unpackInsError (packInsError e) = e := by
  ext i <;> fin_cases i <;> simp [unpackInsError, packInsError]

theorem pack_unpack_insError (x : Fin 15 → ℝ) : packInsError (unpackInsError x) = x := by
  funext i
  fin_cases i <;> simp [unpackInsError, packInsError]

theorem unpack_pack_ahrsError (e : AhrsError) : unpackAhrsError (packAhrsError e) = e := rfl
theorem pack_unpack_ahrsError (x : Fin 3 → ℝ) : packAhrsError (unpackAhrsError x) = x := rfl

def zeroInsError : InsError := ⟨0, 0, 0, 0, 0⟩
def zeroAhrsError : AhrsError := ⟨0⟩
def initialInsState : InsState := ⟨0, 0, Quaternion.identity, 0, 0⟩
def initialAhrsState : AhrsState := ⟨Quaternion.identity⟩

theorem unpackInsError_zero : unpackInsError 0 = zeroInsError := by
  ext i <;> fin_cases i <;> rfl

theorem unpackAhrsError_zero : unpackAhrsError 0 = zeroAhrsError := rfl

/-- The chosen local attitude coordinates; this is not a covariance reset. -/
noncomputable def localAttitude (q_nb : Quaternion ℝ) (delta_theta_b : Vector3 ℝ) :=
  Quaternion.hamiltonMul q_nb (quaternionExp delta_theta_b)

theorem localAttitude_zero (q_nb : Quaternion ℝ) : localAttitude q_nb 0 = q_nb := by
  simp [localAttitude]

theorem localAttitude_hasUnitNorm {q_nb : Quaternion ℝ}
    (hq : Quaternion.HasUnitNorm q_nb) (delta_theta_b : Vector3 ℝ) :
    Quaternion.HasUnitNorm (localAttitude q_nb delta_theta_b) :=
  hq.hamiltonMul (quaternionExp_hasUnitNorm delta_theta_b)

/-- The body-frame increment acts before the body-to-navigation attitude. -/
theorem localAttitude_action (q_nb : Quaternion ℝ) (delta_theta_b v_b : Vector3 ℝ) :
    rotate (localAttitude q_nb delta_theta_b) v_b =
      rotate q_nb (rotate (quaternionExp delta_theta_b) v_b) :=
  rotate_hamiltonMul _ _ _

end FormalESKF.ESKF
