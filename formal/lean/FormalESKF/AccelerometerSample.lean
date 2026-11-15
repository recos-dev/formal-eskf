/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/
import FormalESKF.SO3

/-! Real endpoint-rotation accumulation. This is a discrete trapezoidal rule,
not an exact continuous-time or sculling integration theorem. -/
namespace FormalESKF.ESKF
noncomputable section
open SO3

def accumulateForce (q : Quaternion ℝ) (prior increment : Vector3 ℝ) : Vector3 ℝ :=
  fun i => inverseRotate q prior i + (increment i + inverseRotate q increment i) / 2

theorem accumulateForce_identity (prior increment : Vector3 ℝ) :
    accumulateForce Quaternion.identity prior increment = prior + increment := by
  ext i
  simp [accumulateForce, inverseRotate]

theorem accumulateForce_zero_increment (q : Quaternion ℝ) (prior : Vector3 ℝ) :
    accumulateForce q prior 0 = inverseRotate q prior := by
  ext i
  simp [accumulateForce, inverseRotate]

theorem accumulateForce_zero_prior (q : Quaternion ℝ) (increment : Vector3 ℝ) :
    accumulateForce q 0 increment = fun i => (increment i + inverseRotate q increment i) / 2 := by
  ext i
  simp [accumulateForce, inverseRotate]

theorem accumulateForce_transport (q : Quaternion ℝ) (prior increment : Vector3 ℝ) :
    accumulateForce q prior increment = fun i =>
      inverseRotate q (prior + increment) i + (increment i - inverseRotate q increment i) / 2 := by
  ext i
  simp only [accumulateForce, inverseRotate, Matrix.mulVec_add, Pi.add_apply]
  ring

end
end FormalESKF.ESKF
