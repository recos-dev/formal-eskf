/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/
import FormalESKF.MagneticReference

/-! Local geographic NED to fixed-origin NED magnetic reference conversion.
Angles are radians. These are basis identities, not a proof of Earth geometry,
external field accuracy, coordinate provenance or floating-point libm. -/
namespace FormalESKF.ESKF
noncomputable section
open Matrix SO3

def geographicNedBasis (latitude longitude : ℝ) : Matrix3 ℝ :=
  !![-Real.sin latitude * Real.cos longitude, -Real.sin latitude * Real.sin longitude, Real.cos latitude;
     -Real.sin longitude, Real.cos longitude, 0;
     -Real.cos latitude * Real.cos longitude, -Real.cos latitude * Real.sin longitude, -Real.sin latitude]

theorem geographicNedBasis_mul_transpose (latitude longitude : ℝ) :
    geographicNedBasis latitude longitude * (geographicNedBasis latitude longitude)ᵀ = 1 := by
  have ha : (Real.cos latitude)^2 = 1 - (Real.sin latitude)^2 := by nlinarith [Real.sin_sq_add_cos_sq latitude]
  have ho : (Real.cos longitude)^2 = 1 - (Real.sin longitude)^2 := by nlinarith [Real.sin_sq_add_cos_sq longitude]
  ext i j
  fin_cases i <;> fin_cases j <;>
    simp [geographicNedBasis, Matrix.mul_apply, Fin.sum_univ_succ] <;>
    ring_nf <;> simp only [ha, ho] <;> ring

theorem geographicNedBasis_transpose_mul (latitude longitude : ℝ) :
    (geographicNedBasis latitude longitude)ᵀ * geographicNedBasis latitude longitude = 1 := by
  have ha : (Real.cos latitude)^2 = 1 - (Real.sin latitude)^2 := by nlinarith [Real.sin_sq_add_cos_sq latitude]
  have ho : (Real.cos longitude)^2 = 1 - (Real.sin longitude)^2 := by nlinarith [Real.sin_sq_add_cos_sq longitude]
  ext i j
  fin_cases i <;> fin_cases j <;>
    simp [geographicNedBasis, Matrix.mul_apply, Fin.sum_univ_succ] <;>
    ring_nf <;> simp only [ha, ho] <;> ring

theorem geographicNedBasis_det (latitude longitude : ℝ) :
    (geographicNedBasis latitude longitude).det = 1 := by
  simp [geographicNedBasis, Matrix.det_fin_three]
  calc
    _ = ((Real.sin latitude)^2 + (Real.cos latitude)^2) *
        ((Real.sin longitude)^2 + (Real.cos longitude)^2) := by ring
    _ = 1 := by rw [Real.sin_sq_add_cos_sq, Real.sin_sq_add_cos_sq]; norm_num

/-- Columns convert the local NED components to ECEF, then rows to origin NED. -/
def magneticFrameChange (originLatitude originLongitude latitude longitude : ℝ) : Matrix3 ℝ :=
  geographicNedBasis originLatitude originLongitude * (geographicNedBasis latitude longitude)ᵀ

theorem magneticFrameChange_same (latitude longitude : ℝ) :
    magneticFrameChange latitude longitude latitude longitude = 1 :=
  geographicNedBasis_mul_transpose latitude longitude

theorem magneticFrameChange_transpose (a b c d : ℝ) :
    (magneticFrameChange a b c d)ᵀ = magneticFrameChange c d a b := by
  simp [magneticFrameChange]

theorem magneticFrameChange_compose (a b c d e f : ℝ) :
    magneticFrameChange a b c d * magneticFrameChange c d e f = magneticFrameChange a b e f := by
  unfold magneticFrameChange
  rw [Matrix.mul_assoc, ← Matrix.mul_assoc (geographicNedBasis c d)ᵀ,
    geographicNedBasis_transpose_mul, Matrix.one_mul]

theorem magneticFrameChange_transpose_mul (a b c d : ℝ) :
    (magneticFrameChange a b c d)ᵀ * magneticFrameChange a b c d = 1 := by
  rw [magneticFrameChange_transpose, magneticFrameChange_compose, magneticFrameChange_same]

theorem magneticFrameChange_normSquared (a b c d : ℝ) (field : Vector3 ℝ) :
    vectorNormSquared (magneticFrameChange a b c d *ᵥ field) = vectorNormSquared field := by
  unfold vectorNormSquared
  rw [Matrix.dotProduct_mulVec, Matrix.vecMul_mulVec, magneticFrameChange_transpose_mul, Matrix.vecMul_one]

end
end FormalESKF.ESKF
