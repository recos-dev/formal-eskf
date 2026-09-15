/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.Correction

/-!
# Observation models and local-error Jacobians

Models in `position.hpp`, `velocity.hpp`, `magnetometer.hpp`, `accelerometer.hpp`.
H differentiates h, while residual is y-h. Navigation is NED, including positive
down position. All nonlinear derivatives use right-multiplicative attitude
errors and hold measurements/reference vectors fixed. These theorems do not
establish time alignment, calibration, gravity/magnetic reference validity,
motion gating, or prior/noise independence.
-/

namespace FormalESKF.ESKF

noncomputable section

open SO3 Quaternion Matrix

def selectCoordinates {n m : ℕ} (indices : Fin m → Fin n) : Matrix (Fin m) (Fin n) ℝ :=
  fun i j => if j = indices i then 1 else 0

theorem selectCoordinates_mulVec {n m : ℕ} (indices : Fin m → Fin n) (v : Fin n → ℝ) :
    selectCoordinates indices *ᵥ v = fun i => v (indices i) := by
  funext i
  simp [selectCoordinates, Matrix.mulVec, dotProduct]

def horizontalPosition (s : InsState) : Fin 2 → ℝ := ![s.p_n 0, s.p_n 1]
def verticalPosition (s : InsState) : Fin 1 → ℝ := ![s.p_n 2]
def horizontalVelocity (s : InsState) : Fin 2 → ℝ := ![s.v_n 0, s.v_n 1]
def navigationVelocity (s : InsState) : Vector3 ℝ := s.v_n

def horizontalPositionJacobian : Matrix (Fin 2) (Fin 15) ℝ := selectCoordinates ![0, 1]
def verticalPositionJacobian : Matrix (Fin 1) (Fin 15) ℝ := selectCoordinates ![2]
def horizontalVelocityJacobian : Matrix (Fin 2) (Fin 15) ℝ := selectCoordinates ![3, 4]
def navigationVelocityJacobian : Matrix (Fin 3) (Fin 15) ℝ := selectCoordinates ![3, 4, 5]

/-- These affine models satisfy the linearization exactly, not just at zero. -/
theorem horizontalPosition_injection (s : InsState) (e : InsError) :
    horizontalPosition (injectIns s e) - horizontalPosition s =
      horizontalPositionJacobian *ᵥ packInsError e := by
  rw [horizontalPositionJacobian, selectCoordinates_mulVec]
  funext i
  fin_cases i <;> simp [horizontalPosition, injectIns, packInsError]

theorem verticalPosition_injection (s : InsState) (e : InsError) :
    verticalPosition (injectIns s e) - verticalPosition s =
      verticalPositionJacobian *ᵥ packInsError e := by
  rw [verticalPositionJacobian, selectCoordinates_mulVec]
  funext i
  fin_cases i
  simp [verticalPosition, injectIns, packInsError]

theorem horizontalVelocity_injection (s : InsState) (e : InsError) :
    horizontalVelocity (injectIns s e) - horizontalVelocity s =
      horizontalVelocityJacobian *ᵥ packInsError e := by
  rw [horizontalVelocityJacobian, selectCoordinates_mulVec]
  funext i
  fin_cases i <;> simp [horizontalVelocity, injectIns, packInsError]

theorem navigationVelocity_injection (s : InsState) (e : InsError) :
    navigationVelocity (injectIns s e) - navigationVelocity s =
      navigationVelocityJacobian *ᵥ packInsError e := by
  rw [navigationVelocityJacobian, selectCoordinates_mulVec]
  funext i
  fin_cases i <;> simp [navigationVelocity, injectIns, packInsError]

def residual {m : ℕ} (measurement prediction : Fin m → ℝ) := measurement - prediction

theorem residual_change {m : ℕ} (y h delta : Fin m → ℝ) :
    residual y (h + delta) = residual y h - delta := by
  simp [residual, sub_add_eq_sub_sub]

def insErrorScale (t : ℝ) (e : InsError) : InsError :=
  ⟨vectorScale t e.delta_p_n, vectorScale t e.delta_v_n,
    vectorScale t e.delta_theta_b, vectorScale t e.delta_b_a, vectorScale t e.delta_b_g⟩

def insObservationJacobian (A : Fin 5 → Matrix3 ℝ) : Matrix (Fin 3) (Fin 15) ℝ :=
  fun i j => A ((finProdFinEquiv (m := 5) (n := 3)).symm j).1 i
    ((finProdFinEquiv (m := 5) (n := 3)).symm j).2

theorem insObservationJacobian_mulVec (A : Fin 5 → Matrix3 ℝ) (e : InsError) :
    insObservationJacobian A *ᵥ packInsError e =
      A 0 *ᵥ e.delta_p_n + A 1 *ᵥ e.delta_v_n + A 2 *ᵥ e.delta_theta_b +
        A 3 *ᵥ e.delta_b_a + A 4 *ᵥ e.delta_b_g := by
  funext i
  change (∑ j : Fin 15, insObservationJacobian A i j * packInsError e j) = _
  rw [← (finProdFinEquiv (m := 5) (n := 3)).sum_comp]
  simp only [Fintype.sum_prod_type]
  have hb (b : Fin 5) (j : Fin 3) :
      insObservationJacobian A i (blockIndex b j) = A b i j := by
    simp [insObservationJacobian, blockIndex]
  simp only [← blockIndex.eq_def, hb, packInsError_block]
  simp [Matrix.mulVec, dotProduct, Fin.sum_univ_succ]
  ring

theorem inverseRotate_localAttitude (q : Quaternion ℝ) (d v : Vector3 ℝ) :
    inverseRotate (localAttitude q d) v = inverseRotate (quaternionExp d) (inverseRotate q v) := by
  simp [inverseRotate, localAttitude, rotationMatrix_hamiltonMul, transpose_mul, Matrix.mulVec_mulVec]

theorem inverseRotate_local_hasDerivAt {v : ℝ → Vector3 ℝ} {dv : Vector3 ℝ}
    (hv : HasVectorDerivAt v dv 0) (q : Quaternion ℝ) (d : Vector3 ℝ) :
    HasVectorDerivAt (fun t => inverseRotate (localAttitude q (vectorScale t d)) (v t))
      (hat (inverseRotate q (v 0)) *ᵥ d + inverseRotate q dv) 0 := by
  simpa only [inverseRotate, localAttitude, rotationMatrix_hamiltonMul, transpose_mul,
    Matrix.mulVec_mulVec] using!
    inverseRotate_exp_hasDerivAt (hv.const_matrix (rotationMatrix q)ᵀ) d

def magnetometerModel (q : Quaternion ℝ) (m_n : Vector3 ℝ) : Vector3 ℝ := inverseRotate q m_n
def ahrsMagnetometerJacobian (s : AhrsState) (m_n : Vector3 ℝ) : Matrix3 ℝ :=
  hat (magnetometerModel s.q_nb m_n)
def insMagnetometerJacobian (s : InsState) (m_n : Vector3 ℝ) : Matrix (Fin 3) (Fin 15) ℝ :=
  insObservationJacobian ![0, 0, hat (magnetometerModel s.q_nb m_n), 0, 0]

theorem magnetometerModel_hasDerivAt (q : Quaternion ℝ) (m_n d : Vector3 ℝ) :
    HasVectorDerivAt (fun t => magnetometerModel (localAttitude q (vectorScale t d)) m_n)
      (hat (magnetometerModel q m_n) *ᵥ d) 0 := by
  have h := inverseRotate_local_hasDerivAt
    (v := fun _ => m_n) (dv := 0) (fun i => hasDerivAt_const 0 (m_n i)) q d
  simpa [inverseRotate, magnetometerModel] using! h

theorem insMagnetometerJacobian_hasDerivAt (s : InsState) (m_n : Vector3 ℝ) (e : InsError) :
    HasVectorDerivAt (fun t => magnetometerModel (injectIns s (insErrorScale t e)).q_nb m_n)
      (insMagnetometerJacobian s m_n *ᵥ packInsError e) 0 := by
  simpa [insMagnetometerJacobian, insObservationJacobian_mulVec, injectIns, insErrorScale] using
    magnetometerModel_hasDerivAt s.q_nb m_n e.delta_theta_b

theorem magnetometer_single_vector_null_direction (q : Quaternion ℝ) (m_n : Vector3 ℝ) :
    hat (magnetometerModel q m_n) *ᵥ magnetometerModel q m_n = 0 := hat_self _

def ahrsAccelerometerModel (q : Quaternion ℝ) (g_n : Vector3 ℝ) : Vector3 ℝ :=
  -inverseRotate q g_n

def ahrsAccelerometerJacobian (s : AhrsState) (g_n : Vector3 ℝ) : Matrix3 ℝ :=
  hat (ahrsAccelerometerModel s.q_nb g_n)

/-- AHRS assumes negligible navigation acceleration, with raw specific force
equal to negative body-frame gravity. -/
theorem ahrsAccelerometerModel_hasDerivAt (q : Quaternion ℝ) (g_n d : Vector3 ℝ) :
    HasVectorDerivAt (fun t => ahrsAccelerometerModel (localAttitude q (vectorScale t d)) g_n)
      (hat (ahrsAccelerometerModel q g_n) *ᵥ d) 0 := by
  have h := (magnetometerModel_hasDerivAt q g_n d).neg
  have hn : hat (-inverseRotate q g_n) = -hat (inverseRotate q g_n) := by
    ext i j
    fin_cases i <;> fin_cases j <;> simp [hat]
  simpa [ahrsAccelerometerModel, magnetometerModel, hn, Matrix.neg_mulVec] using! h

def insAccelerometerModel (s : InsState) (angular_rate_b gravity_n : Vector3 ℝ) : Vector3 ℝ :=
  cross (angular_rate_b - s.b_g) (inverseRotate s.q_nb s.v_n) -
    inverseRotate s.q_nb gravity_n + s.b_a

def accelerometerJacobianBlocks (Rt : Matrix3 ℝ) (v_b omega_b g_b : Vector3 ℝ) : Fin 5 → Matrix3 ℝ :=
  ![0, hat omega_b * Rt, hat omega_b * hat v_b - hat g_b, 1, hat v_b]

def insAccelerometerJacobian (s : InsState) (w g : Vector3 ℝ) : Matrix (Fin 3) (Fin 15) ℝ :=
  insObservationJacobian (accelerometerJacobianBlocks (rotationMatrix s.q_nb)ᵀ
    (inverseRotate s.q_nb s.v_n) (w - s.b_g) (inverseRotate s.q_nb g))

theorem accelerometerJacobian_mulVec (Rt : Matrix3 ℝ) (v_b omega_b g_b : Vector3 ℝ) (e : InsError) :
    insObservationJacobian (accelerometerJacobianBlocks Rt v_b omega_b g_b) *ᵥ packInsError e =
      cross (-e.delta_b_g) v_b +
        cross omega_b (hat v_b *ᵥ e.delta_theta_b + Rt *ᵥ e.delta_v_n) -
        hat g_b *ᵥ e.delta_theta_b + e.delta_b_a := by
  rw [insObservationJacobian_mulVec]
  simp [accelerometerJacobianBlocks, Matrix.sub_mulVec, ← Matrix.mulVec_mulVec, hat_mulVec]
  funext i
  fin_cases i <;> simp [cross, vecHead, vecTail] <;> ring

/-- INS uses dot(v_b)=0, not a_n=0. The gyro-bias block is +hat(v_b). -/
theorem insAccelerometerJacobian_hasDerivAt (s : InsState) (w g : Vector3 ℝ) (e : InsError) :
    HasVectorDerivAt (fun t => insAccelerometerModel (injectIns s (insErrorScale t e)) w g)
      (insAccelerometerJacobian s w g *ᵥ packInsError e) 0 := by
  have hv := inverseRotate_local_hasDerivAt (vectorLine_hasDerivAt s.v_n e.delta_v_n 0)
    s.q_nb e.delta_theta_b
  have hg := inverseRotate_local_hasDerivAt
    (v := fun _ => g) (dv := 0) (fun i => hasDerivAt_const 0 (g i)) s.q_nb e.delta_theta_b
  have hw : HasVectorDerivAt (fun t => w - (s.b_g + vectorScale t e.delta_b_g)) (-e.delta_b_g) 0 :=
    fun i => ((hasDerivAt_const 0 (w i)).sub (vectorLine_hasDerivAt s.b_g e.delta_b_g 0 i)).congr_deriv
      (by simp)
  have hb := vectorLine_hasDerivAt s.b_a e.delta_b_a 0
  have h := ((hw.cross hv).sub hg).add hb
  have hz : ∀ v : Vector3 ℝ, vectorScale 0 v = 0 := by intro v; ext i; simp [vectorScale]
  simpa [insAccelerometerModel, injectIns, insErrorScale, insAccelerometerJacobian,
    accelerometerJacobian_mulVec, hz, localAttitude_zero, inverseRotate] using! h

/-- Matrix action in the finite-dimensional local error chart. -/
def observationLinearMap {n m : ℕ} (H : Matrix (Fin m) (Fin n) ℝ) :
    (Fin n → ℝ) →L[ℝ] (Fin m → ℝ) := (Matrix.toLin' H).toContinuousLinearMap

private theorem inverseRotate_local_differentiableAt
    {E : Type*} [NormedAddCommGroup E] [NormedSpace ℝ E]
    {d v : E → Vector3 ℝ} {x : E} (q : Quaternion ℝ)
    (hd : DifferentiableAt ℝ d x) (hv : DifferentiableAt ℝ v x) :
    DifferentiableAt ℝ (fun e => inverseRotate (localAttitude q (d e)) (v e)) x := by
  have h0 := (quaternionExp_scalar_differentiableAt (d x)).comp x hd
  have hh := (quaternionExp_vector_differentiableAt (d x)).comp x hd
  have h1 : DifferentiableAt ℝ (fun e => (quaternionExp (d e)).q1) x := by
    simpa [vectorPart] using! differentiableAt_pi.mp hh 0
  have h2 : DifferentiableAt ℝ (fun e => (quaternionExp (d e)).q2) x := by
    simpa [vectorPart] using! differentiableAt_pi.mp hh 1
  have h3 : DifferentiableAt ℝ (fun e => (quaternionExp (d e)).q3) x := by
    simpa [vectorPart] using! differentiableAt_pi.mp hh 2
  have hv0 := differentiableAt_pi.mp hv 0
  have hv1 := differentiableAt_pi.mp hv 1
  have hv2 := differentiableAt_pi.mp hv 2
  apply differentiableAt_pi.mpr
  intro i
  fin_cases i <;>
    simp [inverseRotate, rotationMatrix, Matrix.mulVec,
      dotProduct, Fin.sum_univ_succ] <;> fun_prop

-- Directional derivatives identify a Frechet derivative only AFTER joint
-- differentiability has been established. Do not infer it from rays alone.
private theorem observation_hasFDerivAt {n m : ℕ}
    {f : (Fin n → ℝ) → (Fin m → ℝ)} {H : Matrix (Fin m) (Fin n) ℝ}
    (hd : DifferentiableAt ℝ f 0)
    (hr : ∀ v i, HasDerivAt (fun t : ℝ => f (t • v) i) ((H *ᵥ v) i) 0) :
    HasFDerivAt f (observationLinearMap H) 0 := by
  apply hd.hasFDerivAt.congr_fderiv
  ext v i
  have h := (hasFDerivAt_pi'.mp hd.hasFDerivAt i).hasLineDerivAt v
  have hh : HasDerivAt (fun t : ℝ => f (t • v) i) ((fderiv ℝ f 0) v i) 0 := by
    simpa [HasLineDerivAt] using! h
  exact hh.unique (hr v i)

theorem ahrsMagnetometerJacobian_hasFDerivAt (s : AhrsState) (m_n : Vector3 ℝ) :
    HasFDerivAt (fun e => magnetometerModel (localAttitude s.q_nb e) m_n)
      (observationLinearMap (ahrsMagnetometerJacobian s m_n)) 0 := by
  apply observation_hasFDerivAt
  · exact inverseRotate_local_differentiableAt s.q_nb differentiableAt_id (differentiableAt_const m_n)
  · intro v i
    simpa [ahrsMagnetometerJacobian, vectorScale] using! magnetometerModel_hasDerivAt s.q_nb m_n v i

theorem ahrsAccelerometerJacobian_hasFDerivAt (s : AhrsState) (g_n : Vector3 ℝ) :
    HasFDerivAt (fun e => ahrsAccelerometerModel (localAttitude s.q_nb e) g_n)
      (observationLinearMap (ahrsAccelerometerJacobian s g_n)) 0 := by
  apply observation_hasFDerivAt
  · exact (inverseRotate_local_differentiableAt s.q_nb differentiableAt_id (differentiableAt_const g_n)).neg
  · intro v i
    simpa [ahrsAccelerometerJacobian, vectorScale] using! ahrsAccelerometerModel_hasDerivAt s.q_nb g_n v i

private theorem unpackInsError_scale (t : ℝ) (e : Fin 15 → ℝ) :
    unpackInsError (t • e) = insErrorScale t (unpackInsError e) := by
  ext i <;> fin_cases i <;> simp [unpackInsError, insErrorScale, vectorScale]

theorem insMagnetometerJacobian_hasFDerivAt (s : InsState) (m_n : Vector3 ℝ) :
    HasFDerivAt (fun e => magnetometerModel (injectIns s (unpackInsError e)).q_nb m_n)
      (observationLinearMap (insMagnetometerJacobian s m_n)) 0 := by
  apply observation_hasFDerivAt
  · apply inverseRotate_local_differentiableAt s.q_nb _ (differentiableAt_const m_n)
    apply differentiableAt_pi.mpr
    intro i
    fin_cases i <;> simp [unpackInsError] <;> fun_prop
  · intro v i
    simpa [unpackInsError_scale, pack_unpack_insError] using!
      insMagnetometerJacobian_hasDerivAt s m_n (unpackInsError v) i

theorem insAccelerometerJacobian_hasFDerivAt (s : InsState) (w g : Vector3 ℝ) :
    HasFDerivAt (fun e => insAccelerometerModel (injectIns s (unpackInsError e)) w g)
      (observationLinearMap (insAccelerometerJacobian s w g)) 0 := by
  apply observation_hasFDerivAt
  · have hd : DifferentiableAt ℝ (fun e => (unpackInsError e).delta_theta_b) 0 := by
      apply differentiableAt_pi.mpr
      intro i
      fin_cases i <;> simp [unpackInsError] <;> fun_prop
    have hv : DifferentiableAt ℝ (fun e => (injectIns s (unpackInsError e)).v_n) 0 := by
      apply differentiableAt_pi.mpr
      intro i
      fin_cases i <;> simp [injectIns, unpackInsError] <;> fun_prop
    have hbody := inverseRotate_local_differentiableAt s.q_nb hd hv
    have hgravity := inverseRotate_local_differentiableAt s.q_nb hd
      (differentiableAt_const (c := g))
    have hw : DifferentiableAt ℝ (fun e => w - (injectIns s (unpackInsError e)).b_g) 0 := by
      apply differentiableAt_pi.mpr
      intro i
      fin_cases i <;> simp [injectIns, unpackInsError] <;> fun_prop
    have hb : DifferentiableAt ℝ (fun e => (injectIns s (unpackInsError e)).b_a) 0 := by
      apply differentiableAt_pi.mpr
      intro i
      fin_cases i <;> simp [injectIns, unpackInsError] <;> fun_prop
    have hc : DifferentiableAt ℝ (fun e =>
        cross (w - (injectIns s (unpackInsError e)).b_g)
          (inverseRotate (localAttitude s.q_nb (unpackInsError e).delta_theta_b)
            (injectIns s (unpackInsError e)).v_n)) 0 := by
      have hv0 := differentiableAt_pi.mp hbody 0
      have hv1 := differentiableAt_pi.mp hbody 1
      have hv2 := differentiableAt_pi.mp hbody 2
      have hw0 := differentiableAt_pi.mp hw 0
      have hw1 := differentiableAt_pi.mp hw 1
      have hw2 := differentiableAt_pi.mp hw 2
      apply differentiableAt_pi.mpr
      intro i
      fin_cases i
      · exact (hw1.mul hv2).sub (hw2.mul hv1)
      · exact (hw2.mul hv0).sub (hw0.mul hv2)
      · exact (hw0.mul hv1).sub (hw1.mul hv0)
    exact (hc.sub hgravity).add hb
  · intro v i
    simpa [unpackInsError_scale, pack_unpack_insError] using!
      insAccelerometerJacobian_hasDerivAt s w g (unpackInsError v) i

end

end FormalESKF.ESKF
