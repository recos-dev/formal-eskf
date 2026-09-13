/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/

import FormalESKF.Covariance
import FormalESKF.State
import Mathlib.Logic.Equiv.Fin.Basic

/-! Three-axis block algebra. Flattening uses index `3 * block + axis`,
matching INS error/noise offsets in the production fixed-size matrices. -/

namespace FormalESKF.ESKF

open SO3 Matrix

def blockIndex {n : ℕ} (b : Fin n) (i : Fin 3) : Fin (n * 3) :=
  finProdFinEquiv (b, i)

theorem blockIndex_val {n : ℕ} (b : Fin n) (i : Fin 3) :
    (blockIndex b i).val = 3 * b.val + i.val := by
  simp [blockIndex, finProdFinEquiv, Nat.add_comm]

def blocks {n m : ℕ} (A : Fin n → Fin m → Matrix3 ℝ) :
    Matrix (Fin (n * 3)) (Fin (m * 3)) ℝ :=
  fun i j => A (finProdFinEquiv.symm i).1 (finProdFinEquiv.symm j).1
    (finProdFinEquiv.symm i).2 (finProdFinEquiv.symm j).2

@[simp] theorem blocks_apply {n m : ℕ} (A : Fin n → Fin m → Matrix3 ℝ)
    (b : Fin n) (c : Fin m) (i j : Fin 3) :
    blocks A (blockIndex b i) (blockIndex c j) = A b c i j := by
  simp [blocks, blockIndex]

theorem blocks_ext {n m : ℕ} {A B : Matrix (Fin (n * 3)) (Fin (m * 3)) ℝ}
    (h : ∀ b c i j, A (blockIndex b i) (blockIndex c j) =
      B (blockIndex b i) (blockIndex c j)) : A = B := by
  ext i j
  obtain ⟨⟨b, a⟩, rfl⟩ := finProdFinEquiv.surjective i
  obtain ⟨⟨c, d⟩, rfl⟩ := finProdFinEquiv.surjective j
  exact h b c a d

theorem blocks_mul {n m k : ℕ} (A : Fin n → Fin m → Matrix3 ℝ)
    (B : Fin m → Fin k → Matrix3 ℝ) :
    blocks A * blocks B = blocks (fun i j => ∑ b, A i b * B b j) := by
  apply blocks_ext
  intro i j a c
  rw [blocks_apply, Matrix.mul_apply, ← finProdFinEquiv.sum_comp]
  simp only [Fintype.sum_prod_type, Matrix.sum_apply, Matrix.mul_apply]
  apply Finset.sum_congr rfl
  intro b _
  apply Finset.sum_congr rfl
  intro d _
  change blocks A (blockIndex i a) (blockIndex b d) *
    blocks B (blockIndex b d) (blockIndex j c) = _
  rw [blocks_apply, blocks_apply]

theorem blocks_transpose {n m : ℕ} (A : Fin n → Fin m → Matrix3 ℝ) :
    (blocks A)ᵀ = blocks (fun i j => (A j i)ᵀ) := by
  apply blocks_ext
  intro i j a b
  simp [Matrix.transpose_apply]

theorem blocks_add {n m : ℕ} (A B : Fin n → Fin m → Matrix3 ℝ) :
    blocks A + blocks B = blocks (fun i j => A i j + B i j) := by
  apply blocks_ext
  intro i j a b
  simp

theorem blocks_one (n : ℕ) :
    blocks (fun i j : Fin n => if i = j then (1 : Matrix3 ℝ) else 0) = 1 := by
  apply blocks_ext
  intro i j a b
  rw [blocks_apply]
  simp [Matrix.one_apply, blockIndex, Prod.ext_iff]
  split_ifs <;> simp_all [Matrix.one_apply]

theorem blocks_zero (n m : ℕ) : blocks (fun (_ : Fin n) (_ : Fin m) => 0) = 0 := rfl

def matrixBlocks {n m : ℕ} (P : Matrix (Fin (n * 3)) (Fin (m * 3)) ℝ) :
    Fin n → Fin m → Matrix3 ℝ := fun b c i j => P (blockIndex b i) (blockIndex c j)

theorem blocks_matrixBlocks {n m : ℕ} (P : Matrix (Fin (n * 3)) (Fin (m * 3)) ℝ) :
    blocks (matrixBlocks P) = P := by
  apply blocks_ext
  intro b c i j
  simp [matrixBlocks]

def blockDiagonal {n : ℕ} (D : Fin n → Matrix3 ℝ) : Covariance (n * 3) :=
  blocks (fun i j => if i = j then D i else 0)

/-- Both sides of every cross-covariance block are transformed. -/
theorem blockDiagonal_sandwich {n : ℕ} (D : Fin n → Matrix3 ℝ)
    (P : Covariance (n * 3)) :
    sandwich (blockDiagonal D) P =
      blocks (fun i j => D i * matrixBlocks P i j * (D j)ᵀ) := by
  unfold sandwich blockDiagonal
  conv_lhs => arg 1; arg 2; rw [← blocks_matrixBlocks P]
  rw [blocks_transpose, blocks_mul, blocks_mul]
  apply congrArg blocks
  funext i j
  simp [ite_mul, apply_ite]

/-- Error packing and matrix flattening use the same block offsets. -/
theorem packInsError_block (e : InsError) (b : Fin 5) (i : Fin 3) :
    packInsError e (blockIndex b i) =
      (![e.delta_p_n, e.delta_v_n, e.delta_theta_b, e.delta_b_a, e.delta_b_g] b) i := by
  fin_cases b <;> fin_cases i <;> rfl

def blockVector {n : ℕ} (v : Fin n → Vector3 ℝ) : Fin (n * 3) → ℝ :=
  fun i => v (finProdFinEquiv.symm i).1 (finProdFinEquiv.symm i).2

@[simp] theorem blockVector_apply {n : ℕ} (v : Fin n → Vector3 ℝ) (b : Fin n) (i : Fin 3) :
    blockVector v (blockIndex b i) = v b i := by simp [blockVector, blockIndex]

theorem blocks_mulVec {n m : ℕ} (A : Fin n → Fin m → Matrix3 ℝ) (v : Fin m → Vector3 ℝ) :
    blocks A *ᵥ blockVector v = blockVector (fun b => ∑ c, A b c *ᵥ v c) := by
  funext i
  obtain ⟨⟨b, a⟩, rfl⟩ := finProdFinEquiv.surjective i
  change (blocks A *ᵥ blockVector v) (blockIndex b a) =
    blockVector (fun b => ∑ c, A b c *ᵥ v c) (blockIndex b a)
  rw [blockVector_apply, Matrix.mulVec, dotProduct, ← finProdFinEquiv.sum_comp]
  simp only [Fintype.sum_prod_type, Finset.sum_apply, Matrix.mulVec, dotProduct]
  apply Finset.sum_congr rfl
  intro c _
  apply Finset.sum_congr rfl
  intro d _
  change blocks A (blockIndex b a) (blockIndex c d) * blockVector v (blockIndex c d) = _
  rw [blocks_apply, blockVector_apply]

theorem packInsError_eq_blockVector (e : InsError) :
    packInsError e = blockVector ![e.delta_p_n, e.delta_v_n, e.delta_theta_b, e.delta_b_a, e.delta_b_g] := by
  funext i
  obtain ⟨⟨b, a⟩, rfl⟩ := (finProdFinEquiv (m := 5) (n := 3)).surjective i
  exact (packInsError_block e b a).trans (blockVector_apply _ b a).symm

end FormalESKF.ESKF
