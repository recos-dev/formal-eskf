/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this
file.

SPDX-License-Identifier: BSD-3-Clause
-/
import Mathlib.Data.List.Basic
import Mathlib.Data.List.Range
import Mathlib.Tactic

/-! Source-compositional ring-loop model. This does not translate C++ or
establish floating-point copy semantics. Actual typed step contracts and a
reviewed source-loop correspondence are separate dependencies. -/
namespace FormalESKF.Runtime.Ring

structure SearchResult (α : Type) where
  candidate : Option α
  removed : ℕ
  deriving DecidableEq

/-- The input list is in search order, newest to oldest. A match stops even
when too old to publish: freshness is a separate decision. -/
def search {α : Type} (time : α → ℕ) (horizon : ℕ) : List α → SearchResult α
  | [] => ⟨none, 0⟩
  | a :: rest => if time a ≤ horizon then ⟨some a, rest.length + 1⟩
                 else search time horizon rest

theorem search_none_iff {α : Type} (time : α → ℕ) (horizon : ℕ) (xs : List α) :
    (search time horizon xs).candidate = none ↔ ∀ a ∈ xs, horizon < time a := by
  induction xs with
  | nil => simp [search]
  | cons a rest ih =>
    by_cases h : time a ≤ horizon
    · simp [search, h, Nat.not_lt.mpr h]
    · simp [search, h, ih, Nat.lt_of_not_ge h]

theorem search_none_removed {α : Type} (time : α → ℕ) (horizon : ℕ) (xs : List α)
    (h : (search time horizon xs).candidate = none) :
    (search time horizon xs).removed = 0 := by
  induction xs with
  | nil => rfl
  | cons a rest ih =>
    by_cases ha : time a ≤ horizon
    · simp [search, ha] at h
    · simpa [search, ha] using ih (by simpa [search, ha] using h)

theorem search_match {α : Type} (time : α → ℕ) (horizon : ℕ) (xs : List α) (a : α)
    (h : (search time horizon xs).candidate = some a) :
    ∃ skipped rest, xs = skipped ++ a :: rest ∧
      (∀ b ∈ skipped, horizon < time b) ∧ time a ≤ horizon ∧
      (search time horizon xs).removed = rest.length + 1 := by
  induction xs with
  | nil => simp [search] at h
  | cons b tail ih =>
    by_cases hb : time b ≤ horizon
    · have hba : b = a := by simpa [search, hb] using h
      subst b
      exact ⟨[], tail, rfl, by simp, hb, by simp [search, hb]⟩
    · obtain ⟨skipped, rest, hs, hf, ht, hr⟩ := ih (by simpa [search, hb] using h)
      exact ⟨b :: skipped, rest, by simp [hs], by simpa using And.intro (Nat.lt_of_not_ge hb) hf,
        ht, by simpa [search, hb] using hr⟩

theorem search_removed_le {α : Type} (time : α → ℕ) (horizon : ℕ) (xs : List α) :
    (search time horizon xs).removed ≤ xs.length := by
  induction xs with
  | nil => simp [search]
  | cons a rest ih =>
    by_cases h : time a ≤ horizon
    · simp [search, h]
    · simp only [search, h, ↓reduceIte, List.length_cons]; omega

/-- Publication has the same stop candidate as search, but may reject its age. -/
def publish {α : Type} (time : α → ℕ) (horizon age : ℕ) (old : α) (candidate : Option α) : Bool × α :=
  match candidate with
  | none => (false, old)
  | some a => if horizon - time a < age then (true, a) else (false, old)

theorem publish_failure_frame {α : Type} (time : α → ℕ) (horizon age : ℕ) (old : α)
    (candidate : Option α) (h : (publish time horizon age old candidate).1 = false) :
    (publish time horizon age old candidate).2 = old := by
  cases candidate with
  | none => rfl
  | some a =>
    simp only [publish] at *
    split_ifs at *; simp_all

theorem expired_match_retires {α : Type} (time : α → ℕ) (horizon age : ℕ) (old a : α)
    (xs : List α) (hm : (search time horizon xs).candidate = some a)
    (he : age ≤ horizon - time a) :
    publish time horizon age old (search time horizon xs).candidate = (false, old) ∧
      0 < (search time horizon xs).removed := by
  obtain ⟨skipped, rest, _, _, _, hr⟩ := search_match time horizon xs a hm
  constructor
  · simp [hm, publish, Nat.not_lt.mpr he]
  · omega

structure Cell (α : Type) where
  time : ℕ
  payload : α

def clear {α : Type} (storage : ℕ → Cell α) (index : ℕ) : ℕ → Cell α :=
  Function.update storage index ⟨0, (storage index).payload⟩

def retire {α : Type} (storage : ℕ → Cell α) : List ℕ → ℕ → Cell α
  | [] => storage
  | i :: rest => retire (clear storage i) rest

theorem retire_payload {α : Type} (indices : List ℕ) (storage : ℕ → Cell α) (i : ℕ) :
    (retire storage indices i).payload = (storage i).payload := by
  induction indices generalizing storage with
  | nil => rfl
  | cons j rest ih =>
    rw [retire, ih]
    by_cases h : i = j <;> simp [clear, Function.update, h]

theorem retire_time (indices : List ℕ) {α : Type} (storage : ℕ → Cell α) (i : ℕ) :
    (retire storage indices i).time = if i ∈ indices then 0 else (storage i).time := by
  induction indices generalizing storage with
  | nil => simp [retire]
  | cons j rest ih =>
    rw [retire, ih]
    by_cases h : i = j <;> by_cases hr : i ∈ rest <;> simp [clear, Function.update, h, hr]

def next (capacity index : ℕ) : ℕ := if index + 1 = capacity then 0 else index + 1

def previous (capacity index : ℕ) : ℕ := if index = 0 then capacity - 1 else index - 1

theorem next_lt {capacity index : ℕ} (hc : 0 < capacity) (hi : index < capacity) :
    next capacity index < capacity := by
  unfold next; split_ifs <;> omega

theorem previous_lt {capacity index : ℕ} (hc : 0 < capacity) (hi : index < capacity) :
    previous capacity index < capacity := by
  unfold previous; split_ifs <;> omega

theorem next_previous {capacity index : ℕ} (hc : 0 < capacity) (hi : index < capacity) :
    next capacity (previous capacity index) = index := by
  unfold next previous; split_ifs <;> omega

theorem previous_next {capacity index : ℕ} (hc : 0 < capacity) :
    previous capacity (next capacity index) = index := by
  by_cases h : index + 1 = capacity
  · simp only [next, h, ↓reduceIte, previous]; omega
  · simp [next, h, previous]


/-- Cursor order follows the actual previous(index) operation. -/
def backward {α : Type} (read : ℕ → α) (capacity cursor : ℕ) : ℕ → List α
  | 0 => []
  | n + 1 => read (previous capacity cursor) :: backward read capacity (previous capacity cursor) n

/-- remaining = initial count - inspected. Stop omits the for-increment. -/
def searchLoop {α : Type} (read : ℕ → α) (time : α → ℕ) (horizon capacity cursor : ℕ) :
    ℕ → SearchResult α
  | 0 => ⟨none, 0⟩
  | n + 1 =>
    let index := previous capacity cursor
    if time (read index) ≤ horizon then ⟨some (read index), n + 1⟩
    else searchLoop read time horizon capacity index n

theorem backward_length {α : Type} (read : ℕ → α) (capacity cursor n : ℕ) :
    (backward read capacity cursor n).length = n := by
  induction n generalizing cursor with
  | zero => rfl
  | succ n ih => simp [backward, ih]

theorem searchLoop_eq {α : Type} (read : ℕ → α) (time : α → ℕ) (horizon capacity cursor n : ℕ) :
    searchLoop read time horizon capacity cursor n = search time horizon (backward read capacity cursor n) := by
  induction n generalizing cursor with
  | zero => rfl
  | succ n ih => simp [searchLoop, backward, search, backward_length, ih]

def forward (capacity oldest : ℕ) : ℕ → List ℕ
  | 0 => []
  | n + 1 => oldest :: forward capacity (next capacity oldest) n

def retireLoop {α : Type} (capacity : ℕ) (storage : ℕ → Cell α) (oldest : ℕ) :
    ℕ → (ℕ → Cell α) × ℕ
  | 0 => (storage, oldest)
  | n + 1 => retireLoop capacity (clear storage oldest) (next capacity oldest) n

theorem retireLoop_storage {α : Type} (capacity n : ℕ) (storage : ℕ → Cell α) (oldest : ℕ) :
    (retireLoop capacity storage oldest n).1 = retire storage (forward capacity oldest n) := by
  induction n generalizing storage oldest with
  | zero => rfl
  | succ n ih => simp [retireLoop, forward, retire, ih]

theorem next_mod {capacity index : ℕ} (hi : index < capacity) :
    next capacity index = (index + 1) % capacity := by
  by_cases h : index + 1 = capacity
  · simp [next, h]
  · have hl : index + 1 < capacity := by omega
    simp [next, h, Nat.mod_eq_of_lt hl]

theorem retireLoop_oldest {α : Type} {capacity oldest : ℕ} (hc : 0 < capacity)
    (ho : oldest < capacity) (n : ℕ) (storage : ℕ → Cell α) :
    (retireLoop capacity storage oldest n).2 = (oldest + n) % capacity := by
  induction n generalizing storage oldest with
  | zero => simpa [retireLoop] using (Nat.mod_eq_of_lt ho).symm
  | succ n ih =>
    simp only [retireLoop]
    rw [ih (next_lt hc ho), next_mod ho, Nat.mod_add_mod]
    congr 1
    omega

theorem retireLoop_frame {α : Type} (capacity n oldest i : ℕ) (storage : ℕ → Cell α) :
    ((retireLoop capacity storage oldest n).1 i).payload = (storage i).payload ∧
    ((retireLoop capacity storage oldest n).1 i).time =
      if i ∈ forward capacity oldest n then 0 else (storage i).time := by
  rw [retireLoop_storage]
  exact ⟨retire_payload _ _ _, retire_time _ _ _⟩

/-- Restoring count happens only after all selected tags have been retired. -/
theorem retirement_restores_metadata {capacity oldest count removed : ℕ}
    (hr : removed ≤ count) :
    (((oldest + removed) % capacity) + (count - removed)) % capacity =
      (oldest + count) % capacity := by
  rw [Nat.mod_add_mod]
  congr 1
  omega

theorem bounded_counters {capacity count inspected : ℕ} (hc : capacity ≤ 128)
    (hn : count ≤ capacity) (hi : inspected < count) :
    0 < count - inspected ∧ count - inspected ≤ capacity ∧ inspected + 1 ≤ 128 := by
  omega


def searchCursor (capacity write : ℕ) : ℕ → ℕ
  | 0 => write
  | n + 1 => previous capacity (searchCursor capacity write n)

theorem searchCursor_closed {capacity write n : ℕ} (hw : write < capacity) (hn : n ≤ capacity) :
    searchCursor capacity write n = if n ≤ write then write - n else capacity + write - n := by
  induction n with
  | zero => simp [searchCursor]
  | succ n ih =>
    have hprev : n ≤ capacity := by omega
    rw [searchCursor, ih hprev]
    unfold previous
    split_ifs <;> omega

theorem searchCursor_mod {capacity write n : ℕ} (hw : write < capacity) (hn : n ≤ capacity) :
    searchCursor capacity write n = (write + capacity - n) % capacity := by
  rw [searchCursor_closed hw hn]
  by_cases h : n ≤ write
  · simp only [h, ↓reduceIte]
    have he : write + capacity - n = (write - n) + capacity := by omega
    have hl : write - n < capacity := by omega
    rw [he, Nat.add_mod_right, Nat.mod_eq_of_lt hl]
  · simp only [h, ↓reduceIte]
    have hl : write + capacity - n < capacity := by omega
    rw [Nat.mod_eq_of_lt hl]
    omega

theorem searchCursor_bound {capacity write n : ℕ} (hw : write < capacity) (hn : n ≤ capacity) :
    searchCursor capacity write n < capacity := by
  rw [searchCursor_mod hw hn]
  exact Nat.mod_lt _ (by omega)

/-- The next iteration is exactly the next recursive cursor. This equation is
used only on a continuation, never on the source break edge. -/
theorem searchCursor_increment (capacity write inspected : ℕ) :
    previous capacity (searchCursor capacity write inspected) =
      searchCursor capacity write (inspected + 1) := rfl


def advance (capacity oldest : ℕ) : ℕ → ℕ
  | 0 => oldest
  | n + 1 => next capacity (advance capacity oldest n)

theorem advance_lt {capacity oldest : ℕ} (hc : 0 < capacity) (ho : oldest < capacity) (n : ℕ) :
    advance capacity oldest n < capacity := by
  induction n with
  | zero => exact ho
  | succ n ih => exact next_lt hc ih

theorem advance_next (capacity oldest n : ℕ) :
    advance capacity (next capacity oldest) n = advance capacity oldest (n + 1) := by
  induction n with
  | zero => rfl
  | succ n ih => simp [advance, ih]

theorem advance_mod {capacity oldest : ℕ} (hc : 0 < capacity) (ho : oldest < capacity) (n : ℕ) :
    advance capacity oldest n = (oldest + n) % capacity := by
  induction n with
  | zero => simpa [advance] using (Nat.mod_eq_of_lt ho).symm
  | succ n ih =>
    rw [advance, next_mod (advance_lt hc ho n), ih, Nat.mod_add_mod]
    congr 1

theorem forward_snoc (capacity oldest n : ℕ) :
    forward capacity oldest (n + 1) = forward capacity oldest n ++ [advance capacity oldest n] := by
  induction n generalizing oldest with
  | zero => rfl
  | succ n ih =>
    change oldest :: forward capacity (next capacity oldest) (n + 1) =
      (oldest :: forward capacity (next capacity oldest) n) ++ [advance capacity oldest (n + 1)]
    rw [ih]
    simp only [List.cons_append, advance_next]

theorem backward_forward {α : Type} (read : ℕ → α) {capacity oldest : ℕ}
    (hc : 0 < capacity) (n : ℕ) :
    backward read capacity (advance capacity oldest n) n =
      ((forward capacity oldest n).reverse.map read) := by
  induction n with
  | zero => rfl
  | succ n ih =>
    simp only [advance, backward, previous_next hc]
    rw [ih, forward_snoc]
    simp

theorem logical_search_order {α : Type} (read : ℕ → α) {capacity oldest count write : ℕ}
    (hc : 0 < capacity) (ho : oldest < capacity) (hw : write = (oldest + count) % capacity) :
    backward read capacity write count = ((forward capacity oldest count).reverse.map read) := by
  rw [hw, ← advance_mod hc ho]
  exact backward_forward read hc count


/-- Logical offset from oldest, for a physical slot in the allocated ring. -/
def rank (capacity oldest index : ℕ) : ℕ :=
  if oldest ≤ index then index - oldest else capacity - oldest + index

theorem rank_lt {capacity oldest index : ℕ} (ho : oldest < capacity) (hi : index < capacity) :
    rank capacity oldest index < capacity := by
  unfold rank; split_ifs <;> omega

theorem slot_at_rank {capacity oldest index : ℕ} (ho : oldest < capacity) (hi : index < capacity) :
    (oldest + rank capacity oldest index) % capacity = index := by
  unfold rank
  split_ifs with h
  · rw [Nat.add_sub_of_le h, Nat.mod_eq_of_lt hi]
  · have he : oldest + (capacity - oldest + index) = capacity + index := by omega
    rw [he, Nat.add_mod_left, Nat.mod_eq_of_lt hi]

theorem slot_rank_unique {capacity oldest index offset : ℕ} (ho : oldest < capacity)
    (hi : index < capacity) (hj : offset < capacity)
    (he : (oldest + offset) % capacity = index) : offset = rank capacity oldest index := by
  by_cases hs : oldest + offset < capacity
  · rw [Nat.mod_eq_of_lt hs] at he
    unfold rank; split_ifs <;> omega
  · have hm : oldest + offset - capacity < capacity := by omega
    rw [Nat.mod_eq_sub_mod (by omega), Nat.mod_eq_of_lt hm] at he
    unfold rank; split_ifs <;> omega

theorem forward_range {capacity oldest : ℕ} (hc : 0 < capacity) (ho : oldest < capacity) (n : ℕ) :
    forward capacity oldest n = (List.range n).map (fun j => (oldest + j) % capacity) := by
  induction n with
  | zero => rfl
  | succ n ih =>
    rw [forward_snoc, ih, List.range_succ, List.map_append]
    simp [advance_mod hc ho]

theorem forward_mem_rank {capacity oldest index n : ℕ} (ho : oldest < capacity) (hi : index < capacity)
    (hn : n ≤ capacity) : index ∈ forward capacity oldest n ↔ rank capacity oldest index < n := by
  rw [forward_range (by omega) ho, List.mem_map]
  constructor
  · rintro ⟨j, hj, he⟩
    have hjn : j < n := by simpa using hj
    have hjc : j < capacity := by omega
    rw [← slot_rank_unique ho hi hjc he]
    exact hjn
  · intro hr
    exact ⟨rank capacity oldest index, by simpa using hr, slot_at_rank ho hi⟩


/-- Search by physical slot, with the remaining count also identifying its rank.
This permits repeated payload values without confusing two distinct slots. -/
def searchPhysical (time : ℕ → ℕ) (horizon capacity oldest : ℕ) : ℕ → SearchResult ℕ
  | 0 => ⟨none, 0⟩
  | n + 1 =>
    let index := (oldest + n) % capacity
    if time index ≤ horizon then ⟨some index, n + 1⟩ else searchPhysical time horizon capacity oldest n

theorem forward_length (capacity oldest n : ℕ) : (forward capacity oldest n).length = n := by
  induction n generalizing oldest with
  | zero => rfl
  | succ n ih => simp [forward, ih]

theorem searchPhysical_eq (time : ℕ → ℕ) (horizon : ℕ) {capacity oldest : ℕ}
    (hc : 0 < capacity) (ho : oldest < capacity) (n : ℕ) :
    searchPhysical time horizon capacity oldest n =
      search time horizon (forward capacity oldest n).reverse := by
  induction n with
  | zero => rfl
  | succ n ih =>
    rw [forward_snoc]
    simp [searchPhysical, search, List.reverse_append, advance_mod hc ho, forward_length, ih]

theorem physical_match (time : ℕ → ℕ) (horizon : ℕ) {capacity oldest count index : ℕ}
    (ho : oldest < capacity) (hi : index < capacity) (hn : count ≤ capacity)
    (hm : (searchPhysical time horizon capacity oldest count).candidate = some index) :
    (searchPhysical time horizon capacity oldest count).removed = rank capacity oldest index + 1 ∧
    rank capacity oldest index < count ∧ time index ≤ horizon ∧
    ∀ j, rank capacity oldest index < j → j < count → horizon < time ((oldest + j) % capacity) := by
  induction count with
  | zero => simp [searchPhysical] at hm
  | succ n ih =>
    by_cases hit : time ((oldest + n) % capacity) ≤ horizon
    · have he : (oldest + n) % capacity = index := by simpa [searchPhysical, hit] using hm
      have hr : rank capacity oldest index = n := (slot_rank_unique ho hi (by omega) he).symm
      refine ⟨by simp [searchPhysical, hit, hr], by omega, by simpa [he] using hit, ?_⟩
      intro j hj hlimit
      omega
    · obtain ⟨hr, hrange, htime, hfuture⟩ := ih (by omega) (by simpa [searchPhysical, hit] using hm)
      refine ⟨by simpa [searchPhysical, hit] using hr, by omega, htime, ?_⟩
      intro j hj hlimit
      by_cases hjn : j < n
      · exact hfuture j hj hjn
      · have he : j = n := by omega
        subst j
        omega

theorem selected_has_retirement {α : Type} (time : α → ℕ) (horizon age : ℕ) (old : α)
    (xs : List α) (hs : (publish time horizon age old (search time horizon xs).candidate).1 = true) :
    0 < (search time horizon xs).removed := by
  cases hm : (search time horizon xs).candidate with
  | none => simp [hm, publish] at hs
  | some a =>
    obtain ⟨skipped, rest, _, _, _, hr⟩ := search_match time horizon xs a hm
    omega

theorem owner_retired_count {count removed : ℕ} (selected : Bool) (hr : removed ≤ count)
    (hs : selected = true → 0 < removed) :
    let published := if selected then 1 else 0
    published ≤ count - (count - removed) ∧
      count - (count - removed) - published = removed - published := by
  cases selected <;> simp_all <;> omega

end FormalESKF.Runtime.Ring

namespace FormalESKF.Runtime.Ring

/-- Physical storage after the single typed assignment. The actual C++ producer
must establish this equation and its complete metadata/input frame separately. -/
def store {α : Type} (read : ℕ → α) (index : ℕ) (value : α) : ℕ → α :=
  fun i => if i = index then value else read i

def logical {α : Type} (read : ℕ → α) (capacity oldest count : ℕ) : List α :=
  (forward capacity oldest count).map read

theorem logical_length {α : Type} (read : ℕ → α) (capacity oldest count : ℕ) :
    (logical read capacity oldest count).length = count := by
  simp [logical, forward_length]

theorem advance_cycle {capacity oldest : ℕ} (hc : 0 < capacity) (ho : oldest < capacity) :
    advance capacity oldest capacity = oldest := by
  rw [advance_mod hc ho, Nat.add_mod_right, Nat.mod_eq_of_lt ho]

theorem append_slot_not_retained {capacity oldest count : ℕ}
    (hc : 0 < capacity) (ho : oldest < capacity) (hn : count < capacity) :
    advance capacity oldest count ∉ forward capacity oldest count := by
  have hi := advance_lt hc ho count
  have hr := slot_rank_unique ho hi hn (advance_mod hc ho count).symm
  rw [forward_mem_rank ho hi (by omega), ← hr]
  omega

theorem logical_store_append {α : Type} (read : ℕ → α) (value : α)
    {capacity oldest count : ℕ} (hc : 0 < capacity) (ho : oldest < capacity)
    (hn : count < capacity) :
    logical (store read (advance capacity oldest count) value) capacity oldest (count + 1) =
      logical read capacity oldest count ++ [value] := by
  unfold logical
  rw [forward_snoc, List.map_append]
  have hframe : (forward capacity oldest count).map (store read (advance capacity oldest count) value) =
      (forward capacity oldest count).map read := by
    apply List.map_congr_left
    intro i hi
    have hne : i ≠ advance capacity oldest count := by
      intro he
      subst i
      exact append_slot_not_retained hc ho hn hi
    simp [store, hne]
  rw [hframe]
  simp [store]

theorem logical_tail {α : Type} (read : ℕ → α) (capacity oldest count : ℕ) :
    (logical read capacity oldest (count + 1)).tail =
      logical read capacity (next capacity oldest) count := by
  simp [logical, forward]

theorem logical_store_full {α : Type} (read : ℕ → α) (value : α)
    {capacity oldest : ℕ} (hc : 0 < capacity) (ho : oldest < capacity) :
    logical (store read oldest value) capacity (next capacity oldest) capacity =
      (logical read capacity oldest capacity).tail ++ [value] := by
  have hn : capacity - 1 < capacity := by omega
  have he : capacity - 1 + 1 = capacity := by omega
  have hw : advance capacity (next capacity oldest) (capacity - 1) = oldest := by
    rw [advance_next, he, advance_cycle hc ho]
  have h := logical_store_append read value hc (next_lt hc ho) hn
  rw [hw, he] at h
  have ht : (logical read capacity oldest capacity).tail =
      logical read capacity (next capacity oldest) (capacity - 1) := by
    simpa only [he] using logical_tail read capacity oldest (capacity - 1)
  rw [ht]
  exact h

/-- The full ring drops exactly its oldest logical cell; no retained field is
reconstructed or copied from a different epoch. -/
theorem logical_store_full_drop {α : Type} (read : ℕ → α) (value : α)
    {capacity oldest : ℕ} (hc : 0 < capacity) (ho : oldest < capacity) :
    logical (store read oldest value) capacity (next capacity oldest) capacity =
      (logical read capacity oldest capacity ++ [value]).drop 1 := by
  rw [logical_store_full read value hc ho]
  have hn : logical read capacity oldest capacity ≠ [] := by
    intro he
    have hl := logical_length read capacity oldest capacity
    rw [he] at hl
    simp at hl
    omega
  cases he : logical read capacity oldest capacity with
  | nil => exact False.elim (hn he)
  | cons a xs => simp

/-- The physical write/oldest/count update implements bounded list append,
including full-capacity overwrite. The input value is already fully evaluated;
source-level non-aliasing and complete typed copying are separate obligations. -/
theorem logical_enqueue {α : Type} (read : ℕ → α) (value : α)
    {capacity oldest count write : ℕ} (hc : 0 < capacity) (ho : oldest < capacity)
    (hn : count ≤ capacity) (hw : write = advance capacity oldest count) :
    logical (store read write value) capacity
        (if count = capacity then next capacity oldest else oldest)
        (if count = capacity then count else count + 1) =
      (logical read capacity oldest count ++ [value]).drop (count + 1 - capacity) := by
  by_cases hfull : count = capacity
  · subst count
    rw [advance_cycle hc ho] at hw
    subst write
    simpa using logical_store_full_drop read value hc ho
  · have hlt : count < capacity := by omega
    have hz : count + 1 - capacity = 0 := by omega
    simp only [hfull, ↓reduceIte, hw, hz, List.drop_zero]
    exact logical_store_append read value hc ho hlt

/-- The source writes a single cell and then advances the write cursor. Full
and nonfull updates preserve the same oldest/count/write relation. -/
theorem push_write_coherent {capacity oldest count write : ℕ}
    (hc : 0 < capacity) (ho : oldest < capacity) (_hn : count ≤ capacity)
    (hw : write = (oldest + count) % capacity) :
    next capacity write =
      ((if count = capacity then next capacity oldest else oldest) +
        (if count = capacity then count else count + 1)) % capacity := by
  have hwb : write < capacity := by rw [hw]; exact Nat.mod_lt _ hc
  by_cases hfull : count = capacity
  · subst count
    have hwo : write = oldest := by simpa [Nat.add_mod_right, Nat.mod_eq_of_lt ho] using hw
    simp [hwo, Nat.add_mod_right, Nat.mod_eq_of_lt (next_lt hc ho)]
  · simp only [hfull, ↓reduceIte]
    rw [next_mod hwb, hw, Nat.mod_add_mod]
    congr 1

theorem forward_append (capacity oldest removed retained : ℕ) :
    forward capacity oldest (removed + retained) =
      forward capacity oldest removed ++ forward capacity (advance capacity oldest removed) retained := by
  induction removed generalizing oldest with
  | zero => simp [forward, advance]
  | succ n ih =>
    simp only [Nat.succ_add, forward, List.cons_append]
    rw [ih, advance_next]

theorem retained_not_retired {capacity oldest count removed i : ℕ}
    (hc : 0 < capacity) (ho : oldest < capacity) (hn : count ≤ capacity) (hr : removed ≤ count)
    (hi : i ∈ forward capacity (advance capacity oldest removed) (count - removed)) :
    i ∉ forward capacity oldest removed := by
  rw [forward_range hc (advance_lt hc ho removed), List.mem_map] at hi
  obtain ⟨offset, hmem, he⟩ := hi
  have hk : offset < count - removed := by simpa using hmem
  have he' : (oldest + (removed + offset)) % capacity = i := by
    rw [advance_mod hc ho, Nat.mod_add_mod] at he
    simpa [Nat.add_assoc] using he
  have hib : i < capacity := by rw [← he']; exact Nat.mod_lt _ hc
  have hoff : removed + offset < capacity := by omega
  have hslot := slot_rank_unique ho hib hoff he'
  rw [forward_mem_rank ho hib (by omega), ← hslot]
  omega

theorem retire_untouched {α : Type} (storage : ℕ → Cell α) (indices : List ℕ) (i : ℕ)
    (hi : i ∉ indices) : retire storage indices i = storage i := by
  induction indices generalizing storage with
  | nil => rfl
  | cons j rest ih =>
    have hne : i ≠ j := by intro he; subst i; exact hi (by simp)
    have hrest : i ∉ rest := by intro hm; exact hi (by simp [hm])
    rw [retire, ih _ hrest]
    simp [clear, Function.update, hne]

/-- Clearing retired tags does not change any retained active cell. This is the
physical frame needed before applying list-level chronology/provenance lemmas. -/
theorem logical_retirement_frame {α : Type} (storage : ℕ → Cell α)
    {capacity oldest count removed : ℕ} (hc : 0 < capacity) (ho : oldest < capacity)
    (hn : count ≤ capacity) (hr : removed ≤ count) :
    logical (retire storage (forward capacity oldest removed))
        capacity (advance capacity oldest removed) (count - removed) =
      logical storage capacity (advance capacity oldest removed) (count - removed) := by
  unfold logical
  apply List.map_congr_left
  intro i hi
  exact retire_untouched storage _ i (retained_not_retired hc ho hn hr hi)

theorem logical_drop {α : Type} (read : ℕ → α) (capacity oldest count removed : ℕ)
    (hr : removed ≤ count) :
    logical read capacity (advance capacity oldest removed) (count - removed) =
      (logical read capacity oldest count).drop removed := by
  have he := forward_append capacity oldest removed (count - removed)
  have hs : removed + (count - removed) = count := by omega
  rw [hs] at he
  have hp : (forward capacity oldest removed).drop removed = [] :=
    List.drop_eq_nil_of_le (by rw [forward_length])
  unfold logical
  rw [← List.map_drop, he, List.drop_append]
  simp [hp, forward_length]

/-- Exact logical suffix after physical tag retirement and metadata update. -/
theorem logical_retirement_drop {α : Type} (storage : ℕ → Cell α)
    {capacity oldest count removed : ℕ} (hc : 0 < capacity) (ho : oldest < capacity)
    (hn : count ≤ capacity) (hr : removed ≤ count) :
    logical (retire storage (forward capacity oldest removed))
        capacity (advance capacity oldest removed) (count - removed) =
      (logical storage capacity oldest count).drop removed := by
  rw [logical_retirement_frame storage hc ho hn hr]
  exact logical_drop storage capacity oldest count removed hr

/-- The post-push newest cursor addresses exactly the completed write. -/
theorem newest_store_after_push {α : Type} (read : ℕ → α) (value : α)
    {capacity write : ℕ} (hc : 0 < capacity) :
    store read write value (previous capacity (next capacity write)) = value := by
  rw [previous_next hc]
  simp [store]

/-- A push into an allocated ring establishes the nonempty-view premise. -/
theorem push_nonempty {capacity count : ℕ} (hc : 0 < capacity) :
    0 < (if count = capacity then count else count + 1) := by
  split_ifs with h <;> omega

end FormalESKF.Runtime.Ring
