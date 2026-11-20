/- SPDX-License-Identifier: BSD-3-Clause -/

import FormalESKF.ObservationHistory
import FormalESKF.Quaternion

/-! Output-owner temporal and stage/commit relations. IEEE source steps and the
reviewed loop correspondence are separate obligations, not C++ refinement. -/
namespace FormalESKF.Runtime.Output

/-- Each physical slot receives its own arbitrary candidate and status. The
source reads retained history, never an earlier scratch candidate. -/
def stage {α : Type} (candidate : ℕ → Bool × α) (work : ℕ → α) :
    List ℕ → Bool × (ℕ → α)
  | [] => (true, work)
  | i :: rest =>
    let next := Ring.store work i (candidate i).2
    if (candidate i).1 then stage candidate next rest else (false, next)

theorem stage_success_iff {α : Type} (candidate : ℕ → Bool × α)
    (indices : List ℕ) (work : ℕ → α) :
    (stage candidate work indices).1 = true ↔ ∀ i ∈ indices, (candidate i).1 = true := by
  induction indices generalizing work with
  | nil => simp [stage]
  | cons i rest ih =>
    cases h : (candidate i).1 <;> simp [stage, h, ih]

theorem stage_outside {α : Type} (candidate : ℕ → Bool × α)
    (indices : List ℕ) (work : ℕ → α) (i : ℕ) (hi : i ∉ indices) :
    (stage candidate work indices).2 i = work i := by
  induction indices generalizing work with
  | nil => rfl
  | cons j rest ih =>
    have parts : i ≠ j ∧ i ∉ rest := by simpa using hi
    cases h : (candidate j).1 with
    | false => simp [stage, h, Ring.store, parts.1]
    | true =>
      simpa [stage, h, Ring.store, parts.1] using
        ih (Ring.store work j (candidate j).2) parts.2

theorem stage_success_at {α : Type} (candidate : ℕ → Bool × α)
    (indices : List ℕ) (work : ℕ → α) (i : ℕ) (hi : i ∈ indices)
    (hs : (stage candidate work indices).1 = true) :
    (stage candidate work indices).2 i = (candidate i).2 := by
  induction indices generalizing work with
  | nil => simp at hi
  | cons j rest ih =>
    have hj : (candidate j).1 = true := (stage_success_iff candidate (j :: rest) work).mp hs j (by simp)
    have hr : (stage candidate (Ring.store work j (candidate j).2) rest).1 = true := by
      simpa [stage, hj] using hs
    simp only [stage, hj, ↓reduceIte]
    by_cases hir : i ∈ rest
    · exact ih _ hir hr
    · have hij : i = j := by simpa [hir] using hi
      rw [stage_outside candidate rest _ i hir]
      subst i
      simp [Ring.store]

/-- The failed slot is written to private scratch, then traversal stops. -/
theorem stage_failure_witness {α : Type} (candidate : ℕ → Bool × α)
    (indices : List ℕ) (work : ℕ → α) (hs : (stage candidate work indices).1 = false) :
    ∃ before failed after, indices = before ++ failed :: after ∧
      (∀ i ∈ before, (candidate i).1 = true) ∧ (candidate failed).1 = false := by
  induction indices generalizing work with
  | nil => simp [stage] at hs
  | cons i rest ih =>
    cases h : (candidate i).1 with
    | false => exact ⟨[], i, rest, rfl, by simp, h⟩
    | true =>
      obtain ⟨before, failed, after, he, hp, hf⟩ := ih _ (by simpa [stage, h] using hs)
      exact ⟨i :: before, failed, after, by simp [he], by simpa using And.intro h hp, hf⟩

def commit {α : Type} (work : ℕ → α) (history : ℕ → α) : List ℕ → ℕ → α
  | [] => history
  | i :: rest => commit work (Ring.store history i (work i)) rest

theorem commit_at {α : Type} (work history : ℕ → α) (indices : List ℕ) (i : ℕ) :
    commit work history indices i = if i ∈ indices then work i else history i := by
  induction indices generalizing history with
  | nil => simp [commit]
  | cons j rest ih =>
    rw [commit, ih]
    by_cases hr : i ∈ rest <;> by_cases hj : i = j <;>
      simp [hr, hj, Ring.store]

def transaction {α : Type} (candidate : ℕ → Bool × α) (work history : ℕ → α)
    (indices : List ℕ) : Bool × (ℕ → α) :=
  let staged := stage candidate work indices
  if staged.1 then (true, commit staged.2 history indices) else (false, history)

theorem transaction_failure_frame {α : Type} (candidate : ℕ → Bool × α)
    (work history : ℕ → α) (indices : List ℕ)
    (hf : (stage candidate work indices).1 = false) :
    transaction candidate work history indices = (false, history) := by
  simp [transaction, hf]

theorem transaction_success_fields {α : Type} (candidate : ℕ → Bool × α)
    (work history : ℕ → α) (indices : List ℕ)
    (hs : (stage candidate work indices).1 = true) (i : ℕ) :
    (transaction candidate work history indices).2 i =
      if i ∈ indices then (candidate i).2 else history i := by
  simp only [transaction, hs, ↓reduceIte, commit_at]
  split_ifs with hi
  · exact stage_success_at candidate indices work i hi hs
  · rfl

structure Epochs where
  current : ℕ
  fusion : ℕ

def Epochs.Valid (s : Epochs) : Prop := 0 < s.fusion ∧ s.fusion ≤ s.current

theorem seed_epochs (epoch : ℕ) (h : 0 < epoch) : Epochs.Valid ⟨epoch, epoch⟩ := by
  exact ⟨h, le_refl _⟩

theorem predict_epochs (s : Epochs) (h : s.Valid) (endpoint : ℕ) (he : s.current < endpoint) :
    ({ s with current := endpoint } : Epochs).Valid := by
  exact ⟨h.1, h.2.trans (Nat.le_of_lt he)⟩

theorem correct_epochs (s : Epochs) (h : s.Valid) (delayed : ℕ)
    (hf : s.fusion < delayed) (hc : delayed ≤ s.current) :
    ({ s with fusion := delayed } : Epochs).Valid := by
  exact ⟨lt_trans h.1 hf, hc⟩

/-- Typed copies and transforms preserve all three timing fields. -/
structure Record (α : Type) where
  time : ℕ
  start : ℕ
  period : ℕ
  payload : α

def replacePayload {α β : Type} (f : α → β) (r : Record α) : Record β :=
  ⟨r.time, r.start, r.period, f r.payload⟩

theorem transform_epochs {α β : Type} (f : α → β) (r : Record α) :
    (replacePayload f r).time = r.time ∧ (replacePayload f r).start = r.start ∧
      (replacePayload f r).period = r.period := by exact ⟨rfl, rfl, rfl⟩

theorem transform_order {α β : Type} (f : α → β) (xs : List (Record α))
    (h : xs.Pairwise fun a b => a.time < b.time) :
    (xs.map (replacePayload f)).Pairwise fun a b => a.time < b.time := by
  simpa [List.pairwise_map, replacePayload] using h

theorem predict_order {α : Type} (capacity current endpoint period : ℕ)
    (xs : List (Record α)) (payload : α)
    (ho : xs.Pairwise fun a b => a.time < b.time)
    (hb : ∀ x ∈ xs, x.time ≤ current) (he : current < endpoint) :
    (Observation.enqueue capacity xs ⟨endpoint, current, period, payload⟩).Pairwise
      fun a b => a.time < b.time := by
  apply Observation.enqueue_order (fun r => r.time) capacity xs _ ho
  intro x hx
  exact lt_of_le_of_lt (hb x hx) he

/-- A full history with at least two ordered records has strictly positive
current-minus-oldest lag; feedback does not divide by zero under this invariant. -/
theorem feedback_positive_lag {α : Type} (head last : Record α) (middle : List (Record α))
    (ho : (head :: middle ++ [last]).Pairwise fun a b => a.time < b.time) :
    0 < last.time - head.time := by
  have h := (List.pairwise_cons.mp ho).1 last (by simp)
  omega


/-- Source traversal starts at oldest, processes one physical slot, and advances
only after a successful candidate. The counter is independent of payloads. -/
def stageLoop {α : Type} (capacity : ℕ) (candidate : ℕ → Bool × α)
    (work : ℕ → α) (index : ℕ) : ℕ → Bool × (ℕ → α)
  | 0 => (true, work)
  | n + 1 =>
    let next := Ring.store work index (candidate index).2
    if (candidate index).1 then
      stageLoop capacity candidate next (Ring.next capacity index) n
    else (false, next)

theorem stageLoop_eq_forward {α : Type} (capacity : ℕ) (candidate : ℕ → Bool × α)
    (work : ℕ → α) (index n : ℕ) :
    stageLoop capacity candidate work index n = stage candidate work (Ring.forward capacity index n) := by
  induction n generalizing work index with
  | zero => rfl
  | succ n ih =>
    simp only [stageLoop, Ring.forward, stage]
    split <;> simp [ih]

def commitLoop {α : Type} (capacity : ℕ) (work history : ℕ → α)
    (index : ℕ) : ℕ → ℕ → α
  | 0 => history
  | n + 1 => commitLoop capacity work (Ring.store history index (work index)) (Ring.next capacity index) n

theorem commitLoop_eq_forward {α : Type} (capacity : ℕ) (work history : ℕ → α)
    (index n : ℕ) :
    commitLoop capacity work history index n = commit work history (Ring.forward capacity index n) := by
  induction n generalizing history index with
  | zero => rfl
  | succ n ih => simpa [commitLoop, Ring.forward, commit] using ih _ _

theorem commit_logical {α : Type} (capacity oldest count : ℕ) (work history : ℕ → α) :
    Ring.logical (commit work history (Ring.forward capacity oldest count)) capacity oldest count =
      Ring.logical work capacity oldest count := by
  simp only [Ring.logical]
  apply List.map_congr_left
  intro i hi
  simp [commit_at, hi]

theorem successful_logical_candidates {α : Type} (capacity oldest count : ℕ)
    (candidate : ℕ → Bool × α) (work history : ℕ → α)
    (hs : (stageLoop capacity candidate work oldest count).1 = true) :
    Ring.logical (transaction candidate work history (Ring.forward capacity oldest count)).2
        capacity oldest count =
      (Ring.forward capacity oldest count).map (fun i => (candidate i).2) := by
  rw [stageLoop_eq_forward] at hs
  simp only [Ring.logical]
  apply List.map_congr_left
  intro i hi
  simp [transaction_success_fields candidate work history _ hs i, hi]

/-- The seed-only path uses one scratch record but commits no ring entries. -/
def seedReset {α : Type} (seed : α) (candidate : Bool × α) : Bool × α :=
  if candidate.1 then (true, candidate.2) else (false, seed)

theorem seed_reset_failure {α : Type} (seed : α) (candidate : Bool × α)
    (hf : candidate.1 = false) : seedReset seed candidate = (false, seed) := by
  simp [seedReset, hf]

theorem seed_reset_success {α : Type} (seed : α) (candidate : Bool × α)
    (hs : candidate.1 = true) : seedReset seed candidate = (true, candidate.2) := by
  simp [seedReset, hs]


def chainFrom {α : Type} (start : ℕ) : List (Record α) → Prop
  | [] => True
  | r :: rest => r.start = start ∧ start < r.time ∧ chainFrom r.time rest

def endTime {α : Type} (start : ℕ) : List (Record α) → ℕ
  | [] => start
  | r :: rest => endTime r.time rest

theorem chain_append {α : Type} (start : ℕ) (xs : List (Record α)) (a : Record α)
    (h : chainFrom start xs) (ha : a.start = endTime start xs)
    (ht : endTime start xs < a.time) : chainFrom start (xs ++ [a]) := by
  induction xs generalizing start with
  | nil => simpa [chainFrom, endTime] using And.intro ha ht
  | cons r rest ih =>
    simp only [chainFrom] at h
    simp only [List.cons_append, chainFrom]
    exact ⟨h.1, h.2.1, ih r.time h.2.2 ha ht⟩

theorem chain_drop {α : Type} (n start : ℕ) (xs : List (Record α)) (h : chainFrom start xs) :
    chainFrom (endTime start (xs.take n)) (xs.drop n) := by
  induction n generalizing start xs with
  | zero => simpa [endTime] using h
  | succ n ih =>
    cases xs with
    | nil => simp [endTime, chainFrom]
    | cons r rest =>
      simp only [chainFrom] at h
      simpa [endTime] using ih r.time rest h.2.2

theorem end_append {α : Type} (start : ℕ) (xs : List (Record α)) (a : Record α) :
    endTime start (xs ++ [a]) = a.time := by
  induction xs generalizing start with
  | nil => rfl
  | cons r rest ih => simpa [endTime] using ih r.time

theorem transformed_chain {α β : Type} (mapRecord : Record α → Record β)
    (ht : ∀ r, (mapRecord r).time = r.time) (hs : ∀ r, (mapRecord r).start = r.start)
    (start : ℕ) (xs : List (Record α)) (h : chainFrom start xs) :
    chainFrom start (xs.map mapRecord) := by
  induction xs generalizing start with
  | nil => trivial
  | cons r rest ih =>
    simp only [List.map_cons, chainFrom, ht, hs] at *
    exact ⟨h.1, h.2.1, ih r.time h.2.2⟩

theorem transformed_end {α β : Type} (mapRecord : Record α → Record β)
    (ht : ∀ r, (mapRecord r).time = r.time) (start : ℕ) (xs : List (Record α)) :
    endTime start (xs.map mapRecord) = endTime start xs := by
  induction xs generalizing start with
  | nil => rfl
  | cons r rest ih => simpa [endTime, ht] using ih r.time

/-- The overwrite guard only discards a completed record. Every retained pending
record remains available in insertion order after the typed push. -/
theorem full_push_preserves_pending {α : Type} (fusion : ℕ) (head a x : Record α)
    (rest : List (Record α)) (hc : head.time ≤ fusion) (hx : x ∈ head :: rest)
    (hp : fusion < x.time) : x ∈ rest ++ [a] := by
  have hne : x ≠ head := by intro he; subst x; omega
  have hr : x ∈ rest := by simpa [hne] using hx
  exact List.mem_append_left _ hr

/-- Failure latches the owner until an explicit session reset. The source
caller guards use only this latch; scratch writes never clear it. -/
def active (epoch : ℕ) (faulted : Bool) : Bool := epoch != 0 && !faulted

theorem latched_blocks (epoch : ℕ) : active epoch true = false := by simp [active]

/-- The source permits configuration only before seeding. The timing producer
supplies lengths in [2, 64]; reset retains that length but clears the epoch.
This invariant supplies the nonzero-divisor premise for physical ring traversal.
Fault and feedback fields do not change either component. -/
structure Session where
  length : ℕ
  epoch : ℕ

def Session.Valid (s : Session) : Prop :=
  (s.length = 0 ∨ 2 ≤ s.length) ∧ s.length ≤ 64 ∧ (s.epoch ≠ 0 → 2 ≤ s.length)

theorem session_initial : Session.Valid ⟨0, 0⟩ := by simp [Session.Valid]

theorem session_configure (length : ℕ) (hl : 2 ≤ length ∧ length ≤ 64) :
    Session.Valid ⟨length, 0⟩ := by
  exact ⟨Or.inr hl.1, hl.2, fun h => False.elim (h rfl)⟩

theorem session_seed (s : Session) (h : s.Valid) (hl : 0 < s.length) (epoch : ℕ) :
    ({ s with epoch := epoch } : Session).Valid := by
  have hn : 2 ≤ s.length := by
    rcases h.1 with hz | hn
    · omega
    · exact hn
  exact ⟨h.1, h.2.1, fun _ => hn⟩

theorem session_predict (s : Session) (h : s.Valid) (ha : s.epoch ≠ 0) (endpoint : ℕ) :
    ({ s with epoch := endpoint } : Session).Valid := by
  exact ⟨h.1, h.2.1, fun _ => h.2.2 ha⟩

theorem session_reset (s : Session) (h : s.Valid) :
    ({ s with epoch := 0 } : Session).Valid := by
  exact ⟨h.1, h.2.1, fun hn => False.elim (hn rfl)⟩


/-- Common left reset preserves the exact attitude tracking difference. The
floating implementation normalizes its products and has separate roundoff. -/
theorem common_left_reset_residual {R : Type*} [CommRing R]
    (delta main output : Quaternion R) (hd : Quaternion.HasUnitNorm delta) :
    Quaternion.hamiltonMul (Quaternion.conjugate (Quaternion.hamiltonMul delta main))
        (Quaternion.hamiltonMul delta output) =
      Quaternion.hamiltonMul (Quaternion.conjugate main) output := by
  rw [Quaternion.conjugate_hamiltonMul, Quaternion.hamiltonMul_assoc,
    ← Quaternion.hamiltonMul_assoc (Quaternion.conjugate delta) delta output,
    Quaternion.conjugate_hamiltonMul_of_unit hd, Quaternion.identity_hamiltonMul]

theorem common_translation_residual {V : Type*} [AddCommGroup V]
    (main output delta : V) : (main + delta) - (output + delta) = main - output := by
  abel

/-- Exact-real counterpart of the per-axis shortest-sign tracking proxy.
The q0 = 0 boundary deliberately selects -2, as the source does. -/
noncomputable def attitudeError (q0 component : ℝ) : ℝ :=
  component * (if q0 < 0 then 2 else -2)

theorem attitude_error_zero_boundary (component : ℝ) :
    attitudeError 0 component = -2 * component := by
  simp [attitudeError]
  ring

theorem attitude_error_negation (q0 component : ℝ) (h : q0 ≠ 0) :
    attitudeError (-q0) (-component) = attitudeError q0 component := by
  rcases lt_or_gt_of_ne h with hn | hp
  · simp [attitudeError, hn, not_lt.mpr (le_of_lt (neg_pos.mpr hn))]
  · simp [attitudeError, not_lt.mpr (le_of_lt hp), neg_lt_zero.mpr hp]

noncomputable def rateFeedback (error lag : ℝ) : ℝ := error * ((1 / 2) / lag)

theorem rate_feedback_over_lag (error lag : ℝ) (h : lag ≠ 0) :
    rateFeedback error lag * lag = error / 2 := by
  unfold rateFeedback
  field_simp

noncomputable def filteredPeriod (previous sample : ℝ) : ℝ := (4 / 5) * previous + (1 / 5) * sample

theorem filtered_period_fixed (period : ℝ) : filteredPeriod period period = period := by
  unfold filteredPeriod
  ring

theorem filtered_period_bounds (previous sample lower upper : ℝ)
    (hp : lower ≤ previous ∧ previous ≤ upper) (hs : lower ≤ sample ∧ sample ≤ upper) :
    lower ≤ filteredPeriod previous sample ∧ filteredPeriod previous sample ≤ upper := by
  unfold filteredPeriod
  constructor <;> linarith

/-- Per-axis PI translation uses the updated integral, not its previous value.
This is algebra, not a closed-loop stability or floating-point error theorem. -/
noncomputable def piTranslation (error previousSum interval tau : ℝ) : ℝ :=
  let gain := interval / tau
  error * gain + (previousSum + error) * (gain * gain * (1 / 10))

theorem pi_translation_zero_error (previousSum interval tau : ℝ) :
    piTranslation 0 previousSum interval tau = previousSum * ((interval / tau) ^ 2 / 10) := by
  unfold piTranslation
  ring

theorem pi_translation_zero_memory (interval tau : ℝ) : piTranslation 0 0 interval tau = 0 := by
  simp [piTranslation]

end FormalESKF.Runtime.Output
