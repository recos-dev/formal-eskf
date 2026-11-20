/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this file.
SPDX-License-Identifier: BSD-3-Clause
-/
import FormalESKF.RingBuffer

/-! Induction over the temporal relations supplied by ObservationHistory's
source contracts. This is not a translation of the C++ owner. Concrete typed
storage, call forwarding, IEEE period bounds and source correspondence remain
separate mandatory evidence. -/
namespace FormalESKF.Runtime.Observation

structure Epochs where
  seed : ℕ
  latest : ℕ
  fusion : ℕ
  advanced : ℕ

def Epochs.Valid (s : Epochs) : Prop :=
  0 < s.seed ∧ s.seed ≤ s.fusion ∧ s.fusion ≤ s.latest ∧ s.advanced ≤ s.latest

theorem seed_valid (epoch : ℕ) (h : 0 < epoch) :
    Epochs.Valid ⟨epoch, epoch, epoch, 0⟩ := by
  simpa [Epochs.Valid] using h

theorem record_valid (s : Epochs) (h : s.Valid) (endpoint : ℕ) (he : s.latest < endpoint) :
    ({ s with latest := endpoint } : Epochs).Valid := by
  unfold Epochs.Valid at *
  simp only at *
  omega

theorem begin_valid (s : Epochs) (h : s.Valid) (endpoint : ℕ)
    (hf : s.fusion < endpoint) (hl : endpoint ≤ s.latest) :
    ({ s with fusion := endpoint, advanced := s.latest } : Epochs).Valid := by
  unfold Epochs.Valid at *
  simp only at *
  omega

theorem next_query_not_future (s : Epochs) (h : s.Valid) (endpoint : ℕ)
    (hf : s.fusion < endpoint) (hl : endpoint ≤ s.latest) :
    s.seed < endpoint ∧ endpoint ≤ s.latest := by
  unfold Epochs.Valid at h
  omega

/-- Selection is independent for each source. A successful begin strictly
advances fusion; recording a real IMU alone does not permit a repeated pop. -/
theorem selection_after_begin (previous fusion endpoint : ℕ)
    (h : previous ≤ fusion) (he : fusion < endpoint) : previous < endpoint := by omega

structure Timeline where
  raw : ℕ
  admitted : ℕ
  arrival : ℕ
  generation : ℕ

def Timeline.Valid (s : Timeline) (latest : ℕ) : Prop :=
  s.generation ≤ s.raw ∧ s.admitted ≤ s.raw ∧ s.raw ≤ max s.arrival latest

theorem timeline_seed (raw latest : ℕ) (h : raw ≤ latest) :
    Timeline.Valid ⟨raw, 0, 0, 0⟩ latest := by
  simp [Timeline.Valid]
  exact h

theorem timeline_imu_advance (s : Timeline) (latest endpoint : ℕ)
    (h : s.Valid latest) (he : latest ≤ endpoint) : s.Valid endpoint := by
  unfold Timeline.Valid at *
  exact ⟨h.1, h.2.1, h.2.2.trans (max_le_max_left _ he)⟩

/-- The first mutation is arrival publication. It also covers raw out-of-order
rejection; impossible times are the identity transition. -/
theorem timeline_arrival (s : Timeline) (latest now : ℕ) (h : s.Valid latest)
    (hl : latest ≤ now) (ha : s.arrival ≤ now) :
    ({ s with arrival := now } : Timeline).Valid latest := by
  unfold Timeline.Valid at *
  refine ⟨h.1, h.2.1, ?_⟩
  exact h.2.2.trans ((max_le ha hl).trans (le_max_left _ _))

/-- Source changes occur only with a strictly newer raw timestamp. Retained
admission watermarks and a bounded generation therefore remain valid, even if
all subsequent age/rate checks reject the new sample. -/
theorem timeline_new_raw (s : Timeline) (latest now raw : ℕ) (changed : Bool)
    (h : s.Valid latest) (hr : s.raw < raw) (hn : raw ≤ now) :
    Timeline.Valid ⟨raw, s.admitted, now, s.generation + if changed then 1 else 0⟩ latest := by
  unfold Timeline.Valid at *
  have hg : s.generation ≤ s.raw := h.1
  have had : s.admitted ≤ s.raw := h.2.1
  have hb : raw ≤ max now latest := hn.trans (le_max_left _ _)
  cases changed <;> simp_all <;> omega

theorem generation_no_wrap (s : Timeline) (raw limit : ℕ) (changed : Bool)
    (hg : s.generation ≤ s.raw) (hr : s.raw < raw) (hb : raw ≤ limit) :
    s.generation + (if changed then 1 else 0) ≤ limit := by
  cases changed <;> simp_all <;> omega

theorem timeline_admitted (s : Timeline) (latest effective : ℕ)
    (h : s.Valid latest) (he : effective ≤ s.raw) :
    ({ s with admitted := effective } : Timeline).Valid latest := by
  exact ⟨h.1, he, h.2.2⟩

/-- The half-period and delay subtractions have their unsigned guards discharged
before this relation is used. -/
theorem effective_time (raw delay half : ℕ) (hd : delay < raw) (hh : half < raw - delay) :
    0 < raw - delay - half ∧ raw - delay - half ≤ raw := by omega

/-- Full-capacity push retains the suffix of the insertion-order list. -/
def enqueue {α : Type} (capacity : ℕ) (xs : List α) (a : α) : List α :=
  (xs ++ [a]).drop (xs.length + 1 - capacity)

/-- The typed store/metadata contract refines this owner's bounded enqueue
relation. Word-width bounds and C++ field copying are separate source checks. -/
theorem physical_enqueue_refines {α : Type} (read : ℕ → α) (value : α)
    {capacity oldest count write : ℕ} (hc : 0 < capacity) (ho : oldest < capacity)
    (hn : count ≤ capacity) (hw : write = (oldest + count) % capacity) :
    Ring.logical (Ring.store read write value) capacity
        (if count = capacity then Ring.next capacity oldest else oldest)
        (if count = capacity then count else count + 1) =
      enqueue capacity (Ring.logical read capacity oldest count) value := by
  have he : write = Ring.advance capacity oldest count :=
    hw.trans (Ring.advance_mod hc ho count).symm
  simpa [enqueue, Ring.logical_length] using Ring.logical_enqueue read value hc ho hn he

theorem enqueue_length {α : Type} (capacity : ℕ) (xs : List α) (a : α) :
    (enqueue capacity xs a).length ≤ capacity := by
  simp [enqueue]
  omega

theorem enqueue_order {α : Type} (time : α → ℕ) (capacity : ℕ) (xs : List α) (a : α)
    (h : xs.Pairwise fun x y => time x < time y) (ha : ∀ x ∈ xs, time x < time a) :
    (enqueue capacity xs a).Pairwise fun x y => time x < time y := by
  apply List.Pairwise.drop
  rw [List.pairwise_append]
  exact ⟨h, by simp, by simpa using ha⟩

/-- Admission retains its watermark after pop or source reset. The zero
watermark branch can contain no positive-time active observation. -/
theorem admission_after_retained {α : Type} (time : α → ℕ) (xs : List α) (watermark effective : ℕ)
    (hp : ∀ x ∈ xs, 0 < time x) (hw : ∀ x ∈ xs, time x ≤ watermark)
    (ha : watermark = 0 ∨ watermark < effective) :
    ∀ x ∈ xs, time x < effective := by
  intro x hx
  have hb := hw x hx
  have hpos := hp x hx
  rcases ha with hz | hn <;> omega

/-- P includes current source/generation and provenance fields. On a source
change the caller supplies an empty old list; otherwise admission preserves P.
The new record itself must satisfy P through the complete forwarding contract. -/
theorem enqueue_fields {α : Type} (P : α → Prop) (capacity : ℕ) (xs : List α) (a : α)
    (h : ∀ x ∈ xs, P x) (ha : P a) : ∀ x ∈ enqueue capacity xs a, P x := by
  intro x hx
  have hm : x ∈ xs ++ [a] := List.mem_of_mem_drop hx
  rcases List.mem_append.mp hm with hs | hs
  · exact h x hs
  · have he : x = a := List.mem_singleton.mp hs
    subst x
    exact ha

theorem enqueue_watermark {α : Type} (time : α → ℕ) (capacity : ℕ) (xs : List α) (a : α)
    (h : ∀ x ∈ xs, time x ≤ time a) :
    ∀ x ∈ enqueue capacity xs a, time x ≤ time a :=
  enqueue_fields (fun x => time x ≤ time a) capacity xs a h (le_refl _)

theorem retained_order {α : Type} (time : α → ℕ) (xs : List α) (removed : ℕ)
    (h : xs.Pairwise fun x y => time x < time y) :
    (xs.drop removed).Pairwise fun x y => time x < time y := h.drop

theorem retained_fields {α : Type} (P : α → Prop) (xs : List α) (removed : ℕ)
    (h : ∀ x ∈ xs, P x) : ∀ x ∈ xs.drop removed, P x := by
  intro x hx
  exact h x (List.mem_of_mem_drop hx)

/-- Search order is newest to oldest. Its retained prefix is all future entries;
chronology ensures every retired entry is at or before the horizon. -/
theorem ordered_selection {α : Type} (time : α → ℕ) (horizon : ℕ) (xs : List α) (a : α)
    (ho : xs.Pairwise fun x y => time y < time x)
    (hm : (Ring.search time horizon xs).candidate = some a) :
    ∃ future retired, xs = future ++ a :: retired ∧
      (∀ b ∈ future, horizon < time b) ∧
      (∀ b ∈ a :: retired, time b ≤ horizon) ∧
      (Ring.search time horizon xs).removed = retired.length + 1 := by
  obtain ⟨future, retired, he, hf, ht, hr⟩ := Ring.search_match time horizon xs a hm
  refine ⟨future, retired, he, hf, ?_, hr⟩
  rw [he, List.pairwise_append] at ho
  have tail_order := List.pairwise_cons.mp ho.2.1
  intro b hb
  rcases List.mem_cons.mp hb with rfl | hb
  · exact ht
  · exact (tail_order.1 b hb).le.trans ht

/-- Original message times survive unchanged until a newer, nonfuture source
message is observed. The other payload fields follow the same branch in C++. -/
def snapshotMessage (previous epoch incoming : ℕ) (oldKnown newKnown : Bool) : ℕ × Bool :=
  if previous < incoming ∧ incoming ≤ epoch then (incoming, newKnown) else (previous, oldKnown)

theorem snapshot_message_valid (previous epoch incoming : ℕ) (oldKnown newKnown : Bool)
    (hp : previous ≤ epoch) (hk : oldKnown = true → 0 < previous) :
    (snapshotMessage previous epoch incoming oldKnown newKnown).1 ≤ epoch ∧
      ((snapshotMessage previous epoch incoming oldKnown newKnown).2 = true →
        0 < (snapshotMessage previous epoch incoming oldKnown newKnown).1) := by
  unfold snapshotMessage
  split_ifs with h
  · simp only
    constructor
    · exact h.2
    · intro _; omega
  · exact ⟨hp, hk⟩

theorem context_known_age (message stored query age : ℕ) (known : Bool)
    (hm : message ≤ stored) (hs : stored ≤ query) (hk : known = true → 0 < message)
    (h : (known && decide (query - message < age)) = true) :
    0 < message ∧ message ≤ query ∧ query - message < age := by
  simp only [Bool.and_eq_true, decide_eq_true_eq] at h
  exact ⟨hk h.1, hm.trans hs, h.2⟩

/-- Context lookup uses the same newest-to-oldest stop candidate as the ring
search. It does not consume or retire entries. The typed source step supplies
all payload-copy equations and the two original-message age masks. -/
def contextResult {α : Type} (time : α → ℕ) (mask : α → α) (unknown : α)
    (query hold : ℕ) : Option α → α
  | none => unknown
  | some a => if query - time a < hold then mask a else unknown

def contextLoop {α : Type} (read : ℕ → α) (time : α → ℕ) (mask : α → α)
    (unknown : α) (query hold capacity cursor : ℕ) : ℕ → α
  | 0 => unknown
  | n + 1 =>
    let index := Ring.previous capacity cursor
    if time (read index) ≤ query then
      contextResult time mask unknown query hold (some (read index))
    else contextLoop read time mask unknown query hold capacity index n

theorem contextLoop_eq_search {α : Type} (read : ℕ → α) (time : α → ℕ) (mask : α → α)
    (unknown : α) (query hold capacity cursor count : ℕ) :
    contextLoop read time mask unknown query hold capacity cursor count =
      contextResult time mask unknown query hold
        (Ring.searchLoop read time query capacity cursor count).candidate := by
  induction count generalizing cursor with
  | zero => rfl
  | succ n ih =>
    simp only [contextLoop, Ring.searchLoop]
    split_ifs <;> simp_all

theorem context_missing {α : Type} (time : α → ℕ) (mask : α → α) (unknown : α)
    (query hold : ℕ) (xs : List α) (h : ∀ a ∈ xs, query < time a) :
    contextResult time mask unknown query hold (Ring.search time query xs).candidate = unknown := by
  rw [(Ring.search_none_iff time query xs).mpr h]
  rfl

theorem context_expired {α : Type} (time : α → ℕ) (mask : α → α) (unknown a : α)
    (query hold : ℕ) (xs : List α) (hm : (Ring.search time query xs).candidate = some a)
    (he : hold ≤ query - time a) :
    contextResult time mask unknown query hold (Ring.search time query xs).candidate = unknown := by
  simp [hm, contextResult, Nat.not_lt.mpr he]

theorem context_fresh {α : Type} (time : α → ℕ) (mask : α → α) (unknown a : α)
    (query hold : ℕ) (xs : List α) (hm : (Ring.search time query xs).candidate = some a)
    (hf : query - time a < hold) :
    contextResult time mask unknown query hold (Ring.search time query xs).candidate = mask a := by
  simp [hm, contextResult, hf]

end FormalESKF.Runtime.Observation
