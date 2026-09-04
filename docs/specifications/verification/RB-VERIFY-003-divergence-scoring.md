# RB-VERIFY-003 — Divergence Scoring

- Version: 0.11.0
- Status: Draft (all four functional requirements implemented and wired
  into `rb_verify_cli`; the first three run end-to-end against a real
  replay AND a real BakkesMod capture, closing `PHASE-0-EXIT`'s own
  literal exit criterion; the fourth, a divergence-growth diagnostic, has
  now run against the real capture too, showing the divergence is abrupt
  — a sharp derailment around a dodge maneuver — rather than gradual, see
  Verification plan; open questions remain about calibrating an actual
  "good enough" threshold, see Open questions)
- Owners: baileyrd
- Depends on: RB-VERIFY-001, RB-VERIFY-002
- Supersedes: none

## Purpose and scope

Define and compute the single accuracy metric the whole project tunes
against: how far a candidate physics/simulation implementation's output
trajectory diverges from a recorded ground-truth trajectory, given matching
initial state and (where available) matching recorded inputs.

In scope: the scoring algorithm itself (`rb_domain::divergence`), operating
on two `Vec<PhysicsFrame>` sequences regardless of which adapter produced
them.

## Non-goals

- Not responsible for producing either input sequence (`RB-VERIFY-001`,
  `RB-VERIFY-002`) or for running a candidate physics engine to generate
  its output — that's `RB-PHYSICS-001`'s composition-root responsibility
  once it exists.
- Not a validator of online-specific netcode behavior (lag,
  reconciliation). That is validated separately against the GDC 2018 talk's
  documented architecture, since no ground-truth online-input data source
  exists — see [docs/research/RESEARCH-BACKLOG.md](../../research/RESEARCH-BACKLOG.md).

## Context and terminology

- **Divergence score**: `rb_domain::divergence::DivergenceScore` — mean and
  max ball-position distance across compared frames, the frame count
  compared, and a nested `CarDivergence` (mean/max car position, rotation,
  and velocity distance, plus the car-pair count compared).
- **Recorded trajectory** / **candidate trajectory**: the two
  `Vec<PhysicsFrame>` sequences being compared.

## Requirements

- `RB-VERIFY-003-FR-001` (implemented): Given two frame sequences, compute
  mean and max ball-position distance across the overlapping length.
- `RB-VERIFY-003-FR-002` (implemented): Extend scoring to car position/
  rotation/velocity, not just ball position — needed before this metric
  can validate car-feel fidelity (Phase 1's actual concern), not just
  ball physics. Cars are matched between the two sequences by
  `player_id` within each frame pair (see Data/state and invariants); a
  car present on only one side of a frame pair is skipped for that
  frame, not an error. Rotation distance is the angle between the two
  quaternions (`Quat::angle_to`, radians), not a per-axis Euler
  difference.
- `RB-VERIFY-003-FR-003` (implemented): Timestamp-tolerant alignment
  between sequences sampled at different tick rates or with a start-time
  offset, replacing the original index-pairwise simplification. Each
  recorded frame is matched to the candidate frame nearest it in
  `timestamp_secs` (an `O(recorded.len() + candidate.len())` merge over
  both sequences' existing chronological order, not a binary search per
  frame); a match only counts if the two frames are within a caller-
  supplied `max_timestamp_delta_secs` of each other, so sequences that
  don't actually overlap in time don't get force-matched to whatever's
  nearest-but-still-distant. See Architecture and interfaces.
- `RB-VERIFY-003-NFR-001` (implemented): Scoring an empty or
  mismatched-length pair of sequences never panics or produces `NaN`.
- `RB-VERIFY-003-FR-004` (implemented): A divergence-growth diagnostic —
  report how divergence changes *within* a single run, instead of only
  the one whole-run mean/max pair `score` already produces. Needed
  because `RB-PHYSICS-001-FR-077`'s first real candidate-vs-capture run
  produced a whole-run number consistent with near-total trajectory
  divergence, which cannot by itself distinguish gradual compounding
  error (many small modeling gaps adding up) from an abrupt one (a
  specific early mechanic mismatch derailing everything after it) — a
  distinction `RB-PHYSICS-001-FR-005`'s eventual constant calibration
  needs answered first, per that run's own Interpretation note.
  - **Design, as implemented**: `rb_domain::divergence::score_windows(
    recorded: &[PhysicsFrame], candidate: &[PhysicsFrame],
    max_timestamp_delta_secs: f32, window_secs: f32) -> Vec<(f32,
    DivergenceScore)>`, reusing exactly the same nearest-timestamp
    matching `FR-003` already established (both `score` and
    `score_windows` now build on a shared internal `matched_pairs`/
    `score_pairs` pipeline, so the two can never silently drift apart),
    partitioning the matched pairs into consecutive, non-overlapping
    `window_secs`-wide buckets keyed by each pair's own recorded
    timestamp (the first window starts at the first compared pair's
    timestamp, not a fixed `0.0`, so a mid-file seed frame doesn't
    produce a mostly-empty leading window). A run whose pairs all fall in
    one window reproduces exactly the same numbers `score` computes on
    the same input, verified directly by a unit test.
  - **CLI wiring, as implemented**: `rb_verify_cli::score_capture_growth`,
    a sibling entry point sharing the exact seed-frame selection and
    `simulate_recorded` call `score_capture_against_candidate` already
    used (both now call a shared private `seed_and_simulate` helper), and
    a new `rb-verify --self-growth <capture-file> [window-secs]
    [max-timestamp-delta-secs]` mode printing one line per window (window
    start time, frames compared, mean/max ball distance, mean car
    position/rotation/velocity distance) alongside the existing `--self`
    mode.
  - **Default `window_secs`**: `rb_verify_cli::DEFAULT_GROWTH_WINDOW_SECS
    = 1.0` — the one real capture run recorded so far (~23 seconds, 2,818
    frames) prints as roughly 23 rows: small enough to read on one
    screen, fine enough to localize an abrupt derailment to roughly which
    second it started.
  - **Non-goals**: no automatic gradual-vs-abrupt classification
    (changepoint detection, curve fitting) — a human reads the printed
    series and judges its shape, the same "read together" interpretive
    convention `FR-077`'s own Interpretation note already established.
    No new output format (CSV/file export) — stdout table only, matching
    every existing `rb-verify` mode. Does not itself perform
    `RB-PHYSICS-001-FR-005`'s real-data calibration or change any physics
    constant or the seed-frame heuristic — purely a diagnostic read of
    the one real run already recorded, and this FR does not itself run
    it against that real capture (the owner still needs to do that on
    their own machine, the same as `FR-077`'s own run). Only exercises
    the existing single-car freeplay capture; multi-car growth
    diagnostics stay out of scope until a multi-car capture exists, the
    same limit `FR-077` already carries.

## Architecture and interfaces

`rb_domain::divergence::score(recorded: &[PhysicsFrame], candidate:
&[PhysicsFrame], max_timestamp_delta_secs: f32) -> DivergenceScore`. Pure
function, no I/O — callable from `rb_verify_cli` or any future test
harness. `max_timestamp_delta_secs` is a required parameter, not a baked-in
default: what counts as "the same instant" depends on both sequences'
actual sampling rates, which this function has no way to know on its own.
`rb_domain::divergence::score_windows` (`RB-VERIFY-003-FR-004`) has the
same signature plus a `window_secs: f32` parameter and returns `Vec<(f32,
DivergenceScore)>` instead — one score per non-empty time window rather
than one for the whole run. Both functions share a private
`matched_pairs`/`score_pairs` pipeline internally, so `score` is exactly
`score_windows`'s per-window aggregation applied once to every matched
pair.

`rb_verify_cli::score_replay_against_capture` (in
`crates/rb_verify_cli/src/lib.rs`) is the actual composition-root wiring:
it ingests a replay file via `rb_replay_ingest` as the "recorded" sequence
and a capture file via `rb_capture_ingest` as the "candidate" sequence,
then calls `score`. `rb_verify_cli::score_capture_against_candidate` and
`score_capture_growth` share a private `seed_and_simulate` helper (ingest
a capture, find its first grounded/neutral frame, simulate a candidate
forward from there) and differ only in which `rb_domain::divergence`
function they call on the result. The `rb-verify` binary (`main.rs`) is a
thin argument-parsing/output wrapper over these functions (an optional
third CLI argument overrides `rb_verify_cli::DEFAULT_MAX_TIMESTAMP_DELTA_SECS`
in every mode; `--self-growth`'s own second argument overrides
`DEFAULT_GROWTH_WINDOW_SECS`), kept separate so the wiring itself is
unit-testable without spawning a process.

## Data/state and invariants

`DivergenceScore.frames_compared` counts recorded frames matched to a
candidate frame within `max_timestamp_delta_secs` — **not**
`min(recorded.len(), candidate.len())` (that was the old index-pairwise
behavior; nearest-timestamp matching can compare every recorded frame
against a much shorter candidate sequence, or compare none at all if the
two sequences' timestamps never come within tolerance of each other).
`mean_ball_distance` is `0.0` (not `NaN`) when `frames_compared == 0`.
`DivergenceScore.cars.pairs_compared`
counts matched car pairs across all compared frames (not frames, and not
`min(total recorded cars, total candidate cars)`): within each compared
frame pair, a recorded car is matched to a candidate car by equal
`player_id`; unmatched cars (present in only one side) contribute nothing.
All three `cars.mean_*` fields are `0.0`, not `NaN`, when
`pairs_compared == 0`.

## Errors, failure, recovery, and observability

No fallible operations — `score` is total over its inputs by construction.

## Security, privacy, and compatibility

None beyond what applies to the frame data itself (see
`RB-VERIFY-001`/`RB-VERIFY-002`).

## Acceptance criteria

- Implemented (ball scoring): identical trajectories score zero; a known
  constant offset scores exactly that offset; empty inputs score zero
  without panicking.
- Implemented (car scoring, FR-002): identical car states score zero
  across position/rotation/velocity; known position/velocity offsets score
  exactly that distance; a known rotation offset scores the correct angle;
  a car present in only one side of a frame pair is skipped, not an error
  (and doesn't panic); empty/no-car inputs score zero, not `NaN`. (All
  covered by unit tests in `rb_domain::divergence::tests` and
  `rb_domain::state::tests` for `Quat::angle_to`.)
- Implemented (timestamp alignment, FR-003): two sequences sampled at
  different, irregular tick rates align by nearest timestamp, not list
  index, and the resulting per-pair distances match hand-computed
  expectations; a shorter candidate sequence can still be matched against
  every recorded frame it's within tolerance of (not capped at
  `min(len, len)`); frames whose nearest available match still exceeds
  `max_timestamp_delta_secs` are skipped, not force-matched. (Covered by
  unit tests in `rb_domain::divergence::tests`.)
- Implemented (divergence-growth diagnostic, FR-004): `score_windows` on
  a run whose pairs all fall within one window reproduces `score`'s own
  numbers on the same input exactly; a synthetic two-window case (first
  window's pairs all identical, second window's pairs all offset by a
  known distance) produces exactly the expected per-window means; the
  first window starts at the first matched pair's own timestamp even
  when earlier recorded frames had no match; a run with no matched pairs
  returns no windows. `rb-verify --self-growth` was run manually against
  the synthetic capture fixture, and then for real against
  `RB-PHYSICS-001-FR-077`'s own real capture (see Verification plan for
  the full numbers) — the divergence there is **abrupt**, not gradual: a
  sharp derailment localized to roughly seconds 3–5 of the run, not a
  slow accumulation from frame 0.

## Verification plan

Unit tests (10, in `rb_domain::divergence`) for the scoring algorithm
itself (6 ball/alignment-scoring — including two added for FR-003:
different-tick-rate sequences aligning by nearest timestamp with
hand-computed expected matches, and a shorter sequence still matching
every in-tolerance recorded frame — and 4 car-scoring: identical states,
position/velocity offsets, rotation offset, a car unmatched on one side),
plus 3 tests in `rb_domain::state` for `Quat::angle_to` (identical
rotation, a known quarter-turn angle, and the quaternion double-cover
case), plus 3 tests in `rb_verify_cli` exercising the real wiring
(`score_replay_against_capture`) against `rb_replay_ingest`'s vendored
replay fixture and `rb_capture_ingest`'s synthetic capture fixture: a
happy-path run producing a non-empty score, and a missing-file case for
each input reporting `IngestError::Io`.

Implementing real timestamp alignment surfaced an actual bug in the
synthetic capture fixture: it was originally authored with timestamps
starting at `0.0`, without checking them against the vendored replay
fixture's real timeline — the replay's ball doesn't spawn (and so
produces no `PhysicsFrame`s, per `rb_replay_ingest`'s "frames where the
ball is unavailable are omitted" rule) until roughly **11.78 seconds** in
(kickoff countdown). Under the old index-pairwise comparison this went
unnoticed, since frame 0 was compared to frame 0 regardless of what
timestamp either actually carried — exactly the failure mode FR-003 exists
to catch. The fixture's timestamps were corrected to start at `11.78`
(same relative spacing) so it actually overlaps the replay's real
timeline; see `crates/rb_capture_ingest/fixtures/README.md`.

Manually run once (`cargo run -p rb_verify_cli --bin rb-verify -- <replay>
<capture>`, 2026-08-28, using `rb_verify_cli::DEFAULT_MAX_TIMESTAMP_DELTA_SECS`
= `0.02`) against those same two (now time-aligned) fixtures: `frames
compared: 6, mean ball distance: 0.25 uu, max ball distance: 0.25 uu, car
pairs compared: 6, mean car position/rotation/velocity distance: 2816.42
uu / 2.36 rad / 1307.87 uu/s, max car position/rotation/velocity distance:
2912.18 uu / 2.36 rad / 1627.83 uu/s`. This proves the ingestion → scoring
pipeline runs end-to-end without erroring across both real adapters, with
real timestamp-tolerant alignment now actually engaged (not merely
present but vacuous, as it would have been against the old, disjoint
fixture timestamps) — it is still **not** a fidelity measurement: the
replay and the capture are unrelated matches (one real, one synthetic)
with no physical reason to resemble each other, so the (very large)
car-distance numbers above mean nothing beyond "these are two different
matches," not "car scoring is broken." The actual end-to-end run this
metric exists for — recorded inputs from `RB-VERIFY-002` fed into a real
Phase 1 candidate physics engine, output compared against the recorded
outcome — still needs that candidate engine to exist (car bodies, not
just sphere-vs-plane); `RB-VERIFY-002-FR-001`'s real BakkesMod capture now
exists (see below).

Re-run for real once `RB-VERIFY-002-FR-001`'s BakkesMod capture plugin was
built and used to record a real freeplay session (`cargo run -p
rb_verify_cli -- crates/rb_replay_ingest/fixtures/subtr-actor-sample.replay
<real capture path>`, 2026-09-02, default tolerance): `frames compared:
343, mean ball distance: 3640.81 uu, max ball distance: 6015.71 uu, car
pairs compared: 343, mean car position/rotation/velocity distance: 4714.78
uu / 2.31 rad / 2127.93 uu/s, max car position/rotation/velocity distance:
7721.40 uu / 3.14 rad / 3938.20 uu/s`. This is the first time the pipeline
has run end-to-end on **two genuinely real inputs** — a real vendored
replay and a real BakkesMod recording, not a hand-authored synthetic
capture — closing `PHASE-0-EXIT`'s own literal exit criterion ("produces a
divergence score on ≥1 real replay and ≥1 real BakkesMod capture"). The
numbers themselves remain exactly as meaningless as a fidelity measurement
as the synthetic run above, for the identical reason: the replay and this
capture are two unrelated freeplay sessions with no physical reason to
resemble each other. Closing that separate, harder problem still needs a
Phase 1 candidate engine that consumes the capture's own recorded input
and produces a trajectory to compare against the capture's own recorded
outcome — genuinely out of this phase's scope, not a Phase 0 exit blocker.

`RB-VERIFY-003-FR-004`'s divergence-growth diagnostic added 4 unit tests
to `rb_domain::divergence` (14 total) — a single-window run reproducing
`score`'s own numbers exactly, a two-window run with a known offset in
only the second window, a run whose first several recorded frames have no
match (confirming the first window starts at the first *matched* pair's
timestamp, not the first recorded frame's), and a run with no matched
pairs returning no windows — and 3 to `rb_verify_cli` (9 total): a
happy-path run against the synthetic capture fixture, a missing-file case,
and the same no-grounded-neutral-frame case `score_capture_against_candidate`
already covered. Manually run once (`cargo run -p rb_verify_cli --bin
rb-verify -- --self-growth crates/rb_capture_ingest/fixtures/example.capture.jsonl`,
2026-09-04, default `window_secs = 1.0`) against the synthetic capture
fixture: `t=11.78s frames=5 ball mean/max=0.75/2.17 uu car mean
pos/rot/vel=58.75 uu / 0.05 rad / 600.40 uu/s` — a single window, since the
fixture's own 5 frames all fall within one second, proving the CLI mode
runs end-to-end without erroring.

**Run for real, 2026-09-04, against `RB-PHYSICS-001-FR-077`'s own real
capture** (`test2.jsonl`, owner's machine, default `window_secs = 1.0` and
`max_timestamp_delta_secs = 0.02`), and independently reproduced bit-for-bit
against the same capture file in this sandbox:

```
t=   0.00s  frames= 120  ball mean/max=    0.04/    0.06 uu  car mean pos/rot/vel=    2.23 uu / 0.01 rad /     0.96 uu/s
t=   1.00s  frames= 120  ball mean/max=    0.05/    0.05 uu  car mean pos/rot/vel=    2.33 uu / 0.01 rad /     0.27 uu/s
t=   2.00s  frames= 120  ball mean/max=    0.05/    0.05 uu  car mean pos/rot/vel=    2.33 uu / 0.01 rad /     0.27 uu/s
t=   3.00s  frames= 120  ball mean/max=    0.05/    0.05 uu  car mean pos/rot/vel=   33.81 uu / 0.06 rad /   164.41 uu/s
t=   4.00s  frames= 120  ball mean/max=    0.05/    0.05 uu  car mean pos/rot/vel= 1314.54 uu / 1.37 rad /  2886.90 uu/s
t=   5.00s  frames= 120  ball mean/max=   81.84/  659.64 uu  car mean pos/rot/vel= 4329.74 uu / 1.80 rad /  2796.35 uu/s
t=   6.00s  frames= 120  ball mean/max= 2001.30/ 3290.44 uu  car mean pos/rot/vel= 5625.86 uu / 1.99 rad /  1718.71 uu/s
t=   7.00s  frames= 120  ball mean/max= 4554.02/ 5673.98 uu  car mean pos/rot/vel= 7026.50 uu / 2.34 rad /  1854.51 uu/s
t=   8.00s  frames= 120  ball mean/max= 4897.99/ 5583.16 uu  car mean pos/rot/vel= 7813.11 uu / 1.97 rad /  1423.04 uu/s
t=   9.00s  frames= 120  ball mean/max= 3756.35/ 4277.88 uu  car mean pos/rot/vel= 7647.53 uu / 2.31 rad /  1201.30 uu/s
t=  10.00s  frames= 120  ball mean/max= 2922.14/ 3285.62 uu  car mean pos/rot/vel= 6728.06 uu / 2.14 rad /  3137.57 uu/s
t=  11.00s  frames= 120  ball mean/max= 2377.77/ 2602.75 uu  car mean pos/rot/vel= 6560.45 uu / 2.24 rad /  2719.04 uu/s
t=  12.00s  frames= 120  ball mean/max= 2269.64/ 2713.51 uu  car mean pos/rot/vel= 7429.03 uu / 2.34 rad /  2006.91 uu/s
t=  13.00s  frames= 120  ball mean/max= 3591.46/ 4448.78 uu  car mean pos/rot/vel= 8410.51 uu / 2.67 rad /  1849.93 uu/s
t=  14.00s  frames= 120  ball mean/max= 4449.55/ 4477.16 uu  car mean pos/rot/vel= 8411.24 uu / 2.80 rad /  2031.26 uu/s
t=  15.00s  frames= 120  ball mean/max= 4013.29/ 4309.90 uu  car mean pos/rot/vel= 7184.78 uu / 2.53 rad /  1827.48 uu/s
t=  16.00s  frames= 120  ball mean/max= 3242.05/ 3693.23 uu  car mean pos/rot/vel= 5800.41 uu / 2.91 rad /  1646.69 uu/s
t=  17.00s  frames= 120  ball mean/max= 2429.83/ 2773.42 uu  car mean pos/rot/vel= 4287.66 uu / 2.99 rad /  2181.36 uu/s
t=  18.00s  frames= 120  ball mean/max= 2202.96/ 2244.60 uu  car mean pos/rot/vel= 3138.45 uu / 3.10 rad /  2051.70 uu/s
t=  19.00s  frames= 120  ball mean/max= 2081.18/ 2158.41 uu  car mean pos/rot/vel= 2585.94 uu / 3.14 rad /   906.61 uu/s
t=  20.00s  frames= 120  ball mean/max= 1940.91/ 1990.24 uu  car mean pos/rot/vel= 2994.16 uu / 3.14 rad /   632.83 uu/s
t=  21.00s  frames= 120  ball mean/max= 1951.13/ 1971.75 uu  car mean pos/rot/vel= 3366.79 uu / 3.14 rad /   313.14 uu/s
t=  22.00s  frames= 120  ball mean/max= 2020.38/ 2079.41 uu  car mean pos/rot/vel= 3493.50 uu / 3.14 rad /    35.56 uu/s
t=  23.00s  frames=  58  ball mean/max= 2114.62/ 2151.14 uu  car mean pos/rot/vel= 3498.14 uu / 3.14 rad /     0.26 uu/s
```

23×120 + 58 = 2,818 frames total, matching `FR-077`'s own whole-run
`frames compared: 2818` exactly; the largest single window max
(`5673.98` uu at `t=7`) matches `FR-077`'s own whole-run `max ball
distance: 5673.98 uu` exactly — the two code paths agree, as the
single-window-reproduces-`score` unit test already guarantees
structurally.

**Interpretation: this is abrupt, not gradual.** Seconds 0–2 track the
recording almost perfectly (ball mean `~0.04–0.05` uu; car position mean
`~2.2–2.3` uu, rotation `~0.01` rad, velocity `<1` uu/s — a car sitting
motionless at its kickoff spawn with all-zero input is a trivial case to
match exactly). The car's own divergence starts climbing at `t=3`
(`33.81` uu) and explodes by `t=4` (`1314.54` uu, `1.37` rad,
`2886.90` uu/s) — a roughly 40x jump in one second — while the ball is
*still* essentially untouched (`0.05` uu mean) the whole time. Only at
`t=5` does the ball begin diverging too (`81.84`/`659.64` uu), peaking at
`t=7` (`5673.98` uu max) once the now-badly-diverged car reaches the
ball's vicinity and touches it differently than the recording did (or
doesn't touch it at all). After that the two trajectories fluctuate in a
persistently large but roughly bounded range (ball mean mostly
`2000–4900` uu, car position mean `2600–8400` uu) rather than continuing
to grow without bound — consistent with two now-chaotically-independent
trajectories bouncing around the same bounded arena, not a runaway
blowup. Rotation distance climbs to the hard cap `π` (`3.14` rad, see
`Quat::angle_to`'s range) by `t=19` and stays saturated there — the
candidate's orientation becomes fully decorrelated from the recording's.
This is exactly the "abrupt: a specific early mechanic mismatch derailing
the whole run" branch `RB-PHYSICS-001-FR-077`'s own Interpretation note
and this spec's Open Questions both flagged as the alternative to
gradual, evenly-distributed compounding error — and it's the one this
real run actually shows.

**What the recorded input was doing right then.** Reading `test2.jsonl`
directly around the derailment: the car sits completely stationary at its
kickoff spawn (all-zero input) until `t=3.433`, when throttle and then
boost engage and it accelerates diagonally toward the ball — explaining
why divergence is exactly zero before that instant (nothing to get wrong)
and only starts growing once the car actually starts moving. At
`t=4.133` the car (now moving at `~1130` uu/s) presses jump on the ground
and holds it for roughly `0.33` s (a long hold, engaging this port's own
variable-jump-height hold-acceleration, `RB-PHYSICS-001`'s `FR-015`).
While still ascending, at `t=4.317` — about `0.18` s after releasing the
first jump — a second jump press lands with `pitch=-1, roll=-1` held: a
**diagonal dodge** (this port's `drive::apply_input`, edge-triggered on
`input.jump && !jump_held`, correctly fires exactly once here, not
repeatedly, despite the second press itself being held for a further
`~0.14` s — ruling out a spurious repeated-trigger bug in this port's own
edge detection). The car's divergence from the recording explodes in
almost exactly this same window (`t=4`–`5`).
**Leading hypothesis, not yet isolated or confirmed**: this port's own
dodge implementation applies the flip's entire spin as a single
instantaneous angular-velocity kick (`drive.rs`: "a single instantaneous
spin kick, not a continuous torque"), while `RB-PHYSICS-001-FR-069`
already found and documented, but explicitly left unimplemented, that
real Rocket League's flip spin is a continuous per-tick torque applied
over a fixed `0.65` s window shaped by the real car's own inertia tensor
— a structurally different mechanism whose resulting orientation and
angular velocity would plausibly diverge sharply from an instantaneous
kick, especially compounded with `FR-078`'s own still-approximate
hitbox/inertia and the other already-documented steering/handbrake gaps
(`FR-065`/`FR-066`). This is a concrete, falsifiable next step for
`RB-PHYSICS-001-FR-005` to start from — replaying just this one dodge in
isolation from the same seed state and comparing this port's kick against
a properly time-integrated torque model — rather than a proven root
cause; no code has been changed based on this reading, and no other
candidate mechanic in this same window (the boost-charged approach, the
held first jump, or the ball touch itself) has been ruled out.

## Traceability

See [docs/traceability/TRACEABILITY.md](../../traceability/TRACEABILITY.md).

## Open questions

- What divergence threshold counts as "good enough" fidelity for Phase 1 to
  exit — a first real candidate-engine run now exists
  (`RB-PHYSICS-001-FR-077`: `frames compared: 2818, mean ball distance:
  2206.08 uu, ..., mean car position/rotation/velocity distance: 4508.71
  uu / 2.12 rad / 1421.73 uu/s, ...`), and `RB-VERIFY-003-FR-004`'s
  divergence-growth diagnostic has now actually run against that same
  capture (see Verification plan for the full per-window numbers and
  Interpretation): the divergence is **abrupt**, not gradual — near-zero
  for the run's first ~4 seconds, then a sharp derailment coinciding with
  a diagonal dodge maneuver, after which the trajectories fluctuate in a
  persistently large but roughly bounded range rather than growing
  further. That's real evidence of the run's *shape*, but it still
  doesn't by itself answer "what threshold": a single dodge maneuver is
  one data point, not a distribution across many runs/maneuvers a
  threshold could be calibrated against. This question stays open until
  `RB-PHYSICS-001-FR-005` has fixed enough of what this one run surfaced
  to produce a run whose divergence looks bounded rather than fully
  decorrelated. Applies to ball scoring, car scoring, and the
  timestamp-alignment tolerance (`rb_verify_cli::DEFAULT_MAX_TIMESTAMP_DELTA_SECS`)
  alike — all three are currently reasoned defaults, not empirically
  tuned ones.
- Whether `max_timestamp_delta_secs` should ever become adaptive (e.g.
  derived from each sequence's own observed average tick interval) rather
  than a single caller-supplied constant — not needed yet since no real
  candidate engine exists to expose a case where a fixed value is wrong.

## Change history

- 0.11.0 (2026-09-04): Ran `rb-verify --self-growth` for real against
  `RB-PHYSICS-001-FR-077`'s own real capture (`test2.jsonl`). The
  divergence is abrupt, not gradual: near-perfect for the run's first
  ~4 seconds, then a sharp derailment coinciding with a diagonal dodge
  maneuver (a held first jump followed by a second jump press with
  `pitch=-1, roll=-1`), after which ball and car distances fluctuate in a
  persistently large but roughly bounded range. Leading (not yet
  isolated/confirmed) hypothesis: this port's instantaneous dodge-spin
  kick vs. `RB-PHYSICS-001-FR-069`'s already-documented, unimplemented
  continuous flip torque. See Verification plan for the full per-window
  numbers and reasoning, and Open Questions for what this does and
  doesn't resolve. No code change — a reading of real data, not a fix.
- 0.10.0 (2026-09-04): `RB-VERIFY-003-FR-004` implemented — a new
  `rb_domain::divergence::score_windows` partitions the same
  nearest-timestamp-matched pairs `score` uses into consecutive
  `window_secs`-wide time buckets and scores each one independently
  (`score` and `score_windows` now share a private `matched_pairs`/
  `score_pairs` pipeline, so they can't silently drift apart); a new
  `rb_verify_cli::score_capture_growth` and `rb-verify --self-growth`
  CLI mode expose it. 4 new `rb_domain::divergence` tests (14 total), 3
  new `rb_verify_cli` tests (9 total). Manually run once against the
  synthetic capture fixture, confirming the CLI mode runs end-to-end; the
  real capture run this diagnostic actually exists for is still pending
  the owner's own machine (see Verification plan).
- 0.9.0 (2026-09-04): Scoped `RB-VERIFY-003-FR-004`, a divergence-growth
  diagnostic — a windowed variant of `score` (`score_windows`) reporting
  divergence within successive time slices of a single run, plus a new
  `rb-verify --self-growth` CLI mode, needed to tell whether `FR-077`'s
  real-run divergence is gradual or abrupt before `RB-PHYSICS-001-FR-005`
  calibrates against it. See Requirements for the full design. Not yet
  implemented — no code change.
- 0.8.0 (2026-09-04): Recorded `RB-PHYSICS-001-FR-077`'s real-capture run
  — this spec's own "good enough" Open Question now has a first real
  number to react to, but the number itself (consistent with near-total
  trajectory divergence) doesn't resolve the question; folded into that
  Open Question's own text rather than kept as a separate bullet. No code
  change.
- 0.7.0 (2026-09-03): Noted that the candidate engine this spec's own
  "good enough" fidelity question depends on is now implemented (not just
  scoped) as `RB-PHYSICS-001-FR-076`/`FR-077`, but still hasn't produced a
  real score — that run remains pending a real capture environment. No
  code change here.
- 0.6.0 (2026-09-02): Noted that the candidate engine this spec's own
  "good enough" fidelity question depends on is now scoped (not
  implemented) as `RB-PHYSICS-001-FR-076`/`FR-077` — see that spec's
  Requirements. No code change.
- 0.5.0 (2026-09-02): Re-ran the existing end-to-end pipeline (no code
  change) against a real BakkesMod capture for the first time, now that
  `RB-VERIFY-002-FR-001`'s plugin exists — `frames compared: 343`, real
  numbers on both real inputs, closing `PHASE-0-EXIT`'s own literal exit
  criterion. See Verification plan for the full run and why the score
  itself still isn't a fidelity measurement.
- 0.4.0 (2026-08-28): FR-003 implemented — `score` gains a required
  `max_timestamp_delta_secs` parameter and now aligns frames by nearest
  timestamp (an `O(n+m)` merge over both sequences' existing chronological
  order) instead of list index; a match outside tolerance is skipped, not
  force-matched. `frames_compared`'s meaning changes accordingly (see Data/
  state and invariants). `rb_verify_cli` gains
  `DEFAULT_MAX_TIMESTAMP_DELTA_SECS` and an optional third CLI argument to
  override it. Fixed a real timestamp bug this surfaced in
  `rb_capture_ingest`'s synthetic fixture (see Verification plan). 2 new
  unit tests in `rb_domain::divergence` (10 total); one existing test
  (`mismatched_lengths_compare_only_the_overlap`) was replaced since its
  premise — sequence length alone caps how many frames compare — no
  longer holds under nearest-timestamp matching.
- 0.3.0 (2026-08-28): FR-002 implemented — `DivergenceScore` gains a
  `cars: CarDivergence` field (mean/max position, rotation, velocity
  distance, pairs compared); cars are matched between sequences by
  `player_id` within each frame pair. Added `Quat::angle_to` (rotation
  distance, radians) to `rb_domain::state`. 8 new unit tests (4 in
  `rb_domain::divergence`, 3 for `angle_to`, plus `rb_verify_cli`'s
  output updated to print car stats). Manually re-run end-to-end; see
  Verification plan for the real numbers.
- 0.2.0 (2026-08-28): `rb_verify_cli::score_replay_against_capture` wires
  `score` to the real ingestion adapters (composition root factored out of
  `main.rs` into a testable `lib.rs`). 3 new tests; manually run
  end-to-end against a real replay fixture + synthetic capture fixture,
  proving the pipeline runs without erroring. Explicitly not a fidelity
  measurement yet — see Verification plan.
- 0.1.0 (2026-08-28): Initial draft; core ball-position scoring implemented
  and tested at bootstrap.
