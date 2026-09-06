# Fixtures

## `example.capture.jsonl`

A **synthetic, hand-authored** capture file (5 ticks: a car driving toward
the ball, boosting, then jumping and pitching into a dodge) in the JSON
Lines format ADR-0005 decided. Used only to unit-test
`rb_capture_ingest`'s parser against every field the format defines
(including a car's `input`, which is always present in a real capture,
unlike `rb_replay_ingest`'s frames).

Timestamps start at `11.78` (not `0.0`) so this fixture actually overlaps
`rb_replay_ingest`'s vendored replay fixture's real timeline —
`rb_verify_cli`'s tests and manual verification runs score the two
against each other, and `rb_domain::divergence::score`'s timestamp-
tolerant alignment (`RB-VERIFY-003-FR-003`) can only produce a non-empty
result if their timestamps genuinely overlap. `11.78`s is roughly when
the vendored replay's ball actually spawns (frames before that are
omitted — see `rb_replay_ingest/fixtures/README.md`); an earlier version
of this fixture started at `0.0` and silently never overlapped it at all,
undetected until real timestamp alignment landed (see `RB-VERIFY-003`'s
change history).

- **Not a real BakkesMod capture.** No such file exists yet: the
  BakkesMod-side plugin that would write one (`RB-VERIFY-002-FR-001`) has
  not been built — this sandboxed environment has no Rocket League,
  BakkesMod, or Windows to build/run it on (the same practical blocker
  documented for `RB-RESEARCH-O002`). Unlike `rb_replay_ingest`'s vendored
  third-party fixture (a real replay, just not the owner's own), there is
  no real capture file of any provenance to vendor here at all.
- **What it does *not* satisfy**: `RB-VERIFY-002`'s acceptance criteria
  call for a capture recorded from a real local/offline match, cross-checked
  against BakkesMod's own debug overlay at a manually-verified timestamp.
  This fixture only proves `rb_capture_ingest`'s JSON-Lines parsing logic is
  correct against the format's own schema — it says nothing about whether
  that schema matches what a real BakkesMod plugin would actually produce,
  since none has been written yet.

## `dodge-derailment.capture.jsonl`

A **real, 347-frame excerpt** of the owner's own real BakkesMod capture
from `RB-VERIFY-002-FR-001` (the same file `RB-PHYSICS-001-FR-077`'s own
real-capture run used, elsewhere referred to as `test2.jsonl`) — frames
`t=4.117s` through `t=7.0s`, covering the exact ground jump and diagonal
dodge `RB-PHYSICS-001-FR-079`'s investigation identified as the whole
run's abrupt-derailment trigger. The first frame is deliberately the last
grounded, neutral instant before the jump begins, so `rb_verify_cli`'s own
`is_grounded_and_neutral` seed-frame heuristic selects it immediately —
seeding a candidate simulation here isolates this one maneuver from the
whole run's own ~4 seconds of otherwise near-perfectly-tracked prior
simulation. See `RB-PHYSICS-001-FR-079`'s spec entry for the full
evidence chain this fixture backs, and `rb_verify_cli::tests::isolated_replay_of_the_real_dodge_still_diverges_sharply`
for the regression test built on it.

## `throttle-jump.capture.jsonl`

A **real, 558-frame excerpt** (`t=3.550s` through `t=8.192s`) of the
owner's second capture session's `groundjumpthrottle03` clip
(`RB-PHYSICS-001-FR-085`): a standing start, a throttle-only drive down
the field, a `0.18` s tapped jump, its landing, and the start of a second,
held jump. The first frame is the still, neutral instant before the
throttle comes on, so `is_grounded_and_neutral` seeds there. This is the
flat-floor baseline: the port tracks it to `3.3` uu mean over the whole
excerpt (see `rb_verify_cli::tests::isolated_replay_of_a_real_throttle_drive_with_two_jumps_tracks_to_a_few_uu`).

## `boost-wall-entry.capture.jsonl`

A **real, 271-frame excerpt** (`t=26.850s` through `t=29.100s`) of the
same session's `walldrive04` clip: a car already at the `2300` uu/s cap
with throttle held and boost just released (the recording holds exactly
`2297` for the full straight, so no drag acts above `1410` while the
throttle is down), boost re-engaged at `28.608`, and the car driven
straight up the `+X` wall's floor curve. Seeds on its first frame (boost
off, throttle on, flat). The straight tracks to a uu; the curve is
`RB-PHYSICS-001-FR-085`'s open finding F, and where the excerpt's `64` uu
max sits (see `rb_verify_cli::tests::isolated_replay_of_a_real_boost_run_into_the_wall_curve_stays_under_its_recorded_divergence`).

## `airborne-hit.capture.jsonl`

A **real, 517-frame excerpt** (`t=5.600s` through `t=9.900s`) of the same
session's `hittickjump01b` clip: a still car, throttle then boost, a
jump pressed `7` ticks before the car meets the ball in the air (wheels
already off — the jump-before-hit control `RB-PHYSICS-001-FR-084` finding
4 asked for, not the wheels-down hit-tick jump it still needs), the hit,
and the ball's flight toward the `+Y` goal, cut just before it enters the
goal mouth (where the recording and the port part ways by `~1000` uu —
finding K, open). Seeds on its first frame. Car and ball both track to a
few uu (see `rb_verify_cli::tests::isolated_replay_of_a_real_airborne_hit_tracks_car_and_ball_to_a_few_uu`).

The whole clips these three are cut from (`groundjumpthrottle03`,
`walldrive04`, `hittickjump01`/`hittickjump01b`, `curverun05`,
`onewheellanding06`) are kept untrimmed under `raw/` (see its own
README; no test reads them); `RB-PHYSICS-001-FR-085`'s entry
records what each showed, including the two capture-side defects
(analog axes recorded as zero throughout `hittickjump01`/`01b`; a dodge
at `t=30.175` in `walldrive04` with no `jump` press recorded at all).
