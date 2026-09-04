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
