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
off, throttle on, flat). The straight tracks to a uu; the curve was
`RB-PHYSICS-001-FR-085`'s finding F (a `64` uu max, the recording shedding
`~380` uu/s the port did not) until `RB-PHYSICS-001-FR-086` measured the
fillet at `270` uu and dropped the pushback's positional term — `1.0` uu
mean, `8.4` uu max since (see `rb_verify_cli::tests::isolated_replay_of_a_real_boost_run_into_the_wall_curve_stays_under_its_recorded_divergence`).

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

## `hit-tick-jump.capture.jsonl`

A **real, 492-frame excerpt** (`t=43.808s` through `t=47.900s`) of the
owner's third capture session's `hittickjump01` clip — the first
recorded with the rebuilt plugin (`RB-VERIFY-002` 0.5.0) — cut around
the cleanest of three real wheels-down hit-tick jumps a geometric probe
of the recorded poses found in that clip: the ball starts at rest, the
car presses jump `3` ticks before contact with all four wheels still
raycasting as down, and the excerpt runs long enough to show the
resulting arc without running into the recording's next, unrelated
aerial hit. Seeds on its first grounded, neutral frame.
`RB-PHYSICS-001-FR-087` used this fixture (and the other two hits,
excerpted only for the investigation, not vendored) to close
`RB-PHYSICS-001-FR-084` finding 4: the post-hit suspension excess that
finding measured (`+67` uu/s) is gone, down to `+7` here — see
`rb_verify_cli::tests::isolated_replay_of_a_real_wheels_down_hit_tick_jump_tracks_within_its_recorded_arc`.

The whole clips these three are cut from (`groundjumpthrottle03`,
`walldrive04`, `hittickjump01`/`hittickjump01b`, `curverun05`,
`onewheellanding06`) are kept untrimmed under `raw/` (see its own
README; no test reads them); `RB-PHYSICS-001-FR-085`'s entry
records what each showed, including the two capture-side defects
(analog axes recorded as zero throughout `hittickjump01`/`01b`; a dodge
at `t=30.175` in `walldrive04` with no `jump` press recorded at all).

## `wall-climb-crest.capture.jsonl`

A **real, 824-frame excerpt** (`t=13.142s` through `t=20.000s`) of the
owner's fourth capture session's `wall_curve02` clip (recorded with
plugin 1.1): a full-boost floor-to-wall curve into the `-X` wall
followed by a long, flat, throttle-held climb all the way up to the
wall-to-ceiling fillet, over the crest, a flip past upside-down, wheel
detachment, and the fall back into the arena. Seeds on its first
grounded, neutral frame. `RB-PHYSICS-001-FR-088` (open) used this
fixture to characterize a residual left by `RB-PHYSICS-001-FR-086`: the
port sheds less speed than gravity alone predicts while climbing the
flat wall (the recording matches gravity almost exactly there), so it
reaches the ceiling fillet, crests, and detaches somewhat later than the
recording — a timing lag that compounds over the full ~7-second arc into
a `169` uu mean / `749` uu max position divergence, even though the
qualitative maneuver (curve in, climb, cross the fillet, invert, detach,
fall) matches — see
`rb_verify_cli::tests::a_real_wall_climb_through_the_ceiling_fillet_diverges_the_open_fr_088_residual`.
The whole clip is kept untrimmed under `raw/` as `wall_curve02.jsonl`
(four more wall-climb events in it are not yet excerpted). A later
re-trace ruled the flat-span sub-`g` decay itself out as a bug — it's
`FR-058`'s own already-validated throttle taper, exact on both sides —
and instead localized the real gap to the clip's own initial full-boost
floor-to-wall curve entry, where the port sheds `≈24%` more speed than
the recording over the same transition; `boost-wall-entry`'s own
equivalent curve shows the same asymmetry, just far smaller (`≈9%`).

## `goal-shot-net-entry.capture.jsonl`

A **real, 550-frame excerpt** (`t=8.817s` through `t=13.4s`) of the
owner's fifth capture session's `goal_shot03` clip (recorded with
plugin 1.1): a still car drives into a stationary ball, sending it on a
parabolic arc through the `+Y` goal mouth into the real net. Seeds on
its first grounded, neutral frame. `RB-PHYSICS-001-FR-089` (implemented)
used this fixture to find and fix `RB-PHYSICS-001-FR-085` finding K: the
ball wasn't diverging at the net at all — it never reached the net,
because the back wall's own floor-seam curved fillet (rounding the seam
between the floor and the solid part of the back wall) had no idea the
goal window (`standard_goal_walls`) cuts a hole straight through that
same wall, so a ball flying through the open goal mouth still collided
with the fillet as if the wall behind it were solid. Carving a
`GOAL_HALF_WIDTH`-wide cutout into the fillet lets the ball pass through
untouched and reach the real net mesh; ball divergence over this excerpt
drops `494.0 → 199.3` uu mean, `1351.6 → 1037.2` uu max — see
`rb_verify_cli::tests::a_real_ground_shot_now_reaches_and_tangles_in_the_goal_net_instead_of_a_phantom_fillet`.
The remaining gap is the net mesh's own already-acknowledged chaos/
uncalibrated-constant residual (`RB-PHYSICS-001-FR-033`'s own
Non-goals), not this bug. The whole clip is kept untrimmed under `raw/`
as `goal_shot03.jsonl` (it also records a second, slow rolling shot into
the opposite goal, not excerpted).

## `clean-dodge.capture.jsonl`

A **real, 279-frame excerpt** (`t=6.275s` through `t=8.6s`) of the
owner's sixth capture session's `clean_dodge04` clip: a car on flat
ground, no wall nearby, ground-jumps then dodges left with a pure
stick-axis-aligned input (`yaw = -1`, `pitch = 0`, `roll = 0` — no
diagonal component at all), then lands. Seeds on its first grounded,
neutral frame. `RB-PHYSICS-001-FR-090` (open) used this fixture — and a
second, independent dodge in the same clip (a mirrored right dodge,
`yaw = +1`, with a different pre-dodge spin) that reproduced the exact
same numbers, only sign-flipped — to isolate `RB-PHYSICS-001-FR-083`'s
long-standing dodge residual far more cleanly than any earlier fixture:
decomposed into the car's own body frame (`rotation.conjugate().rotate
(angular_velocity)`), the real flip's angular velocity splits between
the local forward and right axes (`4.09` / `3.67` rad/s, consistently,
regardless of the car's pre-dodge spin) even though the stick input is
purely axis-aligned, while the port's flip torque — implemented and
verified as a pure single-axis kick — lands entirely on local forward
(`5.50` / `0.00`). Car divergence over this excerpt: `351.0` uu mean
position (max `706.7`), `0.44` rad mean rotation (max `1.46`) — see
`rb_verify_cli::tests::a_real_clean_ground_dodge_diverges_the_open_flip_torque_axis_residual`.
The clip's other 8 dodges (2 traced for this finding, 6 unexcerpted) and
the mechanism behind the exact `4.09`/`3.67` split remain open. The
whole clip is kept untrimmed under `raw/` as `clean_dodge04.jsonl`.
