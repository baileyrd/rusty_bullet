//! Driven car input: couples `rb_domain::ControllerInput` into forces and
//! torques on a car `RigidBody`. Ground-driving (throttle, steering),
//! boost, handbrake/drift, a single ground jump, and air control in this
//! increment. Throttle, steering, handbrake, and jump are gated on the car
//! actually touching the ground (a free-floating box has no wheels to
//! grip, lock, or push off of, so airborne input does nothing for any of
//! them here); boost is not gated the same way — it's a rocket, not an
//! engine, so it still fires with no ground contact at all, unlike every
//! grounded-only input above. It isn't identical airborne, though: since
//! `RB-PHYSICS-001-FR-056`, its own acceleration magnitude is higher
//! airborne than grounded (`BOOST_ACCELERATION_AIR` vs
//! `BOOST_ACCELERATION_GROUND`), matching real Rocket League's own real
//! split — a claim this doc comment used to get wrong by rounding "not
//! gated on ground contact" up to "identical everywhere". Air control is
//! the mirror image of the *gating*, not the magnitude question above:
//! gated on the car *not* touching the ground (real air control needs no
//! wheels at all — it's pure torque, so it would be redundant with
//! steering while grounded).
//!
//! Handbrake is modeled as a temporary ground-friction reduction rather
//! than a separate lateral-slip system: this port has no per-wheel tire
//! model (the car is one rigid box), so there's no distinct "rear grip"
//! to lose the way a real car's handbrake works. Instead, while `handbrake`
//! is held and the car is grounded, `RigidBody.friction` — the same
//! material property the ground-contact solver already reads for Coulomb
//! friction — is temporarily reduced, letting the box's existing momentum
//! carry it into a slide instead of gripping the ground and turning
//! cleanly. Releasing handbrake restores the car's original friction. This
//! reuses machinery the solver already has rather than inventing a second
//! grip model. Since `RB-PHYSICS-001-FR-066`, real Rocket League's own
//! handbrake friction reduction is confirmed genuinely anisotropic (a
//! separate, much milder reduction to forward/backward grip than to
//! sideways grip) rather than the single uniform multiplier this port
//! applies to both — see `HANDBRAKE_FRICTION_MULTIPLIER`'s own doc
//! comment for the full finding and why it isn't adopted.
//!
//! Jump is a single, fixed-height vertical impulse fired on the *rising
//! edge* of `ControllerInput.jump` (a fresh press, not merely "held") while
//! grounded — holding the button through the resulting airborne period
//! doesn't re-fire it, and releasing then re-pressing while still airborne
//! doesn't fire it either (this increment has no double jump to grant).
//! Edge detection needs one bit of state to remember "was jump held as of
//! last step," carried by the caller (`PhysicsWorld::car_jump_held`) and
//! passed in as `jump_held`, the same pattern `boost_amount` already uses
//! for a resource that must persist across calls.
//!
//! Pitch, yaw, and roll each apply an angular acceleration about one of the
//! car's three local axes (right, up, forward respectively), scaled
//! directly by the analog `Option<f32>` value — `None` (an input source
//! that can't recover an analog value, e.g. replay-derived input, see
//! `rb_domain`) is treated as zero, same as a centered stick. Unlike ground
//! steering, air control isn't speed-scaled: a car can spin freely from a
//! standing start in the air, since there's no wheel grip to require
//! momentum for. Since `RB-PHYSICS-001-FR-068`, the three axes don't share
//! one flat magnitude: `AIR_CONTROL_PITCH_TORQUE`/`AIR_CONTROL_YAW_TORQUE`/
//! `AIR_CONTROL_ROLL_TORQUE` are RocketSim's own confirmed real
//! `CAR_AIR_CONTROL_TORQUE = Vec(130, 95, 400)` values directly (pitch-yaw-
//! roll order), not this port's own placeholders. And since
//! `RB-PHYSICS-001-FR-079`, these apply as a direct angular-acceleration
//! rate (`RigidBody::apply_angular_acceleration`, scaled by RocketSim's own
//! real `CAR_TORQUE_SCALE`), not a torque divided by this car's own moment
//! of inertia the way `apply_torque` would — see `AIR_CONTROL_PITCH_TORQUE`'s
//! own doc comment for why real Rocket League's own source deliberately
//! cancels that division for this specific mechanism, and this port's
//! earlier model didn't. Since `RB-PHYSICS-001-FR-071`, real air control's
//! own per-axis angular-velocity **damping** applies every airborne step
//! too: each body-axis component of the spin bleeds at
//! `AIR_CONTROL_PITCH_DAMPING`/`AIR_CONTROL_YAW_DAMPING`/
//! `AIR_CONTROL_ROLL_DAMPING` (`30`/`20`/`50`, RocketSim's own
//! `CAR_AIR_CONTROL_DAMPING`) through the same `CAR_TORQUE_SCALE`, the
//! pitch and yaw terms scaled by `1 - |stick|` (so a held stick meets no
//! resistance; roll's isn't) — the mechanism that makes an unsteered
//! airborne car stop tumbling, and the one real capture's post-flip spin
//! decay to the recording's own rounding (see `AIR_CONTROL_PITCH_DAMPING`).
//! Since `RB-PHYSICS-001-FR-057`, sustained air control (or a dodge's own
//! flip torque) can no longer spin a car arbitrarily
//! fast, though: `clamp_angular_speed` caps the result at
//! `MAX_CAR_ANGULAR_SPEED`, a real confirmed Rocket League limit, once per
//! step, the same way `MAX_CAR_SPEED` already bounds linear speed.
//!
//! Double jump reuses the ground jump's own rising-edge detection
//! (`jump_pressed`) rather than a second edge-detector: the same fresh
//! press, while airborne, fires one more instantaneous impulse — gated on a
//! per-car `double_jump_available` flag instead of on `on_ground`. Landing
//! (any step where `on_ground` is true) unconditionally restores
//! availability; an airborne fresh press consumes it, so it can fire at
//! most once per airborne period no matter how many times jump is released
//! and re-pressed after that. That impulse is either a plain vertical
//! `JUMP_SPEED` kick, or a directional **dodge**, depending on the car's
//! `pitch`/`roll` stick input at the moment of the press: if either exceeds
//! `DODGE_DEADZONE`, a dodge fires instead — a purely horizontal
//! `DODGE_SPEED` impulse (along the car's *flattened*, horizontal forward
//! for pitch and horizontal right for roll — RocketSim's own
//! `forwardDir2D`/`rightDir2D`, since `RB-PHYSICS-001-FR-081` finding 2;
//! the tilted 3D axes before that) plus, since `RB-PHYSICS-001-FR-080`,
//! the real continuous flip
//! torque about the perpendicular axis (`right_axis` for pitch at
//! `FLIP_TORQUE_Y`, `forward_axis` for roll at `FLIP_TORQUE_X`) for
//! `FLIP_TORQUE_TIME` seconds, tracked per car as a `DodgeFlip` — the same
//! axis/sign conventions air control's own pitch/roll torque already uses,
//! so a forward dodge looks like a fast version of a forward air-control
//! pitch. Since `RB-PHYSICS-001-FR-079`, those conventions are
//! real Rocket League's own, not this port's: `pitch = -1` (stick forward)
//! is a forward flip translating along `+forward_axis` and spinning about
//! `+right_axis` (nose down first), and `roll`/`yaw = -1` a left dodge
//! translating along `-right_axis` and spinning about `+forward_axis` —
//! matching RocketSim's `dodgeDir = (-pitch, yaw + roll)` and
//! `flipRelTorque = (-dodgeDir.y, dodgeDir.x)` symbol-for-symbol, where
//! this port previously used its own inverted pitch sign (and an inverted
//! roll spin). Since `RB-PHYSICS-001-FR-073`, the roll axis's own
//! stick value also includes `yaw` input (`roll + yaw`, each clamped to
//! `[-1.0, 1.0]` individually first) — matching RocketSim's own confirmed
//! `dodgeDir = (-pitch, yaw + roll, 0)`, so a yaw-only press (no roll held)
//! now fires a sideways dodge too, the same as a real Rocket League player
//! nudging the right stick purely left/right. Since `RB-PHYSICS-001-FR-075`,
//! `DODGE_DEADZONE`'s own trigger (`dodge_pitch.abs() > DODGE_DEADZONE ||
//! dodge_roll.abs() > DODGE_DEADZONE`) is confirmed the same decision as
//! RocketSim's own real cancellation check, once that fold-in is in place —
//! see `DODGE_DEADZONE`'s own doc comment for the full finding. Both pitch
//! and the combined roll/yaw can contribute at once (a diagonal dodge): since
//! `RB-PHYSICS-001-FR-072`, their combined
//! `(pitch, roll)` direction is normalized to unit length before scaling —
//! matching RocketSim's own confirmed real `dodgeDir.safeNormalized()`
//! step — so a diagonal dodge has the same total magnitude as an
//! axis-aligned one, not the larger, independently-summed magnitude a flat
//! per-axis sum would give; since `RB-PHYSICS-001-FR-074`, a near-axis-
//! aligned diagonal input additionally snaps to a pure single-axis dodge
//! (matching RocketSim's own post-normalization small-component zeroing)
//! instead of leaving a tiny, likely-unintentional perpendicular
//! component — see `normalize_dodge_direction`'s own doc
//! comment for the full finding. Since
//! `RB-PHYSICS-001-FR-059`, though, `DODGE_SPEED`'s own magnitude is no
//! longer flat regardless of direction or current speed: a pitch dodge
//! opposing the car's current forward-velocity direction, or any side
//! (roll) dodge, scales up as current speed rises toward `MAX_CAR_SPEED`
//! — see `dodge_speed_scale`'s own doc comment for the confirmed real
//! ratios and `dodge_is_backward`'s for the backward classification.
//! A dodge is purely horizontal (no vertical component, unlike the plain double
//! jump) — real Rocket League's dodge impulse does have a small upward
//! component too, not modeled here beyond what `FLIP_Z_DAMP_120` implies.
//! `RB-PHYSICS-001-FR-069` confirmed real Rocket League's own dodge spin
//! to be a continuous torque applied every step for `FLIP_TORQUE_TIME`
//! (0.65s) with no decay, genuinely different between pitch and roll, and
//! `RB-PHYSICS-001-FR-080` adopted it in full: the dodge's own step applies
//! no spin at all; from the next step, `FLIP_TORQUE_X`/`FLIP_TORQUE_Y`
//! (inertia-cancelled via `apply_angular_acceleration`, without
//! `CAR_TORQUE_SCALE`, per-tick via RocketSim's own `tickTimeScale`) drive
//! the car to `MAX_CAR_ANGULAR_SPEED` within three ticks, where
//! `clamp_angular_speed` holds it until the window ends; pitch is locked
//! out of stick air control while the torque applies and for
//! `FLIP_PITCHLOCK_EXTRA_TIME` more (yaw and roll stay live — the real
//! capture's own in-window ticks demand it, against both RocketSim and
//! RLUtilities), and vertical
//! speed bleeds per `FLIP_Z_DAMP_120` between `FLIP_Z_DAMP_START` and the
//! window's end. Below `DODGE_DEADZONE` on both axes,
//! the plain vertical double jump fires exactly as before dodge existed.
//! Either way, the press still spends the one `double_jump_available` per
//! airborne period — a dodge and a plain double jump share the same
//! resource, matching real Rocket League. A dodge also leaves a per-car
//! `DodgeFlip` in progress, which **flip-cancel** below can scale down; a
//! plain double jump explicitly clears it instead (there's no flip to
//! cancel), and so does landing.
//!
//! Wall jump is a *third* jump variant, alongside the ground jump and the
//! double jump: a fresh press while airborne *and* touching an arena wall
//! (`wall_normal: Some(normal)`, computed by the caller the same way
//! `on_ground` is — see `PhysicsWorld`) fires an impulse combining
//! `WALL_JUMP_HORIZONTAL_SPEED` outward along the wall's normal with
//! `JUMP_SPEED` upward (the same vertical speed the ground jump and double
//! jump use). Touching a wall — whether or not jump is pressed that step —
//! unconditionally restores `double_jump_available`, the same way landing
//! does, matching real Rocket League's "any surface contact refills your
//! second jump" rule; wall jump itself doesn't separately consume or
//! restore it, since contact already did. On a fresh press, wall contact
//! takes priority over consulting `double_jump_available` at all (checked
//! first in the airborne branch), so a player can wall-jump and still have
//! a double jump left afterward.
//! Wall jump has no per-wall-contact limit of its own: touching a (new or
//! the same) wall again always allows another wall jump, unlike the
//! double jump's once-per-airborne-period limit. Since
//! `RB-PHYSICS-001-FR-067`, real Rocket League is confirmed to have no
//! distinct wall-jump mechanic or constant at all — it's the identical
//! grounded-jump impulse applied along the car's own up axis, which tips
//! to match a touched wall via the wheel/suspension system this port's box
//! car doesn't have; see `WALL_JUMP_HORIZONTAL_SPEED`'s own doc comment for
//! the full finding and why this port's two-component substitute isn't
//! adopted away.
//!
//! Wall jump can itself be dodged off of: the same `pitch`/`roll`-vs-
//! `DODGE_DEADZONE` check the ground double jump uses is applied on a wall
//! jump's own fresh press too. Below the deadzone, the plain fixed
//! outward-plus-upward impulse fires exactly as before this existed. At or
//! above it, a **wall-jump dodge** fires instead: the same
//! outward-plus-upward push combined with a horizontal `DODGE_SPEED`
//! impulse and the same real flip torque (identical axis/sign conventions
//! to the ground dodge), starting a fresh flip (`DodgeFlip`) just like a
//! ground dodge does. Unlike the plain
//! wall jump, a wall-jump dodge *does* consume `double_jump_available` —
//! the same resource a ground dodge spends — a deliberate simplification:
//! this port has no way to separately account for "a wall touch refilled
//! it, then the wall-jump dodge spent it" versus a genuinely independent
//! wall-dash resource, and real Rocket League's precise accounting here
//! isn't public to the precision this project would need to model that
//! distinction. Since touching a wall unconditionally restores
//! `double_jump_available` first (see above), a wall-jump dodge is never
//! blocked by an already-spent double jump — only its *stick input*, not
//! prior double-jump state, decides whether a wall-jump press dodges.
//!
//! The ground jump has variable height: continuing to hold `jump` after the
//! fresh press that fires it adds a continuous `JUMP_HOLD_ACCELERATION`
//! upward force, for up to `JUMP_HOLD_MAX_DURATION` seconds, on top of the
//! fixed `JUMP_SPEED` impulse — releasing early (or the window simply
//! running out) stops the extra acceleration, matching real Rocket League's
//! held-vs-tapped jump height difference. Since `RB-PHYSICS-001-FR-064`,
//! that release isn't *always* immediate: for the first `JUMP_MIN_TIME`
//! seconds after the press, the acceleration keeps applying regardless of
//! whether `jump` is still held (scaled down by `JUMP_PRE_MIN_ACCEL_SCALE`)
//! — real Rocket League's own `_UpdateJump` has this same mandatory
//! minimum-hold quirk, so even an instantaneous tap gets a small amount of
//! extra height. Only past that mandatory window does releasing `jump` end
//! it right away. This is scoped to the ground jump alone: the double jump,
//! a dodge, and the wall jump are still each a single fixed instantaneous
//! impulse, completely unaffected by how long jump is held, since firing
//! any of them requires releasing jump first (a fresh press), which itself
//! unconditionally ends the ground jump's hold window (see
//! `apply_driven_forces`'s own doc comment for the exact ordering). Tracked
//! per car via `jump_hold_time_remaining`, the same kind of caller-owned
//! persisted state `jump_held`/`double_jump_available` already are.
//!
//! A dodge's flip can be canceled — **flip-cancel** — by holding `pitch`
//! against it: since `RB-PHYSICS-001-FR-080` step (c), this is real Rocket
//! League's own mechanism (`RB-PHYSICS-001-FR-070`'s finding, read from
//! RocketSim's `_UpdateAirTorque`), not a jump-press trigger. Every step
//! the flip torque applies, if the flip's pitch-axis component
//! (`DodgeFlip::rel_torque.1`) is non-zero and `pitch` is held in the
//! *same* sign, that component alone is scaled by `1 - |pitch|` for that
//! step — a full deflection zeroes it, a half deflection halves it, and
//! releasing the stick restores it, for as long as `FLIP_TORQUE_TIME`
//! lasts; the roll-axis component is never touched, so a sideways
//! (roll-only) dodge can't be canceled at all, and a diagonal dodge under
//! a full cancel keeps rolling on its forward-axis component. Pitch stays
//! locked out of stick air control through the flip and its
//! `FLIP_PITCHLOCK_EXTRA_TIME` (yaw and roll are live throughout, cancel
//! or not), and the spin already built up is left to the real
//! air-control damping (`RB-PHYSICS-001-FR-071`) to bleed off, as in real
//! Rocket League — a fully cancelled forward flip stops gaining pitch rate
//! and then decays at `AIR_CONTROL_PITCH_DAMPING`'s own rate. Sign
//! convention: a forward flip (`pitch = -1` at the dodge) has
//! `rel_torque.1 = +1`, so pulling *back* (`pitch = +1`) cancels it, and
//! vice versa — the same input real players use. A second jump press
//! mid-flip does nothing (RocketSim: `hasFlipped` makes `canUse` false),
//! which the spent `double_jump_available` already models; this port's
//! former jump-press cancel (`RB-PHYSICS-001-FR-016`: a further press
//! zeroing `angular_velocity` outright) is gone, along with its
//! "any time before landing" window. Wall jump keeps priority on a fresh
//! press while touching a wall, unchanged. A plain double jump explicitly
//! clears the `DodgeFlip` rather than leaving it alone, so a stale flip
//! from an earlier dodge (kept alive across a wall touch, which restores
//! the double jump without ending the flip) can't keep applying torque
//! under a *later*, unrelated plain double jump.
//!
//! **No airborne self-righting.** This module used to apply a gentle
//! restoring torque nudging an unsteered airborne car's up axis toward
//! world up (`RB-PHYSICS-001-FR-018`'s landing auto-orientation assist,
//! an invented placeholder for "eventually right yourself before
//! landing"). `RB-PHYSICS-001-FR-060` then fetched RocketSim's real
//! `Car.cpp` and found real Rocket League has no single mechanic matching
//! it — its two closest systems, **auto-flip** (a turtle-recovery flip,
//! firing only on an actual jump press while touching a mostly-upright
//! surface with roll already past a threshold) and **auto-roll** (a torque
//! aligning the car to the ground's surface normal, only while throttle is
//! held with wheel contact), are both *grounded* and input-gated — and
//! `RB-PHYSICS-001-FR-071` retired the placeholder once the real
//! air-control damping was in place: what makes a tumbling airborne car
//! settle in real Rocket League is that its spin bleeds off, not that it
//! is steered upright, and the isolated real capture measured the same
//! with and without the nudge. Implementing auto-flip/auto-roll for real
//! would mean new grounded, input-gated state machinery this port doesn't
//! have — see `RB-PHYSICS-001-FR-060`'s own Non-goals.
//!
//! A car with no input set (or all-neutral `ControllerInput::default()`)
//! behaves exactly as a free rigid box always has — this module only ever
//! adds force/torque/impulse or adjusts the existing friction property,
//! never removes physics outright.
//!
//! This is not a Bullet3 port (Bullet has no concept of "a car's engine")
//! — it's this project's own model of Rocket League's driving mechanics,
//! since the real numbers are not public, sourced instead from the
//! community reverse-engineering effort (RocketSim, RLUtilities, and the
//! RLBot wiki's independently-converging "Useful Game Values" — see
//! `RB-PHYSICS-001-FR-031`'s audit for the full source-by-source
//! breakdown). `MAX_CAR_SPEED`, `UNBOOSTED_MAX_CAR_SPEED`, `MAX_BOOST`,
//! `BOOST_ACCELERATION_GROUND`/`BOOST_ACCELERATION_AIR` (since
//! `RB-PHYSICS-001-FR-056` split the single flat `BOOST_ACCELERATION` this
//! bullet used to name into the two distinct values the same sources
//! actually cite), `JUMP_SPEED`, `JUMP_HOLD_MAX_DURATION`,
//! `JUMP_HOLD_ACCELERATION`, (since `RB-PHYSICS-001-FR-057`)
//! `MAX_CAR_ANGULAR_SPEED`, and (since `RB-PHYSICS-001-FR-064`)
//! `JUMP_MIN_TIME`/`JUMP_PRE_MIN_ACCEL_SCALE` are commonly-cited,
//! multi-source-confirmed community-reverse-engineered approximations (the
//! same body of public research `PhysicsWorld::new`'s gravity constant
//! comes from);
//! `BOOST_CONSUMPTION_RATE` is a simplified constant standing in for Rocket
//! League's real boost-drain behavior; `THROTTLE_ACCELERATION`'s own peak
//! magnitude is likewise a simplified, uncalibrated placeholder, but since
//! `RB-PHYSICS-001-FR-058` it's no longer applied flat — `drive_speed_taper`
//! scales it by RocketSim's own confirmed real curve shape as speed rises,
//! tapering smoothly to zero at `UNBOOSTED_MAX_CAR_SPEED` instead of a hard
//! cutoff (see that function's own doc comment for why the curve's shape,
//! unlike its peak magnitude, transfers cleanly). `DODGE_SPEED` is real
//! end to end: since `RB-PHYSICS-001-FR-059` its per-direction scaling (a
//! backward dodge opposing current motion, or any side dodge, growing
//! stronger as current speed rises) matches RocketSim's own confirmed real
//! ratios via `dodge_speed_scale`, and since `RB-PHYSICS-001-FR-080` its
//! base magnitude is RocketSim's own `FLIP_INITIAL_VEL_SCALE` (`500`,
//! replacing a `1400` placeholder) with the backward dodge's real
//! `16/15` forward-axis factor added — a mass-independent velocity change
//! the "false precision" caveat never applied to, confirmed to `~1%` from
//! a real capture's own dodge tick. `STEER_TORQUE` itself remains an
//! uncalibrated placeholder, but since `RB-PHYSICS-001-FR-065` its own
//! `speed_factor` scale-up is confirmed to have the wrong *shape*, not
//! merely an uncalibrated magnitude — see that requirement's own entry and
//! `STEER_TORQUE`'s own doc comment for the full finding.
//! `HANDBRAKE_FRICTION_MULTIPLIER` itself likewise remains an uncalibrated
//! placeholder, but since `RB-PHYSICS-001-FR-066` its own single uniform
//! reduction is confirmed to have the wrong *shape* too — real Rocket
//! League applies a genuinely anisotropic (direction-dependent) reduction,
//! not this port's one isotropic factor — see that requirement's own
//! entry and `HANDBRAKE_FRICTION_MULTIPLIER`'s own doc comment for the
//! full finding. (The former `LANDING_AUTO_UPRIGHT_TORQUE` placeholder is
//! gone since `RB-PHYSICS-001-FR-071` — see the "No airborne
//! self-righting" section above.) `WALL_JUMP_HORIZONTAL_SPEED` remains an uncalibrated
//! placeholder too, but since `RB-PHYSICS-001-FR-067` real Rocket League is
//! confirmed to have no distinct wall-jump mechanic or constant to
//! calibrate against at all — see that requirement's own entry and
//! `WALL_JUMP_HORIZONTAL_SPEED`'s own doc comment for the full finding.
//! `AIR_CONTROL_PITCH_TORQUE`/`AIR_CONTROL_YAW_TORQUE`/`AIR_CONTROL_ROLL_TORQUE`
//! are, since `RB-PHYSICS-001-FR-068`, RocketSim's own confirmed real
//! absolute values directly, not placeholders — unlike
//! `STEER_TORQUE`/`HANDBRAKE_FRICTION_MULTIPLIER`/`WALL_JUMP_HORIZONTAL_SPEED`'s
//! own confirmed-but-not-adopted findings — since real air control turned
//! out to be the same *kind* of direct per-axis torque mechanism this port
//! already models, unlike those other three findings' own architecture
//! mismatches. `RB-PHYSICS-001-FR-079` went further: the mechanism itself
//! (how that torque is applied) also needed a fix — real Rocket League's
//! own source deliberately cancels the division by this car's own moment of
//! inertia that `apply_torque` would otherwise introduce, so these three
//! constants apply via `apply_angular_acceleration` and `CAR_TORQUE_SCALE`
//! instead — see that requirement's own entry and
//! `AIR_CONTROL_PITCH_TORQUE`'s own doc comment for the full finding.
//! The dodge's former `DODGE_ANGULAR_SPEED` placeholder kick is gone since
//! `RB-PHYSICS-001-FR-080`: `RB-PHYSICS-001-FR-069` had confirmed real
//! Rocket League's own dodge spin to be a continuous per-axis torque over a
//! fixed 0.65s window, and FR-080 adopted exactly that (`FLIP_TORQUE_X`/
//! `FLIP_TORQUE_Y` over `FLIP_TORQUE_TIME`, with the per-car `DodgeFlip`
//! state `RB-PHYSICS-001-FR-059`'s own Non-goals had once flagged as too
//! large a redesign) — see `FLIP_TORQUE_X`'s own doc comment.
//! `RB-PHYSICS-001-FR-031`'s audit
//! found real reference numbers for some of these (a dodge's real ~500
//! uu/s base impulse, which `RB-PHYSICS-001-FR-080` later adopted as
//! `DODGE_SPEED` once its mass-independence was recognized; a wall jump
//! reusing the plain jump impulse rather
//! than its own faster speed, confirmed exact by `RB-PHYSICS-001-FR-067`;
//! real air-control torque/damping coefficients, whose per-axis torque
//! ratio `RB-PHYSICS-001-FR-068` later confirmed and adopted and whose
//! damping `RB-PHYSICS-001-FR-071` adopted in full; a dodge's real
//! spin torque and duration, whose exact mechanism `RB-PHYSICS-001-FR-069`
//! later confirmed), but none of the
//! remaining raw absolute values port directly: they're expressed as
//! torques or velocity-dependent curves calibrated against real Rocket
//! League's own specific car mass/inertia tensor and mechanic shape,
//! neither of which this port's own placeholder car body or simplified
//! single-impulse mechanics are calibrated to match, so adopting the raw
//! numbers here would be false precision, not a real fix — see the
//! audit's own findings for detail. `MAX_CAR_ANGULAR_SPEED` (and, since
//! `RB-PHYSICS-001-FR-059`, `DODGE_SPEED`'s own per-direction scale
//! ratios) don't have that problem even though they also bound
//! rotation/velocity: they cap or scale the *result* (a rad/s or uu/s
//! quantity) rather than prescribing the torque or force that produces
//! it, so they transfer cleanly regardless of this port's own car
//! body/inertia tensor not matching real Rocket League's — see
//! `RB-PHYSICS-001-FR-057`'s own findings for why that distinction let
//! these constants clear the bar the torque-based placeholders above
//! couldn't. Aside from those exceptions, none of these are independently
//! confirmed by this project — see `RB-PHYSICS-001-FR-005`/`FR-031`.

use crate::body::RigidBody;
use rb_domain::{ControllerInput, Vec3};

/// Commonly-cited boosted top speed (uu/s), boost's own speed cap here —
/// confirmed against real Rocket League's own reverse-engineered constants
/// during `RB-PHYSICS-001-FR-031`'s audit (`CAR_MAX_SPEED = 2300.f` in the
/// RocketSim project's `RLConst.h`; matched independently by RLUtilities'
/// `Car::v_max` and the RLBot community wiki's "Useful Game Values" page).
/// Also used as the turning-torque scale-up reference in `speed_factor`
/// below — an arbitrary normalization choice, not a claim that a car's
/// actual turning grip caps out at boosted speed specifically.
/// `RB-PHYSICS-001-FR-065` found `speed_factor`'s own scale-up *direction*
/// against this reference doesn't match real Rocket League's own steering
/// model at all — see `STEER_TORQUE`'s own doc comment for the finding.
pub const MAX_CAR_SPEED: f32 = 2300.0;

/// Hard cap (rad/s) on a car's angular speed, enforced by
/// `clamp_angular_speed` once per step, right after
/// `integrate::integrate_velocities` — a genuine clamp that scales
/// `angular_velocity` back down if it's exceeded, unlike `MAX_CAR_SPEED`/
/// `UNBOOSTED_MAX_CAR_SPEED` above (which only gate *new* throttle/boost
/// force, never reduce velocity already past the cap). Confirmed exact
/// against RocketSim's own `RLConst.h` during `RB-PHYSICS-001-FR-057`'s
/// audit: `CAR_MAX_ANG_SPEED = 5.5f, // Car can never exceed this angular
/// velocity (radians/s)`.
///
/// Since `RB-PHYSICS-001-FR-080` this cap is also load-bearing for the
/// dodge: the real flip torque (`FLIP_TORQUE_X`/`FLIP_TORQUE_Y`) drives the
/// car's angular speed past it within three ticks, and this clamp holding
/// the *stored* angular velocity at the cap for the rest of
/// `FLIP_TORQUE_TIME` is what a real flip's reported `5.5` rad/s spin is —
/// RocketSim's own `_FinishPhysicsTick` clamp plays the same role there.
/// Since that requirement's step (c) the clamp runs after the transform
/// has integrated, as RocketSim's does, so the car actually *turns* by the
/// unclamped `|ω + Δω|` each tick (`≈7.6` rad/s mid-flip) — see
/// `clamp_angular_speed`. (This port's former `DODGE_ANGULAR_SPEED = 5.5`
/// placeholder, an instantaneous kick that happened to equal this cap by
/// coincidence, was removed by that same requirement.)
///
/// Enforced at the end of the step, after contact resolution and the
/// transform integration alike, so a same-step contact-solver impulse
/// (e.g. a hard collision imparting spin) is clamped in the same call — the
/// one-step transient `RB-PHYSICS-001-FR-057`'s original mid-pipeline
/// placement allowed is gone with the move.
pub const MAX_CAR_ANGULAR_SPEED: f32 = 5.5;

/// Commonly-cited *unboosted* top speed (uu/s) — throttle's own speed cap,
/// distinct from `MAX_CAR_SPEED`. Before `RB-PHYSICS-001-FR-031`'s audit,
/// throttle alone could push a car all the way to `MAX_CAR_SPEED` (2300),
/// which is real Rocket League's *boosted* cap, not its unboosted one; the
/// audit found a consistent, independently-corroborated real value (1410)
/// across RocketSim's `RLConst.h` — whose `DRIVE_SPEED_TORQUE_FACTOR_CURVE`
/// drives available drive torque to exactly zero at 1410 uu/s — and the
/// RLBot community wiki's "Useful Game Values" page, so throttle now caps
/// here instead.
pub const UNBOOSTED_MAX_CAR_SPEED: f32 = 1410.0;

/// Peak throttle acceleration (uu/s^2), at a standing start — still an
/// uncalibrated placeholder pending calibration against recorded data
/// (unlike the taper shape it's scaled by, see `drive_speed_taper` below,
/// this magnitude itself has no confirmed real-world source). Since
/// `RB-PHYSICS-001-FR-058`, this is no longer applied flat: it's scaled by
/// `drive_speed_taper`'s own real curve as speed rises, tapering smoothly
/// to zero at `UNBOOSTED_MAX_CAR_SPEED` instead of applying at full
/// strength right up to a hard cutoff.
const THROTTLE_ACCELERATION: f32 = 1600.0;

/// Real Rocket League's own speed-dependent drive-force taper —
/// RocketSim's `DRIVE_SPEED_TORQUE_FACTOR_CURVE`, a 3-point piecewise-
/// linear curve confirmed exact against its own `RLConst.h` during
/// `RB-PHYSICS-001-FR-058`'s audit: full torque from a standing start
/// (`(0, 1.0)`), tapering linearly down to 10% by `1400` uu/s
/// (`(1400, 0.1)`), then a final, much steeper linear drop to exactly
/// zero at `UNBOOSTED_MAX_CAR_SPEED` (`(1410, 0.0)`). Fetching RocketSim's
/// own `Car.cpp` confirmed this curve is looked up by the car's *signed*
/// forward speed (`abs()`'d there, since real RocketSim's own throttle
/// gate isn't direction-aware) and multiplied directly against the drive
/// force applied to each wheel — a pure, unitless ratio, unlike
/// `THROTTLE_TORQUE_AMOUNT` (RocketSim's own name for this project's
/// `THROTTLE_ACCELERATION`), which is expressed in Bullet's own internal
/// mass/distance units and doesn't transfer to this port's own
/// differently-calibrated car body the same clean way — see
/// `RB-PHYSICS-001-FR-031`'s and `FR-057`'s own "false precision" findings
/// for absolute torque/force magnitudes. Only the curve's *shape* is
/// adopted here, not a new peak magnitude.
const DRIVE_SPEED_TAPER_BREAKPOINTS: [(f32, f32); 3] =
    [(0.0, 1.0), (1400.0, 0.1), (UNBOOSTED_MAX_CAR_SPEED, 0.0)];

/// Linearly interpolates `DRIVE_SPEED_TAPER_BREAKPOINTS` at
/// `signed_speed_in_throttle_direction` (the same
/// `throttle.signum() * forward_speed` quantity `apply_driven_forces`'s
/// own throttle gate already computes) — `1.0` (full acceleration) at or
/// below the first breakpoint, `0.0` at or beyond the last. Deliberately
/// evaluated against this port's own pre-existing *signed*,
/// direction-aware speed (clamped to non-negative here, since a negative
/// value means "not yet moving this way," which should read as a
/// standing start, not an out-of-range lookup) rather than switching to
/// real RocketSim's own direction-agnostic `abs(forward speed)` — that
/// would be a second, independent behavioral change (whether accelerating
/// against your own current motion tapers too) this requirement doesn't
/// take on; see its own Non-goals.
fn drive_speed_taper(signed_speed_in_throttle_direction: f32) -> f32 {
    let speed = signed_speed_in_throttle_direction.max(0.0);
    let points = DRIVE_SPEED_TAPER_BREAKPOINTS;
    if speed <= points[0].0 {
        return points[0].1;
    }
    for window in points.windows(2) {
        let (x0, y0) = window[0];
        let (x1, y1) = window[1];
        if speed <= x1 {
            return y0 + (y1 - y0) * (speed - x0) / (x1 - x0);
        }
    }
    points[points.len() - 1].1
}

/// Uncalibrated placeholder steering torque magnitude (about the car's
/// local up axis, at full `steer` input and at/above `MAX_CAR_SPEED`) —
/// chosen only so a full-lock turn is visibly responsive for this car's
/// mass/inertia in tests, not derived from any measured or documented
/// Rocket League value.
///
/// `RB-PHYSICS-001-FR-065` fetched RocketSim's real `Car.cpp` (`_UpdateWheels`,
/// matching `RB-PHYSICS-001-FR-058`/`FR-059`/`FR-064`'s own
/// real-implementation-file method) and found real Rocket League's
/// steering isn't a direct yaw-torque model at all: a wheel's *steer
/// angle* (not a torque) is set from a confirmed real
/// `STEER_ANGLE_FROM_SPEED_CURVE` (`RLConst.h`, radians), and that angled
/// wheel's lateral tire friction — computed per-wheel by `btVehicleRL`, a
/// custom extension of Bullet's own raycast vehicle system
/// (`btDefaultVehicleRaycaster`), through a further confirmed
/// `LAT_FRICTION_CURVE` slip-friction curve — is what actually turns the
/// car. This port has no wheels, raycasting, or tire-slip model at all
/// (the car is one rigid box), so this real mechanism can't be ported
/// without a substantially larger architecture change, the same category
/// `RB-PHYSICS-001-FR-063` already established for per-contact-pair-type
/// restitution/friction.
///
/// One finding is still directly actionable even without that larger
/// change: the confirmed real curve's own *shape* is the opposite of this
/// port's own `speed_factor` below. Real Rocket League's maximum steering
/// angle is highest at a standstill (`0.53356` rad ≈ 30.6° at 0 uu/s) and
/// decreases sharply as speed rises (down to `0.03454` rad ≈ 2° at 3000
/// uu/s) — a car can turn tightest from a stop, only gently at speed. This
/// port's own `speed_factor` does the opposite: zero torque at a
/// standstill, scaling *up* to full `STEER_TORQUE` at `MAX_CAR_SPEED`.
/// Not adopted as a fix: unlike `RB-PHYSICS-001-FR-058`'s throttle taper
/// or `FR-059`'s dodge scale (direct multipliers on a force/impulse this
/// port already applies the same way real Rocket League does), the real
/// curve maps speed to a *wheel angle*, which real Rocket League then
/// feeds through nonlinear tire-slip friction (dependent on wheelbase
/// geometry and friction curves this port doesn't model at all) to
/// produce the actual turning force — there's no principled way to carry
/// even the curve's normalized shape onto this port's own direct-torque
/// model. Reversing `speed_factor`'s direction without that transfer
/// function would substitute one unconfirmed guess for another, not adopt
/// a confirmed real value — the same reasoning that keeps `FR-059`'s
/// `DODGE_SPEED` base magnitude a placeholder despite a real reference
/// existing for it. Air control's own equivalent magnitude was in this same
/// category until `RB-PHYSICS-001-FR-079`: once its application mechanism
/// was also fixed (see `AIR_CONTROL_PITCH_TORQUE`'s own doc comment), the
/// real reference transferred cleanly and is now adopted directly, unlike
/// `DODGE_SPEED`'s own case here.
const STEER_TORQUE: f32 = 1_500_000.0;

/// Boost acceleration while grounded (uu/s^2) — unlike throttle, boost
/// doesn't taper with speed in real Rocket League, so this (like
/// `BOOST_ACCELERATION_AIR` below) is a flat constant, not a curve.
/// Confirmed exact against RocketSim's own `RLConst.h`
/// (`BOOST_ACCEL_GROUND = 2975.f / 3.f`, fetched directly during
/// `RB-PHYSICS-001-FR-056`) — written as the same fraction the reference
/// uses, matching `JUMP_SPEED`'s own precedent for an exact fractional
/// source value, rather than that fraction's earlier `991.667` decimal
/// approximation (the two are equal to float precision; this is a
/// clarity change, not a value change).
const BOOST_ACCELERATION_GROUND: f32 = 2975.0 / 3.0;

/// Boost acceleration while airborne (uu/s^2) — genuinely different from
/// `BOOST_ACCELERATION_GROUND`, not a rounding of the same number.
/// `RB-PHYSICS-001-FR-056` fetched RocketSim's own `RLConst.h` directly
/// and found `BOOST_ACCEL_AIR = 3175.f / 3.f`, distinctly higher than the
/// grounded value — a split this port's own earlier single flat
/// `BOOST_ACCELERATION` constant didn't model at all (every airborne
/// boost this crate ever applied used the *grounded* number, understating
/// real airborne boost strength by about 6.5%). `apply_driven_forces`
/// now selects between the two by `on_ground`, matching the reference
/// split exactly — a genuine behavioral fix, not just a doc correction,
/// found via the same "fetch primary source directly" method this
/// project already applies throughout (see `RB-PHYSICS-001-FR-031`'s own
/// audit and every reference-validation FR since).
const BOOST_ACCELERATION_AIR: f32 = 3175.0 / 3.0;

/// Commonly-cited full boost tank size, in the same units `ControllerInput`
/// and `CarState::boost_amount` use.
pub const MAX_BOOST: f32 = 100.0;

/// Simplified constant boost drain rate (units/s) while `boost` is held —
/// a full tank lasting ~3 seconds nonstop is the commonly-cited number
/// this approximates; real Rocket League's actual drain behavior around
/// zero-throttle/wavedash edge cases isn't modeled.
const BOOST_CONSUMPTION_RATE: f32 = 33.3;

/// Uncalibrated placeholder: while grounded and `handbrake` is held, the
/// car's `RigidBody.friction` is multiplied by this factor before the
/// ground-contact solver runs, sharply reducing grip so existing momentum
/// carries the car into a slide instead of a clean turn. Chosen only to
/// produce a visibly reduced (not zero) grip in tests, not derived from any
/// measured or documented Rocket League value — this port has no per-wheel
/// tire model to calibrate a real rear-grip-loss number against.
///
/// `RB-PHYSICS-001-FR-066` fetched RocketSim's real `Car.cpp`
/// (`_UpdateWheels`, matching `RB-PHYSICS-001-FR-058`/`FR-059`/`FR-064`/
/// `FR-065`'s own real-implementation-file method) and found real Rocket
/// League's own handbrake friction reduction is genuinely anisotropic, not
/// a single uniform multiplier: two separate confirmed real curves,
/// `HANDBRAKE_LAT_FRICTION_FACTOR_CURVE` (`0.1` at every speed — this
/// value's own coincidental exact match to this port's own `0.1` is
/// striking but not a confirmation, see below) and
/// `HANDBRAKE_LONG_FRICTION_FACTOR_CURVE` (`0.5` at a standstill, `0.9` at
/// and above 1 uu/s — effectively a near-constant, barely-reduced `0.9`
/// for any real driving speed), are applied to lateral and longitudinal
/// tire friction independently. Real Rocket League's handbrake drift
/// keeps a car's forward/backward grip almost intact (`x0.9`) while
/// cutting sideways grip to a tenth (`x0.1`) — this port's own single
/// isotropic `RigidBody.friction` scalar, read identically by both of
/// `solver::friction_directions`' own two tangent rows, has no way to
/// apply a different factor to each direction without threading a second,
/// direction-specific friction coefficient through every one of
/// `solver.rs`'s several row-limit-computation call sites
/// (`resolve_contacts`, `resolve_contacts_between`,
/// `resolve_static_manifolds`, `resolve_dynamic_manifolds`,
/// `resolve_manifolds`) — a substantially larger architecture change than
/// this finding alone justifies, the same category
/// `RB-PHYSICS-001-FR-063`/`FR-065` already established. `0.1`'s own
/// coincidental match to the real lateral-only factor is exactly that: a
/// coincidence, since this port's uniform `0.1` also (wrongly) crushes
/// longitudinal grip to a tenth, where real Rocket League keeps it near
/// `0.9` — this port's own handbrake understates real forward-momentum
/// retention during a drift. Not adopted as a fix; left for a future,
/// dedicated requirement.
const HANDBRAKE_FRICTION_MULTIPLIER: f32 = 0.1;

/// Jump impulse speed (uu/s), applied as an instantaneous vertical
/// velocity change (not a continuous force) on a fresh grounded jump
/// press — a flat speed regardless of the car's mass, matching how the
/// real jump impulse doesn't scale with car mass either. Also reused as
/// the double jump's impulse magnitude (see the module doc comment) —
/// `pub` so `world.rs`'s end-to-end tests can assert against it directly,
/// the same way `MAX_CAR_SPEED`/`MAX_BOOST` already are. Refined from an
/// earlier `292.0` approximation to the precise value during
/// `RB-PHYSICS-001-FR-031`'s audit: RocketSim's `RLConst.h` defines
/// `JUMP_IMMEDIATE_FORCE = 875.f/3.f`, and RLUtilities independently
/// hardcodes `Jump::speed = 291.667f` — the same number both projects
/// also apply, unmodified, to the double jump.
pub const JUMP_SPEED: f32 = 875.0 / 3.0;

/// Real Rocket League's own air-control pitch torque (about the car's local
/// right axis) at full analog input, fetched directly from RocketSim's own
/// `RLConst.h`: `CAR_AIR_CONTROL_TORQUE = Vec(130, 95, 400)` ("Angle order
/// is PYR"). `RB-PHYSICS-001-FR-068` first adopted only this constant's
/// per-axis *ratio* onto this port's own then-uncalibrated pitch baseline
/// (`AIR_CONTROL_YAW_SCALE`/`AIR_CONTROL_ROLL_SCALE`), reasoning that the
/// real *absolute* value was calibrated against a specific real inertia
/// tensor this port's own placeholder car body wasn't confirmed to match —
/// the same "false precision" logic `RB-PHYSICS-001-FR-031` already applied
/// elsewhere.
///
/// `RB-PHYSICS-001-FR-079` found that reasoning itself rested on a false
/// premise: reading RocketSim's real `Car.cpp::_UpdateAirTorque` directly
/// showed it applies this torque as
/// `applyTorque(invInertiaTensorWorld.inverse() * (torque - damping) *
/// CAR_TORQUE_SCALE)` — pre-multiplying by the car's own *actual*
/// (non-inverted) inertia tensor specifically to cancel out Bullet's own
/// inverse-inertia multiply during integration (`angularVelocity +=
/// invInertia * appliedTorque * dt`). The two cancel exactly, so real
/// Rocket League's `CAR_AIR_CONTROL_TORQUE` is, by construction, already an
/// inertia-*independent* direct angular-acceleration input (scaled by
/// `CAR_TORQUE_SCALE`, see its own doc comment) — not a genuine physical
/// torque whose absolute magnitude depends on this car's own moment of
/// inertia at all. The "false precision" reasoning above was true for this
/// port's own then-current `apply_torque` mechanism (which *does* divide by
/// inertia, correctly matching real Bullet's own semantics per
/// `RB-PHYSICS-001-FR-046`) but not for real Rocket League's own mechanism,
/// which deliberately sidesteps that division. Confirmed quantitatively:
/// `95.0 * CAR_TORQUE_SCALE ≈ 9.109` rad/s² for full yaw input, matching a
/// real recorded car's own measured yaw acceleration (`≈9.12` rad/s²,
/// `RB-PHYSICS-001-FR-079`'s isolated-replay fixture) almost exactly —
/// computed purely from RocketSim's own real constants, with no reference
/// at all to this port's own (until now incorrect) model. All three axes
/// now apply via `RigidBody::apply_angular_acceleration` instead of
/// `apply_torque` — see the call sites below and
/// `total_angular_accel`'s own doc comment in `body.rs`.
///
/// Sign: the same `_UpdateAirTorque` applies this about `dirPitch_right =
/// -GetRightDir()` — the *negative* of the car's own right axis (and roll
/// about `dirRoll_forward = -GetForwardDir()`; only yaw's `dirYaw_up =
/// GetUpDir()` is unnegated). `RB-PHYSICS-001-FR-079` found this port had
/// applied both about the positive axes, inverting every recorded pitch
/// and roll input it replayed; the call sites below now negate the axis to
/// match, so the recorded stick convention (`pitch = -1` nose-down,
/// `roll = +1` roll-right) produces real Rocket League's own rotation.
const AIR_CONTROL_PITCH_TORQUE: f32 = 130.0;

/// Real Rocket League's own air-control yaw torque (about the car's local
/// up axis) — see `AIR_CONTROL_PITCH_TORQUE`'s own doc comment for the full
/// finding and citation. `CAR_AIR_CONTROL_TORQUE.y` in RocketSim's own
/// `RLConst.h`.
const AIR_CONTROL_YAW_TORQUE: f32 = 95.0;

/// Real Rocket League's own air-control roll torque (about the car's local
/// forward axis) — see `AIR_CONTROL_PITCH_TORQUE`'s own doc comment for the
/// full finding and citation. `CAR_AIR_CONTROL_TORQUE.z` in RocketSim's own
/// `RLConst.h`.
///
/// `RB-PHYSICS-001-FR-071` closes a thread `RB-PHYSICS-001-FR-068`'s own
/// Non-goals left open — RocketSim's `CAR_AIR_CONTROL_DAMPING = Vec(30, 20,
/// 50)`, which that requirement's own fetch of `_UpdateAirTorque` found but
/// didn't examine. The full mechanism: for each axis, real air control
/// subtracts a damping torque `(angular velocity along that axis) *
/// CAR_AIR_CONTROL_DAMPING[axis] * (1 - abs(analog input on that axis))`
/// from the applied torque *before* the same `invInertia.inverse() * ... *
/// CAR_TORQUE_SCALE` treatment `AIR_CONTROL_PITCH_TORQUE`'s own doc comment
/// describes — pitch's own input term additionally multiplies by
/// `pitchTorqueScale` (`RB-PHYSICS-001-FR-070`). Releasing the stick on an
/// axis (input `0`) gives full damping strength on that axis, continuously
/// bleeding off any existing spin; holding it fully (input `±1`) zeroes the
/// damping, granting full torque authority with no resistance. Since
/// `RB-PHYSICS-001-FR-071`'s implementation that mechanism is adopted in
/// full — see `AIR_CONTROL_PITCH_DAMPING` and `air_control_damping`.
/// (`RB-PHYSICS-001-FR-079` had already found the original "false
/// precision" non-adoption reasoning rested on a false premise: `damping`
/// sits inside the same `torque - damping` expression the inertia
/// pre-multiply/cancel applies to, so it's inertia-independent too.)
const AIR_CONTROL_ROLL_TORQUE: f32 = 400.0;

/// Real Rocket League's own air-control damping coefficient about the car's
/// local *right* axis (pitch rate), RocketSim's `CAR_AIR_CONTROL_DAMPING.x
/// = 30` (`RB-PHYSICS-001-FR-071`). Every airborne step, `air_control_damping`
/// subtracts `(angular velocity along the axis) * coefficient * (1 - |stick
/// input on the axis|)` for each axis — as an inertia-independent angular
/// acceleration through `CAR_TORQUE_SCALE`, the same `torque - damping`
/// expression `AIR_CONTROL_PITCH_TORQUE` lives in — so a released stick
/// bleeds existing spin off continuously (pitch at `e^{-30 · 0.0959 · t}`,
/// a `0.35` s time constant) while a fully held one meets no resistance.
/// The pitch input in that factor is the *effective* pitch — zero while
/// `RB-PHYSICS-001-FR-080`'s pitch lock holds, so a post-flip pitch stick
/// doesn't suppress the pitch damping. Active during the flip too:
/// `FR-080` step (c) reproduced the isolated dodge-derailment fixture's
/// 77 in-window ticks to `0.0025` rad/s rms only with this damping (and
/// yaw/roll stick torque) live mid-flip, and its post-window spin decay
/// (`≈3.9` rad/s per second at `|ω| = 5.5`, pitch locked, stick neutral on
/// yaw) is exactly this coefficient's own rate.
pub const AIR_CONTROL_PITCH_DAMPING: f32 = 30.0;

/// Real air-control damping about the car's local *up* axis (yaw rate),
/// RocketSim's `CAR_AIR_CONTROL_DAMPING.y = 20`, scaled by `1 - |yaw|` —
/// see `AIR_CONTROL_PITCH_DAMPING` (`RB-PHYSICS-001-FR-071`).
pub const AIR_CONTROL_YAW_DAMPING: f32 = 20.0;

/// Real air-control damping about the car's local *forward* axis (roll
/// rate), RocketSim's `CAR_AIR_CONTROL_DAMPING.z = 50` — the one axis whose
/// damping is *not* reduced by its own stick input (`dampRoll` has no `(1 -
/// |roll|)` factor in `_UpdateAirTorque`), so full roll input fights its own
/// damping and settles at `AIR_CONTROL_ROLL_TORQUE / AIR_CONTROL_ROLL_DAMPING
/// = 8` rad/s, above `MAX_CAR_ANGULAR_SPEED`, which therefore still caps a
/// sustained roll — see `AIR_CONTROL_PITCH_DAMPING`
/// (`RB-PHYSICS-001-FR-071`).
pub const AIR_CONTROL_ROLL_DAMPING: f32 = 50.0;

/// RocketSim's own `RLConst.h`: `CAR_TORQUE_SCALE = 2 * M_PI / (1 << 16) *
/// 1000` — converts a raw `CAR_AIR_CONTROL_TORQUE`-style "torque" value
/// (also used at the dodge-torque and autoroll-torque call sites, per
/// `RB-PHYSICS-001-FR-079`) directly into an angular-acceleration rate in
/// rad/s², once applied via `apply_angular_acceleration` rather than
/// `apply_torque` — see `AIR_CONTROL_PITCH_TORQUE`'s own doc comment for
/// why that distinction is exactly the fix this constant's real usage
/// requires.
pub const CAR_TORQUE_SCALE: f32 = 2.0 * std::f32::consts::PI / 65536.0 * 1000.0;

/// Uncalibrated placeholder wall-jump horizontal push-off speed (uu/s),
/// applied outward along the wall's normal as an instantaneous velocity
/// change (like `JUMP_SPEED`, not a continuous force) on a fresh airborne
/// jump press while touching a wall — chosen only to produce a visibly
/// distinct push-off from the wall in tests, not derived from any measured
/// or documented Rocket League value. Unlike `JUMP_SPEED`, this port has no
/// public reference for a wall-jump-specific number to reuse. `pub` so
/// `world.rs`'s end-to-end tests can assert against it directly (including
/// distinguishing a wall jump from the differently-sized `DODGE_SPEED`),
/// the same way `JUMP_SPEED` already is.
///
/// `RB-PHYSICS-001-FR-067` looked for that missing reference directly:
/// fetching RocketSim's real `Car.cpp` found real Rocket League has no
/// separate wall-jump mechanic — or constant — at all. `_UpdateJump`
/// applies exactly one impulse, `GetUpDir() * mutatorConfig.jumpImmediateForce`
/// (the same real value this port's own `JUMP_SPEED` already matches),
/// gated only on `isOnGround`, itself defined purely by wheel-contact count
/// (`numWheelsInContact >= 3`) with no floor-vs-wall distinction at all; a
/// dedicated search of `RLConst.h` for any `WALL`-named constant found only
/// an unrelated Heatseeker-mode threshold. Since `RB-PHYSICS-001-FR-065`
/// already confirmed real Rocket League's cars ride on Bullet's own
/// raycast vehicle system (`btVehicleRL`), a car driving on a wall has its
/// own orientation continuously tipped to match that wall by ordinary
/// wheel/suspension contact forces, the same way a real car tilts to match
/// a ramp — so `GetUpDir()` (the car's own local up axis in world space)
/// already points along the wall's outward normal by the time a wall jump
/// fires, with no special-cased direction logic needed. Real Rocket
/// League's "wall jump" is thus the *identical* single grounded-jump
/// impulse, along whatever direction the car's own up axis currently
/// points — never a distinct horizontal-plus-vertical composite with its
/// own separate magnitude. This confirms, with the exact mechanism rather
/// than just the constant's absence, what `RB-PHYSICS-001-FR-031`'s
/// original audit only briefly noted as "a wall jump reusing the plain
/// jump impulse rather than its own faster speed."
///
/// Not adopted as a fix: this port's car has no wheels, raycasting, or
/// surface-tracking orientation system at all (the same architecture gap
/// `RB-PHYSICS-001-FR-065` found for steering) — its orientation doesn't
/// automatically tip to match a touched wall, so its own up axis stays
/// world-vertical throughout a wall touch. Applying only `JUMP_SPEED`
/// straight up on a wall touch, as the confirmed real mechanism would
/// suggest, would produce no push-off from the wall at all in this port,
/// defeating the entire point of a wall jump. This port's own two-component
/// composite (a separate horizontal push-off along the wall's normal, on
/// top of the same vertical `JUMP_SPEED`) remains a deliberate, necessary
/// substitute for the missing surface-tracking orientation mechanism, not
/// an unfilled calibration gap — `WALL_JUMP_HORIZONTAL_SPEED`'s own
/// magnitude is still an uncalibrated placeholder, but the two-component
/// shape itself is not a mistake to correct.
pub const WALL_JUMP_HORIZONTAL_SPEED: f32 = 550.0;

/// Deadzone for `pitch`/`roll` input at the moment of a double jump's
/// fresh press: below this magnitude on both axes, the press is treated as
/// "no directional intent" and fires a plain vertical double jump; at or
/// above it on either axis, it fires a dodge instead.
///
/// `RB-PHYSICS-001-FR-075` found this port's own long-standing "not a
/// physics constant and not derived from any Rocket League value" framing
/// was stale: RocketSim's own confirmed `_UpdateDoubleJumpOrFlip`
/// cancellation check (`RB-PHYSICS-001-FR-072`/`FR-073`/`FR-074`'s own
/// fetches) is `if (abs(controls.yaw + controls.roll) < 0.1f &&
/// abs(controls.pitch) < 0.1f) { dodgeDir = {0,0,0}; }` — a dodge fires iff
/// `abs(yaw + roll) >= 0.1 || abs(pitch) >= 0.1`. Since `FR-073` already
/// folds yaw into this port's own `dodge_roll`/`wall_roll` (`roll + yaw`),
/// this port's own trigger — `dodge_pitch.abs() > DODGE_DEADZONE ||
/// dodge_roll.abs() > DODGE_DEADZONE` — is the *same* boolean expression as
/// the real one once `DODGE_DEADZONE` equals the real threshold, up to an
/// immaterial strict-vs-non-strict inequality at the exact boundary value
/// (a floating-point edge case with no practical effect). `0.1` is exactly
/// that real threshold: this constant already matches real Rocket League,
/// it just wasn't confirmed as such until this fetch. See
/// `normalize_dodge_direction`'s own doc comment for the related, already-
/// adopted normalization/snap findings this same cancellation check feeds
/// into.
const DODGE_DEADZONE: f32 = 0.1;

/// Real confirmed threshold (`RB-PHYSICS-001-FR-074`): RocketSim's own
/// `_UpdateDoubleJumpOrFlip` zeroes any component of the *normalized*
/// `dodgeDir` whose absolute value is below `0.1`, applied after
/// `dodgeDir.safeNormalized()` — `if (abs(dodgeDir.x()) < 0.1f)
/// dodgeDir.x() = 0; if (abs(dodgeDir.y()) < 0.1f) dodgeDir.y() = 0;` —
/// snapping a near-axis-aligned diagonal stick input (e.g. 88 degrees
/// instead of a clean 90) to a pure single-axis dodge instead of leaving a
/// tiny, likely-unintentional perpendicular component from imprecise stick
/// centering. Numerically identical to `DODGE_DEADZONE` above, but a
/// distinct real constant serving a different purpose (a pre-normalization
/// raw-stick trigger threshold vs. a post-normalization direction-snap
/// threshold) — kept as its own name rather than reusing `DODGE_DEADZONE`,
/// so the two can diverge if either is ever recalibrated independently.
/// See `normalize_dodge_direction`'s own doc comment for the adoption
/// reasoning.
const DODGE_DIRECTION_SNAP_THRESHOLD: f32 = 0.1;

/// Real Rocket League's own dodge horizontal impulse speed (uu/s), applied
/// along `forward_axis` (scaled by the dodge's forward component) and/or
/// `right_axis` (scaled by `roll + yaw`) as an instantaneous velocity
/// change (like `JUMP_SPEED`, not a continuous force). RocketSim's own
/// `RLConst.h`: `FLIP_INITIAL_VEL_SCALE = 500.f`, applied in
/// `_UpdateDoubleJumpOrFlip` as `dodgeDir * FLIP_INITIAL_VEL_SCALE` before
/// the per-direction speed scales below. `pub` so `world.rs`'s end-to-end
/// tests can assert against it directly, the same way `JUMP_SPEED` already
/// is. This is the standing-start (and forward-dodge) magnitude
/// specifically — since `RB-PHYSICS-001-FR-059`, a backward or side dodge
/// made at speed scales above this via `dodge_speed_scale`.
///
/// Until `RB-PHYSICS-001-FR-080` this was a `1400.0` placeholder, `2.8x`
/// the real value, kept under `RB-PHYSICS-001-FR-031`'s "false precision"
/// reasoning (real magnitudes calibrated against a car body this port
/// doesn't match). That reasoning never applied here: `apply_impulse`
/// divides by mass and the call site multiplies by `car.mass()`, so this is
/// a mass-independent velocity change, not a force or torque. The real
/// value was also confirmed directly from `FR-079`'s isolated real capture:
/// the recorded dodge-tick velocity change is `≈620` uu/s in magnitude, and
/// `500 * (0.707, -0.707)` with `DODGE_SIDE_SPEED_SCALE` at the recorded
/// forward speed (`≈1170` uu/s) predicts `626` — a `~1%` match.
pub const DODGE_SPEED: f32 = 500.0;

/// Confirmed real ratio: a *backward* pitch-dodge (one opposing the car's
/// own current forward-velocity direction, per `dodge_is_backward`)
/// grows up to this multiple of `DODGE_SPEED` as current speed rises
/// toward `MAX_CAR_SPEED` — RocketSim's own `Car.cpp`
/// (`_UpdateDoubleJumpOrFlip`) confirmed exact against `RLConst.h`:
/// `FLIP_BACKWARD_IMPULSE_MAX_SPEED_SCALE = 2.5f`. A forward pitch-dodge's
/// own real scale is exactly `1.0` (`FLIP_FORWARD_IMPULSE_MAX_SPEED_SCALE`)
/// — unchanged from `DODGE_SPEED`'s own base value — so there's no
/// separate forward-scale constant here. `RB-PHYSICS-001-FR-059` adopted
/// only this *ratio* and deliberately left `DODGE_SPEED` itself a
/// placeholder; since `RB-PHYSICS-001-FR-080`, `DODGE_SPEED` is the real
/// `FLIP_INITIAL_VEL_SCALE` too, so a backward dodge's magnitude is now
/// real end to end (with `DODGE_BACKWARD_SCALE_X` below completing it).
const DODGE_BACKWARD_SPEED_SCALE: f32 = 2.5;

/// Confirmed real factor applied to a backward dodge's forward-axis
/// component on top of `DODGE_BACKWARD_SPEED_SCALE`'s speed ramp —
/// RocketSim's own `RLConst.h` `FLIP_BACKWARD_IMPULSE_SCALE_X = 16.f /
/// 15.f`, applied in `_UpdateDoubleJumpOrFlip` as `if (shouldDodgeBackwards)
/// initalDodgeVel.x *= FLIP_BACKWARD_IMPULSE_SCALE_X` after the speed
/// ramp, so it multiplies (never replaces) that ramp and applies at a
/// standstill too. `RB-PHYSICS-001-FR-059` adopted the other three
/// per-direction scales from the same block but not this one;
/// `RB-PHYSICS-001-FR-080` adds it alongside the real `DODGE_SPEED`.
const DODGE_BACKWARD_SCALE_X: f32 = 16.0 / 15.0;

/// Confirmed real ratio: a side (`roll`) dodge grows up to this multiple of
/// `DODGE_SPEED` as current speed rises toward `MAX_CAR_SPEED`, regardless
/// of left/right direction — RocketSim's own confirmed
/// `FLIP_SIDE_IMPULSE_MAX_SPEED_SCALE = 1.9f`. Since
/// `RB-PHYSICS-001-FR-080`, both the ratio and the base magnitude it
/// scales are real.
const DODGE_SIDE_SPEED_SCALE: f32 = 1.9;

/// Below this current forward speed (uu/s), `dodge_is_backward`
/// falls back to stick direction alone rather than comparing it against a
/// near-zero, noisy current-velocity direction — RocketSim's own
/// confirmed threshold, `abs(forwardSpeed_UU) < 100.0f`.
const DODGE_BACKWARD_CLASSIFICATION_SPEED_THRESHOLD: f32 = 100.0;

/// A dodge's real per-axis magnitude scale as a function of the car's
/// current forward speed — confirmed against RocketSim's own `Car.cpp`
/// during `RB-PHYSICS-001-FR-059`'s audit: `1.0` (no change) at a standing
/// start, rising linearly to `scale_at_max_speed` by `MAX_CAR_SPEED`, then
/// held flat beyond it (`forward_speed` can in principle exceed
/// `MAX_CAR_SPEED` transiently, e.g. after a boosted speed-flip chain;
/// RocketSim's own real ratio isn't itself clamped in the source, but this
/// port clamps it here rather than let an already-uncalibrated dodge
/// magnitude grow unbounded past the one confirmed reference point).
fn dodge_speed_scale(forward_speed: f32, scale_at_max_speed: f32) -> f32 {
    let ratio = (forward_speed.abs() / MAX_CAR_SPEED).min(1.0);
    1.0 + (scale_at_max_speed - 1.0) * ratio
}

/// Whether a pitch-dodge counts as "backward" for `DODGE_BACKWARD_SPEED_SCALE`
/// purposes: opposing the car's own current forward-velocity direction: at
/// or above `DODGE_BACKWARD_CLASSIFICATION_SPEED_THRESHOLD`, that means
/// `dodge_forward` and `forward_speed` disagree in sign (dodging forward
/// while already moving backward counts as "backward" too, the same as
/// the more common backward-dodge-while-moving-forward case — both oppose
/// current motion); below it, classification falls back to
/// `dodge_forward`'s own sign alone, since comparing against a near-zero
/// velocity direction would be noise. Confirmed against RocketSim's own
/// `Car.cpp` (`shouldDodgeBackwards`), and since `RB-PHYSICS-001-FR-079` a
/// symbol-for-symbol match: `dodge_forward` *is* the reference's own
/// `dodgeDir.x` (`= -controls.pitch`, computed by the caller), so
/// `dodge_forward < 0.0` and `(dodge_forward >= 0.0) != (forward_speed >=
/// 0.0)` are the reference's own two branches verbatim. Before that fix
/// this took the raw stick pitch under this port's own (inverted) sign
/// convention; see `normalize_dodge_direction`'s own doc comment.
fn dodge_is_backward(dodge_forward: f32, forward_speed: f32) -> bool {
    if forward_speed.abs() < DODGE_BACKWARD_CLASSIFICATION_SPEED_THRESHOLD {
        dodge_forward < 0.0
    } else {
        (dodge_forward >= 0.0) != (forward_speed >= 0.0)
    }
}

/// Normalizes a dodge's combined `(pitch, roll)` stick direction to unit
/// length, returning `(0.0, 0.0)` if both are exactly zero. `dodge_speed`/
/// `DODGE_ANGULAR_SPEED` are scaled by this normalized pair instead of the
/// raw stick values, so a diagonal dodge (both axes held) has the same
/// total magnitude as an axis-aligned one.
///
/// `RB-PHYSICS-001-FR-059`'s own Non-goals had already found and flagged
/// this exact gap — this port previously summed each axis' own
/// full-strength contribution independently, so a diagonal dodge came out
/// `sqrt(2)`-ish times faster than an axis-aligned one, "a separate,
/// independent behavioral question this requirement doesn't take on."
/// `RB-PHYSICS-001-FR-072` fetched RocketSim's own `Car.cpp`
/// (`_UpdateDoubleJumpOrFlip`) and confirmed the real mechanism: `dodgeDir
/// = btVector3(-controls.pitch, controls.yaw + controls.roll,
/// 0).safeNormalized()`, applied before any further per-axis
/// forward/backward/side speed scaling
/// (`dodge_speed_scale`/`dodge_is_backward`, `FR-059`'s own already-
/// adopted finding). Unlike that per-direction *speed* ratio's own real
/// absolute magnitude (independently uncalibrated, per `FR-031`'s "false
/// precision" reasoning), normalization is a pure geometric operation this
/// port's own model represents exactly — it transfers cleanly regardless
/// of `DODGE_SPEED`'s own uncalibrated base value, the same way
/// `FR-058`/`FR-059`/`FR-068`'s own adopted ratios do.
///
/// One thing was, until `RB-PHYSICS-001-FR-079`, deliberately *not* adopted
/// here: this port kept its own sign convention (`dodge_pitch` positive
/// meant forward) rather than the reference's own negated `-controls.pitch`.
/// That turned out to be a bug, not a free choice — the stick values this
/// port replays come straight from real captures, where `pitch = -1` is a
/// forward flip (and, in air control, nose-down), so a port-private
/// convention silently dodged every recorded forward flip backward. This
/// function still returns the normalized raw stick pair unchanged; the
/// caller (`apply_driven_forces`) now forms `dodge_forward = -norm_pitch`
/// exactly as the reference forms `dodgeDir.x`, and uses that for the
/// impulse, the spin, and `dodge_is_backward`. See
/// `AIR_CONTROL_PITCH_TORQUE`'s own doc comment for the matching air-control
/// finding (`dirPitch_right = -GetRightDir()`).
///
/// Real yaw input's own contribution to `dodgeDir` (`controls.yaw +
/// controls.roll`) *is* folded in, though not by this function itself:
/// since `RB-PHYSICS-001-FR-073`, both call sites in `apply_driven_forces`
/// pass `roll + yaw` (each individually clamped to `[-1.0, 1.0]` first,
/// matching how `apply_driven_forces` already clamps pitch/yaw/roll
/// separately for air control) as this function's `roll` argument — this
/// port already reads `input.yaw` in this same function for air control, so
/// no new input plumbing was needed, just combining an already-available
/// value the same way the reference does. `RB-PHYSICS-001-FR-059`'s own
/// Non-goals had flagged this exact gap ("this port's dodge direction is
/// pitch/roll only").
///
/// Since `RB-PHYSICS-001-FR-074`, the returned pair also matches
/// RocketSim's own post-normalization small-component zeroing: after
/// normalizing, any component whose magnitude falls below
/// `DODGE_DIRECTION_SNAP_THRESHOLD` (real value `0.1`, confirmed identical
/// to `dodgeDir.x()`/`dodgeDir.y()`'s own zeroing in `_UpdateDoubleJumpOrFlip`)
/// is snapped to exactly zero — a near-axis-aligned diagonal stick input
/// no longer leaves a tiny, physically negligible but real perpendicular
/// dodge component. `RB-PHYSICS-001-FR-073`'s own Non-goals had flagged
/// this as a "separate architectural difference," but it isn't one: like
/// normalization itself, it's a pure post-processing step on the already-
/// normalized pair this function already computes, needing no new
/// machinery — the same "pure operation, no new architecture" transfer
/// this function's own earlier finding already used.
fn normalize_dodge_direction(pitch: f32, roll: f32) -> (f32, f32) {
    let magnitude = (pitch * pitch + roll * roll).sqrt();
    if magnitude > 0.0 {
        let mut norm_pitch = pitch / magnitude;
        let mut norm_roll = roll / magnitude;
        if norm_pitch.abs() < DODGE_DIRECTION_SNAP_THRESHOLD {
            norm_pitch = 0.0;
        }
        if norm_roll.abs() < DODGE_DIRECTION_SNAP_THRESHOLD {
            norm_roll = 0.0;
        }
        (norm_pitch, norm_roll)
    } else {
        (0.0, 0.0)
    }
}

/// A dodge's flip in progress — this port's `flipRelTorque`/`flipTime`
/// (`RB-PHYSICS-001-FR-080`). Per-car runtime state owned by the caller
/// exactly like `jump_hold_time_remaining`: `None` when no dodge is in
/// flight (the default, and what landing restores), `Some` from the step
/// a dodge (ground or wall-jump) fires until the car next touches the
/// ground, a plain double jump supersedes it, or a further fresh jump press
/// flip-cancels it (`RB-PHYSICS-001-FR-016`, see the module doc comment).
///
/// `rel_torque` is the flip's torque direction in the car's own local
/// frame as `(forward, right)` — RocketSim's own `flipRelTorque =
/// (-dodgeDir.y, dodgeDir.x, 0)` symbol for symbol (local `x` = forward,
/// `y` = right), captured once at the dodge from the normalized
/// `(pitch, roll)` stick direction, before its `/ tickTimeScale` (applied
/// per step here instead, see `apply_driven_forces`). So a forward flip
/// (`pitch = -1`) is `(0, 1)`, a left dodge (`roll`/`yaw = -1`) `(1, 0)`.
/// `elapsed` is seconds since the dodge's own step, advanced at the end of
/// every airborne `apply_driven_forces` call (RocketSim increments
/// `flipTime` in `_UpdateDoubleJumpOrFlip`, after `_UpdateAirTorque`), so
/// the first step to apply flip torque is the one *after* the dodge, with
/// `elapsed == dt` — and it keeps counting past `FLIP_TORQUE_TIME` for
/// `FLIP_PITCHLOCK_EXTRA_TIME`'s sake, until landing clears the whole
/// `Option`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DodgeFlip {
    /// Flip torque direction in the car's local `(forward, right)` frame —
    /// see the type's own doc comment.
    pub rel_torque: (f32, f32),
    /// Seconds since the dodge fired — see the type's own doc comment.
    pub elapsed: f32,
}

/// Real flip torque about the car's *forward* axis (roll — a sideways
/// dodge), RocketSim's own `RLConst.h` `FLIP_TORQUE_X = 260.f, // Left/
/// Right` (`RB-PHYSICS-001-FR-080`). Applied via
/// `apply_angular_acceleration`, so it's an inertia-independent angular
/// acceleration like `AIR_CONTROL_PITCH_TORQUE` — but, unlike air control,
/// **without** `CAR_TORQUE_SCALE`: RocketSim's `_UpdateAirTorque` applies
/// `applyTorque(invInertiaTensorWorld.inverse() * basis * (flipRelTorque *
/// (FLIP_TORQUE_X, FLIP_TORQUE_Y, 0)))` with no further scale, which is the
/// reference's own omission, not this port's. Because `flipRelTorque` is
/// pre-divided by `tickTimeScale = dt / (1/120)`, the resulting angular
/// velocity step is per-*tick*, not per-second: `260 / 120 ≈ 2.17` rad/s
/// each step regardless of `dt`. That reaches `MAX_CAR_ANGULAR_SPEED =
/// 5.5` within three `1/120` s steps, after which `clamp_angular_speed`
/// holds it there for the rest of `FLIP_TORQUE_TIME` — the real "continuous
/// flip torque" is, in effect, *drive to the angular-speed cap along the
/// flip axis and hold it there for 0.65 s*, confirmed to the tick by the
/// isolated `dodge-derailment` capture (`|ω|` `3.40 → 5.22 → 5.50`, then
/// exactly `5.50` every tick through the window's end). See
/// `apply_driven_forces`.
pub const FLIP_TORQUE_X: f32 = 260.0;

/// Real flip torque about the car's *right* axis (pitch — a forward or
/// backward flip), RocketSim's own `FLIP_TORQUE_Y = 224.f, // Forward/
/// backward`: `224 / 120 ≈ 1.87` rad/s per tick. Genuinely differs from
/// `FLIP_TORQUE_X` — see that constant's own doc comment for the shared
/// mechanism (`RB-PHYSICS-001-FR-080`).
pub const FLIP_TORQUE_Y: f32 = 224.0;

/// How long (seconds) a dodge's flip torque keeps applying after the dodge,
/// RocketSim's own `FLIP_TORQUE_TIME = 0.65f`: `isFlipping = hasFlipped &&
/// flipTime < FLIP_TORQUE_TIME` — a hard cutoff, with no ramp or decay
/// beforehand. While flipping, pitch is locked out of stick air control;
/// yaw and roll air control (and the real damping,
/// `AIR_CONTROL_PITCH_DAMPING`) stay live — RocketSim and RLUtilities both
/// lock all three out,
/// but the real capture shows otherwise, see `apply_driven_forces`
/// (`RB-PHYSICS-001-FR-080`). RocketSim also declares
/// `FLIP_TORQUE_MIN_TIME = 0.41f` and `FLIP_PITCHLOCK_TIME = 1.f` in
/// `RLConst.h`, but references neither anywhere in `Car.cpp`, so this port
/// doesn't carry them.
pub const FLIP_TORQUE_TIME: f32 = 0.65;

/// How much longer (seconds) *after* `FLIP_TORQUE_TIME` pitch input stays
/// locked out of stick air control, RocketSim's own
/// `FLIP_PITCHLOCK_EXTRA_TIME = 0.3f`: `pitchTorqueScale = 0` while
/// `flipTime < FLIP_TORQUE_TIME + FLIP_PITCHLOCK_EXTRA_TIME`, i.e. for the
/// first `0.95` s after a dodge; yaw and roll return to normal the moment
/// the flip torque stops (`RB-PHYSICS-001-FR-080`).
pub const FLIP_PITCHLOCK_EXTRA_TIME: f32 = 0.3;

/// Per-tick vertical-speed bleed while flipping, RocketSim's own
/// `FLIP_Z_DAMP_120 = 0.35f`: `linearVelocity.z *= (1 - 0.35)^tickTimeScale`
/// — a `×0.65` per `1/120` s tick — applied every step from
/// `FLIP_Z_DAMP_START` through `FLIP_TORQUE_TIME`, unconditionally before
/// `FLIP_Z_DAMP_END` and only while already moving downward after it
/// (`RB-PHYSICS-001-FR-080`). Under gravity this settles the car's fall at
/// exactly `vz = 0.65 * (vz - g * dt)`, i.e. `-(650 / 120) / (1 - 0.65) ≈
/// -15.5` uu/s — the plateau the isolated `dodge-derailment` capture holds
/// from `t ≈ 4.47` to `4.97` s. Applied as a direct `linear_velocity.z`
/// write before this step's own `integrate_velocities`, the same ordering
/// as RocketSim's `_UpdateDoubleJumpOrFlip` (which runs before its Bullet
/// step).
const FLIP_Z_DAMP_120: f32 = 0.35;

/// When (seconds since the dodge) `FLIP_Z_DAMP_120` starts applying,
/// RocketSim's own `FLIP_Z_DAMP_START = 0.15f` (`RB-PHYSICS-001-FR-080`).
const FLIP_Z_DAMP_START: f32 = 0.15;

/// Until when (seconds since the dodge) `FLIP_Z_DAMP_120` applies even
/// while still moving *upward*, RocketSim's own `FLIP_Z_DAMP_END = 0.21f`;
/// from here to `FLIP_TORQUE_TIME` it only applies to a downward `vz`
/// (`RB-PHYSICS-001-FR-080`).
const FLIP_Z_DAMP_END: f32 = 0.21;

/// The tick rate (Hz) RocketSim's per-tick flip constants are expressed
/// against — `tickTimeScale = tickTime / (1 / 120.f)` in
/// `_UpdateDoubleJumpOrFlip`. `FLIP_TORQUE_X`/`FLIP_TORQUE_Y` are divided by
/// `dt * FLIP_REFERENCE_TICK_RATE` and `FLIP_Z_DAMP_120`'s bleed raised to
/// it, so both stay per-tick at any `dt` (`RB-PHYSICS-001-FR-080`).
const FLIP_REFERENCE_TICK_RATE: f32 = 120.0;

/// Maximum duration (seconds) that continuing to hold `jump` after a fresh
/// ground-jump press keeps adding extra upward acceleration
/// (`JUMP_HOLD_ACCELERATION`). Confirmed, not just guessed, during
/// `RB-PHYSICS-001-FR-031`'s audit: this port's pre-existing `0.2` already
/// matches both RocketSim's `RLConst.h` (`JUMP_MAX_TIME = 0.2f`) and
/// RLUtilities' `Jump::max_duration = 0.2f`. Real Rocket League also has a
/// `JUMP_MIN_TIME` (0.025s) during which the hold acceleration is scaled
/// down (`JUMP_PRE_MIN_ACCEL_SCALE`) rather than applied at full strength
/// immediately — since `RB-PHYSICS-001-FR-064`, that two-phase ramp is
/// modeled too, see `JUMP_MIN_TIME`'s own doc comment.
const JUMP_HOLD_MAX_DURATION: f32 = 0.2;

/// Continuous upward acceleration (uu/s^2) applied every step `jump` is
/// held and `JUMP_HOLD_MAX_DURATION` hasn't yet elapsed since the ground
/// jump's own fresh press, on top of that press's fixed `JUMP_SPEED`
/// impulse. Refined from an earlier `1400.0` approximation to the precise
/// value during `RB-PHYSICS-001-FR-031`'s audit: RocketSim's `RLConst.h`
/// defines `JUMP_ACCEL = 4375.f/3.f`, matched independently by RLUtilities'
/// `Jump::acceleration = 1458.3333f`. Scaled down by `JUMP_PRE_MIN_ACCEL_SCALE`
/// during `JUMP_MIN_TIME`'s own mandatory window — see that constant's own
/// doc comment.
const JUMP_HOLD_ACCELERATION: f32 = 4375.0 / 3.0;

/// Seconds after a ground-jump press during which `JUMP_HOLD_ACCELERATION`
/// (scaled by `JUMP_PRE_MIN_ACCEL_SCALE`) keeps applying regardless of
/// whether `jump` is still held — a mandatory minimum hold real Rocket
/// League's own engine applies even to an instantaneous tap. Confirmed
/// exact against RocketSim's real `RLConst.h` (`JUMP_MIN_TIME = 0.025f`)
/// during `RB-PHYSICS-001-FR-064`; fetching the same reference's actual
/// `Car.cpp` (`_UpdateJump`) directly confirmed the exact mechanism —
/// `jumpTime < JUMP_MIN_TIME || (jumpPressed && jumpTime < JUMP_MAX_TIME)`
/// gates whether the force applies at all, with the pre-`JUMP_MIN_TIME`
/// branch scaling it down regardless of `jumpPressed` — not merely the
/// constant's existence. That same source's own inline comment (`// TODO:
/// Either move to RLConst or preferably don't use this system at all`)
/// flags this as a stopgap even its own authors consider provisional, not a
/// deliberate permanent design choice — adopted here anyway since it's
/// still the real, currently-shipping behavior, and both this and
/// `JUMP_PRE_MIN_ACCEL_SCALE` are a duration and a dimensionless ratio
/// respectively, not a torque or force calibrated against real Rocket
/// League's own specific car mass/inertia, so unlike most of `drive.rs`'s
/// own torque-shaped placeholders (see the module doc comment's own
/// "false precision" discussion) they transfer cleanly regardless of this
/// port's car body not matching that calibration.
const JUMP_MIN_TIME: f32 = 0.025;

/// Multiplier applied to `JUMP_HOLD_ACCELERATION` during `JUMP_MIN_TIME`'s
/// own mandatory window. Confirmed exact against RocketSim's real
/// `Car.cpp` (`_UpdateJump`, `constexpr float JUMP_PRE_MIN_ACCEL_SCALE =
/// 0.62f;`) during `RB-PHYSICS-001-FR-064` — a hard step-scale applied
/// all-or-nothing for the whole window (`totalJumpForce *=
/// JUMP_PRE_MIN_ACCEL_SCALE`), not an interpolation ramping from `0.62` up
/// to `1.0` as `JUMP_MIN_TIME` approaches.
const JUMP_PRE_MIN_ACCEL_SCALE: f32 = 0.62;

fn forward_axis(car: &RigidBody) -> Vec3 {
    car.orientation.rotate(&Vec3::new(1.0, 0.0, 0.0))
}

fn up_axis(car: &RigidBody) -> Vec3 {
    car.orientation.rotate(&Vec3::new(0.0, 0.0, 1.0))
}

/// The car's local "right" axis (local +Y) — completes the right-handed
/// (forward, right, up) basis `up_axis × forward_axis` gives. Used only for
/// pitch torque (nose up/down about this axis); throttle/steer/boost/
/// handbrake/jump never need it.
fn right_axis(car: &RigidBody) -> Vec3 {
    car.orientation.rotate(&Vec3::new(0.0, 1.0, 0.0))
}

/// The horizontal `(forward, right)` pair a dodge's translation impulse is
/// applied along (`RB-PHYSICS-001-FR-081` finding 2): RocketSim's
/// `_UpdateDoubleJumpOrFlip` uses `forwardDir2D = GetForwardDir().To2D()
/// .Normalized()` and `rightDir2D = (-forwardDir2D.y, forwardDir2D.x, 0)`,
/// so the impulse is exactly horizontal whatever the car's pitch or roll —
/// this port applied it along the car's tilted 3D axes until then, which
/// at the isolated fixture's dodge (nose `3°` down) leaked `-75` uu/s into
/// vertical velocity the real dodge doesn't have. Falls back to the 3D
/// axes for a car pointing straight up or down, where the flattened
/// forward has no direction (RocketSim's own `Normalized()` of a zero
/// vector is undefined there; this port keeps the impulse finite).
fn dodge_axes_2d(car: &RigidBody) -> (Vec3, Vec3) {
    let forward = forward_axis(car);
    match Vec3::new(forward.x, forward.y, 0.0).normalize() {
        Some(forward_2d) => (forward_2d, Vec3::new(-forward_2d.y, forward_2d.x, 0.0)),
        None => (forward, right_axis(car)),
    }
}

/// `RB-PHYSICS-001-FR-037` — whether `input` represents genuine driving
/// intent, for waking a sleeping car (see `apply_driven_forces`'s own doc
/// comment). Treats an unrecovered analog channel (`None`, from a replay
/// that never had one) the same as a recovered-but-literally-neutral one
/// (`Some(0.0)`, from a capture) — both mean "no analog input this tick" —
/// rather than the simpler `*input != ControllerInput::default()` (which
/// would treat any `Some(0.0)` as active purely because it's not `None`,
/// keeping a car receiving a real recorded input stream that always
/// resolves every channel — even at rest — from ever sleeping at all).
fn input_is_active(input: &ControllerInput) -> bool {
    input.throttle != 0.0
        || input.steer != 0.0
        || input.pitch.unwrap_or(0.0) != 0.0
        || input.yaw.unwrap_or(0.0) != 0.0
        || input.roll.unwrap_or(0.0) != 0.0
        || input.jump
        || input.boost
        || input.handbrake
}

/// Applies throttle, steering, boost, handbrake, jump, double jump, wall
/// jump, and air control as forces/torques/impulses (or, for handbrake, a
/// temporary friction adjustment) on `car`. Throttle, steering, handbrake,
/// and the ground jump are a no-op unless `on_ground`; air control, double
/// jump, and wall jump are the reverse — a no-op unless *not* `on_ground`;
/// boost isn't gated on ground contact at all, but is a no-op once
/// `*boost_amount` reaches zero. `base_friction` is the car's own nominal
/// (non-handbraking) friction — handbrake temporarily reduces
/// `car.friction` below it while held and grounded, and restores it
/// otherwise, so callers don't need a separate restore step. `jump_held` is
/// the car's `input.jump` value as of the *previous* call — every jump
/// variant fires only on a rising edge (`input.jump && !*jump_held`), so a
/// continued press doesn't re-fire every step; it's updated to `input.jump`
/// on every call, including while airborne, so a fresh press is still
/// required for a double or wall jump even if the button was never
/// released after the ground jump. `double_jump_available` is whether the
/// car still has a double jump (plain or dodge) to spend this airborne
/// period — landing (`on_ground`) or merely touching a wall
/// (`wall_normal.is_some()`, no jump press required) both unconditionally
/// set it back to `true`; only an airborne fresh press that fires the
/// double jump or a dodge (not a wall jump) sets it to `false`, until the
/// next landing or wall touch. `wall_normal` is the outward normal of the
/// wall the car is currently touching, if any (computed by the caller the
/// same way `on_ground` is — see `PhysicsWorld`); a fresh press while
/// airborne and `wall_normal.is_some()` fires a wall jump instead of
/// consulting `double_jump_available` at all — wall jump never dodges,
/// regardless of `input.pitch`/`input.roll`. `jump_hold_time_remaining` is
/// how much longer, in seconds, continuing to hold `jump` should keep
/// adding extra upward acceleration to a ground jump — checked and
/// decremented *before* this call's own `on_ground`/`jump_pressed` handling
/// below, using whatever value the *previous* call left it at, so a fresh
/// ground-jump press's own step only ever fires the plain `JUMP_SPEED`
/// impulse; that same press then re-arms `jump_hold_time_remaining` to
/// `JUMP_HOLD_MAX_DURATION` for subsequent calls to consume. Since
/// `RB-PHYSICS-001-FR-064`, releasing `jump` doesn't always zero it right
/// away: `JUMP_MIN_TIME` seconds' own mandatory window (derived as
/// `JUMP_HOLD_MAX_DURATION - *jump_hold_time_remaining < JUMP_MIN_TIME`)
/// keeps decrementing it, at a `JUMP_PRE_MIN_ACCEL_SCALE`-scaled
/// acceleration, regardless of `input.jump` — only past that window does
/// releasing `jump` stop the extra acceleration immediately. It's otherwise
/// untouched by the double jump, a dodge, or the wall jump — see the module
/// doc comment. `dodge_flip` is the car's dodge flip in progress, if any
/// (`DodgeFlip`, `RB-PHYSICS-001-FR-080`): the dodge branches (ground and
/// wall-jump) set it to a fresh `Some` with `elapsed = 0.0`, every later
/// airborne call applies the real flip torque while `elapsed <
/// FLIP_TORQUE_TIME` (with pitch locked out of stick air control
/// meanwhile; yaw/roll air control and the real damping stay
/// live, and `input.pitch` held in the flip's own pitch sign is the real
/// **flip cancel**, scaling the torque's pitch component by `1 - |pitch|`,
/// see the module doc comment), keeps pitch locked out of air
/// control for `FLIP_PITCHLOCK_EXTRA_TIME` beyond that, bleeds vertical
/// speed per `FLIP_Z_DAMP_120`, and advances `elapsed` at the end;
/// `on_ground` clears it to `None` unconditionally, and the
/// plain-double-jump branch explicitly clears it too (so a stale flip from
/// an earlier dodge can't run on under a later unrelated double jump). A
/// further fresh press while airborne, not touching a wall, with
/// `double_jump_available` already spent does nothing. Since `RB-PHYSICS-001-FR-037`, any
/// genuinely active `input` (see `input_is_active`) wakes `car`
/// unconditionally before anything else in this call runs, regardless of
/// whether `car` was already asleep or what velocity results this step —
/// see this crate's own `body::RigidBody::wake` doc comment for why a
/// velocity-only wake check isn't enough here. Call once per step, before
/// `integrate::integrate_velocities`, alongside `apply_gravity`; call
/// `clamp_angular_speed` at the end of the step, after the transform has
/// integrated (see that function's own doc comment for why the placement
/// matters).
#[allow(clippy::too_many_arguments)]
pub fn apply_driven_forces(
    car: &mut RigidBody,
    input: &ControllerInput,
    on_ground: bool,
    wall_normal: Option<Vec3>,
    boost_amount: &mut f32,
    jump_held: &mut bool,
    double_jump_available: &mut bool,
    jump_hold_time_remaining: &mut f32,
    dodge_flip: &mut Option<DodgeFlip>,
    base_friction: f32,
    dt: f32,
) {
    // RB-PHYSICS-001-FR-037: any genuinely active input wakes the car
    // unconditionally, before that input's own force/impulse has a chance
    // to move it — a resultant-velocity-only wake check would zero right
    // back out a driving force whose one-frame delta is itself smaller
    // than `body::LINEAR_SLEEP_VELOCITY_THRESHOLD` (e.g. one frame of
    // throttle from a dead stop at a very small `dt`), permanently
    // stranding an asleep car that should be free to start moving. Uses
    // `unwrap_or(0.0)` for the analog channels so a recovered-but-literally-
    // zero `Some(0.0)` reads as neutral the same way an unrecovered `None`
    // does — see `input_is_active`'s own doc comment.
    if input_is_active(input) {
        car.wake();
    }

    let forward = forward_axis(car);
    let jump_pressed = input.jump && !*jump_held;
    *jump_held = input.jump;

    // Variable jump height: apply the continuous hold acceleration using
    // whatever jump_hold_time_remaining the *previous* call left behind,
    // before this call's own on_ground/jump_pressed handling below can
    // re-arm it — so a fresh ground-jump press's own step here never gets
    // the extra force, only continued holding (or the mandatory window
    // below) into later calls does. RB-PHYSICS-001-FR-064: real Rocket
    // League's own `_UpdateJump` keeps applying this force, scaled by
    // `JUMP_PRE_MIN_ACCEL_SCALE`, for the first `JUMP_MIN_TIME` seconds
    // since the press regardless of whether `jump` is still held — derived
    // here as `JUMP_HOLD_MAX_DURATION - *jump_hold_time_remaining` rather
    // than tracked as a second, separate elapsed-time field, since at rest
    // (`jump_hold_time_remaining == 0.0`) that derivation already reads as
    // `JUMP_HOLD_MAX_DURATION`, comfortably past `JUMP_MIN_TIME`, so a car
    // that never pressed jump never spuriously enters this branch. Only
    // once that mandatory window has passed does releasing `jump` end the
    // window immediately, even if time was left.
    let in_mandatory_pre_min_window =
        JUMP_HOLD_MAX_DURATION - *jump_hold_time_remaining < JUMP_MIN_TIME;
    if in_mandatory_pre_min_window || (input.jump && *jump_hold_time_remaining > 0.0) {
        let mut hold_acceleration = JUMP_HOLD_ACCELERATION;
        if in_mandatory_pre_min_window {
            hold_acceleration *= JUMP_PRE_MIN_ACCEL_SCALE;
        }
        car.apply_central_force(Vec3::new(0.0, 0.0, hold_acceleration * car.mass()));
        *jump_hold_time_remaining = (*jump_hold_time_remaining - dt).max(0.0);
    } else {
        *jump_hold_time_remaining = 0.0;
    }

    if on_ground {
        // Landing (or simply resting) always restores the double jump,
        // regardless of this step's input — and ends any dodge flip in
        // progress, pitch lock included (RocketSim: three or more wheels
        // in contact clears `isFlipping`, and `isOnGround` resets
        // `hasFlipped`/`flipTime`; RB-PHYSICS-001-FR-080).
        *double_jump_available = true;
        *dodge_flip = None;

        let forward_speed = car.linear_velocity.dot(&forward);
        let throttle = input.throttle.clamp(-1.0, 1.0);
        if throttle != 0.0 {
            let taper = drive_speed_taper(throttle.signum() * forward_speed);
            if taper > 0.0 {
                car.apply_central_force(
                    forward * (throttle * THROTTLE_ACCELERATION * taper * car.mass()),
                );
            }
        }

        let steer = input.steer.clamp(-1.0, 1.0);
        if steer != 0.0 {
            // A stationary car can't carve a turn — scale the available
            // torque by how fast it's already going, up to MAX_CAR_SPEED.
            // RB-PHYSICS-001-FR-065: real Rocket League's own confirmed
            // steering curve has this backwards — maximum turning ability
            // is highest at a standstill and decreases with speed — but
            // that curve doesn't transfer onto this port's own
            // direct-torque model; see STEER_TORQUE's own doc comment.
            let speed_factor = (car.linear_velocity.length() / MAX_CAR_SPEED).min(1.0);
            if speed_factor > 0.0 {
                car.apply_torque(up_axis(car) * (steer * STEER_TORQUE * speed_factor));
            }
        }

        car.friction = if input.handbrake {
            base_friction * HANDBRAKE_FRICTION_MULTIPLIER
        } else {
            base_friction
        };

        if jump_pressed {
            // An instantaneous velocity change, not a continuous force —
            // apply_impulse divides by mass internally, so scaling by
            // car.mass() here cancels that out and yields a flat
            // JUMP_SPEED velocity change regardless of the car's mass.
            car.apply_impulse(Vec3::new(0.0, 0.0, JUMP_SPEED * car.mass()), Vec3::ZERO);
            // Arms the hold window for continued holding on subsequent
            // calls — this call's own jump_hold_time_remaining check above
            // already ran against the *previous* value (0, since no ground
            // jump was in flight yet), so only the fixed impulse above
            // fires this step.
            *jump_hold_time_remaining = JUMP_HOLD_MAX_DURATION;
        }
    } else {
        if wall_normal.is_some() {
            // Touching a wall restores the double jump unconditionally —
            // the same "any surface contact refills your second jump"
            // rule landing uses — regardless of whether jump is pressed
            // this step.
            *double_jump_available = true;
        }

        // RB-PHYSICS-001-FR-080: the real dodge flip is a continuous torque
        // for FLIP_TORQUE_TIME after the dodge, not an instantaneous kick.
        // `tick_scale` is RocketSim's own `tickTimeScale = tickTime /
        // (1/120)`: the torque is divided by it (so the angular-velocity
        // step per *tick* is `FLIP_TORQUE / 120` at any `dt`) and the
        // vertical bleed below is raised to it.
        let tick_scale = dt * FLIP_REFERENCE_TICK_RATE;
        let is_flipping = dodge_flip.is_some_and(|flip| flip.elapsed < FLIP_TORQUE_TIME);
        let pitch_locked = dodge_flip
            .is_some_and(|flip| flip.elapsed < FLIP_TORQUE_TIME + FLIP_PITCHLOCK_EXTRA_TIME);
        let pitch = input.pitch.unwrap_or(0.0).clamp(-1.0, 1.0);
        let yaw = input.yaw.unwrap_or(0.0).clamp(-1.0, 1.0);
        let roll = input.roll.unwrap_or(0.0).clamp(-1.0, 1.0);
        // Flip cancel (RB-PHYSICS-001-FR-080 step (c), the real mechanism
        // RB-PHYSICS-001-FR-070 found): holding pitch in the *same* sign as
        // the flip's own pitch-axis torque scales that component — only
        // that component — by `1 - |pitch|`, every step, for as long as the
        // stick is held; a roll-only flip has no pitch component and can't
        // be cancelled at all. A second jump press mid-flip does nothing
        // (RocketSim: `hasFlipped` makes `canUse` false), which the spent
        // double jump below already models.
        if let Some(flip) = dodge_flip.filter(|_| is_flipping) {
            // RocketSim `_UpdateAirTorque`: `applyTorque(invInertiaWorld
            // .inverse() * basis * (flipRelTorque * (FLIP_TORQUE_X,
            // FLIP_TORQUE_Y, 0)))` — inertia cancelled like air control
            // (hence apply_angular_acceleration), but with no
            // CAR_TORQUE_SCALE; see FLIP_TORQUE_X's own doc comment.
            // `clamp_angular_speed` after this step's integrate_velocities
            // then supplies the cap-and-hold at MAX_CAR_ANGULAR_SPEED.
            let (rel_forward, mut rel_right) = flip.rel_torque;
            if rel_right != 0.0 && pitch != 0.0 && (rel_right > 0.0) == (pitch > 0.0) {
                rel_right *= 1.0 - pitch.abs();
            }
            car.apply_angular_acceleration(
                (forward * (rel_forward * FLIP_TORQUE_X)
                    + right_axis(car) * (rel_right * FLIP_TORQUE_Y))
                    * (1.0 / tick_scale),
            );
        }

        // Air control: pitch/yaw/roll angular acceleration about the car's
        // local right/up/forward axes. Unlike ground steering, not scaled
        // by speed — a car can spin from a standing start in the air, since
        // there's no wheel grip to require momentum for.
        //
        // RB-PHYSICS-001-FR-079: pitch and roll act about the *negative* of
        // their axis, matching RocketSim's own `dirPitch_right =
        // -GetRightDir()` / `dirRoll_forward = -GetForwardDir()` (only yaw's
        // `dirYaw_up = GetUpDir()` is unnegated) — so the recorded stick
        // convention this port replays (`pitch = -1` is nose-down/forward,
        // `roll = +1` rolls right) produces the same rotation real Rocket
        // League does. See `AIR_CONTROL_PITCH_TORQUE`'s own doc comment.
        //
        // RB-PHYSICS-001-FR-080: pitch is locked out (`pitchTorqueScale =
        // 0`) through the flip and for FLIP_PITCHLOCK_EXTRA_TIME after it;
        // yaw and roll stay live throughout. Both RocketSim
        // (`doAirControl = false` while `isFlipping`) and RLUtilities lock
        // all three out during the flip, but the real capture doesn't:
        // step (c) reproduced every in-window tick of the isolated
        // dodge-derailment fixture to the recording's own rounding only
        // with yaw/roll air control (and the real damping, FR-071) active
        // mid-flip — see the requirement's entry.
        {
            let effective_pitch = if pitch_locked { 0.0 } else { pitch };
            if effective_pitch != 0.0 {
                car.apply_angular_acceleration(
                    -right_axis(car)
                        * (effective_pitch * AIR_CONTROL_PITCH_TORQUE * CAR_TORQUE_SCALE),
                );
            }

            if yaw != 0.0 {
                car.apply_angular_acceleration(
                    up_axis(car) * (yaw * AIR_CONTROL_YAW_TORQUE * CAR_TORQUE_SCALE),
                );
            }

            if roll != 0.0 {
                car.apply_angular_acceleration(
                    -forward * (roll * AIR_CONTROL_ROLL_TORQUE * CAR_TORQUE_SCALE),
                );
            }

            // RB-PHYSICS-001-FR-071: real air control's own per-axis
            // angular-velocity damping, every airborne step — the
            // `- damping` half of RocketSim's `applyTorque(invInertia
            // .inverse() * (torque - damping) * CAR_TORQUE_SCALE)`. The
            // pitch factor reads the *effective* (lock-scaled) pitch, as
            // `controls.pitch * pitchTorqueScale` does there.
            car.apply_angular_acceleration(air_control_damping(car, effective_pitch, yaw));

            // No airborne self-righting beyond the damping above:
            // RB-PHYSICS-001-FR-060 found real Rocket League has no such
            // mechanic (its auto-flip and auto-roll are grounded and
            // input-gated), and RB-PHYSICS-001-FR-071 retired this port's
            // former placeholder `LANDING_AUTO_UPRIGHT_TORQUE` nudge once
            // the real damping — the mechanism that actually makes an
            // unsteered airborne car stop tumbling — was in place.
        }

        if jump_pressed {
            if let Some(wall_normal) = wall_normal {
                // Wall jump takes priority over the double jump on this
                // press: push off outward along the wall's normal, plus
                // the same upward JUMP_SPEED every jump variant uses.
                let wall_pitch = input.pitch.unwrap_or(0.0).clamp(-1.0, 1.0);
                let wall_roll = input.roll.unwrap_or(0.0).clamp(-1.0, 1.0)
                    + input.yaw.unwrap_or(0.0).clamp(-1.0, 1.0);
                if wall_pitch.abs() > DODGE_DEADZONE || wall_roll.abs() > DODGE_DEADZONE {
                    // Wall-jump dodge: the same outward-plus-upward push
                    // combined with a directional DODGE_SPEED impulse and
                    // the real FLIP_TORQUE_X/FLIP_TORQUE_Y flip
                    // (RB-PHYSICS-001-FR-080), reusing the ground dodge's
                    // own axis/sign conventions. Unlike the plain wall jump
                    // below, this *does* consume double_jump_available —
                    // the same resource a ground dodge spends — a
                    // deliberate simplification (see the module doc
                    // comment). Starts a fresh, cancelable flip
                    // (dodge_flip), same as a ground dodge.
                    let wall_jump_forward_speed = car.linear_velocity.dot(&forward);
                    let (norm_wall_pitch, norm_wall_roll) =
                        normalize_dodge_direction(wall_pitch, wall_roll);
                    // RB-PHYSICS-001-FR-079: same sign conventions as the
                    // ground dodge below — see that block's own comment.
                    let wall_dodge_forward = -norm_wall_pitch;
                    let (dodge_forward_2d, dodge_right_2d) = dodge_axes_2d(car);
                    let mut dodge_impulse =
                        wall_normal * WALL_JUMP_HORIZONTAL_SPEED + Vec3::new(0.0, 0.0, JUMP_SPEED);
                    if wall_pitch.abs() > DODGE_DEADZONE {
                        let scale =
                            if dodge_is_backward(wall_dodge_forward, wall_jump_forward_speed) {
                                dodge_speed_scale(
                                    wall_jump_forward_speed,
                                    DODGE_BACKWARD_SPEED_SCALE,
                                ) * DODGE_BACKWARD_SCALE_X
                            } else {
                                1.0
                            };
                        dodge_impulse +=
                            dodge_forward_2d * (wall_dodge_forward * DODGE_SPEED * scale);
                    }
                    if wall_roll.abs() > DODGE_DEADZONE {
                        let scale =
                            dodge_speed_scale(wall_jump_forward_speed, DODGE_SIDE_SPEED_SCALE);
                        dodge_impulse += dodge_right_2d * (norm_wall_roll * DODGE_SPEED * scale);
                    }
                    car.apply_impulse(dodge_impulse * car.mass(), Vec3::ZERO);
                    // `flipRelTorque = (-dodgeDir.y, dodgeDir.x)` — see the
                    // ground dodge below.
                    *dodge_flip = Some(DodgeFlip {
                        rel_torque: (-norm_wall_roll, wall_dodge_forward),
                        elapsed: 0.0,
                    });
                    *double_jump_available = false;
                } else {
                    // Plain wall jump (unchanged): doesn't consume
                    // double_jump_available (already restored
                    // unconditionally above, just from touching the wall)
                    // — matching real Rocket League's "any surface contact
                    // refills your second jump" rule, so a plain wall jump
                    // doesn't cost a player their double jump.
                    car.apply_impulse(
                        (wall_normal * WALL_JUMP_HORIZONTAL_SPEED
                            + Vec3::new(0.0, 0.0, JUMP_SPEED))
                            * car.mass(),
                        Vec3::ZERO,
                    );
                }
            } else if *double_jump_available {
                let dodge_pitch = input.pitch.unwrap_or(0.0).clamp(-1.0, 1.0);
                let dodge_roll = input.roll.unwrap_or(0.0).clamp(-1.0, 1.0)
                    + input.yaw.unwrap_or(0.0).clamp(-1.0, 1.0);
                if dodge_pitch.abs() > DODGE_DEADZONE || dodge_roll.abs() > DODGE_DEADZONE {
                    // Dodge: a directional flip instead of a plain vertical
                    // double jump — forward/back from pitch (translate along
                    // forward_axis, flip about right_axis), left/right from
                    // roll (translate along right_axis, flip about
                    // forward_axis). Purely horizontal, with no vertical
                    // JUMP_SPEED component — see the module doc comment.
                    // The flip itself is the real continuous torque applied
                    // on later steps from the `dodge_flip` state set below
                    // (RB-PHYSICS-001-FR-080), never a kick on this step.
                    //
                    // RB-PHYSICS-001-FR-079: signs follow RocketSim's own
                    // `dodgeDir = (-pitch, yaw + roll)` for translation and
                    // `flipRelTorque = (-dodgeDir.y, dodgeDir.x)` (local
                    // x = forward, y = right) for spin — so the recorded
                    // stick convention this port replays (`pitch = -1` is
                    // a forward flip, `roll/yaw = -1` a left one) produces
                    // the same dodge real Rocket League does: a forward
                    // flip spins about +right (nose down first), a left
                    // dodge about +forward (left side down first).
                    let dodge_forward_speed = car.linear_velocity.dot(&forward);
                    let (norm_dodge_pitch, norm_dodge_roll) =
                        normalize_dodge_direction(dodge_pitch, dodge_roll);
                    let dodge_forward = -norm_dodge_pitch;
                    // RB-PHYSICS-001-FR-081 finding 2: the impulse is
                    // horizontal — along the car's *flattened* forward and
                    // right (RocketSim's forwardDir2D/rightDir2D), not its
                    // tilted 3D axes. The flip's own torque below still
                    // uses the real 3D body axes, as RocketSim's does.
                    let (dodge_forward_2d, dodge_right_2d) = dodge_axes_2d(car);
                    let mut dodge_impulse = Vec3::ZERO;
                    if dodge_pitch.abs() > DODGE_DEADZONE {
                        // RocketSim: the backward speed ramp, then
                        // `initalDodgeVel.x *= FLIP_BACKWARD_IMPULSE_SCALE_X`
                        // on top of it (RB-PHYSICS-001-FR-080).
                        let scale = if dodge_is_backward(dodge_forward, dodge_forward_speed) {
                            dodge_speed_scale(dodge_forward_speed, DODGE_BACKWARD_SPEED_SCALE)
                                * DODGE_BACKWARD_SCALE_X
                        } else {
                            1.0
                        };
                        dodge_impulse += dodge_forward_2d * (dodge_forward * DODGE_SPEED * scale);
                    }
                    if dodge_roll.abs() > DODGE_DEADZONE {
                        let scale = dodge_speed_scale(dodge_forward_speed, DODGE_SIDE_SPEED_SCALE);
                        dodge_impulse += dodge_right_2d * (norm_dodge_roll * DODGE_SPEED * scale);
                    }
                    car.apply_impulse(dodge_impulse * car.mass(), Vec3::ZERO);
                    // RocketSim: `flipRelTorque = (-dodgeDir.y, dodgeDir.x)`
                    // with `dodgeDir = (-pitch, yaw + roll)` normalized —
                    // local (forward, right). The torque starts on the
                    // *next* step (RB-PHYSICS-001-FR-080) and is what
                    // flip-cancel below spends on a later press.
                    *dodge_flip = Some(DodgeFlip {
                        rel_torque: (-norm_dodge_roll, dodge_forward),
                        elapsed: 0.0,
                    });
                } else {
                    // Same fixed-magnitude impulse as the ground jump — reusing
                    // JUMP_SPEED rather than a second, separately-calibrated
                    // constant, since this port has no public reference for a
                    // distinct double-jump speed either.
                    car.apply_impulse(Vec3::new(0.0, 0.0, JUMP_SPEED * car.mass()), Vec3::ZERO);
                    // No flip to cancel — and explicitly clearing this
                    // (rather than leaving it alone) prevents a stale flip
                    // from an earlier, already-landed-from dodge from
                    // leaking into this unrelated plain double jump.
                    *dodge_flip = None;
                }
                *double_jump_available = false;
            }
            // A further fresh press with the double jump spent and no wall
            // to push off does nothing — RocketSim's `hasFlipped` /
            // `hasDoubleJumped` make `canUse` false. This port's former
            // jump-press flip cancel (RB-PHYSICS-001-FR-016) lived here;
            // the real pitch-hold cancel above replaced it
            // (RB-PHYSICS-001-FR-080 step (c)).
        }

        // RB-PHYSICS-001-FR-080: advance the flip clock and bleed vertical
        // speed, the tail of RocketSim's `_UpdateDoubleJumpOrFlip` — after
        // this step's own dodge press above, so a fresh dodge leaves here
        // with `elapsed == dt` and its torque starts next step. `elapsed`
        // keeps counting past FLIP_TORQUE_TIME (RocketSim: "Increment flip
        // time even after we are done flipping ... for
        // FLIP_PITCHLOCK_EXTRA_TIME to work") until landing clears it.
        if let Some(flip) = dodge_flip.as_mut() {
            flip.elapsed += dt;
            if flip.elapsed <= FLIP_TORQUE_TIME
                && flip.elapsed >= FLIP_Z_DAMP_START
                && (car.linear_velocity.z < 0.0 || flip.elapsed < FLIP_Z_DAMP_END)
            {
                car.linear_velocity.z *= (1.0 - FLIP_Z_DAMP_120).powf(tick_scale);
            }
        }
    }

    if input.boost && *boost_amount > 0.0 {
        let forward_speed = car.linear_velocity.dot(&forward);
        if forward_speed < MAX_CAR_SPEED {
            // RB-PHYSICS-001-FR-056: real Rocket League's own boost
            // acceleration is genuinely higher airborne than grounded —
            // `on_ground` gates which magnitude applies, not whether
            // boost applies at all (it always does, regardless of ground
            // contact, per this function's own doc comment above).
            let boost_acceleration = if on_ground {
                BOOST_ACCELERATION_GROUND
            } else {
                BOOST_ACCELERATION_AIR
            };
            car.apply_central_force(forward * (boost_acceleration * car.mass()));
        }
        // Held boost drains the tank even when the force above didn't
        // apply (e.g. already at MAX_CAR_SPEED, or pushing into a wall) —
        // matching real Rocket League, where holding boost costs fuel
        // regardless of whether it's doing anything.
        *boost_amount = (*boost_amount - BOOST_CONSUMPTION_RATE * dt).max(0.0);
    }
}

/// The real air-control damping angular acceleration for `car`'s current
/// spin (`RB-PHYSICS-001-FR-071`): each body-axis component of
/// `angular_velocity`, times that axis' `AIR_CONTROL_*_DAMPING`, times `1 -
/// |stick input|` for pitch (`effective_pitch`, already zeroed under the
/// post-flip pitch lock) and yaw (roll has no such factor), summed, negated,
/// and scaled by `CAR_TORQUE_SCALE` — RocketSim's `_UpdateAirTorque`
/// `damping` vector (`dirPitch_right * dampPitch + dirYaw_up * dampYaw +
/// dirRoll_forward * dampRoll`; the negated pitch/roll axis directions
/// cancel out, since each is used both to project and to reconstruct),
/// applied through the same inertia-cancelled path as the stick torque.
/// Apply once per airborne step via `apply_angular_acceleration`.
fn air_control_damping(car: &RigidBody, effective_pitch: f32, yaw: f32) -> Vec3 {
    let forward = forward_axis(car);
    let right = right_axis(car);
    let up = up_axis(car);
    let spin = car.angular_velocity;
    -(right * (spin.dot(&right) * AIR_CONTROL_PITCH_DAMPING * (1.0 - effective_pitch.abs()))
        + up * (spin.dot(&up) * AIR_CONTROL_YAW_DAMPING * (1.0 - yaw.abs()))
        + forward * (spin.dot(&forward) * AIR_CONTROL_ROLL_DAMPING))
        * CAR_TORQUE_SCALE
}

/// Scales `car.angular_velocity` back down to `MAX_CAR_ANGULAR_SPEED` if
/// its length exceeds it, preserving direction — a no-op otherwise. Call
/// once per step, at the very end — *after* `integrate::integrate_transform`
/// — where RocketSim's `Arena::Step` calls `Car::_FinishPhysicsTick`
/// (after `stepSimulation`). The placement is load-bearing since
/// `RB-PHYSICS-001-FR-080` step (c): the transform integrates with this
/// step's *unclamped* angular velocity, so a car under flip torque turns
/// by `|ω_stored + Δω| ≈ 7.6` rad/s per tick while its stored angular
/// velocity reads `5.5` — exactly what the real capture records (its
/// orientation advances `7.58` rad/s per tick through the flip window at a
/// reported `|ω| = 5.50`). Clamping before the transform integrated, as
/// this port did from `RB-PHYSICS-001-FR-057` until then, under-rotated
/// every flip by about `2` rad/s.
pub fn clamp_angular_speed(car: &mut RigidBody) {
    let speed = car.angular_velocity.length();
    if speed > MAX_CAR_ANGULAR_SPEED {
        car.angular_velocity *= MAX_CAR_ANGULAR_SPEED / speed;
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::integrate;

    const DEFAULT_TEST_FRICTION: f32 = 0.5;

    fn car() -> RigidBody {
        RigidBody::standard_car(Vec3::ZERO)
    }

    fn step_with_input(
        car: &mut RigidBody,
        input: &ControllerInput,
        on_ground: bool,
        boost_amount: &mut f32,
        dt: f32,
    ) {
        let mut jump_held = false;
        step_with_input_and_jump_state(car, input, on_ground, boost_amount, &mut jump_held, dt);
    }

    fn step_with_input_and_jump_state(
        car: &mut RigidBody,
        input: &ControllerInput,
        on_ground: bool,
        boost_amount: &mut f32,
        jump_held: &mut bool,
        dt: f32,
    ) {
        let mut double_jump_available = true;
        step_with_input_and_double_jump_state(
            car,
            input,
            on_ground,
            boost_amount,
            jump_held,
            &mut double_jump_available,
            dt,
        );
    }

    fn step_with_input_and_double_jump_state(
        car: &mut RigidBody,
        input: &ControllerInput,
        on_ground: bool,
        boost_amount: &mut f32,
        jump_held: &mut bool,
        double_jump_available: &mut bool,
        dt: f32,
    ) {
        step_with_input_and_wall(
            car,
            input,
            on_ground,
            None,
            boost_amount,
            jump_held,
            double_jump_available,
            dt,
        );
    }

    #[allow(clippy::too_many_arguments)]
    fn step_with_input_and_wall(
        car: &mut RigidBody,
        input: &ControllerInput,
        on_ground: bool,
        wall_normal: Option<Vec3>,
        boost_amount: &mut f32,
        jump_held: &mut bool,
        double_jump_available: &mut bool,
        dt: f32,
    ) {
        let mut jump_hold_time_remaining = 0.0;
        step_with_input_and_hold(
            car,
            input,
            on_ground,
            wall_normal,
            boost_amount,
            jump_held,
            double_jump_available,
            &mut jump_hold_time_remaining,
            dt,
        );
    }

    #[allow(clippy::too_many_arguments)]
    fn step_with_input_and_hold(
        car: &mut RigidBody,
        input: &ControllerInput,
        on_ground: bool,
        wall_normal: Option<Vec3>,
        boost_amount: &mut f32,
        jump_held: &mut bool,
        double_jump_available: &mut bool,
        jump_hold_time_remaining: &mut f32,
        dt: f32,
    ) {
        let mut dodge_flip = None;
        step_with_input_and_dodge_flip(
            car,
            input,
            on_ground,
            wall_normal,
            boost_amount,
            jump_held,
            double_jump_available,
            jump_hold_time_remaining,
            &mut dodge_flip,
            dt,
        );
    }

    #[allow(clippy::too_many_arguments)]
    fn step_with_input_and_dodge_flip(
        car: &mut RigidBody,
        input: &ControllerInput,
        on_ground: bool,
        wall_normal: Option<Vec3>,
        boost_amount: &mut f32,
        jump_held: &mut bool,
        double_jump_available: &mut bool,
        jump_hold_time_remaining: &mut f32,
        dodge_flip: &mut Option<DodgeFlip>,
        dt: f32,
    ) {
        car.clear_forces();
        apply_driven_forces(
            car,
            input,
            on_ground,
            wall_normal,
            boost_amount,
            jump_held,
            double_jump_available,
            jump_hold_time_remaining,
            dodge_flip,
            DEFAULT_TEST_FRICTION,
            dt,
        );
        integrate::integrate_velocities(car, dt);
        clamp_angular_speed(car);
    }

    /// Fires an airborne dodge with `dodge_input` (a fresh jump press, the
    /// double jump still available, no wall) and returns the flip state it
    /// starts — `None` if `dodge_input` didn't actually dodge.
    fn airborne_dodge(
        car: &mut RigidBody,
        dodge_input: &ControllerInput,
        dt: f32,
    ) -> Option<DodgeFlip> {
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let mut hold_remaining = 0.0;
        let mut dodge_flip = None;
        step_with_input_and_dodge_flip(
            car,
            dodge_input,
            false,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut dodge_flip,
            dt,
        );
        dodge_flip
    }

    /// One further airborne step against a persisted flip state, holding
    /// `input` with no fresh jump press (jump reads as already held) and
    /// the double jump already spent — the situation every step after a
    /// dodge is in.
    fn airborne_flip_step(
        car: &mut RigidBody,
        input: &ControllerInput,
        dodge_flip: &mut Option<DodgeFlip>,
        dt: f32,
    ) {
        let mut boost = MAX_BOOST;
        let mut jump_held = true;
        let mut double_jump_available = false;
        let mut hold_remaining = 0.0;
        step_with_input_and_dodge_flip(
            car,
            input,
            false,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            dodge_flip,
            dt,
        );
    }

    /// The real per-tick angular-velocity step a flip's pitch-axis torque
    /// produces: `FLIP_TORQUE_Y / 120` at any `dt`.
    const FLIP_PITCH_STEP_PER_TICK: f32 = FLIP_TORQUE_Y / FLIP_REFERENCE_TICK_RATE;
    /// Likewise for the roll axis: `FLIP_TORQUE_X / 120`.
    const FLIP_ROLL_STEP_PER_TICK: f32 = FLIP_TORQUE_X / FLIP_REFERENCE_TICK_RATE;

    /// One step's real air-control damping Δω on `car`'s *current* spin
    /// with the stick neutral on every axis (RB-PHYSICS-001-FR-071) —
    /// compute it *before* the step, then fold it into an exact per-tick
    /// expectation alongside whatever torque the step applies.
    fn neutral_damping_step(car: &RigidBody, dt: f32) -> Vec3 {
        air_control_damping(car, 0.0, 0.0) * dt
    }

    fn full_throttle() -> ControllerInput {
        ControllerInput {
            throttle: 1.0,
            ..Default::default()
        }
    }

    fn full_steer() -> ControllerInput {
        ControllerInput {
            steer: 1.0,
            ..Default::default()
        }
    }

    fn full_boost() -> ControllerInput {
        ControllerInput {
            boost: true,
            ..Default::default()
        }
    }

    fn full_handbrake() -> ControllerInput {
        ControllerInput {
            handbrake: true,
            ..Default::default()
        }
    }

    fn full_jump() -> ControllerInput {
        ControllerInput {
            jump: true,
            ..Default::default()
        }
    }

    fn full_pitch() -> ControllerInput {
        ControllerInput {
            pitch: Some(1.0),
            ..Default::default()
        }
    }

    /// Pitch fully *forward* (`-1`, nose down — real Rocket League's own
    /// recorded stick convention); `full_pitch` above is fully *back*
    /// (`+1`), the input that cancels a forward flip.
    fn full_pitch_forward() -> ControllerInput {
        ControllerInput {
            pitch: Some(-1.0),
            ..Default::default()
        }
    }

    fn full_yaw() -> ControllerInput {
        ControllerInput {
            yaw: Some(1.0),
            ..Default::default()
        }
    }

    fn full_roll() -> ControllerInput {
        ControllerInput {
            roll: Some(1.0),
            ..Default::default()
        }
    }

    #[test]
    fn neutral_input_applies_no_force_or_torque() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        step_with_input(
            &mut c,
            &ControllerInput::default(),
            true,
            &mut boost,
            1.0 / 60.0,
        );
        assert_eq!(c.linear_velocity, Vec3::ZERO);
        assert_eq!(c.angular_velocity, Vec3::ZERO);
        assert_eq!(boost, MAX_BOOST, "unused boost shouldn't drain");
        assert_eq!(
            c.friction, DEFAULT_TEST_FRICTION,
            "no handbrake input shouldn't touch friction"
        );
    }

    #[test]
    fn throttle_accelerates_a_grounded_car_forward() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        for _ in 0..60 {
            step_with_input(&mut c, &full_throttle(), true, &mut boost, 1.0 / 60.0);
        }
        assert!(
            c.linear_velocity.x > 0.0,
            "expected forward speed to increase, got {}",
            c.linear_velocity.x
        );
        assert!((c.linear_velocity.y).abs() < 1e-4);
        assert!((c.linear_velocity.z).abs() < 1e-4);
    }

    #[test]
    fn throttle_has_no_effect_while_airborne() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        for _ in 0..60 {
            step_with_input(&mut c, &full_throttle(), false, &mut boost, 1.0 / 60.0);
        }
        assert_eq!(
            c.linear_velocity.x, 0.0,
            "airborne throttle shouldn't add forward speed"
        );
    }

    #[test]
    fn throttle_stops_accelerating_at_unboosted_max_speed() {
        // RB-PHYSICS-001-FR-031: throttle's own cap is UNBOOSTED_MAX_CAR_SPEED
        // (1410 uu/s), not the boosted MAX_CAR_SPEED (2300) — see
        // UNBOOSTED_MAX_CAR_SPEED's own doc comment for why these are now
        // two separate constants.
        let mut c = car();
        let mut boost = MAX_BOOST;
        c.linear_velocity = Vec3::new(UNBOOSTED_MAX_CAR_SPEED, 0.0, 0.0);
        step_with_input(&mut c, &full_throttle(), true, &mut boost, 1.0 / 60.0);
        assert!(
            (c.linear_velocity.x - UNBOOSTED_MAX_CAR_SPEED).abs() < 1e-4,
            "expected throttle to stop pushing past UNBOOSTED_MAX_CAR_SPEED, got {}",
            c.linear_velocity.x
        );
    }

    #[test]
    fn throttle_alone_cannot_reach_the_boosted_top_speed() {
        // The real bug RB-PHYSICS-001-FR-031 fixed: before the audit,
        // throttle shared MAX_CAR_SPEED (2300, the *boosted* cap) as its
        // own ceiling, letting a car reach boosted top speed on throttle
        // alone. Held throttle for a generous 20 simulated seconds (no
        // drag to fight against) should now plateau at UNBOOSTED_MAX_CAR_SPEED,
        // well short of MAX_CAR_SPEED.
        let mut c = car();
        let mut boost = MAX_BOOST;
        let dt = 1.0 / 60.0;
        for _ in 0..(20.0 / dt) as u32 {
            step_with_input(&mut c, &full_throttle(), true, &mut boost, dt);
        }
        assert!(
            // Since RB-PHYSICS-001-FR-058, this is a genuine taper rather
            // than a hard per-step cutoff — acceleration tapers smoothly to
            // zero over the last 10 uu/s below UNBOOSTED_MAX_CAR_SPEED, so
            // overshoot here should in practice be far smaller than a full
            // THROTTLE_ACCELERATION*dt step; the generous +30.0 margin is
            // kept only to avoid a brittle exact-value assertion.
            c.linear_velocity.x <= UNBOOSTED_MAX_CAR_SPEED + 30.0,
            "expected throttle alone to cap out at UNBOOSTED_MAX_CAR_SPEED, got {}",
            c.linear_velocity.x
        );
        assert!(
            c.linear_velocity.x < MAX_CAR_SPEED - 1.0,
            "expected throttle alone to stay well short of the boosted MAX_CAR_SPEED, got {}",
            c.linear_velocity.x
        );
    }

    #[test]
    fn drive_speed_taper_matches_the_real_curve_breakpoints_exactly() {
        // RB-PHYSICS-001-FR-058: RocketSim's own DRIVE_SPEED_TORQUE_FACTOR_CURVE
        // is (0, 1.0), (1400, 0.1), (1410, 0.0) — confirmed exact against
        // its own RLConst.h.
        assert_eq!(drive_speed_taper(0.0), 1.0);
        assert!(
            (drive_speed_taper(1400.0) - 0.1).abs() < 1e-6,
            "expected the 1400 uu/s breakpoint to be 0.1, got {}",
            drive_speed_taper(1400.0)
        );
        assert_eq!(drive_speed_taper(UNBOOSTED_MAX_CAR_SPEED), 0.0);
        // Linear interpolation partway along each segment.
        assert!(
            (drive_speed_taper(700.0) - 0.55).abs() < 1e-4,
            "expected the midpoint of the first segment (0, 1.0)-(1400, 0.1) \
             to interpolate to 0.55, got {}",
            drive_speed_taper(700.0)
        );
        assert!(
            (drive_speed_taper(1405.0) - 0.05).abs() < 1e-4,
            "expected the midpoint of the final segment (1400, 0.1)-(1410, 0.0) \
             to interpolate to 0.05, got {}",
            drive_speed_taper(1405.0)
        );
        // Clamped outside the curve's own domain in both directions.
        assert_eq!(
            drive_speed_taper(-100.0),
            1.0,
            "expected a negative (not-yet-moving-this-way) input to clamp to full torque"
        );
        assert_eq!(
            drive_speed_taper(2000.0),
            0.0,
            "expected a speed past the curve's own domain to clamp to zero"
        );
    }

    #[test]
    fn throttle_acceleration_tapers_well_before_reaching_unboosted_max_speed() {
        // RB-PHYSICS-001-FR-058: before this requirement, throttle applied
        // THROTTLE_ACCELERATION at full strength right up to a hard cutoff
        // at UNBOOSTED_MAX_CAR_SPEED. Real Rocket League's own curve is
        // already down to 10% strength at 1400 uu/s (10 short of the cap)
        // — this test would fail (see a ~1600 uu/s^2-scale delta instead)
        // without the taper.
        let mut c = car();
        let mut boost = MAX_BOOST;
        let dt = 1.0 / 60.0;
        c.linear_velocity = Vec3::new(1400.0, 0.0, 0.0);
        step_with_input(&mut c, &full_throttle(), true, &mut boost, dt);
        let delta = c.linear_velocity.x - 1400.0;
        let expected_full_strength_delta = THROTTLE_ACCELERATION * dt;
        assert!(
            (delta - 0.1 * expected_full_strength_delta).abs() < 1e-3,
            "expected the delta at 1400 uu/s to be ~10% of a full-strength step \
             ({}), got {}",
            0.1 * expected_full_strength_delta,
            delta
        );
    }

    #[test]
    fn reverse_throttle_accelerates_backward() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let input = ControllerInput {
            throttle: -1.0,
            ..Default::default()
        };
        for _ in 0..60 {
            step_with_input(&mut c, &input, true, &mut boost, 1.0 / 60.0);
        }
        assert!(
            c.linear_velocity.x < 0.0,
            "expected reverse throttle to push backward, got {}",
            c.linear_velocity.x
        );
    }

    #[test]
    fn steer_has_no_effect_on_a_stationary_car() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        step_with_input(&mut c, &full_steer(), true, &mut boost, 1.0 / 60.0);
        assert_eq!(
            c.angular_velocity,
            Vec3::ZERO,
            "a parked car shouldn't be able to turn in place"
        );
    }

    #[test]
    fn steer_yaws_a_moving_car() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        c.linear_velocity = Vec3::new(1000.0, 0.0, 0.0);
        step_with_input(&mut c, &full_steer(), true, &mut boost, 1.0 / 60.0);
        assert!(
            c.angular_velocity.z.abs() > 0.0,
            "expected a moving car to yaw under full steer, got {:?}",
            c.angular_velocity
        );
    }

    #[test]
    fn opposite_steer_yaws_the_opposite_way() {
        let mut left = car();
        let mut left_boost = MAX_BOOST;
        left.linear_velocity = Vec3::new(1000.0, 0.0, 0.0);
        step_with_input(&mut left, &full_steer(), true, &mut left_boost, 1.0 / 60.0);

        let mut right = car();
        let mut right_boost = MAX_BOOST;
        right.linear_velocity = Vec3::new(1000.0, 0.0, 0.0);
        let opposite = ControllerInput {
            steer: -1.0,
            ..Default::default()
        };
        step_with_input(&mut right, &opposite, true, &mut right_boost, 1.0 / 60.0);

        assert!(left.angular_velocity.z * right.angular_velocity.z < 0.0);
    }

    #[test]
    fn boost_accelerates_a_car_regardless_of_ground_contact() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        for _ in 0..60 {
            step_with_input(&mut c, &full_boost(), false, &mut boost, 1.0 / 60.0);
        }
        assert!(
            c.linear_velocity.x > 0.0,
            "expected boost to accelerate an airborne car, got {}",
            c.linear_velocity.x
        );
    }

    #[test]
    fn boost_accelerates_an_airborne_car_faster_than_a_grounded_one() {
        // RB-PHYSICS-001-FR-056: real Rocket League's own boost
        // acceleration is genuinely higher airborne than grounded
        // (RocketSim's own RLConst.h: BOOST_ACCEL_AIR = 3175/3 vs
        // BOOST_ACCEL_GROUND = 2975/3) -- this port's own earlier single
        // flat BOOST_ACCELERATION constant collapsed both into the
        // grounded number, understating airborne boost. One step's worth
        // of full boost from a dead stop produces a velocity delta of
        // exactly `boost_acceleration * dt` regardless of mass (force is
        // `boost_acceleration * mass`, so it cancels on integration),
        // making the exact ratio between the two directly checkable.
        let mut grounded = car();
        let mut grounded_boost = MAX_BOOST;
        step_with_input(
            &mut grounded,
            &full_boost(),
            true,
            &mut grounded_boost,
            1.0 / 60.0,
        );

        let mut airborne = car();
        let mut airborne_boost = MAX_BOOST;
        step_with_input(
            &mut airborne,
            &full_boost(),
            false,
            &mut airborne_boost,
            1.0 / 60.0,
        );

        assert!(
            airborne.linear_velocity.x > grounded.linear_velocity.x,
            "expected airborne boost ({}) to accelerate faster than grounded boost ({})",
            airborne.linear_velocity.x,
            grounded.linear_velocity.x
        );
        let ratio = airborne.linear_velocity.x / grounded.linear_velocity.x;
        let expected_ratio = BOOST_ACCELERATION_AIR / BOOST_ACCELERATION_GROUND;
        assert!(
            (ratio - expected_ratio).abs() < 1e-4,
            "expected the airborne/grounded ratio to match RocketSim's own \
             BOOST_ACCEL_AIR/BOOST_ACCEL_GROUND ratio ({expected_ratio}), got {ratio}"
        );
    }

    #[test]
    fn boost_drains_the_tank_over_time_and_clamps_at_zero() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        for _ in 0..600 {
            step_with_input(&mut c, &full_boost(), false, &mut boost, 1.0 / 60.0);
        }
        assert_eq!(boost, 0.0, "expected a full tank to run out within 10s");
    }

    #[test]
    fn boost_has_no_effect_when_the_tank_is_empty() {
        let mut c = car();
        let mut boost = 0.0;
        step_with_input(&mut c, &full_boost(), false, &mut boost, 1.0 / 60.0);
        assert_eq!(c.linear_velocity, Vec3::ZERO);
        assert_eq!(boost, 0.0);
    }

    #[test]
    fn boost_still_drains_at_max_speed_even_though_it_stops_accelerating() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        c.linear_velocity = Vec3::new(MAX_CAR_SPEED, 0.0, 0.0);
        step_with_input(&mut c, &full_boost(), false, &mut boost, 1.0 / 60.0);
        assert!(
            (c.linear_velocity.x - MAX_CAR_SPEED).abs() < 1e-4,
            "expected boost to stop pushing past MAX_CAR_SPEED, got {}",
            c.linear_velocity.x
        );
        assert!(
            boost < MAX_BOOST,
            "expected held boost to still drain even at max speed"
        );
    }

    #[test]
    fn handbrake_reduces_friction_while_grounded() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        step_with_input(&mut c, &full_handbrake(), true, &mut boost, 1.0 / 60.0);
        assert_eq!(
            c.friction,
            DEFAULT_TEST_FRICTION * HANDBRAKE_FRICTION_MULTIPLIER,
            "expected handbrake to reduce friction below its base value"
        );
    }

    #[test]
    fn handbrake_has_no_effect_on_friction_while_airborne() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        step_with_input(&mut c, &full_handbrake(), false, &mut boost, 1.0 / 60.0);
        assert_eq!(
            c.friction, DEFAULT_TEST_FRICTION,
            "airborne handbrake shouldn't touch friction — no wheels to lock"
        );
    }

    #[test]
    fn releasing_handbrake_restores_friction() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        step_with_input(&mut c, &full_handbrake(), true, &mut boost, 1.0 / 60.0);
        assert!(
            c.friction < DEFAULT_TEST_FRICTION,
            "handbrake should have engaged"
        );
        step_with_input(
            &mut c,
            &ControllerInput::default(),
            true,
            &mut boost,
            1.0 / 60.0,
        );
        assert_eq!(
            c.friction, DEFAULT_TEST_FRICTION,
            "releasing handbrake should restore the car's base friction"
        );
    }

    #[test]
    fn jump_gives_a_grounded_car_upward_velocity() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        step_with_input(&mut c, &full_jump(), true, &mut boost, 1.0 / 60.0);
        assert!(
            (c.linear_velocity.z - JUMP_SPEED).abs() < 1.0,
            "expected roughly JUMP_SPEED upward velocity, got {}",
            c.linear_velocity.z
        );
    }

    #[test]
    fn holding_jump_does_not_refire_every_step() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        step_with_input_and_jump_state(
            &mut c,
            &full_jump(),
            true,
            &mut boost,
            &mut jump_held,
            1.0 / 60.0,
        );
        let velocity_after_first_press = c.linear_velocity.z;
        // Still held, still (nominally) grounded — a second call with the
        // same jump_held state must not add a second impulse.
        step_with_input_and_jump_state(
            &mut c,
            &full_jump(),
            true,
            &mut boost,
            &mut jump_held,
            1.0 / 60.0,
        );
        assert!(
            c.linear_velocity.z <= velocity_after_first_press + 1.0,
            "expected holding jump to not re-fire a second impulse, \
             velocity after first press={velocity_after_first_press}, after second={}",
            c.linear_velocity.z
        );
    }

    #[test]
    fn releasing_and_repressing_jump_fires_again() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        step_with_input_and_jump_state(
            &mut c,
            &full_jump(),
            true,
            &mut boost,
            &mut jump_held,
            1.0 / 60.0,
        );
        let velocity_after_first_press = c.linear_velocity.z;
        // Release, then press again — this must fire a second impulse.
        step_with_input_and_jump_state(
            &mut c,
            &ControllerInput::default(),
            true,
            &mut boost,
            &mut jump_held,
            1.0 / 60.0,
        );
        step_with_input_and_jump_state(
            &mut c,
            &full_jump(),
            true,
            &mut boost,
            &mut jump_held,
            1.0 / 60.0,
        );
        assert!(
            c.linear_velocity.z > velocity_after_first_press,
            "expected releasing then re-pressing jump to fire again, \
             velocity after first press={velocity_after_first_press}, after second press={}",
            c.linear_velocity.z
        );
    }

    #[test]
    fn double_jump_gives_an_airborne_car_upward_velocity_when_available() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        // step_with_input's throwaway jump_held/double_jump_available both
        // start fresh (unheld, available), so a single airborne jump press
        // here is exactly the "double jump available" case.
        step_with_input(&mut c, &full_jump(), false, &mut boost, 1.0 / 60.0);
        assert!(
            (c.linear_velocity.z - JUMP_SPEED).abs() < 1.0,
            "expected roughly JUMP_SPEED upward velocity from an available double jump, got {}",
            c.linear_velocity.z
        );
    }

    #[test]
    fn double_jump_has_no_effect_when_unavailable() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = false;
        step_with_input_and_double_jump_state(
            &mut c,
            &full_jump(),
            false,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        assert_eq!(
            c.linear_velocity.z, 0.0,
            "expected an unavailable double jump to add no upward velocity"
        );
    }

    #[test]
    fn double_jump_is_consumed_after_use_and_does_not_refire() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        step_with_input_and_double_jump_state(
            &mut c,
            &full_jump(),
            false,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        let velocity_after_double_jump = c.linear_velocity.z;
        assert!(
            !double_jump_available,
            "expected using the double jump to consume it"
        );

        // Release, then press again while still airborne — must not fire a
        // second impulse now that the double jump is spent.
        step_with_input_and_double_jump_state(
            &mut c,
            &ControllerInput::default(),
            false,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        step_with_input_and_double_jump_state(
            &mut c,
            &full_jump(),
            false,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        assert!(
            (c.linear_velocity.z - velocity_after_double_jump).abs() < 1.0,
            "expected a spent double jump to not refire, velocity after first use=\
             {velocity_after_double_jump}, after second press={}",
            c.linear_velocity.z
        );
    }

    #[test]
    fn landing_restores_double_jump_availability() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = false;
        step_with_input_and_double_jump_state(
            &mut c,
            &ControllerInput::default(),
            true,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        assert!(
            double_jump_available,
            "expected touching the ground to restore the double jump"
        );
    }

    #[test]
    fn wall_jump_pushes_an_airborne_car_outward_and_upward() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let wall_normal = Vec3::new(1.0, 0.0, 0.0);
        step_with_input_and_wall(
            &mut c,
            &full_jump(),
            false,
            Some(wall_normal),
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        assert!(
            (c.linear_velocity.x - WALL_JUMP_HORIZONTAL_SPEED).abs() < 1.0,
            "expected roughly WALL_JUMP_HORIZONTAL_SPEED outward velocity along the wall normal, \
             got {}",
            c.linear_velocity.x
        );
        assert!(
            (c.linear_velocity.z - JUMP_SPEED).abs() < 1.0,
            "expected roughly JUMP_SPEED upward velocity, got {}",
            c.linear_velocity.z
        );
    }

    #[test]
    fn wall_jump_has_no_effect_while_grounded() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        step_with_input_and_wall(
            &mut c,
            &full_jump(),
            true,
            Some(Vec3::new(1.0, 0.0, 0.0)),
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        assert!(
            (c.linear_velocity.z - JUMP_SPEED).abs() < 1.0,
            "expected the ordinary ground jump, not a wall-jump push-off, got {:?}",
            c.linear_velocity
        );
        assert_eq!(
            c.linear_velocity.x, 0.0,
            "grounded jump shouldn't apply any wall push-off even if wall_normal is Some"
        );
    }

    #[test]
    fn wall_jump_takes_priority_over_double_jump_without_consuming_it() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        step_with_input_and_wall(
            &mut c,
            &full_jump(),
            false,
            Some(Vec3::new(1.0, 0.0, 0.0)),
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        assert!(
            c.linear_velocity.x > 0.0,
            "expected a wall jump (outward velocity), not a plain double jump, got {:?}",
            c.linear_velocity
        );
        assert!(
            double_jump_available,
            "expected a wall jump to leave the double jump available (not consume it)"
        );
    }

    #[test]
    fn wall_contact_restores_double_jump_availability() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = false;
        step_with_input_and_wall(
            &mut c,
            &ControllerInput::default(),
            false,
            Some(Vec3::new(1.0, 0.0, 0.0)),
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        assert!(
            double_jump_available,
            "expected touching a wall to restore the double jump, matching real Rocket \
             League's any-surface-contact-refills-your-second-jump rule"
        );
    }

    #[test]
    fn dodge_gives_forward_velocity_and_spin_when_pitched_in_the_air() {
        // RB-PHYSICS-001-FR-079: `pitch = -1` (stick forward) is a forward
        // flip, matching real Rocket League's own recorded stick convention
        // (RocketSim's `dodgeDir.x = -controls.pitch`); the spin about the
        // right axis is positive (nose down first, `flipRelTorque.y =
        // dodgeDir.x`). RB-PHYSICS-001-FR-080: that spin is the real
        // FLIP_TORQUE_Y torque starting on the step *after* the dodge, not
        // a kick on the dodge's own step.
        let dt = 1.0 / 120.0;
        let mut c = car();
        let input = ControllerInput {
            jump: true,
            pitch: Some(-1.0),
            ..Default::default()
        };
        let mut flip = airborne_dodge(&mut c, &input, dt);
        assert!(
            (c.linear_velocity.x - DODGE_SPEED).abs() < 1.0,
            "expected roughly DODGE_SPEED forward velocity, got {}",
            c.linear_velocity.x
        );
        assert_eq!(
            flip,
            Some(DodgeFlip {
                rel_torque: (0.0, 1.0),
                elapsed: dt
            }),
            "expected a forward dodge to start a flip about +right with one step elapsed"
        );
        // The dodge's own step gets only that step's ordinary stick
        // air-control pitch (RocketSim runs air torque before the flip
        // begins) — a small fraction of one flip tick, not a kick.
        let air_control_only = c.angular_velocity;
        assert!(
            air_control_only.y > 0.0 && air_control_only.y < 0.2,
            "expected only the step's own air-control pitch on the dodge step, got {:?}",
            air_control_only
        );

        let damp = neutral_damping_step(&c, dt);
        airborne_flip_step(&mut c, &ControllerInput::default(), &mut flip, dt);
        assert!(
            (c.angular_velocity.y - air_control_only.y - damp.y - FLIP_PITCH_STEP_PER_TICK).abs()
                < 1e-3,
            "expected FLIP_TORQUE_Y / 120 rad/s about +right after one tick of flip torque, got {}",
            c.angular_velocity.y
        );
        assert_eq!(c.angular_velocity.x, 0.0);
        assert_eq!(c.angular_velocity.z, 0.0);
    }

    #[test]
    fn dodge_gives_lateral_velocity_and_spin_when_rolled_in_the_air() {
        let dt = 1.0 / 120.0;
        let mut c = car();
        let input = ControllerInput {
            jump: true,
            roll: Some(1.0),
            ..Default::default()
        };
        let mut flip = airborne_dodge(&mut c, &input, dt);
        assert!(
            (c.linear_velocity.y - DODGE_SPEED).abs() < 1.0,
            "expected roughly DODGE_SPEED lateral velocity, got {}",
            c.linear_velocity.y
        );
        // RB-PHYSICS-001-FR-079: a right dodge (`roll = +1`) spins about
        // the *negative* forward axis (right side down first), matching
        // RocketSim's `flipRelTorque.x = -dodgeDir.y`; RB-PHYSICS-001-FR-080:
        // at the real FLIP_TORQUE_X per-tick rate, from the next step.
        assert_eq!(
            flip,
            Some(DodgeFlip {
                rel_torque: (-1.0, 0.0),
                elapsed: dt
            })
        );
        let air_control_only = c.angular_velocity;
        assert!(
            air_control_only.x < 0.0 && air_control_only.x > -0.5,
            "expected only the step's own air-control roll on the dodge step, got {:?}",
            air_control_only
        );

        let damp = neutral_damping_step(&c, dt);
        airborne_flip_step(&mut c, &ControllerInput::default(), &mut flip, dt);
        assert!(
            (c.angular_velocity.x - air_control_only.x - damp.x + FLIP_ROLL_STEP_PER_TICK).abs()
                < 1e-3,
            "expected -FLIP_TORQUE_X / 120 rad/s about the forward axis after one tick, got {}",
            c.angular_velocity.x
        );
        assert_eq!(c.angular_velocity.y, 0.0);
    }

    #[test]
    fn a_flips_torque_reaches_the_angular_speed_cap_within_three_ticks_and_holds_it_through_flip_torque_time(
    ) {
        // RB-PHYSICS-001-FR-080: 224 / 120 ≈ 1.87 rad/s per tick reaches
        // MAX_CAR_ANGULAR_SPEED = 5.5 on the third tick, and
        // clamp_angular_speed then holds it there for the rest of the
        // 0.65 s window — the isolated dodge-derailment capture's own
        // `3.40 → 5.22 → 5.50, then 5.50 every tick` trace.
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut flip = airborne_dodge(
            &mut c,
            &ControllerInput {
                jump: true,
                pitch: Some(-1.0),
                ..Default::default()
            },
            dt,
        );
        let neutral = ControllerInput::default();
        // The dodge step's own air-control pitch (≈0.1 rad/s).
        let head_start = c.angular_velocity.y;

        let damp = neutral_damping_step(&c, dt).y;
        airborne_flip_step(&mut c, &neutral, &mut flip, dt);
        assert!((c.angular_velocity.y - head_start - damp - FLIP_PITCH_STEP_PER_TICK).abs() < 1e-3);
        let after_one = c.angular_velocity.y;
        let damp = neutral_damping_step(&c, dt).y;
        airborne_flip_step(&mut c, &neutral, &mut flip, dt);
        assert!((c.angular_velocity.y - after_one - damp - FLIP_PITCH_STEP_PER_TICK).abs() < 1e-3);
        airborne_flip_step(&mut c, &neutral, &mut flip, dt);
        assert!(
            (c.angular_velocity.y - MAX_CAR_ANGULAR_SPEED).abs() < 1e-3,
            "expected the third tick to land on the cap, got {}",
            c.angular_velocity.y
        );

        // Steps 4..=76 are all still inside the window (elapsed before
        // each is at most 76/120 ≈ 0.633 < 0.65): torque every step, held
        // at the cap.
        for step in 4..=76 {
            airborne_flip_step(&mut c, &neutral, &mut flip, dt);
            assert!(
                c.total_angular_acceleration().length() > 0.0,
                "expected flip torque on step {step}, elapsed {:?}",
                flip
            );
            assert!(
                (c.angular_velocity.length() - MAX_CAR_ANGULAR_SPEED).abs() < 1e-3,
                "expected |ω| held at the cap on step {step}, got {}",
                c.angular_velocity.length()
            );
        }

        // Well past the window (elapsed ≥ 80/120 ≈ 0.667): no more flip
        // torque — only the real air-control damping (RB-PHYSICS-001-FR-071)
        // acts on the spin now.
        for _ in 77..80 {
            airborne_flip_step(&mut c, &neutral, &mut flip, dt);
        }
        let held = c.angular_velocity;
        let damp = neutral_damping_step(&c, dt);
        airborne_flip_step(&mut c, &neutral, &mut flip, dt);
        assert!(
            (c.total_angular_acceleration() * dt - damp).length() < 1e-5,
            "expected only damping after FLIP_TORQUE_TIME, got {:?}",
            c.total_angular_acceleration()
        );
        assert!((c.angular_velocity - held - damp).length() < 1e-5);
        assert!(
            damp.y < 0.0,
            "expected the damping to bleed the flip's spin"
        );
        assert!(
            flip.is_some_and(|f| f.elapsed > FLIP_TORQUE_TIME),
            "expected the flip state to persist past the window (for the pitch lock), got {:?}",
            flip
        );
    }

    #[test]
    fn flip_torque_is_per_tick_not_per_second() {
        // RB-PHYSICS-001-FR-080: RocketSim divides flipRelTorque by
        // tickTimeScale, so a 1/60 s step gets the same Δω as a 1/120 s
        // one — the flip is tick-rate invariant per *tick*, not per second.
        let input = ControllerInput {
            jump: true,
            pitch: Some(-1.0),
            ..Default::default()
        };
        for dt in [1.0 / 120.0, 1.0 / 60.0] {
            let mut c = car();
            let mut flip = airborne_dodge(&mut c, &input, dt);
            let head_start = c.angular_velocity.y;
            let damp = neutral_damping_step(&c, dt).y;
            airborne_flip_step(&mut c, &ControllerInput::default(), &mut flip, dt);
            assert!(
                (c.angular_velocity.y - head_start - damp - FLIP_PITCH_STEP_PER_TICK).abs() < 1e-3,
                "expected FLIP_TORQUE_Y / 120 per tick at dt={dt}, got {}",
                c.angular_velocity.y - head_start - damp
            );
        }
    }

    #[test]
    fn yaw_and_roll_air_control_stay_live_mid_flip() {
        // RB-PHYSICS-001-FR-080: the real capture keeps stick yaw/roll
        // active during the flip (against RocketSim's `doAirControl =
        // false`), so full roll input adds its ordinary air-control roll on
        // top of the flip's own torque.
        let dt = 1.0 / 120.0;
        let forward_dodge = ControllerInput {
            jump: true,
            pitch: Some(-1.0),
            ..Default::default()
        };

        let mut c = car();
        let mut flip = airborne_dodge(&mut c, &forward_dodge, dt);
        let head_start = c.angular_velocity.y;
        let damp = neutral_damping_step(&c, dt).y;
        airborne_flip_step(&mut c, &full_roll(), &mut flip, dt);
        let expected_roll = -AIR_CONTROL_ROLL_TORQUE * CAR_TORQUE_SCALE * dt;
        assert!(
            (c.angular_velocity.x - expected_roll).abs() < 1e-4,
            "expected one tick of roll air control mid-flip, got {:?}",
            c.angular_velocity
        );
        assert!((c.angular_velocity.y - head_start - damp - FLIP_PITCH_STEP_PER_TICK).abs() < 1e-3);
    }

    #[test]
    fn a_spinning_car_with_a_neutral_stick_bleeds_spin_at_the_real_per_axis_rates() {
        // RB-PHYSICS-001-FR-071: each body-axis component decays by its own
        // CAR_AIR_CONTROL_DAMPING coefficient through CAR_TORQUE_SCALE —
        // roll (forward, 50) fastest, then pitch (right, 30), then yaw
        // (up, 20). A level car's body axes are the world axes.
        let dt = 1.0 / 120.0;
        let mut c = car();
        c.angular_velocity = Vec3::new(1.0, 1.0, 1.0);
        let mut boost = MAX_BOOST;
        step_with_input(&mut c, &ControllerInput::default(), false, &mut boost, dt);
        let k = CAR_TORQUE_SCALE * dt;
        assert!((c.angular_velocity.x - (1.0 - AIR_CONTROL_ROLL_DAMPING * k)).abs() < 1e-6);
        assert!((c.angular_velocity.y - (1.0 - AIR_CONTROL_PITCH_DAMPING * k)).abs() < 1e-6);
        assert!((c.angular_velocity.z - (1.0 - AIR_CONTROL_YAW_DAMPING * k)).abs() < 1e-6);
    }

    #[test]
    fn a_fully_held_stick_removes_its_own_axis_damping_except_for_roll() {
        // RocketSim's `dampPitch`/`dampYaw` carry a `(1 - |input|)` factor;
        // `dampRoll` doesn't — so full roll input fights its own damping.
        let dt = 1.0 / 120.0;
        let mut c = car();
        c.angular_velocity = Vec3::new(1.0, 1.0, 1.0);
        let mut boost = MAX_BOOST;
        let input = ControllerInput {
            pitch: Some(1.0),
            yaw: Some(1.0),
            roll: Some(1.0),
            ..Default::default()
        };
        step_with_input(&mut c, &input, false, &mut boost, dt);
        let k = CAR_TORQUE_SCALE * dt;
        // pitch = +1 acts about -right: torque only, no damping.
        assert!((c.angular_velocity.y - (1.0 - AIR_CONTROL_PITCH_TORQUE * k)).abs() < 1e-6);
        // yaw = +1 acts about +up: torque only, no damping.
        assert!((c.angular_velocity.z - (1.0 + AIR_CONTROL_YAW_TORQUE * k)).abs() < 1e-6);
        // roll = +1 acts about -forward, *and* the roll damping still bleeds.
        assert!(
            (c.angular_velocity.x
                - (1.0 - AIR_CONTROL_ROLL_DAMPING * k - AIR_CONTROL_ROLL_TORQUE * k))
                .abs()
                < 1e-6
        );
    }

    #[test]
    fn damping_acts_along_the_cars_own_axes_not_the_worlds() {
        // A car rolled 90° has its right axis along world +z, so a world-z
        // spin is a *pitch* rate for it and decays at 30, not yaw's 20.
        let dt = 1.0 / 120.0;
        let mut c = tilted_car();
        c.angular_velocity = Vec3::new(0.0, 0.0, 1.0);
        let mut boost = MAX_BOOST;
        step_with_input(&mut c, &ControllerInput::default(), false, &mut boost, dt);
        let k = CAR_TORQUE_SCALE * dt;
        assert!(
            (c.angular_velocity.z - (1.0 - AIR_CONTROL_PITCH_DAMPING * k)).abs() < 1e-5,
            "expected the pitch coefficient on a body-right spin, got {:?}",
            c.angular_velocity
        );
    }

    #[test]
    fn air_control_damping_does_not_apply_while_grounded() {
        let dt = 1.0 / 120.0;
        let mut c = car();
        c.angular_velocity = Vec3::new(1.0, 1.0, 1.0);
        let mut boost = MAX_BOOST;
        step_with_input(&mut c, &ControllerInput::default(), true, &mut boost, dt);
        assert_eq!(c.angular_velocity, Vec3::new(1.0, 1.0, 1.0));
    }

    #[test]
    fn pitch_stays_locked_out_of_air_control_for_flip_pitchlock_extra_time_after_the_flip() {
        // RB-PHYSICS-001-FR-080: `pitchTorqueScale = 0` until flipTime
        // reaches FLIP_TORQUE_TIME + FLIP_PITCHLOCK_EXTRA_TIME (0.95 s);
        // yaw (and roll) come back the moment the flip torque stops.
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut flip = airborne_dodge(
            &mut c,
            &ControllerInput {
                jump: true,
                pitch: Some(-1.0),
                ..Default::default()
            },
            dt,
        );
        // Run out the flip torque window with a margin (elapsed ≈ 0.7 s).
        for _ in 0..84 {
            airborne_flip_step(&mut c, &ControllerInput::default(), &mut flip, dt);
        }
        assert!(flip.is_some_and(|f| f.elapsed > FLIP_TORQUE_TIME));

        // With pitch locked, a held pitch stick changes nothing: the only
        // acceleration is the real damping (RB-PHYSICS-001-FR-071) — at
        // full strength, since the *effective* pitch is zero.
        let damp = neutral_damping_step(&c, dt);
        airborne_flip_step(&mut c, &full_pitch(), &mut flip, dt);
        assert!(
            (c.total_angular_acceleration() * dt - damp).length() < 1e-5,
            "expected pitch input to still be locked out after the flip torque stopped, got {:?}",
            c.total_angular_acceleration()
        );
        let damp = neutral_damping_step(&c, dt);
        airborne_flip_step(&mut c, &full_yaw(), &mut flip, dt);
        assert!(
            (c.total_angular_acceleration() * dt - damp).length() > 1e-3,
            "expected yaw input to work again as soon as the flip torque stopped"
        );

        // Past the lock (elapsed ≈ 1.0 s): pitch works again.
        for _ in 0..36 {
            airborne_flip_step(&mut c, &ControllerInput::default(), &mut flip, dt);
        }
        assert!(flip.is_some_and(|f| f.elapsed > FLIP_TORQUE_TIME + FLIP_PITCHLOCK_EXTRA_TIME));
        let damp = neutral_damping_step(&c, dt);
        airborne_flip_step(&mut c, &full_pitch(), &mut flip, dt);
        assert!(
            (c.total_angular_acceleration() * dt - damp).length() > 1e-3,
            "expected pitch input to work again after FLIP_PITCHLOCK_EXTRA_TIME"
        );
    }

    #[test]
    fn flip_z_damp_bleeds_vertical_speed_only_inside_its_window() {
        // RB-PHYSICS-001-FR-080: `vz *= (1 - 0.35)^tickTimeScale` each step
        // from FLIP_Z_DAMP_START (0.15 s) through FLIP_TORQUE_TIME — for any
        // vz before FLIP_Z_DAMP_END (0.21 s), only a downward one after.
        let dt = 1.0 / 120.0;
        let neutral = ControllerInput::default();
        let mut c = car();
        c.linear_velocity.z = 100.0;
        let mut flip = airborne_dodge(
            &mut c,
            &ControllerInput {
                jump: true,
                pitch: Some(-1.0),
                ..Default::default()
            },
            dt,
        );

        // Before the window (elapsed after 15 more steps = 16/120 ≈ 0.133).
        for _ in 0..15 {
            airborne_flip_step(&mut c, &neutral, &mut flip, dt);
        }
        assert_eq!(
            c.linear_velocity.z, 100.0,
            "expected no bleed before FLIP_Z_DAMP_START"
        );

        // Squarely inside the unconditional part (elapsed 20/120 ≈ 0.167 →
        // 21/120 = 0.175): one step bleeds ×0.65 even though vz > 0.
        for _ in 0..4 {
            airborne_flip_step(&mut c, &neutral, &mut flip, dt);
        }
        let before = c.linear_velocity.z;
        assert!(
            before < 100.0,
            "expected the bleed to have started, got {before}"
        );
        airborne_flip_step(&mut c, &neutral, &mut flip, dt);
        assert!(
            (c.linear_velocity.z - before * (1.0 - FLIP_Z_DAMP_120)).abs() < 1e-3,
            "expected a ×0.65 bleed per tick, got {} from {before}",
            c.linear_velocity.z
        );

        // Past FLIP_Z_DAMP_END (elapsed ≈ 0.24 s): an upward vz is left
        // alone, a downward one still bleeds.
        for _ in 0..8 {
            airborne_flip_step(&mut c, &neutral, &mut flip, dt);
        }
        c.linear_velocity.z = 50.0;
        airborne_flip_step(&mut c, &neutral, &mut flip, dt);
        assert_eq!(
            c.linear_velocity.z, 50.0,
            "expected an upward vz untouched after FLIP_Z_DAMP_END"
        );
        c.linear_velocity.z = -50.0;
        airborne_flip_step(&mut c, &neutral, &mut flip, dt);
        assert!(
            (c.linear_velocity.z + 50.0 * (1.0 - FLIP_Z_DAMP_120)).abs() < 1e-3,
            "expected a downward vz to keep bleeding until FLIP_TORQUE_TIME, got {}",
            c.linear_velocity.z
        );

        // After the window (elapsed ≈ 0.76 s): nothing, even downward.
        for _ in 0..60 {
            airborne_flip_step(&mut c, &neutral, &mut flip, dt);
        }
        assert!(flip.is_some_and(|f| f.elapsed > FLIP_TORQUE_TIME));
        c.linear_velocity.z = -50.0;
        airborne_flip_step(&mut c, &neutral, &mut flip, dt);
        assert_eq!(
            c.linear_velocity.z, -50.0,
            "expected no bleed after FLIP_TORQUE_TIME"
        );
    }

    #[test]
    fn landing_clears_the_flip_state_and_its_torque() {
        // RB-PHYSICS-001-FR-080: RocketSim's `isOnGround` resets
        // hasFlipped/flipTime — a grounded step ends the flip, pitch lock
        // included, so a later airborne step gets no flip torque.
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = true;
        let mut double_jump_available = false;
        let mut hold_remaining = 0.0;
        let mut flip = airborne_dodge(
            &mut c,
            &ControllerInput {
                jump: true,
                pitch: Some(-1.0),
                ..Default::default()
            },
            dt,
        );
        assert!(flip.is_some());

        step_with_input_and_dodge_flip(
            &mut c,
            &ControllerInput::default(),
            true,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut flip,
            dt,
        );
        assert_eq!(flip, None, "expected landing to clear the flip state");

        let spin_before = c.angular_velocity;
        let damp = neutral_damping_step(&c, dt);
        airborne_flip_step(&mut c, &ControllerInput::default(), &mut flip, dt);
        assert!((c.total_angular_acceleration() * dt - damp).length() < 1e-5);
        assert!((c.angular_velocity - spin_before - damp).length() < 1e-5);
    }

    #[test]
    fn a_wall_jump_dodge_restarts_the_flip_state() {
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = true;
        let mut double_jump_available = false;
        let mut hold_remaining = 0.0;
        let mut flip = airborne_dodge(
            &mut c,
            &ControllerInput {
                jump: true,
                pitch: Some(-1.0),
                ..Default::default()
            },
            dt,
        );
        for _ in 0..10 {
            airborne_flip_step(&mut c, &ControllerInput::default(), &mut flip, dt);
        }
        assert!(flip.is_some_and(|f| f.elapsed > 10.0 * dt));

        // Release, then a left wall-jump dodge (`yaw = -1`) off a +x wall.
        jump_held = false;
        step_with_input_and_dodge_flip(
            &mut c,
            &ControllerInput {
                jump: true,
                yaw: Some(-1.0),
                ..Default::default()
            },
            false,
            Some(Vec3::new(1.0, 0.0, 0.0)),
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut flip,
            dt,
        );
        assert_eq!(
            flip,
            Some(DodgeFlip {
                rel_torque: (1.0, 0.0),
                elapsed: dt
            }),
            "expected the wall-jump dodge to start a fresh left flip about +forward"
        );
    }

    #[test]
    fn small_stick_deflection_below_deadzone_still_gives_a_plain_double_jump() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let input = ControllerInput {
            jump: true,
            pitch: Some(0.05), // below DODGE_DEADZONE (0.1)
            ..Default::default()
        };
        step_with_input_and_double_jump_state(
            &mut c,
            &input,
            false,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        assert_eq!(
            c.linear_velocity.x, 0.0,
            "expected no dodge push-off from a below-deadzone stick deflection"
        );
        assert!(
            (c.linear_velocity.z - JUMP_SPEED).abs() < 1.0,
            "expected a plain double jump instead, got {}",
            c.linear_velocity.z
        );
    }

    #[test]
    fn dodge_consumes_the_double_jump_same_as_a_plain_one() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let input = ControllerInput {
            jump: true,
            pitch: Some(1.0),
            ..Default::default()
        };
        step_with_input_and_double_jump_state(
            &mut c,
            &input,
            false,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        assert!(
            !double_jump_available,
            "expected a dodge to spend the double jump, same as a plain one"
        );
    }

    #[test]
    fn opposite_pitch_dodges_the_opposite_direction() {
        let mut left = car();
        let mut left_boost = MAX_BOOST;
        let mut left_jump_held = false;
        let mut left_double_jump_available = true;
        let forward_input = ControllerInput {
            jump: true,
            pitch: Some(-1.0),
            ..Default::default()
        };
        step_with_input_and_double_jump_state(
            &mut left,
            &forward_input,
            false,
            &mut left_boost,
            &mut left_jump_held,
            &mut left_double_jump_available,
            1.0 / 60.0,
        );

        let mut right = car();
        let mut right_boost = MAX_BOOST;
        let mut right_jump_held = false;
        let mut right_double_jump_available = true;
        let backward_input = ControllerInput {
            jump: true,
            pitch: Some(1.0),
            ..Default::default()
        };
        step_with_input_and_double_jump_state(
            &mut right,
            &backward_input,
            false,
            &mut right_boost,
            &mut right_jump_held,
            &mut right_double_jump_available,
            1.0 / 60.0,
        );

        assert!(left.linear_velocity.x * right.linear_velocity.x < 0.0);
        assert!(left.angular_velocity.y * right.angular_velocity.y < 0.0);
    }

    #[test]
    fn a_diagonal_dodge_combines_pitch_and_roll() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        // Forward (`pitch = -1`) plus right (`roll = +1`), in real Rocket
        // League's own stick convention (RB-PHYSICS-001-FR-079).
        let input = ControllerInput {
            jump: true,
            pitch: Some(-1.0),
            roll: Some(1.0),
            ..Default::default()
        };
        step_with_input_and_double_jump_state(
            &mut c,
            &input,
            false,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        // RB-PHYSICS-001-FR-072: a diagonal dodge's combined (pitch, roll)
        // direction is normalized to unit length before scaling by
        // DODGE_SPEED, matching RocketSim's own confirmed real
        // `dodgeDir.safeNormalized()` step — so each axis gets
        // DODGE_SPEED / sqrt(2), not a full DODGE_SPEED each (which would
        // make a diagonal dodge sqrt(2) times faster than an
        // axis-aligned one).
        let expected = DODGE_SPEED / std::f32::consts::SQRT_2;
        assert!(
            (c.linear_velocity.x - expected).abs() < 1.0,
            "expected the forward component of a diagonal dodge, got {}",
            c.linear_velocity.x
        );
        assert!(
            (c.linear_velocity.y - expected).abs() < 1.0,
            "expected the lateral component of a diagonal dodge, got {}",
            c.linear_velocity.y
        );
        let total_magnitude = (c.linear_velocity.x.powi(2) + c.linear_velocity.y.powi(2)).sqrt();
        assert!(
            (total_magnitude - DODGE_SPEED).abs() < 1.0,
            "expected a diagonal dodge's total magnitude to match an \
             axis-aligned one's DODGE_SPEED, got {total_magnitude}"
        );
    }

    #[test]
    fn a_yaw_only_press_fires_a_sideways_dodge_like_roll() {
        // RB-PHYSICS-001-FR-073: real Rocket League's dodgeDir.y combines
        // yaw + roll, so a pure yaw stick nudge (no roll held) fires the
        // same sideways dodge a roll-only press would.
        let mut c = car();
        let input = ControllerInput {
            jump: true,
            yaw: Some(1.0),
            ..Default::default()
        };
        let mut flip = airborne_dodge(&mut c, &input, 1.0 / 60.0);
        assert!(
            (c.linear_velocity.y - DODGE_SPEED).abs() < 1.0,
            "expected roughly DODGE_SPEED lateral velocity from yaw alone, got {}",
            c.linear_velocity.y
        );
        // Right dodge spins about -forward (RB-PHYSICS-001-FR-079), same
        // as `dodge_gives_lateral_velocity_and_spin_when_rolled_in_the_air`
        // — from the next step (RB-PHYSICS-001-FR-080).
        airborne_flip_step(&mut c, &ControllerInput::default(), &mut flip, 1.0 / 60.0);
        assert!(
            (c.angular_velocity.x + FLIP_ROLL_STEP_PER_TICK).abs() < 1e-3,
            "expected -FLIP_TORQUE_X / 120 spin about the forward axis after one tick, got {}",
            c.angular_velocity.x
        );
    }

    #[test]
    fn yaw_and_roll_combine_in_the_dodge_direction() {
        // RB-PHYSICS-001-FR-073: yaw and roll both feed the same combined
        // roll-axis stick value (roll + yaw) before normalization, so equal
        // opposite yaw and roll cancel out to no sideways dodge at all.
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let input = ControllerInput {
            jump: true,
            roll: Some(1.0),
            yaw: Some(-1.0),
            ..Default::default()
        };
        step_with_input_and_double_jump_state(
            &mut c,
            &input,
            false,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        assert_eq!(
            c.linear_velocity.y, 0.0,
            "expected equal-and-opposite roll and yaw to cancel out, got {}",
            c.linear_velocity.y
        );
        assert!(
            (c.linear_velocity.z - JUMP_SPEED).abs() < 1.0,
            "expected a plain double jump instead, since combined roll+yaw \
             and pitch are both below DODGE_DEADZONE, got {}",
            c.linear_velocity.z
        );
    }

    #[test]
    fn normalize_dodge_direction_preserves_a_single_axis() {
        // A pure axis-aligned dodge (the other axis exactly zero) is
        // unaffected: normalizing (1.0, 0.0) or (0.0, 1.0) yields the same
        // unit value back.
        assert_eq!(normalize_dodge_direction(1.0, 0.0), (1.0, 0.0));
        assert_eq!(normalize_dodge_direction(0.0, 1.0), (0.0, 1.0));
        assert_eq!(normalize_dodge_direction(-1.0, 0.0), (-1.0, 0.0));
    }

    #[test]
    fn normalize_dodge_direction_normalizes_a_diagonal_to_unit_length() {
        let (pitch, roll) = normalize_dodge_direction(1.0, 1.0);
        let magnitude = (pitch * pitch + roll * roll).sqrt();
        assert!(
            (magnitude - 1.0).abs() < 1e-6,
            "expected a diagonal direction to normalize to unit length, got {magnitude}"
        );
        assert!(
            (pitch - roll).abs() < 1e-6,
            "expected an equal 45-degree split"
        );
    }

    #[test]
    fn normalize_dodge_direction_snaps_a_near_axis_aligned_input_to_a_pure_axis() {
        // RB-PHYSICS-001-FR-074: RocketSim's own post-normalization
        // small-component zeroing snaps a near-axis-aligned diagonal (e.g.
        // a stick nudged almost, but not quite, purely forward) to a pure
        // single-axis dodge instead of leaving a tiny perpendicular
        // component. pitch=1.0, roll=0.05 normalizes to roll ~= 0.05,
        // well below DODGE_DIRECTION_SNAP_THRESHOLD (0.1).
        let (pitch, roll) = normalize_dodge_direction(1.0, 0.05);
        assert_eq!(
            roll, 0.0,
            "expected the tiny roll component to snap to zero"
        );
        assert!(
            pitch > 0.9,
            "expected the pitch component to stay close to its full unit magnitude, got {pitch}"
        );
    }

    #[test]
    fn normalize_dodge_direction_does_not_snap_a_clearly_diagonal_input() {
        // A genuinely diagonal input (both axes well above the snap
        // threshold once normalized) is unaffected by the snap.
        let (pitch, roll) = normalize_dodge_direction(1.0, 0.5);
        assert!(
            pitch.abs() >= DODGE_DIRECTION_SNAP_THRESHOLD
                && roll.abs() >= DODGE_DIRECTION_SNAP_THRESHOLD,
            "expected neither component to snap to zero, got ({pitch}, {roll})"
        );
        assert!(roll > 0.0, "expected the roll component to stay nonzero");
    }

    #[test]
    fn normalize_dodge_direction_is_zero_for_zero_input() {
        assert_eq!(normalize_dodge_direction(0.0, 0.0), (0.0, 0.0));
    }

    #[test]
    fn dodge_speed_scale_matches_the_real_curve() {
        // RB-PHYSICS-001-FR-059: RocketSim's own
        // FLIP_BACKWARD_IMPULSE_MAX_SPEED_SCALE (2.5) and
        // FLIP_SIDE_IMPULSE_MAX_SPEED_SCALE (1.9) confirmed exact against
        // its own RLConst.h.
        assert_eq!(dodge_speed_scale(0.0, 2.5), 1.0);
        assert_eq!(dodge_speed_scale(MAX_CAR_SPEED, 2.5), 2.5);
        assert!(
            (dodge_speed_scale(MAX_CAR_SPEED / 2.0, 2.5) - 1.75).abs() < 1e-4,
            "expected the midpoint to interpolate to 1.75, got {}",
            dodge_speed_scale(MAX_CAR_SPEED / 2.0, 2.5)
        );
        assert_eq!(
            dodge_speed_scale(MAX_CAR_SPEED * 2.0, 2.5),
            2.5,
            "expected speed past MAX_CAR_SPEED to clamp"
        );
        assert_eq!(
            dodge_speed_scale(-MAX_CAR_SPEED, 1.9),
            1.9,
            "expected the scale to use the speed's magnitude, not its sign"
        );
    }

    #[test]
    fn dodge_is_backward_matches_the_real_classification() {
        // The argument is the dodge's *forward* component (RocketSim's own
        // `dodgeDir.x = -pitch`), not the raw stick pitch — see
        // RB-PHYSICS-001-FR-079.
        // Below DODGE_BACKWARD_CLASSIFICATION_SPEED_THRESHOLD, classification
        // falls back to stick direction alone.
        assert!(dodge_is_backward(-1.0, 0.0));
        assert!(!dodge_is_backward(1.0, 0.0));
        // At speed, classification compares dodge direction to current
        // motion: opposing directions count as "backward" (a real backward
        // dodge, or a forward dodge while already moving backward).
        assert!(dodge_is_backward(-1.0, MAX_CAR_SPEED));
        assert!(dodge_is_backward(1.0, -MAX_CAR_SPEED));
        assert!(!dodge_is_backward(1.0, MAX_CAR_SPEED));
        assert!(!dodge_is_backward(-1.0, -MAX_CAR_SPEED));
    }

    #[test]
    fn a_backward_dodge_scales_up_with_current_forward_speed() {
        // RB-PHYSICS-001-FR-059: a dodge opposing the car's own current
        // motion now scales up toward DODGE_BACKWARD_SPEED_SCALE as speed
        // rises, matching RocketSim's own confirmed
        // FLIP_BACKWARD_IMPULSE_MAX_SPEED_SCALE — this test would see a
        // plain DODGE_SPEED-sized delta instead, without the scale.
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        c.linear_velocity = Vec3::new(MAX_CAR_SPEED, 0.0, 0.0);
        let before = c.linear_velocity.x;
        // `pitch = +1` (stick back) is the backward flip in real Rocket
        // League's own stick convention (RB-PHYSICS-001-FR-079).
        let input = ControllerInput {
            jump: true,
            pitch: Some(1.0),
            ..Default::default()
        };
        step_with_input_and_double_jump_state(
            &mut c,
            &input,
            false,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        let delta = c.linear_velocity.x - before;
        // RB-PHYSICS-001-FR-080: the real backward dodge also carries
        // `FLIP_BACKWARD_IMPULSE_SCALE_X = 16/15` on its forward-axis
        // component, multiplied on top of the speed ramp.
        assert!(
            (delta - (-DODGE_SPEED * DODGE_BACKWARD_SPEED_SCALE * DODGE_BACKWARD_SCALE_X)).abs()
                < 1.0,
            "expected a backward dodge at max speed to scale to DODGE_SPEED \
             * DODGE_BACKWARD_SPEED_SCALE * DODGE_BACKWARD_SCALE_X, got delta {}",
            delta
        );
    }

    #[test]
    fn a_backward_dodge_at_a_standstill_still_carries_the_real_16_15_forward_factor() {
        // RB-PHYSICS-001-FR-080: RocketSim applies
        // `FLIP_BACKWARD_IMPULSE_SCALE_X` whenever the dodge is classified
        // backward — at a standstill that classification falls back to
        // stick direction alone (`dodge_is_backward`'s low-speed branch),
        // so even with no speed ramp (`dodge_speed_scale` == 1.0 at zero
        // speed) the backward dodge is 16/15 of a forward one.
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let input = ControllerInput {
            jump: true,
            pitch: Some(1.0),
            ..Default::default()
        };
        step_with_input_and_double_jump_state(
            &mut c,
            &input,
            false,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        assert!(
            (c.linear_velocity.x - (-DODGE_SPEED * DODGE_BACKWARD_SCALE_X)).abs() < 1.0,
            "expected a standstill backward dodge of -DODGE_SPEED * 16/15, got {}",
            c.linear_velocity.x
        );
    }

    #[test]
    fn a_forward_dodge_does_not_scale_with_current_forward_speed() {
        // The real forward-dodge scale is exactly 1.0 (RocketSim's own
        // FLIP_FORWARD_IMPULSE_MAX_SPEED_SCALE) — a forward dodge stays at
        // DODGE_SPEED regardless of current speed, unlike a backward or
        // side dodge.
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        c.linear_velocity = Vec3::new(MAX_CAR_SPEED, 0.0, 0.0);
        let before = c.linear_velocity.x;
        let input = ControllerInput {
            jump: true,
            pitch: Some(-1.0),
            ..Default::default()
        };
        step_with_input_and_double_jump_state(
            &mut c,
            &input,
            false,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        let delta = c.linear_velocity.x - before;
        assert!(
            (delta - DODGE_SPEED).abs() < 1.0,
            "expected a forward dodge to stay at plain DODGE_SPEED \
             regardless of current speed, got delta {}",
            delta
        );
    }

    #[test]
    fn a_side_dodge_scales_up_with_current_forward_speed() {
        // RB-PHYSICS-001-FR-059: a side (roll) dodge scales up toward
        // DODGE_SIDE_SPEED_SCALE as current forward speed rises, regardless
        // of direction, matching RocketSim's own confirmed
        // FLIP_SIDE_IMPULSE_MAX_SPEED_SCALE.
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        c.linear_velocity = Vec3::new(MAX_CAR_SPEED, 0.0, 0.0);
        let input = ControllerInput {
            jump: true,
            roll: Some(1.0),
            ..Default::default()
        };
        step_with_input_and_double_jump_state(
            &mut c,
            &input,
            false,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        assert!(
            (c.linear_velocity.y - DODGE_SPEED * DODGE_SIDE_SPEED_SCALE).abs() < 1.0,
            "expected a side dodge at max speed to scale to DODGE_SPEED * \
             DODGE_SIDE_SPEED_SCALE, got {}",
            c.linear_velocity.y
        );
    }

    #[test]
    fn dodge_has_no_effect_while_grounded() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let input = ControllerInput {
            jump: true,
            pitch: Some(1.0),
            ..Default::default()
        };
        step_with_input_and_double_jump_state(
            &mut c,
            &input,
            true,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        assert_eq!(
            c.linear_velocity.x, 0.0,
            "expected no dodge push-off from a grounded jump, regardless of stick input"
        );
        assert!(
            (c.linear_velocity.z - JUMP_SPEED).abs() < 1.0,
            "expected the ordinary ground jump instead, got {}",
            c.linear_velocity.z
        );
    }

    #[test]
    fn a_wall_jump_dodges_outward_and_upward_with_a_flip_when_touching_a_wall_with_stick_input() {
        // Regression guard for the *reversed* premise: a wall jump used to
        // always ignore stick input; now directional stick input at or
        // above DODGE_DEADZONE fires a wall-jump dodge instead, combining
        // the wall's own push-off with a DODGE_SPEED horizontal component
        // and a visible flip (from the next step, RB-PHYSICS-001-FR-080).
        let dt = 1.0 / 60.0;
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let mut hold_remaining = 0.0;
        let mut flip = None;
        let input = ControllerInput {
            jump: true,
            pitch: Some(-1.0),
            ..Default::default()
        };
        step_with_input_and_dodge_flip(
            &mut c,
            &input,
            false,
            Some(Vec3::new(1.0, 0.0, 0.0)),
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut flip,
            dt,
        );
        assert!(
            (c.linear_velocity.x - (WALL_JUMP_HORIZONTAL_SPEED + DODGE_SPEED)).abs() < 1.0,
            "expected the wall push-off plus the forward dodge component, got {}",
            c.linear_velocity.x
        );
        assert!(
            (c.linear_velocity.z - JUMP_SPEED).abs() < 1.0,
            "expected the wall jump's upward component, got {}",
            c.linear_velocity.z
        );
        assert!(
            flip.is_some(),
            "expected the wall-jump dodge to start a flip"
        );
        airborne_flip_step(&mut c, &ControllerInput::default(), &mut flip, dt);
        assert!(
            c.angular_velocity.length() > 0.0,
            "expected the wall-jump dodge to give the car a visible flip, got {:?}",
            c.angular_velocity
        );
    }

    #[test]
    fn a_wall_jump_dodge_consumes_the_double_jump_unlike_a_plain_wall_jump() {
        let dt = 1.0 / 60.0;

        let mut dodging = car();
        let mut dodging_boost = MAX_BOOST;
        let mut dodging_jump_held = false;
        let mut dodging_double_jump_available = true;
        let dodge_input = ControllerInput {
            jump: true,
            pitch: Some(1.0),
            ..Default::default()
        };
        step_with_input_and_wall(
            &mut dodging,
            &dodge_input,
            false,
            Some(Vec3::new(1.0, 0.0, 0.0)),
            &mut dodging_boost,
            &mut dodging_jump_held,
            &mut dodging_double_jump_available,
            dt,
        );
        assert!(
            !dodging_double_jump_available,
            "expected a wall-jump dodge to consume the double jump, unlike a plain wall jump"
        );

        let mut plain = car();
        let mut plain_boost = MAX_BOOST;
        let mut plain_jump_held = false;
        let mut plain_double_jump_available = true;
        step_with_input_and_wall(
            &mut plain,
            &full_jump(),
            false,
            Some(Vec3::new(1.0, 0.0, 0.0)),
            &mut plain_boost,
            &mut plain_jump_held,
            &mut plain_double_jump_available,
            dt,
        );
        assert!(
            plain_double_jump_available,
            "expected a plain wall jump (no stick input) to leave the double jump available"
        );
    }

    #[test]
    fn a_wall_jump_dodges_flip_can_be_cancelled_by_holding_pitch_against_it() {
        // A wall-jump dodge starts the same DodgeFlip a ground dodge does,
        // so the real pitch-hold cancel (RB-PHYSICS-001-FR-080 step (c))
        // applies to it identically.
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let mut hold_remaining = 0.0;
        let mut dodge_flip = None;

        // Forward wall-jump dodge (`pitch = -1`) off a +x wall.
        step_with_input_and_dodge_flip(
            &mut c,
            &ControllerInput {
                jump: true,
                pitch: Some(-1.0),
                ..Default::default()
            },
            false,
            Some(Vec3::new(1.0, 0.0, 0.0)),
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut dodge_flip,
            dt,
        );
        assert_eq!(
            dodge_flip,
            Some(DodgeFlip {
                rel_torque: (0.0, 1.0),
                elapsed: dt
            })
        );

        // Off the wall, pull back (`pitch = +1`, the flip's own pitch sign):
        // the flip's pitch torque is scaled to zero, so nothing changes the
        // spin this step.
        let spin_before = c.angular_velocity;
        let damp = neutral_damping_step(&c, dt);
        airborne_flip_step(&mut c, &full_pitch(), &mut dodge_flip, dt);
        assert!(
            (c.total_angular_acceleration() * dt - damp).length() < 1e-5,
            "expected a full pitch hold against the flip to zero its torque (damping only), got {:?}",
            c.total_angular_acceleration()
        );
        assert!((c.angular_velocity - spin_before - damp).length() < 1e-5);
        assert!(
            dodge_flip.is_some(),
            "expected the flip state to persist through a cancel"
        );
    }

    #[test]
    fn below_deadzone_stick_input_at_a_wall_still_gives_a_plain_wall_jump() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let input = ControllerInput {
            jump: true,
            pitch: Some(0.05), // below DODGE_DEADZONE (0.1)
            ..Default::default()
        };
        step_with_input_and_wall(
            &mut c,
            &input,
            false,
            Some(Vec3::new(1.0, 0.0, 0.0)),
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        assert!(
            (c.linear_velocity.x - WALL_JUMP_HORIZONTAL_SPEED).abs() < 1.0,
            "expected a plain wall jump from a below-deadzone stick deflection, got {}",
            c.linear_velocity.x
        );
        // A small additional contribution from air control's own
        // continuous pitch torque (applied unconditionally while airborne,
        // same as ever) is expected and tolerated here — only a flip must
        // be absent (and since RB-PHYSICS-001-FR-080 a flip's torque would
        // only show from the next step anyway; the state itself is what
        // `a_wall_jump_dodge_consumes_the_double_jump_unlike_a_plain_wall_jump`
        // and the resource check below pin down).
        assert!(
            c.angular_velocity.length() < 1.0,
            "expected no dodge-scale flip from a below-deadzone stick deflection, got {:?}",
            c.angular_velocity
        );
        assert!(
            double_jump_available,
            "expected a plain wall jump to leave the double jump available"
        );
    }

    #[test]
    fn opposite_pitch_wall_jump_dodges_the_opposite_direction() {
        let wall_normal = Vec3::new(1.0, 0.0, 0.0);

        let mut left = car();
        let mut left_boost = MAX_BOOST;
        let mut left_jump_held = false;
        let mut left_double_jump_available = true;
        let forward_input = ControllerInput {
            jump: true,
            pitch: Some(-1.0),
            ..Default::default()
        };
        step_with_input_and_wall(
            &mut left,
            &forward_input,
            false,
            Some(wall_normal),
            &mut left_boost,
            &mut left_jump_held,
            &mut left_double_jump_available,
            1.0 / 60.0,
        );

        let mut right = car();
        let mut right_boost = MAX_BOOST;
        let mut right_jump_held = false;
        let mut right_double_jump_available = true;
        let backward_input = ControllerInput {
            jump: true,
            pitch: Some(1.0),
            ..Default::default()
        };
        step_with_input_and_wall(
            &mut right,
            &backward_input,
            false,
            Some(wall_normal),
            &mut right_boost,
            &mut right_jump_held,
            &mut right_double_jump_available,
            1.0 / 60.0,
        );

        // Both still get the same fixed wall push-off along wall_normal;
        // only the dodge's own forward-axis contribution should differ in
        // sign, so compare velocity relative to the fixed WALL_JUMP_HORIZONTAL_SPEED
        // baseline rather than raw velocity.
        assert!(
            (left.linear_velocity.x - WALL_JUMP_HORIZONTAL_SPEED)
                > (right.linear_velocity.x - WALL_JUMP_HORIZONTAL_SPEED)
        );
        assert!(left.angular_velocity.y * right.angular_velocity.y < 0.0);
    }

    #[test]
    fn a_diagonal_wall_jump_dodge_combines_pitch_and_roll() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let input = ControllerInput {
            jump: true,
            pitch: Some(-1.0),
            roll: Some(1.0),
            ..Default::default()
        };
        step_with_input_and_wall(
            &mut c,
            &input,
            false,
            Some(Vec3::new(1.0, 0.0, 0.0)),
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        // RB-PHYSICS-001-FR-072: the dodge's own (pitch, roll) direction is
        // normalized before scaling — see
        // `a_diagonal_dodge_combines_pitch_and_roll`'s own comment — so
        // each dodge component is DODGE_SPEED / sqrt(2), on top of the
        // unaffected wall push-off.
        let expected_dodge_component = DODGE_SPEED / std::f32::consts::SQRT_2;
        assert!(
            (c.linear_velocity.x - (WALL_JUMP_HORIZONTAL_SPEED + expected_dodge_component)).abs()
                < 1.0,
            "expected the wall push-off plus the forward dodge component, got {}",
            c.linear_velocity.x
        );
        assert!(
            (c.linear_velocity.y - expected_dodge_component).abs() < 1.0,
            "expected the lateral dodge component, got {}",
            c.linear_velocity.y
        );
    }

    #[test]
    fn a_yaw_only_press_fires_a_sideways_wall_jump_dodge_like_roll() {
        // RB-PHYSICS-001-FR-073: the wall-jump-dodge path folds yaw into
        // the same combined roll-axis stick value as the ground dodge.
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let input = ControllerInput {
            jump: true,
            yaw: Some(1.0),
            ..Default::default()
        };
        step_with_input_and_wall(
            &mut c,
            &input,
            false,
            Some(Vec3::new(1.0, 0.0, 0.0)),
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            1.0 / 60.0,
        );
        assert!(
            (c.linear_velocity.y - DODGE_SPEED).abs() < 1.0,
            "expected roughly DODGE_SPEED lateral dodge velocity from yaw alone, got {}",
            c.linear_velocity.y
        );
    }

    #[test]
    fn air_control_has_no_effect_while_grounded() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        let input = ControllerInput {
            pitch: Some(1.0),
            yaw: Some(1.0),
            roll: Some(1.0),
            ..Default::default()
        };
        step_with_input(&mut c, &input, true, &mut boost, 1.0 / 60.0);
        assert_eq!(
            c.angular_velocity,
            Vec3::ZERO,
            "grounded air control shouldn't spin the car — steering already owns yaw on the ground"
        );
    }

    #[test]
    fn a_stationary_airborne_car_can_pitch_yaw_and_roll() {
        // Unlike ground steering, air control isn't speed-scaled — a car
        // with zero velocity should still spin freely.
        let mut c = car();
        let mut boost = MAX_BOOST;
        step_with_input(&mut c, &full_pitch(), false, &mut boost, 1.0 / 60.0);
        assert!(
            c.angular_velocity.y.abs() > 0.0,
            "expected pitch to produce angular velocity about the local right (Y) axis, got {:?}",
            c.angular_velocity
        );

        let mut c = car();
        step_with_input(&mut c, &full_yaw(), false, &mut boost, 1.0 / 60.0);
        assert!(
            c.angular_velocity.z.abs() > 0.0,
            "expected yaw to produce angular velocity about the local up (Z) axis, got {:?}",
            c.angular_velocity
        );

        let mut c = car();
        step_with_input(&mut c, &full_roll(), false, &mut boost, 1.0 / 60.0);
        assert!(
            c.angular_velocity.x.abs() > 0.0,
            "expected roll to produce angular velocity about the local forward (X) axis, got {:?}",
            c.angular_velocity
        );
    }

    #[test]
    fn air_control_angular_acceleration_matches_the_real_torque_values_directly_with_no_inertia_division(
    ) {
        // RB-PHYSICS-001-FR-079: real Rocket League applies air control as a
        // direct angular-acceleration rate (RocketSim's own `_UpdateAirTorque`
        // deliberately cancels Bullet's inverse-inertia integration step), so
        // this port's own `apply_angular_acceleration` path must reproduce
        // `raw_torque * CAR_TORQUE_SCALE` exactly, regardless of this car's
        // own (anisotropic) moment of inertia — unlike the old `apply_torque`
        // mechanism this replaced, `inv_inertia_world` plays no role at all.
        //
        // Signs: pitch and roll act about the *negative* of their axis
        // (RocketSim's `dirPitch_right = -GetRightDir()`, `dirRoll_forward =
        // -GetForwardDir()`), so full positive pitch (stick back, nose up)
        // gives negative angular velocity about +right, and full positive
        // roll (roll right) negative angular velocity about +forward; only
        // yaw's `dirYaw_up = GetUpDir()` is unnegated.
        let dt = 1.0 / 60.0;

        let mut pitch_car = car();
        let mut pitch_boost = MAX_BOOST;
        step_with_input(&mut pitch_car, &full_pitch(), false, &mut pitch_boost, dt);
        let expected_pitch = -AIR_CONTROL_PITCH_TORQUE * CAR_TORQUE_SCALE * dt;
        assert!(
            (pitch_car.angular_velocity.y - expected_pitch).abs() < 1e-3,
            "expected pitch's own angular velocity to match AIR_CONTROL_PITCH_TORQUE * \
             CAR_TORQUE_SCALE directly, got {} (expected {})",
            pitch_car.angular_velocity.y,
            expected_pitch
        );

        let mut yaw_car = car();
        let mut yaw_boost = MAX_BOOST;
        step_with_input(&mut yaw_car, &full_yaw(), false, &mut yaw_boost, dt);
        let expected_yaw = AIR_CONTROL_YAW_TORQUE * CAR_TORQUE_SCALE * dt;
        assert!(
            (yaw_car.angular_velocity.z - expected_yaw).abs() < 1e-3,
            "expected yaw's own angular velocity to match AIR_CONTROL_YAW_TORQUE * \
             CAR_TORQUE_SCALE directly, got {} (expected {})",
            yaw_car.angular_velocity.z,
            expected_yaw
        );

        let mut roll_car = car();
        let mut roll_boost = MAX_BOOST;
        step_with_input(&mut roll_car, &full_roll(), false, &mut roll_boost, dt);
        let expected_roll = -AIR_CONTROL_ROLL_TORQUE * CAR_TORQUE_SCALE * dt;
        assert!(
            (roll_car.angular_velocity.x - expected_roll).abs() < 1e-3,
            "expected roll's own angular velocity to match AIR_CONTROL_ROLL_TORQUE * \
             CAR_TORQUE_SCALE directly, got {} (expected {})",
            roll_car.angular_velocity.x,
            expected_roll
        );
    }

    #[test]
    fn no_analog_value_is_treated_as_neutral_air_control() {
        let mut c = car();
        let mut boost = MAX_BOOST;
        step_with_input(
            &mut c,
            &ControllerInput::default(),
            false,
            &mut boost,
            1.0 / 60.0,
        );
        assert_eq!(
            c.angular_velocity,
            Vec3::ZERO,
            "pitch/yaw/roll all None (e.g. replay-derived input) should behave like neutral input"
        );
    }

    #[test]
    fn opposite_yaw_spins_the_opposite_way_in_the_air() {
        let mut left = car();
        let mut left_boost = MAX_BOOST;
        step_with_input(&mut left, &full_yaw(), false, &mut left_boost, 1.0 / 60.0);

        let mut right = car();
        let mut right_boost = MAX_BOOST;
        let opposite = ControllerInput {
            yaw: Some(-1.0),
            ..Default::default()
        };
        step_with_input(&mut right, &opposite, false, &mut right_boost, 1.0 / 60.0);

        assert!(left.angular_velocity.z * right.angular_velocity.z < 0.0);
    }

    #[test]
    fn holding_jump_after_a_ground_jump_adds_more_upward_velocity_than_a_tap() {
        let dt = 1.0 / 120.0;

        let mut tapped = car();
        let mut tapped_boost = MAX_BOOST;
        let mut tapped_jump_held = false;
        let mut tapped_double_jump_available = true;
        let mut tapped_hold_remaining = 0.0;
        step_with_input_and_hold(
            &mut tapped,
            &full_jump(),
            true,
            None,
            &mut tapped_boost,
            &mut tapped_jump_held,
            &mut tapped_double_jump_available,
            &mut tapped_hold_remaining,
            dt,
        );
        for _ in 0..12 {
            step_with_input_and_hold(
                &mut tapped,
                &ControllerInput::default(),
                true,
                None,
                &mut tapped_boost,
                &mut tapped_jump_held,
                &mut tapped_double_jump_available,
                &mut tapped_hold_remaining,
                dt,
            );
        }

        let mut held = car();
        let mut held_boost = MAX_BOOST;
        let mut held_jump_held = false;
        let mut held_double_jump_available = true;
        let mut held_hold_remaining = 0.0;
        for _ in 0..13 {
            step_with_input_and_hold(
                &mut held,
                &full_jump(),
                true,
                None,
                &mut held_boost,
                &mut held_jump_held,
                &mut held_double_jump_available,
                &mut held_hold_remaining,
                dt,
            );
        }

        assert!(
            held.linear_velocity.z > tapped.linear_velocity.z + 1.0,
            "expected holding jump to accrue more upward velocity than tapping it, \
             tapped={}, held={}",
            tapped.linear_velocity.z,
            held.linear_velocity.z
        );
    }

    #[test]
    fn releasing_jump_early_stops_the_extra_acceleration_from_a_held_ground_jump() {
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let mut hold_remaining = 0.0;

        // Press, hold for a few steps (well short of JUMP_HOLD_MAX_DURATION),
        // then release.
        for _ in 0..5 {
            step_with_input_and_hold(
                &mut c,
                &full_jump(),
                true,
                None,
                &mut boost,
                &mut jump_held,
                &mut double_jump_available,
                &mut hold_remaining,
                dt,
            );
        }
        assert!(
            hold_remaining > 0.0,
            "expected the hold window to still have time left at this point"
        );
        step_with_input_and_hold(
            &mut c,
            &ControllerInput::default(),
            true,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            dt,
        );
        assert_eq!(
            hold_remaining, 0.0,
            "expected releasing jump to immediately end the hold window"
        );
        let velocity_at_release = c.linear_velocity.z;

        // Continue with jump released for several more steps — no further
        // gain expected even though the hold window had time remaining.
        for _ in 0..10 {
            step_with_input_and_hold(
                &mut c,
                &ControllerInput::default(),
                true,
                None,
                &mut boost,
                &mut jump_held,
                &mut double_jump_available,
                &mut hold_remaining,
                dt,
            );
        }
        assert!(
            (c.linear_velocity.z - velocity_at_release).abs() < 1e-3,
            "expected no further upward velocity gain after releasing jump early, \
             velocity at release={velocity_at_release}, after={}",
            c.linear_velocity.z
        );
    }

    #[test]
    fn jump_hold_acceleration_is_scaled_down_during_the_mandatory_pre_min_time_window() {
        // RB-PHYSICS-001-FR-064: real Rocket League's own `_UpdateJump`
        // scales the hold acceleration by `JUMP_PRE_MIN_ACCEL_SCALE` for the
        // first `JUMP_MIN_TIME` seconds after a ground-jump press, applied
        // regardless of whether `jump` is still held.
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let mut hold_remaining = 0.0;

        // Press: arms the window; only the fixed JUMP_SPEED impulse fires
        // this step.
        step_with_input_and_hold(
            &mut c,
            &full_jump(),
            true,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            dt,
        );
        let velocity_after_press = c.linear_velocity.z;

        // The very next step is still well within JUMP_MIN_TIME (0.025s = 3
        // steps at this dt).
        step_with_input_and_hold(
            &mut c,
            &full_jump(),
            true,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            dt,
        );

        let expected_gain = JUMP_HOLD_ACCELERATION * JUMP_PRE_MIN_ACCEL_SCALE * dt;
        assert!(
            (c.linear_velocity.z - (velocity_after_press + expected_gain)).abs() < 1e-2,
            "expected the mandatory pre-min-time window's own scaled acceleration, \
             got a gain of {}, expected {}",
            c.linear_velocity.z - velocity_after_press,
            expected_gain
        );
    }

    #[test]
    fn releasing_jump_within_the_mandatory_pre_min_time_window_does_not_immediately_stop_the_extra_acceleration(
    ) {
        // Unlike releasing jump after JUMP_MIN_TIME has already elapsed (see
        // releasing_jump_early_stops_the_extra_acceleration_from_a_held_ground_jump,
        // which releases well past it), releasing within the mandatory
        // window doesn't end it early — real Rocket League's own engine
        // keeps applying the scaled acceleration regardless of `jump`.
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let mut hold_remaining = 0.0;

        step_with_input_and_hold(
            &mut c,
            &full_jump(),
            true,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            dt,
        );
        let velocity_after_press = c.linear_velocity.z;

        // Release immediately (a tap) — still within JUMP_MIN_TIME.
        step_with_input_and_hold(
            &mut c,
            &ControllerInput::default(),
            true,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            dt,
        );

        let expected_gain = JUMP_HOLD_ACCELERATION * JUMP_PRE_MIN_ACCEL_SCALE * dt;
        assert!(
            (c.linear_velocity.z - (velocity_after_press + expected_gain)).abs() < 1e-2,
            "expected a tap to still gain the mandatory window's own scaled acceleration \
             despite releasing jump immediately, got a gain of {}, expected {}",
            c.linear_velocity.z - velocity_after_press,
            expected_gain
        );
        assert!(
            hold_remaining > 0.0,
            "expected the mandatory window to still have time left, not yet closed"
        );
    }

    #[test]
    fn mandatory_pre_min_time_window_closes_on_schedule_even_when_jump_is_never_held() {
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let mut hold_remaining = 0.0;

        step_with_input_and_hold(
            &mut c,
            &full_jump(),
            true,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            dt,
        );

        // Release immediately and stay released well past JUMP_MIN_TIME
        // (0.025s = 3 steps at this dt) — the mandatory window must still
        // close on its own schedule even though jump was never held past
        // the press.
        for _ in 0..6 {
            step_with_input_and_hold(
                &mut c,
                &ControllerInput::default(),
                true,
                None,
                &mut boost,
                &mut jump_held,
                &mut double_jump_available,
                &mut hold_remaining,
                dt,
            );
        }
        assert_eq!(
            hold_remaining, 0.0,
            "expected the mandatory window to have closed by now"
        );
        let velocity_after_window_closes = c.linear_velocity.z;

        for _ in 0..5 {
            step_with_input_and_hold(
                &mut c,
                &ControllerInput::default(),
                true,
                None,
                &mut boost,
                &mut jump_held,
                &mut double_jump_available,
                &mut hold_remaining,
                dt,
            );
        }
        assert!(
            (c.linear_velocity.z - velocity_after_window_closes).abs() < 1e-3,
            "expected no further upward velocity gain once the mandatory window has closed, \
             velocity at window close={velocity_after_window_closes}, after={}",
            c.linear_velocity.z
        );
    }

    #[test]
    fn held_jump_stops_gaining_extra_velocity_once_the_hold_window_expires() {
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let mut hold_remaining = 0.0;

        // Press, then hold well past JUMP_HOLD_MAX_DURATION (0.2s = 24
        // steps at this dt).
        for _ in 0..30 {
            step_with_input_and_hold(
                &mut c,
                &full_jump(),
                true,
                None,
                &mut boost,
                &mut jump_held,
                &mut double_jump_available,
                &mut hold_remaining,
                dt,
            );
        }
        let velocity_at_30_steps = c.linear_velocity.z;
        assert_eq!(
            hold_remaining, 0.0,
            "expected the hold window to have fully expired by now"
        );

        // Continue holding for several more steps — no further gain
        // expected, since the window has already run out.
        for _ in 0..10 {
            step_with_input_and_hold(
                &mut c,
                &full_jump(),
                true,
                None,
                &mut boost,
                &mut jump_held,
                &mut double_jump_available,
                &mut hold_remaining,
                dt,
            );
        }
        assert!(
            (c.linear_velocity.z - velocity_at_30_steps).abs() < 1e-3,
            "expected no further upward velocity gain once the hold window has expired, \
             velocity at 30 steps={velocity_at_30_steps}, after 10 more={}",
            c.linear_velocity.z
        );
    }

    #[test]
    fn double_jump_after_a_held_ground_jump_is_not_boosted_by_the_hold_window() {
        // Regression guard: variable jump height is scoped to the ground
        // jump alone — holding jump through the ground jump's whole hold
        // window must not leak any extra acceleration into a later double
        // jump.
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let mut hold_remaining = 0.0;

        for _ in 0..24 {
            step_with_input_and_hold(
                &mut c,
                &full_jump(),
                true,
                None,
                &mut boost,
                &mut jump_held,
                &mut double_jump_available,
                &mut hold_remaining,
                dt,
            );
        }
        let velocity_after_held_ground_jump = c.linear_velocity.z;

        // Release, then press again while airborne — a plain double jump.
        step_with_input_and_hold(
            &mut c,
            &ControllerInput::default(),
            false,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            dt,
        );
        step_with_input_and_hold(
            &mut c,
            &full_jump(),
            false,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            dt,
        );

        assert!(
            (c.linear_velocity.z - (velocity_after_held_ground_jump + JUMP_SPEED)).abs() < 1.0,
            "expected the double jump to add exactly one more JUMP_SPEED kick, not an extra \
             variable-height boost left over from holding the ground jump, after held ground \
             jump={velocity_after_held_ground_jump}, after double jump={}",
            c.linear_velocity.z
        );
    }

    #[test]
    fn a_second_jump_press_mid_flip_does_nothing() {
        // RB-PHYSICS-001-FR-080 step (c): RocketSim's `hasFlipped` makes a
        // further press unusable — this port's former jump-press flip
        // cancel (RB-PHYSICS-001-FR-016) is gone, so the flip torque simply
        // carries on through the press.
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let mut hold_remaining = 0.0;
        let mut dodge_flip = None;

        step_with_input_and_dodge_flip(
            &mut c,
            &ControllerInput {
                jump: true,
                pitch: Some(-1.0),
                ..Default::default()
            },
            false,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut dodge_flip,
            dt,
        );
        assert!(dodge_flip.is_some());
        assert!(!double_jump_available);

        // Release (one tick of flip torque), then press again.
        step_with_input_and_dodge_flip(
            &mut c,
            &ControllerInput::default(),
            false,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut dodge_flip,
            dt,
        );
        let spin_before_press = c.angular_velocity;
        let velocity_before_press = c.linear_velocity;
        let damp = neutral_damping_step(&c, dt).y;
        step_with_input_and_dodge_flip(
            &mut c,
            &full_jump(),
            false,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut dodge_flip,
            dt,
        );

        assert!(
            (c.angular_velocity.y - spin_before_press.y - damp - FLIP_PITCH_STEP_PER_TICK).abs()
                < 1e-3,
            "expected the flip torque to carry on through the press, got {:?} from {:?}",
            c.angular_velocity,
            spin_before_press
        );
        assert_eq!(c.linear_velocity, velocity_before_press);
        assert!(
            dodge_flip.is_some(),
            "expected the press to leave the flip running"
        );
        assert!(!double_jump_available);
    }

    #[test]
    fn flip_cancel_does_not_touch_linear_velocity_the_flip_state_or_the_double_jump_resource() {
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let mut hold_remaining = 0.0;
        let mut dodge_flip = None;

        step_with_input_and_dodge_flip(
            &mut c,
            &ControllerInput {
                jump: true,
                pitch: Some(-1.0),
                ..Default::default()
            },
            false,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut dodge_flip,
            dt,
        );
        let linear_velocity_after_dodge = c.linear_velocity;
        let flip_after_dodge = dodge_flip;
        assert!(
            !double_jump_available,
            "expected the dodge to have already spent the double jump"
        );

        // Hold pitch against the flip for a few steps (still well before
        // FLIP_Z_DAMP_START, so no vertical bleed either).
        for _ in 0..3 {
            step_with_input_and_dodge_flip(
                &mut c,
                &full_pitch(),
                false,
                None,
                &mut boost,
                &mut jump_held,
                &mut double_jump_available,
                &mut hold_remaining,
                &mut dodge_flip,
                dt,
            );
        }

        assert_eq!(
            c.linear_velocity, linear_velocity_after_dodge,
            "expected flip-cancel to leave the dodge's own translation untouched"
        );
        assert_eq!(
            dodge_flip.map(|f| f.rel_torque),
            flip_after_dodge.map(|f| f.rel_torque),
            "expected flip-cancel to scale the torque per step, not rewrite the flip state"
        );
        assert!(dodge_flip.is_some_and(|f| (f.elapsed - 4.0 * dt).abs() < 1e-6));
        assert!(
            !double_jump_available,
            "expected flip-cancel to neither consume nor restore the double jump"
        );
    }

    #[test]
    fn a_plain_double_jump_clears_a_stale_dodge_flip_from_an_earlier_dodge() {
        // Regression guard: a dodge starts a flip, and if nothing ever
        // explicitly cleared it, its torque would keep running under a
        // later, completely unrelated plain double jump. Landing clears the
        // flip itself (see `landing_clears_the_flip_state_and_its_torque`),
        // so the route that still leaves it stale is a wall touch: it
        // restores the double jump without ending the flip.
        let dt = 1.0 / 60.0;
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let mut hold_remaining = 0.0;
        let mut dodge_flip = None;

        let dodge_input = ControllerInput {
            jump: true,
            pitch: Some(1.0),
            ..Default::default()
        };
        step_with_input_and_dodge_flip(
            &mut c,
            &dodge_input,
            false,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut dodge_flip,
            dt,
        );
        assert!(dodge_flip.is_some(), "expected the dodge to start a flip");

        // Brush a wall (restores double_jump_available, leaves the flip
        // alone), then move off it and fire a plain double jump (no stick
        // input) — this must clear the stale flip from the earlier dodge.
        step_with_input_and_dodge_flip(
            &mut c,
            &ControllerInput::default(),
            false,
            Some(Vec3::new(1.0, 0.0, 0.0)),
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut dodge_flip,
            dt,
        );
        assert!(double_jump_available);
        assert!(
            dodge_flip.is_some(),
            "expected a wall touch to leave the flip alone"
        );
        step_with_input_and_dodge_flip(
            &mut c,
            &full_jump(),
            false,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut dodge_flip,
            dt,
        );
        assert!(
            dodge_flip.is_none(),
            "expected a plain double jump to clear any stale dodge flip"
        );

        // And with it gone, no flip torque runs under the double jump.
        let spin_after_plain_double_jump = c.angular_velocity;
        let damp = neutral_damping_step(&c, dt);
        step_with_input_and_dodge_flip(
            &mut c,
            &full_jump(),
            false,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut dodge_flip,
            dt,
        );
        assert!((c.total_angular_acceleration() * dt - damp).length() < 1e-5);
        assert!((c.angular_velocity - spin_after_plain_double_jump - damp).length() < 1e-5);
    }

    #[test]
    fn a_fresh_press_at_a_wall_mid_flip_wall_jumps_and_leaves_the_flip_running() {
        let dt = 1.0 / 60.0;
        let mut c = car();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let mut hold_remaining = 0.0;
        let mut dodge_flip = None;

        // A forward dodge (`pitch = -1` in real Rocket League's own stick
        // convention, RB-PHYSICS-001-FR-079), so the later wall push-off
        // along +x adds to the dodge's own +x velocity rather than fighting
        // a backward dodge's.
        let dodge_input = ControllerInput {
            jump: true,
            pitch: Some(-1.0),
            ..Default::default()
        };
        step_with_input_and_dodge_flip(
            &mut c,
            &dodge_input,
            false,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut dodge_flip,
            dt,
        );
        assert!(dodge_flip.is_some());

        // Release (the flip torque's first step), then press again while
        // touching a wall — fires a plain wall jump; the flip carries on.
        step_with_input_and_dodge_flip(
            &mut c,
            &ControllerInput::default(),
            false,
            None,
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut dodge_flip,
            dt,
        );
        let spin_before_press = c.angular_velocity;
        let damp = neutral_damping_step(&c, dt).y;
        step_with_input_and_dodge_flip(
            &mut c,
            &full_jump(),
            false,
            Some(Vec3::new(1.0, 0.0, 0.0)),
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut dodge_flip,
            dt,
        );

        assert!(
            c.linear_velocity.x > 0.0,
            "expected the wall jump's outward push-off, got {:?}",
            c.linear_velocity
        );
        assert!(
            (c.angular_velocity.y - spin_before_press.y - damp - FLIP_PITCH_STEP_PER_TICK).abs()
                < 1e-3,
            "expected the wall jump to leave the flip's torque running, got {:?} from {:?}",
            c.angular_velocity,
            spin_before_press
        );
        assert!(
            dodge_flip.is_some(),
            "expected the wall jump to leave the flip untouched"
        );
    }

    #[test]
    fn holding_pitch_against_a_forward_flip_scales_its_pitch_torque_by_one_minus_the_deflection() {
        // RB-PHYSICS-001-FR-080 step (c): RocketSim's `pitchScale = 1 -
        // |controls.pitch|` on `flipRelTorque.y` when the signs match — a
        // full pull-back zeroes a forward flip's torque, a half one halves
        // it, and letting go restores it, step by step.
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut flip = airborne_dodge(
            &mut c,
            &ControllerInput {
                jump: true,
                pitch: Some(-1.0),
                ..Default::default()
            },
            dt,
        );
        assert_eq!(flip.map(|f| f.rel_torque), Some((0.0, 1.0)));

        let before = c.angular_velocity.y;
        let damp = neutral_damping_step(&c, dt);
        airborne_flip_step(&mut c, &full_pitch(), &mut flip, dt);
        assert!(
            (c.total_angular_acceleration() * dt - damp).length() < 1e-5,
            "expected a full pull-back to zero the pitch torque (and pitch air control stays \
             locked), leaving only damping, got {:?}",
            c.total_angular_acceleration()
        );
        assert!((c.angular_velocity.y - before - damp.y).abs() < 1e-5);

        let before = c.angular_velocity.y;
        let damp = neutral_damping_step(&c, dt).y;
        airborne_flip_step(
            &mut c,
            &ControllerInput {
                pitch: Some(0.5),
                ..Default::default()
            },
            &mut flip,
            dt,
        );
        assert!(
            (c.angular_velocity.y - before - damp - 0.5 * FLIP_PITCH_STEP_PER_TICK).abs() < 1e-3,
            "expected a half pull-back to halve the pitch torque, got Δ{}",
            c.angular_velocity.y - before - damp
        );

        let before = c.angular_velocity.y;
        let damp = neutral_damping_step(&c, dt).y;
        airborne_flip_step(&mut c, &ControllerInput::default(), &mut flip, dt);
        assert!(
            (c.angular_velocity.y - before - damp - FLIP_PITCH_STEP_PER_TICK).abs() < 1e-3,
            "expected releasing the stick to restore the full torque, got Δ{}",
            c.angular_velocity.y - before - damp
        );
        assert!(flip.is_some());
    }

    #[test]
    fn holding_pitch_with_the_flip_does_not_cancel_it_and_a_backward_flip_cancels_on_push_forward()
    {
        let dt = 1.0 / 120.0;

        // Forward flip, stick still forward: signs differ, full torque —
        // and no extra pitch air control, since pitch is locked mid-flip.
        let mut c = car();
        let mut flip = airborne_dodge(
            &mut c,
            &ControllerInput {
                jump: true,
                pitch: Some(-1.0),
                ..Default::default()
            },
            dt,
        );
        let before = c.angular_velocity.y;
        let damp = neutral_damping_step(&c, dt).y;
        airborne_flip_step(&mut c, &full_pitch_forward(), &mut flip, dt);
        assert!(
            (c.angular_velocity.y - before - damp - FLIP_PITCH_STEP_PER_TICK).abs() < 1e-3,
            "expected pitch held *with* a forward flip to leave its torque whole, got Δ{}",
            c.angular_velocity.y - before - damp
        );

        // Backward flip (`rel_torque.1 = -1`): pushing forward cancels it.
        let mut c = car();
        let mut flip = airborne_dodge(
            &mut c,
            &ControllerInput {
                jump: true,
                pitch: Some(1.0),
                ..Default::default()
            },
            dt,
        );
        assert_eq!(flip.map(|f| f.rel_torque), Some((0.0, -1.0)));
        let before = c.angular_velocity.y;
        let damp = neutral_damping_step(&c, dt);
        airborne_flip_step(&mut c, &full_pitch_forward(), &mut flip, dt);
        assert!((c.total_angular_acceleration() * dt - damp).length() < 1e-5);
        assert!((c.angular_velocity.y - before - damp.y).abs() < 1e-5);
    }

    #[test]
    fn a_roll_only_dodge_ignores_pitch_entirely() {
        // No pitch-axis component means nothing for the cancel to scale,
        // whichever way pitch is held — and pitch itself is locked out of
        // air control, so it contributes nothing at all.
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut flip = airborne_dodge(
            &mut c,
            &ControllerInput {
                jump: true,
                roll: Some(1.0),
                ..Default::default()
            },
            dt,
        );
        assert_eq!(flip.map(|f| f.rel_torque), Some((-1.0, 0.0)));

        for input in [full_pitch_forward(), full_pitch()] {
            let before = c.angular_velocity.x;
            let damp = neutral_damping_step(&c, dt).x;
            airborne_flip_step(&mut c, &input, &mut flip, dt);
            assert!(
                (c.angular_velocity.x - before - damp + FLIP_ROLL_STEP_PER_TICK).abs() < 1e-3,
                "expected the full roll torque regardless of pitch, got Δ{}",
                c.angular_velocity.x - before - damp
            );
            assert_eq!(c.angular_velocity.y, 0.0, "expected pitch to be locked out");
        }
    }

    #[test]
    fn pitch_stays_locked_out_of_air_control_mid_flip_while_yaw_works_cancel_or_not() {
        let dt = 1.0 / 120.0;
        let forward_dodge = ControllerInput {
            jump: true,
            pitch: Some(-1.0),
            ..Default::default()
        };
        let expected_yaw = AIR_CONTROL_YAW_TORQUE * CAR_TORQUE_SCALE * dt;

        // Yaw alone mid-flip: live.
        let mut c = car();
        let mut flip = airborne_dodge(&mut c, &forward_dodge, dt);
        airborne_flip_step(&mut c, &full_yaw(), &mut flip, dt);
        assert!(
            (c.angular_velocity.z - expected_yaw).abs() < 1e-4,
            "expected one tick of yaw air control mid-flip, got {}",
            c.angular_velocity.z
        );

        // Yaw while pulling back (a cancel): yaw still works, pitch (flip
        // torque and air control alike) contributes nothing.
        let mut c = car();
        let mut flip = airborne_dodge(&mut c, &forward_dodge, dt);
        let before = c.angular_velocity;
        let damp = neutral_damping_step(&c, dt);
        airborne_flip_step(
            &mut c,
            &ControllerInput {
                yaw: Some(1.0),
                ..full_pitch()
            },
            &mut flip,
            dt,
        );
        assert!((c.angular_velocity.z - expected_yaw).abs() < 1e-4);
        assert!((c.angular_velocity.y - before.y - damp.y).abs() < 1e-5);
    }

    #[test]
    fn a_diagonal_flips_full_cancel_leaves_its_roll_component() {
        // Forward-left dodge: rel_torque = (0.707, 0.707). A full pull-back
        // zeroes only the pitch (right-axis) half; the roll (forward-axis)
        // half keeps flipping at its own FLIP_TORQUE_X rate.
        let dt = 1.0 / 120.0;
        let mut c = car();
        let mut flip = airborne_dodge(
            &mut c,
            &ControllerInput {
                jump: true,
                pitch: Some(-1.0),
                roll: Some(-1.0),
                ..Default::default()
            },
            dt,
        );
        let (rel_forward, rel_right) = flip.unwrap().rel_torque;
        assert!((rel_forward - std::f32::consts::FRAC_1_SQRT_2).abs() < 1e-4);
        assert!((rel_right - std::f32::consts::FRAC_1_SQRT_2).abs() < 1e-4);

        let before = c.angular_velocity;
        let damp = neutral_damping_step(&c, dt);
        airborne_flip_step(&mut c, &full_pitch(), &mut flip, dt);
        assert!(
            (c.angular_velocity.x - before.x - damp.x - rel_forward * FLIP_ROLL_STEP_PER_TICK)
                .abs()
                < 1e-3,
            "expected the roll component untouched, got Δ{}",
            c.angular_velocity.x - before.x - damp.x
        );
        assert!(
            (c.angular_velocity.y - before.y - damp.y).abs() < 1e-5,
            "expected the pitch component zeroed (damping only)"
        );
    }

    #[test]
    fn a_pitched_cars_dodge_impulse_is_horizontal_at_full_magnitude() {
        // RB-PHYSICS-001-FR-081 finding 2: RocketSim applies the dodge
        // impulse along forwardDir2D/rightDir2D, so a nose-down car still
        // dodges exactly horizontally with the full DODGE_SPEED — this port
        // used to tilt the impulse with the car (a 30° nose-down dodge lost
        // half its speed to a downward component).
        let dt = 1.0 / 120.0;
        let pitch_down = std::f32::consts::FRAC_PI_6;
        for (input, expected_dir) in [
            (
                ControllerInput {
                    jump: true,
                    pitch: Some(-1.0),
                    ..Default::default()
                },
                Vec3::new(1.0, 0.0, 0.0),
            ),
            (
                ControllerInput {
                    jump: true,
                    roll: Some(1.0),
                    ..Default::default()
                },
                Vec3::new(0.0, 1.0, 0.0),
            ),
        ] {
            let mut c = car();
            // Nose down 30° about +y (right): forward = (cos, 0, -sin).
            c.orientation =
                rb_domain::Quat::new(0.0, (pitch_down / 2.0).sin(), 0.0, (pitch_down / 2.0).cos());
            c.update_inertia_tensor();
            let fwd = forward_axis(&c);
            assert!(
                fwd.z < -0.4,
                "expected a nose-down car, got forward {fwd:?}"
            );
            airborne_dodge(&mut c, &input, dt);
            assert!(
                (c.linear_velocity - expected_dir * DODGE_SPEED).length() < 1e-2,
                "expected a horizontal DODGE_SPEED impulse along {expected_dir:?}, got {:?}",
                c.linear_velocity
            );
        }
    }

    #[test]
    fn a_pitched_cars_wall_jump_dodge_impulse_is_horizontal_too() {
        let dt = 1.0 / 120.0;
        let pitch_down = std::f32::consts::FRAC_PI_6;
        let mut c = car();
        c.orientation =
            rb_domain::Quat::new(0.0, (pitch_down / 2.0).sin(), 0.0, (pitch_down / 2.0).cos());
        c.update_inertia_tensor();
        let mut boost = MAX_BOOST;
        let mut jump_held = false;
        let mut double_jump_available = true;
        let mut hold_remaining = 0.0;
        let mut flip = None;
        step_with_input_and_dodge_flip(
            &mut c,
            &ControllerInput {
                jump: true,
                pitch: Some(-1.0),
                ..Default::default()
            },
            false,
            Some(Vec3::new(0.0, -1.0, 0.0)),
            &mut boost,
            &mut jump_held,
            &mut double_jump_available,
            &mut hold_remaining,
            &mut flip,
            dt,
        );
        // Wall push-off along -y, jump along +z, dodge along the flattened
        // forward (+x) only — no dodge component along z.
        assert!(
            (c.linear_velocity.x - DODGE_SPEED).abs() < 1e-2,
            "got {:?}",
            c.linear_velocity
        );
        assert!((c.linear_velocity.y + WALL_JUMP_HORIZONTAL_SPEED).abs() < 1e-2);
        assert!((c.linear_velocity.z - JUMP_SPEED).abs() < 1e-2);
    }

    #[test]
    fn dodge_axes_2d_flatten_the_forward_and_fall_back_when_pointing_straight_up() {
        let mut c = car();
        let pitch_down = std::f32::consts::FRAC_PI_6;
        c.orientation =
            rb_domain::Quat::new(0.0, (pitch_down / 2.0).sin(), 0.0, (pitch_down / 2.0).cos());
        let (f, r) = dodge_axes_2d(&c);
        assert!((f - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-5);
        assert!((r - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-5);

        // Straight up (nose to +z): the flattened forward vanishes, so the
        // 3D axes are used instead of producing NaNs.
        let half = std::f32::consts::FRAC_PI_4;
        c.orientation = rb_domain::Quat::new(0.0, -half.sin(), 0.0, half.cos());
        let fwd = forward_axis(&c);
        assert!(fwd.z > 0.999, "expected nose up, got {fwd:?}");
        let (f, r) = dodge_axes_2d(&c);
        assert!((f - fwd).length() < 1e-5);
        assert!((r - right_axis(&c)).length() < 1e-5);
    }

    /// A car rolled 90 degrees about its local forward axis — its right
    /// axis becomes world +z and its up axis world -y. Drive.rs's test
    /// helpers only call `integrate::integrate_velocities`, never
    /// `integrate::integrate_transform`, so a car's `orientation` never
    /// actually changes step to step here — the only way to exercise
    /// orientation-dependent behaviour (the body-axis damping) in
    /// isolation is to set it directly like this.
    fn tilted_car() -> RigidBody {
        let mut c = car();
        c.orientation = rb_domain::Quat::new(
            std::f32::consts::FRAC_1_SQRT_2,
            0.0,
            0.0,
            std::f32::consts::FRAC_1_SQRT_2,
        );
        c.update_inertia_tensor();
        c
    }

    #[test]
    fn clamp_angular_speed_is_a_no_op_below_the_cap() {
        let mut c = car();
        c.angular_velocity = Vec3::new(1.0, 2.0, 0.0);
        clamp_angular_speed(&mut c);
        assert_eq!(
            c.angular_velocity,
            Vec3::new(1.0, 2.0, 0.0),
            "expected an already-under-cap angular velocity to pass through unchanged, got {:?}",
            c.angular_velocity
        );
    }

    #[test]
    fn clamp_angular_speed_scales_an_over_cap_velocity_down_to_the_cap_preserving_direction() {
        let mut c = car();
        c.angular_velocity = Vec3::new(0.0, 0.0, 20.0);
        clamp_angular_speed(&mut c);
        assert!(
            (c.angular_velocity.length() - MAX_CAR_ANGULAR_SPEED).abs() < 1e-4,
            "expected the clamp to scale magnitude down to exactly MAX_CAR_ANGULAR_SPEED, got \
             {:?}",
            c.angular_velocity
        );
        assert_eq!(
            c.angular_velocity.x, 0.0,
            "expected the clamp to preserve direction (x), got {:?}",
            c.angular_velocity
        );
        assert_eq!(
            c.angular_velocity.y, 0.0,
            "expected the clamp to preserve direction (y), got {:?}",
            c.angular_velocity
        );
        assert!(
            c.angular_velocity.z > 0.0,
            "expected the clamp to preserve direction (z), got {:?}",
            c.angular_velocity
        );
    }

    #[test]
    fn sustained_full_roll_input_never_exceeds_the_hard_angular_speed_cap() {
        // RB-PHYSICS-001-FR-057: before this cap existed, nothing bounded
        // the continuous angular-acceleration contribution air control adds
        // every step, so holding full roll input indefinitely spun a car
        // arbitrarily fast. Two real seconds of full roll at this car's own
        // mass/inertia gains far more than MAX_CAR_ANGULAR_SPEED (5.5
        // rad/s) worth of angular velocity if nothing clamps it, so this
        // test would fail without `clamp_angular_speed` in `step_with_input`'s
        // own step helper.
        let mut c = car();
        let mut boost = MAX_BOOST;
        let dt = 1.0 / 60.0;
        for _ in 0..(2.0 / dt) as u32 {
            step_with_input(&mut c, &full_roll(), false, &mut boost, dt);
        }
        assert!(
            c.angular_velocity.length() <= MAX_CAR_ANGULAR_SPEED + 1e-3,
            "expected sustained full roll input to cap out at MAX_CAR_ANGULAR_SPEED, got {:?} \
             (length {})",
            c.angular_velocity,
            c.angular_velocity.length()
        );
        assert!(
            c.angular_velocity.length() > MAX_CAR_ANGULAR_SPEED - 0.5,
            "expected sustained full roll input to actually reach the cap, not merely stay under \
             it, got {:?}",
            c.angular_velocity
        );
    }
}
