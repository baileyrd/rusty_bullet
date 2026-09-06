# RB-PHYSICS-001 — Physics Core Port

- Version: 0.105.0
- Status: In Progress (sphere-vs-plane, box-vs-plane, sphere-vs-box
  (ball-vs-car), box-vs-box (car-vs-car), body-vs-arena-wall, and
  ball-and-car-vs-curved-fillet collision all implemented, tested, and wired into a
  real N-body `PhysicsWorld` scene; ground-driving car input (throttle,
  steering), boost, handbrake, a variable-height ground jump, a double jump
  (plain or a directional dodge, itself flip-cancelable), a wall jump
  (itself dodgeable), air control (pitch/yaw/roll), a gentle landing
  auto-orientation assist, a modeled arena footprint
  (`PhysicsWorld::standard_arena`'s octagonal boundary plus a ceiling), and
  curved fillets throughout the arena's vertical boundary — wall-to-floor/
  wall-to-ceiling seams for all 9 walls (the 4 cardinal walls and, since
  FR-021, the 4 diagonal corner walls too), all 8 of the corner walls' own
  vertical edges where they meet their neighboring side/back walls (since
  FR-022), a spherical patch at each of the 16 compound corners where a
  vertical-edge fillet meets a floor- or ceiling-seam fillet (since
  FR-023), and, since FR-024, an actual goal-mouth window cut into each
  back wall with its own 3 rounded edges per goal, with the 4 diagonal
  corner walls' own floor/ceiling-seam arches and all 16 compound-corner
  fillets that touch them now sized with a distinctly larger,
  non-cardinal-wall radius since FR-025, and, since FR-026, a
  compound-corner fillet rounding each goal's own remaining sharp
  post-crossbar vertex too (4 total, 2 per goal), and, since FR-027, a car
  (box) actually being deflected by every one of those fillets too — a
  car's own box, ignored by every fillet's contact test until now, is
  tested via its 8 corners against the curved surface
  (`collision::box_vs_quarter_pipe`/`box_vs_corner_fillet`, the same
  "test every corner" technique `box_vs_plane` already used for a flat
  plane) — exact, not an approximation, for this containment-style
  contact (`RB-PHYSICS-001-FR-032` rigorously confirmed a once-suspected
  under-detection gap here doesn't actually exist) — implemented,
  and, since FR-028, a car actually driving into a goal too — a new
  `collision::box_vs_goal_wall` tests each of the car's 8 corners against
  the goal-mouth window exactly the same way `sphere_vs_goal_wall`
  already tests the ball's single center point — implemented, and, since
  FR-029, a modeled bounded interior behind each goal window too — a
  solid bounding box, stopping the ball or a car
  that passes through the window instead of letting it sail into
  unbounded open space — implemented, and, since FR-033, a real mass-spring
  net panel per goal catching the *ball* specifically (a car still passes
  through the panel's own footprint untouched, stopped instead by that same
  pre-existing solid bounding box) — implemented; and, since FR-030, every
  ball-vs-car and car-vs-car contact touching in the same step is now
  resolved by one shared, interleaved solve
  (`solver::resolve_dynamic_manifolds`) instead of a sequence of fully
  independent pairwise solves — implemented; and, since FR-031, a
  constant-calibration audit against the RocketSim/RLUtilities/RLBot-wiki
  community reverse-engineering effort corrected `JUMP_SPEED`,
  `JUMP_HOLD_ACCELERATION`, and split `MAX_CAR_SPEED` into separate
  boosted/unboosted caps (`UNBOOSTED_MAX_CAR_SPEED`), confirmed several
  more constants already correct, and explicitly flagged the rest as
  audited-but-still-uncalibrated rather than silently unresolved —
  implemented, and explicitly does NOT close `FR-005`'s real-data
  calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started; and, since FR-033, each
  goal gets a real mass-spring net panel catching the ball (`net::NetMesh`)
  — implemented, scoped to the ball only at the time (since `FR-038`, a
  car is caught too — see that entry); and, since FR-034, every contact's
  positional/penetration correction runs on its own separate split-impulse
  "push" channel instead of folding into the body's real velocity, so
  resolving deep overlap no
  longer injects the spurious velocity a combined Baumgarte term used to —
  implemented; and, since FR-035, `solver::resolve_dynamic_manifolds`
  (every ball-vs-car/car-vs-car manifold) warm-starts each call from the
  previous one's converged impulses instead of zero, converging measurably
  closer to the true answer for an under-converged manifold like FR-030's
  own extreme-mass-ratio example — implemented for that one call site;
  and, since FR-036, real source-level research (RocketSim, RLUtilities,
  and the RLBot wiki, read directly rather than guessed at) corrected the
  ball's collision radius (`92.75` to `93.15`, not the real games'
  separate, smaller inertia radius `91.25`, since this port's single
  unified radius field has no room for that distinction) and
  `arena::CEILING_Z` (`2044.0` to `2048.0`, confirmed the same reference
  point as RocketSim's `ARENA_HEIGHT`), and corrected two mis-documented
  claims that `arena::CORNER_LENGTH` and `arena::GOAL_DEPTH` were
  uncalibrated placeholders when both are in fact confirmed exact —
  implemented; and, since FR-037, sleeping
  (`body::RigidBody::update_sleep_state`/`wake`) forcibly zeroes a body's
  velocity once it's stayed below both a linear and an angular threshold
  for a sustained time, fixing the "bouncy resting contact never settles"
  limitation warm-starting alone couldn't (see FR-035's own entry) — a car
  wakes unconditionally on any genuinely active input, before that input's
  own force has had a chance to move it, so an asleep car can always start
  moving again — implemented; and, since FR-038, `net::NetMesh::step` takes
  every body that can touch a net (the ball and every car, via a `&mut
  [RigidBody]` slice) instead of the ball alone, closing this port's own
  former "a car still passes through untouched" Non-goal — no new collision
  code was needed, since `collision::contacts_between` already dispatches
  box-vs-sphere for a car against a net point the same way it always has for
  ball-vs-car — implemented; and, since FR-039, a wall jump at a corner
  (a car touching two walls at once) pushes off along every touched wall's
  normal summed and normalized, instead of picking whichever wall came
  first in `PhysicsWorld.walls` — implemented; and, since FR-040, a
  dedicated research pass looked for a real reference for
  `arena::FILLET_RADIUS`/`CORNER_ARCH_RADIUS` and found only one uncited,
  self-disclaimed-non-circular, likely-conflated wiki value — deliberately
  not adopted; both constants remain genuinely uncalibrated, closing this
  for real needs actual extracted mesh data — investigated; and, since
  FR-041, `resolve_dynamic_manifolds` scales each manifold's velocity-row
  impulse by `1 / k` for a body shared by `k >= 2` manifolds this step,
  narrowing FR-030's own documented "sandwiched" under-convergence gap
  (measured ~89.5 to ~32 units/s on that requirement's own test) at zero
  added iteration cost, with a global over-relaxation factor investigated
  and rejected (provably diverges for this exact case) — implemented; and,
  since FR-042, `box_vs_box`'s edge-edge contact point and face-clipping
  degenerate fallback were validated directly against `btBoxBoxDetector`'s
  own real source (not guessed at): this port's finite-segment closest-point
  derivation is confirmed strictly more rigorous than Bullet's own
  unclamped-infinite-line one, and this port's own decision to synthesize a
  contact rather than drop it on the same "should never happen" defensive
  branch Bullet's own authors left unproven too is confirmed a deliberate,
  favorable divergence; a candidate fix for the edge-edge tangent
  sign-selection heuristic was built and empirically tested against a
  brute-force ground truth and found genuinely mixed (better for realistic
  shallow penetration, worse for deep penetration, neither reliably
  optimal), so not adopted — investigated; and, since FR-043, this spec's
  own prior claim that Bullet's default restitution/friction combine mode
  is `btMax` was checked directly against `btManifoldResult`'s real source
  and found wrong — the actual default for both is an unclamped product
  (`a * b`), with no `max` mode anywhere in the reference — this port's own
  choice to keep averaging instead is now justified by a different, correct
  reason: unlike the reference's product, average preserves the identity
  that two surfaces sharing a coefficient combine back to that same
  coefficient, which matters given most bodies here still share the same
  uncalibrated placeholder value — investigated, doc-only correction, no
  runtime behavior changed; and, since FR-044, this spec's own top-level
  Non-goals section was found to still carry a stale "split impulse isn't
  implemented" bullet, contradicted by FR-034's own already-shipped
  implementation — corrected to match reality, matching the
  strikethrough-and-close convention already used for two other resolved
  Non-goals items in the same section — doc-only correction, no runtime
  behavior changed; and, since FR-045, `integrate.rs`'s own claims about
  matching Bullet's real `applyDamping`/`integrateVelocities`/
  `btTransformUtil::integrateTransform` were checked directly against that
  fetched reference source and confirmed byte-for-byte accurate, with one
  genuine finding: `integrate_transform`'s check-then-normalize
  degenerate-quaternion guard isn't defensive theater, it's necessary to
  match Bullet's real fallback choice (preserve the prior orientation, not
  reset to identity) — pinned by a new regression test — investigated,
  1 new test; and, since FR-046, `body.rs`/`mat3.rs`'s own Bullet-reference
  claims (local inertia formulas, `update_inertia_tensor`,
  `Mat3::scaled_columns`) were all confirmed byte-for-byte accurate against
  fetched reference source, with one genuine finding: `Mat3::from_quat`
  doesn't self-correct a non-unit-length input quaternion the way Bullet's
  own `setRotation` does, safe only because this function's single
  production call site always receives an already-renormalized
  orientation — pinned by a new regression test — investigated, 1 new
  test; and, since FR-047, `collision.rs`'s remaining closed-form shape
  pairings (`sphere_vs_plane`, `box_vs_plane`, `sphere_vs_box`,
  `sphere_vs_sphere`) were checked directly against fetched
  `btConvexPlaneCollisionAlgorithm`/`btSphereBoxCollisionAlgorithm`/
  `btSphereSphereCollisionAlgorithm` source: `sphere_vs_plane` and
  `sphere_vs_sphere` confirmed exact, `sphere_vs_box`'s deep-penetration
  face selection confirmed to reproduce Bullet's own exact face-check
  tie-break order (not just a mathematically valid alternative one), and
  one genuine, deliberate divergence found in `box_vs_plane` — real
  Bullet's default configuration generates only one contact point per
  frame via a single GJK support query, relying on several frames of
  persistent-manifold accumulation to reach a resting box's full 4-corner
  manifold, where this port computes all 4 corners exactly in one pass —
  confirmed a more-rigorous simplification in the same spirit as
  `box_vs_box`'s own FR-042 finding, not adopted — pinned by a new
  regression test — investigated, 1 new test; and, since FR-048,
  `solver.rs`'s own `restitution_curve`/`plane_space`/`setup_rows`/
  `resolve_row` were checked directly against fetched
  `btSequentialImpulseConstraintSolver`/`btContactSolverInfo`/`btVector3`
  source: `plane_space` confirmed byte-for-byte exact against real
  `btPlaneSpace1`, `restitution_curve` confirmed behaviorally exact (its own
  `.max(0.0)` folds in a clamp real Bullet applies at its one call site
  instead), `setup_rows`'s normal- and friction-row ERP/CFM/velocity-error/
  positional-error formulas confirmed exact against real
  `setupContactConstraint`/`setupFrictionConstraint`, `resolve_row`'s single
  unified two-bound resolver confirmed behaviorally equivalent to real
  Bullet's own two separate resolvers (one lower-bound-only, one two-bound)
  given the normal row's effectively-infinite upper limit, and all 6 of
  `btContactSolverInfo`'s cited default constants confirmed exact; one
  genuine, significant divergence found and not adopted — this port always
  derives both friction directions from a fixed, velocity-independent
  `plane_space(&contact.normal)` basis, where real Bullet's actual default
  aligns friction direction 1 with the tangential component of the current
  relative sliding velocity itself, falling back to `btPlaneSpace1` only
  when that velocity is negligible — a physically meaningful difference
  (a fixed two-axis friction limit can over- or under-estimate the true
  circular friction cone by up to `sqrt(2)` relative to the actual slide
  direction) left open for a dedicated future FR rather than folded into
  this reference-validation pass — investigated, 1 new test; and, since
  FR-049, that divergence was closed: a new `friction_directions` helper in
  `solver.rs` aligns friction direction 1 with the tangential component of
  the current relative sliding velocity (`relative_velocity` minus its own
  component along `normal`), matching real Bullet's own default, and
  direction 2 completes a right-handed basis via `dir1.cross(normal)`,
  matching real Bullet's own `lateralFrictionDir1.cross(normalWorldOnB)` —
  falling back to `plane_space`'s fixed basis both when tangential velocity
  is negligible (matching real Bullet's own `SIMD_EPSILON` threshold) and,
  found empirically fixing this crate's own test suite rather than
  something real Bullet's unguarded `normalize()` needs to handle, when a
  near-head-on collision's catastrophic floating-point cancellation leaves
  a residual tangential vector whose direction is dominated by rounding
  error rather than the true (near-zero) tangential velocity, occasionally
  landing degenerate relative to `normal`; confirmed via a dedicated
  isotropic-friction regression test verified to fail under the old fixed
  `plane_space`-only basis — investigated and adopted, 3 new tests; and,
  since FR-050, `net::NetMesh::step`'s own per-point sequential
  `solver::resolve_contacts_between` loop was investigated for the same
  independent-pairwise gap `RB-PHYSICS-001-FR-030` already fixed for
  ball-vs-car/car-vs-car — a ball or car pressing into the net commonly
  overlaps 2+ free points at once (`NET_POINT_RADIUS`'s own generous
  coverage radius), and this module's own prior "net-point mass is tiny
  enough to not matter" claim was found untested and false (`NET_POINT_MASS`
  is only half a typical ball's own mass); a dedicated single-shot test
  confirmed the old sequential loop is genuinely order-dependent for a
  perfectly symmetric impact, not merely slower to converge, and a
  `NetMesh::step`-level test measured the resulting real-world residual
  bias directly (~0.25 units/s out of a 2000 units/s impact) — adopted
  `solver::resolve_dynamic_manifolds`'s combined solve for every
  body-vs-point contact each sub-step, reducing that residual roughly
  15-fold; 2 new tests; and, since FR-051, the same independent-pairwise
  gap was found and closed one level up — `PhysicsWorld::step` resolved a
  body's contact against each different static shape type (ground, each
  wall, each curve, each corner fillet, each goal wall, each bounded wall)
  fully independently and sequentially, which a dedicated single-shot test
  (a ball wedged into a symmetric two-wall corner) confirmed is genuinely
  order-dependent, not merely slow to converge, and an actual
  `PhysicsWorld::step` end-to-end test confirmed manifests in real gameplay
  scenarios too (a car driving near a wall close to the ground, or wedged
  into any corner, is a routine occurrence, not an edge case); a new
  `solver::resolve_static_manifolds` generalizes `resolve_contacts` to
  combine every static-shape manifold `body` touches into one shared solve,
  replacing `step`'s old five-function-per-body sequence with a single
  `resolve_static_contacts` call; 2 new tests; and, since FR-052, the same
  independent-pairwise gap was found and closed one level higher still —
  `PhysicsWorld::step` resolved a body's now-combined static contacts and
  its combined dynamic manifolds as two separate solves, a body's static
  contact fully resolved and applied before the dynamic solve's own setup
  for that same body ever read the result, confirmed genuinely
  order-dependent by reusing FR-051's own two-wall corner setup with one
  wall replaced by a very-heavy dynamic body; a new
  `solver::resolve_manifolds` folds a step's static and dynamic manifolds
  into one shared solve, replacing `step`'s two separate calls with one;
  2 new tests; and, since FR-053, `solver::combine_friction`'s own result
  now clamps to `[-10.0, 10.0]`, matching real Bullet's own
  `calculateCombinedFriction` (confirmed by re-fetching `btManifoldResult.cpp`)
  — a detail FR-043's own reference read surfaced but didn't separately
  examine; currently inert for every friction coefficient this crate
  itself ever sets, adopted for reference conformance against every
  static/dynamic body's own unvalidated public `friction` field; 1 new
  test; and, since FR-054, a car face bigger than the goal window and
  centered on it (every corner outside the window while the window itself
  sits entirely inside the face) was confirmed, by a convex-hull argument
  distinct from FR-032's own, to collide exactly like an unwindowed plane
  — a genuine resolution, not a doc-only correction, of the one
  `box_vs_goal_wall` question FR-032's own finding didn't cover; the same
  investigation found the mirror-image case for `box_vs_bounded_wall` (a
  face bigger than a bound and centered on it) *is* a real under-detection
  gap, confirmed unreachable given this project's own car/ball sizes
  against the standard arena's own bound sizes and left open rather than
  fixed; 2 new tests; and, since FR-055, `arena::GOAL_HALF_WIDTH`/
  `GOAL_HEIGHT` were confirmed exact against the current RLBot wiki's own
  cited goal dimensions (fetched directly), closing the one goal-geometry
  constant question `GOAL_DEPTH`'s own earlier FR-036 confirmation hadn't
  reached, and a stale Open Questions passage that had never been updated
  when FR-036 shipped — still describing `GOAL_DEPTH` as an uncalibrated
  invention, contradicting FR-036's own already-shipped Requirements entry
  and this spec's own Non-goals section — was corrected; no new tests,
  matching FR-036's own precedent for a pure constant/doc-correctness
  change with no behavioral difference; static-contact warm-starting, `arena::FILLET_RADIUS`/
  `CORNER_ARCH_RADIUS` calibration, full convergence of the sandwiched
  case, a rigorous (non-heuristic) edge-edge nearest-pair selection,
  `box_vs_bounded_wall`'s own under-detection gap, and real-data
  calibration (including which combine mode, if either, actually matches
  real Rocket League) are open follow-up work)
- Owners: baileyrd
- Depends on: RB-VERIFY-003
- Supersedes: none

## Purpose and scope

Define and implement the physics core that produces a simulated
`PhysicsFrame` sequence for `RB-VERIFY-003` to score, per
[ADR-0004](../../adr/0004-bullet3-source-port-for-physics-core.md): a
from-scratch Rust port of specific Bullet3 (zlib-licensed) algorithms —
rigid-body integration and the sequential-impulse contact solver — not an
integrated third-party engine and not an unguided from-scratch design.

**Implemented scope** (in `crates/rb_physics_bullet`): a dynamic sphere
(the ball) and zero or more dynamic boxes (cars), each against a static
plane (the ground), against zero or more arena walls (`PhysicsWorld.walls`
— generic flat `StaticPlane`s, not a modeled Rocket League arena footprint),
and against every other dynamic body in the scene.
Gravity, damping, semi-implicit Euler velocity integration, exponential-map
orientation integration, analytic sphere-vs-plane and box-vs-plane contact
detection (the latter generating a 1-4 point manifold depending on the
box's orientation), analytic sphere-vs-box contact detection (always
exactly one point), a separating-axis box-vs-box contact test (0 to 4
points — a clipped face manifold or a single edge-edge point), and a
sequential-impulse solver with restitution and Coulomb friction (two
tangent directions) — resolving an entire ground-contact manifold together
(`resolve_contacts`) or an entire two-dynamic-body manifold
(`resolve_contacts_between`) — using a general 3x3 inverse inertia tensor
(`RigidBody`/`Mat3`, see Architecture) shared by both shapes.
`PhysicsWorld` carries `cars: Vec<RigidBody>` and, every step, collects
every non-empty ball-vs-car and car-vs-car manifold and resolves all of
them together in one shared, interleaved solve
(`solver::resolve_dynamic_manifolds`, since FR-030), so `box_vs_box` now
runs for real in a live scene, not just in isolation under a unit test.
Each car
also has a current `ControllerInput` (`PhysicsWorld::set_car_input`)
driving ground throttle and steering forces/torques on it (`drive`
module) — see FR-007 — plus a depletable boost resource
(`PhysicsWorld::set_car_boost`) giving it a flat forward force usable in
the air, unlike throttle — see FR-008 — a handbrake that temporarily
reduces its ground friction while held, letting it slide instead of
gripping cleanly through a turn — see FR-009 — a ground jump, fired once
per fresh press — see FR-010 — air control (pitch/yaw/roll torque about
its own local axes while airborne) — see FR-011 — a double jump, an
airborne impulse spendable once per airborne period and restored on
landing — see FR-012 — a wall jump, an outward-plus-upward impulse fired
while touching an arena wall, which also restores the double jump the same
way landing does — see FR-013 — a dodge, a directional variant of the
double jump fired when the stick is held in a direction at the moment of
the press — see FR-014 — variable height on that ground jump, adding
extra upward acceleration for as long as jump stays held, up to a cap —
see FR-015 — flip-cancel, letting a further jump press stop a dodge's
spin early instead of always completing it — see FR-016 — a wall-jump
dodge, the same directional-flip treatment applied to the wall jump's own
fresh press — see FR-017 — a landing auto-orientation assist, a
gentle continuous restoring torque nudging an airborne car's local up
axis back toward world up whenever it isn't actively air-controlling or
mid-jump-press — see FR-018 — a modeled arena footprint,
`PhysicsWorld::standard_arena` building Rocket League's real octagonal
boundary and a ceiling from the same generic `StaticPlane`/`with_wall`
machinery FR-013 introduced, rather than a caller assembling ad-hoc walls
itself — see FR-019 — and curved fillets throughout the arena's vertical
boundary: wall-to-floor/wall-to-ceiling transitions at all 9 walls (the 4
cardinal walls, FR-020, and, since FR-021, the 4 diagonal corner walls
too), all 8 of the corner walls' own vertical edges where they meet their
neighboring side/back walls (since FR-022) — a `StaticQuarterPipe` fillet
each deflecting the ball, and, since FR-027, a car too (via corner-testing,
confirmed exact rather than approximate for this containment-style contact
by `RB-PHYSICS-001-FR-032` — see FR-027) away from the sharp edge a flat wall and the floor/ceiling, or
two flat walls, would otherwise meet at — a `StaticCornerFillet` spherical
patch at each of the 16 compound corners
where a vertical-edge fillet meets a floor- or ceiling-seam fillet, near a
corner wall's own top/bottom endpoint (since FR-023), and, since FR-024,
an actual goal-mouth window — a `StaticGoalWall` — cut into each back
wall, letting the ball, and, since FR-028, a car too, pass straight
through into the goal, with its own 3 rounded edges (two posts and a
crossbar, `StaticQuarterPipe`s again), and, since FR-025, a distinctly
larger `arena::CORNER_ARCH_RADIUS` (instead of the cardinal walls' own
`arena::FILLET_RADIUS`) governing a corner wall's own floor/ceiling-seam
arches and all 16 compound-corner fillets that touch them, and, since
FR-026, a compound-corner fillet rounding each goal's own remaining sharp
post-crossbar vertex too (4 total, 2 per goal), and, since FR-029, a
modeled bounded interior volume behind each goal window too — a solid
bounding box (2 plain back-of-net `StaticPlane`s, 4 `StaticBoundedWall`
side walls reusing the goal posts' own planes, and 2 `StaticBoundedWall`
roofs reusing the crossbar's own plane), stopping the ball or a car that
passes through the window instead of letting it sail into unbounded open
space — see
FR-020/FR-021/FR-022/FR-023/FR-024/FR-025/FR-026/FR-027/FR-028/FR-029.

## Non-goals (this increment)

- **Team structure, car limits, or any Rocket-League-specific scene
  policy.** `with_car` can be called any number of times; this crate
  itself imposes no cap (Rocket League's real max is 8, but that's a
  gameplay/matchmaking rule, not a physics-core one) and has no concept of
  teams — a caller (eventually `rb_verify_cli`, once real multi-car
  recorded data exists) owns that policy.
- **Any geometry finer than a flat plane, single-radius edge fillet, or
  single-radius corner fillet per boundary segment.** (A car actually being
  deflected by a curved fillet
  was this same bullet's other half through FR-026 — that half is now
  resolved, see `RB-PHYSICS-001-FR-027`. A car actually driving into a
  goal was this bullet's third half through FR-027 — that half is now
  resolved too, see `RB-PHYSICS-001-FR-028`. A modeled bounded interior
  volume behind the goal window was this bullet's fourth half through
  FR-028 — that half is now resolved too, see `RB-PHYSICS-001-FR-029`. A
  genuine net *mesh* catching the ball — the "ball tangles in netting"
  behavior — was this bullet's fifth half through FR-029; that half is now
  resolved too, see `RB-PHYSICS-001-FR-033`'s `net::NetMesh` (a real
  mass-spring grid, not FR-029's own solid bounding box), scoped to the
  ball only at the time — a car's own contact against a net is no longer
  open, since `RB-PHYSICS-001-FR-038` closed it; a full 3D "sock" shape and
  bending stiffness remain open, see FR-033's own Non-goals and FR-038's
  own entry.)
  `arena::standard_curves`
  builds 24
  `StaticQuarterPipe` fillets — 16 floor/ceiling-seam fillets (one
  floor-side and one ceiling-side per wall, for all 9 walls including the 4
  diagonal corner walls since FR-021) plus, since FR-022, 8 vertical-edge
  fillets (one per corner wall endpoint, where it meets its neighboring
  side/back wall) — `arena::standard_corner_fillets` builds 16
  `StaticCornerFillet`s (since
  FR-023, one per compound corner where a vertical-edge fillet meets a
  floor- or ceiling-seam fillet) — and `arena::standard_goal_cutout_fillets`
  builds 6 more `StaticQuarterPipe`s (since FR-024, two posts and a crossbar
  per goal, rounding the goal-mouth window's own rim) and
  `arena::standard_goal_corner_fillets` builds 4 more `StaticCornerFillet`s
  (since FR-026, one per goal post per goal, rounding each goal's own
  post-crossbar compound corner) — every one of these now deflects the ball
  and, since FR-027, a car too: `collision::contacts_vs_quarter_pipe`/
  `contacts_vs_corner_fillet` dispatch a box to `box_vs_quarter_pipe`/
  `box_vs_corner_fillet`'s corner-testing (testing the box's
  own 8 corners — exact, not an approximation, for this containment-style
  contact, per `RB-PHYSICS-001-FR-032` — see FR-027's own entry) instead of the "always empty" no-op they returned for
  a box through FR-026. `contacts_vs_goal_wall`, unaffected by FR-027 since
  a goal wall isn't a curved fillet, deliberately ignored the goal-mouth
  window for a box too through FR-027 — that gap is now closed by
  `RB-PHYSICS-001-FR-028`'s own `collision::box_vs_goal_wall`, which tests
  each of a car's 8 corners against the window exactly the way
  `sphere_vs_goal_wall` tests the ball's single center point, so a car can
  now drive into a goal in this port too. The goal-mouth
  window itself (since FR-024) opened onto completely open, unbounded
  space through FR-028 — nothing behind it could stop the ball or a car
  that passed through; `RB-PHYSICS-001-FR-029` closes that gap with a
  modeled bounded interior volume (a solid bounding box: 2 back-of-net
  planes, 4 side walls, 2 roofs) — a solid volume standing
  in for the net's functional role of stopping the ball or car, at the time
  deliberately not yet a genuine net mesh; `RB-PHYSICS-001-FR-033` closes
  that gap for the ball specifically with a real mass-spring `net::NetMesh`
  panel in front of that same solid back-of-net plane (a car still passes
  through the panel's own footprint, stopped by the pre-existing solid
  volume instead — see FR-033's own entry). `FR-020`'s fillet
  radius (`arena::FILLET_RADIUS`, also reused by FR-022's vertical-edge
  fillets, FR-024's goal-cutout fillets, and FR-026's goal post-crossbar
  compound-corner fillets; FR-021's corner-wall seams and
  FR-023's compound corners instead reuse FR-025's `arena::CORNER_ARCH_RADIUS`) remains this project's own uncalibrated placeholder, not
  measured against real field mesh data — `SIDE_WALL_X`/`BACK_WALL_Y`/`CEILING_Z`
  are commonly-cited, sourced dimensions, and, since `RB-PHYSICS-001-FR-036`,
  so is `FR-019`'s corner-cut inset distance (`arena::CORNER_LENGTH`),
  confirmed exact against real extracted collision-mesh data rather than
  the uncalibrated placeholder this project previously took it for; `FR-024`'s
  own `arena::GOAL_HALF_WIDTH`/`GOAL_HEIGHT` were commonly-cited too, but,
  until `RB-PHYSICS-001-FR-055`, likewise not independently confirmed —
  FR-055 fetched the same current RLBot wiki page directly and found both
  values match its own cited "Goal center-to-post"/"Goal height" numbers
  exactly. `FR-029`'s own `arena::GOAL_DEPTH`,
  by contrast, was also already confirmed against the current RLBot wiki's own
  cited value (`RB-PHYSICS-001-FR-036`) — this project's earlier claim
  that no reference existed for it at all, making it an uncalibrated
  invention, was itself incorrect. FR-026's 4
  goal post-crossbar compound-corner fillets introduce no new radius
  constant at all — they reuse `FILLET_RADIUS` unchanged. `FR-025`'s
  own `arena::CORNER_ARCH_RADIUS` — the distinctly larger radius now
  governing a corner wall's own floor/ceiling-seam arches and all 16
  compound-corner fillets that touch them, in place of `FILLET_RADIUS` —
  is likewise this project's own uncalibrated placeholder, chosen only to
  read as visibly larger than `FILLET_RADIUS` (enforced at compile time by
  a `const _: () = assert!(CORNER_ARCH_RADIUS > FILLET_RADIUS);` check),
  not measured against real field mesh data either. `RB-PHYSICS-001-FR-040`
  looked for a real reference for both and came back empty-handed: the
  only candidate anywhere in this port's established reference tier is the
  RLBot wiki's uncited "wall bottom ramp radius: approx. 256, not
  circular", which doesn't distinguish the two constants, disclaims being
  a true circular radius, and is suspiciously identical to RLGym's
  unrelated `RAMP_HEIGHT` (a ramp's height, not a curve's radius) —
  deliberately not adopted for either constant (see `FILLET_RADIUS`'s own
  doc comment for the full finding). Both remain genuinely uncalibrated;
  closing this for real needs actual extracted collision-mesh geometry, not
  further wiki research.
- ~~Disambiguating or blending a car's simultaneous contact with two walls
  at a corner, for wall-jump purposes.~~ Physical collision resolution
  already handled a car touching two walls at once correctly — `step`
  resolves every wall independently, so both contacts are resolved on the
  same step regardless of the arena's shape (see FR-013). Which wall's
  normal `drive::apply_driven_forces` uses to decide a wall jump's push-off
  direction when a car is touching more than one wall at a corner at once
  used to pick whichever wall came first in `PhysicsWorld.walls`, not a
  blend of the two normals — a documented simplification for a case
  FR-019's new corner walls made reachable in the standard arena but this
  port's test scenes didn't yet exercise. `RB-PHYSICS-001-FR-039` closes
  that gap: the push-off direction is now every touched wall's normal
  summed and normalized, so a corner wall jump pushes diagonally away from
  the corner instead of along only one of the two walls.
- **Per-axis air-control torque, and any assisted/auto-rotation
  behavior.** FR-011's `AIR_CONTROL_TORQUE` is one shared constant for
  pitch, yaw, and roll; real Rocket League's actual per-axis rates differ
  from each other (roll fastest, pitch and yaw slower), which this port
  doesn't model. Real Rocket League also has an "air roll only" input mode
  and camera-relative stick mapping subtleties — none of that is modeled
  here; this increment is a direct, camera-independent pitch/yaw/roll
  torque, nothing more. (A landing auto-orientation assist is now
  implemented, as a separate, deliberately gentler continuous
  restoring torque rather than an extension of this per-axis input
  torque — see FR-018.)
- **A per-wheel tire/slip model.** Handbrake (FR-009) is modeled as a
  uniform, temporary reduction of the car's single `RigidBody.friction`
  value, not a distinct front/rear grip split or a slip-angle-driven tire
  curve — this port has no wheels at all (the car is one rigid box), so
  there's no rear-specific grip to lose the way a real car's handbrake
  works. See FR-009 and `drive`'s own module doc.
- **Consuming a recorded input sequence.** `PhysicsWorld::set_car_input`
  sets a car's *current* input, persisting until changed — a caller can
  drive a car through a whole `simulate()` run, or update it every step,
  but nothing here yet reads a real `RB-VERIFY-002` capture file
  frame-by-frame to do that automatically; that's `rb_verify_cli`'s
  concern once real capture data exists.
- ~~**Split impulse.** This port always takes Bullet's non-split
  contact-resolution branch (position and velocity correction combined into
  one `rhs`).~~ Implemented, see `RB-PHYSICS-001-FR-034`: every contact's
  normal row now solves a second, entirely separate "push" pseudo-velocity
  channel fed only by that contact's own positional/penetration error,
  applied directly to the body's position/orientation after the solve
  instead of folding into its real velocity — mirroring Bullet's own
  `btSolverBody::writebackVelocity`. `RB-PHYSICS-001-FR-044` found this
  bullet had gone stale (still asserting the pre-`FR-034` behavior,
  contradicting `FR-034`'s own already-shipped Requirements entry and
  `rb_physics_bullet::solver`'s own module doc comment) and corrected it.
- **Warm-starting for `resolve_contacts`/`resolve_contacts_between`.**
  `resolve_dynamic_manifolds` gained warm-starting in
  `RB-PHYSICS-001-FR-035`; the other two paths still re-derive every
  contact's impulses from zero each frame — a difference in convergence
  speed only, not correctness, since this port's fixed `SOLVER_ITERATIONS`
  already fully converges every scenario those two paths cover (see
  `rb_physics_bullet::solver`'s module doc). Sleeping is no longer part of
  this gap — `RB-PHYSICS-001-FR-037` implemented it, fixing the bouncy
  (restitution > 0) resting contact that previously never truly settled
  (see `world::tests::a_bouncy_resting_ball_actually_settles_once_asleep`).
- **Calibrated constants.** Gravity (-650 uu/s^2), restitution, and
  friction defaults are placeholders (commonly-cited community estimates
  or reasonable guesses), not confirmed against real Rocket League data —
  see `RB-PHYSICS-001-FR-005`.
- **`collision::box_vs_bounded_wall`'s corner-only overlap test can
  under-detect a face larger than the bound, centered on it.**
  `RB-PHYSICS-001-FR-054` found this while resolving a related, older
  open question about `box_vs_goal_wall` (see FR-054's own entry for both
  findings): a face whose every corner falls outside a
  `body::StaticBoundedWall`'s own bound, while the bound's rectangle sits
  entirely within that face's interior, has no corner touching solid
  material anywhere — `box_vs_bounded_wall` reports zero contacts even
  though the middle of the face is genuinely resting on real bound
  material. Deliberately not fixed: this project's only two
  `StaticBoundedWall`s (`arena::goal_side_wall`/`goal_roof`, both hundreds
  of units on their shortest side) are always far larger than this
  project's own established car or ball dimensions, so no scene this
  port's own public API can currently construct reaches it — closing it
  for real needs a proper 2D convex-polygon overlap test, not just
  corner-in/corner-out sampling.

## Context and terminology

- **Physics core**: `rb_physics_bullet`'s `PhysicsWorld` — whatever
  produces a simulated `PhysicsFrame` sequence, the thing `RB-VERIFY-003`
  scores.
- **Port** (as in "ported from Bullet3"): a from-scratch Rust translation
  of Bullet3's algorithms, not a binding or vendored build — see
  `THIRD_PARTY_NOTICES.md`.

## Requirements

- `RB-PHYSICS-001-FR-001` (implemented): `rb_physics_bullet::simulate`
  given a `PhysicsWorld` (initial sphere + plane state), a duration, and a
  fixed timestep, produces a `Vec<PhysicsFrame>` `RB-VERIFY-003::score` can
  consume directly.
- `RB-PHYSICS-001-FR-002` (implemented): Rigid-body integration
  (`integrate` module) ports `btRigidBody::applyGravity`/`applyDamping`/
  `integrateVelocities` and `btTransformUtil::integrateTransform`'s
  exponential-map orientation update.
- `RB-PHYSICS-001-FR-003` (implemented): Sphere-vs-static-plane contact
  detection and resolution (`collision`, `solver` modules) — restitution
  via `restitutionCurve`, Coulomb friction via two tangent constraint rows
  clamped to the current normal impulse, matching
  `btSequentialImpulseConstraintSolver`'s structure.
- `RB-PHYSICS-001-FR-004` (implemented): Extend to box-shaped car bodies,
  including their collision with the ball. Delivered: a general 3x3
  inverse inertia tensor (`Mat3`, recomputed from orientation each step
  via `RigidBody::update_inertia_tensor`, shared by both sphere and box
  bodies), analytic box-vs-plane contact generation (testing all 8
  corners against the plane — exact for a box vs. an infinite plane, not
  an approximation), multi-contact manifold resolution (the solver
  resolves all of a manifold's 1-4 points together, sharing one
  accumulated velocity delta, rather than one contact at a time), analytic
  sphere-vs-box contact generation (`collision::sphere_vs_box`, a
  closed-form closest-point-on-box query handling both the ordinary
  exterior case and a deep-penetration interior case), and a
  two-dynamic-body manifold solver path (`solver::resolve_contacts_between`)
  that carries both bodies' mass/inertia contributions instead of assuming
  one side is a static plane. `PhysicsWorld::step` now detects and resolves
  a ball-vs-car contact every step a car is present.
- `RB-PHYSICS-001-FR-005` (open, unblocked, not yet started, but now
  genuinely actionable): Calibrate gravity/restitution/friction constants
  against real recorded ground truth now that `RB-VERIFY-001`/
  `RB-VERIFY-002` produce real data (`PHASE-0-EXIT` closed) and a real
  capture exists, rather than relying on the current placeholder
  defaults. Prerequisite plumbing to actually produce a real fidelity
  score is implemented as `FR-076`/`FR-077`, and `FR-077`'s own first
  real run against the real capture now exists (see that entry's
  Interpretation note) — a very large whole-run divergence (mean car
  position distance `4508.71` uu against a `5120`-uu half-length field),
  consistent with total trajectory divergence rather than a small,
  directly-calibratable gap. The recommended follow-up diagnostic,
  `RB-VERIFY-003-FR-004`, has now actually run against this same real
  capture (see that spec's Verification plan for the full per-window
  numbers): the divergence is **abrupt**, not gradual — near-perfect for
  the run's first ~4 seconds, then a sharp derailment coinciding almost
  exactly with a diagonal dodge maneuver in the recorded input (a held
  first jump followed ~0.18s later by a second jump press with
  `pitch=-1, roll=-1`), after which the two trajectories fluctuate in a
  persistently large but roughly bounded range rather than growing
  further. Leading hypothesis, **not yet isolated or confirmed**: this
  port's dodge applies the flip's entire spin as a single instantaneous
  angular-velocity kick, while `FR-069` already found and documented (but
  left unimplemented) that real Rocket League's flip spin is a continuous
  per-tick torque over a `0.65`s window — a structurally different
  mechanism whose result would plausibly diverge sharply from an
  instantaneous kick. `FR-079` then actually carried out that isolated
  replay: it confirms the maneuver as the proximate cause (divergence
  reproduces standalone with no earlier-drift head start) but refines the
  hypothesis into two parts — an orientation-rate divergence that begins
  smoothly *during the grounded jump hold, before the dodge itself fires*
  (reaching `~12.5°` by the time the dodge triggers), which the dodge's
  own orientation-relative impulse then amplifies into a dramatically
  different translation kick, plus a likely-separate post-dodge spin-rate
  mismatch consistent with `FR-069`'s own finding. This requirement still
  hasn't started; the concrete next step is now isolating the pre-dodge
  orientation-rate divergence's own root cause (see `FR-079`'s own entry
  for the full evidence), not yet the dodge's spin model in isolation.
- `RB-PHYSICS-001-FR-006` (car-vs-car collision, implemented): A general
  separating-axis test between two oriented boxes (`collision::box_vs_box`),
  producing either a clipped face manifold (0-4 points) or a single
  edge-edge point, reusing the two-body solver path FR-004 introduced
  (`resolve_contacts_between` was generalized from a single contact to a
  manifold for this). `PhysicsWorld` now carries `cars: Vec<RigidBody>`
  (any number, via repeated `with_car` calls) and resolves every car-vs-car
  pair each step, so this pairing runs for real in a live scene — not just
  under a unit test, as it did before multi-car `PhysicsWorld` support
  landed.
- `RB-PHYSICS-001-FR-007` (driven car input, ground throttle and steering,
  implemented; since `FR-082` step (a) both act through the wheels — per-
  wheel engine force and steer angle, see that entry): `drive::apply_driven_forces` couples
  `rb_domain::ControllerInput` into forces/torques on a car: throttle
  (accelerate/reverse along the car's local forward axis, capped at
  `MAX_CAR_SPEED`) and steering (yaw torque about the car's local up axis,
  scaled by current speed so a stationary car can't turn in place), both
  gated on the car actually touching the ground. `PhysicsWorld` gains
  `set_car_input` (persists a car's current input across steps) and
  `frame()` now reports each car's actual driving input instead of
  `None`. A car with no input set behaves exactly as before this
  requirement existed (neutral `ControllerInput::default()` applies zero
  force/torque).
- `RB-PHYSICS-001-FR-008` (boost, implemented): `drive::apply_driven_forces`
  also applies a flat forward force (`BOOST_ACCELERATION * mass`, not
  speed-tapered like throttle) along the car's local forward axis whenever
  `ControllerInput.boost` is set and the car has boost remaining, capped at
  the same `MAX_CAR_SPEED` ceiling as throttle. Unlike throttle and
  steering, boost is *not* gated on ground contact — it's modeled as a
  rocket, not an engine, so it works airborne too (this requirement's own
  original claim that it works "identically" airborne was itself wrong —
  `RB-PHYSICS-001-FR-056` found and corrected it: the *gating* is
  identical, not the acceleration magnitude, which real Rocket League
  genuinely varies by ground contact). Boost is a
  depletable resource: `PhysicsWorld` gains a parallel `car_boost: Vec<f32>`
  (initialized to a full tank, `drive::MAX_BOOST`, by `with_car`) and
  `set_car_boost` to set it directly; holding boost input drains the tank
  at `BOOST_CONSUMPTION_RATE` per second whenever held, even if the forward
  force itself doesn't apply because the car is already at `MAX_CAR_SPEED`
  (matching real Rocket League's "holding boost drains fuel regardless of
  whether it's still accelerating you"), and the tank clamps at zero
  (no effect once empty). `frame()` now reports each car's actual
  `boost_amount` instead of a hardcoded `0.0`.
- `RB-PHYSICS-001-FR-009` (handbrake, implemented; since `FR-082` step
  (a) the handbrake is the wheels' lateral friction factor, not a chassis
  friction swap — see that entry): `drive::apply_driven_forces`
  temporarily multiplies the car's `RigidBody.friction` by
  `HANDBRAKE_FRICTION_MULTIPLIER` (an uncalibrated placeholder) whenever
  `ControllerInput.handbrake` is held and the car is grounded, restoring it
  to the car's own base friction otherwise — gated on ground contact like
  throttle/steering (a free-floating box has no wheels to lock regardless).
  `PhysicsWorld::with_car` snapshots each car's constructed `friction` into
  a new parallel `car_base_friction: Vec<f32>` so handbrake has the car's
  own value, not a hardcoded default, to restore to on release. This models
  handbrake as a temporary grip reduction — letting the car's existing
  momentum carry it into a slide rather than tracking a new heading
  cleanly — reusing the ground-contact solver's existing Coulomb-friction
  machinery rather than a separate lateral-slip system (this port has no
  per-wheel tire model to build a real rear-grip-loss mechanic on top of;
  see Non-goals).
- `RB-PHYSICS-001-FR-010` (single ground jump, implemented; since
  `FR-082` step (a) fired along the car's own up on a three-wheel ground
  test — see that entry):
  `drive::apply_driven_forces` applies a fixed `JUMP_SPEED` instantaneous
  upward velocity change (via `RigidBody::apply_impulse`, not a continuous
  force) on the *rising edge* of `ControllerInput.jump` while the car is
  grounded — a fresh press, not merely "held"; holding the button through
  the resulting airborne period doesn't re-fire it, and releasing then
  re-pressing while still airborne doesn't fire it either (no double jump
  in this scope). Edge detection needs one bit of state per car,
  remembering "was jump held as of the previous step" — `PhysicsWorld`
  gains a parallel `car_jump_held: Vec<bool>` (initialized `false` by
  `with_car`) threaded into `apply_driven_forces` as `jump_held`, the same
  pattern `boost_amount` already uses for cross-call state. A second
  airborne jump and wall jump are explicitly out of scope for this
  requirement — see FR-012. Variable jump height (holding for a higher
  jump) was originally out of scope here too, but is now implemented as
  FR-015.
- `RB-PHYSICS-001-FR-011` (air control, implemented):
  `drive::apply_driven_forces` applies torque about the car's local right,
  up, and forward axes, scaled directly by `ControllerInput.pitch`/`yaw`/
  `roll` (each an `Option<f32>`, `None` treated as zero) times one shared
  `AIR_CONTROL_TORQUE` constant, gated on the car *not* touching the
  ground — the mirror image of throttle/steering/handbrake/jump's
  ground-only gating, so it never competes with ground steering for the
  yaw axis. Unlike ground steering, air control is not speed-scaled: a
  car can spin from a standing start in the air, since there's no wheel
  grip requiring momentum. `AIR_CONTROL_TORQUE` is an uncalibrated
  placeholder shared by all three axes — a documented simplification,
  since real Rocket League's pitch/yaw/roll rates differ from each other
  — see Non-goals.
- `RB-PHYSICS-001-FR-012` (double jump, implemented):
  `drive::apply_driven_forces` fires one additional, identical `JUMP_SPEED`
  instantaneous upward velocity change on a fresh (`jump_pressed`) press of
  `ControllerInput.jump` while the car is airborne — reusing the ground
  jump's own rising-edge detection rather than a second edge-detector, and
  reusing `JUMP_SPEED` itself rather than a separately-calibrated constant
  (this port has no public reference for a distinct double-jump speed
  either). Gated on a new per-car `double_jump_available` flag instead of
  on ground contact: landing (any step where `on_ground` is true)
  unconditionally restores it to `true`, and a fresh airborne press that
  fires the double jump sets it to `false` until the next landing, so it
  can fire at most once per airborne period regardless of how many times
  jump is released and re-pressed after that. `PhysicsWorld` gains a
  parallel `car_double_jump_available: Vec<bool>` (initialized `true` by
  `with_car`, matching a car that's effectively "just landed" before its
  first step) threaded into `apply_driven_forces` alongside `jump_held`.
  Deliberately excludes the directional "dodge" impulse/torque a real
  double jump pairs with — see Non-goals.
- `RB-PHYSICS-001-FR-013` (arena walls and wall jump, implemented):
  `PhysicsWorld` gains `walls: Vec<StaticPlane>` (via a new `with_wall`
  builder, mirroring `with_car`) — generic flat static-plane geometry every
  body (ball and cars alike) now collides with via the same
  body-vs-static-plane machinery the ground already uses
  (`resolve_ground_contact` is renamed `resolve_plane_contact` and called
  once per wall in addition to the ground, for both the ball and every
  car). On top of that physical substrate, `drive::apply_driven_forces`
  gains a wall jump: a fresh `jump_pressed` press while airborne and
  touching a wall (`wall_normal: Some(normal)`, computed by `PhysicsWorld`
  up front the same way `on_ground` is) fires an impulse combining a new
  `WALL_JUMP_HORIZONTAL_SPEED` (uncalibrated placeholder) outward along the
  wall's normal with `JUMP_SPEED` upward. Wall jump takes priority over the
  double jump on that press but is otherwise independent of it: it doesn't
  consume `double_jump_available`; merely touching a wall (whether or not
  jump is pressed) unconditionally restores it, the same "any surface
  contact refills your second jump" rule landing already uses — so a
  player can wall-jump and still have a double jump left afterward, and
  can wall-jump again off the same or a different wall with no
  once-per-airborne-period limit of its own. Deliberately excludes
  variable jump height and any modeled arena footprint beyond generic flat
  walls — see Non-goals. (The directional "dodge" a real wall jump can pair
  with was excluded at the time this requirement first shipped; it is now
  implemented as FR-017.)
- `RB-PHYSICS-001-FR-014` (dodge, implemented): the double jump's fresh
  press (see FR-012) now checks `ControllerInput.pitch`/`roll` at the
  moment it fires: if either exceeds a new `DODGE_DEADZONE`, it fires a
  directional dodge instead of the plain vertical double jump — a purely
  horizontal `DODGE_SPEED` impulse (along `forward_axis`, scaled by
  `pitch`, and/or `right_axis`, scaled by `roll`) plus an instantaneous
  `DODGE_ANGULAR_SPEED` spin added directly to `RigidBody.angular_velocity`
  about the perpendicular axis (`right_axis` for pitch, `forward_axis` for
  roll) — reusing air control's own pitch/roll axis and sign conventions,
  so a forward dodge looks like a fast version of a forward air-control
  pitch. Both axes can contribute at once (a diagonal dodge), simply summed
  rather than normalized — a documented simplification, since real Rocket
  League normalizes the stick direction so a diagonal dodge isn't faster
  than an axis-aligned one. A dodge has no vertical component (unlike the
  plain double jump); below `DODGE_DEADZONE` on both axes, the plain
  vertical double jump fires exactly as it did before this requirement.
  Either way the press still spends the shared `double_jump_available`
  resource — a dodge and a plain double jump aren't separate resources.
  Wall jump was untouched at the time this requirement shipped: it never
  checked `pitch`/`roll` at all, so touching a wall always got the fixed
  wall-jump push-off, never a dodge. `DODGE_SPEED` is now `pub`, alongside
  the newly-`pub` `WALL_JUMP_HORIZONTAL_SPEED`, so `world.rs`'s end-to-end
  tests can assert against — and distinguish between — both. (The wall jump
  itself gained its own dodge variant later, as FR-017.)
- `RB-PHYSICS-001-FR-015` (variable jump height, implemented): the ground
  jump (FR-010) gains a hold window — continuing to hold
  `ControllerInput.jump` after the fresh press that fires it adds a
  continuous `JUMP_HOLD_ACCELERATION` upward force, for up to
  `JUMP_HOLD_MAX_DURATION` seconds, on top of the press's own fixed
  `JUMP_SPEED` impulse. A new per-car `jump_hold_time_remaining: f32`
  (`PhysicsWorld`'s parallel `car_jump_hold_time_remaining: Vec<f32>`,
  starting `0.0`) is checked and decremented at the very top of
  `apply_driven_forces` — against whatever value the *previous* call left
  it at — before that same call's own `on_ground`/`jump_pressed` handling
  can re-arm it to `JUMP_HOLD_MAX_DURATION`, so a fresh ground-jump press's
  own step always fires only the plain `JUMP_SPEED` impulse; only
  continued holding into later calls earns the extra height. Releasing
  `jump` zeroes the remaining window immediately, stopping the extra
  acceleration right away even if time was left — matching real Rocket
  League's held-vs-tapped jump height difference. Scoped to the ground
  jump alone: the double jump, a dodge, and the wall jump are each still a
  single fixed instantaneous impulse, unaffected by how long jump is held,
  since firing any of them requires releasing jump first (a fresh press),
  which itself unconditionally zeroes the ground jump's hold window before
  that press's own branch ever fires. `JUMP_HOLD_MAX_DURATION` and
  `JUMP_HOLD_ACCELERATION` are both uncalibrated placeholders — this port
  has no public reference for real Rocket League's actual hold-window
  length or acceleration the way `JUMP_SPEED` does.
- `RB-PHYSICS-001-FR-016` (flip-cancel, implemented; superseded by
  `RB-PHYSICS-001-FR-080` step (c), which replaced this jump-press trigger
  and outright zeroing with the real pitch-hold mechanism): a dodge's spin
  (FR-014) can be canceled early — a further fresh `ControllerInput.jump`
  press while airborne, not touching a wall, with `double_jump_available`
  already spent by that dodge, zeroes `RigidBody.angular_velocity` outright
  instead of leaving the flip to spin indefinitely. A new per-car
  `dodge_flip_active: bool` (`PhysicsWorld`'s parallel
  `car_dodge_flip_active: Vec<bool>`, starting `false`) tracks whether the
  most recent double-jump-or-dodge press was a dodge whose spin hasn't been
  canceled or superseded yet: the directional-dodge branch sets it `true`;
  the plain-double-jump branch explicitly sets it `false` rather than
  leaving it alone, so a stale `true` left over from an earlier,
  already-landed-from dodge can't leak into spuriously canceling a later,
  unrelated plain double jump's non-existent flip. Flip-cancel doesn't
  touch linear velocity (the dodge's own translation is unaffected) and
  doesn't consume or restore `double_jump_available` (already spent by the
  dodge that set the flag). Wall jump keeps its existing priority — checked
  first in the airborne branch, unchanged — so a fresh press while touching
  a wall always wall-jumps, never flip-cancels. This port has no timed
  flip animation to interrupt (a dodge is one instantaneous
  angular-velocity kick, not a sustained torque over a fixed duration —
  see FR-014), so "mid-flip" here means "any time before landing or a wall
  touch re-arms the double jump," a documented simplification of real
  Rocket League's actual flip-duration window. No new physics constants —
  this is a state-flag-gated zeroing action, not a magnitude to calibrate.
  (Since `RB-PHYSICS-001-FR-080` step (b), the flag is the real
  `Option<DodgeFlip>` flip state and landing clears it; step (c) then
  removed this trigger and its outright zeroing altogether in favour of the
  real pitch-hold scale — a further jump press mid-flip now does nothing.)
  `RB-PHYSICS-001-FR-070` later fetched RocketSim's real `Car.cpp` and found
  this simplification runs deeper than the flip-duration window alone: real
  Rocket League's own flip-cancel mechanism (continuous pitch-stick input,
  proportional, pitch-axis-only) differs substantially from this
  jump-press-triggered, all-axis, binary one.
- `RB-PHYSICS-001-FR-017` (wall-jump dodge, implemented): the wall jump's
  own fresh press (see FR-013) now checks `ControllerInput.pitch`/`roll`
  the same way the ground double jump's press does (FR-014): at or above
  `DODGE_DEADZONE` on either axis, it fires a **wall-jump dodge** instead
  of the plain fixed push-off — the same outward-plus-upward impulse
  combined with a horizontal `DODGE_SPEED` component and
  `DODGE_ANGULAR_SPEED` spin (identical axis/sign conventions to the ground
  dodge), leaving `dodge_flip_active` set so its spin is flip-cancelable
  (FR-016) exactly like a ground dodge's. Below `DODGE_DEADZONE` on both
  axes, the plain wall jump fires exactly as it did before this
  requirement, still never touching `double_jump_available`. Unlike the
  plain wall jump, a wall-jump dodge *does* consume `double_jump_available`
  — the same resource a ground dodge spends — a deliberate simplification:
  since touching a wall unconditionally restores `double_jump_available`
  before this check ever runs (see FR-013), gating the dodge variant on it
  would be vacuous (it's always true here), so this port instead has the
  dodge variant spend it, the same way a ground dodge does, keeping the
  invariant "`dodge_flip_active` is only ever true while
  `double_jump_available` is false" intact without any changes to
  flip-cancel's own branch ordering or new landing/wall-touch-clearing
  logic. This port has no way to separately account for "a wall touch
  refilled the double jump, then the wall-jump dodge spent it" versus a
  genuinely independent wall-dash resource, and real Rocket League's
  precise accounting here isn't public to the precision this project would
  need to model that distinction. No new physics constants — reuses
  `DODGE_SPEED`/`DODGE_ANGULAR_SPEED`/`WALL_JUMP_HORIZONTAL_SPEED`/
  `JUMP_SPEED`, all already introduced by earlier requirements. Two
  pre-existing tests (`drive::wall_jump_fires_instead_of_a_dodge_when_touching_a_wall`, `world::wall_jump_still_fires_instead_of_a_dodge_when_touching_a_wall`) asserted the *old* "wall jump always ignores stick
  input" premise this requirement deliberately reverses; both were
  repurposed (not silently deleted) to assert the new wall-jump-dodge
  behavior instead, keeping their scenario (touching a wall with
  directional stick input) but updating the expected outcome.
- `RB-PHYSICS-001-FR-018` (landing auto-orientation assist, implemented;
  superseded and removed by `RB-PHYSICS-001-FR-071`, which replaced this
  invented nudge with real Rocket League's own air-control damping):
  `drive::apply_driven_forces` gains a gentle continuous restoring torque,
  applied while airborne, that nudges the car's local up axis back toward
  world up — real Rocket League auto-corrects a car's orientation somewhat
  on approach to landing; this port has no ground-proximity raycast or
  distance query to replicate that trigger condition, so instead the assist
  applies continuously whenever airborne, gated on two conditions instead:
  no active `pitch`/`roll` air-control input this step (`pitch == 0.0 &&
  roll == 0.0`, so the assist never fights the player's own air control —
  it only fills in when the stick is neutral) and no fresh
  `ControllerInput.jump` press this step (so it never interacts, within the
  same `integrate_velocities` call, with a dodge's, wall-jump-dodge's,
  double-jump's, or flip-cancel's own same-step direct velocity/
  angular-velocity change). The correction itself is `up_axis(car).cross(
  &world_up) * LANDING_AUTO_UPRIGHT_TORQUE`: since both vectors are unit
  length, the cross product's magnitude is already proportional to the
  sine of the car's tilt off level, so a level car earns no correction and
  a heavily tilted one earns a proportionally stronger nudge, with no
  separate angle computation needed. `LANDING_AUTO_UPRIGHT_TORQUE` is a new
  uncalibrated placeholder, deliberately one full order of magnitude
  smaller than `AIR_CONTROL_TORQUE` so the assist reads as "gentle
  assistance," not "full control." Known, accepted, unaddressed limitation:
  a car resting exactly upside-down gives an exactly antiparallel
  `up_axis`/`world_up` pair, whose cross product is also zero — no
  correction is computed in that unlikely exact singularity. No new
  `PhysicsWorld` state — the assist is a pure function of the car's current
  orientation, input, and ground contact, all already in scope.
- `RB-PHYSICS-001-FR-019` (modeled arena footprint, implemented): a new
  `arena` module builds Rocket League's real standard-arena boundary
  entirely from FR-013's existing generic `StaticPlane`/`with_wall`
  machinery — no new collision code, since a ceiling and a corner-cut wall
  are each just another flat plane. `arena::standard_ground` is the flat
  floor at `z = 0` (identical to the `flat_ground()` test helper this crate
  has used since v0); `arena::standard_walls` returns 9 `StaticPlane`s: 2
  side walls (`x = ±SIDE_WALL_X`), 2 back walls (`y = ±BACK_WALL_Y`), a
  ceiling (`z = CEILING_Z`), and 4 diagonal corner walls (one per
  quadrant) that cut off the true rectangular corner where a side wall
  would otherwise meet a back wall at 90 degrees — giving the field its
  real octagonal footprint instead of a plain rectangle. `SIDE_WALL_X`
  (4096), `BACK_WALL_Y` (5120), and `CEILING_Z` (2048, corrected from an
  earlier `2044` by `RB-PHYSICS-001-FR-036`) are commonly-cited
  community-measured field dimensions (the same sourcing convention as
  `drive::MAX_CAR_SPEED`/`JUMP_SPEED`); the corner walls' inset distance
  (`CORNER_LENGTH`, equal along both axes, giving a 45-degree cut) is
  likewise now confirmed exact, not the uncalibrated placeholder this
  project once took it for (`RB-PHYSICS-001-FR-036`) — its flat corner-wall
  plane matches real extracted collision-mesh data precisely, even though
  the real arena's corners aren't a single flat plane all the way to the
  floor/ceiling seam (they curve, and blend into ramps this port doesn't
  model either). `PhysicsWorld::standard_arena`
  is a new convenience constructor — `PhysicsWorld::new(ball,
  arena::standard_ground())` followed by a `with_wall` call for each of
  `standard_walls()`'s 9 planes — offered alongside, not replacing,
  `PhysicsWorld::new`/`with_wall`'s existing ad-hoc-wall capability (a
  caller building a non-standard test scene, as most of this crate's own
  tests do, still uses those directly). Still not modeled at the time this
  requirement shipped: curved wall-to-floor/wall-to-ceiling transitions
  (now implemented for the ball at the 4 cardinal walls, see FR-020), goal
  cutouts in the back walls, and disambiguating or blending a car's
  simultaneous contact with two walls at a corner for wall-jump purposes
  (see Non-goals) — `resolve_plane_contact`'s own physical resolution of a
  car touching two walls at once already works correctly regardless (each
  wall is resolved independently every step), only the wall-jump push-off
  direction picker still isn't.
- `RB-PHYSICS-001-FR-020` (curved wall-to-floor/wall-to-ceiling transitions,
  implemented): a new `body::StaticQuarterPipe` shape — an immovable
  partial-cylinder fillet connecting two perpendicular flat planes, infinite
  along its own axis like `StaticPlane` — and `collision::contacts_vs_quarter_pipe`, a sphere-only narrow-phase test (a box/car
  always returns no contact — see FR-020's own Non-goals). The playable side
  is the *inside* of the fillet's concave face (the same geometry a
  skateboard quarter-pipe is named after and ridden on the inside of): a
  point is governed by a fillet at all only when its direction from
  `axis_point`, projected perpendicular to `axis_direction`, falls within
  the 90-degree sector from `sector_start` to `sector_end` (checked via
  `dot(dir, sector_start) >= 0 && dot(dir, sector_end) >= 0`, exact for a
  90-degree sector since the two vectors are perpendicular); within that
  sector, contact fires as the sphere's surface approaches or crosses the
  fillet's own radius *from the inside*, and the correction pushes the
  sphere back toward the axis — the opposite direction convention from
  `sphere_vs_plane`'s always-away-from-the-plane push. `StaticQuarterPipe::between_planes(plane_a, plane_b, radius, axis_direction)` derives a
  fillet's axis/sector automatically from the two flat planes it bridges
  (offsetting each plane inward by `radius` along its own normal, and
  negating each plane's normal for the sector vector pointing back to its
  own tangent point) — exact whenever `plane_a`/`plane_b`'s normals and
  `axis_direction` form an orthonormal basis, which only requires the two
  bridged planes to be mutually *perpendicular* (true for every arena wall's
  own floor/ceiling seam, cardinal or diagonal — see FR-021 — not for two
  walls meeting at a corner, which generally aren't perpendicular — see
  Non-goals). `PhysicsWorld` gains `curves: Vec<StaticQuarterPipe>` and
  a `with_curve` builder (mirroring `walls`/`with_wall`), resolved via a new
  `resolve_curve_contact` alongside `resolve_plane_contact` for the ball and
  every car (a no-op for cars at the time this requirement shipped, since
  the box arm of `contacts_vs_quarter_pipe` was always empty — now
  implemented, see FR-027). `solver::resolve_contacts`'s second parameter changed
  from `&StaticPlane` to plain `restitution: f32, friction: f32` — the only
  two fields it ever actually used — so this same solver path serves a
  `StaticQuarterPipe` fillet exactly as it already served a `StaticPlane`,
  with no new solver code needed. `arena::standard_curves` builds the 8
  fillets (floor-side and ceiling-side, for each of the 4 cardinal walls)
  the standard arena needs via `between_planes`, using a new uncalibrated
  placeholder `FILLET_RADIUS` (this port has no verified reference for the
  real transition radius either); `PhysicsWorld::standard_arena` now adds
  these 8 curves alongside its existing 9 walls. Still not modeled at the
  time this requirement shipped: a car actually being deflected by a
  fillet (now implemented, see FR-027), fillets at the 4 diagonal corner
  walls (now implemented, see FR-021), and goal cutouts (see Non-goals).
- `RB-PHYSICS-001-FR-021` (curved corner-wall-to-floor/wall-to-ceiling
  transitions, implemented): extends FR-020's fillet treatment to the 4
  diagonal corner walls `FR-019` introduced — `arena::standard_curves` now
  builds 16 `StaticQuarterPipe`s total (still one floor-side and one
  ceiling-side fillet per wall, now for all 9 walls) instead of 8.
  `StaticQuarterPipe::between_planes` itself needed no code changes: its
  only real correctness requirement is that the two bridged planes'
  normals, plus `axis_direction`, form an orthonormal basis, which only
  needs the two planes to be mutually *perpendicular* — true for a corner
  wall meeting the floor or ceiling regardless of the corner wall's own
  horizontal rotation (a vertical wall's normal always has zero Z component,
  and the floor/ceiling's is always purely Z), not something limited to
  axis-aligned cardinal walls the way FR-020's own doc comment had
  (incorrectly) claimed. The only new work is in `arena.rs`'s
  `standard_curves`: a cardinal wall's fillet axis direction was always
  hand-picked as a coordinate axis (`(0,1,0)` for a side wall, `(1,0,0)` for
  a back wall — each wall's own "along the wall" direction), but a corner
  wall's "along the wall" direction isn't a coordinate axis, so it's instead
  computed via a cross product, `floor.normal.cross(&wall.normal)` (and the
  ceiling equivalent) — already exactly unit length by construction (the two
  operands are always-perpendicular unit vectors, so `|a x b| = |a||b|
  sin(90 deg) = 1` exactly, up to floating-point precision), so no
  `.normalize()`/`.unwrap()` is needed or used (avoiding a
  `clippy::unwrap_used` violation in production code, which the workspace's
  lint config promotes to a hard CI error). A new `corner_wall_plane(sx,
  sy)` helper in `arena.rs` factors out the existing (unchanged)
  `standard_walls` corner-wall construction so `standard_curves` can reuse
  it, rather than duplicating the corner-wall plane math. `PhysicsWorld::standard_arena` picks up the extra 8 curves automatically, since it
  already loops over every curve `arena::standard_curves()` returns. Still
  not modeled at the time this requirement shipped: a car actually being
  deflected by any fillet (now implemented, see FR-027), a fillet at a
  corner wall's own *vertical* edges (now implemented, see FR-022), and
  goal cutouts (see Non-goals).
- `RB-PHYSICS-001-FR-022` (curved corner-wall vertical-edge fillets,
  implemented): rounds off the last sharp edges the standard arena's
  octagonal footprint has — the 8 vertical edges where each of the 4
  diagonal corner walls meets its neighboring side or back wall.
  `arena::standard_curves` now builds 24 `StaticQuarterPipe`s total (the 16
  floor/ceiling-seam fillets FR-020/FR-021 already built, plus 8
  vertical-edge fillets, one per corner-wall endpoint). Unlike every prior
  fillet in this port, the two planes a vertical-edge fillet bridges
  *aren't* perpendicular: a corner wall meets its neighboring side/back wall
  at 135 degrees (given `standard_walls`' 45-degree corner cut), not 90.
  This exposed a real gap in `StaticQuarterPipe::between_planes`, which
  previously only worked correctly for perpendicular planes (silently
  computing the wrong axis point otherwise, via a shortcut formula — adding
  the two scaled normals together — that only happens to equal the correct
  answer when the normals are orthogonal). `between_planes` is now fully
  general: it solves the axis point via the actual 2x2 linear system in the
  (possibly non-orthogonal) basis the two normals form, and its own sector
  angle comes out to exactly `arccos(dot(plane_a.normal, plane_b.normal))`
  — a right angle for perpendicular planes as before, or (for this
  requirement's own corner-wall geometry) a shallow 45 degrees, the
  supplement of the 135-degree dihedral angle the two flat walls actually
  meet at. `sphere_vs_quarter_pipe`'s sector-membership test is likewise
  generalized: the old two-dot-products check only worked because a
  90-degree sector's two edges are perpendicular; the new test uses signed
  cross products against `axis_direction` instead (`dir` is in-sector iff
  sweeping from `sector_start` toward it, and from it toward `sector_end`,
  both go the positive way around `axis_direction`), which is exact for any
  sector up to 180 degrees, the widest a sensible fillet-replacing-a-corner
  can ever be. Since a general (non-orthogonal) sector's own containment
  test depends on `axis_direction`'s sign/handedness — unlike the old
  perpendicular-only test, which never used `axis_direction` at all —
  `between_planes` now also self-corrects a "backwards" `axis_direction`
  internally (flipping it if `cross(sector_start, sector_end)` doesn't
  already point the same way), so a caller can pass either of the two
  opposite directions along the shared edge line without needing to reason
  about which one is "correct." New `arena::corner_wall_plane` reuse aside,
  the vertical-edge fillets' own `axis_direction` is simply `(0, 0, 1)` (the
  edge itself is vertical) — no cross product needed here, unlike the
  corner-wall floor/ceiling-seam case FR-021 introduced. `FILLET_RADIUS` is
  reused as-is once again, rather than a separate, smaller radius for these
  visibly shallower edges. Still not modeled: a car actually being
  deflected by any fillet (now implemented, see FR-027), and goal cutouts
  (see Non-goals) — the
  compound corner where a vertical-edge fillet meets a floor- or
  ceiling-seam fillet, near a wall's own endpoint, is addressed next (see
  FR-023).
- `RB-PHYSICS-001-FR-023` (compound-corner fillets, implemented): rounds off
  the 16 remaining sharp vertices in the standard arena's vertical
  boundary — the compound corners where a corner wall's own vertical-edge
  fillet (FR-022) meets a floor- or ceiling-seam fillet (FR-020/FR-021),
  near that corner wall's own top or bottom endpoint. Unlike every prior
  fillet in this port, which each bridge exactly two flat planes with a
  cylindrical (`StaticQuarterPipe`) shape, a compound corner is where
  *three* planes meet (floor or ceiling, the neighboring side or back wall,
  and the corner wall itself) — a cylinder can't blend three planes at
  once, so this requirement introduces a genuinely different static shape,
  `body::StaticCornerFillet`: an immovable sphere, riding the concave
  inside of the sharp vertex exactly the way each `StaticQuarterPipe`
  already rides the concave inside of its own sharp edge. Its constructor,
  `StaticCornerFillet::between_three_planes(plane_a, plane_b, plane_d,
  radius)`, exploits the same "radius-in from every bridged plane"
  invariant `StaticQuarterPipe::between_planes` already established: since
  the fillet's center must sit exactly `radius` in from all three planes,
  it's also exactly `radius` in from each *pair* of them — meaning it
  already lies on all three of that vertex's own pairwise
  `StaticQuarterPipe::between_planes` axis lines simultaneously, so the
  center is nothing more than those three lines' common intersection
  point, solved directly via the classic three-plane-intersection
  cross-product form of Cramer's rule (`center = (cross(n_b, n_d) *
  target_a + cross(n_d, n_a) * target_b + cross(n_a, n_b) * target_d) /
  (n_a . (n_b x n_d))`, where each `target` is that plane's own offset plus
  `radius`) rather than solved from scratch. Containment
  (`sphere_vs_corner_fillet` in `collision.rs`) generalizes a
  `StaticQuarterPipe`'s 2-sided sector test to a "spherical triangle": a
  direction from the center is inside the fillet iff its dot product with
  each of 3 `bounds` vectors is non-negative. Each bound is the raw
  (deliberately non-normalized, since only its sign is used) cross product
  of one pair of the three normals, sign-corrected — via
  `signed_pair_axis`, checking the third (non-pair) plane's own normal
  against it — to always point toward the sharp corner this fillet
  replaces, which is provably correct because that dot product is exactly
  the derivative of the third plane's own signed distance along a
  candidate direction. This sign-correction, like `between_planes`'s own
  `axis_direction` self-correction (FR-022), needs no
  `.normalize()`/`.unwrap()` anywhere in production code. `arena::standard_corner_fillets` builds all 16 (4 per corner wall — floor+side,
  floor+back, ceiling+side, ceiling+back — times the 4 corner walls) by
  calling `between_three_planes` directly on the same three flat planes
  `standard_walls` already builds, reusing `FILLET_RADIUS` once again
  rather than introducing a fourth radius constant (later switched to the
  distinctly larger `CORNER_ARCH_RADIUS`, since FR-025, once a corner
  wall's own floor/ceiling-seam arches did — see FR-025's own entry).
  `PhysicsWorld` gains a
  parallel `corner_fillets: Vec<StaticCornerFillet>` field and
  `with_corner_fillet` builder, resolved for the ball and every car exactly
  like `curves` (a no-op for a car at the time this requirement shipped,
  since `contacts_vs_corner_fillet`
  returned no contact for a box — the same documented deferred case as every
  other fillet here; now implemented, see FR-027); `PhysicsWorld::standard_arena` wires in all 16 via
  `arena::standard_corner_fillets` automatically. Still not modeled: a car
  actually being deflected by any fillet (now implemented, see FR-027), and
  goal cutouts (addressed next, see FR-024).
- `RB-PHYSICS-001-FR-024` (goal cutouts, implemented; its edge fillets
  withdrawn by `RB-PHYSICS-001-FR-085` finding C): opens an actual
  goal-mouth window in each back wall — until now every back wall was a
  single solid, flat `StaticPlane` spanning the full width, with no
  opening at all. A plain `StaticPlane` has no notion of a bounded hole, so
  this requirement introduces a new static shape, `body::StaticGoalWall`:
  a flat plane (`plane`) plus a rectangular window defined in the plane's
  own local 2D coordinate system (`window_center`, unit `u_axis`/`v_axis`,
  `half_width`/`half_height`) — the same "derive an axis/window in the
  plane's own local frame rather than assuming a world axis" discipline
  `StaticQuarterPipe::between_planes`'s `axis_direction` generalization
  (FR-022) established, even though every arena wall this port builds
  today happens to be axis-aligned. `contains_in_window` tests a point by
  its projection onto `u_axis`/`v_axis` alone — the point's own depth from
  the plane along `plane.normal` doesn't matter, since both axes are
  perpendicular to it by construction, so the test is exactly as correct
  approaching the plane as sitting right on it. Containment
  (`collision::sphere_vs_goal_wall`) suppresses contact entirely for a
  sphere (the ball) whose center falls inside the window, letting it pass
  straight through; `contacts_vs_goal_wall`'s box (car) path deliberately
  ignores the window altogether, falling straight through to the ordinary
  `contacts_vs_plane` against the wrapped `plane` — ball-only deflection,
  the same documented deferred case as every fillet here, and a
  zero-regression choice for a car, which now sees literally the same
  contact-generation call it always did against a back wall. `arena::standard_walls` accordingly drops the 2 back-wall `StaticPlane`s it used
  to return (now 7 planes instead of 9); the new `arena::standard_goal_walls` returns them instead as 2 `StaticGoalWall`s, each
  wrapping the same `back_wall_plane` construction as before, windowed at
  `GOAL_HALF_WIDTH`/`GOAL_HEIGHT` (new commonly-cited, uncalibrated-against-
  real-field-mesh-data constants, same sourcing status as `SIDE_WALL_X`)
  centered on the wall at half the goal's own height. `PhysicsWorld` gains
  a parallel `goal_walls: Vec<StaticGoalWall>` field and `with_goal_wall`
  builder, resolved for the ball *and* every car (unlike `curves`/
  `corner_fillets`'s ball-only resolution) — safe precisely because the box
  path is a no-op change from the prior plain-`StaticPlane` behavior.
  New `arena::standard_goal_cutout_fillets` rounds each window's 3 edges
  (two vertical posts, one horizontal crossbar, times 2 goals — 6
  `StaticQuarterPipe`s total, added to the same `curves` list
  `standard_curves`'s 24 already populate), each derived via the existing
  `StaticQuarterPipe::between_planes` from the real back-wall plane and a
  second, purely-geometric plane (`goal_post_plane`/`goal_crossbar_plane`)
  representing the post's or crossbar's own inward-/downward-facing
  surface — positioned at exactly the window's own edge, so the fillet's
  tangent point lands exactly on the window boundary with no gap or
  overlap, the same "sits radius-in from the real wall" property every
  other fillet-to-window/wall pairing in this crate already has. Unlike
  `standard_walls`' corner-wall planes or `standard_curves`' bridged walls,
  `goal_post_plane`/`goal_crossbar_plane` are never themselves added as
  real collision walls — an infinite plane perpendicular to X (or capping
  Z) would incorrectly wall off the *entire* rest of the field at that
  coordinate, unlike a diagonal corner wall's own orientation, which stays
  non-binding everywhere except right at the true corner. At the time this
  requirement shipped, the two compound corners per goal where a post's own
  fillet meets the crossbar's were left as independent, additive fillets,
  not blended into a single smooth vertex — the same "no blended 3D corner"
  approach the arena's other edge fillets used before FR-023 introduced one
  for the corner walls specifically; a dedicated `StaticCornerFillet` was
  later added for these goal corners too, see FR-026. Still not modeled: a
  car actually being deflected by any fillet (now implemented, see FR-027)
  or driving into a goal (now implemented, see FR-028), and a
  modeled goal interior/net beyond the cutout itself (the goal's own two
  compound top corners are now modeled, see FR-026).
- `RB-PHYSICS-001-FR-025` (corner-wall floor/ceiling arch radius,
  implemented; its `750` uu guess measured at `FILLET_RADIUS` by
  `RB-PHYSICS-001-FR-085` finding D): gives a diagonal corner wall's own floor-seam and
  ceiling-seam fillets — 8 of `standard_curves`' 24 entries, the ones
  bridging one of the 4 corner walls to the floor or ceiling — a
  distinctly larger, dedicated radius instead of reusing the cardinal
  walls' own `arena::FILLET_RADIUS`, matching real Rocket League's
  corner-boost area reading as a noticeably bigger, more swept curve than
  a cardinal wall's small rounding, not just a scaled-down copy of the
  same shape. New constant `arena::CORNER_ARCH_RADIUS` (750.0, versus
  `FILLET_RADIUS`'s 292.0) — documented, like every other arena dimension
  in this module, as this project's own uncalibrated placeholder, not
  measured against real field mesh data — governs those 8 arches; a
  compile-time `const _: () = assert!(CORNER_ARCH_RADIUS >
  FILLET_RADIUS);` right after the constant's own definition enforces the
  "distinctly larger" relationship so the two constants can't quietly
  converge or invert under a future edit. Because
  `StaticCornerFillet::between_three_planes` derives its center and
  containment `bounds` from a single shared `radius` argument blended
  across all three planes it bridges — there is no way to ask it for a
  center that sits one radius in from two planes and a different radius in
  from the third, and doing so would also break the "meets its adjoining
  edge fillets exactly where their axes cross" no-gap property FR-023
  established — all 16 of `standard_corner_fillets`'s compound-corner
  fillets switch to `CORNER_ARCH_RADIUS` too, since every one of them
  touches one of the 8 now-larger arches. Unaffected, and still using
  `FILLET_RADIUS` exactly as before: the 8 cardinal-wall (side/back)
  floor/ceiling-seam fillets, the 8 vertical corner-edge fillets
  (`RB-PHYSICS-001-FR-022`, where a corner wall meets its neighboring
  side/back wall), and the 6 goal-cutout edge fillets
  (`RB-PHYSICS-001-FR-024`) — these remain independent, additive contact
  sources next to the bigger arches, not blended with them, the same
  "no blended 3D corner" convention this port has used since before
  FR-023. Still not modeled: a car actually being deflected by any fillet
  (now implemented, see FR-027),
  and everything else `FR-024`'s own Non-goals already cover.
- `RB-PHYSICS-001-FR-026` (goal post-crossbar corner fillets, implemented,
  withdrawn by `RB-PHYSICS-001-FR-085` finding C):
  rounds off the two remaining sharp compound corners per goal — where a
  post's own vertical-edge fillet (`arena::standard_goal_cutout_fillets`,
  FR-024) meets the crossbar's own horizontal-edge fillet — an explicitly
  documented gap FR-024's own doc comment left open ("The two compound
  corners per goal where a post's fillet meets the crossbar's are
  deliberately not blended into a single smooth vertex"). New
  `arena::standard_goal_corner_fillets` returns 4 `StaticCornerFillet`s —
  one per goal post per goal (2 posts x 2 goals) — mirroring FR-023's own
  approach for the arena's 16 compound corners exactly: each is built by
  calling `StaticCornerFillet::between_three_planes` directly on the three
  real flat planes that meet at that vertex (the back wall, that post's own
  plane, and the crossbar), rather than derived from the two
  `StaticQuarterPipe` edge fillets already built at that vertex — since a
  corner fillet's center is already exactly those two edge fillets' own
  common axis intersection, the same property `between_three_planes`'s own
  doc comment (FR-023) already explains. No new shape or collision code was
  needed: `StaticCornerFillet` and `sphere_vs_corner_fillet` (FR-023) are
  already fully generic to any three non-parallel planes. Unlike FR-025 (a
  new, distinctly larger `CORNER_ARCH_RADIUS` was needed there because the
  arena's own cardinal-vs-corner-wall arches use different radii), this
  requirement reuses `FILLET_RADIUS` unchanged for all 4 new fillets, since
  both edge fillets meeting at a goal's post-crossbar corner already share
  `FILLET_RADIUS` — no mismatched-radius concern exists here.
  `PhysicsWorld::standard_arena` wires these 4 in via the same generic
  `with_corner_fillet` builder `standard_corner_fillets`'s 16 already use,
  so `PhysicsWorld.corner_fillets` now holds 20 total, not 16 — the
  pre-existing test `standard_arena_has_sixteen_compound_corner_fillets`
  was renamed `standard_arena_has_twenty_compound_corner_fillets` and its
  assertion updated to match. Explicitly still out of scope: the goal's
  other two corners, where a post meets the floor — the window's own bottom
  edge sits exactly at floor level (`z = 0`), so a post's own fillet there
  simply ends flush with the ground, with no sharp, unrounded vertex left
  to round off, unlike the top post-crossbar corner. Still not modeled at
  the time this requirement shipped: a car actually being deflected by any
  fillet (now implemented, see FR-027) or driving into a goal (now
  implemented, see FR-028), and a
  modeled goal interior/net beyond the cutout itself (see Non-goals).
- `RB-PHYSICS-001-FR-027` (car deflection by curved fillets, implemented):
  a car (box) drove straight through every curved fillet in this port —
  `StaticQuarterPipe` and `StaticCornerFillet` alike — until now, since
  `collision::contacts_vs_quarter_pipe`/`contacts_vs_corner_fillet` always
  returned an empty manifold for a `Shape::Box`, a documented deferred case
  repeated across FR-020 through FR-026's own Non-goals. This requirement
  closes that gap by reducing a box-vs-curve test to the same "test every
  corner" technique `box_vs_plane` already uses for a flat plane (already
  in this codebase, generating up to 4 contacts for a box resting on a
  plane): new `collision::box_vs_quarter_pipe(position, orientation,
  half_extents, pipe)` checks each of the box's 8 corners as a
  zero-radius sphere via the existing `sphere_vs_quarter_pipe(world_corner,
  0.0, pipe)` call, collecting every corner that reports a contact into the
  manifold; new `collision::box_vs_corner_fillet` does the identical thing
  against `sphere_vs_corner_fillet` for the sphere-shaped compound-corner
  fillets. `contacts_vs_quarter_pipe`/`contacts_vs_corner_fillet` now
  dispatch a `Shape::Box` to these new functions instead of returning
  `Vec::new()`. Each surviving contact's `point` field is overwritten to
  the corner's own world position rather than the fillet-surface point
  `sphere_vs_quarter_pipe`/`sphere_vs_corner_fillet` itself computes — the
  same rel_pos/torque-accuracy reason `box_vs_plane`'s own doc comment
  already documents: a tilted box's corner isn't generally "below" the body
  center along the surface normal, so the solver needs the true
  contact-to-center offset to compute correct torque, not a point that
  merely lies on the curved surface along the contact normal. This was
  originally documented, in both new functions' own doc comments, as an
  approximation, not a full convex-vs-curved-surface narrow phase — the
  concern being that a face of the box resting flush against a shallow
  curve (a radius large relative to the box) could have every one of its
  own corners still just clear of the fillet while the face's middle
  already overlaps it, under-detecting that case.
  `RB-PHYSICS-001-FR-032` subsequently investigated this concern directly
  (building a genuine GJK-based replacement specifically to fix it) and
  found it doesn't actually apply here: a quarter-pipe/corner-fillet's
  contact test is a *containment* question (is the box's farthest point
  from the axis/center at or beyond radius), and distance-from-a-line/point
  is a convex function whose maximum over a convex polytope (the box) is
  always attained at a corner — so per-corner testing is mathematically
  exact for this question, not an approximation. See
  `RB-PHYSICS-001-FR-032`'s own entry and `box_vs_quarter_pipe`'s current
  doc comment for the full argument and its empirical confirmation. No new
  shape or fundamentally new collision primitive was needed — this is a
  generalization of existing dispatch, not new physics/math machinery.
  `PhysicsWorld::step` already called `resolve_curve_contact`/
  `resolve_corner_fillet_contact` for every car in the scene — that call
  was wired in from FR-023 onward, just always a no-op for a car until now,
  since `contacts_vs_quarter_pipe`/`contacts_vs_corner_fillet` always
  returned empty for a box — so `world.rs`'s step loop itself needed no
  changes, only doc-comment updates reflecting the new real behavior (the
  "no-op for a box" language in `resolve_curve_contact`/
  `resolve_corner_fillet_contact`'s own doc comments, the `curves`/
  `corner_fillets` field doc comments, the `with_curve`/`with_corner_fillet`
  builder doc comments, and `arena.rs`'s own module-level "Still not
  modeled" list). Explicitly unaffected by this requirement:
  `StaticGoalWall`/`contacts_vs_goal_wall` — a car still sees the exact same
  solid, full-width back wall it always has, deliberately ignoring the
  goal-mouth window, since a goal wall isn't a curved fillet at all and this
  generalization only touches `StaticQuarterPipe`/`StaticCornerFillet`
  dispatch; a car actually driving into a goal is now implemented, see
  `RB-PHYSICS-001-FR-028`.
- `RB-PHYSICS-001-FR-028` (car actually driving into a goal, implemented):
  a car (box) drove straight into the full, solid back wall even at the
  goal mouth until now — `collision::contacts_vs_goal_wall` dispatched a
  `Shape::Box` straight through to plain `contacts_vs_plane` against the
  wrapped `StaticPlane`, completely ignoring the goal-mouth window
  `StaticGoalWall` carries, even though a sphere (the ball) already
  passed through the window via `sphere_vs_goal_wall` — a documented
  Non-goal repeated across FR-024 through FR-027's own "Still not
  modeled" lists. This requirement closes that gap with a new
  `collision::box_vs_goal_wall(position, orientation, half_extents, wall)
  -> Vec<Contact>`: it iterates the box's 8 corners exactly like
  `box_vs_plane` does, but for each corner first checks
  `wall.contains_in_window(&world_corner)` — if the corner's own
  projection onto the plane's `u_axis`/`v_axis` falls inside the window,
  that corner contributes no contact at all (skipped via `continue`),
  regardless of how deep it might be penetrating along the plane's
  normal (this exactly mirrors `sphere_vs_goal_wall`'s existing
  convention of ignoring distance-along-normal for the window test —
  `contains_in_window` was already written that way, no changes needed
  to `body::StaticGoalWall` itself). A corner outside the window falls
  through to an ordinary `box_vs_plane`-style corner test (signed
  distance against the wrapped plane, contact if within
  `CONTACT_PROCESSING_THRESHOLD`). This is the exact same "test every
  corner" approximation technique `RB-PHYSICS-001-FR-027` established
  for curved fillets (`box_vs_quarter_pipe`/`box_vs_corner_fillet`),
  applied here to a flat windowed plane instead of a curved one. One
  real behavioral consequence worth documenting explicitly: because
  each corner is tested independently, a car that's only partially
  lined up with the window (e.g. straddling one of its edges) gets a
  *partial* block — the corners still outside the window register
  contacts and stop the car there, while the corners inside the window
  register none — rather than an all-or-nothing result the way a
  single-point sphere test necessarily produces; this is a real
  emergent behavior of the technique, not a separate feature.
  `contacts_vs_goal_wall`'s dispatch changed from `Shape::Box { .. } =>
  contacts_vs_plane(body, &wall.plane)` to `Shape::Box { half_extents } =>
  box_vs_goal_wall(body.position, body.orientation, half_extents, wall)`.
  No `world.rs` step-loop changes were needed — exactly like FR-027's own
  discovery: `PhysicsWorld::step`'s `resolve_goal_wall_contact` was
  *already* being called for every car in the scene (it always had been,
  since a car needed the wall's plain-plane collision even before this
  fix), so this is a pure dispatch-function change; only doc-comment
  updates were needed — in `world.rs` (`goal_walls`' field doc comment,
  `with_goal_wall`'s doc comment, and `resolve_goal_wall_contact`'s doc
  comment all now say a car passes through too, instead of describing it
  as unwindowed or falling straight through to an ordinary plane
  contact), `body.rs`'s `StaticGoalWall` doc comment, `arena.rs`'s
  module-level and `standard_goal_walls`'s own doc comments, and
  `lib.rs`'s crate-level module doc comment. Explicitly still not
  implemented by this requirement: a modeled goal interior/net — the
  goal still opens onto open, unbounded space beyond the back wall for
  both the ball and now a car (see Non-goals); the goal-cutout edge
  fillets and goal-corner fillets (FR-024/FR-026) already deflect a car
  via FR-027's own generalization, unaffected by this specific change —
  this requirement is purely about the flat windowed wall itself, not
  the fillets around its rim.
- `RB-PHYSICS-001-FR-029` (modeled goal interior, implemented): until now
  the goal-mouth window (`StaticGoalWall`, since FR-024) opened onto
  completely open, unbounded space — a ball or car passing through (the
  ball since FR-024, a car since FR-028) sailed forever, with nothing
  behind the window to stop it — a documented Non-goal repeated across
  FR-024 through FR-028's own "Still not modeled" lists. This requirement
  closes that gap by modeling a bounded interior volume behind each
  goal-mouth window: explicitly a solid bounding box, **not** a
  springy/catching net mesh — no cloth/soft-body simulation was added.
  This is a deliberate, honest scoping decision, not an oversight: a
  genuine net mesh was a real, separate, not-yet-implemented Non-goal at
  the time (see Non-goals) — since resolved for the ball specifically by
  `RB-PHYSICS-001-FR-033`'s `net::NetMesh`, sitting in front of this
  requirement's own solid back-of-net plane (unaffected, still a car's real
  backstop). Three pieces, all in `crates/rb_physics_bullet`: (1) a
  new constant `arena::GOAL_DEPTH: f32 = 880.0` — an uncalibrated
  placeholder, since this port has no verified reference for Rocket
  League's real net depth at all (unlike `GOAL_HALF_WIDTH`/`GOAL_HEIGHT`),
  chosen only to be a visibly real interior volume comparable in scale to
  the goal mouth's own dimensions; (2) a new shape type,
  `body::StaticBoundedWall` — a flat `StaticPlane` that only collides
  *within* a rectangular bound in the plane's own local 2D frame
  (`bound_center`/`u_axis`/`v_axis`/`half_u`/`half_v`, plus a
  `contains_in_bound` method) — the **opposite** gate convention from
  `StaticGoalWall`'s window (which collides everywhere *except* inside a
  rectangle). This is needed because the goal's own side walls and roof
  can't be plain unbounded `StaticPlane`s: an infinite plane at, say,
  `x = GOAL_HALF_WIDTH` would incorrectly wall off the *entire* main field
  at that x coordinate — the exact same problem `arena::goal_post_plane`'s
  own pre-existing doc comment already documented for a different
  purely-geometric plane used only to derive fillets. New dispatch
  functions in `collision.rs`: `sphere_vs_bounded_wall`/
  `box_vs_bounded_wall`/`contacts_vs_bounded_wall` — `box_vs_bounded_wall`
  uses the same "test every corner" technique established by
  FR-027/FR-028, but a corner *outside* the bound is skipped (the opposite
  of `box_vs_goal_wall`'s per-corner window test, where a corner *inside*
  the window is skipped); and (3) new arena geometry functions in
  `arena.rs`: `goal_back_wall_plane(sign)`/`standard_goal_back_walls()` —
  2 plain, unbounded `StaticPlane`s (one per goal), positioned
  `GOAL_DEPTH` behind the real back wall, added to `PhysicsWorld.walls` via
  the existing `with_wall` builder (no new field needed) — deliberately
  plain planes, not `StaticBoundedWall`s, since nothing can ever reach this
  plane except by first passing through the goal-mouth window (the real
  back wall is solid everywhere else), so an unbounded plane here is
  exact, not an approximation; `goal_side_wall(back_sign,
  post_sign)`/`standard_goal_side_walls()` — 4 `StaticBoundedWall`s total
  (2 per goal), each reusing `goal_post_plane(post_sign)` completely
  unchanged as its own flat plane (the post's own inward-facing surface
  already sits exactly where the goal box's own side wall needs to be),
  bounded to the goal's own depth (`y` from the real back wall out to
  `GOAL_DEPTH` behind it) and height (`z` from the floor up to
  `GOAL_HEIGHT`) range; and `goal_roof(sign)`/`standard_goal_roofs()` — 2
  `StaticBoundedWall`s total (1 per goal), each reusing
  `goal_crossbar_plane()` completely unchanged, bounded to the goal's own
  width (`x` within `GOAL_HALF_WIDTH` either way) and depth range.
  `PhysicsWorld` (in `world.rs`) gains a new field
  `bounded_walls: Vec<StaticBoundedWall>` and a `with_bounded_wall` builder
  (mirroring `with_goal_wall`), plus a new `resolve_bounded_wall_contact`
  (mirroring `resolve_goal_wall_contact`), resolved for the ball and every
  car in `PhysicsWorld::step`, exactly like `goal_walls`.
  `PhysicsWorld::standard_arena` wires in `standard_goal_back_walls()` via
  `with_wall`, and `standard_goal_side_walls()`/`standard_goal_roofs()` via
  `with_bounded_wall`. No changes to the actual step-loop resolution
  pattern were needed beyond adding the new loop — unlike FR-027/FR-028,
  this isn't a "silent no-op that later got activated" story; it's
  straightforwardly new geometry wired in from the start.
  `PhysicsWorld.walls` grew from 7 to 9 real entries once
  `standard_arena` is built (the 2 new back-of-net planes), so the
  pre-existing `world.rs` test
  `standard_arena_has_seven_walls_and_the_standard_ground`
  was renamed `standard_arena_has_nine_walls_and_the_standard_ground` and
  its assertion updated — a test-count correction, not new capability.
  New tests (242 total, net +21 over FR-028's 221): 4 new in `body.rs` for
  `StaticBoundedWall::contains_in_bound`
  (`contains_in_bound_is_true_for_the_bounds_own_center`,
  `contains_in_bound_is_true_just_inside_each_edge`,
  `contains_in_bound_is_false_just_outside_each_edge`,
  `contains_in_bound_ignores_distance_from_the_plane_itself`) — mirroring
  the pre-existing `StaticGoalWall::contains_in_window` tests exactly, just
  with the boolean gate meaning inverted; 5 new in `collision.rs`
  (`sphere_inside_the_bound_behaves_like_an_ordinary_plane`,
  `sphere_outside_the_bound_has_no_contact`,
  `box_squarely_inside_the_bound_behaves_like_an_ordinary_plane`,
  `box_straddling_the_bounds_edge_only_collides_on_the_corners_still_inside_it`,
  `box_entirely_outside_the_bound_has_no_contact`) against a synthetic
  fixture, mirroring the `StaticGoalWall` collision tests with the gate
  inverted; 8 new in `arena.rs` proving the geometry functions place
  things correctly (`standard_goal_back_walls_has_two_walls`,
  `every_goal_back_wall_sits_goal_depth_behind_the_real_back_wall`,
  `standard_goal_side_walls_has_four_walls`,
  `every_goal_side_walls_plane_matches_some_goal_post_plane`,
  `every_goal_side_walls_bound_covers_the_real_goal_depth_and_height`,
  `standard_goal_roofs_has_two_roofs`,
  `every_goal_roofs_plane_is_the_goal_crossbar_plane`,
  `every_goal_roofs_bound_covers_the_real_goal_width`) — the same "prove
  the real geometry, not an arbitrary point" discipline this crate's other
  arena tests already use; and 4 new in `world.rs` (plus the renamed
  wall-count test above) — 1 wiring-count test
  (`standard_arena_has_six_bounded_walls`) and 3 new live end-to-end
  `PhysicsWorld` proofs
  (`a_ball_shot_into_the_goal_is_stopped_by_the_goal_back_wall`,
  `a_ball_shot_sideways_inside_the_goal_is_stopped_by_a_goal_side_wall`,
  `a_ball_shot_upward_inside_the_goal_is_stopped_by_the_goal_roof`).
  These 3 end-to-end tests are **deliberately** isolated to a minimal
  `PhysicsWorld` built from just the specific new wall(s) under test (via
  `PhysicsWorld::new` plus `with_wall`/`with_bounded_wall`, **not**
  `PhysicsWorld::standard_arena`), rather than the full standard arena
  every other end-to-end goal test in this file uses — see Verification
  plan for the two real test-design findings (a sector-membership
  isolation issue, and a wall-restitution-zeroing fix) this discovery led
  to. Explicitly still not implemented by this requirement: a genuine net
  *mesh* — no cloth/soft-body simulation, no visual net sag, no "ball
  tangles in netting" behavior; this is a solid bounding volume standing
  in for the net's functional role (stopping the ball/car), nothing more
  (see Non-goals).
- `RB-PHYSICS-001-FR-030` (combined multi-body solve, implemented): until
  now, `PhysicsWorld::step` resolved every ball-vs-car and car-vs-car
  contact manifold with its own independent call to
  `solver::resolve_contacts_between` — each call ran its own full
  `SOLVER_ITERATIONS` (10) Gauss-Seidel pass and applied the resulting
  velocity change to both bodies before the next pair's setup even read a
  body's velocity, so a body touching two others in the same step (e.g. a
  car pinned between the ball and another car) never had both contacts
  reasoned about together — the second-resolved pair's setup used the
  first pair's already-finished result as if it were the honest starting
  velocity, and could end up discarding almost all of the first contact's
  effect (see Verification plan for the measured magnitude). This
  requirement closes that gap with a new `solver::resolve_dynamic_manifolds(bodies: &mut [RigidBody], manifolds: &[(usize, usize,
  Vec<Contact>)], dt: f32)`: it sets up rows for every manifold once (the
  same per-contact `setup_two_body_rows` as before), gives every body
  *index* that appears in at least one manifold its own `DeltaVelocity`
  accumulator, and runs `SOLVER_ITERATIONS` iterations where *every*
  iteration processes *every* manifold once, each manifold's rows reading
  and updating the shared accumulators for its own two body indices —
  only after all iterations finish does it apply each body's own
  accumulated delta to its actual velocity, once. This is the real fix: a
  shared, interleaved iteration budget across every dynamic-vs-dynamic
  manifold touching in the scene that step, not a sequence of
  fully-independent pairwise solves. A new helper,
  `solver::delta_pair_mut(deltas: &mut [DeltaVelocity], a: usize, b:
  usize) -> (&mut DeltaVelocity, &mut DeltaVelocity)`, is a
  `split_at_mut`-based disjoint-borrow helper for arbitrary index pairs,
  generalizing the `Vec::split_at_mut` trick `PhysicsWorld::step`'s
  car-vs-car loop already used for the special case `b == a + 1`, so
  multiple manifolds sharing a body index can also share that body's
  single accumulator. The old `TwoBodyDelta` struct is removed:
  `resolve_two_body_row` now takes two separate `&mut DeltaVelocity`
  parameters (`delta_a`, `delta_b`) instead of one combined struct, which
  is what makes sharing an accumulator across manifolds possible.
  `resolve_contacts` (one dynamic body vs. static geometry) and
  `resolve_contacts_between` (still present, used directly by
  two-body-only callers and tests) are unchanged in their public
  behavior — `resolve_contacts_between`'s internals were adjusted to use
  two separate `DeltaVelocity`s instead of `TwoBodyDelta`, a pure refactor
  matching the new shared-solver internals, not a behavior change. In
  `world.rs`, `PhysicsWorld::step` no longer calls the two independent
  loops (`for car in &mut self.cars { Self::resolve_dynamic_contact(&mut
  self.ball, car, dt); }` then a nested car-vs-car loop, both through a
  now-deleted private `resolve_dynamic_contact` helper); instead it builds
  a `Vec<RigidBody>` (index 0 the ball, index `i+1` `self.cars[i]`),
  collects every non-empty ball-vs-car and car-vs-car
  `collision::contacts_between` result into a `Vec<(usize, usize,
  Vec<Contact>)>`, calls `solver::resolve_dynamic_manifolds` on the whole
  set once, then copies the resolved velocities back into `self.ball` and
  `self.cars`.
  - **Non-goals (this requirement).** Scoped strictly to dynamic-vs-dynamic
    contacts. Static contacts — ground, arena walls, curves, corner
    fillets, goal walls, and bounded walls — are deliberately *not* part
    of this combined solve: each body's contact with static geometry only
    depends on that one body, so resolving it independently loses no
    cross-body information, and `resolve_contacts` is untouched. Split
    impulse, warm-starting/sleeping, and the average-not-max
    restitution/friction combine mode remain exactly as documented before
    this requirement (see this spec's own Non-goals and Open questions) —
    this requirement shares the iteration *budget* across manifolds, it
    doesn't change what each contact row itself solves for.
  - **Acceptance criteria.**
    - `solver::resolve_dynamic_manifolds` runs `SOLVER_ITERATIONS`
      iterations, each iteration visiting every manifold in the input
      slice exactly once, before any body's velocity is updated from its
      accumulated delta.
    - Two (or more) manifolds that share a body index read and write that
      body's *same* `DeltaVelocity` accumulator across iterations — never
      a separate, independent one per manifold.
    - `resolve_contacts` and `resolve_contacts_between`'s existing
      public behavior (a single manifold solved against a static body, or
      between exactly two dynamic bodies in isolation) is unchanged;
      every pre-existing test exercising either function still passes
      with no assertion changes.
    - `PhysicsWorld::step` produces exactly one `solver::resolve_dynamic_manifolds` call per step covering every ball-vs-car and car-vs-car
      manifold that step, not one call per pair.
  - **Verification plan.** A dedicated solver-level test,
    `solver::tests::resolve_dynamic_manifolds_keeps_more_of_every_bodys_contact_than_resolving_pairs_independently`, and a world-level
    end-to-end test,
    `world::tests::a_ball_pinched_between_two_closing_cars_is_resolved_by_a_shared_multi_body_solve`, both build a left-right symmetric
    "pinch": a ball (mass 1) exactly touching two identical cars (mass
    180 each) closing in from opposite sides at equal (100 units/s)
    speed, restitution zero throughout. The true simultaneous-solve
    answer for this exact setup is all three bodies ending near zero
    velocity (total momentum is exactly zero, and every body is mutually
    constrained to the others). Measured results: resolving each pair
    independently (the pre-FR-030 approach, reproduced directly by
    calling `resolve_contacts_between` twice in sequence) leaves the ball
    at around 98-99% of a single car's own closing speed (~98.9 units/s)
    — as if the first-resolved contact's effect was almost entirely
    discarded by the second; the new combined `resolve_dynamic_manifolds`,
    at this crate's existing `SOLVER_ITERATIONS = 10`, leaves the ball
    noticeably slower (~89.5 units/s in the isolated solver-level
    measurement) — a real, measurable improvement, but *not* full
    convergence to the true zero-velocity answer. This residual error at
    only 10 iterations is a known, common limitation of projected
    Gauss-Seidel iterative contact solvers for an extreme mass-ratio
    "sandwiched" body configuration (a light body pinned between two much
    heavier ones), not unique to this port. It was confirmed during test
    design (not shipped as a change) that increasing the iteration count
    substantially (verified manually with 300 iterations) converges the
    combined solve's result much closer to the true zero-velocity answer,
    while the old independent-pairwise approach's result does *not*
    change at all no matter how many iterations each individual pairwise
    call gets — proving the old approach's error is structural
    (information genuinely thrown away), not an iteration-count
    shortfall. Both tests assert the *direction and magnitude* of the
    improvement (the combined result measurably slower/closer-to-centered
    than the independent-pairwise result), not exact convergence to zero
    — a deliberate, honest test-design choice, matching this project's
    convention (see FR-025's, FR-027's, and FR-029's own entries above) of
    not overclaiming what a test actually proves. New tests: 1 in
    `solver.rs`, 1 in `world.rs` (net +2 tests over FR-029's 242, bringing
    the crate to 244).
- `RB-PHYSICS-001-FR-031` (constant-calibration audit, implemented —
  explicitly does NOT close `FR-005`): `FR-005`'s real calibration against
  recorded ground truth stays blocked on `PHASE-0-EXIT` (no real BakkesMod
  capture exists yet), so this requirement is narrower: source every
  uncalibrated placeholder constant in `drive.rs`/`arena.rs`/`world.rs`
  against the best available community reference, correct the ones with a
  solid one, and explicitly flag the rest as still uncalibrated rather
  than silently leaving their status ambiguous. Sources cross-checked
  directly from source code (not blog paraphrases, which is where this
  project's own earlier placeholder numbers came from): the **RocketSim**
  project (`ZealanL/RocketSim`, `src/RLConst.h`/`Car.cpp`) and
  **RLUtilities** (`samuelpmish/RLUtilities`,
  `src/simulation/car.cc`/`ball.cc`, `src/mechanics/jump.cc`/`dodge.cc`) —
  two independently-written reverse-engineering projects — plus the
  **RLBot community wiki**'s "Useful Game Values" page. Agreement across
  all three, independently, is treated as high-confidence; a single source
  or a casual/older reference is treated as lower-confidence and flagged
  as such below, not silently trusted.
  - **Corrected, with code changes (all expressed in this port's own
    linear-velocity/acceleration/time/distance units, which — unlike a
    torque or an angular rate — don't depend on matching real Rocket
    League's specific car mass/inertia tensor to port correctly):**
    - `drive::JUMP_SPEED`: `292.0` → `875.0/3.0` (≈291.667 uu/s) —
      RocketSim's `JUMP_IMMEDIATE_FORCE = 875.f/3.f` and RLUtilities'
      `Jump::speed = 291.667f` agree exactly; both projects also confirm
      the double jump reuses this same value unmodified, matching what
      this port already did.
    - `drive::JUMP_HOLD_ACCELERATION`: `1400.0` → `4375.0/3.0` (≈1458.33
      uu/s²) — RocketSim's `JUMP_ACCEL = 4375.f/3.f` and RLUtilities'
      `Jump::acceleration = 1458.3333f` agree exactly. Real Rocket League
      also scales this down (0.62x) during the jump's first 0.025s
      (RocketSim's `JUMP_MIN_TIME`) rather than applying it at full
      strength immediately; that two-phase ramp is a further refinement
      this port doesn't model, tracked as a documented simplification, not
      silently dropped.
    - `drive::JUMP_HOLD_MAX_DURATION` (`0.2`): audited, not changed — this
      port's pre-existing value already matches RocketSim's
      `JUMP_MAX_TIME = 0.2f` and RLUtilities' `Jump::max_duration = 0.2f`
      exactly. Recorded here as *confirmed*, not just *unchanged*, so a
      future reader doesn't have to re-derive that this one was already
      right.
    - New `drive::UNBOOSTED_MAX_CAR_SPEED = 1410.0`: a genuine behavioral
      fix, not just a doc update. Before this audit, `drive::MAX_CAR_SPEED`
      (2300, Rocket League's *boosted* top speed — confirmed correct by
      all three sources) doubled as throttle's own speed cap too, letting
      a car reach boosted top speed on throttle alone. RocketSim's
      `DRIVE_SPEED_TORQUE_FACTOR_CURVE` drives available drive torque to
      exactly zero at 1410 uu/s, and the RLBot wiki independently states
      the same 1410 uu/s unboosted cap — so throttle now caps at this new,
      separate constant instead, while `MAX_CAR_SPEED` keeps its
      already-correct role as boost's own cap (and as the turning-torque
      scale-up reference, an arbitrary normalization unaffected by this
      change).
    - `drive::BOOST_ACCELERATION` (`991.667`), `drive::MAX_BOOST` (`100`),
      `world::PhysicsWorld::new`'s gravity (`-650`), and
      `arena::GOAL_DEPTH` (`880`): all audited and confirmed correct
      against the same three sources — no change needed, recorded here as
      confirmed rather than merely unchanged.
  - **Explicitly flagged, NOT changed (real reference numbers exist but
    don't safely port, or no reference exists at all — recorded as
    audited-and-still-uncalibrated, not silently left ambiguous):**
    - `drive::DODGE_SPEED` (`1400.0`): real Rocket League's dodge impulse
      is not a single flat magnitude — RocketSim/RLUtilities agree on a
      base `500.0` uu/s scaled by direction (forward ×1.0, side ×1.9,
      backward ×2.5-ish) and by current forward speed (reduced as speed
      approaches `MAX_CAR_SPEED`). Adopting just the bare `500` would
      collide with `WALL_JUMP_HORIZONTAL_SPEED` (`550`, currently used to
      keep a wall jump distinguishable from a dodge in this port's own
      tests) and would still misrepresent the real direction/speed-scaled
      shape as a flat number — a real fix here is a mechanic redesign
      (its own future FR), not a constant substitution, so the placeholder
      stays as-is.
    - `drive::DODGE_ANGULAR_SPEED` (`5.5` rad/s): real Rocket League
      implements a flip's spin as an applied *torque* (RocketSim's
      `FLIP_TORQUE_X=260`/`FLIP_TORQUE_Y=224` for 0.65s, min 0.41s;
      RLUtilities matches exactly), not a flat angular-velocity kick —
      the resulting spin rate depends on the real game's own specific
      hitbox inertia tensor, which this port's placeholder car body
      doesn't match, so there is no single rad/s number to safely port.
      `RB-PHYSICS-001-FR-069` later confirmed the exact mechanism behind
      those constants: `_UpdateDoubleJumpOrFlip` records a per-axis
      `flipRelTorque` once, at flip start, and `_UpdateAirTorque` — a
      separate, later step — then applies `flipRelTorque * Vec(FLIP_TORQUE_X,
      FLIP_TORQUE_Y, 0)` continuously, every physics tick, for as long as
      `isFlipping = hasFlipped && flipTime < FLIP_TORQUE_TIME` holds; there
      is no decay or ramp before that hard `0.65`s cutoff. Reproducing this
      shape (rather than just its magnitude) would additionally require
      per-car elapsed-flip-time state threaded through `PhysicsWorld`, which
      `RB-PHYSICS-001-FR-059`'s own Non-goals already flagged as out of
      scope — so this remains a mechanism/state-shape mismatch, not a
      constant substitution.
    - `drive::WALL_JUMP_HORIZONTAL_SPEED` (`550.0`): real Rocket League has
      no separate wall-jump speed constant at all — a wall jump just fires
      the ordinary jump impulse (`JUMP_SPEED`) along the wall's contact
      normal instead of straight up (confirmed in both RocketSim and
      RLUtilities). This port's separate, faster constant is a deliberate
      structural simplification (a distinguishable wall-jump feel for
      tests), not an unconfirmed guess anymore, but still not what real
      Rocket League does.
    - `drive::STEER_TORQUE`, `drive::AIR_CONTROL_TORQUE`,
      `drive::HANDBRAKE_FRICTION_MULTIPLIER`,
      `drive::LANDING_AUTO_UPRIGHT_TORQUE`: real reference numbers exist
      for some of these (air-control torque/damping coefficients — pitch
      130, yaw 95, roll 400, capped at an overall 5.5 rad/s, confirmed
      identically in RocketSim and RLUtilities; handbrake as a slip-ratio
      friction *curve*, not a flat multiplier), but all of them are
      calibrated against real Rocket League's own specific car mass and
      inertia tensor, which this port's placeholder `RigidBody::car_box`
      isn't independently confirmed to match — porting the raw numbers
      into a different inertia scale would produce false-precision
      angular accelerations, not a real fix. Remain uncalibrated
      placeholders, now with their real-world analog documented instead of
      simply absent.
    - `arena::FILLET_RADIUS`/`arena::CORNER_ARCH_RADIUS`: audited with no
      analytic single-number reference found anywhere in the serious
      community sources — Rocket League's actual corner geometry is a
      triangulated collision mesh (see `ZealanL/RLArenaCollisionDumper`,
      a tool that extracts the raw mesh rather than a formula), not a
      mathematical arc, and even the RLBot wiki explicitly notes "the
      curvature at the intersections is ignored" in its own published
      wall-length figures. Remain uncalibrated placeholders; closing this
      one for real would mean ingesting an actual dumped mesh, not sourcing
      a better number.
  - **Open ambiguity surfaced by the audit, deliberately not acted on
    without stronger confidence (recorded as a genuinely open question,
    not resolved either way):**
    - Ball radius: this port's `92.75` is an older, casually-cited figure;
      RocketSim's `BALL_COLLISION_RADIUS_SOCCAR`, RLUtilities'
      `Ball::soccar_radius`, and the current `wiki.rlbot.org/v4` rewrite
      all instead converge on `91.25` as the actual simulation collision
      radius (with the ball's *rest height* separately at `93.15`, larger
      because of a mesh collision margin — a different number for a
      different purpose, easy to conflate with the collision radius
      itself). Not changed here: `92.75` is load-bearing across a large
      fraction of this crate's existing tests as a magic number, and
      medium-confidence evidence isn't enough justification to ripple a
      change through all of them in an audit whose own scope is
      "don't misrepresent this as full calibration." Tracked as an open
      question for a future, deliberate change.
    - `arena::CEILING_Z` (`2044.0`) vs. RocketSim's `ARENA_HEIGHT = 2048.f`:
      a small (4 uu) discrepancy the audit surfaced but couldn't resolve
      with confidence — it's unclear whether the two numbers describe the
      same reference point (e.g. usable ceiling height vs. the raw mesh's
      own extent) without deeper investigation this audit's scope doesn't
      cover. Not changed; tracked as an open question.
  - **Acceptance criteria.** Every named constant in `drive.rs`/`arena.rs`
    this audit examined has an explicit, current disposition (corrected,
    confirmed, or flagged-uncalibrated-with-reason) in this entry or in its
    own doc comment — none left in the prior "not derived from any measured
    or documented Rocket League value" state without this audit having
    actually looked for one. No behavior change is claimed beyond the two
    corrected constants and the new `UNBOOSTED_MAX_CAR_SPEED` split; every
    other constant's runtime value is provably unchanged (see the diff).
  - **Verification plan.** `drive.rs` gains
    `throttle_stops_accelerating_at_unboosted_max_speed` (renamed from
    `throttle_stops_accelerating_at_max_speed`, now asserting against
    `UNBOOSTED_MAX_CAR_SPEED` instead of `MAX_CAR_SPEED`) and a new
    `throttle_alone_cannot_reach_the_boosted_top_speed`, holding throttle
    for 20 simulated seconds and confirming the car plateaus near
    `UNBOOSTED_MAX_CAR_SPEED`, well short of the boosted `MAX_CAR_SPEED` —
    the real bug this audit fixed, made concrete as a regression test
    rather than left as a doc-comment-only claim. 1 net new test (245
    total in `rb_physics_bullet`, +1 over FR-030's 244); the `JUMP_SPEED`/
    `JUMP_HOLD_ACCELERATION` precision refinements needed no new tests —
    every existing assertion involving either already used a tolerance
    (`.abs() < 1.0` or looser) comfortably wider than the ~0.33 and ~58.33
    uu/s²-scale differences from their prior placeholder values.
- `RB-PHYSICS-001-FR-032` (genuine convex-vs-curved-surface narrow phase
  investigation, resolved — no code change to the narrow phase itself):
  this requirement set out to replace `box_vs_quarter_pipe`/
  `box_vs_corner_fillet`'s per-corner technique with a real GJK/EPA
  convex-vs-convex narrow phase, on the strength of a limitation FR-027's
  own doc comments claimed: "a face resting flush against a shallow curve
  (a radius large relative to the box) can have every one of its own
  corners still just clear of the fillet while the face's middle already
  overlaps it, under-detecting that case." Building that replacement (a
  from-scratch GJK closest-points implementation, `gjk::closest_points`,
  treating a quarter-pipe's axis as a long finite segment and a corner
  fillet's center as a single point) surfaced a wrong assumption before it
  ever shipped: swapping it in broke two pre-existing, previously-passing
  end-to-end tests
  (`world::tests::a_car_resting_within_a_curved_transitions_footprint_is_pushed_up_off_the_flat_floor_height`,
  `world::tests::a_car_embedded_in_a_compound_corner_fillets_footprint_has_its_penetration_reduced`),
  because it answered a *different* question than the one this contact
  actually needs. A quarter-pipe/corner-fillet's contact test is a
  *containment* question — "is the box's farthest point from
  `axis_point`/`axis_direction` (or a corner fillet's `center`) at or
  beyond `radius`" (see `sphere_vs_quarter_pipe`'s own doc comment: the
  playable side is the *inside* of the curve, so a body is only in
  contact once some part of it reaches or exceeds the boundary) — not a
  *closest*-point question. `gjk::closest_points` correctly finds the
  box's single *nearest* point to the axis/center, which is the *opposite*
  of what determines contact here, and can be a genuinely different point
  of the box than the farthest one. Distance-from-a-line (or
  distance-from-a-point) is a *convex* function of position, and the
  *maximum* of a convex function over a convex polytope (the box) is
  always attained at one of its extreme points — its 8 corners — never a
  face's interior; this is standard convex-analysis fact, not specific to
  this shape. So the per-corner technique isn't approximating the
  farthest-point question at all: it's computing the exact same answer a
  full box-vs-cylinder narrow phase would, just via enumeration instead of
  an iterative solver. The claimed under-detection gap doesn't exist for
  this contact's actual question.
  - **Non-goals (this requirement).** Does not address manifold
    *richness* (when 2+ corners simultaneously violate the radius, each
    is still resolved as its own independent contact point rather than a
    single unified manifold a full convex-vs-convex narrow phase might
    produce) — a genuinely different question from detection accuracy,
    and out of this requirement's scope, since nothing in this
    investigation found evidence it causes a real problem either.
  - **Acceptance criteria.** The claimed under-detection gap is either
    fixed with a genuine narrow phase, or rigorously shown not to exist;
    either way, `box_vs_quarter_pipe`/`box_vs_corner_fillet`'s own doc
    comments (and this crate's other doc comments referencing the same
    claim — `lib.rs`'s crate doc, this spec's own Open questions) reflect
    the actual, verified state, not an unverified inherited claim.
  - **Verification plan.** Proven two ways: (1) analytically — the
    convex-maximum argument above, which is a general mathematical fact,
    not something that needs simulating to be true; (2) empirically, via
    a new `collision.rs` test,
    `no_point_on_a_boxs_face_is_ever_farther_from_a_quarter_pipes_axis_than_its_own_corners`,
    which densely samples (50×50 grid per face) every point across all 6
    faces of a car-sized box positioned exactly the way the two broken
    end-to-end tests above did (resting flat on the floor, close enough to
    a large-radius — 292 uu — wall-floor quarter-pipe that its corners
    straddle the curve), and confirms no sampled face-interior point ever
    exceeds the box's own 8 corners' maximum distance from the axis.
    `box_vs_quarter_pipe`/`box_vs_corner_fillet` themselves are unchanged
    from `RB-PHYSICS-001-FR-027` (the GJK-based replacement was built,
    found to regress two real tests, and reverted rather than shipped —
    the honest outcome of this investigation is a corrected doc comment,
    not new production code). 1 new test (246 total in `rb_physics_bullet`,
    +1 over FR-031's 245).
- `RB-PHYSICS-001-FR-033` (genuine net mesh, implemented, ball only):
  closes the "genuine net mesh" Non-goal `RB-PHYSICS-001-FR-029`'s own doc
  comment left open — until now, `RB-PHYSICS-001-FR-029`'s solid bounding
  box was the ball/car's *entire* interior boundary behind each goal
  window, with no springy/catching netting anywhere. New module `net`
  (`net::NetMesh`) is a rectangular mass-spring grid: `NetMesh::
  rectangular_grid` builds a `cols` x `rows` grid of point masses (each a
  real `RigidBody::sphere`, deliberately tiny and light — see the
  module's own doc comment for why reusing this crate's existing
  rigid-body/collision/solver machinery, rather than a bespoke
  penalty-force system, was the design choice) spanning the goal-mouth
  window's own `arena::GOAL_HALF_WIDTH`/`GOAL_HEIGHT` footprint, anchoring
  (fixing in place) every point on the grid's own perimeter — representing
  the net's real attachment to the rigid goal frame (crossbar, both posts,
  the ground/back line) — and leaving every interior point free, connected
  by structural (horizontal/vertical) and shear (diagonal) springs
  (Hooke's law plus velocity damping along each spring's own axis, the one
  genuinely new piece of physics math this requirement adds — no
  precedent for it exists anywhere else in this Bullet3-derived port,
  since Bullet's own soft-body code was never part of this port's scope —
  see ADR-0004). `NetMesh::step` advances the mesh's own internal physics
  by the caller's `dt`, split into `net::NET_SUBSTEPS` smaller sub-steps
  for numerical stability (a mass-spring system this stiff would go
  unstable integrated with a single large Bullet-style step), and resolves
  the ball's contact against every free point it currently overlaps via a
  new `collision::sphere_vs_sphere` (this crate's first real sphere-vs-sphere
  contact test — `contacts_between` previously returned empty for that
  shape pairing since it had no caller at all) plus the *existing*
  `solver::resolve_contacts_between` two-body path, mutating the ball's own
  velocity progressively across sub-steps exactly the way any other
  dynamic-vs-dynamic contact in this crate already does. New
  `arena::standard_nets` builds one `net::NetMesh` per goal, `NET_DEPTH`
  (an uncalibrated placeholder, less than `GOAL_DEPTH`) behind the real
  back wall — well in front of `FR-029`'s own rigid back-of-net plane,
  which stays completely unchanged as an always-there backstop.
  `PhysicsWorld` gains `nets: Vec<net::NetMesh>` and `with_net`, resolved
  after every other contact each step (`net.step(&mut self.ball, ...)`,
  mutating the ball directly rather than through
  `solver::resolve_dynamic_manifolds`' shared multi-body solve, since a
  net's own points aren't part of that scene-wide body list at all).
  - **Non-goals (this requirement).** A car's own contact against a net —
    a car still passes straight through a `net::NetMesh`'s spatial
    footprint untouched, exactly as it did before this module existed,
    stopped instead by `FR-029`'s pre-existing rigid machinery. A full 3D
    "sock" shape billowing backward from the goal mouth (this models a
    single flat rest-shape panel, which still deforms backward dynamically
    under a real ball impact via its own springs — just not a pre-shaped
    pocket). Bending stiffness (only structural + shear springs; no
    springs resisting the mesh folding along a diagonal). Manifold
    richness beyond one contact per overlapping point. Every new constant
    (`net::NET_POINT_MASS`, `NET_POINT_RADIUS`, `NET_SPRING_CONSTANT`,
    `NET_SPRING_DAMPING`, `NET_LINEAR_DAMPING`, `NET_RESTITUTION`,
    `NET_FRICTION`, `arena::NET_DEPTH`) is an uncalibrated placeholder —
    real Rocket League's actual net material properties have never been
    published, and this port's own point-mass/spring topology is already
    a simplification of a real net's continuum cloth behavior, so a
    "correct" numeric match isn't a coherent target yet either way.
  - **Acceptance criteria.** A ball fired at a lone net panel loses a
    large fraction of its speed compared to firing it through the same
    empty space with no net present — the real "catching" behavior, not
    just a geometric pass/fail contact test. An undisturbed net settles to
    a low residual velocity instead of oscillating forever. Every anchored
    point never moves under any force.
  - **Verification plan.** `net.rs` unit tests, in isolation from
    `PhysicsWorld`: perimeter points are anchored and interior points are
    not; every spring starts at exactly zero stretch (proving
    `rectangular_grid` measures `rest_length` from its own just-built flat
    geometry, not a hardcoded value); anchored points never move under
    120 steps of gravity alone; an undisturbed net's maximum free-point
    speed settles below a low threshold after 600 steps (the mass-spring
    analog of `world::tests::resting_ball_stays_at_rest`); and the real
    catching proof — a ball fired at the net's own center loses over half
    its speed within 1 simulated second (gravity zeroed to isolate the
    net's own effect), verified by manually integrating the ball's own
    transform between `NetMesh::step` calls in the test itself, since
    `NetMesh::step` (matching `PhysicsWorld`'s own staged pipeline)
    mutates only velocity, never a body's position. `collision.rs` gains
    a direct `sphere_vs_sphere` proof (two overlapping spheres produce a
    contact with the correct normal/penetration; two far-apart spheres
    produce none), replacing the old
    `contacts_between_two_spheres_is_empty` regression test whose entire
    premise this requirement reverses. `arena.rs` proves `standard_nets`
    returns exactly 2 nets, each sitting exactly `NET_DEPTH` behind the
    real back wall and spanning exactly the goal mouth's own
    `GOAL_HALF_WIDTH`/`GOAL_HEIGHT` footprint (not an arbitrary size).
    `world.rs` adds a wiring-count test (`standard_arena` carries exactly
    2 nets) plus the real live end-to-end proof: a ball fired at a lone
    net panel (`PhysicsWorld::new` plus `with_net`, isolated from the full
    `standard_arena` for the same full-arena-interference reason
    `RB-PHYSICS-001-FR-029`'s own isolated proofs are isolated) loses at
    least half its speed compared to the identical shot through the same
    scene with no net added — the actual `PhysicsWorld`-integrated
    "catching" proof, not just the isolated `net.rs`-level one. 10 net new
    tests (5 in `net.rs`, 2 in `collision.rs` net of replacing 1, 2 in
    `arena.rs`, 2 in `world.rs`, minus 1 test replaced), bringing the crate
    to 256 total (+10 over FR-032's 246).
- `RB-PHYSICS-001-FR-034` (split impulse, implemented): every contact's
  normal row now also solves a second, entirely separate "push"
  pseudo-velocity channel (`solver::resolve_push_row`/
  `resolve_two_body_push_row`), fed only by that contact's own positional
  (penetration/ERP) error — `ConstraintRow`/`TwoBodyRow` gained a new
  `rhs_penetration` field, split out of the single combined `rhs` this
  port used before this requirement — never the velocity/restitution
  error, which stays on the real `rhs`/`applied_impulse` channel exactly
  as before. Each of `SOLVER_ITERATIONS` iterations resolves both channels
  for every contact's normal row, the push channel's own accumulator
  (`applied_push_impulse`) entirely separate scratch state from the real
  channel's `applied_impulse`; after the loop, the real delta is applied
  to the body's velocity exactly as before, and the new push delta is
  applied directly to the body's position/orientation via a new
  `solver::apply_push_delta` (built on the existing
  `integrate::integrate_transform`, not new integration math) — mirroring
  Bullet's own `btSolverBody::writebackVelocity`, which performs the
  identical second, independent `integrateTransform` call using the push
  velocity right after writing back the real velocity delta. Wired into
  all three of this module's resolve entry points — `resolve_contacts`,
  `resolve_contacts_between`, and `resolve_dynamic_manifolds` (the last
  carrying one `push_deltas[i]` accumulator per body index, shared across
  manifolds the same way its pre-existing real `deltas[i]` already is) —
  with zero call-site changes anywhere outside `solver.rs` (`world.rs`,
  `net.rs`, and every other caller of these three functions is
  unaffected). A friction row never receives positional correction (its
  `rhs_penetration` is always `0.0`), matching Bullet's own split-impulse
  resolve, which only ever runs against a contact's normal row.
  - **Non-goals (this requirement).** Warm-starting and sleeping remain
    exactly as documented before this requirement (see this spec's own
    Non-goals and Open questions) — split impulse and warm-starting are
    independent fixes for two different symptoms of "this port re-derives
    every contact from zero each frame": split impulse stops deep
    penetration from injecting spurious velocity; warm-starting, still
    open, is what would actually let a bouncy resting contact settle. The
    restitution/friction average-not-max combine mode is untouched.
    `LINEAR_SLOP` stays `0.0` (Bullet's own default), so a contact with
    zero or negative penetration still takes the same `positional_error =
    0.0` branch as before this requirement — nothing about this
    requirement changes behavior for an already-settled, non-penetrating
    contact.
  - **Acceptance criteria.** A deeply-penetrating contact between two
    bodies starting and staying at rest (zero restitution, zero incoming
    velocity) leaves the real post-solve velocity along the contact normal
    near zero — no spurious velocity injected purely from resolving
    penetration — while the bodies' positions measurably separate to
    relieve the overlap. A body embedded well past a curved fillet's own
    resting distance, given enough simulated time, settles at (not past)
    that resting distance instead of coasting past it under residual
    velocity the old combined `rhs` term would have left behind.
  - **Verification plan.** Two new `solver.rs` unit tests prove the core
    claim directly:
    `split_impulse_corrects_deep_penetration_via_position_not_velocity`
    (`resolve_contacts`, one body vs. a static plane) and
    `split_impulse_corrects_deep_penetration_via_position_not_velocity_between_two_bodies`
    (`resolve_contacts_between`, two dynamic bodies) — each starts a body
    deeply overlapping with zero restitution and zero incoming velocity,
    resolves once, and checks the real velocity along the contact normal
    stayed near zero while position/separation moved measurably. All 12 of
    `solver.rs`'s pre-existing tests (resting, bouncing, friction,
    momentum, multi-body-pinch, box-symmetry) pass unchanged, confirming
    the `rhs`/`rhs_penetration` split is behavior-preserving for every
    case they already covered — the key piece of upfront analysis this
    requirement relied on: every one of those fixtures uses either zero
    relative velocity at exact, zero-penetration contact (`touching_ball`,
    `symmetric_pinch`, `resting_sphere`) or otherwise already computed
    `positional_error` as `0.0` before this requirement, so splitting it
    into a separate channel changes nothing observable for them.
    `world.rs`'s existing curved-fillet "embedded past resting distance"
    live end-to-end proofs
    (`a_ball_embedded_in_a_vertical_corner_edges_fillet_footprint_is_pushed_toward_the_axis`,
    `a_ball_embedded_in_a_compound_corner_fillets_footprint_is_pushed_toward_the_center`,
    `a_ball_embedded_in_a_goal_posts_fillet_footprint_is_pushed_toward_the_axis`,
    `a_ball_embedded_in_a_goal_corner_fillets_footprint_is_pushed_toward_the_center`)
    had their own assertions tightened by this requirement: before it,
    each only checked the ball moved "meaningfully" toward the resting
    surface, since the old combined `rhs` term left residual velocity for
    the ball to keep coasting on after the correction resolved; after it,
    each instead checks the ball settles at (not past) its exact resting
    distance, since the push channel no longer leaves any such residual
    velocity behind — a direct, live-`PhysicsWorld` confirmation of this
    requirement's own claim, not just an isolated `solver.rs`-level one.
    2 new tests, bringing the crate to 258 total (+2 over FR-033's 256).
- `RB-PHYSICS-001-FR-035` (warm-starting, implemented for
  `resolve_dynamic_manifolds` only): a new `solver::ContactCache` carries a
  manifold's converged real-channel impulses (normal plus both friction
  rows) from one call to the next, matched by each contact's approximate
  world position (`CONTACT_MATCH_DISTANCE`, an uncalibrated placeholder
  mirroring Bullet's own fixed `gContactBreakingThreshold`-style matching,
  since this port's narrow phase re-derives every contact from scratch and
  has no genuine per-point stable ID to track instead). Before iterating,
  a new `warm_start_two_body_row` applies each row's cached impulse
  directly to that manifold's shared `DeltaVelocity` accumulators — not
  merely setting `TwoBodyRow::applied_impulse` to the cached value, which
  (with `GLOBAL_CFM` always `0.0`) would leave the first iteration's
  correction unchanged from a cold start; the seed has to be baked into
  the starting delta itself, mirroring Bullet's own warm-start applying
  the cached impulse to the solver body's temporary velocity at setup
  time, before any iteration runs. `resolve_dynamic_manifolds` gained a
  new `caches: &mut HashMap<(usize, usize), ContactCache>` parameter (key
  normalized `(min, max)` so either argument order finds the same entry);
  every call rebuilds `caches` from only that call's own manifolds,
  replacing rather than merging, so a pair no longer touching is dropped
  automatically. `PhysicsWorld` gains one `dynamic_manifold_caches` field,
  persisted across steps and passed into its one
  `resolve_dynamic_manifolds` call site.
  - **Non-goals (this requirement).** `resolve_contacts`/
    `resolve_contacts_between` don't take a `ContactCache` — this port's
    fixed `SOLVER_ITERATIONS` already fully converges every one-body and
    two-body scenario this crate tests, so warm-starting either has no
    scenario to demonstrate value against yet; the wiring is deliberately
    scoped to the one call site (`resolve_dynamic_manifolds`) where a
    documented under-convergence limitation already exists
    (`RB-PHYSICS-001-FR-030`'s own extreme-mass-ratio "sandwiched" case),
    since that's the only place in this crate warm-starting can currently
    change anything observable. Consequently, static contacts (ground,
    walls, curves, corner fillets, goal walls, bounded walls, for both the
    ball and every car) are NOT warm-started, even though the same
    `ContactCache` mechanism could seed `resolve_contacts` identically if
    a future scenario needs it — deferred, not an oversight. Warm-starting
    does NOT resolve this spec's own documented "bouncy resting contact
    never settles" limitation (see this module's own doc comment): that
    symptom comes from restitution re-triggering off a fresh
    gravity-induced closing velocity every frame, not from where the
    solver's iteration starts, so warm-starting converges the same
    wrong-looking bounce faster without stopping it from recurring —
    sleeping (still unimplemented) is the actual fix for that, a
    genuinely separate mechanism.
  - **Acceptance criteria.** All of `solver.rs`'s pre-existing tests
    (`resolve_dynamic_manifolds`'s own included) pass unchanged when
    called with a fresh, empty `ContactCache` map, confirming
    warm-starting is a no-op unless something has actually been cached.
    Given the same post-call-1 state, seeding call 2 from call 1's
    converged impulses lands measurably closer to the true converged
    answer than repeating call 2 cold (a fresh, empty cache) does, for a
    manifold `RB-PHYSICS-001-FR-030` already showed doesn't fully converge
    within one call's `SOLVER_ITERATIONS`.
  - **Verification plan.** One new `solver.rs` test — mangled here only by
    this spec's own line width, its real name has no line break —
    `warm_starting_a_sandwiched_ball_across_two_calls_converges_closer_than_a_repeated_cold_start`,
    reuses `symmetric_pinch` (the same extreme-mass-ratio scenario
    `resolve_dynamic_manifolds_keeps_more_of_every_bodys_contact_than_resolving_pairs_independently`
    already uses): call 1 (cold) partially converges and populates a
    cache; from that identical post-call-1 state, call 2 is then run
    twice on independent copies — once warm (reusing call 1's cache) and
    once cold (a fresh map) — with identical positions, contacts,
    velocities, and iteration budget both times, isolating exactly what
    the warm seed itself contributes. The warm run's ball ends up
    measurably slower (closer to the true zero-velocity equilibrium) than
    the cold repeat's. All 14 of `solver.rs`'s pre-existing tests pass
    unchanged when given an empty cache, confirming this requirement is
    behavior-preserving for every case they already covered. 1 new test,
    bringing the crate to 259 total (+1 over FR-034's 258).
- `RB-PHYSICS-001-FR-036` (constant-ambiguity resolution, implemented): a
  dedicated follow-up to `RB-PHYSICS-001-FR-031`'s own audit, resolving the
  two genuine ambiguities that audit surfaced but deliberately didn't act
  on, using real source-level research (cloning and reading RocketSim's and
  RLUtilities' own source, and the current RLBot wiki, rather than guessing
  from prior training-data recall the way FR-031's own initial framing
  apparently had). Ball radius: FR-031 framed this as a straight "`92.75`
  vs. `91.25`" choice, but the real games actually split the ball into two
  separate radii — a smaller inertia radius (`91.25`) and a distinctly
  larger collision radius (`93.15`, the mesh's own collision margin) — a
  distinction this port's `RigidBody::sphere` has no room for, since it
  carries a single unified radius used for both roles and this port has no
  separate Bullet-style collision margin of its own. The mathematically
  correct single-constant analog for this port's unified field is therefore
  the collision radius, so every `92.75` literal (`solver.rs`, `world.rs`,
  `net.rs`, `collision.rs`) became `93.15`, not `91.25` — switching to
  `91.25` would have been a regression, not a fix. `arena::CEILING_Z`:
  confirmed, via both RocketSim's own `ARENA_HEIGHT = 2048.f` and an
  independent reconstruction from real extracted collision-mesh geometry,
  to describe the same reference point this port's `CEILING_Z` does, so
  `2044.0` became `2048.0`. Two further corrections were made as a low-risk
  byproduct of the same research pass, not new findings requiring their own
  requirement: `arena::CORNER_LENGTH` (the octagon corner-cut inset) was
  wrongly documented, by FR-031 and FR-019 before it, as an uncalibrated
  placeholder with no public reference at all — it's actually confirmed
  exact against real extracted collision-mesh data, so only its doc
  comments changed, not its value (`1152.0`, already correct); likewise
  `arena::GOAL_DEPTH` was wrongly documented, by FR-029, as an uncalibrated
  invention with no reference — it's confirmed against the current RLBot
  wiki's own cited value, so again only its doc comments changed
  (`880.0`, already correct).
  - **Non-goals (this requirement).** `arena::FILLET_RADIUS` and
    `arena::CORNER_ARCH_RADIUS` remain untouched and still genuinely
    uncalibrated — the research this requirement relied on found no
    analytic single-number reference for either anywhere in the serious
    community sources (the real corner/transition geometry is a
    triangulated collision mesh, not a mathematical arc), the same
    conclusion FR-031's own audit already reached; closing that gap for
    real would mean ingesting an actual dumped mesh, a genuinely different
    and more involved follow-up this requirement's research explicitly
    recommended treating separately rather than folding in here. No new
    behavioral bug motivated this requirement — like FR-031 before it, it's
    a constant-correctness change with no associated fix to solver or
    collision logic, so (matching FR-031's own precedent) no new test was
    added; the fix is proven by the existing suite still passing unchanged.
    This requirement does not touch `RB-VERIFY-002` real-data calibration,
    no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `arena::CEILING_Z` reads `2048.0` and every
    ball-radius literal in `solver.rs`/`world.rs`/`net.rs`/`collision.rs`
    reads `93.15` (not `91.25`); `arena::CORNER_LENGTH` and
    `arena::GOAL_DEPTH` keep their existing values, with doc comments no
    longer misdescribing either as uncalibrated. All 259 pre-existing tests
    across the crate pass unchanged, since every affected test used the
    ball-radius/`CEILING_Z` constants themselves rather than a hardcoded
    duplicate of the old values, so the correction is transparent to them.
  - **Verification plan.** No new tests: this is a constant-only correction
    with no new behavior to characterize, the same precedent FR-031
    established for its own constant changes. `cargo test --workspace`
    re-run clean at 259 total (unchanged from FR-035) confirms the four
    source-file substitutions and the two `arena.rs` constant/doc-comment
    edits are behavior-preserving everywhere the old values were previously
    exercised. A targeted `grep -c "892\.75"` across the four
    non-`arena.rs` files, run before the substitution, confirmed zero
    matches in each — ruling out any risk of the mechanical `92.75` →
    `93.15` substitution corrupting `arena::GOAL_HALF_WIDTH`'s unrelated
    `892.755` literal, which contains `92.75` as a substring but lives only
    in `arena.rs`, deliberately excluded from that substitution and edited
    by hand instead.
- `RB-PHYSICS-001-FR-037` (sleeping, implemented): closes the "no sleeping"
  half of `solver`'s own documented gap `RB-PHYSICS-001-FR-035` left open
  (warm-starting closed the other half — see that entry) — the actual fix
  for a *bouncy* resting contact (restitution > 0) never settling, since
  restitution re-triggers off a fresh gravity-induced closing velocity
  every frame regardless of where the solver's own iteration starts, so
  nothing about warm-starting or split impulse could ever stop the
  residual bounce, only refusing to integrate it at all once it's small
  and old enough to call "at rest" does. New `body::RigidBody` fields
  `is_sleeping: bool` (public) and `sleep_timer: f32` (private), and two
  new methods: `update_sleep_state(&mut self, dt: f32)` — call once per
  step, after every contact is resolved but before the transform
  integrates — accumulates `sleep_timer` while both
  `linear_velocity.length()` and `angular_velocity.length()` stay under
  new constants `body::LINEAR_SLEEP_VELOCITY_THRESHOLD`/
  `ANGULAR_SLEEP_VELOCITY_THRESHOLD`, setting `is_sleeping = true` once
  `sleep_timer` reaches `body::SLEEP_TIME_THRESHOLD` and forcibly zeroing
  both velocities every call thereafter (repeated every subsequent call
  while still under threshold, since gravity/restitution would otherwise
  recompute a fresh nonzero value each step); crossing either threshold
  resets the timer and clears `is_sleeping` immediately — and
  `wake(&mut self)`, which does the same reset unconditionally, independent
  of velocity. `PhysicsWorld::step` calls `update_sleep_state` for the ball
  and every car right after the net panels step (every other contact
  already resolved) and right before `integrate_transform_and_refresh_inertia`,
  so a body newly asleep this step also freezes in place this same step.
  `drive::apply_driven_forces` calls `car.wake()` unconditionally, before
  anything else in that call runs, whenever a new private helper
  `input_is_active` finds `input` genuinely active (any nonzero
  throttle/steer/analog channel, or `jump`/`boost`/`handbrake` set) —
  necessary because a resultant-velocity-only wake check isn't enough for a
  driven body: a car accelerating from rest under a small per-frame driving
  force whose own one-frame velocity delta is itself smaller than
  `LINEAR_SLEEP_VELOCITY_THRESHOLD` would otherwise have that delta zeroed
  right back out every single frame by the still-asleep check, permanently
  stranding it. `input_is_active` treats an unrecovered analog channel
  (`None`) the same as a recovered-but-literally-neutral one (`Some(0.0)`)
  — both mean "no analog input this tick" — rather than the simpler
  `*input != ControllerInput::default()`, which would treat any `Some(0.0)`
  as active purely because it isn't `None`, keeping a car fed a real
  recorded input stream that always resolves every channel (even at rest)
  from ever sleeping at all.
  - **Non-goals (this requirement).** No persistent-island sleeping the way
    real Bullet's own architecture does it (a body wakes or sleeps
    independently of whatever else it's touching) — this crate's solver
    already treats each body independently rather than via Bullet's
    persistent islands (see `solver`'s own module doc comment), so
    per-body sleeping is the natural fit, not a simplification of a richer
    mechanism this crate lacks. No kinematic/deactivation-disabled body
    concept exists to exempt from sleeping, since this crate has no
    kinematic bodies at all. `LINEAR_SLEEP_VELOCITY_THRESHOLD`,
    `ANGULAR_SLEEP_VELOCITY_THRESHOLD`, and `SLEEP_TIME_THRESHOLD` are this
    project's own uncalibrated placeholders — no public reference states
    what threshold, if any, real Rocket League's own physics engine uses
    internally for this (a purely implementation-internal stabilization
    detail no replay or capture could ever directly reveal, even with real
    `RB-VERIFY-002` data); chosen only to sit clearly above this crate's own
    single-frame gravity/restitution velocity noise and clearly below any
    deliberate motion this crate models (see the constants' own doc
    comment in `body.rs` for the specific numbers behind that reasoning). A
    sleeping body waking only from a contact impulse or active input, never
    from a sustained-but-below-threshold external push (e.g. a very slow
    car nudging it), is a deliberate modeling choice matching real Bullet's
    own behavior, not a bug.
  - **Acceptance criteria.** A ball resting on the ground with nonzero
    restitution on both surfaces eventually has its velocity forced to
    exactly zero and `is_sleeping` set, instead of bouncing indefinitely.
    A car seeded already asleep wakes immediately (within the same step)
    the moment it receives genuinely active input, and actually
    accelerates that same step. A sleeping ball wakes when a moving car's
    contact impulse pushes its resultant velocity back above threshold,
    with no special-case contact-wake logic required beyond the ordinary
    `update_sleep_state` check. Every one of this crate's pre-existing
    tests passes unchanged, confirming sleeping doesn't alter any
    already-covered scenario's outcome within the timeframes those tests
    already used.
  - **Verification plan.** 5 new `body.rs` unit tests exercise
    `update_sleep_state`/`wake` directly and in isolation: under-threshold
    velocity doesn't sleep before the time threshold elapses; sustained
    under-threshold velocity does sleep and zeroes both velocities;
    velocity above either threshold alone never sleeps; a sleeping body
    regaining speed above threshold wakes immediately without `wake()`;
    and `wake()` itself clears both `is_sleeping` and the timer regardless
    of velocity (checked by confirming a single further sub-threshold `dt`
    right after waking isn't enough on its own to re-sleep). 3 new
    `world.rs` end-to-end tests prove the same claims through a live
    `PhysicsWorld`:
    `a_bouncy_resting_ball_actually_settles_once_asleep` (a nonzero-restitution
    ball/ground pair, run long enough to fall asleep, asserting both
    `is_sleeping` and exactly-zero velocity — the direct fix for the
    limitation `resting_ball_stays_at_rest`'s own comment, now corrected,
    used to document instead of demonstrate), `a_sleeping_car_wakes_up_the_instant_throttle_is_applied`
    (a car seeded already asleep via direct field assignment, not simulated
    settling, isolating the wake-on-input claim from how long settling
    itself takes), and `a_sleeping_ball_wakes_up_when_a_moving_car_hits_it`
    (a ball put to sleep before the car even exists in the scene, then a
    fast-moving car added and driven into it). 8 new tests, bringing the
    crate to 267 total (+8 over FR-036's 259).
- `RB-PHYSICS-001-FR-038` (car-vs-net contact, implemented): closes this
  port's own former Non-goal that "a car still passes straight through a
  `net::NetMesh`'s spatial footprint untouched" (`RB-PHYSICS-001-FR-033`'s
  own entry). `net::NetMesh::step` changed from taking the ball alone
  (`&mut RigidBody`) to taking every body that can touch the net (`&mut
  [RigidBody]`) — a single-element slice for the ball alone behaves
  identically to how this function behaved before this requirement, so
  every one of its own pre-existing unit tests updates only its call
  syntax (`std::slice::from_mut(&mut ball)`), not its own assertions. Its
  inner contact-resolution loop now iterates every body in the slice
  against each free point, instead of just the one `ball` parameter — no
  new collision code was needed at all, since `collision::contacts_between`
  already dispatches to `sphere_vs_box` for a box-vs-sphere pair (a car
  against a net point) the exact same way it always has for ball-vs-car.
  `PhysicsWorld::step` reuses the same ball-plus-cars `bodies` snapshot
  `solver::resolve_dynamic_manifolds` already resolved that step for its
  own net-step call, deferring the sync back to `self.ball`/`self.cars`
  until after every net has had its turn, instead of syncing immediately
  and rebuilding a second snapshot just for the net loop.
  - **Non-goals (this requirement).** Everything FR-033's own Non-goals
    already scoped out for the ball stays out for a car too: manifold
    richness beyond one contact per overlapping point per body, a full 3D
    "sock" shape, and bending stiffness. No new per-body distinction
    exists in how a net treats a car versus the ball — same restitution/
    friction, same point-mass contact resolution — since nothing about a
    real net's own physical behavior toward a car should differ from
    toward a ball at this level of fidelity, and this port has no
    evidence (real or cited) suggesting otherwise.
  - **Acceptance criteria.** A car fired at a net panel loses at least half
    its speed compared to an identical shot through the same empty space
    with no net present — the same "caught vs. free flight" proof
    `RB-PHYSICS-001-FR-033` already established for the ball. Every one of
    `net.rs`'s and `world.rs`'s pre-existing tests passes unchanged (after
    the mechanical slice-syntax update), confirming this requirement is
    behavior-preserving for the ball's own already-covered scenarios.
  - **Verification plan.** 2 new `net.rs` tests:
    `a_car_shot_into_the_net_is_measurably_slowed_compared_to_free_flight`
    (the direct car analog of the pre-existing ball version) and
    `a_ball_and_a_car_are_both_resolved_against_the_same_net_step` (both
    bodies in one `step` call, positioned far enough apart along the net's
    own width that they can't touch each other, proving the slice's own
    iteration genuinely resolves every element, not just the first — a
    claim the old single-body signature couldn't even represent). 1 new
    `world.rs` end-to-end test,
    `a_car_shot_at_a_goal_net_is_caught_instead_of_passing_through_untouched`,
    mirrors the existing ball version exactly, through a real
    `PhysicsWorld` — floated near the net panel's own vertical center
    (matching the ball version's positioning) rather than resting on the
    ground, since a car sized to rest at ground height would only ever
    reach the panel's anchored bottom row, which `NetMesh::step`'s own
    contact-resolution loop deliberately skips (see that function's own
    doc comment) — a real trap this test's own first draft fell into
    before being corrected. 3 new tests, bringing the crate to 271 total
    (+3 over FR-039's 268).
- `RB-PHYSICS-001-FR-039` (wall-jump corner disambiguation, implemented):
  closes the "first wall in `self.walls`" simplification `RB-PHYSICS-001-FR-013`
  originally documented and `FR-019`'s new diagonal corner walls made
  reachable in the standard arena for the first time (see both entries'
  own Non-goals). `PhysicsWorld::step`'s per-car wall-normal computation
  (feeding `drive::apply_driven_forces`'s `wall_normal` parameter) now
  collects *every* wall a car is touching this step, sums their normals,
  and normalizes the result, instead of `Iterator::find`-ing the first
  match. A car touching exactly one wall gets that wall's own normal back
  unchanged — summing a single unit vector and normalizing it is a no-op —
  so the far more common single-wall case is bit-for-bit unaffected (every
  pre-existing wall-jump test, including
  `a_car_touching_a_wall_wall_jumps_outward_and_upward`, passes unchanged).
  A car touching two walls at once now pushes off diagonally, along the
  sum of both normals, rather than firing along only one of them depending
  on which wall happens to be earlier in `PhysicsWorld.walls`. No new
  collision code was needed: `resolve_plane_contact` already resolved a
  car touching two walls simultaneously correctly (each wall's own contact
  independently, every step) — only the wall-jump *push-off direction*
  picker was affected, since it's `drive::apply_driven_forces`'s own
  input, not part of physical contact resolution at all.
  - **Non-goals (this requirement).** No genuine multi-wall solid-angle
    weighting (e.g. giving a near-tangent wall's normal less influence than
    a near-perpendicular one) — an unweighted sum, same as summing
    unit-length contact normals in `collision`'s own multi-contact
    manifolds elsewhere in this crate. No change to physical contact
    resolution itself (`resolve_plane_contact` already handled simultaneous
    multi-wall contact correctly, see FR-013) — this closes only the
    wall-jump-direction gap, not a collision-detection one. The
    exactly-opposite-normals degenerate case (summing to the zero vector,
    geometrically impossible for a convex arena interior but handled
    defensively by falling back to the first touched wall's normal) is not
    exercised by any test, since no reachable scene in this crate's own
    arena or test helpers can produce it.
  - **Acceptance criteria.** A car touching exactly one wall wall-jumps
    identically to before this change (all pre-existing wall-jump tests
    pass unchanged). A car touching two walls at a right-angle corner
    wall-jumps with both horizontal velocity components positive and, for
    a symmetric (equal-angle) corner, roughly equal in magnitude — proof
    the push-off direction blends both walls rather than picking one.
  - **Verification plan.** New `world.rs` test
    `a_car_touching_two_walls_at_a_corner_wall_jumps_diagonally_outward`:
    two perpendicular walls (normals `(1,0,0)` and `(0,1,0)`), a car
    positioned to touch both simultaneously (zero gap on each, matching the
    existing single-wall test's own contact convention), jump pressed while
    airborne with gravity zeroed out to isolate the wall jump. Asserts both
    `vx > 0.0` and `vy > 0.0` (the old "first wall wins" picker would zero
    one of them) and `|vx - vy| < 1.0` (the two walls' normals are
    symmetric, so a true blend gives equal components), plus the same
    roughly-`JUMP_SPEED` vertical check every wall-jump test already makes.
    1 new test, bringing the crate to 268 total (+1 over FR-037's 267).
- `RB-PHYSICS-001-FR-040` (fillet-radius calibration research, investigated;
  both radii since measured from a real capture by `RB-PHYSICS-001-FR-085`):
  a dedicated research pass, matching `RB-PHYSICS-001-FR-036`'s own method
  (real source-level research, not guessed at), specifically targeting the
  two remaining uncalibrated placeholder constants `RB-PHYSICS-001-FR-036`
  itself deliberately left untouched: `arena::FILLET_RADIUS` and
  `arena::CORNER_ARCH_RADIUS`. Searched this port's established reference
  tier — RocketSim's and RLUtilities' own source, the RLBot wiki, and
  RLGym's own documented game-value list — for any named constant or
  cited measurement describing either radius. Found exactly one candidate,
  on the RLBot wiki's "Useful Game Values" page: "Wall bottom ramp radius:
  Aprox. 256 (but they are not circular)". This does not clear the bar
  `RB-PHYSICS-001-FR-036` set for adopting a community value: it carries no
  citation or attribution (unlike RocketSim's own named `ARENA_HEIGHT =
  2048.f` constant `FR-036` read directly from source), it doesn't
  distinguish a cardinal wall's own `FILLET_RADIUS` from a diagonal corner
  wall's distinctly bigger `CORNER_ARCH_RADIUS` — this project's two
  separate constants, since `RB-PHYSICS-001-FR-025` — and its own wording
  explicitly disclaims describing a true circular arc at all. Cross-checking
  RLGym's own documented game values surfaced a further reason for caution:
  RLGym separately documents `RAMP_HEIGHT = 256` — the corner boost-pad
  ramp's vertical height *from the ground*, an entirely different
  geometric quantity from a floor-seam curve's radius — the same numeral
  as the wiki's "ramp radius" entry, suggesting the wiki page may conflate
  the two rather than independently measuring a radius at all. Given this,
  `256` was deliberately NOT adopted for either constant: doing so would
  trade one honestly-labeled uncalibrated placeholder for a
  differently-uncertain number dressed up as a real citation, a worse
  outcome than leaving the honest placeholder in place. Both
  `arena::FILLET_RADIUS`/`CORNER_ARCH_RADIUS` remain unchanged
  (`292.0`/`750.0`) and genuinely uncalibrated. Doc comments on both
  constants, this spec's Non-goals, and this spec's Open Questions were
  updated to record this finding, so a future contributor doesn't
  re-tread the same wiki search — genuinely closing this gap needs actual
  extracted collision-mesh geometry (e.g. via
  `ZealanL/RLArenaCollisionDumper`'s real triangle-mesh dump), which needs
  the owner's own Windows/Rocket League environment, the same blocker
  `RB-VERIFY-002-FR-001` already documents.
  - **Non-goals (this requirement).** No mesh-ingestion tooling was built
    or attempted — this requirement is a documentation-and-research-only
    increment, matching `RB-PHYSICS-001-FR-032`'s own precedent for a
    negative research result being real, valuable work in its own right.
    No change to either constant's value: this requirement's entire
    contribution is confirming that no reliable value exists yet, not
    picking one. Does not touch `RB-PHYSICS-001-FR-005`'s real-data
    calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `arena::FILLET_RADIUS`/`CORNER_ARCH_RADIUS`'s
    own doc comments, this spec's Non-goals section, and this spec's Open
    Questions section all accurately describe the current sourcing
    status — no longer "no reference exists at all" (a claim this research
    found to be imprecise, since one low-confidence candidate does exist),
    but "a real candidate was found and deliberately rejected, with the
    specific reasoning recorded" instead.
  - **Verification plan.** No new tests: this is a documentation-only
    correction with no new runtime behavior to characterize, the same
    precedent `RB-PHYSICS-001-FR-031`/`FR-036` established for their own
    constant-audit findings that didn't change a value. All 268 of
    `rb_physics_bullet`'s pre-existing tests (as of `FR-039`) pass
    unchanged, since neither constant's value changed.
- `RB-PHYSICS-001-FR-041` (sandwiched-solve convergence, implemented):
  investigates whether anything short of real recorded data could narrow
  `RB-PHYSICS-001-FR-030`'s own documented extreme-mass-ratio "sandwiched"
  under-convergence gap at this crate's fixed `SOLVER_ITERATIONS = 10`. An
  experiment tried a naive global SOR-style relaxation factor first, scaling
  every manifold's normal-row impulse update by a fixed `omega` before
  clamping: factors above `1.0` (over-relaxation) made `FR-030`'s own
  symmetric-pinch test scenario measurably *diverge* — the sandwiched
  ball ended up faster than even the pre-`FR-030` independent-pairwise
  approach, not just under-converged — while factors below `1.0`
  (under-relaxation) made it monotonically *better* the smaller they got,
  with no instability observed down to `0.1`. This matches standard
  PGS/SOR theory for a tightly-coupled multi-body constraint system: a
  body touched by `k` other bodies in the same step has each of those `k`
  manifolds independently apply its own full correction against that
  body's own accumulating delta velocity every iteration, over-correcting
  by roughly a factor of `k`. Rather than adopt a tuned magic `omega`
  (itself the kind of unvalidated calibration this project's own
  precedent — `RB-PHYSICS-001-FR-031`/`FR-036`/`FR-040` — treats as
  needing real data to justify), `solver::resolve_dynamic_manifolds` now
  scales each manifold's velocity-row impulse by a parameter-free `1 / k`,
  where `k` is the largest number of manifolds either of that manifold's
  two bodies takes part in this step — the same "fair share" weighting
  position-based-dynamics solvers use for a point mass under several
  simultaneous constraints. This is mathematically dominant rather than a
  fidelity trade-off: it can only reduce, never increase, a shared body's
  per-iteration overshoot, so it needed no real recorded data to justify
  adopting, unlike raising `SOLVER_ITERATIONS` itself (a real added
  per-step cost). Measured directly on `FR-030`'s own symmetric-pinch
  scenario: the combined solve's result narrows from ~89.5 to ~32 units/s
  (independent-pairwise stays ~98.9 units/s), a real, further reduction of
  the gap to the true zero-velocity answer, at zero added iteration cost.
  A body touched by only one other body this step (`k == 1`, the
  overwhelming majority of contacts) is a mathematical no-op — `1 / 1 ==
  1.0` — so every pre-existing single-manifold scenario this crate already
  tests stays bit-for-bit unaffected.
  - **Non-goals (this requirement).** Does not achieve full convergence to
    the true simultaneous-solve answer within one call's fixed
    `SOLVER_ITERATIONS` — the sandwiched case is narrowed, not closed;
    real recorded multi-car contact data would still be needed to know
    whether the remaining residual error matters for fidelity in
    practice. Does not raise `SOLVER_ITERATIONS` itself, and does not
    adopt any tuned relaxation factor other than the parameter-free
    `1 / k`. Does not touch `resolve_contacts`/`resolve_contacts_between`
    (both already fully converge every one-body/two-body scenario this
    crate tests, per `RB-PHYSICS-001-FR-035`'s own Non-goals — there is no
    shared-body contention for either path to correct). Does not touch the
    split-impulse push channel (`resolve_two_body_push_row`) — only the
    real velocity-resolving rows (normal plus both friction directions)
    are scaled. Does not touch `RB-PHYSICS-001-FR-005`'s real-data
    calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** A body shared by `k >= 2` manifolds this step
    lands measurably closer to the true simultaneous-solve answer than
    before this requirement, on the same scenario `RB-PHYSICS-001-FR-030`'s
    own tests already measure. A body touched by only one other body this
    step is completely unaffected — a single-manifold call to
    `resolve_dynamic_manifolds` matches `resolve_contacts_between` called
    directly, to within floating-point tolerance. All pre-existing tests
    pass unchanged.
  - **Verification plan.** 2 new `solver.rs` tests:
    `resolve_dynamic_manifolds_relaxes_a_shared_bodys_impulse_by_its_own_contact_degree`
    reuses `FR-030`'s own `symmetric_pinch` scenario and asserts the
    combined-solve ball speed lands well below the pre-`FR-041` ~89.5
    units/s (asserted `< 50.0`, comfortably below the measured ~32); and
    `resolve_dynamic_manifolds_with_one_manifold_per_body_matches_resolve_contacts_between`
    builds an ordinary single ball-vs-car manifold and asserts calling
    `resolve_dynamic_manifolds` on it gives the same final velocities
    (within `1e-4`) as calling `resolve_contacts_between` directly,
    proving the `k == 1` case is a genuine no-op rather than merely
    "close enough". All 271 of `rb_physics_bullet`'s pre-existing tests
    (as of `FR-040`) pass unchanged. 2 new tests, bringing the crate to
    273 total (+2 over `FR-040`'s 271).
- `RB-PHYSICS-001-FR-042` (box-vs-box reference validation, investigated):
  this spec's own Open Questions flagged `collision::box_vs_box`'s
  edge-edge contact-point derivation and its face-clipping degenerate
  ("zero points") fallback as "reasonable, tested choices" never actually
  validated against Bullet's own `btBoxBoxDetector::dBoxBox` output. This
  requirement fetched and read that reference source directly (not guessed
  at, matching `RB-PHYSICS-001-FR-036`'s own method) and checked three
  specific things against it.
  1. **Edge-edge contact point.** `dBoxBox`'s own reference computes the
     edge-edge contact point via `dLineClosestApproach` — closest approach
     between two *infinite lines*, with the resulting `alpha`/`beta`
     offsets applied with **no clamping to the finite edge length at all**
     (confirmed directly in the fetched source: `dLineClosestApproach(pa,
     ua, pb, ub, &alpha, &beta); pa[i] += ua[i]*alpha; pb[i] += ub[i]*beta;`
     — no bounds check anywhere near it). This port's own
     `closest_points_on_segments` instead implements Ericson's proper
     finite-*segment* closest-point construction (clamping `s`/`t` to
     `[0, 1]` with the two-pass re-projection that correctness requires),
     which is strictly more rigorous than the reference it ports from, not
     merely equivalent to it — a genuine, sourced improvement, confirmed
     rather than assumed.
  2. **Face-clipping degenerate fallback.** `dBoxBox`'s own reference
     contains the *exact same* undocumented judgment call this port's own
     code comment already made: two separate `// this should never happen`
     comments (one after `intersectRectQuad2` itself, one after filtering
     clipped points to penetrating-only) with zero geometric justification
     given anywhere nearby, confirming this port's own "reasonable, tested
     choice, not rigorously proven" framing wasn't a weaker position than
     the reference author's own. Where the two diverge is *policy*, not
     algorithm: `dBoxBox`'s own fallback for that branch is `return 0`
     (drop the collision entirely), while this port's own fallback
     synthesizes one contact at a clamped-center point instead. This is a
     deliberate, favorable divergence: SAT has already confirmed real
     geometric overlap along this axis by the time either fallback
     triggers, so silently dropping a confirmed collision (as the
     reference does) risks a car tunneling through in a rare grazing
     configuration, while reporting a synthesized contact does not.
  3. **Edge-edge tangent sign-selection heuristic.** Deriving which one of
     a box's 4 candidate parallel edges (along the SAT-chosen axis) is the
     "near" one requires a heuristic either way — this port's own
     `edge_contact` picks it via the raw center-to-center vector `d`;
     `dBoxBox`'s own reference instead uses the actual resolved
     collision-normal direction. A candidate fix (swap `d` for the
     already-available `normal`) was built and empirically tested against
     a true brute-force ground truth (trying all 16 sign combinations per
     configuration and taking the minimum segment-to-segment distance,
     across 50,000 randomized two-box configurations): the result was
     genuinely mixed, not a clear win either way. For large/arbitrary
     penetration depths, the current `d`-based heuristic matched the true
     optimal edge pair more often than the `normal`-based candidate
     (~11.6% vs. ~8.7%) and was rarely worse in head-to-head comparison
     (1,019 cases vs. 6,653 the other way); restricted to realistic
     near-first-contact penetration depths (`< 0.5` units, the regime this
     port's solver actually operates in every substep), the `normal`-based
     candidate matched optimal far more often (~93% vs. ~77%), but both
     heuristics still had individual outlier cases tens of units off the
     true optimum. Neither heuristic is a rigorous fix; swapping one
     imperfect approximation for a different imperfect one, with no real
     recorded car-vs-car contact data available to judge which regime
     matters more for this project's actual fidelity goals, isn't a
     justified change — so `d`-based selection is kept unchanged.
  - **Non-goals (this requirement).** Does not implement a genuinely
    rigorous (non-heuristic) closest-edge-pair selection — e.g. the full
    16-combination brute-force search this requirement's own investigation
    used only as a throwaway ground-truth oracle, or a proper
    closest-features-between-OBBs algorithm — left as a still-open item
    for whoever has real recorded car-vs-car contact data (or a concrete
    visible-artifact motivation) to justify the added complexity. Does not
    change `box_vs_box`'s SAT axis selection, `face_contact`'s clipping
    algorithm itself, or the face-contact degenerate fallback's own
    clamped-center-point construction — only the edge-edge tangent
    sign-selection heuristic was investigated as a change candidate, and
    it wasn't adopted. Does not touch `RB-PHYSICS-001-FR-005`'s real-data
    calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** Both validated design choices (finite-segment
    edge-edge contact point, synthesize-rather-than-drop face-clipping
    fallback) are confirmed correct or favorable relative to the real
    Bullet3 reference source, with the specific reference code quoted in
    this spec and in `collision.rs`'s own doc comments. The rejected
    candidate fix (normal-based edge-edge sign selection) is documented
    with its own quantified empirical findings so a future contributor
    doesn't re-build and re-test the same candidate from scratch.
  - **Verification plan.** No new tests: this is a documentation/research
    correction with no adopted runtime-behavior change, the same precedent
    `RB-PHYSICS-001-FR-032`/`FR-040` established for a rigorously
    investigated negative result being real, valuable work in its own
    right. The candidate fix's own empirical comparison (a temporary,
    deterministic pseudo-random probe test comparing the current
    heuristic, the candidate fix, and a brute-force ground truth across
    50,000 configurations) was built and run during investigation, not
    shipped as a change, the same "confirmed during test design, not
    shipped" precedent `RB-PHYSICS-001-FR-030`'s own 300-iteration
    manual check established. All 273 of `rb_physics_bullet`'s
    pre-existing tests pass unchanged, since no production code changed.
- `RB-PHYSICS-001-FR-043` (restitution/friction combine-mode reference
  validation, investigated): this spec's own "Restitution/friction combine
  mode" Open Question asserted, without ever having checked, that "Bullet's
  actual default is `btMax` for both". This requirement fetched and read
  Bullet's real `btManifoldResult::calculateCombinedRestitution`/
  `calculateCombinedFriction` reference source directly (matching
  `RB-PHYSICS-001-FR-036`/`FR-042`'s own method) and found that claim
  simply wrong: the actual default for both is an **unclamped product**
  (`a * b`; friction's own version additionally clamps the result to
  `[-10, 10]`), with no `max` mode, no `sqrt`/geometric-mean, and no
  per-pair combine-mode override anywhere in the reference short of a
  custom `gContactAddedCallback` — confirmed by reading both
  `btManifoldResult.h` and `btManifoldResult.cpp` in full, not a partial or
  summarized read. This port's own `solver::combine_restitution`/
  `combine_friction` already use average (`(a + b) * 0.5`), which was
  previously justified by the now-corrected wrong claim; this requirement
  re-examined whether that choice still holds up now that the real default
  is known, and found a genuine, positive reason to keep it: unlike the
  reference's product, average preserves the identity `combine(a, a) ==
  a` — two surfaces sharing a coefficient combine back to that same
  coefficient (`0.5` and `0.5` average to `0.5`; the reference's own product
  gives `0.25`). This matters specifically for this port, where every
  `RigidBody`/`StaticPlane`/`StaticQuarterPipe`/`StaticCornerFillet`/
  `StaticGoalWall` variant's `Default` impl currently assigns the same
  uncalibrated placeholder `0.5` for both coefficients (see `body.rs`) —
  under the reference's real product default, the overwhelming majority of
  this port's own contacts today would silently combine to `0.25`, a value
  nobody chose or reasoned about, purely as an artifact of squaring an
  already-arbitrary placeholder twice. Average avoids that specific
  artifact; whether either formula is what real Rocket League itself
  actually does remains genuinely unknown either way, and stays exactly as
  open as it was before this requirement — this requirement only corrected
  which of the two known quantities (this port's choice, and Bullet's real
  default) was being compared, not which one is game-truthful.
  - **Non-goals (this requirement).** Does not change
    `combine_restitution`/`combine_friction`'s formula — average is kept,
    not switched to product, for the identity-preservation reason above.
    Does not calibrate `RB-PHYSICS-001-FR-005`'s real-data question of
    which combine mode (if either) matches real Rocket League — no longer
    blocked on `PHASE-0-EXIT` (now closed), but not itself started,
    unaffected by this requirement's own reference-source correction. Does not touch any other Bullet-reference
    claim elsewhere in this spec — only the one this requirement's own Open
    Questions bullet made about combine mode.
  - **Acceptance criteria.** The wrong "Bullet's default is `btMax`" claim
    is replaced everywhere it appeared (this spec's Open Questions,
    `solver.rs`'s module doc comment, `body.rs`'s field doc comment) with
    the verified real default (an unclamped product), cited against the
    actual fetched reference source. This port's own choice to diverge from
    that real default carries a positive, stated justification (identity
    preservation) rather than a justification built on the wrong claim.
    `combine_restitution`/`combine_friction`'s own behavior is pinned by a
    dedicated unit test each, independent of any full contact-resolution
    scenario. All pre-existing tests pass unchanged, since no production
    behavior changed.
  - **Verification plan.** 2 new `solver.rs` tests:
    `combine_restitution_preserves_a_uniform_coefficients_identity` and
    `combine_friction_preserves_a_uniform_coefficients_identity`, each
    asserting `combine(0.5, 0.5) == 0.5` (and a second same-value pair) and
    explicitly asserting the result differs from the reference's own
    `0.5 * 0.5` product, pinning the exact property this requirement's
    justification depends on. All 273 of `rb_physics_bullet`'s pre-existing
    tests (as of `FR-042`) pass unchanged. 2 new tests, bringing the crate
    to 275 total (+2 over `FR-042`'s 273).
- `RB-PHYSICS-001-FR-044` (stale Non-goals correction, investigated): this
  spec's own top-level "Non-goals (this increment)" section carried a
  "**Split impulse.** This port always takes Bullet's non-split
  contact-resolution branch" bullet that had gone stale — `RB-PHYSICS-001-
  FR-034` implemented split impulse (a second, separate "push"
  pseudo-velocity channel per contact's normal row, fed only by positional/
  penetration error, applied directly to position/orientation rather than
  folding into real velocity) well before this requirement, and that
  Requirements entry, the version 0.34.0 Change History entry, and
  `rb_physics_bullet::solver`'s own module doc comment all already
  correctly describe it as implemented — only this one Non-goals bullet
  had never been updated to match. Confirmed the implementation is
  genuinely present (not merely documented) by locating
  `solver::resolve_push_row`/`resolve_two_body_push_row`/`apply_push_delta`
  directly in `solver.rs`, and confirmed via `grep` across the whole repo
  that this was the only occurrence of the stale claim anywhere in code or
  docs. Corrected the bullet to a strikethrough-and-close note, matching
  the same convention this section already uses for its own two other
  resolved Non-goals items (the wall-jump-corner disambiguation, closed via
  `FR-039`; the curved-geometry Non-goal, closed progressively via `FR-026`
  through `FR-033`) — a pattern this spec had already established for
  exactly this situation, just not yet applied here.
  - **Non-goals (this requirement).** Does not change any production code
    — `solver::combine_restitution`/`combine_friction`,
    `resolve_push_row`/`resolve_two_body_push_row`/`apply_push_delta`, and
    every other function this bullet touches are unchanged; only the
    Non-goals bullet's own text was wrong, not the implementation it
    described. Does not re-audit the rest of this spec's Non-goals section
    for other staleness beyond the one bullet this requirement found —
    every other bullet in that section was checked against its own
    referenced FR and confirmed still accurate as of this requirement.
  - **Acceptance criteria.** The "Split impulse" Non-goals bullet no longer
    contradicts `RB-PHYSICS-001-FR-034`'s own Requirements entry,
    `rb_physics_bullet::solver`'s module doc comment, or the version 0.34.0
    Change History entry. All pre-existing tests pass unchanged, since no
    production code changed.
  - **Verification plan.** No new tests: this is a pure documentation
    correction with no runtime behavior to characterize, the same
    precedent `RB-PHYSICS-001-FR-032`/`FR-040`/`FR-042` established for
    documentation-only findings being real, valuable work. All 275 of
    `rb_physics_bullet`'s pre-existing tests (as of `FR-043`) pass
    unchanged.
- `RB-PHYSICS-001-FR-045` (`integrate.rs` reference validation,
  investigated): `integrate.rs`'s own doc comments claim close fidelity to
  three real Bullet functions (`btRigidBody::applyDamping`,
  `btRigidBody::integrateVelocities`, and
  `btTransformUtil::integrateTransform`) — this requirement fetched and
  read all three reference files directly (`btRigidBody.cpp`/`.h`,
  `btTransformUtil.h`, plus `btQuaternion.h`/`btScalar.h` for the
  constants each depends on), matching `RB-PHYSICS-001-FR-036`/`FR-042`/
  `FR-043`'s own method, and checked every specific claim against it.
  1. **`apply_damping`'s "Bullet's default" claim.** Confirmed
     `BT_USE_OLD_DAMPING_METHOD` is never `#define`d anywhere in the
     fetched reference, so the `#else` branch (`pow(1 - damping, dt)` for
     both linear and angular) is genuinely what an unmodified Bullet build
     runs, not an assumption — the formula itself matches exactly too.
  2. **`integrate_velocities`'s `MAX_ANGVEL` clamp.** Confirmed
     `#define MAX_ANGVEL SIMD_HALF_PI` directly in the reference
     (`SIMD_HALF_PI == PI / 2`, matching this port's `FRAC_PI_2`), and the
     clamp formula (`angular_velocity *= (MAX_ANGVEL / dt) / angvel`)
     matches byte-for-byte.
  3. **`integrate_transform`'s exponential-map math.** Confirmed
     `ANGULAR_MOTION_THRESHOLD` (`0.5 * SIMD_HALF_PI`), the small-angle
     Taylor coefficient (`1 / 48`, written `0.020833333333` in the
     reference), and the sinc-based rotation-axis formula all match
     byte-for-byte against `btTransformUtil.h`.
  4. **A genuine, minor numeric difference found, not adopted.** This
     function's own degenerate-quaternion guard uses `length_squared() >
     1e-12`; the reference's equivalent guard (inside `safeNormalize` and
     again in the caller) compares against `SIMD_EPSILON`, which is
     `FLT_EPSILON` — about `1.19e-7` for `f32`, roughly 5 orders of
     magnitude larger than this port's own threshold. Not adopted: both
     values are far below any physically realistic quaternion magnitude,
     so the two are behaviorally indistinguishable for every scenario this
     crate's test suite (or any plausible simulation state) can reach —
     the same standard `RB-PHYSICS-001-FR-031`/`FR-036`/`FR-040` already
     hold uncalibrated constants to applies here: swapping one arbitrary
     tiny epsilon for a different arbitrary tiny epsilon needs a concrete
     reason this investigation didn't find one for.
  5. **A more significant finding: the fallback branch is load-bearing,
     not defensive theater.** The reference's own `integrateTransform`
     calls `predictedOrn.safeNormalize()` (itself internally guarded — it
     only normalizes when `length2() > SIMD_EPSILON`, otherwise leaves the
     quaternion untouched), then only accepts the result if it *still*
     clears that same threshold; otherwise it keeps the transform's
     pre-existing basis, i.e. the body's *old* orientation. This function's
     own `else { orientation }` branch mirrors that exact choice. Had this
     function instead called `predicted.normalize()` unconditionally,
     `rb_domain::Quat::normalize`'s own generic internal guard would have
     silently substituted `IDENTITY` for a degenerate result instead — a
     real, observable divergence from Bullet's actual reference behavior,
     which never resets to identity here, only ever falls back to the
     orientation already in hand.
  - **Non-goals (this requirement).** Does not change `apply_damping`'s or
    `integrate_velocities`'s formulas or constants — both confirmed already
    exact, nothing to change. Does not change `integrate_transform`'s
    degenerate-quaternion epsilon threshold — kept at `1e-12` for lack of a
    concrete reason to adopt Bullet's own `SIMD_EPSILON` value instead (see
    finding 4 above). Does not touch `RB-PHYSICS-001-FR-005`'s real-data
    calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** Every Bullet-reference claim in `integrate.rs`'s
    doc comments is now backed by a citation to the specific fetched
    reference file and line-level behavior it was checked against, not
    merely asserted. The distinction between this function's own
    check-then-normalize fallback and a bare, unconditional call to
    `Quat::normalize` is documented and pinned by a dedicated test, so a
    future refactor that "simplifies" this function by dropping the outer
    guard would fail a test rather than silently changing behavior. All
    pre-existing tests pass unchanged.
  - **Verification plan.** 1 new `integrate.rs` test:
    `integrate_transform_preserves_a_degenerate_orientation_instead_of_snapping_to_identity`
    passes a deliberately degenerate (all-zero) orientation as input and
    asserts the function returns that same degenerate value back unchanged
    — not `Quat::IDENTITY` — confirming the fallback branch's real,
    load-bearing purpose rather than merely confirming it doesn't panic.
    All 275 of `rb_physics_bullet`'s pre-existing tests (as of `FR-044`)
    pass unchanged. 1 new test, bringing the crate to 276 total (+1 over
    `FR-044`'s 275).
- `RB-PHYSICS-001-FR-046` (`body.rs`/`mat3.rs` reference validation,
  investigated): `body.rs`'s `Shape::local_inertia`/
  `RigidBody::update_inertia_tensor` and `mat3.rs`'s `Mat3::scaled_columns`/
  `Mat3::from_quat` all claim close fidelity to specific real Bullet
  functions — this requirement fetched and read every one directly
  (`btSphereShape.cpp`, `btBoxShape.cpp`, `btRigidBody.cpp`/`.h`,
  `btMatrix3x3.h`), matching `RB-PHYSICS-001-FR-036`/`FR-042`/`FR-043`/
  `FR-045`'s own method, and checked every specific claim against it.
  1. **`local_inertia`'s sphere/box formulas.** Confirmed
     `btSphereShape::calculateLocalInertia`'s `0.4 * mass * margin^2`
     (Bullet's sphere shape uses its own collision margin as the radius
     for this purpose, the same analog `RB-PHYSICS-001-FR-036` already
     established this port's own single radius field plays) and
     `btBoxShape::calculateLocalInertia`'s `mass / 12 * (ly^2 + lz^2)` for
     x (and the corresponding cyclic permutations for y/z) both match this
     port's own formulas byte-for-byte, including axis ordering.
  2. **`update_inertia_tensor`.** Confirmed `btRigidBody::updateInertiaTensor`
     (`m_invInertiaTensorWorld = basis.scaled(invInertiaLocal) *
     basis.transpose()`) matches this function's own implementation
     exactly, and that `Mat3::scaled_columns` matches `btMatrix3x3::scaled`'s
     own per-column scaling (`m_el[row][col] * s[col]`) byte-for-byte.
  3. **A genuine difference found in `Mat3::from_quat`, not adopted.**
     The reference's own `btMatrix3x3::setRotation` (non-SIMD branch)
     computes `s = 2 / q.length2()` and scales every cross term by `s`,
     self-correcting for a quaternion that isn't already unit length; this
     function instead hardcodes the `2` (`x2 = x + x` etc.), assuming
     exact unit length. Confirmed empirically (new test, below) that
     feeding this function a deliberately scaled, non-unit-length
     quaternion produces a matrix whose rows are no longer unit
     length — unlike Bullet's own self-correcting version, which would
     still produce a valid rotation matrix for the same input. Not
     adopted as a fix: this function's only production call site
     (`RigidBody::update_inertia_tensor`) always receives `orientation`
     immediately after `integrate::integrate_transform`'s own
     renormalization (see that function's own `RB-PHYSICS-001-FR-045`
     doc comment), so the input here is never meaningfully non-unit-length
     in practice — unlike `btMatrix3x3::setRotation`, a general-purpose
     utility called from many places in real Bullet with far less
     controlled inputs, adding the reference's own self-correction here
     would be pure defensive theater for an unreachable case.
  - **Non-goals (this requirement).** Does not change
    `local_inertia`/`update_inertia_tensor`/`scaled_columns` — all three
    confirmed already exact, nothing to change. Does not add
    self-correction to `Mat3::from_quat` for a non-unit-length input, for
    lack of a reachable production scenario that would exercise it (see
    finding 3 above). Does not touch `RB-PHYSICS-001-FR-005`'s real-data
    calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** Every Bullet-reference claim in
    `body.rs`/`mat3.rs`'s doc comments is now backed by a citation to the
    specific fetched reference file and behavior it was checked against.
    `Mat3::from_quat`'s lack of self-correction for a non-unit-length
    input is documented and pinned by a dedicated test, so a future
    refactor that starts passing it a less-controlled quaternion would
    fail a test rather than silently producing a non-orthonormal matrix.
    All pre-existing tests pass unchanged.
  - **Verification plan.** 1 new `mat3.rs` test:
    `from_quat_does_not_self_correct_a_non_unit_length_quaternion` builds
    a known unit quaternion and a scaled (length² = 4) copy of it, and
    asserts the unit quaternion's own resulting matrix has unit-length
    rows while the scaled quaternion's own resulting matrix does not —
    confirming the exact distinction from Bullet's own self-correcting
    reference. All 276 of `rb_physics_bullet`'s pre-existing tests (as of
    `FR-045`) pass unchanged. 1 new test, bringing the crate to 277 total
    (+1 over `FR-045`'s 276).
- `RB-PHYSICS-001-FR-047` (`collision.rs` remaining closed-form shape
  pairings reference validation, investigated): `collision.rs`'s
  `sphere_vs_plane`, `box_vs_plane`, `sphere_vs_box`, and `sphere_vs_sphere`
  all claim to be closed-form reductions of specific real Bullet collision
  algorithms — `box_vs_box` was already checked this way
  (`RB-PHYSICS-001-FR-042`); this requirement fetched and read the
  remaining four's own real references directly
  (`btConvexPlaneCollisionAlgorithm.cpp`/`.h`,
  `btSphereBoxCollisionAlgorithm.cpp`, `btSphereSphereCollisionAlgorithm.cpp`,
  `btManifoldPoint.h`) and checked every specific claim against them.
  1. **`sphere_vs_plane` and `sphere_vs_sphere` confirmed exact.** A
     sphere's GJK support vertex along `-planeNormal` is exactly
     `center - radius * planeNormal`, so `sphere_vs_plane` is Bullet's own
     `btConvexPlaneCollisionAlgorithm::processCollision` reduced
     analytically, not an approximation of it; `sphere_vs_sphere`'s
     `diff`/`normalOnSurfaceB`/`pos1`/`dist` construction matches real
     `btSphereSphereCollisionAlgorithm::processCollision` line for line.
  2. **`sphere_vs_box`'s deep-penetration face selection confirmed to
     reproduce Bullet's own exact tie-break order.** Real
     `btSphereBoxCollisionAlgorithm::getSpherePenetration` initializes its
     running minimum to the `+x` face and checks `+x, -x, +y, -y, +z, -z`
     in that fixed order, only overriding on a *strictly* smaller
     distance — so an exact tie always resolves to whichever face is
     checked earliest. This function's own per-axis-margin-then-sign
     approach was worked through several non-symmetric tied cases by hand
     and confirmed to reproduce the identical resolution (`x` preferred
     over `y` over `z` on an axis-level tie via `<=` comparisons; `+`
     preferred over `-` within an axis via `sign(v) >= 0.0`), not merely a
     different mathematically-valid alternative. Two harmless, unadopted
     numeric-epsilon differences also found, in the same category as
     `RB-PHYSICS-001-FR-045`'s degenerate-quaternion-guard finding: this
     function's outside/inside-branch threshold is linear
     (`outside_distance > 1e-6`) versus real Bullet's squared
     (`dist2 <= SIMD_EPSILON`, ≈ a 2.5-orders-of-magnitude-looser linear
     bound), and `sphere_vs_sphere`'s degenerate-coincident-centers
     threshold (`1e-6`) versus Bullet's `SIMD_EPSILON` (~1.19e-7).
  3. **A genuine, deliberate divergence found in `box_vs_plane`, not
     adopted.** Real `btConvexPlaneCollisionAlgorithm` does not compute
     every extreme corner in one pass: `processCollision` calls a single
     GJK support-vertex query along `-planeNormal`, producing exactly one
     contact point per frame. Its own optional multi-point "perturbation"
     path exists specifically to approximate more contact points for a
     resting polyhedral shape, but is configured off by real Bullet's own
     default (`btConvexPlaneCollisionAlgorithm::CreateFunc`'s real default
     is `m_numPerturbationIterations = 1`,
     `m_minimumPointsPerturbationThreshold = 0`, making the perturbation
     loop's own `getNumContacts() < m_minimumPointsPerturbationThreshold`
     guard never true) — so a box resting flat on a plane only reaches a
     4-point manifold gradually, via several frames of persistent-manifold
     accumulation as numerical jitter shifts which corner the single
     support query happens to return. This function's own instantaneous,
     exact 4-corner computation is confirmed a deliberate, more rigorous
     simplification of that real single-vertex-plus-persistence
     dance — the same favorable divergence already established for
     `box_vs_box` against `dBoxBox` (`RB-PHYSICS-001-FR-042`) — not
     adopted, since replicating Bullet's own frame-by-frame settling
     behavior would only reintroduce several frames of a box visibly
     "sinking in" before all 4 corners register, with no compensating
     benefit.
  - **Non-goals (this requirement).** Does not change
    `sphere_vs_plane`/`sphere_vs_box`/`sphere_vs_sphere`'s behavior — all
    confirmed already exact or an already-correct favorable divergence,
    nothing to change. Does not change `box_vs_plane` to match Bullet's
    real single-contact-plus-persistence behavior (finding 3 above; this
    port's own instantaneous exact computation is deliberately kept). Does
    not touch `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** Every Bullet-reference claim in these four
    functions' doc comments is now backed by a citation to the specific
    fetched reference file and behavior it was checked against.
    `sphere_vs_box`'s exact face-tie-break-order match is pinned by a
    dedicated test using a non-symmetric tied case, so a future refactor
    that changed the axis- or sign-preference order would fail a test
    rather than silently diverging from Bullet's own tie-break behavior.
    All pre-existing tests pass unchanged.
  - **Verification plan.** 1 new `collision.rs` test:
    `sphere_embedded_at_an_axis_tie_prefers_the_lower_axis_like_bullets_own_face_check_order`
    embeds a sphere in a box at a position chosen so the box's own `-x`
    and `+y` faces are exactly tied at margin 3.0, and asserts the
    resulting contact picks `-x` (the face Bullet's own fixed
    `+x, -x, +y, -y, +z, -z` check order would settle on), not `+y`.
    All 277 of `rb_physics_bullet`'s pre-existing tests (as of `FR-046`)
    pass unchanged. 1 new test, bringing the crate to 278 total (+1 over
    `FR-046`'s 277).
- `RB-PHYSICS-001-FR-048` (`solver.rs` constraint-row setup/resolve
  reference validation, investigated): `solver.rs`'s `restitution_curve`,
  `plane_space`, `setup_rows`, and `resolve_row` all claim close fidelity
  to specific real Bullet solver functions and `btContactSolverInfo`'s
  cited defaults — this requirement fetched and read the real references
  directly (`btSequentialImpulseConstraintSolver.cpp`/`.h`,
  `btContactSolverInfo.h`, `btVector3.h`), matching
  `RB-PHYSICS-001-FR-036`/`FR-042`/`FR-043`/`FR-045`/`FR-046`/`FR-047`'s
  own method, and checked every specific claim against it.
  1. **`plane_space` confirmed byte-for-byte exact** against real
     `btPlaneSpace1`, including its `|n.z| > 1/sqrt(2)` branch threshold.
  2. **`restitution_curve` confirmed behaviorally exact.** Real
     `restitutionCurve` returns the raw, unclamped `restitution * -rel_vel`
     (which can be negative — e.g. a contact still registered while
     already separating faster than the velocity threshold); its one
     caller, `setupContactConstraint`, immediately clamps a non-positive
     result to exactly `0.` before it reaches `velocityError`. This
     function's own `.max(0.0)` folds that call-site clamp directly into
     the curve rather than as a separate step — the value `setup_rows`
     ultimately uses is identical either way, so this is a confirmed
     equivalent restructuring, not a divergence.
  3. **`setup_rows` confirmed exact** against real
     `setupContactConstraint`/`setupFrictionConstraint` (reached via
     `addFrictionConstraint`'s own default `desiredVelocity = 0`,
     `cfmSlip = 0` — the doc comment previously cited a differently-named,
     unrelated function, `setFrictionConstraintImpulse`, which only resets
     a cached impulse to zero; corrected). The normal row's `velocity_error`/
     `positional_error` split on `gap_with_slop > 0.0` matches the
     reference's identical split on `penetration > 0` exactly
     (`penetration = cp.getDistance() + m_linearSlop` equals this port's
     own `gap_with_slop`, given `Contact::penetration_depth =
     -getDistance()`), and the friction row's `rhs: -rel_vel *
     jac_diag_ab_inv` matches the reference's `rhs = (desiredVelocity -
     rel_vel) * jacDiagABInv` at its real default `desiredVelocity = 0`
     (this port has no conveyor-belt/friction-anchor feature, so that
     default is its only reachable case).
  4. **`resolve_row` confirmed a behaviorally-equivalent unification.**
     This crate uses one function (checking both bounds) for every row,
     where real Bullet dispatches to two — `resolveSingleConstraintRowLowerLimit`
     (lower bound only) for the normal row, `resolveSingleConstraintRowGeneric`
     (both bounds) for friction rows. Confirmed this changes nothing: the
     normal row's own `upper_limit` (`1e10`, matching real Bullet's own
     `m_upperLimit = 1e10f` for that row exactly) is astronomically larger
     than any impulse a real contact produces, so the unified function's
     extra upper check is unreachable there, and the two real functions are
     otherwise byte-for-byte identical.
  5. **All 6 of `btContactSolverInfo`'s cited default constants confirmed
     exact**: `ERP2 = 0.2`, `GLOBAL_CFM = 0.0`, `LINEAR_SLOP = 0.0`,
     `RESTITUTION_VELOCITY_THRESHOLD = 0.2`, `RELAXATION = 1.0`,
     `SOLVER_ITERATIONS = 10` all match real `btContactSolverInfoData`'s
     own constructor exactly.
  6. **A genuine, significant divergence found, not adopted.** This port's
     `setup_rows` always derives both friction directions from
     `plane_space(&contact.normal)` — a fixed, velocity-independent
     tangent-plane basis. Real Bullet's actual default (in `convertContact`)
     instead derives friction direction 1 from the tangential component of
     the *current relative sliding velocity itself*
     (`cp.m_lateralFrictionDir1 = vel - normal * rel_vel`, normalized),
     falling back to `btPlaneSpace1`'s own fixed basis only when that
     tangential velocity is negligible (`lat_rel_vel <= SIMD_EPSILON`) — so
     `btPlaneSpace1` is Bullet's own degenerate-case fallback, not its
     everyday default, contrary to what this port's own doc comments
     previously implied. This is a physically meaningful difference, not
     cosmetic: each friction row is independently clamped to
     `[-mu * N, +mu * N]`, so two *fixed* orthogonal rows approximate the
     true circular friction cone with a square in tangent space, letting up
     to `sqrt(2)` times the correct friction magnitude be achieved at the
     square's diagonal for a body sliding along neither fixed axis — real
     Bullet's velocity-aligned direction 1 avoids this by keeping direction
     2 orthogonal to the actual slide (so it solves near zero) and letting
     direction 1 alone reproduce the correct single-direction-sliding
     result. Not adopted in this requirement: implementing velocity-aligned
     friction direction selection (plus its own degenerate-velocity
     fallback) is a real, testable behavioral change in its own right,
     deserving a dedicated follow-up requirement with its own before/after
     tests, the same scoping this port already used for
     `RB-PHYSICS-001-FR-030`/`FR-034`/`FR-035`/`FR-037` (each one dedicated
     solver feature).
  - **Non-goals (this requirement).** Does not change
    `restitution_curve`/`plane_space`/`setup_rows`/`resolve_row`'s
    behavior — all confirmed already exact or a confirmed-equivalent
    restructuring, nothing to change. Does not implement velocity-aligned
    friction direction selection (finding 6 above) — left as an explicitly
    tracked open item for a dedicated future requirement. Does not touch
    `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** Every Bullet-reference claim in these four
    functions' doc comments (plus the `btContactSolverInfo` defaults'
    own) is now backed by a citation to the specific fetched reference
    file and behavior it was checked against, including correcting the
    stale `setFrictionConstraintImpulse` citation. The
    `restitution_curve`/call-site-clamp equivalence is pinned by a
    dedicated test. The fixed-vs-velocity-aligned friction-direction
    divergence is documented in both `solver.rs`'s module doc comment and
    this spec, flagged as open follow-up work rather than silently
    dropped. All pre-existing tests pass unchanged.
  - **Verification plan.** 1 new `solver.rs` test:
    `restitution_curve_clamps_a_fast_separating_relative_velocity_to_zero`
    asserts `restitution_curve` never returns a negative value for a fast,
    separating relative velocity, confirming its inlined clamp reproduces
    real Bullet's own call-site clamp exactly. (`plane_space`'s own
    byte-for-byte match was already pinned by the pre-existing
    `plane_space_directions_are_orthonormal_and_perpendicular_to_normal`
    test — no need for a second one.) All 278 of `rb_physics_bullet`'s
    pre-existing tests (as of `FR-047`) pass unchanged. 1 new test,
    bringing the crate to 279 total (+1 over `FR-047`'s 278).
- `RB-PHYSICS-001-FR-049` (velocity-aligned friction direction selection,
  implemented): `RB-PHYSICS-001-FR-048` found and explicitly left open a
  genuine, significant divergence — this port's `setup_rows` and
  `setup_two_body_rows` always derived both friction directions from a
  fixed, velocity-independent `plane_space(&contact.normal)` basis, where
  real Bullet's actual default (in `convertContact`) aligns friction
  direction 1 with the tangential component of the current relative
  sliding velocity itself, falling back to `btPlaneSpace1` only when that
  velocity is negligible. This requirement closes that divergence.
  1. **A new `friction_directions` helper implements real Bullet's actual
     default.** `let tangential = relative_velocity - normal * rel_vel;`
     (the component of relative velocity perpendicular to `normal`)
     becomes direction 1 when normalizable and not negligible, matching
     real Bullet's own `cp.m_lateralFrictionDir1 = vel - normal * rel_vel`;
     direction 2 completes a right-handed orthonormal basis via
     `dir1.cross(normal)`, matching real Bullet's own
     `lateralFrictionDir1.cross(normalWorldOnB)`. Falls back to
     `plane_space`'s fixed basis when `tangential.length_squared()` is at
     or below `f32::EPSILON`, matching real Bullet's own `SIMD_EPSILON`
     threshold for negligible sliding (e.g. a body resting with zero
     tangential velocity).
  2. **A second, genuinely new fallback case was found and fixed: near-
     head-on catastrophic cancellation.** When `relative_velocity` is
     almost entirely along `normal` (a near-head-on collision), subtracting
     two nearly-equal-magnitude vectors (`relative_velocity` and
     `normal * rel_vel`) is a textbook catastrophic cancellation: the tiny
     residual `tangential` can pass the length-squared check while its
     *direction* is dominated by rounding error rather than the true
     (near-zero) tangential velocity, occasionally landing close enough to
     `normal` itself that `dir1.cross(normal)` comes out degenerate and
     fails to normalize. This is a real theoretical vulnerability in real
     Bullet's own algorithm too, but real Bullet's unguarded `normalize()`
     doesn't panic on it (it silently produces `NaN`/`Inf`), whereas this
     crate's own `Vec3::normalize()` returns `Option<Vec3>` — so
     `friction_directions` falls back to `plane_space` gracefully whenever
     either normalize step fails, rather than panicking. `plane_space`
     never subtracts two comparable vectors, so it is immune to this
     cancellation and always well-defined for any nonzero `normal`. This
     case was found empirically, by running the full test suite against
     the initial `.expect()`-based implementation and observing a real
     panic in `world::tests::a_ball_shot_through_the_goal_mouth_passes_the_standard_arenas_back_wall`.
  3. **Both one-body and two-body contact setup were updated.** `setup_rows`
     now hoists `body.velocity_at_point(&rel_pos)` into a shared
     `relative_velocity` local (previously only used inline for `rel_vel`)
     and passes it to `friction_directions`. `setup_two_body_rows`
     similarly hoists what was a per-call
     `a.velocity_at_point(&rel_pos_a) - b.velocity_at_point(&rel_pos_b)`
     closure recomputation into a single shared `relative_velocity` local,
     reused both by `friction_directions` and by the existing
     `relative_velocity_along` closure.
  - **Non-goals (this requirement).** Does not touch `restitution_curve`,
    `resolve_row`, or any of the `btContactSolverInfo` defaults — all
    already confirmed exact by `RB-PHYSICS-001-FR-048`. Does not change
    `plane_space` itself, which remains exact and is now `friction_directions`'s
    documented fallback rather than `setup_rows`/`setup_two_body_rows`'s
    unconditional choice. Does not touch `RB-PHYSICS-001-FR-005`'s
    real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** Friction direction 1 aligns with the
    tangential component of relative sliding velocity whenever that
    velocity is non-negligible and the resulting basis is well-defined;
    falls back to `plane_space` otherwise (negligible tangential velocity,
    or the near-head-on catastrophic-cancellation case). Both fallback
    cases are covered by tests. A dedicated isotropic-friction test proves
    the fix has real behavioral bite: it was verified to fail when
    `friction_directions` is reverted to unconditionally call
    `plane_space` (discarding velocity alignment), confirming this is a
    genuine regression test and not one that trivially passes regardless
    of the fix. All pre-existing tests pass unchanged.
  - **Verification plan.** 3 new `solver.rs` tests:
    `friction_directions_aligns_with_the_tangential_component_of_relative_velocity`
    asserts direction 1 matches the normalized tangential component of a
    known relative velocity and that both directions remain an orthonormal
    basis perpendicular to `normal`;
    `friction_directions_falls_back_to_plane_space_with_no_tangential_velocity`
    asserts purely-normal relative velocity reproduces `plane_space`'s own
    output exactly; `friction_deceleration_is_isotropic_regardless_of_slide_direction`
    asserts a sliding sphere loses the same fraction of tangential speed to
    friction regardless of slide direction (axis-aligned vs. diagonal) —
    a property the old fixed-basis approach could not guarantee, and
    confirmed (per the acceptance criteria above) to fail under it. All
    279 of `rb_physics_bullet`'s pre-existing tests (as of `FR-048`) pass
    unchanged. 3 new tests, bringing the crate to 282 total (+3 over
    `FR-048`'s 279).
- `RB-PHYSICS-001-FR-050` (net-point contact combined-solve investigation,
  implemented): `net::NetMesh::step` resolves every one of `bodies`' contact
  against every free net point it overlaps via
  `solver::resolve_contacts_between`, one pair at a time — the exact
  independent-pairwise shape `RB-PHYSICS-001-FR-030` already proved
  under-converges (and, worse, is genuinely order-dependent) for a shared
  body touched by 2+ others in the same step. This module's own doc comment
  had waved that off as irrelevant here, reasoning that a net point's own
  mass is "tiny enough" relative to a real ball or car — an untested claim.
  This requirement investigated it directly.
  1. **The "tiny enough" claim was checked and found false.** `NET_POINT_MASS`
     is `0.5`, exactly half of the `1.0` mass this crate's own tests
     consistently use for the ball — not a lopsided ratio at all. A ball or
     car pressing into the net commonly overlaps two or more free points at
     once, since `NET_POINT_RADIUS` (`120.0`) is deliberately a generous
     "coverage radius" relative to typical grid spacing.
  2. **A dedicated single-shot test confirmed the underlying mechanism is
     genuinely order-dependent, not merely slow to converge.** For a ball
     placed exactly symmetrically between two net-point-like bodies (so the
     true answer, by symmetry, has zero sideways velocity), resolving each
     point fully independently in one order left the ball with a nonzero
     sideways velocity; resolving in the opposite order left it with the
     mirror-image (opposite-sign) velocity — a purely arbitrary artifact of
     iteration order, not a physically meaningful result either way.
     `solver::resolve_dynamic_manifolds`'s combined solve, sharing one
     accumulator across both contacts, landed close to the true symmetric
     answer instead.
  3. **A `NetMesh::step`-level test measured the real-world size of the
     bias directly.** A ball fired squarely at the net's own center,
     straddling two symmetric free interior points, was measurably deflected
     sideways by the old sequential loop — a residual of ~0.25 units/s out
     of a 2000 units/s impact after a full second of `step` calls. Smaller
     in absolute terms than the single-shot proof (each of `NetMesh::step`'s
     own many small `NET_SUBSTEPS`-sized sub-steps gets a chance to partially
     self-correct the previous one's bias via freshly re-detected contacts),
     but nonzero, and with no physical justification for either sign.
  4. **Adopted `solver::resolve_dynamic_manifolds` for every body-vs-point
     contact within a sub-step.** `NetMesh::step` now gathers every
     overlapping body-vs-point manifold detected in a sub-step and resolves
     them together (bodies and free points combined into one temporary
     array for the call, `RigidBody` being `Copy`), instead of resolving
     each pair immediately and independently. Measured directly: this
     reduces the squarely-centered-impact residual from ~0.25 units/s to
     ~0.016 units/s, roughly a 15-fold improvement. Warm-starting is
     deliberately not part of this fix — a fresh, empty `ContactCache` is
     still passed every sub-step, cold-starting every call exactly as
     before — left as the same kind of open follow-up work
     `RB-PHYSICS-001-FR-035` already scoped out for
     `resolve_contacts`/`resolve_contacts_between` generally.
  - **Non-goals (this requirement).** Does not add warm-starting to
    `net::NetMesh::step`'s own contacts (finding 4's own scoping). Does not
    touch `resolve_contacts` (static-body contacts, e.g. ground/arena walls)
    — a body's own contact with a static surface only depends on that one
    body, so there is no cross-body information for independent resolution
    to lose there, the same reasoning `resolve_dynamic_manifolds`'s own doc
    comment already gives for excluding static contacts from its combined
    solve. Does not touch `RB-PHYSICS-001-FR-005`'s real-data calibration,
    no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `net::NetMesh::step` resolves every
    body-vs-point contact detected in a sub-step together via
    `solver::resolve_dynamic_manifolds`, not as a sequence of independent
    `solver::resolve_contacts_between` calls. A symmetric double-point
    impact no longer depends on iteration order for its qualitative
    direction, and its residual sideways deflection is measurably smaller
    than before this requirement. All pre-existing tests pass unchanged.
  - **Verification plan.** 2 new `net.rs` tests:
    `sequential_net_point_resolution_is_order_dependent_but_the_combined_solve_is_not`
    pins the exact root-cause mechanism at the raw solver level (order
    dependence for a symmetric two-point impact, and the combined solve's
    own order-independence); `a_ball_shot_squarely_into_the_net_stays_close_to_a_straight_line_instead_of_veering_sideways`
    proves it at `NetMesh::step`'s own public level, with the fix verified
    to reduce (from ~0.25 to ~0.016 units/s) rather than merely mask the
    measured residual. All 282 of `rb_physics_bullet`'s pre-existing tests
    (as of `FR-049`) pass unchanged. 2 new tests, bringing the crate to 284
    total (+2 over `FR-049`'s 282).
- `RB-PHYSICS-001-FR-051` (static multi-surface contact combined-solve
  investigation, implemented): `PhysicsWorld::step` resolved a body's
  contact against each static shape type it touches (the ground, then
  every wall, then every curve, then every corner fillet, then every goal
  wall, then every bounded wall) via one independent `solver::resolve_contacts`
  call per shape, one pair at a time — the exact independent-pairwise
  shape `RB-PHYSICS-001-FR-030`/`RB-PHYSICS-001-FR-050` already proved
  under-converges (and can be genuinely order-dependent) for a shared body
  touched by 2+ others in the same step. This port's own module doc
  comment had claimed resolving each independently was safe "since a
  body's contact with static geometry never depends on another dynamic
  body" — true, but silent on a body touching two different *static*
  surfaces at once (a car driving along a wall near the floor, or wedged
  into any corner — `RB-PHYSICS-001-FR-039`'s own wall-jump-at-a-corner
  handling already has to account for exactly this). This requirement
  investigated it directly.
  1. **A dedicated single-shot test confirmed the underlying mechanism is
     genuinely order-dependent, not merely slow to converge.** A ball
     wedged symmetrically into a corner formed by two static walls
     (perpendicular normals, identical restitution/friction), moving
     diagonally into both at once: resolving each wall fully independently
     in one order left the ball with velocity components biased toward
     whichever wall was resolved last; the opposite order gave the exact
     mirror image. The true answer, by symmetry, has equal components —
     neither sequential order matched it.
  2. **A new `solver::resolve_static_manifolds` generalizes `resolve_contacts`
     to combine every static-shape manifold a body touches into one shared
     solve.** Each manifold group still computes its own
     `combine_restitution`/`combine_friction` against the body's own
     restitution/friction (different static shapes can have different
     material properties), but every group now shares one `DeltaVelocity`/
     push-delta accumulator across the whole `SOLVER_ITERATIONS` loop,
     mirroring `resolve_dynamic_manifolds`'s (`FR-030`) and
     `net::NetMesh::step`'s (`FR-050`) own "one shared accumulator instead
     of independent sequential passes" fix. Measured directly on the
     two-wall corner scenario: the combined solve lands far closer to the
     true symmetric answer than either sequential order.
  3. **`PhysicsWorld::step` was rewired to use it.** A new
     `resolve_static_contacts` (taking a `StaticScene` bundling the six
     static-shape slices, to stay under clippy's argument-count limit)
     gathers every one of a body's contacts across every static shape into
     one manifold list, then resolves them all together — replacing the
     old five-function-per-body call sequence (`resolve_plane_contact`/
     `resolve_curve_contact`/`resolve_corner_fillet_contact`/
     `resolve_goal_wall_contact`/`resolve_bounded_wall_contact`, all
     removed) for both the ball and every car.
  4. **A `PhysicsWorld::step`-level test proves the fix at the real public
     API.** A ball fired diagonally into a symmetric two-wall corner via an
     actual `PhysicsWorld` (two `with_wall` calls) settles with nearly
     equal x/y velocity components after one real `step` call — confirmed
     to fail under the old sequential per-shape loop before the rewire.
  - **Non-goals (this requirement).** Does not add warm-starting to
    `resolve_static_manifolds`'s own contacts (still cold-started every
    call, the same scoping `RB-PHYSICS-001-FR-035` already established for
    `resolve_contacts`/`resolve_contacts_between` generally). Does not
    change any single-static-shape scenario's behavior — the combined
    solve is bit-for-bit equivalent to the old `resolve_contacts` for a
    body touching only one static shape this step (the overwhelming
    majority of contacts), since a one-manifold combined solve degenerates
    to exactly the old single-manifold loop. Does not touch
    `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `PhysicsWorld::step` resolves every one of a
    body's static-surface contacts detected in a step together via
    `solver::resolve_static_manifolds`, not as a sequence of independent
    per-shape `solver::resolve_contacts` calls. A symmetric two-wall corner
    impact no longer depends on `self.walls`' own iteration order for its
    qualitative direction, and its residual asymmetry is measurably smaller
    than before this requirement. All pre-existing tests pass unchanged.
  - **Verification plan.** 2 new tests:
    `solver::tests::sequential_wall_resolution_is_order_dependent_but_the_combined_solve_is_not`
    pins the exact root-cause mechanism at the raw solver level (order
    dependence for a symmetric two-wall impact, and the combined solve's
    own much-closer-to-symmetric result);
    `world::tests::a_ball_wedged_into_a_two_wall_corner_settles_symmetrically_instead_of_favoring_one_wall`
    proves it at `PhysicsWorld::step`'s own public level, confirmed to fail
    under the old per-shape sequential loop before the fix. All 284 of
    `rb_physics_bullet`'s pre-existing tests (as of `FR-050`) pass
    unchanged. 2 new tests, bringing the crate to 286 total (+2 over
    `FR-050`'s 284).
- `RB-PHYSICS-001-FR-052` (static-vs-dynamic combined-solve ordering
  investigation, implemented): `RB-PHYSICS-001-FR-051` closed the
  independent-pairwise gap for a body's own multiple static-shape
  contacts, and `RB-PHYSICS-001-FR-030` closed it for a body's own
  multiple dynamic-vs-dynamic manifolds — but `PhysicsWorld::step` still
  resolved those two combined solves as two separate calls: a body's
  static contacts fully resolved and applied via `solver::resolve_static_manifolds`
  before `solver::resolve_dynamic_manifolds`'s own setup for that same
  body (touching another car, say) ever read the result. This requirement
  investigated whether that boundary itself was the exact same gap one
  level up, and found it was.
  1. **A dedicated single-shot test confirmed the underlying mechanism is
     genuinely order-dependent, not merely slow to converge.** Reusing
     `RB-PHYSICS-001-FR-051`'s own symmetric two-wall corner setup, with
     one wall replaced by a very-heavy dynamic body (`mass = 1e9`)
     positioned so its own contact against the ball is geometrically
     identical to that wall's — as immovable as a real wall for all
     practical purposes, but routed through the dynamic-manifold code
     path instead of the static one: resolving the static wall fully
     first, then the dynamic body (`PhysicsWorld::step`'s own pre-fix
     order), left the ball biased toward whichever channel was resolved
     last; the reversed order gave the exact mirror image. Neither matched
     the true, by-symmetry answer.
  2. **A new `solver::resolve_manifolds` folds a step's static and
     dynamic manifolds into one shared solve.** `static_manifolds` is
     `(body_index, restitution, friction, contacts)` tuples indexed into
     the same `bodies` array `dynamic_manifolds` already uses; every
     body's own static rows and dynamic-manifold rows share one
     `DeltaVelocity`/push-delta accumulator (per body index) for the whole
     `SOLVER_ITERATIONS` loop, so an earlier static row's correction
     already influences a later dynamic row's `rhs` baseline within the
     same iteration, and vice versa. `RB-PHYSICS-001-FR-041`'s own `1 / k`
     relaxation keeps its existing meaning unchanged — `k` is still
     counted purely from `dynamic_manifolds`, and a body's static rows are
     never relaxed, matching `resolve_static_manifolds`'s own established,
     tested convergence behavior exactly (extending that relaxation to a
     body's static contacts was investigated and found to *regress*
     `RB-PHYSICS-001-FR-051`'s own two-static-wall test's convergence —
     not adopted). Only the dynamic channel still warm-starts from
     `caches`, unchanged.
  3. **`PhysicsWorld::step` was rewired to use it.** `resolve_static_contacts`
     became `static_contact_manifolds`, now returning a body's gathered
     `(restitution, friction, contacts)` groups instead of resolving them
     directly; `step` builds the `bodies` array first, gathers every
     body's static manifolds and every ball-vs-car/car-vs-car dynamic
     manifold, then makes one `solver::resolve_manifolds` call instead of
     two separate ones.
  4. **A `PhysicsWorld::step`-level test proves the fix at the real public
     API.** A ball fired diagonally into a wall-and-heavy-car corner via
     an actual `PhysicsWorld` (one real `StaticPlane` wall, one real
     heavy-mass car) settles with nearly equal x/y velocity components
     after one real `step` call — confirmed to fail under the old
     two-call sequence before the rewire.
  - **Non-goals (this requirement).** Does not add relaxation to a body's
    static rows (see finding 2 above — investigated and rejected as a
    regression). Does not change any single-channel scenario's behavior —
    a body touching only static contacts, or only dynamic manifolds, this
    step degenerates to exactly the pre-existing `resolve_static_manifolds`/
    `resolve_dynamic_manifolds` math respectively. Does not add
    warm-starting to a body's static contacts (still cold-started every
    call, the same scoping `RB-PHYSICS-001-FR-051`'s own Non-goals already
    left open). Does not touch `RB-PHYSICS-001-FR-005`'s real-data
    calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `PhysicsWorld::step` resolves every body's
    static-surface contacts and every dynamic manifold detected in a step
    together via `solver::resolve_manifolds`, not as two separate combined
    solves. A symmetric wall-and-heavy-body corner impact no longer
    depends on which solve ran first for its qualitative direction, and
    its residual asymmetry is measurably smaller than before this
    requirement. All pre-existing tests pass unchanged.
  - **Verification plan.** 2 new tests:
    `solver::tests::resolving_a_bodys_static_and_dynamic_contact_together_avoids_the_order_dependent_bias_sequential_resolution_has`
    pins the exact root-cause mechanism at the raw solver level (order
    dependence between the static and dynamic channel, and the combined
    solve's own much-closer-to-symmetric result);
    `world::tests::a_ball_wedged_between_a_wall_and_a_heavy_car_settles_symmetrically_instead_of_favoring_one`
    proves it at `PhysicsWorld::step`'s own public level, confirmed to
    fail under the old two-call sequence before the fix. All 286 of
    `rb_physics_bullet`'s pre-existing tests (as of `FR-051`) pass
    unchanged. 2 new tests, bringing the crate to 288 total (+2 over
    `FR-051`'s 286).
- `RB-PHYSICS-001-FR-053` (`combine_friction` defensive clamp, implemented):
  `RB-PHYSICS-001-FR-043` fetched and read real Bullet's own
  `btManifoldResult::calculateCombinedFriction`/`calculateCombinedRestitution`
  source and corrected this spec's wrong claim about the reference's
  default combine mode (an unclamped product, not `btMax`), but its own
  investigation stopped at the formula question and never separately
  examined one more detail visible in that same fetched source: real
  Bullet's own `calculateCombinedFriction` additionally clamps its product
  result to `[-10.0, 10.0]` (`calculateCombinedRestitution` has no such
  clamp). This requirement re-fetched and re-read
  `btManifoldResult.cpp` directly to confirm that clamp's exact mechanics
  (a plain `if` clamp, not `btClamped`, applied only to friction) and
  closed the gap.
  1. **Confirmed the clamp is currently inert for this crate's own actual
     material-property values.** Every `RigidBody`/`StaticPlane`/
     `StaticQuarterPipe`/`StaticCornerFillet`/`StaticGoalWall`/
     `StaticBoundedWall` this crate itself ever constructs uses a friction
     coefficient in `0.1..=0.9` (uncalibrated placeholders and
     `net::NET_FRICTION`/`drive::HANDBRAKE_FRICTION_MULTIPLIER`-scaled
     values alike) — averaging any two of those never approaches `±10`, so
     adopting the clamp changes zero behavior for any scene this port's
     own public API can currently construct.
  2. **Adopted the clamp anyway, for reference conformance against a
     genuinely unvalidated boundary.** Every one of those types' own
     `friction` field is a public, unvalidated `f32` — nothing in this
     crate enforces a range on it today, so a future caller (or a bug
     elsewhere) setting an extreme or negative value would hit
     `combine_friction` with no defense at all, unlike real Bullet, which
     always has this clamp regardless of caller-supplied values.
     `solver::combine_friction` now clamps its own average result to the
     same `[-10.0, 10.0]` bound, keeping the average formula
     `RB-PHYSICS-001-FR-043` already decided to keep (this requirement
     only adds the clamp, not a formula change). `combine_restitution` is
     left unclamped, matching the reference's own choice not to clamp
     restitution either.
  - **Non-goals (this requirement).** Does not change
    `combine_friction`'s own average formula (kept per
    `RB-PHYSICS-001-FR-043`'s own identity-preservation reasoning). Does
    not add any clamp to `combine_restitution` (the reference itself has
    none). Does not add range validation to `RigidBody`/`StaticPlane`/etc's
    own `friction`/`restitution` fields themselves — the clamp lives only
    at the point of combining two coefficients, matching exactly where
    real Bullet's own clamp lives. Does not touch
    `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `solver::combine_friction`'s result is
    clamped to `[-10.0, 10.0]`, matching real Bullet's own
    `calculateCombinedFriction` exactly. A dedicated test confirms the
    clamp has real bite for an out-of-range input pair, not just a
    doc-comment claim. All pre-existing tests pass unchanged, since no
    value this crate itself ever produces crosses either bound.
  - **Verification plan.** 1 new `solver.rs` test:
    `combine_friction_clamps_to_the_same_bound_real_bullet_uses` asserts
    `combine_friction(15.0, 15.0) == 10.0`,
    `combine_friction(-15.0, -15.0) == -10.0`, and an in-range pair
    (`9.0, 9.0`) stays the plain average, unaffected. All 288 of
    `rb_physics_bullet`'s pre-existing tests (as of `FR-052`) pass
    unchanged. 1 new test, bringing the crate to 289 total (+1 over
    `FR-052`'s 288).
- `RB-PHYSICS-001-FR-054` (goal-wall/bounded-wall corner-testing overlap
  investigation, implemented): `RB-PHYSICS-001-FR-028`'s own doc comment
  left one question genuinely open: could `collision::box_vs_goal_wall`'s
  per-corner window test under-detect a car's face resting flush against
  the window's own edge, with every corner just clear of it while the
  face's middle already overlapped the window — the same category of
  concern `RB-PHYSICS-001-FR-032` investigated and resolved for a curved
  fillet, but explicitly not covered by that finding since a goal
  window's boundary is a flat rectangle, not a curve. This requirement
  investigated that question directly, and its structurally-identical
  sibling `collision::box_vs_bounded_wall` (`RB-PHYSICS-001-FR-029`)
  alongside it, since both share the same "test each of the box's 8
  corners against a 2D rectangle in the plane's own `u_axis`/`v_axis`
  frame" technique — one gating contact on being *outside* a rectangle
  (the goal window), the other on being *inside* one (a bound).
  1. **`box_vs_goal_wall`: resolved, no bug.** A convex-hull argument
     closes this one cleanly. The window is a convex 2D region; a box's
     touching face is the convex hull of whichever corners individually
     penetrate the plane. If every one of those corners lies outside the
     window (a convex region), the window cannot contain the whole face
     either — full containment of a convex hull requires every one of its
     extreme points (the corners) to already be inside, so "some corner
     outside" is exactly equivalent to "the face doesn't fully fit
     through the window," the correct condition for treating it as
     blocked. And since a flat rigid face resting on a flat surface only
     ever needs its own corner points to determine the contact response
     in this port at all (the same precondition `box_vs_plane` itself
     already relies on for an *un*-windowed plane), a hole sitting
     strictly inside that face's interior — never touching a corner —
     changes nothing about which corners are individually on solid
     material. Corner-only testing is exact here, distinct from but
     analogous to FR-032's own convex-*maximum* argument for a curved
     fillet (this one is a convex-*containment* argument for a flat
     rectangle instead).
  2. **`box_vs_bounded_wall`: a genuine, currently-unreachable gap.** The
     mirror image does *not* resolve the same way, because a bound is
     solid *inside*, not outside — the safe condition would need "some
     corner inside the bound," not "some corner outside of anything." A
     face *larger* than the bound and centered on it (every corner
     outside the bound, while the bound's own rectangle sits entirely
     within the face's interior) has no corner touching solid material
     anywhere, so `box_vs_bounded_wall` returns zero contacts even though
     the middle of the face is genuinely resting on real bound material —
     a true under-detection letting a large-enough body pass straight
     through a wall it should be blocked by. Confirmed this project's own
     two `StaticBoundedWall`s (`arena::goal_side_wall`'s
     `GOAL_DEPTH * 0.5`-by-`GOAL_HEIGHT * 0.5` bound, `arena::goal_roof`'s
     `GOAL_HALF_WIDTH`-by-`GOAL_DEPTH * 0.5` one — hundreds of units on
     their shortest side) are always far larger than this project's own
     established car (`60x30x18` half-extents) or ball (`93.15` radius),
     so this gap is unreachable in `arena::standard_arena` as built today
     — deliberately left open rather than fixed, since closing it for
     real needs a proper 2D convex-polygon overlap test (the box's own
     touching-corner footprint against the bound's rectangle), not just
     corner-in/corner-out sampling, and no scene this project's own public
     API can currently construct can trigger it.
  - **Non-goals (this requirement).** Does not change either function's
    contact-generation algorithm — `box_vs_goal_wall` needed no change at
    all (confirmed exact as written); `box_vs_bounded_wall`'s own gap is
    documented, not fixed, since a real fix (2D convex-polygon overlap)
    is more machinery than any currently-constructible scene justifies.
    Does not touch `box_vs_quarter_pipe`/`box_vs_corner_fillet` (FR-027/
    FR-032's own curved-geometry functions) — this requirement is scoped
    to the two flat-rectangle-windowed shapes only. Does not touch
    `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `box_vs_goal_wall`'s doc comment states and
    justifies the convex-hull argument closing FR-028's own open
    question, with a passing test confirming a face bigger than the
    window and centered on it collides bit-for-bit identically to an
    unwindowed plane. `box_vs_bounded_wall`'s doc comment documents its
    own mirror-image gap, with a passing test confirming the current
    (undesirable but deliberately-not-fixed) zero-contact result for a
    synthetic oversized face, plus the concrete numeric argument for why
    no body this project actually constructs can reach it.
  - **Verification plan.** 2 new `collision.rs` tests:
    `box_bigger_than_the_goal_window_and_centered_on_it_collides_on_all_four_corners_matching_an_unwindowed_plane`
    (a `25x1x35` half-extent face centered on the `20`-wide/`30`-tall
    `goal_wall()` fixture's own window collides identically to
    `contacts_vs_plane` against the same wrapped plane, 4 contacts) and
    `box_much_bigger_than_the_bound_and_centered_on_it_is_missed_entirely_a_known_gap`
    (a `1x15x35` half-extent face centered on the `10`-wide/`30`-tall
    `bounded_wall()` fixture's own bound reports zero contacts, the known
    gap made concrete). All 289 of `rb_physics_bullet`'s pre-existing
    tests (as of `FR-053`) pass unchanged. 2 new tests, bringing the crate
    to 291 total (+2 over `FR-053`'s 289).
- `RB-PHYSICS-001-FR-055` (`GOAL_HALF_WIDTH`/`GOAL_HEIGHT` reference
  confirmation, stale doc correction, implemented): `arena::GOAL_HALF_WIDTH`/
  `GOAL_HEIGHT` (`RB-PHYSICS-001-FR-024`) had carried a "commonly-cited
  community number, not independently confirmed" caveat since they were
  introduced — the same sourcing tier `SIDE_WALL_X`/`BACK_WALL_Y`/
  `CEILING_Z`/`CORNER_LENGTH`/`GOAL_DEPTH` all once carried, before
  `RB-PHYSICS-001-FR-036` upgraded most of them to independently
  confirmed. This requirement closes that remaining gap and, in doing so,
  found and fixed a second, unrelated problem: this spec's own "Open
  questions" section still described `GOAL_DEPTH` as an unconfirmed
  "uncalibrated invention" — directly contradicting FR-036's own
  already-shipped Requirements entry and this spec's own Non-goals
  section, both of which already say it's confirmed. That passage was
  simply never updated when FR-036 shipped.
  1. **Fetched the current RLBot wiki's "Useful Game Values" page
     directly** (`https://github.com/RLBot/RLBot/wiki/Useful-Game-Values`,
     the same page `RB-PHYSICS-001-FR-036`'s own research already used to
     confirm `GOAL_DEPTH`), rather than trusting a paraphrase or prior
     training-data recall — matching this project's own established
     "always verify against primary fetched source" discipline
     (`RB-PHYSICS-001-FR-036`/`FR-040`/`FR-042`/`FR-043`/`FR-045`/`FR-046`/
     `FR-047`/`FR-048`/`FR-053`/`FR-054`'s own shared method).
  2. **Confirmed both constants exact.** The page's own cited "Goal
     center-to-post: 892.755" and "Goal height: z=642.775" match
     `arena::GOAL_HALF_WIDTH`/`GOAL_HEIGHT`'s existing values bit-for-bit
     — no value change, only a sourcing-status upgrade from "commonly-cited,
     unconfirmed" to "confirmed", the same non-behavioral outcome
     `RB-PHYSICS-001-FR-036` reached for `GOAL_DEPTH`/`CORNER_LENGTH`. (The
     same fetch also re-confirmed the wiki's own "Ceiling: z=2044" still
     disagrees with RocketSim's `ARENA_HEIGHT = 2048.f` — unsurprising and
     not a new finding, since `RB-PHYSICS-001-FR-036` already investigated
     and deliberately preferred the RocketSim/mesh-reconstruction value
     over this same wiki page's own ceiling number for `arena::CEILING_Z`.)
  3. **Corrected the stale Open Questions passage.** Rewrote it (with a
     strikethrough over the superseded text, matching
     `RB-PHYSICS-001-FR-039`/`FR-044`'s own precedent for closing a stale
     bullet) to state plainly that all three goal-geometry constants
     (`GOAL_HALF_WIDTH`, `GOAL_HEIGHT`, `GOAL_DEPTH`) are now confirmed,
     leaving only `arena::NET_DEPTH` (how far into that confirmed depth
     the net panel itself sits, a distinct, still-genuinely-uncalibrated
     quantity `RB-PHYSICS-001-FR-033` invented) as an open invention in
     that vicinity.
  - **Non-goals (this requirement).** Does not change either constant's
    value (both were already exact). Does not touch `arena::FILLET_RADIUS`/
    `CORNER_ARCH_RADIUS`, still genuinely uncalibrated per
    `RB-PHYSICS-001-FR-040`'s own finding. Does not touch `arena::NET_DEPTH`,
    still this project's own uncalibrated invention (no reference exists
    for how far into a goal's depth a real net actually hangs, only for
    the goal box's own total depth). Does not touch `RB-PHYSICS-001-FR-005`'s
    real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `arena::GOAL_HALF_WIDTH`/`GOAL_HEIGHT`'s own
    doc comments state they're confirmed against the RLBot wiki, not
    merely commonly-cited. The spec's own Non-goals and Open Questions
    sections no longer contradict each other or FR-036's own Requirements
    entry about `GOAL_DEPTH`'s sourcing status. All pre-existing tests
    pass unchanged, since neither constant's value changed.
  - **Verification plan.** No new tests: a pure constant-sourcing-status
    and doc-correctness change with no behavioral difference, the same
    precedent `RB-PHYSICS-001-FR-031`/`FR-036` established for their own
    constant/doc-only corrections — proven by the existing suite passing
    unchanged. `cargo test --workspace` re-run clean at 291 total
    (unchanged from `FR-054`).
- `RB-PHYSICS-001-FR-056` (boost acceleration ground/air split, implemented):
  `drive::BOOST_ACCELERATION` was a single flat constant (`991.667`)
  applied identically whether `apply_driven_forces`'s own `on_ground`
  parameter was `true` or `false` — this requirement's own doc comment
  and `RB-PHYSICS-001-FR-008`'s own Requirements entry both explicitly
  claimed boost "works identically airborne", framing that as a settled
  fact rather than an unverified assumption. Fetching RocketSim's own
  `RLConst.h` directly (`https://raw.githubusercontent.com/ZealanL/RocketSim/main/src/RLConst.h`)
  found that claim was itself wrong: the reference defines two distinct
  boost-acceleration constants, `BOOST_ACCEL_GROUND = 2975.f / 3.f`
  (≈991.667, exactly matching this port's own existing flat value) and
  `BOOST_ACCEL_AIR = 3175.f / 3.f` (≈1058.333, about 6.5% higher) — a
  genuine ground/air split this port's own single-constant model didn't
  capture, meaning every airborne boost this crate ever applied
  understated real Rocket League's own airborne boost strength.
  1. **Split the constant.** `BOOST_ACCELERATION` became two:
     `BOOST_ACCELERATION_GROUND` (unchanged value, rewritten as the exact
     `2975.0 / 3.0` fraction the reference uses, matching `JUMP_SPEED`'s
     own existing precedent for writing a sourced value as its own
     fraction rather than a rounded decimal) and `BOOST_ACCELERATION_AIR`
     (`3175.0 / 3.0`, new).
  2. **Wired the split into `apply_driven_forces`.** The boost force's
     own magnitude now selects `BOOST_ACCELERATION_GROUND` or
     `BOOST_ACCELERATION_AIR` by the same `on_ground` parameter every
     other grounded/airborne behavior in this function already reads —
     no new parameter, no new gating logic, since boost already applied
     in both cases; only which magnitude it uses changed.
  3. **Corrected every doc comment that claimed "identical".** This
     module's own doc comment, `RB-PHYSICS-001-FR-008`'s own Requirements
     entry, and this spec's own Non-goals/Open-questions mentions of
     `BOOST_ACCELERATION` all previously said or implied boost's
     magnitude (not just its ground-contact gating) was the same
     everywhere — corrected to state the gating is identical (boost
     always applies) while the magnitude genuinely isn't.
  - **Non-goals (this requirement).** Does not touch `BOOST_CONSUMPTION_RATE`
    (drain rate) or `MAX_BOOST` (tank size) — RocketSim's own
    `BOOST_USED_PER_SECOND = BOOST_MAX / 3` already matches this port's
    existing `BOOST_CONSUMPTION_RATE`/`MAX_BOOST` pair, confirmed as a
    byproduct of this same fetch but not a new finding requiring its own
    fix. Does not touch `MAX_CAR_SPEED`, `JUMP_SPEED`, or any other
    `drive.rs` constant — this requirement is scoped to the boost
    acceleration split specifically. Does not touch
    `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** A grounded car's own boost acceleration is
    unchanged (`BOOST_ACCELERATION_GROUND` matches the old
    `BOOST_ACCELERATION` value exactly). An airborne car's own boost
    acceleration is measurably higher, in the exact ratio RocketSim's own
    `BOOST_ACCEL_AIR`/`BOOST_ACCEL_GROUND` cites. No doc comment anywhere
    in this crate or spec still claims boost's magnitude is identical
    grounded and airborne. All pre-existing tests pass unchanged, since
    every one of them already either tests only grounded boost, only
    checks a sign/direction (not an exact magnitude) for airborne boost,
    or doesn't exercise boost at all.
  - **Verification plan.** 1 new `drive.rs` test:
    `boost_accelerates_an_airborne_car_faster_than_a_grounded_one` — one
    step of full boost from a dead stop, grounded vs. airborne, confirms
    the airborne velocity delta is strictly greater and its ratio to the
    grounded delta matches `BOOST_ACCELERATION_AIR / BOOST_ACCELERATION_GROUND`
    to within `1e-4` (the force scaling by mass cancels exactly on
    integration, making the ratio directly checkable without needing a
    specific car mass). All 291 of `rb_physics_bullet`'s pre-existing
    tests (as of `FR-055`) pass unchanged (net +1 test over `FR-055`'s
    291, bringing the crate to 292).
- `RB-PHYSICS-001-FR-057` (hard cap on car angular speed, implemented;
  its placement moved from mid-pipeline to after the transform integration
  by `RB-PHYSICS-001-FR-080` step (c), where RocketSim's own
  `_FinishPhysicsTick` runs — the real capture turns `≈7.6` rad/s per tick
  mid-flip at a reported `5.5`):
  `RB-PHYSICS-001-FR-056`'s fetch of RocketSim's `RLConst.h` proved a
  richer, previously-unsuspected level of detail than this port assumed
  existed for one constant (the boost ground/air split); this requirement
  fetched the same reference a second time, targeting the constants this
  crate's own module doc comment already listed as having "no public
  reference at all" for (`STEER_TORQUE`, `HANDBRAKE_FRICTION_MULTIPLIER`,
  `AIR_CONTROL_TORQUE`, `WALL_JUMP_HORIZONTAL_SPEED`, `DODGE_SPEED`,
  `DODGE_ANGULAR_SPEED`, `JUMP_HOLD_MAX_DURATION`,
  `JUMP_HOLD_ACCELERATION`, `LANDING_AUTO_UPRIGHT_TORQUE`), to check
  whether any of them had a genuine real counterpart after all. The
  fetch surfaced `CAR_MAX_ANG_SPEED = 5.5f, // Car can never exceed this
  angular velocity (radians/s)` — a hard ceiling on a car's angular
  speed that this port had never modeled at all: nothing previously
  bounded how fast sustained air control torque (or a dodge's own kick,
  or the landing-orientation assist) could spin a car, so holding full
  pitch/yaw/roll indefinitely spun it arbitrarily fast, unlike real
  Rocket League.

  Several other real constants the same fetch surfaced (dodge
  per-direction impulse scaling, auto-flip/landing-assistance thresholds,
  a ramping powerslide model, a steering-torque mapping) were considered
  and explicitly not adopted here — see this requirement's own Non-goals
  below and `RB-PHYSICS-001-FR-031`'s own "false precision" finding, which
  already worked through why porting a *torque* magnitude from
  RocketSim's engine doesn't transfer soundly to this port's own
  differently-calibrated car body/inertia tensor. `CAR_MAX_ANG_SPEED`
  is different in kind from those: it bounds the *result* (angular
  velocity, a rad/s quantity independent of whatever torque and inertia
  combination produced it), not the torque that produces it, so it
  transfers cleanly regardless of this port's own car body/inertia
  mismatch — the one candidate this fetch surfaced that actually cleared
  `RB-PHYSICS-001-FR-031`'s own bar.

  Also incidentally confirmed: `drive::DODGE_ANGULAR_SPEED` (an existing,
  explicitly uncalibrated placeholder for a dodge's own instantaneous
  spin kick) is numerically equal to `5.5`, the same value this
  requirement confirms for an unrelated purpose. That's flagged as a
  coincidence in both constants' own doc comments, not treated as a
  second confirmation of `DODGE_ANGULAR_SPEED` itself — it predates this
  cap, was chosen independently only to "look visibly fast" per its own
  doc comment, and nothing about a dodge's real kick strength was
  actually checked against RocketSim by this requirement.
  1. **Added `drive::MAX_CAR_ANGULAR_SPEED = 5.5`** (rad/s), doc-commented
     with the exact RocketSim citation above and its scope caveats (only
     covers this port's own driven-forces sources, once per step — not a
     universal post-solver clamp against every velocity source, matching
     this port's own existing precedent for its linear-speed caps rather
     than promising a stricter guarantee than this port's architecture
     actually enforces).
  2. **Added `drive::clamp_angular_speed`**, a small function that scales
     `RigidBody.angular_velocity` back down to `MAX_CAR_ANGULAR_SPEED` if
     exceeded, preserving direction — a genuine clamp, unlike
     `MAX_CAR_SPEED`/`UNBOOSTED_MAX_CAR_SPEED`, which only ever gate
     *new* throttle/boost force and never reduce velocity already past
     their own cap.
  3. **Wired the clamp in after integration, in both places driven forces
     are integrated**: `world.rs`'s `drive_and_integrate_velocities`
     (production) and `drive.rs`'s own test helper
     `step_with_input_and_dodge_flip` (so `drive.rs`'s own unit tests
     exercise the identical ordering) — both call `clamp_angular_speed`
     immediately after `integrate::integrate_velocities`, since torque
     `apply_driven_forces` applies isn't reflected in `angular_velocity`
     until that integration call runs; clamping any earlier would miss
     this step's own torque contribution entirely.
  - **Non-goals (this requirement).** Does not adopt RocketSim's dodge
    per-direction impulse-scaling constants
    (`FLIP_FORWARD/SIDE/BACKWARD_IMPULSE_MAX_SPEED_SCALE`,
    `FLIP_TORQUE_X/Y`, `FLIP_INITIAL_VEL_SCALE`) — a real, larger
    divergence from this port's own flat `DODGE_SPEED`/
    `DODGE_ANGULAR_SPEED`, but understanding the exact formula these
    constants combine into needs reading RocketSim's actual dodge
    implementation, not just its constant declarations; left as a
    candidate for a later, more careful requirement. Does not adopt
    RocketSim's auto-flip constants
    (`CAR_AUTOFLIP_IMPULSE/TORQUE/TIME/NORMZ_THRESH/ROLL_THRESH`) as a
    reference for `drive::LANDING_AUTO_UPRIGHT_TORQUE` — real Rocket
    League's auto-flip appears to be conditional/threshold-driven, which
    may not map onto this port's own continuous-torque assist model
    without further investigation. **Resolved by
    `RB-PHYSICS-001-FR-060`:** confirmed it doesn't map at all — real
    auto-flip is a grounded, jump-triggered turtle-recovery mechanic, a
    different shape entirely from this port's airborne, input-free nudge;
    see that requirement's own entry. Does not adopt RocketSim's powerslide
    constants (`POWERSLIDE_RISE_RATE`/`FALL_RATE`) for
    `drive::HANDBRAKE_FRICTION_MULTIPLIER` — those imply a ramping state
    variable, a different model shape than this port's own flat
    multiplier, too large a change to fold into this requirement. Does
    not adopt RocketSim's `THROTTLE_TORQUE_AMOUNT` for `drive::STEER_TORQUE`
    — confirming that mapping needs the actual usage-site source, not
    just the constant declaration. Does not split `AIR_CONTROL_TORQUE`
    into distinct per-axis constants, despite RocketSim defining exactly
    that (`CAR_AIR_CONTROL_TORQUE`, a per-axis pitch/yaw/roll vector) —
    see this requirement's own findings above for why a torque constant,
    unlike an angular-speed cap, doesn't clear `RB-PHYSICS-001-FR-031`'s
    "false precision" bar. Does not touch `RB-PHYSICS-001-FR-005`'s
    real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `drive::MAX_CAR_ANGULAR_SPEED`'s own doc
    comment states the exact RocketSim citation and its enforcement
    scope. Sustained full-axis air control input can no longer drive a
    car's angular speed past `MAX_CAR_ANGULAR_SPEED`, in production
    (`world.rs`) and in `drive.rs`'s own test helper alike. All
    pre-existing tests pass unchanged — none of them sustain full-axis
    air control input for long enough, in a single call, to have
    approached the cap before this requirement (confirmed by inspection:
    every existing air-control test is a single `step_with_input` call at
    this crate's own test car's mass/inertia, well under the cap in one
    step).
  - **Verification plan.** 3 new `drive.rs` tests:
    `clamp_angular_speed_is_a_no_op_below_the_cap` and
    `clamp_angular_speed_scales_an_over_cap_velocity_down_to_the_cap_preserving_direction`
    unit-test the clamp function directly; `sustained_full_roll_input_never_exceeds_the_hard_angular_speed_cap`
    holds full roll for 2 simulated seconds (120 steps at this test
    module's usual `1/60` `dt`) — far more than enough torque-time to
    exceed `MAX_CAR_ANGULAR_SPEED` without the clamp, given this test
    car's own mass/inertia — and asserts the resulting angular speed both
    stays at or under the cap and actually reaches near it (ruling out a
    vacuous pass from `AIR_CONTROL_TORQUE` being too weak to matter). All
    292 of `rb_physics_bullet`'s pre-existing tests (as of `FR-056`) pass
    unchanged (net +3 tests over `FR-056`'s 292, bringing the crate to
    295).
- `RB-PHYSICS-001-FR-058` (real speed-dependent throttle taper,
  implemented): `drive::THROTTLE_ACCELERATION`'s own doc comment had
  named this exact gap since it was first introduced — "Rocket League's
  real throttle curve tapers off nonlinearly as speed rises toward
  `UNBOOSTED_MAX_CAR_SPEED`; this port uses one constant instead, a real
  simplification (not a taper)" — applying the full flat acceleration
  right up to a hard cutoff at `UNBOOSTED_MAX_CAR_SPEED` instead of a
  genuine taper. Fetching RocketSim's own `Car.cpp` (not just its
  `RLConst.h` constants, this time) to find exactly how its own
  `THROTTLE_TORQUE_AMOUNT` was actually used surfaced the real mechanism:
  `driveEngineForce = engineThrottle * (THROTTLE_TORQUE_AMOUNT * UU_TO_BT)
  * driveSpeedScale`, where `driveSpeedScale` is
  `DRIVE_SPEED_TORQUE_FACTOR_CURVE.GetOutput(abs(forwardSpeed_UU))` — a
  genuine 3-point piecewise-linear curve
  (`{0, 1.0}, {1400, 0.1}, {1410, 0.0}`, confirmed exact against
  `RLConst.h`), not a flat value. `THROTTLE_TORQUE_AMOUNT` itself
  (`CAR_MASS_BT * 400.f`) is expressed in Bullet's own internal units and
  doesn't transfer to this port's own differently-calibrated car body the
  same clean way real acceleration constants like `BOOST_ACCEL_GROUND`/
  `AIR` do (`RB-PHYSICS-001-FR-031`'s and `FR-057`'s own "false precision"
  findings apply to it too) — but the taper *curve itself* is a pure,
  unitless ratio multiplying whatever peak acceleration applies at a
  standing start, so it transfers cleanly regardless of that mismatch,
  the same reasoning `FR-057` used to distinguish `MAX_CAR_ANGULAR_SPEED`
  (adoptable) from `AIR_CONTROL_TORQUE`'s own real per-axis split
  (not adoptable).
  1. **Added `drive::DRIVE_SPEED_TAPER_BREAKPOINTS`** (the 3 confirmed
     breakpoints, the last reusing the pre-existing `UNBOOSTED_MAX_CAR_SPEED`
     constant rather than a second literal `1410.0`) and
     **`drive::drive_speed_taper`**, a small piecewise-linear interpolator
     evaluated against this port's own pre-existing *signed*
     `throttle.signum() * forward_speed` quantity (the same value
     `apply_driven_forces`'s throttle gate already computed), clamped to
     non-negative before lookup — full torque (`1.0`) below the curve's
     domain, zero beyond it.
  2. **Replaced the hard cutoff with the taper.** `apply_driven_forces`'s
     throttle block no longer gates on
     `throttle.signum() * forward_speed < UNBOOSTED_MAX_CAR_SPEED`;
     instead it always computes `drive_speed_taper(...)` and scales
     `THROTTLE_ACCELERATION` by it, applying nothing once the taper
     reaches exactly zero (still exactly at `UNBOOSTED_MAX_CAR_SPEED`,
     the curve's own last breakpoint, so the effective cap is unchanged —
     only how acceleration approaches it changed).
  3. **Corrected the doc comments that described this as "not a
     taper".** `THROTTLE_ACCELERATION`'s own doc comment, and the
     module-level "commonly-cited constants" paragraph's claim that it
     "stands in for Rocket League's real speed-dependent throttle curve,"
     both previously described the curve itself as unmodeled — now only
     the peak magnitude remains an uncalibrated placeholder; the curve
     shape is confirmed and modeled.
  - **Non-goals (this requirement).** Does not adopt real RocketSim's own
    direction-agnostic `abs(forward speed)` curve input — this port's own
    pre-existing throttle gate was already direction-aware
    (`throttle.signum() * forward_speed`, treating "not yet moving this
    way" as a standing start), and switching to direction-agnostic input
    (tapering even when accelerating against current motion) would be a
    second, independent behavioral change this requirement doesn't take
    on. Does not change `THROTTLE_ACCELERATION`'s own peak magnitude
    (`1600.0`), still an uncalibrated placeholder — only the curve's real
    shape is adopted. Does not touch `BOOST_ACCELERATION_GROUND`/`AIR`,
    `MAX_CAR_SPEED`, `MAX_CAR_ANGULAR_SPEED`, or any other `drive.rs`
    constant. Does not touch `RB-PHYSICS-001-FR-005`'s real-data
    calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `drive_speed_taper`'s own doc comment states
    the exact RocketSim citation, its 3 confirmed breakpoints, and why
    the shape (not the magnitude) transfers cleanly. Throttle acceleration
    now measurably tapers before reaching `UNBOOSTED_MAX_CAR_SPEED`
    (10% strength at `1400` uu/s, down from full strength) rather than
    applying at full strength until a hard cutoff. The effective top
    speed throttle alone can reach is unchanged (`UNBOOSTED_MAX_CAR_SPEED`
    remains the taper's own last breakpoint). All pre-existing tests pass
    unchanged — none of them exercised the gap between `1400` and `1410`
    uu/s closely enough to distinguish the old hard cutoff from the new
    taper.
  - **Verification plan.** 2 new `drive.rs` tests:
    `drive_speed_taper_matches_the_real_curve_breakpoints_exactly`
    unit-tests the interpolator directly at both breakpoints, both
    segment midpoints, and both out-of-domain clamps;
    `throttle_acceleration_tapers_well_before_reaching_unboosted_max_speed`
    confirms a car at exactly `1400` uu/s gains only ~10% of a
    full-strength step's velocity delta from one step of full throttle —
    the exact regression this requirement closes (this test would see a
    full-strength delta instead, on the old flat-then-cutoff code). All
    295 of `rb_physics_bullet`'s pre-existing tests (as of `FR-057`) pass
    unchanged (net +2 tests over `FR-057`'s 295, bringing the crate to
    297).
- `RB-PHYSICS-001-FR-059` (real forward-speed-dependent dodge impulse
  scaling, implemented): `RB-PHYSICS-001-FR-031`'s own audit had already
  found that real Rocket League's dodge impulse has "direction/speed-dependent
  scaling" but didn't adopt it, since the audit only had `RLConst.h`'s
  constant declarations, not the formula they combine into — flagged as a
  candidate for "a later, more careful requirement" by `RB-PHYSICS-001-FR-057`'s
  own Non-goals. This requirement fetched RocketSim's own `Car.cpp`
  (`_UpdateDoubleJumpOrFlip`, the same file/technique `RB-PHYSICS-001-FR-058`
  used for the throttle taper) and found the real mechanism: a dodge's
  base impulse (`FLIP_INITIAL_VEL_SCALE = 500.f`) is scaled per-axis by
  `((maxSpeedScale - 1) * forwardSpeedRatio) + 1`, where
  `forwardSpeedRatio = abs(forwardSpeed_UU) / CAR_MAX_SPEED` and
  `maxSpeedScale` is `FLIP_FORWARD_IMPULSE_MAX_SPEED_SCALE` (`1.f` — no
  change, ever), `FLIP_BACKWARD_IMPULSE_MAX_SPEED_SCALE` (`2.5f`, applied
  when the dodge direction opposes the car's current velocity direction,
  per `shouldDodgeBackwards`), or `FLIP_SIDE_IMPULSE_MAX_SPEED_SCALE`
  (`1.9f`, applied to any side/roll dodge regardless of direction).
  1. **Adopted the confirmed real *ratios* (`2.5`, `1.9`), not the real
     base magnitude (`500`).** Since the real forward-dodge scale is
     exactly `1.0`, `DODGE_SPEED`'s own existing (still-uncalibrated)
     value doubles as the real forward-dodge case unchanged — the same
     "shape confirmed, magnitude not" split `RB-PHYSICS-001-FR-058` used
     for `THROTTLE_ACCELERATION`. Added `DODGE_BACKWARD_SPEED_SCALE =
     2.5`, `DODGE_SIDE_SPEED_SCALE = 1.9`, and
     `DODGE_BACKWARD_CLASSIFICATION_SPEED_THRESHOLD = 100.0` (RocketSim's
     own `abs(forwardSpeed_UU) < 100.0f` fallback threshold), plus two new
     functions: `dodge_speed_scale` (the confirmed linear interpolation
     from `1.0` to `scale_at_max_speed` as speed rises to `MAX_CAR_SPEED`,
     clamped beyond it) and `dodge_pitch_is_backward` (the real backward
     classification, re-derived in this port's own pitch-sign convention
     rather than translated symbol-for-symbol from the reference's own
     stick-sign convention).
  2. **Wired the scale into both dodge sites.** `apply_driven_forces`'s
     ground-dodge block and its wall-jump-dodge variant both now scale
     `DODGE_SPEED` by `dodge_speed_scale` before applying it: the pitch
     axis uses `DODGE_BACKWARD_SPEED_SCALE` when
     `dodge_pitch_is_backward` is true, `1.0` otherwise; the roll axis
     always uses `DODGE_SIDE_SPEED_SCALE`.
  3. **Corrected doc comments** (the module-level "commonly-cited
     constants" paragraph, and a new note on the module's own dodge
     description) that previously described `DODGE_SPEED` as a flat
     magnitude regardless of direction or speed.
  - **Non-goals (this requirement).** Does not adopt RocketSim's own real
    base magnitude (`FLIP_INITIAL_VEL_SCALE = 500.f`) for `DODGE_SPEED`
    itself — still an independently uncalibrated placeholder, per the
    "ratio confirmed, magnitude not" split above
    (`RB-PHYSICS-001-FR-080` step (a) later adopted it, once the isolated
    dodge fixture confirmed the real value to `~1%`). Does not adopt
    RocketSim's own direction-normalization for a diagonal dodge (this
    port's own pre-existing, already-documented simplification: pitch and
    roll contribute independently rather than being normalized into one
    direction vector, so a diagonal dodge is faster than an axis-aligned
    one here, unlike real Rocket League) — a separate, independent
    behavioral question this requirement doesn't take on
    (`RB-PHYSICS-001-FR-072` later closed this thread with a genuine fix).
    Does not adopt
    RocketSim's own direction-agnostic `abs(forward speed)` semantics for
    *which* axis counts as "backward" beyond what `dodge_pitch_is_backward`
    already re-derives — that function's own behavior is a direct,
    faithful port of the reference's real classification logic, not a
    simplification. Does not adopt RocketSim's own continuous
    torque-over-`FLIP_TORQUE_TIME` (`0.65f`) spin model — this port's
    `DODGE_ANGULAR_SPEED` remains a single instantaneous kick, a
    substantially different (and substantially larger, out-of-scope-for-
    one-requirement) redesign left for a future requirement. Does not
    adopt real yaw input's contribution to dodge direction (RocketSim's
    own `dodgeDir` combines `yaw + roll`; this port's dodge direction is
    pitch/roll only, matching its own pre-existing convention) —
    (`RB-PHYSICS-001-FR-073` later closed this thread with a genuine fix).
    Does not touch `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `dodge_speed_scale`/`dodge_pitch_is_backward`'s
    own doc comments state the exact RocketSim citations and their scope
    caveats. A backward pitch-dodge or a side (roll) dodge made at
    `MAX_CAR_SPEED` now measurably scales up to `2.5x`/`1.9x`
    `DODGE_SPEED` respectively, instead of a flat `DODGE_SPEED` regardless
    of speed or direction. A forward pitch-dodge stays at plain
    `DODGE_SPEED` regardless of current speed, matching the real forward
    scale of exactly `1.0`. All pre-existing tests pass unchanged — every
    existing dodge test dodges from a standing start (`forward_speed =
    0`), where `dodge_speed_scale` evaluates to `1.0` regardless of
    direction, an explicit zero-regression-risk property confirmed by
    inspection before implementation, not merely by the suite passing
    afterward.
  - **Verification plan.** 5 new `drive.rs` tests:
    `dodge_speed_scale_matches_the_real_curve` and
    `dodge_pitch_is_backward_matches_the_real_classification` unit-test
    the two new functions directly; `a_backward_dodge_scales_up_with_current_forward_speed`,
    `a_forward_dodge_does_not_scale_with_current_forward_speed`, and
    `a_side_dodge_scales_up_with_current_forward_speed` are integration
    tests confirming the exact scaled magnitudes from a car already at
    `MAX_CAR_SPEED` — the backward and side tests would see a plain
    `DODGE_SPEED`-sized delta instead, on the old flat-magnitude code. All
    297 of `rb_physics_bullet`'s pre-existing tests (as of `FR-058`) pass
    unchanged (net +5 tests over `FR-058`'s 297, bringing the crate to
    302).
- `RB-PHYSICS-001-FR-060` (landing auto-orientation vs. real auto-flip/
  auto-roll — audit finding, documentation only; the assist it examined
  was later retired by `RB-PHYSICS-001-FR-071`): `RB-PHYSICS-001-FR-057`'s
  own Non-goals had flagged RocketSim's auto-flip constants
  (`CAR_AUTOFLIP_IMPULSE/TORQUE/TIME/NORMZ_THRESH/ROLL_THRESH`) as a
  possible reference for `drive::LANDING_AUTO_UPRIGHT_TORQUE`, but noted
  auto-flip "appears to be conditional/threshold-driven, which may not map
  onto this port's own continuous-torque assist model without further
  investigation." This requirement resolves that open investigation by
  fetching and reading RocketSim's actual `Car.cpp` implementation (the
  same technique `RB-PHYSICS-001-FR-058`/`FR-059` used).
  1. **Found real Rocket League has no single mechanic matching this
     port's own "landing auto-orientation assistance."** It instead has two
     distinct, real, *grounded*, input-gated systems: **auto-flip** — a
     turtle-recovery flip, firing only when the player presses jump while
     `worldContact.hasContact` is true and the contact normal's Z exceeds
     `CAR_AUTOFLIP_NORMZ_THRESH` (roughly horizontal ground) and the car's
     current roll exceeds `CAR_AUTOFLIP_ROLL_THRESH` — it then applies a
     downward impulse (`CAR_AUTOFLIP_IMPULSE`, pinning the car against the
     ground as a pivot) plus a forward-axis torque (`CAR_AUTOFLIP_TORQUE`,
     signed by roll direction) sustained over a timer that starts at
     `CAR_AUTOFLIP_TIME * (absRoll / PI)`; and **auto-roll** — a continuous
     torque aligning the car's right/forward axes to the ground's surface
     normal, active only while `throttle` is held and the car has partial
     or full wheel contact, combined with a downward `CAR_AUTOROLL_FORCE`
     toward the ground. Neither is airborne, and neither fires without a
     specific player input (jump, or throttle) — both are the opposite
     shape from this port's own continuous, input-free, airborne-only
     nudge.
  2. **Corrected the `drive` module's doc comments** (the "Landing
     auto-orientation assistance" section and the "commonly-cited
     constants" paragraph) to state this finding directly, replacing the
     prior speculation that real Rocket League "likely gates this on
     actual proximity to the ground" — that speculation is now known to be
     the wrong shape of guess entirely, not merely unconfirmed.
  3. **Corrected the stale Open Questions bullet and `RB-PHYSICS-001-FR-057`'s
     own Non-goals bullet** that had left this an open "may not map...
     without further investigation" question — now resolved with a
     concrete negative/clarifying answer.
  - **Non-goals (this requirement).** Does not implement either real
    auto-flip or real auto-roll — both need new grounded, input-gated state
    machinery (a per-car timer and roll-direction sign for auto-flip; a
    ground-alignment torque computed from the contact surface normal for
    auto-roll) this port doesn't have, a substantially larger feature than
    a documentation correction; left as a candidate for a future,
    dedicated requirement if this port ever adds a wheel/tire-contact model
    fine-grained enough to support it. Does not change
    `LANDING_AUTO_UPRIGHT_TORQUE`'s value, trigger condition, or any other
    behavior — this is a pure audit/documentation finding, not a behavioral
    fix. Does not touch `RB-PHYSICS-001-FR-005`'s real-data calibration,
    no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** The `drive` module's doc comments and this
    spec's Open Questions/FR-057 Non-goals no longer describe real Rocket
    League's landing-assist trigger condition as an open "may not map...
    without further investigation" question or a ground-proximity guess —
    both now state the confirmed real auto-flip/auto-roll trigger
    conditions and why neither corresponds to this port's own mechanic.
  - **Verification plan.** No new tests (documentation-only, matching
    `RB-PHYSICS-001-FR-044`'s own precedent); all 302 of `rb_physics_bullet`'s
    pre-existing tests (as of `FR-059`) pass unchanged, confirming zero
    behavioral change.
- `RB-PHYSICS-001-FR-061` (hard caps on ball linear/angular speed,
  implemented): unlike a car, which has had a hard angular-speed ceiling
  since `RB-PHYSICS-001-FR-057`, the ball had no linear or angular speed
  cap of any kind — its `RigidBody.linear_damping`/`angular_damping` both
  default to `0.0` and nothing else ever bounds its velocity, so a large
  enough impact (or an accumulating chain of them) could in principle push
  it to an arbitrarily high speed, unlike real Rocket League's own ball.
  This requirement fetched RocketSim's own `RLConst.h` a third time
  (matching `RB-PHYSICS-001-FR-057`/`FR-060`'s own method) and found two
  confirmed real hard caps: `BALL_MAX_SPEED = 6000.f` and
  `BALL_MAX_ANG_SPEED = 6.f, // Ball can never exceed this angular
  velocity (radians/s)`. Both are pure velocity caps (not a torque or
  force constant calibrated against a specific mass/inertia), the same
  category `RB-PHYSICS-001-FR-057`'s own findings established as
  transferring cleanly regardless of this port's own ball not being
  calibrated to real Rocket League's — see that requirement's own
  reasoning for why a *result* cap clears the bar a torque-based
  placeholder can't.
  1. **Fetched RocketSim's own `Ball.cpp`** to confirm the exact
     enforcement mechanism and placement, not just the constant
     declarations: `BALL_MAX_SPEED`/`BALL_MAX_ANG_SPEED` are enforced via
     `if (vel.length2() > maxSpeedBT * maxSpeedBT) vel =
     vel.normalized() * maxSpeedBT` (and the same shape for angular
     velocity) inside `_FinishPhysicsTick()`, called after collision
     resolution, at the end of the physics tick — a hard clamp, not a
     force/torque that merely opposes exceeding it.
  2. **Added `world::BALL_MAX_SPEED = 6000.0` and
     `world::BALL_MAX_ANG_SPEED = 6.0`, plus a new `world::clamp_ball_velocity`**
     that scales `RigidBody.linear_velocity`/`angular_velocity` back down
     to each cap (preserving direction) if exceeded — the same shape
     `drive::clamp_angular_speed` already uses for the car, generalized
     to both linear and angular speed here since the ball has no
     drive-input-gated mechanic of its own to house a car-specific
     version of this in `drive.rs`. Placed in `world.rs` rather than
     `drive.rs` for that reason.
  3. **Wired `clamp_ball_velocity` into `PhysicsWorld::step`** right
     after this step's contact resolution (including any net) — the same
     point `self.ball = bodies[0]` already syncs the resolved ball back —
     and before sleep evaluation or transform integration. This matches
     real RocketSim's own "after collision resolution, end of tick"
     placement more precisely than `drive::clamp_angular_speed`'s own
     placement for the car (mid-pipeline, right after `integrate_velocities`
     but *before* that same step's own contact resolution — see that
     function's own doc comment for why it couldn't do the same).
  - **Non-goals (this requirement).** Does not adopt `BALL_DRAG = 0.03f`
    ("net-velocity drag multiplier") — fetching `Ball.cpp` found this is
    set once at ball construction (`constructionInfo.m_linearDamping =
    mutatorConfig.ballDrag`), a per-match mutator-config default, not a
    hardcoded system invariant like the two speed caps above. This port's
    own `RigidBody::sphere` constructor takes no opinion on what a "real"
    ball's own `linear_damping` should default to — the ball is
    caller-constructed, not owned by this crate — so adopting `BALL_DRAG`
    would mean either changing that constructor's own default (a
    separate, deliberate design decision this requirement doesn't take
    on) or introducing a new dedicated ball-construction helper, both out
    of scope for a narrow constant-adoption requirement; left as a
    candidate for a future, dedicated requirement. Does not touch
    `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** A ball launched far past `BALL_MAX_SPEED`
    never exceeds it after a step; `clamp_ball_velocity` scales down an
    over-cap linear or angular velocity while preserving direction, and
    is a no-op below both caps. All pre-existing tests pass unchanged —
    no existing test ever set the ball's speed anywhere close to `6000.0`
    uu/s or its angular speed anywhere close to `6.0` rad/s (the highest
    directly-assigned ball speed in the crate's own tests is `3000.0`
    uu/s; no test assigns the ball's angular velocity directly at all),
    an explicit zero-regression-risk property confirmed by inspection
    before implementation.
  - **Verification plan.** 4 new tests: `clamp_ball_velocity_is_a_no_op_below_both_caps`
    and the two `..._scales_an_over_cap_..._velocity_down_to_the_cap_preserving_direction`
    tests unit-test `clamp_ball_velocity` directly (linear and angular,
    separately); `a_ball_launched_far_past_ball_max_speed_never_exceeds_it_after_a_step`
    is an integration test through the real `PhysicsWorld::step` public
    API. All 302 of `rb_physics_bullet`'s pre-existing tests (as of
    `FR-060`) pass unchanged (net +4 tests over `FR-060`'s 302, bringing
    the crate to 306).
- `RB-PHYSICS-001-FR-062` (real ball material properties via a new
  `RigidBody::ball` constructor, implemented): `RB-PHYSICS-001-FR-061`'s
  own Non-goals had explicitly deferred adopting RocketSim's own
  `BALL_DRAG` (a per-match mutator-config default in the reference, not a
  hardcoded system invariant), noting this port's `RigidBody::sphere`
  constructor takes no opinion on a "real" ball's own damping default —
  the ball is caller-constructed, and every sphere (ball or otherwise)
  gets an identical generic `restitution = 0.5`/`friction = 0.5`/
  `linear_damping = 0.0` placeholder, with no way to say "this one is a
  real ball." This requirement resolves that exact gap.
  1. **Fetched RocketSim's own `RLConst.h`** (matching
     `RB-PHYSICS-001-FR-057`/`FR-060`/`FR-061`'s own method) and confirmed
     three real material-property constants: `BALL_RESTITUTION = 0.6f`
     ("Bounce factor"), `BALL_FRICTION = 0.35f`, and `BALL_DRAG = 0.03f`
     ("Net-velocity drag multiplier"). None of the three is a torque or
     force calibrated against a specific mass/inertia (the "false
     precision" category `RB-PHYSICS-001-FR-031` ruled out) — restitution
     and friction are dimensionless coefficients combined at contact time
     (`solver::combine_restitution`/`combine_friction`), and drag is a
     pure per-second decay rate (`integrate::apply_damping`) — so all
     three transfer cleanly regardless of this port's own ball not being
     calibrated to real Rocket League's, the same category
     `RB-PHYSICS-001-FR-061`'s own speed caps cleared.
  2. **Added `body::RigidBody::ball(radius, mass, position)`**, a new,
     additive constructor alongside the existing generic `sphere` and
     `car_box`: identical to `sphere` for `radius`/`mass`/`position`, but
     sets `restitution = 0.6`, `friction = 0.35`, and `linear_damping =
     0.03` instead of the generic placeholders. `sphere` itself is
     unchanged — every existing test's own non-ball spheres, and any test
     that deliberately wants a non-real ball (e.g. an inelastic
     `restitution = 0.0` one for a settling test), keep working exactly
     as before.
  - **Non-goals (this requirement).** Does not adopt `BALL_MASS_BT =
    CAR_MASS_BT / 6.f` — unlike the three constants above, this is an
    absolute mass expressed in Bullet's own internal units, and while the
    `1:6` ratio between car and ball mass is in principle a portable,
    dimensionless quantity the same way `RB-PHYSICS-001-FR-059`'s dodge
    scale ratios are, this project has no canonical "real" car
    construction site yet (no game binary consumes this crate; every
    `car_box` call site today is test-only) to normalize that ratio
    against, so there is nothing yet to keep a `1:6` ratio with — left for
    a future requirement once a canonical car exists. Does not change
    `sphere`'s own generic default, or retrofit any existing test to call
    `ball` instead — this is new API surface, not a migration. Does not
    touch `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `RigidBody::ball(radius, mass, position)`
    returns a body with `restitution == 0.6`, `friction == 0.35`, and
    `linear_damping == 0.03`, otherwise identical to
    `RigidBody::sphere(radius, mass, position)` (same shape, position,
    mass, inertia tensor). `RigidBody::sphere`'s own defaults
    (`restitution == 0.5`, `friction == 0.5`, `linear_damping == 0.0`)
    are unchanged. All pre-existing tests pass unchanged.
  - **Verification plan.** 3 new `body.rs` tests:
    `ball_sets_confirmed_real_material_properties` and
    `ball_otherwise_behaves_identically_to_sphere` pin the new
    constructor's own exact behavior directly;
    `sphere_still_defaults_to_the_generic_placeholder_material_properties`
    is a regression pin confirming `sphere`'s own generic default stayed
    untouched. All 306 of `rb_physics_bullet`'s pre-existing tests (as of
    `FR-061`) pass unchanged (net +3 tests over `FR-061`'s 306, bringing
    the crate to 309).
- `RB-PHYSICS-001-FR-063` (real Rocket League uses per-contact-pair-type
  restitution/friction overrides, not a per-body combine — audit finding,
  documentation only; closed by `RB-PHYSICS-001-FR-083` finding 5, which
  represents the override as `solver::PairMaterial`):
  `RB-PHYSICS-001-FR-043`'s own investigation had
  already checked which *formula* Bullet's own generic
  `combine_restitution`/`combine_friction` should use (an unclamped
  product, not this port's kept average) and left open "which formula (if
  either) actually matches real Rocket League itself... still needs real
  recorded ball/ground behavior to calibrate against." This requirement
  resolves that differently than expected: the real answer isn't "a
  different formula" at all.
  1. **Fetched RocketSim's own `RLConst.h`** (matching
     `RB-PHYSICS-001-FR-057`/`FR-060`/`FR-061`/`FR-062`'s own method) and
     found real Rocket League's gameplay layer doesn't compute restitution
     or friction from a generic per-body combine at all for its own
     named contact-pair types — it hardcodes a distinct override per pair,
     bypassing whatever either body's own material would otherwise
     combine to. Confirmed exact: `CAR_COLLISION_FRICTION = 0.3f`/
     `CAR_COLLISION_RESTITUTION = 0.1f` (a generic fallback),
     `CARWORLD_COLLISION_FRICTION = 0.3f`/
     `CARWORLD_COLLISION_RESTITUTION = 0.3f` (car vs. static geometry),
     `CARCAR_COLLISION_FRICTION = 0.09f`/`CARCAR_COLLISION_RESTITUTION =
     0.1f`, and `CARBALL_COLLISION_FRICTION = 2.0f`/
     `CARBALL_COLLISION_RESTITUTION = 0.0f`.
  2. **Found two of these are individually striking, not just
     "different from an average".** `CARBALL_COLLISION_RESTITUTION =
     0.0f` means a car hitting the ball has *zero* restitution-driven
     bounce in real Rocket League regardless of either body's own
     material — a stark contrast with this port's own
     `combine_restitution(ball.restitution, car.restitution)`, which
     since `RB-PHYSICS-001-FR-062` averages the ball's own confirmed real
     `0.6` against the car's still-generic `0.5` to a real `~0.55` bounce
     for exactly the pairing real Rocket League gives zero. Separately,
     `CARBALL_COLLISION_FRICTION = 2.0f` is a friction coefficient
     *above* `1.0` — a value no combine of two bodies' own sane
     (`0.0..=1.0`-range) per-material friction fields could ever produce,
     confirming this isn't merely an uncalibrated-magnitude question the
     way most of this project's other placeholder constants are (see
     `RB-PHYSICS-001-FR-031`'s own "false precision" category) but a
     genuinely different *model* real Rocket League uses.
  3. **Corrected `solver::combine_restitution`/`combine_friction`'s own
     doc comments** to state this finding directly, and corrected the
     stale Open Questions bullet that had framed the still-open question
     as merely "which formula" rather than "a per-body combine at all."
  - **Non-goals (this requirement).** Does not implement per-contact-
    pair-type overrides — `combine_restitution`/`combine_friction`'s own
    signature (two `f32` material values in, one combined value out) has
    no way to know which *kind* of pair produced those two values (car
    vs. world, car vs. car, car vs. ball, or a car/ball vs. some other
    static shape entirely); doing so for real would mean threading body/
    shape identity into every one of `solver.rs`'s several call sites
    (`resolve_contacts`, `resolve_contacts_between`,
    `resolve_static_manifolds`, `resolve_dynamic_manifolds`,
    `resolve_manifolds`) and adding a lookup keyed on that identity ahead
    of (or instead of) the existing combine call — a substantially larger
    architecture change than a single documentation-audit requirement
    should take on, left as a candidate for a future, dedicated
    requirement. Does not adopt `CAR_COLLISION_FRICTION`/
    `CAR_COLLISION_RESTITUTION` as the car's own default material
    properties (mirroring `RB-PHYSICS-001-FR-062`'s own `RigidBody::ball`)
    — unlike the ball, real Rocket League has no single "the car's own"
    restitution/friction value to adopt that way; every real value found
    here is contact-pair-specific, so setting the car's own generic
    default to any one of them would be arbitrary, not confirmed. Does
    not change `combine_restitution`/`combine_friction`'s own kept
    average formula (still `RB-PHYSICS-001-FR-043`'s own correct-reason
    choice for the *within-model* question that finding answered — this
    requirement is about the model itself, a separate question). Does not
    touch `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `solver::combine_restitution`/
    `combine_friction`'s own doc comments and this spec's Open Questions
    no longer frame the real-Rocket-League-combine-mode question as
    "which formula" — both now state the confirmed real per-pair-type
    override values and why this port's own per-body-combine
    architecture can't represent them without a larger, separate change.
  - **Verification plan.** No new tests (documentation-only, matching
    `RB-PHYSICS-001-FR-044`/`FR-060`'s own precedent); all 309 of
    `rb_physics_bullet`'s pre-existing tests (as of `FR-062`) pass
    unchanged, confirming zero behavioral change.
  - **Closed (2026-09-06).** `RB-PHYSICS-001-FR-083` finding 5 added the
    "larger, separate change" this entry deferred: `solver::PairMaterial`
    on each dynamic manifold, with `PhysicsWorld::step` naming the
    `CARBALL` and `CARCAR` values recorded here, plus `Ball::_OnHit`'s
    extra impulse. See that entry for the measurements.
- `RB-PHYSICS-001-FR-064` (real mandatory minimum-hold window for a ground
  jump's variable-height acceleration; its `0.62` pre-minimum scale is
  contradicted by the real capture — see `FR-083` finding 2): `drive::JUMP_HOLD_MAX_DURATION`'s
  own doc comment had flagged, since `RB-PHYSICS-001-FR-031`'s original
  audit, that real Rocket League scales its jump-hold acceleration down
  during a `JUMP_MIN_TIME` (0.025s) mandatory window rather than applying
  it flat from the first held step — "that two-phase ramp isn't modeled
  here, only the flat-acceleration approximation." This requirement
  implements it.
  1. **Fetched RocketSim's own `Car.cpp`** (`_UpdateJump`, matching
     `RB-PHYSICS-001-FR-058`/`FR-059`'s own real-implementation-file
     method, not just `RLConst.h`'s own constants) and confirmed the exact
     mechanism, not merely `JUMP_MIN_TIME`'s existence: `jumpTime <
     JUMP_MIN_TIME || (jumpPressed && jumpTime < JUMP_MAX_TIME)` gates
     whether the hold force applies at all this tick, with a separate
     `if (jumpTime < JUMP_MIN_TIME) totalJumpForce *=
     JUMP_PRE_MIN_ACCEL_SCALE;` (`0.62f`) scaling it down during that
     window — applied as a hard step-scale, not an interpolation, and
     regardless of whether the jump button is still held. The same
     source's own inline comment (`// TODO: Either move to RLConst or
     preferably don't use this system at all`) flags this as a stopgap its
     own authors consider provisional, not a deliberate permanent design
     choice — adopted anyway since it's still the real, currently-shipping
     behavior, the same standard `RB-PHYSICS-001-FR-042`'s own
     `box_vs_box` fallback finding already applied to an equally
     "never proven necessary" branch in Bullet's own reference source.
  2. **Confirmed both quantities transfer cleanly**, unlike most of
     `drive.rs`'s own torque-shaped placeholders (see the module doc
     comment's own "false precision" discussion): `JUMP_MIN_TIME` is a
     duration, and `JUMP_PRE_MIN_ACCEL_SCALE` is a dimensionless ratio
     applied to this port's own already-adopted `JUMP_HOLD_ACCELERATION`
     — neither is a torque or force calibrated against real Rocket
     League's own specific car mass/inertia tensor, the same category
     `RB-PHYSICS-001-FR-057`'s `MAX_CAR_ANGULAR_SPEED` and
     `RB-PHYSICS-001-FR-059`'s dodge-speed-scale ratios already cleared.
  3. **Implemented the mandatory window** by adding `drive::JUMP_MIN_TIME
     = 0.025` and `drive::JUMP_PRE_MIN_ACCEL_SCALE = 0.62`, and reworking
     `apply_driven_forces`'s own hold-acceleration check to derive the
     time elapsed since the ground-jump press as
     `JUMP_HOLD_MAX_DURATION - *jump_hold_time_remaining` rather than
     tracking a second, separate elapsed-time field — at rest (
     `jump_hold_time_remaining == 0.0`, meaning no ground jump is in
     flight) this derivation already reads as `JUMP_HOLD_MAX_DURATION`,
     comfortably past `JUMP_MIN_TIME`, so a car that never pressed jump
     never spuriously enters the mandatory branch, with zero new state and
     zero changes needed anywhere `jump_hold_time_remaining` is threaded
     (`PhysicsWorld`, every existing test call site). The hold-acceleration
     condition now applies (scaled during the mandatory window) whenever
     `elapsed < JUMP_MIN_TIME`, regardless of `input.jump`, or whenever
     `jump` is still held with time left — matching RocketSim's own gate
     exactly.
  - **Non-goals (this requirement).** Does not touch `JUMP_HOLD_MAX_DURATION`
    or `JUMP_HOLD_ACCELERATION`'s own values — both already confirmed by
    `RB-PHYSICS-001-FR-031`'s audit and unchanged here. Does not adopt
    `JUMP_ACCEL`'s own real magnitude beyond what `FR-031` already did;
    this requirement is scoped to the two-phase ramp's *shape* alone. Does
    not model real Rocket League's own further `_UpdateJump` nuances this
    same fetch surfaced but didn't need for this specific fix (e.g. how
    `jumpTime` itself resets across a fresh jump versus a double
    jump/dodge) — this port's own pre-existing `jump_hold_time_remaining`
    re-arming already handles that distinction correctly for this port's
    own simplified model, unaffected by this change. Does not touch the
    double jump, a dodge, or the wall jump — all three remain single fixed
    instantaneous impulses, as before.
  - **Acceptance criteria.** For the first `JUMP_MIN_TIME` seconds after a
    ground-jump press, `apply_driven_forces` applies
    `JUMP_HOLD_ACCELERATION * JUMP_PRE_MIN_ACCEL_SCALE` regardless of
    `input.jump`, including when jump is released immediately (a tap).
    Once that window has passed, releasing `jump` ends the extra
    acceleration immediately, exactly as before this requirement. While
    `jump` stays held past the mandatory window, the acceleration returns
    to full `JUMP_HOLD_ACCELERATION` for the remainder of
    `JUMP_HOLD_MAX_DURATION`, unchanged from before this requirement.
  - **Verification plan.** 3 new `rb_physics_bullet` unit tests
    (`jump_hold_acceleration_is_scaled_down_during_the_mandatory_pre_min_time_window`,
    `releasing_jump_within_the_mandatory_pre_min_time_window_does_not_immediately_stop_the_extra_acceleration`,
    `mandatory_pre_min_time_window_closes_on_schedule_even_when_jump_is_never_held`)
    pin the new mandatory-window behavior directly, including the exact
    scaled-acceleration magnitude. All 309 of `rb_physics_bullet`'s
    pre-existing tests (as of `FR-063`) pass unchanged — confirmed
    empirically, not just by inspection: every existing hold-window test's
    own release/expiry timing happens to fall at or after `JUMP_MIN_TIME`
    has already elapsed, so none exercises the new early-release-within-
    the-mandatory-window case this requirement adds — bringing the crate
    to 312.
- `RB-PHYSICS-001-FR-065` (real steering is a wheeled-vehicle raycast
  model, not a torque, with an inverted speed-vs-turning-ability curve —
  audit finding, documentation only; **superseded by `FR-082` step (a)**,
  which adopted the curve on the front wheels and removed `STEER_TORQUE`): `drive::STEER_TORQUE`'s own doc
  comment had no public reference at all; this requirement fetched
  RocketSim's real `Car.cpp` directly to check.
  1. **Fetched RocketSim's own `Car.cpp`** (`_UpdateWheels`, matching
     `RB-PHYSICS-001-FR-058`/`FR-059`/`FR-064`'s own
     real-implementation-file method) and found real Rocket League's
     steering isn't a direct yaw-torque model at all: a wheel's *steer
     angle* (not a torque) is set from a confirmed real
     `STEER_ANGLE_FROM_SPEED_CURVE` (`RLConst.h`, a piecewise-linear
     curve in radians), and that angled wheel's lateral tire friction —
     computed per-wheel by `btVehicleRL`, a custom extension of Bullet's
     own raycast vehicle system (`btDefaultVehicleRaycaster`), through a
     further confirmed `LAT_FRICTION_CURVE` slip-friction curve — is what
     actually turns the car. This port has no wheels, raycasting, or
     tire-slip model at all (the car is one rigid box), so this real
     mechanism can't be ported without a substantially larger
     architecture change — the same category `RB-PHYSICS-001-FR-063`
     already established for per-contact-pair-type restitution/friction.
  2. **Found the confirmed curve's own shape is the opposite of this
     port's own `speed_factor`.** Real Rocket League's maximum steering
     angle is highest at a standstill (`0.53356` rad ≈ 30.6° at 0 uu/s)
     and decreases sharply as speed rises (down to `0.03454` rad ≈ 2° at
     3000 uu/s) — a car can turn tightest from a stop, only gently at
     speed. This port's own `speed_factor` (`(car.linear_velocity.length()
     / MAX_CAR_SPEED).min(1.0)`) does the opposite: zero torque at a
     standstill, scaling *up* to full `STEER_TORQUE` at `MAX_CAR_SPEED` —
     a stark, directional mismatch, not merely an uncalibrated magnitude.
  3. **Corrected `drive::STEER_TORQUE`'s and `MAX_CAR_SPEED`'s own doc
     comments**, and the `speed_factor` call site's own inline comment, to
     state this finding directly.
  - **Non-goals (this requirement).** Does not implement a wheeled-vehicle
    raycast/tire-slip model — this port's single-rigid-box car has no
    wheels to raycast or slip-friction curves to apply; adopting the real
    mechanism for real would mean a substantially larger architecture
    change than a single documentation-audit requirement should take on,
    left as a candidate for a future, dedicated requirement. Does not
    reverse `speed_factor`'s own direction to match the real curve's
    shape: unlike `RB-PHYSICS-001-FR-058`'s throttle taper or `FR-059`'s
    dodge scale (direct multipliers on a force/impulse this port already
    applies the same way real Rocket League does), the real curve maps
    speed to a *wheel angle*, which real Rocket League then feeds through
    nonlinear tire-slip friction (dependent on wheelbase geometry and
    friction curves this port doesn't model at all) to produce the actual
    turning force — there's no principled way to carry even the curve's
    normalized shape onto this port's own direct-torque model. Reversing
    `speed_factor`'s direction without that transfer function would
    substitute one unconfirmed guess for another, not adopt a confirmed
    real value — the same reasoning that kept `RB-PHYSICS-001-FR-057`'s
    `AIR_CONTROL_TORQUE` and `FR-059`'s `DODGE_SPEED` base magnitude as
    placeholders despite a real reference existing for each. Does not
    change `STEER_TORQUE`'s own magnitude. Does not touch
    `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `drive::STEER_TORQUE`'s own doc comment
    states the confirmed real steering mechanism (a wheeled-vehicle
    raycast/tire-slip model, not a torque) and the confirmed real
    speed-vs-steering-angle curve's inverted shape relative to this
    port's own `speed_factor`, along with why neither is adopted.
  - **Verification plan.** No new tests (documentation-only, matching
    `RB-PHYSICS-001-FR-044`/`FR-060`/`FR-063`'s own precedent); all 312 of
    `rb_physics_bullet`'s pre-existing tests (as of `FR-064`) pass
    unchanged, confirming zero behavioral change.
- `RB-PHYSICS-001-FR-066` (real handbrake friction reduction is
  anisotropic, not a single uniform multiplier — audit finding,
  documentation only; **superseded by `FR-082` steps (a) and (b)**: step
  (a) made the handbrake the wheels' lateral factor and removed
  `HANDBRAKE_FRICTION_MULTIPLIER`, step (b) added the analog ramp and the
  longitudinal factor curve): `drive::HANDBRAKE_FRICTION_MULTIPLIER` had no
  public reference at all; this requirement fetched RocketSim's real
  `Car.cpp` directly to check, continuing the same `_UpdateWheels`
  investigation `RB-PHYSICS-001-FR-065` started.
  1. **Fetched RocketSim's own `Car.cpp`** (`_UpdateWheels`, matching
     `RB-PHYSICS-001-FR-058`/`FR-059`/`FR-064`/`FR-065`'s own
     real-implementation-file method) and found real Rocket League's
     handbrake friction reduction is genuinely anisotropic: two separate
     confirmed real curves, `HANDBRAKE_LAT_FRICTION_FACTOR_CURVE`
     (`RLConst.h`, a constant `0.1` factor at every speed) and
     `HANDBRAKE_LONG_FRICTION_FACTOR_CURVE` (`0.5` at a standstill,
     `0.9` at and above 1 uu/s — effectively a near-constant, barely-
     reduced `0.9` for any real driving speed), are applied to lateral
     and longitudinal tire friction independently, not one shared
     multiplier applied to both.
  2. **Found a striking coincidence that is not a confirmation.** This
     port's own pre-existing `HANDBRAKE_FRICTION_MULTIPLIER = 0.1`
     happens to match the real *lateral-only* factor exactly — but this
     port applies that same `0.1` to its own single isotropic
     `RigidBody.friction` scalar, which the ground-contact solver reads
     identically for every direction. Real Rocket League's own handbrake
     drift keeps a car's forward/backward grip almost intact (`x0.9`)
     while cutting sideways grip to a tenth (`x0.1`) — this port's own
     uniform `0.1` wrongly crushes longitudinal grip to a tenth too,
     understating real forward-momentum retention during a drift.
  3. **Corrected `drive::HANDBRAKE_FRICTION_MULTIPLIER`'s own doc
     comment**, the module doc comment's "Handbrake" paragraph, and the
     "commonly-cited constants" paragraph to state this finding directly.
  - **Non-goals (this requirement).** Does not implement anisotropic
    friction — this port's `solver::friction_directions` already computes
    two separate tangent directions per contact (since
    `RB-PHYSICS-001-FR-049`), but both directions currently read the same
    single combined-friction scalar when their row limits are computed;
    giving handbrake a genuinely different lateral-vs-longitudinal factor
    would mean threading a second, direction-specific friction
    coefficient through every one of `solver.rs`'s several row-limit
    call sites (`resolve_contacts`, `resolve_contacts_between`,
    `resolve_static_manifolds`, `resolve_dynamic_manifolds`,
    `resolve_manifolds`) plus a way for those call sites to know a
    specific body is currently handbraking — a substantially larger
    architecture change than a single documentation-audit requirement
    should take on, the same category `RB-PHYSICS-001-FR-063`/`FR-065`
    already established. Does not change
    `HANDBRAKE_FRICTION_MULTIPLIER`'s own value — its coincidental match
    to the real lateral factor is not grounds to keep it as-is or change
    it without also fixing the longitudinal side, which needs the
    architecture change above. Does not touch
    `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `drive::HANDBRAKE_FRICTION_MULTIPLIER`'s own
    doc comment states both confirmed real curves, the coincidental
    (not confirming) match to the real lateral factor, and why the
    anisotropic model isn't adopted.
  - **Verification plan.** No new tests (documentation-only, matching
    `RB-PHYSICS-001-FR-044`/`FR-060`/`FR-063`/`FR-065`'s own precedent);
    all 312 of `rb_physics_bullet`'s pre-existing tests (as of `FR-065`)
    pass unchanged, confirming zero behavioral change.
- `RB-PHYSICS-001-FR-067` (real Rocket League has no distinct wall-jump
  mechanic or constant at all — audit finding, documentation only):
  `drive::WALL_JUMP_HORIZONTAL_SPEED` had no public reference at all; this
  requirement fetched RocketSim's real `Car.cpp` directly to check, closing
  a thread `RB-PHYSICS-001-FR-031`'s original audit only briefly noted
  ("a wall jump reusing the plain jump impulse rather than its own faster
  speed") without confirming the exact mechanism.
  1. **Fetched RocketSim's own `Car.cpp`** (`_UpdateJump`, matching
     `RB-PHYSICS-001-FR-058`/`FR-059`/`FR-064`/`FR-065`/`FR-066`'s own
     real-implementation-file method) and found real Rocket League has no
     separate wall-jump mechanic — or constant — at all. `_UpdateJump`
     applies exactly one impulse, `GetUpDir() *
     mutatorConfig.jumpImmediateForce` (the same real value this port's
     own `JUMP_SPEED` already matches), gated only on `isOnGround`, itself
     defined purely by wheel-contact count (`numWheelsInContact >= 3`)
     with no floor-vs-wall distinction at all. A dedicated search of
     `RLConst.h` for any `WALL`-named constant found only an unrelated
     Heatseeker-mode threshold (`WALL_BOUNCE_CHANGE_Y_THRESH`).
  2. **Confirmed why the same impulse still ends up horizontal on a
     wall.** Since `RB-PHYSICS-001-FR-065` already established real Rocket
     League's cars ride on Bullet's own raycast vehicle system
     (`btVehicleRL`), a car driving on a wall has its own orientation
     continuously tipped to match that wall by ordinary wheel/suspension
     contact forces, the same way a real car tilts to match a ramp — so
     `GetUpDir()` (the car's own local up axis in world space) already
     points along the wall's outward normal by the time a wall jump fires,
     with no special-cased direction logic needed anywhere in
     `_UpdateJump`. Real Rocket League's "wall jump" is thus the
     *identical* single grounded-jump impulse, along whatever direction
     the car's own up axis currently points — never a distinct
     horizontal-plus-vertical composite with its own separate magnitude.
  3. **Corrected `drive::WALL_JUMP_HORIZONTAL_SPEED`'s own doc comment**,
     the module doc's own wall-jump section, and the module doc's
     "commonly-cited constants" paragraph (which had briefly noted the
     same underlying fact since `RB-PHYSICS-001-FR-031`'s original audit
     without the exact mechanism) to state this finding directly.
  - **Non-goals (this requirement).** Does not remove this port's own
    two-component composite impulse (a horizontal push-off along the
    wall's normal, stacked with the existing vertical `JUMP_SPEED`) —
    this port's car has no wheels, raycasting, or surface-tracking
    orientation system at all (the same architecture gap
    `RB-PHYSICS-001-FR-065` found for steering), so its own orientation
    doesn't automatically tip to match a touched wall the way a real car
    does. Applying only `JUMP_SPEED` straight up on a wall touch, as the
    confirmed real mechanism would otherwise suggest, would produce no
    push-off from the wall at all in this port's own model, defeating the
    entire point of a wall jump — the composite shape is a deliberate,
    necessary substitute for the missing orientation mechanism, not an
    unfilled calibration gap. Does not change
    `WALL_JUMP_HORIZONTAL_SPEED`'s own magnitude, which remains an
    uncalibrated placeholder. Does not touch `RB-PHYSICS-001-FR-005`'s
    real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `drive::WALL_JUMP_HORIZONTAL_SPEED`'s own doc
    comment states the confirmed real finding (no distinct wall-jump
    mechanic or constant; the same grounded-jump impulse applied along the
    car's own, wall-tipped up axis) and why this port's own two-component
    composite substitute isn't adopted away despite that.
  - **Verification plan.** No new tests (documentation-only, matching
    `RB-PHYSICS-001-FR-044`/`FR-060`/`FR-063`/`FR-065`/`FR-066`'s own
    precedent); all 312 of `rb_physics_bullet`'s pre-existing tests (as of
    `FR-066`) pass unchanged, confirming zero behavioral change.
- `RB-PHYSICS-001-FR-068` (real per-axis air-control torque ratio,
  implemented): `RB-PHYSICS-001-FR-031`'s own audit had already found real
  air-control torque/damping coefficients exist but didn't adopt them,
  since they're expressed as absolute torques calibrated against real
  Rocket League's own specific car mass/inertia tensor — the same "false
  precision" reasoning that kept `AIR_CONTROL_TORQUE`'s own magnitude a
  placeholder. This requirement fetched RocketSim's own `Car.cpp`
  (`_UpdateAirTorque`, the same file/technique
  `RB-PHYSICS-001-FR-058`/`FR-059`/`FR-064`/`FR-065`/`FR-066`/`FR-067`
  used) and found the real mechanism: `torque = pitch * dirPitch_right *
  CAR_AIR_CONTROL_TORQUE.x + yaw * dirYaw_up * CAR_AIR_CONTROL_TORQUE.y +
  roll * dirRoll_forward * CAR_AIR_CONTROL_TORQUE.z`, with `RLConst.h`
  confirming `CAR_AIR_CONTROL_TORQUE = Vec(130, 95, 400)` ("Angle order is
  PYR").
  1. **Confirmed the real mechanism is structurally identical to this
     port's own** — a direct per-axis torque scaled by analog stick input,
     unlike steering (`RB-PHYSICS-001-FR-065`, a wheeled-vehicle
     raycast/tire-slip model) or handbrake (`FR-066`, a friction split this
     port's architecture can't represent). Because the mechanism itself
     matches, the confirmed per-axis *ratio* — unlike the real *absolute*
     torque values, which the pre-existing "false precision" finding
     already ruled out — is adoptable the same way `RB-PHYSICS-001-FR-058`'s
     throttle taper and `FR-059`'s dodge scale ratios are: a direct
     multiplier on a torque this port already applies the same way real
     Rocket League does.
  2. **Adopted the confirmed real ratios (`95/130`, `400/130`), not the
     real absolute magnitudes.** Added `AIR_CONTROL_YAW_SCALE = 95.0 /
     130.0` and `AIR_CONTROL_ROLL_SCALE = 400.0 / 130.0`;
     `AIR_CONTROL_TORQUE` itself is redefined as *pitch's own* magnitude
     specifically (unchanged value, `1_000_000.0`, still an uncalibrated
     placeholder) rather than a flat value shared by all three axes.
  3. **Wired the scales into `apply_driven_forces`'s air-control block.**
     Yaw's torque is now `AIR_CONTROL_TORQUE * AIR_CONTROL_YAW_SCALE`; roll's
     is `AIR_CONTROL_TORQUE * AIR_CONTROL_ROLL_SCALE`. Pitch is unchanged.
  4. **Corrected doc comments** (the module-level air-control paragraph and
     "commonly-cited constants" paragraph, `AIR_CONTROL_TORQUE`'s own doc
     comment) that previously described the three axes as sharing one flat
     magnitude with no ratio modeled.
  - **Non-goals (this requirement).** Does not adopt RocketSim's own real
    absolute torque magnitudes (`130`, `95`, `400`) for this port's own
    axes — still independently uncalibrated, per the "ratio confirmed,
    magnitude not" split above, the same reasoning `RB-PHYSICS-001-FR-058`/
    `FR-059` already established. Does not adopt RocketSim's own
    `CAR_AIR_CONTROL_DAMPING` (`Vec(30, 20, 50)`) — this port has no
    per-axis air-control damping term at all, a separate, independent
    addition left for a future requirement (`RB-PHYSICS-001-FR-071` later
    characterized the full mechanism, and then adopted it).
    Does not adopt RocketSim's own
    `pitchTorqueScale` factor applied only to the pitch component in
    `_UpdateAirTorque` (an additional speed- or state-dependent scale this
    requirement's own fetch surfaced but didn't fully characterize) —
    scoped out to keep this requirement to the confirmed, fully-characterized
    per-axis ratio alone (`RB-PHYSICS-001-FR-070` later closed this thread).
    Does not touch `RB-PHYSICS-001-FR-005`'s
    real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `AIR_CONTROL_YAW_SCALE`/`AIR_CONTROL_ROLL_SCALE`'s
    own doc comments state the exact RocketSim citations. Full yaw input
    produces measurably less angular velocity than full pitch input (scaled
    by `95/130`, further modulated by this port's own per-axis moment of
    inertia); full roll input produces measurably more (scaled by
    `400/130`). All 312 pre-existing tests pass unchanged — none asserts
    cross-axis magnitude equality, only per-axis nonzero response and sign,
    an explicit zero-regression-risk property confirmed by inspection
    before implementation, not merely by the suite passing afterward.
  - **Verification plan.** 2 new `drive.rs` tests:
    `yaw_air_control_is_scaled_down_from_pitch_by_the_confirmed_real_ratio`
    and `roll_air_control_is_scaled_up_from_pitch_by_the_confirmed_real_ratio`,
    each computing the exact expected angular velocity in closed form from
    `AIR_CONTROL_TORQUE`/the new scale constant/`car().inv_inertia_world()`
    and asserting the actual post-step value matches within `1e-3`. All 314
    of `rb_physics_bullet`'s tests (312 pre-existing plus these 2 new ones)
    pass.
- `RB-PHYSICS-001-FR-069` (real dodge spin is a continuous per-axis torque
  over a fixed window, not an instantaneous kick — audit finding,
  documentation only): `RB-PHYSICS-001-FR-031`'s own original audit had
  already found the real mechanism is a *torque*, not a flat kick (via
  `RLConst.h`'s `FLIP_TORQUE_X = 260.f`/`FLIP_TORQUE_Y = 224.f` for
  `FLIP_TORQUE_TIME = 0.65f` seconds), but only had the constant
  declarations, not the exact application mechanism. This requirement
  fetched RocketSim's real `Car.cpp` directly to confirm it, continuing
  `RB-PHYSICS-001-FR-059`'s own investigation of `_UpdateDoubleJumpOrFlip`.
  1. **Fetched RocketSim's own `Car.cpp`** (`_UpdateDoubleJumpOrFlip` and
     `_UpdateAirTorque`, matching
     `RB-PHYSICS-001-FR-058`/`FR-059`/`FR-064`/`FR-065`/`FR-066`/`FR-067`/`FR-068`'s
     own real-implementation-file method) and found the exact mechanism:
     `_UpdateDoubleJumpOrFlip` stores a per-dodge relative torque direction
     (`flipRelTorque`) at the moment a flip begins; `_UpdateAirTorque` then
     applies `flipRelTorque * Vec(FLIP_TORQUE_X, FLIP_TORQUE_Y, 0)` as a
     continuous torque every step, gated by `isFlipping = hasFlipped &&
     flipTime < FLIP_TORQUE_TIME` — a hard cutoff at exactly `0.65`
     seconds, with no decay or ramp beforehand (constant magnitude the
     whole window, then an abrupt stop).
  2. **Confirmed a second, independent axis-shaped divergence**:
     `FLIP_TORQUE_X` (roll/left-right, `260`) and `FLIP_TORQUE_Y`
     (pitch/forward-backward, `224`) genuinely differ from each other —
     this port's own single shared `DODGE_ANGULAR_SPEED` doesn't model
     that difference either, on top of the instant-kick-vs-continuous-
     torque mismatch `FR-031` already found.
  3. **Corrected `drive::DODGE_ANGULAR_SPEED`'s own doc comment**, the
     module doc's own dodge section, and the "commonly-cited constants"
     paragraph, which had previously described this constant as having no
     public reference at all, contradicting `FR-031`'s own
     already-established finding.
  - **Non-goals (this requirement).** Does not implement the real
    continuous-torque-over-`FLIP_TORQUE_TIME` shape — the resulting spin
    rate depends on real Rocket League's own specific hitbox inertia
    tensor, which this port's placeholder car body doesn't match (the same
    "false precision" reasoning `RB-PHYSICS-001-FR-031` already
    established), and adopting the real shape for real, not just its
    constants, would also mean threading new per-car elapsed-flip-time
    state through `PhysicsWorld` the way `jump_hold_time_remaining`
    already does for the ground jump — a substantially larger redesign
    than a single documentation-audit requirement should take on,
    matching `RB-PHYSICS-001-FR-059`'s own Non-goals, which already
    flagged this exact redesign as out of scope. Does not change
    `DODGE_ANGULAR_SPEED`'s own magnitude. Does not touch
    `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
    Since `RB-PHYSICS-001-FR-080`, adopting this mechanism is scoped for
    implementation (the "false precision" objection above no longer holds:
    `FR-079` found the real torque is applied inertia-independently, so
    the spin rate does not depend on the hitbox inertia tensor at all —
    it pins to `CAR_MAX_ANG_SPEED`, which this port already has) — and
    its step (b) adopted it, removing `DODGE_ANGULAR_SPEED`.
  - **Acceptance criteria.** `drive::DODGE_ANGULAR_SPEED`'s own doc
    comment states the confirmed real mechanism (a continuous per-axis
    torque over a fixed 0.65s window with no decay, not an instantaneous
    shared kick) and why it isn't adopted.
  - **Verification plan.** No new tests (documentation-only, matching
    `RB-PHYSICS-001-FR-044`/`FR-060`/`FR-063`/`FR-065`/`FR-066`/`FR-067`'s
    own precedent); all 314 of `rb_physics_bullet`'s pre-existing tests
    (as of `FR-068`) pass unchanged, confirming zero behavioral change.
- `RB-PHYSICS-001-FR-070` (real flip-cancel is continuous, pitch-stick-driven,
  and pitch-axis-only, not jump-press-triggered and all-axis — audit
  finding, documentation only; adopted by `RB-PHYSICS-001-FR-080` step
  (c)): `RB-PHYSICS-001-FR-069`'s own fetch of
  `_UpdateAirTorque` surfaced a `pitchTorqueScale` factor scaling only the
  pitch component of air-control torque, flagged there as "an additional
  speed- or state-dependent scale this requirement's own fetch surfaced but
  didn't fully characterize" and scoped out. This requirement closes that
  thread by reading the rest of `_UpdateAirTorque` and the adjacent
  "Flip cancel check" block in `_UpdateDoubleJumpOrFlip`'s own call site.
  1. **Fetched RocketSim's own `Car.cpp`** (`_UpdateAirTorque`, matching
     `RB-PHYSICS-001-FR-058`/`FR-059`/`FR-064`/`FR-065`/`FR-066`/`FR-067`/
     `FR-068`/`FR-069`'s own real-implementation-file method) and found real
     Rocket League's actual flip-cancel mechanism, which this port's own
     `drive::apply_driven_forces` flip-cancel branch (`RB-PHYSICS-001-FR-016`)
     had labeled "matching real Rocket League" without having fetched the
     real mechanism to check: while still flipping (`isFlipping`), if the
     flip's own stored pitch-torque component (`flipRelTorque.y()`) is
     nonzero and `controls.pitch` is held in that same sign, real Rocket
     League sets `pitchScale = 1 - abs(controls.pitch)` and multiplies only
     `relDodgeTorque.y()` (the pitch-axis component) by it, every tick, for
     as long as the flip continues — a continuous, proportional, pitch-only
     reduction driven by *holding* the stick, not a discrete jump-press
     trigger. A sideways (roll-only) dodge has no pitch-torque component
     (`flipRelTorque.y() == 0`) at all, so this check never engages for
     it — real Rocket League cannot pitch-cancel a purely sideways dodge's
     spin.
  2. **Confirmed this port's own flip-cancel (`FR-016`) diverges on three
     independent axes from the real mechanism**: trigger (a fresh
     `ControllerInput.jump` press vs. continuously-held pitch input),
     magnitude (an outright zero of `RigidBody.angular_velocity` vs. a
     proportional `1 - abs(pitch)` scale applied only while pitch stays
     held), and scope (every dodge direction alike vs. only a dodge with a
     nonzero pitch-torque component).
  3. **Corrected the `drive` module's flip-cancel doc comment**, removing
     the inaccurate "matching real Rocket League" claim and replacing it
     with the confirmed real mechanism and why this port's simplification
     isn't upgraded to match it. Also added a forward citation from
     `RB-PHYSICS-001-FR-016`'s own entry.
  - **Non-goals (this requirement).** Does not implement real flip-cancel's
    actual mechanism: this port's dodge is `RB-PHYSICS-001-FR-069`'s own
    already-confirmed single flat angular-velocity kick, with no per-axis
    torque split to partially, continuously reduce — the same architecture
    gap `FR-069` found for the dodge's own spin applies identically here.
    Reproducing the real trigger shape (continuous stick-hold rather than a
    discrete press) and the real per-axis scope (pitch-only, direction-
    dependent) would both need the same continuous per-axis torque and
    elapsed-flip-time state `RB-PHYSICS-001-FR-059`'s own Non-goals already
    flagged as out of scope for the dodge itself. Does not change
    `apply_driven_forces`'s flip-cancel behavior (still a jump-press-
    triggered, all-axis, outright zero). Does not touch
    `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** The `drive` module's flip-cancel doc comment no
    longer claims to match real Rocket League's own mechanism; it instead
    states the confirmed real mechanism (continuous, pitch-stick-driven,
    pitch-axis-only, direction-restricted) and explains why this port's
    jump-press/all-axis/binary simplification remains as-is.
    `RB-PHYSICS-001-FR-016`'s own entry carries a forward citation to this
    requirement.
  - **Verification plan.** No new tests (documentation-only, matching
    `RB-PHYSICS-001-FR-044`/`FR-060`/`FR-063`/`FR-065`/`FR-066`/`FR-067`/
    `FR-069`'s own precedent); all 314 of `rb_physics_bullet`'s pre-existing
    tests (as of `FR-069`) pass unchanged, confirming zero behavioral
    change.
- `RB-PHYSICS-001-FR-071` (real air-control damping — audit finding, now
  implemented, replacing the placeholder landing auto-orientation assist):
  `RB-PHYSICS-001-FR-068`'s own Non-goals had already found RocketSim's
  `CAR_AIR_CONTROL_DAMPING = Vec(30, 20, 50)` exists but left it as "a
  separate, independent addition left for a future requirement" without
  examining the mechanism behind it. The original pass closed that thread
  as documentation; `RB-PHYSICS-001-FR-080` step (c) then pinned the
  mechanism to real data and made it the next gap, and this pass adopts
  it.
  1. **The mechanism** (read from `_UpdateAirTorque`, the same fetch
     `RB-PHYSICS-001-FR-070` used): for each axis, real air control
     computes `(angular velocity along that axis) *
     CAR_AIR_CONTROL_DAMPING[axis] * (1 - abs(analog input on that axis))`
     — pitch's own input term additionally multiplied by
     `pitchTorqueScale` (`FR-070`/`FR-080`'s pitch lock), roll's carrying
     no input factor at all — and subtracts the combined damping vector
     from the stick torque inside the same `invInertia.inverse() * (torque
     - damping) * CAR_TORQUE_SCALE` expression. Releasing the stick on an
     axis gives full damping strength there, continuously bleeding off any
     existing spin; holding it fully zeroes the damping, granting full
     torque authority with no resistance. It is inertia-independent for the
     same reason the torque is (`FR-079`), so the original "false
     precision" objection never applied.
  2. **What the real capture says.** `FR-080` step (c)'s tick-by-tick fit
     of the isolated `dodge-derailment` fixture's 77 in-flip ticks — run
     forward from each recorded orientation, spin, and input — reproduces
     the next tick's `ω` to `0.0025` rad/s rms (the recording's rounding
     floor) only with this damping active, mid-flip included; without it,
     `0.082`. After the window the recording's spin decays at `≈3.9` rad/s
     per second from `|ω| = 5.5` with pitch locked and the stick neutral
     on yaw: exactly `5.5 · 30 · CAR_TORQUE_SCALE = 15.8` rad/s² per unit
     of spin, i.e. this coefficient's own rate.
  3. **Implemented.** New `drive::AIR_CONTROL_PITCH_DAMPING = 30.0`,
     `AIR_CONTROL_YAW_DAMPING = 20.0`, `AIR_CONTROL_ROLL_DAMPING = 50.0`
     (public, named for the axis they damp, citing
     `CAR_AIR_CONTROL_DAMPING`) and `drive::air_control_damping(car,
     effective_pitch, yaw) -> Vec3`, the damping angular acceleration:
     each body-axis component of `angular_velocity`, times its
     coefficient, times `1 - |effective_pitch|` (right axis) or `1 - |yaw|`
     (up axis), summed, negated, and scaled by `CAR_TORQUE_SCALE`.
     `apply_driven_forces` applies it via `apply_angular_acceleration`
     every airborne step, right after the three stick torques and inside
     the same block, so it runs during a flip too. `CAR_TORQUE_SCALE` is
     now public for tests.
  4. **The placeholder landing assist is retired.** `FR-018`'s
     `LANDING_AUTO_UPRIGHT_TORQUE` nudge (an airborne, input-free restoring
     torque toward world up) was an invented stand-in for "eventually
     right yourself before landing"; `FR-060` had established real Rocket
     League has no such mechanic (its auto-flip and auto-roll are grounded
     and input-gated), and what actually makes a tumbling airborne car
     settle there is this damping. With the real mechanism in, the
     fixture was measured both ways: with the nudge kept,
     `cars.mean_position_distance` `≈243` uu / mean rotation `0.83` rad;
     without it `≈240` uu / `0.77` rad — a wash inside the airborne phase
     (the nudge never fires under active stick input) and marginally
     better overall. The constant, its block, and its four `drive.rs`
     tests and one `world.rs` test are removed; the module doc's
     "Landing auto-orientation assistance" section is replaced by "No
     airborne self-righting", and `FR-018` is marked superseded.
  - **Real-data effect, measured alone.** Isolated fixture, whole run:
    `cars.mean_position_distance` `≈237` → `≈240` uu, mean rotation `1.51`
    → `0.77` rad, mean velocity `≈254` → `≈337` uu/s, max position `≈459`
    → `≈791` uu; `mean_ball_distance` unchanged at `≈730`. The shape is
    what matters: the rotation gap now stays within `0.03`–`0.10` rad from
    the dodge (`t ≈ 4.32` s) through the flip window *and* the whole
    post-window decay to `t ≈ 5.52` s, with the velocity gap flat at
    `≈90`–`113` uu/s — the entire airborne phase of the fixture matches
    the recording. The whole-run position and velocity means went slightly
    *up* because the divergence now starts at the landing (`t ≈ 5.57` s:
    velocity gap `113` → `192` uu/s, then `≈800` at `t ≈ 5.77` s) and the
    now-correctly-oriented car's grounded phase (landing contact, steering
    — `FR-065`'s known placeholder — and the wall/curve interactions that
    follow) diverges differently from step (c)'s wrongly-oriented one,
    which had happened to bounce closer to the recording for the last
    second. That grounded phase is the next domain; nothing airborne is
    left in this fixture.
  - **Tests.** `rb_physics_bullet` stays at 350: 4 assist tests removed, 4
    added (a neutral-stick spin bleeds at exactly `1 - coefficient ·
    CAR_TORQUE_SCALE · dt` per axis; a fully held stick removes its own
    axis' damping for pitch and yaw but not roll; damping acts along the
    car's own axes, not the world's; none while grounded), 1 `world.rs`
    assist test replaced by a tumbling car settling within 2 s, and 19
    existing flip/cancel tests re-pinned with the pre-step damping folded
    into their exact per-tick expectations (a new `neutral_damping_step`
    helper). Full workspace `fmt`/`clippy`/`test` green (411 tests);
    `rb_verify_cli`'s ratchet holds at `< 250` uu.
  - **Non-goals (this requirement).** Does not implement real auto-flip or
    auto-roll (`FR-060`'s Non-goals stand). Does not gate air control on
    wheel contact the way RocketSim's `numWheelsInContact == 0` does (this
    port has no wheels; a wall touch leaves air control live). Does not
    take on the grounded-phase divergence the fixture now exposes. Does
    not touch `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer
    blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** A spinning airborne car with a neutral stick
    loses spin at `30`/`20`/`50 · CAR_TORQUE_SCALE` per second about its
    right/up/forward axes; a fully held pitch or yaw stick removes that
    axis' damping while roll's stays; the damping is active during a flip
    and under the post-flip pitch lock (at full pitch strength there);
    `LANDING_AUTO_UPRIGHT_TORQUE` no longer exists; the isolated fixture's
    rotation gap stays under `0.1` rad through the whole airborne phase;
    full workspace `fmt`/`clippy`/`test` green with the ratchet holding.
  - **Verification plan.** The new and re-pinned tests above, the
    re-measured `--self` / `--self-growth 0.05` numbers recorded in
    `PROJECT-STATUS.md`, and the ratchet.
- `RB-PHYSICS-001-FR-072` (normalized diagonal-dodge direction, implemented):
  `RB-PHYSICS-001-FR-059`'s own Non-goals had already found and flagged a
  genuine behavioral gap — this port's dodge summed each axis' own
  full-strength `(pitch, roll)` contribution independently, so a diagonal
  dodge (both axes held) came out `sqrt(2)`-ish times faster than an
  axis-aligned one, unlike real Rocket League's own normalized direction —
  "a separate, independent behavioral question this requirement doesn't
  take on."
  1. **Fetched RocketSim's own `Car.cpp`** (`_UpdateDoubleJumpOrFlip`,
     matching this port's own established real-implementation-file method)
     and found the exact real mechanism: `dodgeDir = btVector3(-controls.pitch,
     controls.yaw + controls.roll, 0)`, then `dodgeDir.safeNormalized()` —
     normalized to unit length *before* `FLIP_INITIAL_VEL_SCALE` and the
     further per-axis forward/backward/side speed scaling
     (`dodge_speed_scale`/`dodge_pitch_is_backward`, `FR-059`'s own already-
     adopted finding) are applied.
  2. **Confirmed this is a pure geometric operation this port's own model
     represents exactly** — unlike a wheeled-vehicle/tire-slip model or a
     continuous-torque timing state this port's architecture can't
     represent, normalizing a 2D direction vector needs no new machinery,
     so it transfers cleanly the same way `FR-058`/`FR-059`/`FR-068`'s own
     adopted ratios do, regardless of `DODGE_SPEED`'s own uncalibrated base
     magnitude.
  3. **Added `drive::normalize_dodge_direction(pitch, roll) -> (f32, f32)`**,
     a small pure helper normalizing the combined `(pitch, roll)` direction
     to unit length (returning `(0.0, 0.0)` for zero input), and wired it
     into both the ground-dodge and wall-jump-dodge code paths in
     `apply_driven_forces`: the existing per-axis `DODGE_DEADZONE` trigger
     checks and `dodge_pitch_is_backward`'s own sign classification are
     unchanged (both still read the *raw* stick values), but the magnitude
     each axis contributes to `DODGE_SPEED`/`DODGE_ANGULAR_SPEED` now comes
     from the normalized pair instead of the raw one.
  4. **Deliberately kept this port's own sign convention** (`dodge_pitch`
     positive means forward) rather than the reference's own negated
     `-controls.pitch`, and did **not** fold in real yaw input's own
     contribution to `dodgeDir` (`controls.yaw + controls.roll`) — this
     port's dodge direction stays pitch/roll only, `FR-059`'s own Non-goals'
     own separate, pre-existing simplification.
  5. **Updated the two existing diagonal-dodge tests**
     (`a_diagonal_dodge_combines_pitch_and_roll`,
     `a_diagonal_wall_jump_dodge_combines_pitch_and_roll`) to assert the
     new, correct per-axis magnitude (`DODGE_SPEED / sqrt(2)` for an equal
     pitch/roll split, with the total magnitude still matching an
     axis-aligned dodge's own `DODGE_SPEED`) and added 3 new tests directly
     exercising `normalize_dodge_direction` (a single-axis input is
     unaffected; a diagonal input normalizes to unit length; zero input
     stays zero).
  - **Non-goals (this requirement).** Does not adopt `DODGE_SPEED`'s own
    real base magnitude (`FLIP_INITIAL_VEL_SCALE = 500.f`) — still
    independently uncalibrated, unaffected by this requirement
    (`RB-PHYSICS-001-FR-080` step (a) later adopted it). Does not
    fold real yaw input into the dodge direction. Does not adopt
    RocketSim's own continuous torque-over-`FLIP_TORQUE_TIME` spin model —
    `DODGE_ANGULAR_SPEED` remains a single instantaneous kick, scaled by
    the same normalized direction as the linear impulse but still an
    architecture mismatch `RB-PHYSICS-001-FR-069`'s own Non-goals already
    established (`RB-PHYSICS-001-FR-080` step (b) later adopted it; the
    normalized direction this requirement introduced is exactly what
    that flip's `rel_torque` is built from). Does not touch `RB-PHYSICS-001-FR-005`'s real-data
    calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** A diagonal dodge (both `pitch` and `roll` at or
    above `DODGE_DEADZONE`) produces the same total linear-impulse
    magnitude as an axis-aligned dodge in the same direction, matching real
    Rocket League's own confirmed normalized-direction mechanism. A
    pure-single-axis dodge (the other axis exactly zero) is bit-for-bit
    unaffected.
  - **Verification plan.** `a_diagonal_dodge_combines_pitch_and_roll` and
    `a_diagonal_wall_jump_dodge_combines_pitch_and_roll` updated to assert
    the corrected per-axis and total magnitudes; 3 new tests pin
    `normalize_dodge_direction`'s own behavior directly. All 314
    pre-existing tests (as of `FR-071`) pass with these 2 updated plus 3
    new, bringing the crate to 317.
- `RB-PHYSICS-001-FR-073` (fold yaw into the dodge direction, implemented):
  `RB-PHYSICS-001-FR-059`'s own Non-goals (and `FR-072`'s own doc comment)
  had already found and flagged a genuine behavioral gap — this port's
  dodge/wall-jump-dodge direction reads only `pitch`/`roll` stick input,
  never `yaw`, even though real Rocket League's own `dodgeDir` combines
  `yaw + roll` for its horizontal-axis component.
  1. **Confirmed real RocketSim's exact mechanism** (`Car.cpp`,
     `_UpdateDoubleJumpOrFlip`): `dodgeDir = btVector3(-controls.pitch,
     controls.yaw + controls.roll, 0)`, normalized as `FR-072` already
     found; a dodge is cancelled entirely only when *both*
     `abs(controls.yaw + controls.roll) < 0.1` and `abs(controls.pitch) <
     0.1` (not adopted here — see Non-goals below). `controls.yaw` appears
     nowhere else in the function; it only ever feeds `dodgeDir`.
  2. **Confirmed this needs no new machinery**: unlike a wheeled-vehicle
     model or a continuous-torque timing state, this port already reads
     `input.yaw` in the same function, for air control (see
     `apply_driven_forces`'s existing `let yaw = input.yaw.unwrap_or(0.0)…`
     line) — folding it into the dodge's own roll-axis stick value is a
     pure additive combination of an already-available input, not a new
     capability, the same kind of "pure operation, no new architecture"
     transfer `FR-058`/`FR-059`/`FR-068`/`FR-072`'s own adopted findings
     share.
  3. **Changed both dodge call sites** in `apply_driven_forces` (the ground
     double-jump-dodge branch and the wall-jump-dodge branch): `dodge_roll`/
     `wall_roll` are now `input.roll.unwrap_or(0.0).clamp(-1.0, 1.0) +
     input.yaw.unwrap_or(0.0).clamp(-1.0, 1.0)` (each individually clamped
     first, matching how air control already clamps pitch/yaw/roll
     separately), rather than `input.roll` alone. This combined value feeds
     the existing `DODGE_DEADZONE` trigger check, `normalize_dodge_direction`,
     and the `DODGE_SPEED`/`DODGE_ANGULAR_SPEED` scaling — all unchanged
     otherwise. `dodge_pitch_is_backward`'s own sign check still reads raw
     `pitch` only, unaffected.
  4. **Updated `normalize_dodge_direction`'s own doc comment** (its "not
     adopted: yaw isn't folded in" note was now stale) and the module doc's
     dodge paragraph to state the finding, and added a forward citation from
     `RB-PHYSICS-001-FR-059`'s own Non-goals bullet.
  5. **Added 3 new tests**: `a_yaw_only_press_fires_a_sideways_dodge_like_roll`
     (a pure yaw stick nudge, no roll held, fires the same sideways dodge a
     roll-only press would), `yaw_and_roll_combine_in_the_dodge_direction`
     (equal-and-opposite yaw and roll cancel to no sideways dodge, falling
     back to a plain double jump), and
     `a_yaw_only_press_fires_a_sideways_wall_jump_dodge_like_roll` (the same
     fold-in on the wall-jump-dodge path).
  - **Non-goals (this requirement).** Does not adopt RocketSim's own
    all-or-nothing cancellation check (`abs(yaw + roll) < 0.1 &&
    abs(pitch) < 0.1` zeroes the *entire* dodge direction) in place of this
    port's own independent per-axis `DODGE_DEADZONE` trigger — framed here
    as "a real but separate architectural difference... left for a future
    requirement if it turns out to matter", a framing
    `RB-PHYSICS-001-FR-075` later found was itself wrong: once this
    requirement's own yaw fold-in is in place, the two triggers are the
    same boolean decision, not a genuine architectural difference.
    Does not adopt RocketSim's own post-normalization small-component
    zeroing (`abs(x) < 0.1` on each *normalized* direction component,
    confirmed during `FR-072`'s own investigation) — mis-scoped here as "a
    separate, independent simplification"; it is actually the same kind of
    pure post-processing step `normalize_dodge_direction` already performs,
    needing no new machinery (`RB-PHYSICS-001-FR-074` later closed this
    thread with a genuine fix, correcting that mis-scoping). Does not adopt
    `DODGE_SPEED`'s own real base magnitude, still independently
    uncalibrated. Does not touch `RB-PHYSICS-001-FR-005`'s real-data
    calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** A dodge or wall-jump-dodge press with only
    `yaw` held (no `roll`) fires the same sideways dodge a roll-only press
    would. Equal-and-opposite `yaw` and `roll` cancel to no sideways
    contribution. A pitch-only dodge (no `roll`/`yaw`) is bit-for-bit
    unaffected.
  - **Verification plan.** 3 new tests
    (`a_yaw_only_press_fires_a_sideways_dodge_like_roll`,
    `yaw_and_roll_combine_in_the_dodge_direction`,
    `a_yaw_only_press_fires_a_sideways_wall_jump_dodge_like_roll`) added;
    all 317 pre-existing tests (as of `FR-072`) pass unchanged, bringing the
    crate to 320.
- `RB-PHYSICS-001-FR-074` (snap a near-axis-aligned dodge to a pure single
  axis, implemented): `RB-PHYSICS-001-FR-073`'s own Non-goals had flagged
  RocketSim's post-normalization small-component zeroing as "a separate,
  independent simplification" left open — a mis-scoping this requirement
  corrects: it is not a separate mechanism at all, but a further pure
  post-processing step on the exact normalized `(pitch, roll)` pair
  `normalize_dodge_direction` already computes.
  1. **Re-confirmed real RocketSim's exact mechanism** (`Car.cpp`,
     `_UpdateDoubleJumpOrFlip`, same fetch technique as `FR-072`/`FR-073`):
     after `dodgeDir = dodgeDir.safeNormalized()`, `if (abs(dodgeDir.x()) <
     0.1f) dodgeDir.x() = 0; if (abs(dodgeDir.y()) < 0.1f) dodgeDir.y() =
     0;` — applied to the already-normalized direction, not the raw stick
     input, and not re-normalized afterward.
  2. **Confirmed this needs no new machinery**: like normalization itself
     (`FR-072`), zeroing a small component of an already-computed pair is a
     pure post-processing step this function's own existing return value
     already supports — no new state, no new input, no architecture this
     port lacks. The same "pure operation, no new architecture" transfer
     `FR-058`/`FR-059`/`FR-068`/`FR-072`/`FR-073`'s own adopted findings
     share.
  3. **Added `drive::DODGE_DIRECTION_SNAP_THRESHOLD: f32 = 0.1`** (a
     distinct named constant from `DODGE_DEADZONE`, despite sharing the
     same real value, since they serve different real purposes — a raw-
     stick trigger threshold vs. a post-normalization direction-snap
     threshold — and could in principle be recalibrated independently).
     `normalize_dodge_direction` now zeroes either returned component whose
     magnitude falls below this threshold, after normalizing, matching
     RocketSim's own order of operations exactly.
  4. **Effect**: a dodge stick input that is nearly, but not quite,
     axis-aligned (e.g. a diagonal whose secondary axis is just barely
     above `DODGE_DEADZONE`) now snaps to a clean single-axis dodge instead
     of producing a tiny, physically negligible perpendicular component —
     matching real Rocket League's own behavior for imprecise stick
     centering. Both call sites in `apply_driven_forces` already route
     through `normalize_dodge_direction`, so no call-site changes were
     needed.
  5. **Updated `normalize_dodge_direction`'s own doc comment**, the module
     doc's dodge paragraph, and added a forward citation from
     `RB-PHYSICS-001-FR-073`'s own Non-goals bullet correcting its
     "separate, independent simplification" mis-scoping.
  6. **Added 2 new tests**:
     `normalize_dodge_direction_snaps_a_near_axis_aligned_input_to_a_pure_axis`
     (pitch=1.0, roll=0.05 — the tiny roll component snaps to exactly
     zero) and `normalize_dodge_direction_does_not_snap_a_clearly_diagonal_input`
     (pitch=1.0, roll=0.5 — both components stay well above the threshold,
     unaffected).
  - **Non-goals (this requirement).** Does not adopt RocketSim's own
    all-or-nothing cancellation check (`abs(yaw + roll) < 0.1 &&
    abs(pitch) < 0.1` zeroes the entire dodge direction) in place of this
    port's own independent per-axis `DODGE_DEADZONE` trigger, framed here
    as "a genuine architectural difference... left for a future
    requirement if it turns out to matter" —
    `RB-PHYSICS-001-FR-075` later found this framing itself was wrong: once
    `FR-073`'s own yaw fold-in is in place, this port's independent
    per-axis trigger and RocketSim's own combined all-or-nothing check are
    the *same* boolean decision, not a genuine architectural difference at
    all. Does not adopt `DODGE_SPEED`'s own real base magnitude,
    still independently uncalibrated. Does not touch
    `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** A dodge whose secondary stick axis, once the
    combined `(pitch, roll)` direction is normalized, falls below
    `DODGE_DIRECTION_SNAP_THRESHOLD` in magnitude now fires as a pure
    single-axis dodge (that component exactly zero) instead of a slightly
    diagonal one. A clearly diagonal dodge (both normalized components at
    or above the threshold) is unaffected.
  - **Verification plan.** 2 new tests pin
    `normalize_dodge_direction`'s own snapping behavior directly at both
    sides of the threshold; all 320 pre-existing tests (as of `FR-073`)
    pass unchanged, bringing the crate to 322.
- `RB-PHYSICS-001-FR-075` (confirm `DODGE_DEADZONE` matches RocketSim's own
  real cancellation threshold — audit finding, documentation only): this
  spec's own Open Questions section had claimed `DODGE_DEADZONE` "still has
  no public reference at all... so it may be off by a large factor," and
  `RB-PHYSICS-001-FR-074`'s own Non-goals (mirroring `FR-073`'s own
  earlier, identical claim) had separately framed RocketSim's all-or-
  nothing dodge-cancellation check as "a real but separate architectural
  difference" from this port's independent per-axis `DODGE_DEADZONE`
  trigger. Both were wrong.
  1. **Re-examined RocketSim's own confirmed `_UpdateDoubleJumpOrFlip`
     cancellation check** (already fetched and quoted verbatim during
     `FR-072`/`FR-073`/`FR-074`'s own investigations, not a fresh fetch):
     `if (abs(controls.yaw + controls.roll) < 0.1f && abs(controls.pitch) <
     0.1f) { dodgeDir = {0,0,0}; }` — by De Morgan's law, a dodge fires iff
     `abs(yaw + roll) >= 0.1 || abs(pitch) >= 0.1`.
  2. **Derived that this port's own trigger is the same boolean expression**:
     since `RB-PHYSICS-001-FR-073`, this port's own `dodge_roll`/`wall_roll`
     already equal `roll + yaw` combined, and this port's trigger is
     `dodge_pitch.abs() > DODGE_DEADZONE || dodge_roll.abs() >
     DODGE_DEADZONE`. Given `DODGE_DEADZONE == 0.1`, this is the identical
     decision to RocketSim's own real one, differing only in a strict (`>`)
     vs. non-strict (`>=`) comparison at the exact boundary value — an
     unobservable floating-point edge case, not a behavioral or
     architectural difference. `0.1` was already the real value; it simply
     hadn't been confirmed as such.
  3. **Corrected `DODGE_DEADZONE`'s own doc comment** (previously "Not a
     physics constant and not derived from any Rocket League value —
     purely an input-processing threshold"), the module doc's dodge
     paragraph, this spec's own stale Open Questions bullet, and added
     forward citations from `FR-073`'s and `FR-074`'s own Non-goals
     bullets correcting their "separate architectural difference" framing.
  4. **No code change**: this port's dodge trigger already matched real
     Rocket League exactly (given `FR-073`'s own prior yaw fold-in); this
     requirement corrects the record, not the behavior.
  - **Non-goals (this requirement).** Does not change any production
    behavior — `DODGE_DEADZONE`'s own value (`0.1`) and the trigger logic
    around it are unchanged; only doc comments and this spec's own prose
    are corrected. Does not adopt `DODGE_SPEED`'s own real base magnitude
    or any other still-uncalibrated constant. Does not touch
    `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** `DODGE_DEADZONE`'s own doc comment, this
    spec's Open Questions section, and `FR-073`'s/`FR-074`'s own Non-goals
    bullets no longer describe this constant as unreferenced or its
    trigger architecture as diverging from real Rocket League.
  - **Verification plan.** Documentation-only; no new tests. All 322
    pre-existing tests (as of `FR-074`) pass unchanged.
- `RB-PHYSICS-001-FR-076` (implemented): `rb_physics_bullet` gains the
  capability to seed a `PhysicsWorld` from a recorded `PhysicsFrame` and
  simulate it forward using a recorded per-tick controller-input sequence,
  producing a candidate `Vec<PhysicsFrame>` — the missing piece
  `world::simulate`'s own doc comment already named: "Once `RB-VERIFY-002`
  capture data exists, this signature grows an `inputs` parameter rather
  than staying input-free." That capture data now exists (`PHASE-0-EXIT`,
  closed). This FR is the prerequisite plumbing `FR-005`'s real-data
  calibration and `RB-VERIFY-003`'s own Non-goals ("running a candidate
  physics engine to generate its output — that's `RB-PHYSICS-001`'s
  composition-root responsibility") both assume exists but didn't yet.
  - **What shipped, and where it diverged from the original scoping.**
    1. **`body::CAR_MASS`/`CAR_HALF_EXTENTS` and `RigidBody::standard_car`**,
       plus **`body::BALL_RADIUS`/`BALL_MASS` and `RigidBody::standard_ball`**
       (the scoping's own item 1 anticipated `standard_car` mirroring
       `standard_ball`, but `standard_ball` didn't exist yet either — both
       were added together). Fetched directly from RocketSim's own real
       source: `src/RLConst.h`'s `CAR_MASS_BT = 180.f` (already matches
       this crate's own long-standing `180.0` test placeholder — an
       accurate placeholder, confirmed only now) and `BALL_MASS_BT =
       CAR_MASS_BT / 6.f = 30.0` (a **new** finding — every existing test
       uses `1.0`), and `src/Sim/Car/CarConfig/CarConfig.cpp`'s
       `CAR_CONFIG_OCTANE.hitboxSize = { 120.507f, 86.6994f, 38.6591f }`
       (full size; halved to half-extents `(60.2535, 43.3497, 19.32955)`)
       — also a **new** finding, and a substantial one: this crate's own
       long-standing `car_box` test placeholder
       (`Vec3::new(60.0, 30.0, 18.0)`) has the right length and height
       but a width (`30.0` half-extent) off Octane's real `43.3497` by
       ~31% (~44% on the full-width comparison). `BALL_RADIUS` (`93.15`)
       reuses `FR-036`'s own already-confirmed value rather than
       re-deriving it — RocketSim's own `BALL_REST_Z` comment ("greater
       than ball radius because of arena mesh collision margin") means
       93.15 isn't literally RocketSim's own collision radius either, but
       re-litigating `FR-036`'s own settled, dedicated-FR choice is out of
       this FR's scope. Deliberately **not** corrected at any existing
       `car_box`/`sphere`/`ball` call site across this crate's own tests
       — only the four new constants and two new constructors carry the
       corrected values; retuning every dimension/mass-dependent test's
       own expectations to match is a dedicated calibration FR of its
       own, matching `FR-036`'s own precedent, out of this FR's scope.
       Car restitution/friction stay at the generic `0.5`/`0.5`
       placeholder (not "confirmed or flagged" as a single number the way
       the scoping's own item 1 first framed it) — `FR-063` already found
       real Rocket League has no single generic car restitution/friction
       at all, only per-contact-pair overrides this crate's architecture
       can't represent, so inventing one number here would be exactly the
       "false precision" `FR-031`/`FR-040` already refused.
    2. **`PhysicsWorld::from_frame`**, exactly as scoped: seeds ball and
       every car's position/rotation/velocity/angular_velocity directly
       from the frame, `boost_amount` via the existing `set_car_boost`,
       and shape/mass/material via (1)'s new constructors. One addition
       the original scoping missed and had to be caught during
       implementation: `elapsed_secs` (private, so only settable from
       inside `impl PhysicsWorld`) is seeded to `frame.timestamp_secs`,
       not left at `PhysicsWorld::new`'s own default `0.0` — without this,
       every candidate frame `simulate_recorded` produces would land tens
       of seconds away from the real capture's own absolute clock, and
       `rb_domain::divergence::score`'s nearest-timestamp alignment would
       never match anything up (see Data/state and invariants).
    3. **`world::simulate_recorded`**, a sibling function to `simulate`
       (not a breaking signature change, since every existing input-free
       call site — this crate's own tests included — has no recorded
       input to supply): for each consecutive pair of recorded frames,
       applies the *earlier* frame's own per-car `ControllerInput` via
       `set_car_input`, then steps by that pair's own `timestamp_secs`
       delta — exactly the "derive `dt` from the recording's own spacing,
       not a hardcoded rate" the scoping called for, since no confirmed
       real Rocket League tick-rate constant exists anywhere in this
       project. One addition the scoping didn't anticipate: a recorded
       car whose `player_id` doesn't index into the seeded `world.cars`
       (more recorded cars than the seed frame carried) is silently left
       undriven that tick rather than panicking — real mid-capture car
       joins/leaves are unexercised territory this plumbing doesn't need
       to solve yet (see Non-goals).
  - **Non-goals (this requirement).** Does not add setters for the
    per-car runtime state `PhysicsWorld` tracks but never exposes
    (`car_jump_held`, `car_double_jump_available`,
    `car_jump_hold_time_remaining`, `car_dodge_flip_active` — all
    initialized only to fixed defaults by `with_car`); seeding a
    simulation therefore always assumes those defaults (not held,
    double-jump available, zero hold time, no dodge in progress), which
    is only accurate if the seed frame is a genuinely neutral, grounded
    moment. Choosing *which* frame in a capture to seed from — and
    whether a non-neutral seed frame needs those setters after all — is
    `FR-077`'s concern, not this one. Does not wire this capability into
    `rb_verify_cli` or run it against any real capture (`FR-077`). Does
    not calibrate any constant based on a resulting score (a later FR,
    once `FR-077` produces a first real number). Does not correct
    `car_box`'s own existing call sites to the newly-confirmed real
    hitbox — a separate, larger calibration FR (see above).
  - **Acceptance criteria.** A `PhysicsWorld` can be constructed from a
    recorded `PhysicsFrame` and stepped forward using a recorded sequence's
    own controller input and timestamp spacing, producing a
    `Vec<PhysicsFrame>` the existing `rb_domain::divergence::score` can
    consume unmodified, with the candidate's own timestamps landing on the
    same absolute clock the recording used. Met.
  - **Verification plan.** 13 new unit tests against hand-built values (no
    real capture needed for correctness, mirroring
    `rb_capture_ingest::wire`'s own hand-built-value testing precedent):
    6 in `body.rs` (`standard_car`/`standard_ball`'s mass/shape/material,
    and that both new hitbox/mass constants deliberately differ from this
    crate's own existing test placeholders) and 7 in `world.rs`
    (`from_frame`'s ball/car/clock seeding; `simulate_recorded`'s frame
    count, per-pair `dt` derivation — a longer recorded interval produces
    proportionally more fall than a shorter one, which only holds if `dt`
    is read per-pair rather than reused from the first — actual driving
    from recorded input, and the out-of-bounds-car skip). All 335
    `rb_physics_bullet` tests pass (322 pre-existing + 13 new); full
    workspace `cargo fmt`/`clippy`/`test` all green. A real-capture run is
    `FR-077`'s job, not this one's.
- `RB-PHYSICS-001-FR-077` (implemented, verified against a real capture):
  `rb_verify_cli` gains a new composition path — score a real BakkesMod
  capture's own recorded outcome against a candidate trajectory simulated
  from that *same* capture's recorded input via `FR-076`'s new
  `rb_physics_bullet` capability — as opposed to
  `rb_verify_cli::score_replay_against_capture`'s existing mechanical-only
  comparison of two unrelated matches. The wiring and its own unit tests
  were done first (this sandbox has no real Rocket League/BakkesMod
  environment); the owner then ran `cargo run -p rb_verify_cli -- --self
  test2.jsonl` (the real capture from `RB-VERIFY-002-FR-001`, 2,818
  frames) on their own machine and reported back this project's first
  genuine fidelity number:
  ```
  frames compared:    2818
  mean ball distance: 2206.08 uu
  max ball distance:  5673.98 uu
  car pairs compared: 2818
  mean car position/rotation/velocity distance: 4508.71 uu / 2.12 rad / 1421.73 uu/s
  max  car position/rotation/velocity distance: 8798.56 uu / 3.14 rad / 3643.64 uu/s
  ```
  **Interpretation.** This is a large divergence — for scale, the
  standard arena's own half-width is `arena::SIDE_WALL_X = 4096.0` and
  half-length `arena::BACK_WALL_Y = 5120.0`; a mean car position distance
  of `4508.71` and mean ball distance of `2206.08` mean the candidate
  trajectory ends up, on average, in a substantially different part of
  the field than the real one. The mean car *rotation* distance
  (`2.12` rad) is more damning still: `Quat::angle_to`'s range is `[0,
  π]` (`π ≈ 3.14159`, confirmed as this run's own *max* rotation
  distance), and `2.12` is well past `π/2 ≈ 1.57` — the average car
  orientation isn't merely uncorrelated with the real one, it's
  systematically *further* from it than a uniformly random orientation
  would be on average. Read together, this is consistent with essentially
  total trajectory divergence over the run's own ~23-second span (2,818
  frames), not a small, bounded fidelity gap. This is unsurprising rather
  than alarming, for two compounding reasons neither of which this single
  number can separate: (1) physics simulation is chaotic — any modeling
  error, however small, compounds exponentially over dozens of seconds of
  free simulation from one seed frame, and (2) this port's own extensive
  self-documented gap list (uncalibrated `drive`/`arena` placeholder
  constants per `FR-031`; no tire-slip steering model per `FR-065`; no
  per-axis air-control damping per `FR-071`; anisotropic handbrake
  friction unmodeled per `FR-066`; among others) guarantees real modeling
  error exists, not just floating-point noise. **What this number does
  *not* yet establish**: whether the simulation diverges gradually
  (small per-step error compounding) or abruptly (a specific early
  mechanic mismatch derailing the whole run) — that distinction matters
  for what `FR-005`'s actual calibration work should target first, and
  needs a follow-up investigation into divergence *growth over time*
  within this same run, not a second full-run number — now scoped as
  `RB-VERIFY-003-FR-004`.
  - **What shipped, and where it diverged from the original scoping.**
    1. **`rb_verify_cli::score_capture_against_candidate`**, exactly as
       scoped: takes a capture path plus a timestamp tolerance, depends on
       `FR-076`'s new `rb_physics_bullet` capability (`rb_verify_cli`'s
       first dependency on it), and stays a thin composition — the actual
       seeding/stepping logic lives in `rb_physics_bullet::world`, not
       here, matching `AGENTS.md`'s "composition root, no domain logic of
       its own" for this crate.
    2. **Seed-frame heuristic**, implemented as
       `is_grounded_and_neutral`: a frame qualifies only if every car in
       it has `!input.jump && !input.boost && !input.handbrake` and looks
       at rest on the ground (`position.z` within a documented tolerance
       of `rb_physics_bullet::body::CAR_HALF_EXTENTS.z`, `velocity.z`
       within a separate documented tolerance of zero). One thing the
       original scoping didn't spell out: "no dodge in progress" has no
       directly recorded signal at all (a `PhysicsFrame` doesn't carry
       one) — the heuristic can only proxy for it via the grounded +
       no-jump-held check, exactly the gap the Non-goals already called
       out. A capture with no qualifying frame at all returns
       `IngestError::Malformed` rather than silently falling back to frame
       0 — there would be no valid seed to simulate from.
    3. **New CLI entry point**: `rb-verify --self <capture-file>
       [max-timestamp-delta-secs]`, alongside the existing `rb-verify
       <replay-file> <capture-file> [...]` mechanical mode — resolved the
       scoping's "exact flag naming TBD" to a `--self` flag (the candidate
       is simulated from the capture itself, unlike the two-unrelated-
       files mechanical mode).
    4. **The real-capture run**, done: the owner ran `cargo run -p
       rb_verify_cli -- --self test2.jsonl` against the real capture from
       `RB-VERIFY-002-FR-001` on their own machine and reported back the
       numbers quoted above — see the Interpretation note above this
       list.
  - **Non-goals (this requirement).** Does not pre-define a "good enough"
    divergence threshold — per `RB-VERIFY-003`'s own Open Questions, a
    threshold gets calibrated *from* the first real run, not decided
    before it; the number now exists but a threshold decision is deferred
    to `FR-005`, after the divergence-growth diagnostic now implemented
    as `RB-VERIFY-003-FR-004` actually runs against this same result (see
    the Interpretation note above). Does not change any constant based on the result — that
    is explicitly `FR-005`'s job, not this one's, and this result alone
    (whole-run divergence, no per-frame breakdown) isn't yet the right
    shape of evidence to calibrate individual constants from. Does not
    solve multi-car captures beyond what already exists — only a
    single-car freeplay capture has been recorded so far, so multi-car
    candidate simulation is unexercised until a multi-car capture exists.
    Does not add the missing hidden-state setters `FR-076`'s Non-goals
    identified — only works around them via the seed-frame heuristic
    above; given how total this run's own divergence turned out to be,
    whether that heuristic (rather than the seed frame's own hidden-state
    gap) is even a meaningful contributor is itself an open question
    `RB-VERIFY-003-FR-004`'s diagnostic should shed light on before
    assuming it needs fixing.
  - **Acceptance criteria.** `rb_verify_cli` produces a divergence score
    between a real capture's recorded outcome and a candidate trajectory
    `rb_physics_bullet` actually simulated from that capture's own
    recorded input — the first score in this project with a genuine
    physical reason to be small if the physics core is accurate, unlike
    every `score_replay_against_capture` run to date. Met: both for the
    synthetic capture fixture (see Verification plan) and for the real
    capture (numbers above) — the score exists and is fully wired, even
    though the number itself is large.
  - **Verification plan.** 3 new unit tests mirroring
    `score_replay_against_capture`'s own precedent: a happy-path run
    against `rb_capture_ingest`'s synthetic capture fixture (which does
    contain a grounded, neutral frame 0, so exercises the whole path
    end-to-end without needing a real capture), a missing-file I/O-error
    case, and a hand-built capture with no grounded/neutral frame at all
    exercising the new `Malformed` error path. All 6 `rb_verify_cli` tests
    pass; full workspace `cargo fmt`/`clippy`/`test` all green (388 tests
    workspace-wide). The manual end-to-end run against the real capture
    is done — numbers recorded above and in Change history.
- `RB-PHYSICS-001-FR-078` (car hitbox calibration, implemented): retunes
  every existing `car_box` call site across this crate's own test suite
  that models a real car to `body::CAR_HALF_EXTENTS` — the confirmed real
  Octane hitbox `FR-076` introduced but deliberately left every
  pre-existing call site on the old placeholder (`Vec3::new(60.0, 30.0,
  18.0)`, ~44% narrower on its Y half-extent than the real value) — rather
  than leaving that discrepancy indefinitely deferred to "some future
  calibration FR."
  - **Scope and approach.** Every `car_box`/`some_car`/`car_at_origin`/
    `stationary_car`/`car()`-style test helper across `body.rs`,
    `collision.rs`, `drive.rs`, `net.rs`, `solver.rs`, and `world.rs` that
    was building a *real car* stand-in (the literal old placeholder, or a
    local variable initialized to it) was switched to
    `body::CAR_HALF_EXTENTS` (via `RigidBody::standard_car` where mass was
    already `180.0`, or `car_box(CAR_HALF_EXTENTS, ...)` directly
    otherwise). A shape deliberately *not* modeling a real car — a unit
    cube, a symmetric pair of identical boxes for a tie-break or
    box-vs-box test, a tiny probe box for goal-window/bounded-wall
    corner-count tests — was left untouched, since it was never
    representing this hitbox in the first place and changing it would add
    risk for no calibration benefit. Rather than manually recomputing
    every downstream hardcoded expected value by hand (error-prone given
    an anisotropic, non-uniform shape change — X changed ~0.4%, Y ~44.5%,
    Z ~7.4%, unlike `FR-036`'s single-scalar ball-radius substitution),
    each test's own duplicate-literal dependency on the exact half-extents
    was refactored to read the actual half-extents used to construct that
    test's own car (a local variable or `CAR_HALF_EXTENTS` directly)
    rather than a hardcoded duplicate — the same "don't duplicate a
    constant as a bare literal" fix applied throughout this port's own
    history, which also makes the fix verifiable by construction rather
    than by manually re-deriving trigonometry. The empirical,
    test-driven-first approach this FR actually used: swap the
    constructor, run the suite, and let any resulting failure name exactly
    which assertion needs the same treatment, rather than trying to
    predict every affected assertion by static reading alone.
  - **What actually needed a real recompute, not just a variable
    reference.** Only assertions checking a car's *resting height on flat
    ground* (`position.z` settling near the car's own half-extent) were
    genuinely dependent on the old exact value in a way a variable
    reference alone couldn't paper over, since several of those literals
    (`18.0`) were the *assertion's own threshold*, not a value flowing
    into car construction — these were switched to `CAR_HALF_EXTENTS.z`
    (`19.32955`, not `18.0`). Two solver-level tests
    (`resolve_dynamic_manifolds_relaxes_a_shared_bodys_impulse_by_its_own_contact_degree`
    and its sibling FR-030 test) carry doc comments citing specific
    measured velocities (`~89.5`, `~32` units/s) for a symmetric ball-vs-
    two-cars pinch scenario; these were re-measured after the swap and
    confirmed unchanged (`~32.0` for the shared-relaxation figure) — a
    purely 1D, mass/velocity-driven collision along a fixed contact
    normal has no dependency on the absolute half-extent value the
    contact happens to occur at, only on mass and velocity, so no comment
    correction was needed there. `ball_center_embedded_in_car_pushes_out_
    the_nearest_face`'s own doc comment (margin figures for each face)
    was corrected to the real recomputed margins even though the
    underlying assertion (which face wins) still held either way.
  - **Non-goals (this requirement).** Does not touch car
    restitution/friction (still the generic `0.5`/`0.5` placeholder per
    `FR-063`'s own finding that no single real value exists to adopt).
    Does not touch `net.rs`'s two car-in-net tests' mass (`1.0`, a
    deliberate testing choice unrelated to the real `CAR_MASS`, left
    alone) — only their half-extents were corrected. Does not change
    `RigidBody::car_box`'s own generic signature or any production
    (non-test) call site beyond `standard_car` itself, which already used
    the corrected value since `FR-076`. Does not produce a new fidelity
    number or feed into `FR-005`'s calibration — this is a test-suite
    hygiene correction, not new real-data-driven physics.
  - **Acceptance criteria.** No `car_box`/`some_car`-style test helper
    modeling a real car anywhere in `rb_physics_bullet` constructs one
    from the old placeholder half-extents (`Vec3::new(60.0, 30.0,
    18.0)`) — the only surviving literal reference to it is the
    intentional negative-comparison test
    (`car_half_extents_deliberately_differs_from_the_crates_own_test_placeholder`)
    and historical doc-comment citations. All 335 pre-existing
    `rb_physics_bullet` tests still pass (no test count change — a
    constant-correctness change with no new behavior to characterize,
    matching `FR-036`'s own precedent); full workspace `fmt`/`clippy`/
    `test` green (388 tests workspace-wide).
  - **Verification plan.** No new tests, for the same reason `FR-036`
    added none: the fix is proven by the existing suite passing unchanged
    once the switch is made, plus the two solver-level velocity figures
    independently re-measured and confirmed stable. `grep -rn "60.0, 30.0,
    18.0"` across `crates/rb_physics_bullet/src/*.rs`, run after the
    change, confirmed only the four intentional/historical references
    remain (the negative-comparison test and three doc comments).
- `RB-PHYSICS-001-FR-079` (isolated dodge-derailment investigation; the
  inertia-cancellation fix identified for air control is implemented, and
  the residual pre-dodge gap it left open was traced to a further,
  separate pitch/roll sign bug in both air control and the dodge — now
  also fixed, closing the pre-dodge gap; `FR-069` remains): the concrete
  next step
  `RB-VERIFY-003-FR-004`'s real run called for — replaying `FR-077`'s own
  real capture's abrupt-derailment dodge in isolation from the same seed
  state — was carried out, and it both confirms the maneuver as the
  proximate cause and refines *why* into a more precise, multi-part
  picture than the original single-hypothesis framing.
  - **Isolated-replay confirmation.** A new real fixture,
    `crates/rb_capture_ingest/fixtures/dodge-derailment.capture.jsonl`
    (347 frames, `t=4.117`–`7.0` s excerpted directly from `FR-077`'s own
    `test2.jsonl`), starts at the last grounded, neutral instant before
    the recorded jump — `rb_verify_cli`'s existing `is_grounded_and_neutral`
    heuristic selects it as frame 0 with no new code needed.
    `score_capture_against_candidate` against this fixture alone (no
    ~4-second head start of otherwise-correct simulation) still produces
    a large divergence (`mean_ball_distance ≈ 730` uu, `cars.mean_position_distance
    ≈ 2449` uu over the isolated 347 frames) — confirming the derailment
    is not an artifact of compounded earlier drift; the maneuver itself
    reproduces it standalone. Encoded as a permanent regression baseline:
    `rb_verify_cli::tests::isolated_replay_of_the_real_dodge_still_diverges_sharply`.
  - **Finer-grained reading refines the picture beyond "the dodge alone."**
    Stepping `score_capture_growth` down to `0.05`s windows (and, deeper
    still, comparing per-frame `Quat::angle_to` distance directly against
    `simulate_recorded`'s own output) shows the car's *orientation* begins
    drifting smoothly — not abruptly — starting from the ground jump
    itself (`t≈4.13`), reaching `~0.22` rad (`~12.5°`) by the moment the
    second jump press fires the dodge at `t=4.317`, while linear velocity
    still tracks closely (within `~80` uu/s) up to that same instant. The
    dodge itself then does two separate, simultaneous things:
    1. **Translation.** At the exact dodge frame, the recorded car's
       velocity gains mostly a `+X` component (`389→1009` uu on X, `1138→
       1112` uu on Y — a modest Y change); the candidate's velocity
       instead gains mostly a large *negative* `Y` component (`308→646`
       on X, `1134→−1077` uu on Y — Y flips sign entirely). Since this
       port's dodge impulse (`drive.rs`) is computed relative to the
       car's *own current* `forward`/`right_axis` — themselves a function
       of its current orientation — the pre-existing `~12.5°` orientation
       gap from the jump-hold window is enough to rotate an otherwise
       correctly-shaped impulse into a qualitatively different world
       direction. The translation mismatch is a **consequence** of the
       earlier orientation drift, not an independent bug in the dodge
       impulse formula itself.
    2. **Rotation.** After the dodge, `Quat::angle_to` distance shows a
       periodic beat pattern — rising toward the `π` cap, falling back
       toward `~0.5` rad, rising again, with a period of roughly
       `0.5`–`0.6` s — the signature of two bodies spinning at different,
       nearly-constant rates drifting in and out of phase, not a one-time
       offset. This is consistent with (though not yet isolated as fully
       explaining) `FR-069`'s already-documented instantaneous-kick-vs-
       continuous-torque architecture gap: a fixed-rate `5.5` rad/s kick
       applied all at once versus real Rocket League's `FLIP_TORQUE_X =
       260`/`FLIP_TORQUE_Y = 224` continuous torque integrated over a
       `0.65`s window would plausibly produce different net spin *rates*,
       not just different total displacement.
  - **What this changes about `FR-005`'s own starting point.** The
    original hypothesis (this port's instantaneous dodge-spin kick as
    *the* cause) was too narrow: the evidence points to an earlier,
    smaller, and currently unexplained orientation-rate divergence during
    the initial grounded-jump-hold-plus-sustained-air-control window
    (`t≈4.13`–`4.32`, well before the dodge itself fires) as the true
    first departure, which the dodge's own orientation-relative impulse
    then amplifies into a dramatically different-looking outcome — on top
    of a likely-separate post-dodge spin-rate mismatch matching `FR-069`.
    Both need their own dedicated investigation before a fix is
    attempted; neither has been isolated further than described above.
  - **Root cause of the pre-dodge orientation-rate divergence: an
    inertia-cancellation mismatch, not a miscalibrated constant.** Reading
    RocketSim's own real `Car.cpp::_UpdateAirTorque` (the source
    `AIR_CONTROL_TORQUE` was ported from, per `FR-068`) shows it computes
    `torque = ... * CAR_AIR_CONTROL_TORQUE` from stick input exactly as
    `FR-068` already confirmed, but then applies it as
    `_rigidBody.applyTorque(_rigidBody.m_invInertiaTensorWorld.inverse() *
    (torque - damping) * CAR_TORQUE_SCALE)` — pre-multiplying by the car's
    own *actual* (non-inverted) world inertia tensor before handing the
    result to Bullet's own `applyTorque`, which itself divides by the
    inverse inertia tensor again during integration. The two cancel: real
    Rocket League's `CAR_AIR_CONTROL_TORQUE` is, by construction, an
    inertia-*independent* direct angular-acceleration input, not a genuine
    physical torque — and the same pre-multiplication pattern appears at
    the dodge-torque (`Car.cpp` line 631) and autoroll-torque (line 867)
    call sites. This port's own `RigidBody::apply_torque`
    (`crates/rb_physics_bullet/src/body.rs`) and `integrate.rs` implement
    the standard, non-cancelling model already confirmed correct against
    real Bullet by `FR-046` (`angular_velocity += inv_inertia_world *
    total_torque * dt`) — meaning every existing call site that reuses a
    borrowed RocketSim constant this way (confirmed for
    `AIR_CONTROL_TORQUE`; not yet checked for anything else) has its
    intended angular acceleration silently divided by this car's own
    moment of inertia, a scaling step real Rocket League's own code never
    applies.
    - **Quantitative confirmation.** This car's own box `I_zz` (yaw axis),
      via the already-confirmed-correct formula in `body.rs`
      (`local_inertia`), is `CAR_MASS/3 * (CAR_HALF_EXTENTS.x² +
      CAR_HALF_EXTENTS.y²) ≈ 330,581`. The predicted candidate
      steady-state yaw angular acceleration under the current model is
      therefore `AIR_CONTROL_TORQUE * AIR_CONTROL_YAW_SCALE / I_zz =
      1,000,000 * (95/130) / 330,581 ≈ 2.211` rad/s² — matching the
      isolated fixture's own empirically-measured candidate yaw
      acceleration (`≈2.2` rad/s²) to three significant figures, while the
      *recorded* car's own measured yaw acceleration over the same window
      is `≈9.12` rad/s² (a `≈4.1x` gap), matching an independent, purely
      empirical measurement of the same ratio (`≈4.13`–`4.15x`) made
      before this mechanism was identified.
    - **Rejected fix: scaling `AIR_CONTROL_TORQUE` uniformly.** A
      temporary (fully reverted, never committed) experiment multiplying
      `AIR_CONTROL_TORQUE` by `≈4.15x` improved the pure-yaw sub-phase but
      *worsened* the subsequent pitch/roll sub-phase and the whole-window
      aggregate — because the anisotropic box has a different actual
      moment of inertia per axis, so a single uniform multiplier cannot
      correct all three axes at once under a model that still divides by
      inertia. This confirms the mismatch is architectural (a missing
      inertia-cancellation step in how these specific borrowed constants
      are applied), not a single miscalibrated number, and rules out a
      one-line constant tweak as a valid fix — the same "naive global
      factor investigated and rejected" pattern `FR-041` documented for a
      different mechanic.
    - **Scope of the finding.** Confirmed only for `AIR_CONTROL_TORQUE`.
      `STEER_TORQUE` is a pure invented placeholder, not a ported real
      constant (`FR-065`), so it isn't affected. The dodge's own spin kick
      bypasses `apply_torque`/integration entirely via a direct
      `angular_velocity +=` (so it isn't affected either, though
      `FR-069`'s separate instantaneous-kick-vs-continuous-torque gap
      still stands independently). `LANDING_AUTO_UPRIGHT_TORQUE` uses the
      same `apply_torque` mechanism and so is a plausible further
      instance, but this has not been independently confirmed against
      RocketSim's own autoroll-torque call site with the same rigor as
      `AIR_CONTROL_TORQUE` above.
  - **Implemented: the inertia-independent torque path for air control.**
    `RigidBody` (`body.rs`) gained a second accumulator,
    `total_angular_accel`, and `apply_angular_acceleration` to feed it —
    kept deliberately separate from `total_torque`/`apply_torque` because
    `integrate_velocities` (`integrate.rs`) folds it into
    `angular_velocity` directly (`+= total_angular_accel * dt`), with no
    `inv_inertia_world` multiply at all, mirroring exactly what real
    Rocket League's own inertia pre-multiply/cancel achieves. `drive.rs`'s
    three air-control constants were replaced with RocketSim's own real,
    unscaled `CAR_AIR_CONTROL_TORQUE` values directly
    (`AIR_CONTROL_PITCH_TORQUE = 130.0`, `AIR_CONTROL_YAW_TORQUE = 95.0`,
    `AIR_CONTROL_ROLL_TORQUE = 400.0`, replacing the old
    `AIR_CONTROL_TORQUE`/`AIR_CONTROL_YAW_SCALE`/`AIR_CONTROL_ROLL_SCALE`
    placeholder-plus-ratio scheme), applied via
    `apply_angular_acceleration` scaled by a new, separately-fetched real
    constant: RocketSim's own `RLConst.h` defines `CAR_TORQUE_SCALE = 2 *
    M_PI / (1 << 16) * 1000 ≈ 0.095882` — the exact factor
    `_UpdateAirTorque`'s own `CAR_TORQUE_SCALE` multiply requires to turn a
    raw `CAR_AIR_CONTROL_TORQUE` value into rad/s². This is not the
    "uniform rescale" already rejected above: it's a mechanism change
    (bypassing inertia division entirely) plus adopting the real constants
    the mechanism was always meant to receive, so each axis gets its own
    correct real acceleration rather than one shared multiplier distorted
    per-axis by the anisotropic inertia division the old model still
    applied.
    - **A second, independent quantitative confirmation.** Before this fix,
      the only check available was comparing the *old* model's own
      predicted-vs-measured candidate acceleration (`≈2.211` vs `≈2.2`
      rad/s², both under the wrong mechanism). The fix makes a stronger,
      independent check available: computing the *real* car's own expected
      acceleration directly from RocketSim's real constants alone, with no
      reference to this port's own model at all — `AIR_CONTROL_YAW_TORQUE
      * CAR_TORQUE_SCALE = 95.0 * 0.095882 ≈ 9.109` rad/s² for full yaw
      input, matching the *recorded* car's own independently-measured yaw
      acceleration (`≈9.12` rad/s², from the same isolated-replay window
      `FR-079`'s original finding used) even more tightly than the old
      model's own internal self-consistency check did.
    - **Real-data effect: a measured, partial improvement.** Re-running
      `score_capture_growth` at `0.05`s windows against the isolated
      `dodge-derailment.capture.jsonl` fixture with the fix in place shows
      the specific pre-dodge orientation gap this whole investigation
      targeted shrink from `~0.22` rad (`~12.5°`, the original finding's
      own number) to `~0.13` rad (`~7.4°`) at the same point — the last
      full window before the second jump press fires the dodge at
      `t=4.317` — a real, comparable, roughly 40% reduction measured
      directly against real recorded data, not just a unit-test assertion.
      The isolated fixture's own *whole-trajectory* divergence
      (`isolated_replay_of_the_real_dodge_still_diverges_sharply`'s own
      metric) did not shrink to match — `cars.mean_position_distance` rose
      slightly, from `≈2449` to `≈2792` uu, and `mean_ball_distance` stayed
      essentially flat (`≈730` uu both before and after). This is not a
      sign the fix is wrong: a residual `~7°` orientation gap still gets
      amplified by the dodge's own orientation-relative impulse into a
      different world direction (the same amplification mechanism the
      finding above already identified, just starting from a smaller
      input), and `FR-069`'s own separate, still-unfixed
      instantaneous-kick-vs-continuous-torque post-dodge spin-rate
      mismatch continues to dominate the aggregate metric regardless of
      how small the pre-dodge gap gets. The fix's own job — the specific,
      analytically-identified root cause — is confirmed fixed; the
      isolated fixture's own aggregate score was never a target this fix
      alone could be expected to move, given `FR-069`'s gap remains.
    - **What's still open (as of the inertia-cancellation fix alone).** The
      residual `~7°` pre-dodge gap itself was not yet explained at that
      point — it might have been a smaller, separate mismatch, or simply
      the floor of what that fix alone could reach given other
      still-unadopted real mechanics (e.g. `FR-071`'s own air-control
      damping, still not implemented). `FR-069`'s continuous-torque flip
      model remained unimplemented and, more clearly than before, the
      larger remaining piece of the isolated dodge's own divergence.
  - **Root cause of the residual pre-dodge gap: pitch and roll apply about
    the wrong sign of their own axis.** Isolating the exact per-frame
    behavior during the fixture's own second pre-dodge sub-phase
    (`t≈4.24`–`4.32`s: `jump` released, `pitch=-1, roll=-1` held, `yaw=0`)
    found the candidate's angular-velocity change over a single tick to be
    almost exactly the *negative* of the recorded car's own — at
    `t=4.2417`, orientation distance between real and candidate is only
    `0.027` rad (`1.54°`), yet the very next tick's angular-velocity delta
    already diverges in **sign**, not just magnitude, on both the x and y
    world axes (real: `(+0.056, +0.331)`; candidate: `(-0.047, -0.332)` —
    almost exactly equal magnitude, opposite sign). A `1.54°` orientation
    gap cannot rotate a torque vector's world-frame direction by anywhere
    near `180°` in one step, ruling out "accumulated orientation drift" as
    the explanation for *this* symptom (distinct from the earlier,
    correctly-attributed dodge-translation finding, which *is* explained
    by accumulated drift). Re-deriving the candidate's own predicted
    acceleration from its own formula (`right_axis * pitch *
    AIR_CONTROL_PITCH_TORQUE * CAR_TORQUE_SCALE + forward_axis * roll *
    AIR_CONTROL_ROLL_TORQUE * CAR_TORQUE_SCALE`) reproduces the candidate's
    own (wrong-signed) output exactly — this port's own code is internally
    self-consistent, so the bug isn't an implementation slip in applying
    the formula, only in the formula's own sign.
    - **Confirmed against RocketSim's real source, not just inferred.**
      Fetching `Car.cpp` directly shows `_UpdateAirTorque` doesn't use the
      car's plain `GetRightDir()`/`GetForwardDir()` for pitch/roll at all:
      `dirPitch_right = -GetRightDir()`, `dirRoll_forward =
      -GetForwardDir()` (only `dirYaw_up = GetUpDir()` is unnegated). Real
      Rocket League applies pitch and roll about the *negative* of the
      car's own right/forward axes — this port's `drive.rs` applies them
      about the *positive* `right_axis(car)`/`forward` (the same functions
      already correctly used, unnegated, for throttle/steering, so this
      isn't a project-wide axis-convention mismatch, only a pitch/roll-
      specific one). Negating the candidate's own predicted acceleration
      above (equivalent to negating both `dirPitch_right` and
      `dirRoll_forward` together, exactly matching RocketSim's own double
      negation) gives `(+5.7, +39.9)` — matching the recorded car's own
      `(+6.7, +39.7)` far more closely than the unnegated `(-5.7, -39.9)`
      ever could, especially on the dominant roll-driven y component. Yaw
      (`dirYaw_up`, unnegated in both real and this port) was never
      affected, consistent with Phase A of this same fixture already
      tracking real yaw acceleration closely after the inertia-
      cancellation fix.
    - **Scope: the dodge has the same bug, three ways.** The dodge's own
      impulse/spin computation (`drive.rs`'s ground and wall-jump dodge
      blocks — a separate code path reading the same raw `pitch`/`roll`
      input) was checked against RocketSim's `_UpdateDoubleJumpOrFlip` /
      `_UpdateAirTorque` in the same pass. Real: `dodgeDir = (-controls.pitch,
      controls.yaw + controls.roll, 0)` (normalized), impulse `=
      dodgeDir.x * forwardDir2D + dodgeDir.y * rightDir2D`, and the flip's
      local torque `= (-dodgeDir.y * FLIP_TORQUE_X, dodgeDir.x *
      FLIP_TORQUE_Y, 0)` (local x = forward, y = right). So `pitch = -1`
      (stick forward) is a *forward* flip translating along `+forward` and
      spinning about `+right` (nose down first), and `yaw + roll = -1` a
      *left* dodge translating along `-right` and spinning about
      `+forward` (left side down first). This port had the pitch
      translation inverted (`forward * pitch`: stick forward dodged
      backward), the pitch spin inverted (`right * pitch`), and the roll
      spin inverted (`forward * roll`); only the roll translation
      (`right * roll`) already matched. `normalize_dodge_direction`'s own
      doc comment had recorded keeping "this port's own sign convention
      (positive pitch means forward) rather than the reference's negated
      `-controls.pitch`" as a deliberate choice — it was a bug, since the
      stick values this port replays come straight from real captures in
      the reference's convention. This also corrects the earlier
      "Translation" finding above: the dodge-frame velocity mismatch
      (`+X` real vs. `-Y` candidate) was primarily this sign inversion
      acting on a nearly-correct orientation, not accumulated orientation
      drift rotating a correct impulse.
  - **Implemented: the pitch/roll sign fix, for air control and the dodge
    together.** `drive.rs`'s air control now applies pitch about
    `-right_axis(car)` and roll about `-forward`, matching
    `dirPitch_right`/`dirRoll_forward` (yaw unchanged). Both dodge blocks
    now form `dodge_forward = -norm_pitch` exactly as the reference forms
    `dodgeDir.x`, and use it for the forward impulse, the spin about
    `+right`, and the backward classification; the roll spin is now about
    `-norm_roll * forward` (the reference's `-dodgeDir.y`), with the roll
    translation unchanged. `dodge_pitch_is_backward` was renamed
    `dodge_is_backward` and now takes that forward component, making it a
    symbol-for-symbol match for RocketSim's `shouldDodgeBackwards`
    (previously it re-derived the same rule under the inverted
    convention). No constant changed; `DODGE_SPEED`/`DODGE_ANGULAR_SPEED`
    remain the same placeholders.
    - **Real-data effect: the pre-dodge gap is closed, and the aggregate
      finally moves.** On the isolated `dodge-derailment.capture.jsonl`
      fixture at `0.05`s windows, the last full pre-dodge window's
      orientation gap went from `~0.13` rad (after the inertia-cancellation
      fix) to `~0.03` rad (`~1.7°`) — `~0.22` → `~0.13` → `~0.03` rad across
      the three fixes, so the pre-dodge orientation-rate divergence this
      entry set out to isolate is now essentially gone. The whole-fixture
      score moved for the first time: `cars.mean_position_distance` `≈2792`
      → `≈937` uu (`-66%`; `≈2449` at the original finding), max `≈5919` →
      `≈2606` uu, mean rotation `1.63` → `1.39` rad, mean velocity `≈2177`
      → `≈1369` uu/s. `mean_ball_distance` is unchanged at `≈730` uu — the
      ball is only touched at `t≈5.6`s, so its divergence follows the
      car's post-dodge path. What remains is now clearly post-dodge: at
      the dodge tick the velocity gap jumps to `≈1030` uu/s (the dodge
      impulse's own uncalibrated `DODGE_SPEED` magnitude and missing
      vertical component), and the rotation gap then grows steadily
      (`0.13` → `1.14` rad over the following `0.4`s, a `~2.5` rad/s rate
      difference) — the instantaneous-kick-vs-continuous-torque spin-rate
      mismatch `FR-069` already documented, now the dominant remaining
      piece.
  - **Non-goals (this requirement).** Does not implement `FR-069`'s
    continuous-torque flip model, or `FR-071`'s air-control damping
    mechanism — both remain separate, scoped candidates for future work,
    and `FR-069` is now the clearly dominant remaining piece of this
    fixture's own divergence. Does not calibrate `DODGE_SPEED`/
    `DODGE_ANGULAR_SPEED` or add the real dodge's small vertical component
    (`FR-059`'s and the module doc's own already-recorded gaps) — the sign
    fix corrects direction only. Does not touch `LANDING_AUTO_UPRIGHT_TORQUE`
    or `STEER_TORQUE` — see "Scope of the finding" above for why neither is
    a confirmed instance of the inertia-cancellation mismatch (the sign
    bug is a separate, independent one). Does not claim any quantitative
    confirmation above is the *complete* explanation for every remaining
    discrepancy; only what this real maneuver's own recorded data and
    RocketSim's own real source directly show.
  - **Acceptance criteria.** The new fixture's own first frame is
    grounded/neutral by construction (verified: `is_grounded_and_neutral`
    accepts it without special-casing); `score_capture_against_candidate`
    against it produces `frames_compared == 347` and
    `cars.pairs_compared == 347` (every frame matched, confirming the
    fixture's own internal timestamp continuity); the isolated replay's
    own divergence is bounded *above* by a ratchet
    (`cars.mean_position_distance < 1000` uu, `mean_ball_distance < 1000`
    uu, set just above the `≈937`/`≈730` uu measured after the sign fix)
    that fails if a later change makes this real replay worse — replacing
    the earlier known-bad lower bounds (`> 1000`/`> 100`), which the sign
    fix itself pushed the car figure under. `apply_angular_acceleration`'s
    own output is provably independent of `inv_inertia_world` (unit tests
    in `integrate.rs`). Every pitch/roll-direction-asserting test in
    `drive.rs`/`world.rs` now uses real Rocket League's own stick
    convention (`pitch = -1` forward, `roll = +1` right) and asserts the
    real spin direction (a forward flip about `+right`, a right dodge
    about `-forward`); all other tests pass unchanged.
  - **Verification plan.** `body.rs`: 1 new getter
    (`total_angular_acceleration`), `apply_angular_acceleration`, and the
    new field reset in `clear_forces`. `integrate.rs`: 2 new tests
    (`angular_acceleration_input_is_not_divided_by_inertia`,
    `angular_acceleration_and_torque_accumulators_are_independent`).
    `drive.rs`: the three old per-axis air-control tests collapsed into 1
    combined test asserting each axis's own angular velocity matches
    `raw_torque * CAR_TORQUE_SCALE * dt` directly, with no
    `inv_inertia_world` term at all (net: `-2` tests). `net.rs`'s own
    stale cross-reference to `drive::AIR_CONTROL_TORQUE` (now renamed)
    updated to `drive::STEER_TORQUE`, still a valid uncalibrated-placeholder
    example. Full workspace `fmt`/`clippy`/`test` green (397 tests: 336 in
    `rb_physics_bullet`, up from 335 net of the test-count changes above;
    10 in `rb_verify_cli`'s own `isolated_replay_of_the_real_dodge_still_diverges_sharply`
    still passing unchanged against its own loose bounds, now with
    different real numbers behind them, as described above). The
    pitch/roll sign finding itself was investigated via a temporary,
    never-committed example (`crates/rb_verify_cli/examples/`, deleted
    after use, matching this project's own established convention for
    throwaway per-frame investigation scripts) plus a direct fetch of
    RocketSim's real `Car.cpp`/`Car.h` source. The sign fix then touched
    `drive.rs` only (air control's two axis negations; `dodge_forward =
    -norm_pitch` and the roll-spin negation in both dodge blocks;
    `dodge_pitch_is_backward` → `dodge_is_backward`) plus the tests that
    encoded the old convention: 12 `drive.rs` tests and 2 `world.rs` tests
    switched their stick sign or expected spin sign (no test added or
    removed; `rb_physics_bullet` stays at 336), and `rb_verify_cli`'s
    baseline test was renamed
    `isolated_replay_of_the_real_dodge_stays_under_its_last_recorded_divergence`
    and turned into the ratchet described above (10 `rb_verify_cli` tests,
    unchanged). Full workspace `fmt`/`clippy`/`test` green (397 tests).
- `RB-PHYSICS-001-FR-080` (real continuous flip torque — implemented in
  three steps: (a) the real dodge impulse magnitude; (b) the flip state,
  torque, vertical bleed, and pitch lock; (c) the real pitch-hold flip
  cancel, plus the two corrections the real capture forced — yaw/roll air
  control live mid-flip, and the angular-speed clamp after the transform
  integration): `FR-069` confirmed
  the mechanism as a documentation
  finding and `FR-079` made it the dominant remaining piece of the
  isolated dodge's own divergence. This entry scopes actually adopting
  it: the complete real mechanism (not just the torque line), what the
  real capture confirms about it, the proposed design for this port, and
  the blast radius — so the implementation can start from a settled plan.
  - **The real mechanism, complete.** Read directly from RocketSim's
    `Car.cpp` (`_UpdateDoubleJumpOrFlip`, `_UpdateAirTorque`,
    `_FinishPhysicsTick`, `_PreTickUpdate`) and `RLConst.h`:
    1. **Start** (`_UpdateDoubleJumpOrFlip`, on a fresh airborne jump press
       with stick input past the deadzone and the flip still unused):
       `flipTime = 0`, `hasFlipped = isFlipping = true`, `flipRelTorque =
       (-dodgeDir.y, dodgeDir.x, 0) / tickTimeScale` in the car's local
       frame (x = forward, y = right; `tickTimeScale = dt / (1/120)`), and
       the translation impulse `FR-079` already matched. The initial
       velocity is `dodgeDir * FLIP_INITIAL_VEL_SCALE` (`500`) before the
       per-direction speed scales `FR-059` already adopted — this port's
       `DODGE_SPEED = 1400` placeholder is `2.8x` that (see the real-data
       paragraph below for the confirmation).
    2. **Every airborne tick while flipping** (`_UpdateAirTorque`, called
       whenever fewer than three wheels touch; `isFlipping = hasFlipped &&
       flipTime < FLIP_TORQUE_TIME (0.65)`): apply
       `applyTorque(invInertiaTensorWorld.inverse() * basis *
       (flipRelTorque * (FLIP_TORQUE_X, FLIP_TORQUE_Y, 0)))` — the same
       inertia pre-multiply/cancel `FR-079` found for air control (so it
       goes through this port's `apply_angular_acceleration` path), but
       **without** `CAR_TORQUE_SCALE`. That makes it a per-tick angular
       velocity step of `224 / 120 ≈ 1.87` rad/s (pitch, `FLIP_TORQUE_Y`)
       or `260 / 120 ≈ 2.17` rad/s (roll, `FLIP_TORQUE_X`) — which reaches
       `CAR_MAX_ANG_SPEED = 5.5` within three ticks and is then held there
       by `_FinishPhysicsTick`'s per-tick clamp for the rest of the window.
       The "continuous torque" is, in effect, *drive to the angular-speed
       cap along the flip axis and hold it there for 0.65 s*. Dividing
       `flipRelTorque` by `tickTimeScale` makes that step per-*tick*, not
       per-second, so it's the same at any tick rate.
    3. **Stick air control is off while flipping** (`doAirControl = false`)
       — no pitch/yaw/roll torque and no `CAR_AIR_CONTROL_DAMPING`
       (`FR-071`) at all — with two exceptions: a stalled flip (zero rel
       torque) allows it, and the flip-cancel below allows it with pitch
       zeroed. `_UpdateAirTorque` also runs the flip torque only, never
       the landing-assist-like auto-roll, while `isAutoFlipping`.
    4. **Flip cancel** (`FR-070`'s finding, now to be adopted): while
       flipping, if `flipRelTorque.y != 0` and `controls.pitch` is held in
       that same sign, `pitchScale = 1 - |pitch|` multiplies only the
       torque's pitch (y) component, every tick, for as long as the stick
       is held — and air control is re-enabled for yaw/roll (pitch torque
       is scaled to zero during any flip). A second jump press does nothing
       (`hasFlipped` makes `canUse` false), which this port's
       `double_jump_available == false` already models.
    5. **Vertical damping** (`FLIP_Z_DAMP`, in `_UpdateDoubleJumpOrFlip`):
       while `0.15 ≤ flipTime ≤ 0.65` and (`vz < 0` or `flipTime < 0.21`),
       `linearVelocity.z *= (1 - 0.35)^tickTimeScale` — a `×0.65` per tick
       bleed of vertical speed.
    6. **After the window**: `flipTime` keeps counting while `hasFlipped`;
       pitch input stays locked (`pitchTorqueScale = 0`) until `flipTime <
       0.65 + FLIP_PITCHLOCK_EXTRA_TIME (0.3)`, during which pitch damping
       acts at full strength (`1 - |pitch * 0|`). `FLIP_PITCHLOCK_TIME =
       1.0` is declared in `RLConst.h` but referenced nowhere in `Car.cpp`.
    7. **Ground**: three or more wheels in contact sets `isFlipping =
       false`; `isOnGround` resets `hasFlipped`, `flipTime`, and the
       double-jump flag.
  - **What the real capture confirms, to the tick.** In the isolated
    `dodge-derailment.capture.jsonl` fixture (dodge at `t = 4.3167`):
    `|ω|` goes `3.40 → 5.22 → 5.50` in two ticks and reads exactly `5.50`
    (three significant figures) every tick through `t = 4.975` — the
    window end `4.3167 + 0.65 = 4.967` — then decays at `≈3.9`/s under
    `pitch = +1` with pitch locked (that decay is `FR-071`'s damping, not
    a torque). `vel.z` drops `222 → 131 → 24 → -5` from `t ≈ 4.467`
    (`FLIP_Z_DAMP_START`), then holds at `-15.5` uu/s until `t ≈ 4.967`:
    exactly the equilibrium of `vz = 0.65 * (vz - 650/120)`, i.e.
    gravity-per-tick over `(1 - 0.65)`, `-5.42 / 0.35 = -15.5`. The
    dodge-tick velocity change is `≈620` uu/s in magnitude; `500 * (0.707,
    -0.707)` with `FR-059`'s side scale at the recorded forward speed
    (`≈1170` uu/s, ratio `0.51`) gives `(354, -516)`, magnitude `626` —
    `FLIP_INITIAL_VEL_SCALE = 500` confirmed to `~1%` from real data. Every
    piece of the mechanism above lands on its own constant. What this port
    does instead: a single `5.5` rad/s kick along the normalized
    `(forward, right)` mix, then stick air control (pitch/roll are held
    throughout the fixture's dodge) keeps re-steering `ω` every step with
    `clamp_angular_speed` re-normalizing it, no vertical damping, a
    `1400` uu/s impulse, and a spin that never stops on its own. That is
    the `~2.5` rad/s growing rotation gap and most of the `≈1030` uu/s
    velocity jump `FR-079` left at the dodge tick.
  - **Proposed design for this port.**
    1. **State.** Replace the per-car `dodge_flip_active: bool`
       (`PhysicsWorld::car_dodge_flip_active`) with `Option<DodgeFlip>`,
       `DodgeFlip { rel_torque: (f32, f32) /* (forward, right) */, elapsed:
       f32 }`. `elapsed` keeps counting past `FLIP_TORQUE_TIME` (for the
       pitch lock) until landing clears the whole `Option`; a plain double
       jump clears it too (as `FR-016`'s flag already does). Threaded
       exactly like `jump_hold_time_remaining`: a parallel `Vec` on
       `PhysicsWorld` defaulting to `None` in `with_car`/`from_frame`, one
       more `&mut` through `drive_and_integrate_velocities` into
       `apply_driven_forces` (replacing the `bool`, not adding a
       parameter). `from_frame`'s hidden-state caveat applies unchanged —
       a flip in progress can't be seeded from a `PhysicsFrame`, which
       `is_grounded_and_neutral` already guards against.
    2. **At the dodge** (both the ground and wall-jump-dodge blocks): keep
       the translation impulse (its direction is now right per `FR-079`;
       its magnitude becomes `FLIP_INITIAL_VEL_SCALE = 500`, replacing
       `DODGE_SPEED`), drop `car.angular_velocity += dodge_spin`, and set
       `Some(DodgeFlip { rel_torque: (-norm_roll, dodge_forward), elapsed:
       0.0 })` — `flipRelTorque = (-dodgeDir.y, dodgeDir.x)` symbol for
       symbol, with `dodge_forward = -norm_pitch` as `FR-079` already
       forms it.
    3. **Each airborne step**, before the stick air-control block: if
       `elapsed < FLIP_TORQUE_TIME`, compute `pitch_scale` (flip cancel:
       `rel.1 != 0 && pitch != 0 && pitch.signum() == rel.1.signum()` →
       `1 - |pitch|`, else `1`), apply
       `car.apply_angular_acceleration((forward * (rel.0 * FLIP_TORQUE_X)
       + right_axis * (rel.1 * pitch_scale * FLIP_TORQUE_Y)) /
       tick_scale)` with `tick_scale = dt / (1/120)` and **no**
       `CAR_TORQUE_SCALE` (documented as the reference's own omission, not
       an oversight), and skip the stick air-control block and the
       landing-orientation assist unless cancelling (then yaw/roll only).
       The existing `clamp_angular_speed` after `integrate_velocities`
       supplies the cap-and-hold with no change. If `FLIP_TORQUE_TIME ≤
       elapsed < FLIP_TORQUE_TIME + FLIP_PITCHLOCK_EXTRA_TIME`, run stick
       air control with the pitch term zeroed. Apply `FLIP_Z_DAMP` to
       `car.linear_velocity.z` per rule 5 above. Advance `elapsed += dt`
       at the end of the step (RocketSim increments in
       `_UpdateDoubleJumpOrFlip`, after `_UpdateAirTorque`).
    4. **Flip cancel** replaces `FR-016`'s second-press outright zero with
       the real continuous, proportional, pitch-only mechanism above; a
       second airborne press with the double jump spent does nothing.
    5. **Constants**, all real (`RLConst.h`): `FLIP_TORQUE_X = 260.0`,
       `FLIP_TORQUE_Y = 224.0`, `FLIP_TORQUE_TIME = 0.65`,
       `FLIP_Z_DAMP_120 = 0.35`, `FLIP_Z_DAMP_START = 0.15`,
       `FLIP_Z_DAMP_END = 0.21`, `FLIP_PITCHLOCK_EXTRA_TIME = 0.3`,
       `FLIP_INITIAL_VEL_SCALE = 500.0` (replacing `DODGE_SPEED`).
       `DODGE_ANGULAR_SPEED` is removed, and `MAX_CAR_ANGULAR_SPEED`'s
       `FR-057` note about being "numerically equal, by coincidence" to it
       goes with it. Should the implementation confirm this port never
       adopted `FLIP_BACKWARD_IMPULSE_SCALE_X = 16/15` alongside `FR-059`'s
       other scales, that goes in too.
    6. **Suggested sequencing**, each independently measurable against
       the fixture: (a) `DODGE_SPEED → FLIP_INITIAL_VEL_SCALE` alone (a
       constant change; expected to remove most of the `≈1030` uu/s
       dodge-tick velocity jump); (b) the flip state, torque, vertical
       damping, pitch lock, and air-control lockout; (c) the real flip
       cancel replacing `FR-016`'s.
  - **Step (a), implemented: the real dodge impulse magnitude.**
    `drive::DODGE_SPEED` is now RocketSim's own `FLIP_INITIAL_VEL_SCALE =
    500.0` (from `1400.0`; the name is kept, per this port's convention of
    its own names citing the real one), and the backward dodge's
    forward-axis component now also carries the real
    `FLIP_BACKWARD_IMPULSE_SCALE_X = 16/15` (`DODGE_BACKWARD_SCALE_X`,
    multiplied on top of `DODGE_BACKWARD_SPEED_SCALE`'s speed ramp exactly
    as `_UpdateDoubleJumpOrFlip` does, so it applies at a standstill too)
    — the one scale from that block `FR-059` had not adopted, confirmed
    absent by grep before adding. `DODGE_SPEED`'s doc comment now records
    why `FR-031`'s "false precision" reasoning never applied to it (a
    mass-independent velocity change) and the `~1%` real-capture
    confirmation. Both dodge blocks (ground and wall-jump) changed
    identically. Nothing else about the dodge changed in step (a) — the
    instant spin kick and `DODGE_ANGULAR_SPEED` were step (b)'s, and
    `FR-016`'s flip cancel is step (c)'s.
    - **Real-data effect, measured alone.** Isolated fixture, whole run:
      `cars.mean_position_distance` `≈937` → `≈573` uu (`-39%`), mean
      velocity distance `≈1369` → `≈744` uu/s, max position `≈2606` →
      `≈2005` uu, mean rotation `1.39` → `1.28` rad; `mean_ball_distance`
      unchanged at `≈730`. At `0.05`s windows, the window containing the
      dodge tick (`t = 4.32`s) drops from `≈1032` to `≈126` uu/s mean
      velocity distance — the `≈1030` uu/s jump `FR-079` left at the dodge
      was almost entirely the `1400` placeholder. The remaining divergence
      now grows steadily *after* the dodge (`0.13 → 1.14` rad over the
      following `0.4`s is unchanged), which is exactly the spin-rate
      mismatch steps (b)/(c) address, plus the post-window decay
      (`FR-071`).
    - **Tests.** `a_backward_dodge_scales_up_with_current_forward_speed`
      now expects the `16/15` factor on top of the speed ramp; one new
      test, `a_backward_dodge_at_a_standstill_still_carries_the_real_16_15_forward_factor`,
      pins the standstill case (`rb_physics_bullet`: 336 → 337). Every
      other `DODGE_SPEED` assertion is symbolic and passed unchanged.
      `rb_verify_cli`'s ratchet tightened to `cars.mean_position_distance
      < 600` uu. Full workspace `fmt`/`clippy`/`test` green (398 tests).
  - **Step (b), implemented: the real flip state, torque, vertical
    bleed, pitch lock, and air-control lockout.** Exactly the proposed
    design above, read against RocketSim's `_UpdateAirTorque` and
    `_UpdateDoubleJumpOrFlip` once more while writing it:
    1. **State.** New `pub struct drive::DodgeFlip { rel_torque: (f32,
       f32), elapsed: f32 }` (`Copy`, `PartialEq`); `apply_driven_forces`'s
       `dodge_flip_active: &mut bool` became `dodge_flip: &mut
       Option<DodgeFlip>` and `PhysicsWorld::car_dodge_flip_active:
       Vec<bool>` became `car_dodge_flip: Vec<Option<DodgeFlip>>`
       (defaulting `None`; `from_frame`'s hidden-state caveat unchanged).
       Both dodge blocks set `Some(DodgeFlip { rel_torque: (-norm_roll,
       dodge_forward), elapsed: 0.0 })` — `flipRelTorque = (-dodgeDir.y,
       dodgeDir.x)` symbol for symbol — and no longer touch
       `angular_velocity` at all; the plain double jump clears it, and so
       now does any `on_ground` step (RocketSim's `isOnGround` reset). A
       wall touch leaves it alone, as RocketSim's `numWheelsInContact < 3`
       does.
    2. **Torque.** Every airborne step with `elapsed < FLIP_TORQUE_TIME`
       applies `apply_angular_acceleration((forward * rel.0 *
       FLIP_TORQUE_X + right * rel.1 * FLIP_TORQUE_Y) / tick_scale)`,
       `tick_scale = dt * 120`, with **no** `CAR_TORQUE_SCALE` (documented
       on `FLIP_TORQUE_X` as the reference's own omission) — before the
       stick air-control block, the same order as RocketSim (the dodge's
       own step therefore still gets that step's ordinary air-control
       pitch, ≈`0.1` rad/s, and the flip torque starts on the *next* step
       with `elapsed == dt`). The existing `clamp_angular_speed` supplies
       the cap-and-hold unchanged.
    3. **Lockout and pitch lock.** While the torque applies, the stick
       air-control block and this port's own landing-orientation assist
       are skipped entirely (`doAirControl = false`); for
       `FLIP_PITCHLOCK_EXTRA_TIME` after it, air control runs with the
       pitch term zeroed (`pitchTorqueScale = 0`), yaw and roll normal.
       The assist's "no active stick input" gate reads the raw stick, so a
       pitch held through the lock still counts as active input.
    4. **Vertical bleed and clock.** At the end of every airborne call,
       `elapsed += dt`, then while `FLIP_Z_DAMP_START ≤ elapsed ≤
       FLIP_TORQUE_TIME` and (`vz < 0` or `elapsed < FLIP_Z_DAMP_END`),
       `linear_velocity.z *= (1 - FLIP_Z_DAMP_120)^tick_scale` — the tail
       of `_UpdateDoubleJumpOrFlip`, before this step's own
       `integrate_velocities`, so gravity lands after the bleed exactly as
       in the reference. `elapsed` keeps counting past the window until
       landing.
    5. **Constants**, all real and named as in `RLConst.h`:
       `FLIP_TORQUE_X = 260.0`, `FLIP_TORQUE_Y = 224.0`, `FLIP_TORQUE_TIME =
       0.65`, `FLIP_PITCHLOCK_EXTRA_TIME = 0.3` (public), `FLIP_Z_DAMP_120 =
       0.35`, `FLIP_Z_DAMP_START = 0.15`, `FLIP_Z_DAMP_END = 0.21`,
       `FLIP_REFERENCE_TICK_RATE = 120.0` (private). `DODGE_ANGULAR_SPEED`
       is removed, with `MAX_CAR_ANGULAR_SPEED`'s "coincidence" note
       replaced by why the cap is now load-bearing for the flip.
       `FLIP_TORQUE_MIN_TIME = 0.41` and `FLIP_PITCHLOCK_TIME = 1.0` are
       declared in `RLConst.h` but referenced nowhere in `Car.cpp`, so this
       port doesn't carry them (recorded on `FLIP_TORQUE_TIME`).
    6. **`FR-016`'s flip cancel, kept for now** as this port's own
       interim: a further fresh press still zeroes `angular_velocity`, and
       now also clears the `DodgeFlip` (ending the remaining torque, pitch
       lock, and bleed) and retracts the flip acceleration already
       accumulated that same step — found by the rewritten cancel tests,
       which otherwise saw one tick of spin (`-1.87` rad/s) reappear right
       after the zeroing. Step (c) replaces this trigger with the real
       pitch-hold scale.
    - **Real-data effect, measured alone.** Isolated fixture, whole run:
      `cars.mean_position_distance` `≈573` → `≈259` uu (`-55%`), mean
      velocity distance `≈744` → `≈377` uu/s, max position `≈2005` →
      `≈528` uu, max velocity `≈1541` → `≈1019` uu/s, mean rotation
      `1.28` → `1.22` rad; `mean_ball_distance` unchanged at `≈730`. At
      `0.05`s windows the dodge-tick window is unchanged (`≈126` uu/s) and
      the pre-dodge windows are untouched (`0.03` rad). What remains is
      now visible in shape: the rotation gap grows roughly linearly
      *inside* the flip window (`0.05` → `1.33` rad from `t = 4.32` to
      `4.97` s, about `2` rad/s of mismatch while both `|ω|` traces are
      pinned at `5.5`), then shrinks again, and the velocity gap only
      starts growing after the window (`≈92` → `≈524` uu/s by `t = 5.57`
      s). A rotation-rate mismatch at a pinned magnitude is an *axis*
      mismatch — which is what the fixture's pitch stick, held in the
      flip's own pitch sign, would produce through the real flip cancel's
      `pitchScale = 1 - |pitch|` (zeroing the torque's pitch component,
      leaving only its roll component to pin the cap) — so step (c) is the
      next measurement, with `FR-071`'s damping behind it for the
      post-window growth.
    - **Tests.** `rb_physics_bullet` 337 → 345. Rewritten: the three
      dodge-spin tests (now assert the dodge step's own air-control-only
      spin, then exactly `FLIP_TORQUE_Y / 120` or `-FLIP_TORQUE_X / 120`
      after one tick, with the `DodgeFlip` value pinned), the wall-jump
      dodge test and the five flip-cancel tests (spin appears one step
      later; the stale-flip regression guard now routes through a wall
      touch, since landing clears the flip itself), plus their three
      `world.rs` counterparts. New (`drive.rs`): cap reached on the third
      tick and held within `1e-3` through step 76 then no torque past the
      window; per-tick invariance at `1/60` vs `1/120`; air control and
      the landing assist locked out while flipping; pitch locked for
      `0.3` s after with yaw working, then pitch back; the vertical bleed
      absent before `0.15` s, `×0.65` per tick inside, upward-only exempt
      after `0.21` s, absent after `0.65` s; landing clears the state and
      its torque; a wall-jump dodge restarts it. New (`world.rs`): under
      real gravity, `|ω|` held at the cap mid-flip and `vz` settled within
      `0.05` of `-(650/120)/0.35 = -15.476` uu/s at `0.5` s, then free
      fall resuming after the window. `rb_verify_cli`'s ratchet tightened
      to `cars.mean_position_distance < 300` uu. Full workspace
      `fmt`/`clippy`/`test` green (406 tests).
  - **Step (c), implemented: the real flip cancel — and two corrections
    the capture forced.** The cancel itself is `FR-070`'s mechanism, symbol
    for symbol: while the flip torque applies, if the flip's pitch-axis
    component (`rel_torque.1`) is non-zero and `input.pitch` is held in
    the same sign, that component alone is multiplied by `1 - |pitch|` for
    that step (a forward flip, `rel_torque.1 = +1`, is cancelled by pulling
    back); the roll component is never touched, so a roll-only dodge is
    immune and a diagonal one keeps rolling under a full cancel. `FR-016`'s
    jump-press cancel branch is gone: a further fresh press mid-flip with
    the double jump spent now does nothing (RocketSim's `hasFlipped`). The
    plain-double-jump clear of a stale flip stays (the wall-touch route).
    Measured alone, the cancel changed nothing inside the fixture's flip
    window — the recorded pitch (`-1`, the flip's own direction) never
    meets the sign gate — and only trimmed the whole-run figures slightly
    (`cars.mean_position_distance` `≈259` → `≈259`, mean rotation `1.22`
    → `1.14` rad, mean velocity `≈377` → `≈339` uu/s, from later in the
    run). So the in-window rotation gap step (b) left (`0.05 → 1.33` rad
    at a pinned `|ω|`) was *not* the cancel, and this step ran it to
    ground with two tick-level comparisons against the recording:
    1. **Yaw/roll air control (and damping) are live during the flip;
       only pitch is locked.** Both RocketSim (`doAirControl = false`
       while `isFlipping`, unless cancelling or stalled) and RLUtilities
       (`aerial_control`: dodge torque *instead of* air control for
       `torque_time`) lock all stick air control out during the flip.
       The capture does not: on the first flip tick the recorded body-frame
       `Δω` is `(+1.75, +1.30, +0.03)` (forward, right, up) where the pure
       flip torque predicts `(+1.53, +1.32, 0)`; the differences are, to
       the recording's two decimals, the held `roll = -1`'s air-control
       torque (`+0.32`) minus roll damping (`-0.10`), pitch damping
       (`-0.02`), and yaw damping (`+0.03`) — three independent axes. Run
       forward from each recorded tick with the recorded orientation and
       input over all 77 in-window ticks: pure flip torque (the references)
       mispredicts the next tick's `ω` by `0.102` rad/s rms; flip torque +
       yaw/roll stick torque + `CAR_AIR_CONTROL_DAMPING = (30, 20, 50)`
       with pitch zeroed by `0.0025` rms (`0.0026` max — the rounding
       floor of a two-decimal recording); flip torque + air control with
       pitch *not* zeroed by `0.082`. The port now keeps yaw/roll air
       control live through the flip (pitch locked as before, the landing
       assist still off), matching the capture over the references. The
       damping half of that finding is `FR-071`'s, now pinned to real data
       and next in line.
    2. **The angular-speed clamp belongs after the transform integration.**
       Before the flip the recorded orientation advances at exactly its
       recorded `|ω|` per tick (`3.06`/`3.07`, `5.22`/`5.22`); through the
       flip window it advances at `7.58` rad/s per tick while `|ω|` reads
       `5.50` — and the simulation, with the same `ω`, advanced at `5.50`.
       The `2.08` rad/s difference is exactly the growth rate of the
       in-window rotation gap, and exactly this tick's `Δω`: RocketSim's
       `Arena::Step` calls `stepSimulation` (which integrates the transform
       with the unclamped `ω + Δω`) *before* `Car::_FinishPhysicsTick`
       clamps `ω` to `CAR_MAX_ANG_SPEED` (fetched `Arena.cpp` lines
       `718`–`722`: `stepSimulation`, then `_PostTickUpdate`, then
       `_FinishPhysicsTick`, for cars and then the ball). So the stored
       angular velocity is capped, but the car *turns* by `|ω_stored + Δω|`
       every tick — `≈7.6` rad/s under flip torque. This port clamped in
       `drive_and_integrate_velocities`, before its own contact solve and
       transform integration (`FR-057`'s original placement), under-rotating
       every flip by `≈2` rad/s. `drive::clamp_angular_speed` now runs at
       the end of `PhysicsWorld::step`, after
       `integrate_transform_and_refresh_inertia`, for each car; the
       simulated rotation rate through the window is now `7.6`–`7.75`
       rad/s. (`FR-057`'s "same-step contact impulse isn't re-clamped until
       next step" caveat is gone with the move.) Adjacent finding, not
       acted on: `FR-061`'s ball clamp is documented as "before the
       transform integrates — matching `_FinishPhysicsTick`", but the same
       `Arena.cpp` lines show `ball->_FinishPhysicsTick` after
       `stepSimulation` too; the ball rarely sits at its caps, so the
       placement is left as is and noted here for `FR-061`'s own follow-up.
    - **Real-data effect, measured together.** Isolated fixture, whole
      run: `cars.mean_position_distance` `≈259` → `≈237` uu (`-9%`), max
      `≈528` → `≈459` uu, mean velocity distance `≈339` → `≈254` uu/s;
      `mean_ball_distance` unchanged at `≈730`. Inside the flip window the
      rotation gap is now `0.03 → 0.10` rad (from `0.05 → 1.33`) and the
      velocity gap `≈92` uu/s throughout — the flip itself matches the
      recording. The whole-run mean rotation distance rose, `1.14` → `1.51`
      rad, and honestly so: the recording's spin decays at `≈3.9` rad/s
      from `t ≈ 4.98` (pitch locked, so pure `FR-071` damping) while this
      port's keeps spinning at `5.5` with no angular damping at all, so
      the simulated car now reaches the ground (`t ≈ 5.32`) at a very
      different orientation and the velocity gap jumps to `≈700` uu/s
      there; step (b)'s under-rotation had been masking that by
      coincidence. That post-window decay is the next gap, and the
      77-tick fit above already pins its constants.
    - **Tests.** `rb_physics_bullet` 345 → 350. Rewritten: the five
      `FR-016` tests (a second press mid-flip now does nothing and leaves
      the torque running; a wall-jump dodge's flip and the stale-flip
      guard adapted) and their three `world.rs` counterparts (cancel by
      holding pitch, resuming on release; landing and a later plain double
      jump leave no flip torque). New (`drive.rs`): full pull-back zeroes a
      forward flip's pitch torque, half halves it, release restores it;
      pitch held *with* the flip doesn't cancel, and a backward flip
      cancels on push-forward; a roll-only dodge ignores pitch entirely;
      yaw works mid-flip with pitch locked, cancel or not; a diagonal
      flip's full cancel leaves its roll component; roll air control adds
      to the flip torque mid-flip while the landing assist stays off.
      `rb_verify_cli`'s ratchet tightened to `cars.mean_position_distance
      < 250` uu. Full workspace `fmt`/`clippy`/`test` green (411 tests).
  - **Blast radius (as scoped).** `drive.rs`: the three dodge-spin tests
    (`dodge_gives_forward_velocity_and_spin_when_pitched_in_the_air`,
    `dodge_gives_lateral_velocity_and_spin_when_rolled_in_the_air`,
    `a_yaw_only_press_fires_a_sideways_dodge_like_roll`) assert
    `±DODGE_ANGULAR_SPEED` after one step and become "`±1.87`/`±2.17`
    rad/s after one tick, at the cap after three"; every `DODGE_SPEED`
    velocity assertion is symbolic and survives the constant change; the
    five flip-cancel tests (`a_second_jump_press_cancels_a_dodges_spin`,
    `flip_cancel_does_not_touch_linear_velocity_or_the_double_jump_resource`,
    `a_plain_double_jump_clears_a_stale_dodge_flip_flag_from_an_earlier_dodge`,
    `wall_jump_still_takes_priority_over_flip_cancel_when_touching_a_wall`,
    `a_wall_jump_dodges_spin_can_be_flip_cancelled`) are rewritten for the
    real pitch-hold cancel. `world.rs`: the three live flip-cancel tests
    likewise; the two live dodge tests survive symbolically. New tests:
    cap reached within three ticks and held through `0.65` s, then the
    torque stops; the vertical-damping window and its `-15.5` uu/s
    equilibrium under gravity; the `0.3` s pitch lock; flip cancel is
    proportional, pitch-only, and direction-gated (a roll-only dodge can't
    be cancelled); landing and a wall-jump dodge both clear/restart the
    state; per-tick invariance (`60` Hz vs `120` Hz steps give the same
    per-tick `Δω`). `rb_verify_cli`'s ratchet is re-measured and
    tightened after each sequencing step.
  - **Expected real-data effect.** The fixture's post-dodge `|ω|` should
    pin at `5.5` through `t ≈ 4.967` like the recording; the dodge-tick
    velocity jump should fall from `≈1030` to the low hundreds of uu/s;
    what remains afterward is the post-window decay (`FR-071`'s damping,
    the next gap in line) and any residual dodge-direction subtlety.
  - **Non-goals (this requirement).** Did not take on `FR-071`'s
    air-control damping, even though step (c)'s 77-tick fit had pinned the
    mechanism to the recording's own rounding — `FR-071`'s own
    implementation pass adopted it next and closed the post-window decay.
    Does not move `FR-061`'s ball clamp (see the adjacent finding above), nor take
    on `DOUBLEJUMP_MAX_DELAY` (`1.25` s; this port has no
    double-jump/flip timeout at all — an adjacent gap worth its own
    entry), auto-flip/auto-roll (`FR-060`), or the real dodge's small
    upward component beyond what `FLIP_Z_DAMP` implies.
  - **Acceptance criteria (all met by steps (a)–(c), except that
    `DODGE_SPEED` is kept by name, holding the real value, per this port's
    naming convention).** A forward dodge's
    `|ω|` reaches `MAX_CAR_ANGULAR_SPEED` within three `1/120` s steps and
    stays within `1e-3` of it through `FLIP_TORQUE_TIME`, then receives no
    further flip torque; `vel.z` under gravity converges to
    `-(650/120) / (1 - 0.65)` while damped; holding pitch against a
    forward flip's own sign scales only the pitch torque by `1 - |pitch|`;
    a roll-only dodge ignores pitch entirely; `DODGE_ANGULAR_SPEED` and
    `DODGE_SPEED` no longer exist; the isolated fixture's `|ω|` trace
    matches the recording's cap-and-hold window; full workspace
    `fmt`/`clippy`/`test` green with the ratchet tightened.
  - **Verification plan.** The new tests above plus the rewritten
    flip-cancel and dodge-spin tests, the re-measured `--self` /
    `--self-growth 0.05` numbers recorded in `PROJECT-STATUS.md`, and the
    `rb_verify_cli` ratchet tightened to just above the new figure.
- `RB-PHYSICS-001-FR-081` (landing and grounded-phase divergence —
  diagnosis; finding 2, the horizontal dodge impulse, and finding 5, the
  hitbox offset for body-vs-body contact, implemented):
  `FR-071` left the isolated
  `dodge-derailment` fixture matching the recording to within `0.1` rad
  through its entire airborne phase, with the remaining divergence
  starting at the landing (`t ≈ 5.57` s). This pass traced that remainder
  tick by tick, side by side with the recording (position, velocity, spin,
  orientation, inputs, and the ball), and found it is not one grounded
  mechanism but a chain of five, listed here in the order they act. No
  physics changed; this entry is the plan.
  1. **The `≈110` uu/s velocity gap the car carries through the whole
     flight is born in the four ticks after the ground jump, not in the
     air.** At the seed frame (`t = 4.117`) the simulation matches the
     recording exactly. Over the next four ticks the recorded `vx` climbs
     `308 → 322 → 341 → 362 → 385` uu/s (`vy` flat at `≈1132`) while the
     port's stays at `308` — and it never recovers: `389` vs `308` at the
     dodge tick, `1276` vs `1169` at the landing approach. The mechanism
     is the suspension: RocketSim's Octane hangs its wheels on
     `38.755`/`37.055` uu rest-length springs (`CarConfig.cpp`,
     `FRONT/BACK_WHEEL_SUS_REST`), compressed `≈13` uu at rest, so after a
     jump the wheels stay in ground contact for the `≈4` ticks it takes the
     body to rise `≈20` uu and the springs to extend — and the tires keep
     applying throttle and lateral grip meanwhile (the recorded gain,
     `(+77, +1)` uu/s, is throttle along the `57°` heading plus lateral
     grip pulling a `17°` slip angle back in). This port cuts every ground
     force the tick its box leaves the plane (`car_on_ground` is a
     box-vs-plane contact test at the start of the step), so its jump
     leaves with the pre-jump velocity. About three quarters of the
     velocity gap, and with it the `172` uu the car is behind in `x` when
     it reaches the ball.
  2. **The dodge impulse is applied along the car's tilted 3D axes; the
     real one is horizontal.** RocketSim's `_UpdateDoubleJumpOrFlip` builds
     the impulse from `forwardDir2D = GetForwardDir().To2D().Normalized()`
     and `rightDir2D = (-forwardDir2D.y, forwardDir2D.x, 0)`; this port's
     dodge blocks (`FR-059`/`FR-072`) use `forward_axis(car)` and
     `right_axis(car)` unflattened. At the fixture's dodge the car's nose
     is `3°` down with a little roll, so the port's impulse carried `-75`
     uu/s of vertical velocity the real one didn't: recorded `Δv = (620,
     -28, -6)`, port `(612, -3, -75)`; the flattened axes predict `(628,
     -38, 0)`. That is the `13`–`15` uu the port flies lower through the
     whole arc and the remaining `≈10` uu/s of the horizontal gap. A
     one-line fix per dodge block.
  3. **The recorded car hits the ball; the port's never does.** At `t =
     5.758` the recorded car is `162` uu from the ball (`(-74, -121, 15)`
     vs `(0, 0, 93)`), the ball starts moving that same tick and leaves at
     `≈(1590, 2130, 625)` uu/s, and the car's velocity drops by `(-183,
     -158, -66)` with `|ω|` jumping to `5.5` — a car-ball collision, not
     the jump press that coincides with it. The port's car passes `172` uu
     behind in `x` (finding 1) and `≈8` uu lower, and the ball never moves:
     that is why `mean_ball_distance` has read exactly `729.95` uu through
     every fix since `FR-079` — it is `1.24` s of a stationary ball scored
     against a ball flying at `2700` uu/s, and nothing airborne could ever
     have touched it.
  4. **The landing itself: suspension vs. a rigid box.** The recording's
     wheels touch at `z ≈ 41`; over the next `0.13` s the springs take
     `vz` from `-312` to `0` with no bounce, the nose levels (`forward.z`
     `-0.24 → 0.00`), and the car settles at `z = 15.5`–`16` (its rest
     `17.0` minus compression at speed). The port's box catches a corner at
     `z ≈ 34.5`, takes an angular kick to `5.0` rad/s, bounces (`vz` `-200
     → +44`), and hovers at `z ≈ 22` — so when the jump press arrives at
     `t = 5.758` the port's car reads *airborne*, has its double jump
     restored by the earlier contact, and fires a sideways dodge (`yaw =
     +1` at `≈2000` uu/s forward speed: `500 · 1.9 ≈ 950` uu/s of lateral
     velocity) where the recording's grounded car does a ground jump (`vz`
     `-66 → +229`, the `292` uu/s impulse). That dodge is the `≈800` uu/s
     velocity spike at `t ≈ 5.77` s in the growth diagnostic.
  5. **The hitbox is in the wrong place relative to the recorded
     position.** RocketSim's Octane hitbox (`120.507 × 86.6994 × 38.6591`,
     this port's `CAR_HALF_EXTENTS` exactly) is centred `(13.8757, 0,
     20.755)` uu from the car's position in the car's own frame
     (`HITBOX_OFFSETS[OCTANE]`), the same `20.755` the wheel mounts sit at;
     the recorded `position` is that origin, at `z = 17.0` on flat ground,
     so the real hitbox spans `z = 18.4`–`57.1` and starts `13.9` uu ahead
     of the origin. This port centres its box *on* the position: `20.8` uu
     too low (its box bottom is `2.3` uu below the floor at the seed
     frame, so the solver lifts it to a `19.3` rest height the real car
     never has) and `13.9` uu too far back. It changes which face meets
     the ball and at what height, and every ground-contact corner.
  - **What each finding is worth, in order of cost.** (a) Finding 2 is a
    constant-free one-liner per dodge block and closes the altitude gap
    and `≈10%` of the velocity gap alone. (b) Finding 5 is a moderate
    geometry change — the box centre becomes `position +
    rotation · HITBOX_OFFSET`, threaded through `from_frame`, `frame()`,
    the collision routines' use of `RigidBody::position`, and every car
    test's rest height — with no new physics. (c) Findings 1 and 4 are the
    same missing subsystem: wheels on spring-damper suspension with
    longitudinal/lateral tire forces, the `btVehicleRL` model `FR-065`
    (steering) and `FR-066` (handbrake) already established this port's
    single-rigid-box car cannot represent. That is the largest remaining
    piece of `RB-PHYSICS-001` and a multi-requirement project of its own:
    it would replace this port's ground contact for the car, its steering
    torque, its handbrake friction multiplier, and its jump's contact
    cut-off in one model. (d) Finding 3 follows from the others — the ball
    can only be hit once the car arrives where the recording's did.
  - **Suggested sequencing.** Finding 2 first (cheap, isolated,
    measurable on the fixture's altitude and dodge-tick `Δv`), then
    finding 5 (measurable on the rest height and the landing corner
    geometry), then scope the wheel/suspension model as its own entry
    with `FR-065`/`FR-066` folded in. Do not tune any grounded constant
    before then: every grounded number this port has (`STEER_TORQUE`,
    `HANDBRAKE_FRICTION_MULTIPLIER`, the ground contact's friction) sits
    on the wrong mechanism, and the fixture's grounded segment is `1.4` s
    of one car's one landing — the wrong data to fit them to.
  - **Finding 2, implemented: the dodge impulse is horizontal.** New
    `drive::dodge_axes_2d(car) -> (Vec3, Vec3)`: the car's forward
    flattened to the horizontal plane and normalized, and the horizontal
    right `(-forward_2d.y, forward_2d.x, 0)` — RocketSim's `forwardDir2D`/
    `rightDir2D` symbol for symbol — falling back to the 3D axes only for
    a car pointing straight up or down (where the flattened forward has no
    direction; RocketSim's own `Normalized()` is undefined there, this
    port keeps the impulse finite). Both dodge blocks (ground and
    wall-jump) apply their translation impulse along that pair; the flip
    torque keeps the real 3D body axes, as RocketSim's does; the
    backward/side speed scales and `normalize_dodge_direction` are
    untouched. Three new tests (a `30°` nose-down car's forward and side
    dodges are exactly horizontal at full `DODGE_SPEED`; the wall-jump
    dodge likewise; the flattening and the straight-up fallback) —
    `rb_physics_bullet` 350 → 353. Measured alone on the isolated fixture:
    the dodge-tick velocity window `121 → 88` uu/s, the through-flight
    velocity gap `≈113 → ≈87`–`109` uu/s, whole-run mean velocity `≈337 →
    ≈303` uu/s and mean rotation `0.77 → 0.68` rad, max position `≈791 →
    ≈776` uu; `cars.mean_position_distance` unchanged at `≈240` (finding
    1's `≈80` uu/s post-jump gap owns it) and the ball still untouched
    (finding 3). Full workspace `fmt`/`clippy`/`test` green (414 tests);
    the ratchet holds at `< 250` uu.
  - **Finding 5, implemented — with its scoping corrected first.** The
    diagnosis above sequenced the offset as "a moderate geometry change
    measurable on the rest height (`19.3 → 17.0`)". That was wrong: the
    real car rests at `z = 17.0` because its *wheels* hold it there, with
    the hitbox floating `18.4` uu clear of the ground. A box centred on the
    offset with no wheels rests with the car's origin `1.4` uu *below* the
    floor, so a car seeded from a recorded frame at `z = 17.0` would fall
    `18` uu (`0.24` s) before its first step — the fixture's ground jump,
    `0.016` s after the seed, would never fire. The offset everywhere is
    inseparable from the wheel/suspension model (findings 1 and 4). What
    stands alone, and what this pass implements, is the offset where the
    real hitbox is what matters and no wheel is involved: **body-vs-body
    contact**. New `body::CAR_HITBOX_OFFSET = (13.8757, 0, 20.755)`
    (RocketSim's `HITBOX_OFFSETS[OCTANE]`; its `_BulletSetup` mounts the
    `btBoxShape` in a `btCompoundShape` at that transform and computes the
    body's inertia from the box alone, so the centre of mass stays at the
    origin and the inertia tensor is the box's own — exactly what this
    port already has); new `RigidBody::hitbox_offset` (local frame,
    `Vec3::ZERO` by default, set by `standard_car`) and
    `RigidBody::hitbox_center()`; `collision::contacts_between` (ball,
    cars, net points) meets each shape at its `hitbox_center`, while every
    static-surface routine (`contacts_vs_plane` and the rest) keeps the
    shape centred on `position`, the unoffset box standing in for the
    wheel support (underside `19.3` uu below the origin, against the
    wheels' `17.0` at rest) until the suspension replaces it. The solver's
    lever arms are taken from `position`, the centre of mass, whichever
    geometry produced the contact — correct for both. `from_frame` seeds
    the recorded position unchanged and `frame()` reports it unchanged.
    Tests: two in `body.rs` (the mount, and that it rotates with the car),
    two in `collision.rs` (a ball `91` uu from the offset hitbox's
    top-front edge but `107` from the unoffset box's touches only the
    former, from both sides of the pair; two nose-to-nose cars `147` uu
    apart touch at their offset fronts but not unoffset), two in
    `world.rs` (a seeded car carries the offset and keeps its recorded
    position; a car driving into the ball strikes it on the raised
    hitbox); the three sphere-vs-box arithmetic tests that borrowed
    `standard_car` as a box at the origin now use an unoffset real-sized
    `car_box`, which is what they were testing — `rb_physics_bullet` 353
    → 359. Measured on the isolated fixture: no change at all
    (`239.55 uu / 0.68 rad / 302.85 uu/s`), as expected — its car never
    reaches the ball (finding 3) and static contact is untouched; the
    change is correctness for every future ball or car contact. Full
    workspace `fmt`/`clippy`/`test` green (420 tests).
  - **Non-goals (this requirement).** Changes no constant. Does not
    implement findings 1, 3, or 4, nor the offset against static surfaces
    (see finding 5's correction above — that is the suspension's). Does
    not touch
    `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on
    `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** This entry records the five findings with
    the tick-level evidence for each, their cost ranking, and the
    sequencing; a pitched car's dodge impulse is exactly horizontal at
    full `DODGE_SPEED` in both dodge paths; a ball or car meets a
    `standard_car` at the real hitbox mount while the car still rests on
    the unoffset box; `PROJECT-STATUS.md`'s Next item points at scoping
    the wheel/suspension model as the next step.
  - **Verification plan.** The three finding-2 and six finding-5 tests,
    the re-measured `--self` / `--self-growth 0.05` numbers in
    `PROJECT-STATUS.md`, and the ratchet; the full workspace stays green
    (420 tests).
- `RB-PHYSICS-001-FR-082` (wheel/suspension/tire model — scoped; steps
  (a), (b) and (c) implemented): `FR-065`/`FR-066` established that real steering
  and real handbrake friction live in a wheeled-vehicle model this port's
  single rigid box cannot express, and `FR-081` traced everything left in
  the isolated `dodge-derailment` fixture after the airborne phase
  (findings 1, 3, 4, and the static half of 5) to the same missing
  subsystem. This entry scopes adopting it: the complete real mechanism
  as read from RocketSim's `btVehicleRL.cpp`/`.h`, `Car.cpp`
  (`_PreTickUpdate`, `_BulletSetup`, `_UpdateWheels`, `_UpdateJump`,
  `_UpdateAutoRoll`), `CarConfig.cpp`, and `RLConst.h`; what its constants
  predict and what the real capture confirms; the proposed design for
  this port; the blast radius; and a three-step sequencing — so the
  implementation can start from a settled plan, as `FR-080`'s did. No
  physics changed.
  - **The real mechanism, complete.**
    1. **Tick order** (`_PreTickUpdate`): `updateVehicleFirst` (each
       wheel's world transform, its raycast, and its friction impulse are
       computed but not applied) → `wheelsWithContact` and `isOnGround =
       numWheelsInContact >= 3` → `forwardSpeed = v · forward` →
       `_UpdateWheels` (handbrake value, engine/brake force, steer angle,
       per-wheel friction factors, sticky force) → air torque and flip if
       fewer than three wheels touch, else `isFlipping = false` →
       `_UpdateJump` → `_UpdateAutoFlip` → `_UpdateDoubleJumpOrFlip` →
       `_UpdateAutoRoll` when throttle is held and one to three wheels (or
       the chassis) touch → `updateVehicleSecond` (suspension impulses,
       then the friction impulses) → `_UpdateBoost`. Every vehicle impulse
       lands on the rigid body *before* Bullet's `stepSimulation`, i.e.
       before the contact solver sees the body — the same position this
       port's `apply_driven_forces` already occupies.
    2. **Geometry** (`_BulletSetup`, `CarConfig.cpp`, Octane): four
       wheels at `FRONT_WHEELS_OFFSET = (51.25, ±25.90, 20.755)` and
       `BACK_WHEELS_OFFSET = (-33.75, ±29.50, 20.755)` uu in the car's
       frame (the same `20.755` the hitbox is mounted at, finding 5),
       radii `12.5` front / `15.0` back, ray direction `(0, 0, -1)` and
       axle `(0, -1, 0)` in the car's frame, and spring rest lengths
       `FRONT/BACK_WHEEL_SUS_REST = 38.755 / 37.055` **minus**
       `MAX_SUSPENSION_TRAVEL = 12` → `26.755 / 25.055` uu. Per wheel:
       `SUSPENSION_STIFFNESS = 500`, `WHEELS_DAMPING_COMPRESSION = 25`,
       `WHEELS_DAMPING_RELAXATION = 40`, `SUSPENSION_FORCE_SCALE_FRONT =
       35.75`, `SUSPENSION_FORCE_SCALE_BACK = 54.265`, no force cap. A
       correction to `FR-081` finding 1's wording: the springs are not
       "compressed `≈13` uu at rest" — the spring's own rest is the
       `26.755` figure and it sits `≈1.5` uu compressed; the `12` uu is the
       *travel* the ray reaches beyond rest, which is what keeps the
       wheels in contact after a jump (below).
    3. **Raycast** (`btVehicleRL::rayCast`, every tick per wheel): from
       the mount, along the car's down axis, length `rest + travel +
       radius - SUSPENSION_SUBTRACTION (0.05)` = `51.205` front /
       `52.005` back uu, against the static scene (and other bodies; the
       chassis itself is excluded). On a hit: `suspensionLength` = (hit
       distance along the car's up) `- radius`, clamped to `rest ± 12`;
       the contact normal is the surface's; `inv = 1 / (normal · up)` and
       `relVel = (normal · velocity_at_contact) · inv` when `normal · up
       > 0.1` (else `relVel = 0`, `inv = 10`); against a *static* object,
       when the hit is shorter than `rest_unsubtracted + radius - 0.05`
       (the wheel bottom is through the surface), an `extraPushback`
       impulse is taken from Bullet's `resolveSingleCollision` with the
       shortfall as penetration and divided by the wheel count. No hit:
       `suspensionLength = rest + 12`, `relVel = 0`, normal `= -down`.
    4. **Suspension** (`updateSuspension`, applied in
       `updateVehicleSecond`): per touching wheel `force = (rest -
       suspensionLength) · 500 · inv - damping · relVel`, `damping = 25`
       when compressing (`relVel < 0`) else `40`, times the front/back
       force scale, floored at `0` ("RL never uses downwards suspension
       forces"); applied as the impulse `normal · (force · dt +
       extraPushback)` at the contact point (lever arm from the centre of
       mass, so it also torques the body). Units: RocketSim runs these in
       Bullet units (`uu / 50`) with the same `180` mass, and because
       `a = k · Δx / m` scales identically on both sides, every stiffness,
       damping, and force-scale number above is unit-invariant when
       treated as *acceleration per uu of compression per unit mass* —
       this port can use them in uu unchanged.
    5. **Tire friction** (`calcFrictionImpulses`, applied in
       `applyFrictionImpulses`): `frictionScale = mass / 3 = 60`. Per
       touching wheel: the axle direction (including the steer angle) is
       projected onto the surface and normalized; `forward = normal ×
       axle`; `sideImpulse` from Bullet's `resolveSingleBilateral` along
       the axle (the impulse that would zero the contact point's lateral
       velocity through the contact's effective mass, i.e. ideal lateral
       grip); rolling term: engine held → `-engineForce / frictionScale`;
       no engine but brake → `clamp(-relVel_forward · 113.73963, ±brake)`
       (`ROLLING_FRICTION_SCALE_MAGIC`, RocketSim's own "no idea where
       this number comes from"); neither → `0`. `impulse = (forward ·
       rolling · longFriction + axle · side · latFriction) ·
       frictionScale`, applied `× dt` at the contact offset with its
       component along the car's up removed (so tire forces never pitch
       the body about the contact height). The magnitudes check out
       against numbers this port already has: `THROTTLE_TORQUE_AMOUNT =
       180 · 400` per wheel over four wheels is `1600` uu/s²
       (`THROTTLE_ACCELERATION`, `FR-007`), `BRAKE_TORQUE_AMOUNT = 180 ·
       (14.25 + 1/3)` per wheel through `frictionScale` over four wheels
       is `3500` uu/s² (the real brake deceleration), and the proportional
       band of the brake (`|relVel| < 52.5 / 113.74` Bullet units) is
       `≈23` uu/s — `STOPPING_FORWARD_VEL = 25` to within rounding.
    6. **`_UpdateWheels`, every tick.** Handbrake is an analog
       `handbrakeVal` rising at `POWERSLIDE_RISE_RATE = 5`/s and falling at
       `POWERSLIDE_FALL_RATE = 2`/s, clamped `[0, 1]`. Throttle: boost with
       boost left forces `throttle = 1`; unless handbraking, throttle below
       `THROTTLE_DEADZONE = 0.001` means *coasting* — engine `0`, brake
       `COASTING_BRAKE_FACTOR = 0.15`, or full brake `1` below
       `STOPPING_FORWARD_VEL = 25` uu/s; throttle against the direction of
       travel above `25` uu/s means full brake and (above
       `BRAKING_NO_THROTTLE_SPEED_THRESH = 0.01`) engine `0`. `engineForce
       = throttle · THROTTLE_TORQUE_AMOUNT · DRIVE_SPEED_TORQUE_FACTOR_CURVE
       (|forwardSpeed|)` — `FR-058`'s taper, exactly — divided by `4` when
       fewer than three wheels touch; `brake = realBrake ·
       BRAKE_TORQUE_AMOUNT`; both set on all four wheels. Steering: the
       front two wheels get `steer · angle(|forwardSpeed|)` with
       `STEER_ANGLE_FROM_SPEED_CURVE = {0: 0.53356, 500: 0.31930, 1000:
       0.18203, 1500: 0.10570, 1750: 0.08507, 3000: 0.03454}` rad blended
       toward `POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE = {0: 0.39235, 2500:
       0.12610}` by `handbrakeVal` (`FR-065`'s curve, and its inverted
       shape relative to this port's `speed_factor`). Friction factors per
       touching wheel: with `lat = |v_contact · latDir|` (`v_contact = ω ×
       mount_offset + v`) and `long = |v_contact · longDir|`, the curve
       input is `lat / (long + lat)` when `lat > 5` uu/s else `0`;
       `latFriction = LAT_FRICTION_CURVE {0: 1.0, 1: 0.2}`, `longFriction =
       LONG_FRICTION_CURVE` (empty → `1`); if `handbrakeVal > 0`, `lat *=
       1 + (0.1 - 1) · handbrakeVal` and `long *= 1 + (LONG_FACTOR(input)
       - 1) · handbrakeVal` with `HANDBRAKE_LONG_FRICTION_FACTOR_CURVE =
       {0: 0.5, 1: 0.9}` (`FR-066`'s anisotropy), else `longFriction = 1`;
       and when throttle is zero (not "sticky") both are scaled by
       `NON_STICKY_FRICTION_FACTOR_CURVE {0: 0.1, 0.7075: 0.5, 1: 1.0}` of
       the contact normal's `z`. Sticky force, whenever at least one wheel
       touches the world: `applyCentralForce(upwardsDir · scale · g ·
       mass)` with `upwardsDir` the normalized sum of the touching wheels'
       contact normals, `g = -650`, and `scale = 0.5 + (1 - |upwardsDir.z|)`
       when throttle is held or `|forwardSpeed| > 25`, else `0.5` — half a
       g pressed into the floor at all times on the ground, a full g into
       a vertical wall when driving on it.
    7. **Jump** (`_UpdateJump`): `isOnGround` (three or more wheels) is
       the jump's precondition and what resets `hasJumped` (with a
       `JUMP_RESET_TIME_PAD = 1/40` s guard after a minimum-time jump so
       the wheels' lingering contact cannot reset it early); the impulse
       `JUMP_IMMEDIATE_FORCE = 875/3` and the hold force `JUMP_ACCEL =
       4375/3` (`× 0.62` before `JUMP_MIN_TIME`) are along the **car's own
       up axis**, not world `z` — identical on flat ground, different on
       a wall or a curve. `FR-010`/`FR-015`'s constants are all confirmed;
       only the direction and the ground test differ.
    8. **Auto-roll** (`_UpdateAutoRoll`, `CAR_AUTOROLL_FORCE = 100`,
       `CAR_AUTOROLL_TORQUE = 80`): with throttle held and one to three
       wheels touching (or a chassis-world contact), a central force of
       `100 · mass` toward the ground (along the averaged wheel-contact
       normal) and an inertia-cancelled torque of `80` toward aligning the
       car's up with it. This is the real counterpart of `FR-018`'s
       retired landing assist — and it needs partial wheel contact and
       throttle, so it could not exist before the wheels do.
    9. **The chassis still collides.** The hitbox (a `btCompoundShape` at
       `HITBOX_OFFSETS[OCTANE]`, finding 5) meets the arena with
       `CAR_COLLISION_FRICTION = 0.3` / `CAR_COLLISION_RESTITUTION = 0.1`
       and other cars with `CARCAR_COLLISION_FRICTION = 0.09` /
       `RESTITUTION = 0.1` (`FR-063`); normally the wheels hold it `18.4`
       uu clear of the floor and the box only touches when the car is on
       its roof, its side, or a corner.
  - **What the constants predict, and what the capture confirms.**
    1. **Rest height.** Balancing the four springs against the car's
       weight *plus the half-g sticky force* (`180 · 650 · 1.5`), with the
       front deficit `39.255 - h` and back `40.055 - h` (`h` the mount
       height): `2 · 500 · 35.75 · Δf + 2 · 500 · 54.265 · (Δf + 0.8) =
       175500` gives `Δf = 1.47` uu, `h = 37.79`, origin `z = 17.03` —
       RocketSim's `CAR_SPAWN_REST_Z = 17` and the fixture's recorded
       `17.0` (without the sticky term it would be `17.68`). Every
       constant above is load-bearing and they land on the recorded
       number together.
    2. **Contact after a jump.** The front ray reaches `51.205` uu below
       the mount, so contact persists until the origin is `13.4` uu above
       rest (`z ≈ 30.45`; back wheels `31.25`). At the jump's `292` uu/s
       minus gravity, the sticky half-g, and the `0.62`-scaled hold that
       is `≈4`–`5` ticks — the fixture's four ticks of `+77` uu/s
       (`FR-081` finding 1), during which the springs are extended past
       rest (force floored at `0`, so no downward spring pull) and only
       the tires and the sticky force act. `driveSpeedScale / 4` once
       fewer than three wheels touch tapers the gain, as the recorded
       `+14, +19, +21, +23` uu/s steps do not — the ramp-*up* shape is the
       tire's slip curve at a `17°` slip angle, to be checked in step (b).
    3. **The landing.** The recording's wheels touch at origin `z ≈ 41`
       with the nose `14°` down — consistent: the front mounts at `≈48.7`
       uu are inside the `51.2` uu ray. From there the damping term acts
       across the whole `12` uu of extension while the spring only engages
       below rest (`z < 18.5`), which is exactly a soft, no-bounce `0.13`
       s stop (`vz` `-312 → 0`) settling at `15.5`–`16` (rest `17.0` minus
       the extra sticky-force compression at speed) — `FR-081` finding 4,
       and the reason the port's rigid box (corner catch at `34.5`, `5.0`
       rad/s kick, bounce to `+44` uu/s, hover at `22`) cannot be tuned
       into it.
    4. **The ball hit** (finding 3) follows: with the post-jump tire
       forces the car arrives where the recording's does at `t = 5.758`.
  - **What this port does instead.** Support and friction come from the
    unoffset box's corners against the plane (rest `19.3`, finding 5's
    stand-in); `on_ground` is that box touching the ground at the start
    of the step, so every ground force cuts the tick the box lifts;
    `THROTTLE_ACCELERATION` is a central force (right total, wrong point
    of application and no per-wheel gating); steering is `STEER_TORQUE`
    scaled *up* with speed (`FR-065`); handbrake is
    `HANDBRAKE_FRICTION_MULTIPLIER` on the box's isotropic friction
    (`FR-066`); coasting has no brake; there is no sticky force, no
    auto-roll, and the jump is along world `z`. None of these constants
    is wrong in a way tuning could fix; they sit on the wrong mechanism
    (`FR-081`'s "do not tune any grounded constant before then").
  - **Proposed design for this port.**
    1. **Descriptors and state.** `drive::WHEELS: [WheelMount; 4]`
       (`mount`, `radius`, `rest_length` (already minus travel),
       `force_scale`, `is_front`) — Octane values only. Per car, in
       `PhysicsWorld`: `car_wheels: Vec<[WheelState; 4]>` (`in_contact`,
       `contact_point`, `contact_normal`, `suspension_length`,
       `relative_velocity`, `inv_normal_dot_up`, `extra_pushback`,
       `friction_impulse`) and `car_handbrake_value: Vec<f32>`;
       `wheels_in_contact` derived. `frame()` and `PhysicsFrame` are
       unchanged — wheel state is internal, exactly as `car_dodge_flip`
       is. `from_frame` seeds no wheel state; the first step's raycast
       establishes contact, so a car seeded at the recorded `z = 17.0`
       is supported from its first tick (the property finding 5's
       correction needed and could not have).
    2. **Raycast against the static scene.** New
       `collision::raycast_static(origin, direction, max_length,
       &StaticScene) -> Option<RayHit { point, normal, fraction }>`,
       nearest hit across `StaticPlane`, `StaticQuarterPipe`,
       `StaticCornerFillet`, `StaticGoalWall`, `StaticBoundedWall` — each
       shape's own `ray_vs_*`, the same per-shape decomposition
       `contacts_vs_*` already follows. Other bodies are not raycast
       (non-goal below).
    3. **Step order** (`PhysicsWorld::step`): for each car, raycast the
       four wheels from the *start-of-step* transform (replacing the
       `car_on_ground` box test; `car_wall_normal` stays for the wall
       jump until step (c) makes it the wheels' averaged normal) →
       `wheels_in_contact`, `on_ground = wheels_in_contact >= 3` →
       `drive::apply_driven_forces` computes, in the real order, the
       handbrake value, engine/brake/steer, per-wheel friction factors and
       impulses, the sticky force, then the existing air/jump/flip logic
       gated on the new `on_ground` → the suspension impulses and the
       friction impulses are applied to the car (`apply_impulse` at the
       contact offsets, exactly the two `updateVehicleSecond` loops) →
       velocity integration → the contact solve, where the car's static
       contact now uses `hitbox_center()` (finishing finding 5) with
       `CAR_COLLISION_FRICTION`/`RESTITUTION` → transform integration →
       `clamp_angular_speed`. `resolveSingleBilateral` and
       `resolveSingleCollision` are ported from Bullet's
       `btContactConstraint.cpp` (the effective-mass and
       penetration-error formulas `solver.rs` already carries in row
       form; expose them as two small functions) under the existing zlib
       attribution.
    4. **What `drive.rs` loses.** `STEER_TORQUE` and its `speed_factor`,
       `HANDBRAKE_FRICTION_MULTIPLIER`, `car_base_friction` and the
       per-step `car.friction` swap, the central throttle force, and the
       world-`z` jump direction — replaced by the steer-angle curves, the
       handbrake value with its two factor curves, per-wheel engine/brake
       with `drive_speed_taper` reused as `driveSpeedScale`, and the
       car-up jump. `FR-065` and `FR-066` close with it. A small
       `PiecewiseLinear` helper generalizes `DRIVE_SPEED_TAPER_BREAKPOINTS`
       for the six new curves.
    5. **Fixed-point subtlety worth stating now.** The brake's
       proportional band (`113.74 × relVel`) at `120` Hz over-corrects
       (`151.7/s` decay, per-tick factor `-0.26`), which is why RocketSim
       rounds small `relVel` to zero only below `80` tps; at this port's
       `120` Hz the real behaviour is the oscillating one and is kept.
  - **Blast radius.** `world.rs` (`step`'s front half, four new per-car
    vectors threaded through `with_car`/`from_frame`, `static_contact_
    manifolds` reading `hitbox_center()` for cars); `drive.rs` (the
    ground branch rewritten; the air branch, boost, jump hold, flip, and
    air-control damping untouched apart from the jump direction);
    `collision.rs` (five `ray_vs_*` routines and `raycast_static`);
    `body.rs` (wheel descriptors; `standard_car` friction/restitution to
    `0.3`/`0.1`); `solver.rs` (two exposed helpers, no behaviour change).
    Tests: every grounded test in `drive.rs` (`68` references to the
    grounded helpers across its `95` tests) and every `world.rs` test
    that seeds a car at `CAR_HALF_EXTENTS.z` or asserts the `19.3` rest
    height (a dozen) move to the wheel model's numbers — the largest test
    churn of any entry, and unavoidable: their expectations encode the
    stand-in. `rb_domain`, the ingest crates, `PhysicsFrame`, and the
    ball's physics are untouched.
  - **Sequencing.**
    - **(a) Flat-ground wheels, old tire forces.** Descriptors, raycast
      against `StaticPlane` only, suspension with the sticky force, the
      chassis on `hitbox_center()` for static contact, `on_ground` from
      the wheel count, the car-up jump — with throttle, steering, and
      handbrake left on today's direct model but gated on the new
      `on_ground`. Measurable alone: rest height `17.0` (the derivation
      above becomes a test), the fixture's post-jump contact ticks, and
      its landing (`vz` `-312 → 0` in `≈0.13` s, no bounce, no spurious
      airborne read at `t = 5.758`, so the recorded ground jump fires
      instead of the port's sideways dodge — the `≈800` uu/s spike).
    - **(b) Tire friction.** Engine/brake/coast, the lateral bilateral
      impulse, the friction curves and the non-sticky curve, the
      steer-angle curves replacing `STEER_TORQUE`, the handbrake value and
      factor curves replacing `HANDBRAKE_FRICTION_MULTIPLIER`. Measurable:
      the four post-jump ticks' `+77` uu/s, the `172` uu arrival gap, and
      the ball hit (finding 3 — `mean_ball_distance` moves off `729.95`
      for the first time since `FR-079`).
    - **(c) The rest of the arena.** `raycast_static` over the curves,
      fillets, goal walls, and bounded walls; the wall jump's normal from
      the wheels; auto-roll; `extraPushback`. Measurable on wall-driving
      and curve-landing scenarios, not on this fixture.
    Each step keeps the workspace green, re-measures the fixture, and
    tightens the ratchet as the number drops.
  - **Step (a), implemented — with three corrections to the scoping
    above, recorded here.** New `wheels` module (`rb_physics_bullet::
    wheels`): `WheelMount`/`WHEELS` (the four Octane mounts, radii, rest
    lengths minus travel, force scales), the `BTVehicle` and drive
    constants as declared, `WheelState` (contact, contact point and
    normal, spring length, suspension relative velocity, the pushback,
    and the drive fields — engine force, brake force, steer angle,
    lateral/longitudinal friction factors — plus the stored friction
    impulse), `raycast_wheels` (`btVehicleRL::rayCast` against the
    scene's flat planes, the ground and the walls),
    `compute_friction_impulses`/`apply_friction_impulses`
    (`calcFrictionImpulses`/`applyFrictionImpulses`: Bullet's
    `resolveSingleBilateral` side impulse through
    `solver::effective_mass_denom`, now `pub(crate)`, and the
    engine/brake rolling term with `ROLLING_FRICTION_SCALE_MAGIC`),
    `update_wheels` (`_UpdateWheels`: throttle/brake/coast logic, boost
    forcing full throttle, the steer angle, the friction factors, the
    sticky force), `apply_suspension_impulses` (`updateSuspension` and
    its impulse loop), `upwards_dir_from_contacts`, and
    `piecewise_linear` (`LinearPieceCurve::GetOutput`). New
    `collision::ray_vs_plane`/`RayHit`. `PhysicsWorld` carries
    `car_wheels: Vec<[WheelState; 4]>` (readable through `car_wheels`),
    casts the rays at the start of `step` from the start-of-step
    transform in place of the old box-on-plane `car_on_ground` test,
    derives `on_ground` from the wheel count, runs
    `drive_and_integrate_velocities` in RocketSim's `_PreTickUpdate`
    order (friction impulses from the start-of-step velocity and the
    *previous* tick's drive fields — the one-tick lag is RocketSim's,
    since `updateVehicleFirst` precedes `_UpdateWheels` — then this
    tick's drive fields and the sticky force, the driven forces, then the
    suspension and friction impulses as velocity changes ahead of the
    contact solve), and meets the static arena with the chassis at
    `hitbox_center()` through a `static_probe` (finishing `FR-081`
    finding 5, now that the wheels hold the chassis `18.4` uu clear of
    the floor; a no-op for the ball). `drive::apply_driven_forces` lost
    its throttle force, its `STEER_TORQUE` block, and its `car.friction`
    swap — the grounded branch is the jump alone, now along the car's own
    up as `_UpdateJump` fires it — and `THROTTLE_ACCELERATION`,
    `STEER_TORQUE`, `HANDBRAKE_FRICTION_MULTIPLIER`, and
    `PhysicsWorld::car_base_friction` are gone. `THIRD_PARTY_NOTICES.md`
    gains a RocketSim (MIT) section for the first port of RocketSim
    control flow, and a Bullet row for `resolveSingleBilateral` and
    `btRaycastVehicle`.
    1. **The tire mechanism came into step (a).** The plan left throttle,
       steering, and the handbrake "on today's direct model" for step
       (a). That model was the box's Coulomb contact with the floor — and
       the wheels lift the box off the floor, so without tire forces the
       car would have had no grip at all. The per-wheel mechanism
       (bilateral lateral impulse, engine/brake/coast rolling term,
       `frictionScale = mass / 3`, the flattened contact offset) is
       therefore step (a)'s; what step (b) still owns is the *curves*:
       the analog `handbrakeVal` and its lateral/longitudinal factor
       curves (step (a) applies the real lateral `0.1`,
       `HANDBRAKE_LAT_FRICTION_FACTOR`, as a switch), the slip-driven
       `LAT_FRICTION_CURVE`, and the non-sticky curve.
    2. **The steer-angle curve came in too.** Measured with the real
       tires and the old `STEER_TORQUE`, the fixture got *worse*
       (`239.55 → 310.89` uu mean position, a `0.28` rad rotation error
       born in the ticks around the jump): the recorded car yaws faster
       and faster under full steer through those grounded ticks (`ω_z`
       `-1.35 → -1.49 → -1.61` before the jump and `→ -2.02` through the
       four contact ticks after it), and unsteered tires fight any torque
       that imitates that. With `STEER_ANGLE_FROM_SPEED_CURVE` (blended to
       `POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE` while the handbrake is
       held) on the front wheels and `STEER_TORQUE` removed, the same
       ticks match to `0.00` rad. `FR-065` and `FR-066` close here.
    3. **`SUSPENSION_SUBTRACTION` is `0.05` *Bullet* units — `2.5` uu.**
       RocketSim subtracts it from lengths already converted to Bullet
       units. The scoping's `51.2` uu ray is really `48.755` (front) /
       `49.555` (back), contact after a jump ends `11.0` uu above rest
       (origin `z ≈ 28.0` / `28.8`), and — the part that matters — the
       `extraPushback` threshold (`rest + radius - 2.5`) sits `2.5` uu
       *past* rest, so the pushback is a hard stop that never acts at
       rest (the springs sit `≈1.5`–`2.3` uu compressed) and the
       rest-height derivation above stands. Read with `0.05` uu the
       pushback would have engaged at rest and put the car at `z ≈ 18.2`.
       So the pushback is ported in step (a) after all (`PUSHBACK_ERP =
       0.2`, `positional error + approach velocity` through the
       contact's effective mass, floored at zero, shared over the four
       wheels, riding on the suspension impulse only when the spring
       force is nonzero, exactly RocketSim's variant of
       `resolveSingleCollision` that returns without applying): the
       landing test below bottoms out `15.46` with it against the
       recording's `15.54`, and rebounds `+17.5` uu/s against the
       recording's `+14`. `FR-081` finding 4's "no bounce" was an
       overstatement — the recording rebounds from `z = 15.54` to `+14`
       uu/s over five ticks before its jump — and finding 1's "`≈13` uu
       compressed" is corrected under this entry's mechanism, item 2.
    - **Measured.** A `standard_car` seeded at `z = 17.0` on flat ground
      stays within `0.03` uu of it for `2` s on four touching wheels
      (the derivation: `17.03`); a level car dropped through the
      fixture's landing (`-312` uu/s) bottoms at `15.46`, rebounds to
      `+17.5` uu/s, never reads airborne once down, and settles at
      `17.0`; after a ground jump from rest all four wheels keep
      touching for `≥ 4` ticks and the car is airborne within `20`; the
      chassis of a car at rest makes no static contact. On the isolated
      fixture: `239.55 uu / 0.68 rad / 302.85 uu/s → 160.19 uu / 0.44
      rad / 264.09 uu/s` (max `776.31 / 3.11 / 961.41 → 741.26 / 1.78 /
      690.57`), and `mean_ball_distance` `729.95 → 79.55` uu (max
      `3311.68 → 320.66`): **the port's car hits the ball at `t = 5.758`
      for the first time** (`FR-081` finding 3). `--self-growth 0.05`:
      the grounded ticks and the whole flight now match to `0.00`–`0.04`
      rad (from `0.05`–`0.10`) and `1`–`45` uu; the landing reads `53
      uu / 0.01 rad / 27 uu/s` at `t = 5.67` s and `0.02` rad at `5.72`
      — no spurious airborne read, no sideways dodge; the ball leaves
      within `60` uu of the recording and drifts to `320` uu by the end.
      What remains starts *after* the hit: the car's velocity error
      steps `27 → 220` uu/s at the hit itself (`t = 5.77`) and `194 →
      660` uu/s at `t ≈ 6.02`–`6.07` with the rotation climbing to
      `0.9` rad — the recorded ground jump that coincides with the hit,
      the hit's own impulse (`FR-063`'s per-pair car-ball restitution and
      friction are still the defaults), and whatever the recording does
      at `6.0` s are the next diagnosis, ahead of step (b). Without the
      pushback the position figure was `156.62` uu (`0.46` rad, `266.67`
      uu/s); with it `160.19` — the hard stop costs `3.6` uu of mean
      position on this fixture and buys the recorded landing depth.
    - **Tests.** `19` in `wheels.rs` (the rests and ray lengths, the
      mounts, the rest-height contact geometry, contact loss with height,
      the pushback margin, the travel clamp, the relative velocity, the
      suspension impulse arithmetic, extended springs never pulling
      down, `1600` and `3500` uu/s² over four wheels, coasting and the
      brake's proportional band, boost and handbrake on the engine, the
      lateral impulse and the handbrake's tenth, the sticky force on the
      floor and on a wall, the quartered engine under three wheels, the
      steer curve on the front wheels only, a steered car yawing through
      its front wheels, a wall plane seen by a car on it); `4` new
      `world.rs` acceptance tests (rest height, no chassis contact, the
      landing, the post-jump contact ticks), the handbrake world test
      rewritten onto the wheels' lateral factor, and the
      `simulate_recorded` input test rewritten for the one-tick lag;
      `12` `drive.rs` throttle/steer/handbrake tests moved onto a
      resting-wheels helper (`step_on_wheels`). `rb_physics_bullet` `359
      → 382`, the workspace `420 → 443`. The ratchet tightens `< 250 →
      < 165` uu on the car and `< 1000 → < 100` uu on the ball. Full
      workspace `fmt`/`clippy`/`test` green.
  - **Step (b), implemented.** The curves, in `update_wheels`, exactly
    where `_UpdateWheels` has them. (1) The analog handbrake: `PhysicsWorld`
    carries `car_handbrake_val` per car (RocketSim's `handbrakeVal`), and
    `update_wheels` ramps it first — `+POWERSLIDE_RISE_RATE · dt` (`5`/s,
    full in `0.2` s) while the handbrake is held, `-POWERSLIDE_FALL_RATE
    · dt` (`2`/s, gone in `0.5` s) otherwise, clamped to `0..=1` — then
    reads it: the steer angle is `steer · (normal + (powerslide - normal)
    · handbrakeVal)` instead of the switch, and the friction factors are
    blended `factor · ((curve(slip) - 1) · handbrakeVal + 1)`. (2) The
    slip-driven lateral curve: per touching wheel, `latDir` is the wheel
    transform's own axle (the car's right steered about its up, not
    flattened onto the surface) and `longDir = latDir × contactNormal`;
    the *mount's* velocity `v + ω × (hardPoint - origin)` gives `lateral
    = |v_mount · latDir|`, and when that exceeds `LATERAL_SLIP_THRESHOLD =
    5` uu/s the slip ratio is `lateral / (|v_mount · longDir| +
    lateral)`, else `0`. `LAT_FRICTION_CURVE {0: 1, 1: 0.2}` of that
    ratio is the lateral factor; `LONG_FRICTION_CURVE` is empty in
    RocketSim, so `piecewise_linear` now returns `GetOutput`'s default
    `1` for an empty curve. (3) The handbrake factor curves:
    `HANDBRAKE_LAT_FRICTION_FACTOR_CURVE {0: 0.1}` (a constant tenth, the
    switch step (a) applied) and `HANDBRAKE_LONG_FRICTION_FACTOR_CURVE
    {0: 0.5, 1: 0.9}`, blended in by the value; with no handbrake the
    longitudinal factor is forced to `1` ("if we aren't powersliding,
    it's not scaled down"). (4) The non-sticky curve: with `realThrottle
    == 0` (boost with boost left counts as throttle), both factors scale
    by `NON_STICKY_FRICTION_FACTOR_CURVE {0: 0.1, 0.7075: 0.5, 1: 1}` of
    the contact normal's `z` — a coasting car on a wall keeps a tenth of
    its grip. A wheel in the air keeps its last factors, as RocketSim's
    does; nothing reads them there. `HANDBRAKE_LAT_FRICTION_FACTOR` is
    gone (the curve replaces it); `prime_car_wheels` ramps the value once
    at the seed like any other tick. `FR-066`'s remaining pieces — the
    analog ramp and the longitudinal curve — close here.
    - **Measured.** Isolated fixture `117.41 uu / 0.46 rad / 228.81 uu/s
      → 102.64 uu / 0.40 rad / 200.35 uu/s` (max `615.60 / 1.91 / 636.01
      → 533.26 / 1.68 / 552.91`); `mean_ball_distance` `75.22 → 79.28`
      uu (max `361.25 → 377.66`). The growth diagnostic: the grounded
      ticks and the flight unchanged (`0.1`–`23` uu), the landing window
      `23.3` uu / `0.03` rad, and the hit window at `t = 5.77` reads `6.8`
      uu on the ball (from `31`) and `23` uu / `0.06` rad / `38` uu/s on
      the car (from `20` / `0.06` / `79`); the post-`6.05` step is still
      finding 6's (`552` uu/s at `6.07`). Tick by tick: the post-jump
      `vx` gains now read `+9, +10, +11, +11, +11` against the recorded
      `+9, +10, +11, +11, +12, +11` — the same shape the scoping
      predicted the slip curve would give, one tick shorter (`FR-083`
      finding 7: the recording's wheels act through `t = 4.183` where the
      port's ray lets go at `4.175`, and the recording then holds `ω_z =
      -2.02` and `vx = 385` for one more tick with neither tires nor air
      control acting, which the port cannot represent). That tick is the
      whole remaining arrival gap: the port's car is `10` uu/s slower in
      `x` through the flight, `≈17` uu behind at the landing, hits the
      ball on the tick after the recorded `5.758`, and the ball leaves at
      `(1788, 2347, 954)` against `(1602, 2148, 790)` — `11%` fast, on a
      car whose yaw rate at contact reads `1.60` vs `1.49` rad/s. The
      landing ticks show a second residual of the same kind: the
      recording's wheels touch at `t = 5.575` (its `ω_x` starts climbing
      there) where the port's rays touch at `5.583`, and the recorded
      car's yaw rate turns *negative* through the landing (`0.64 →
      -0.49` rad/s over `5.575`–`5.642`, steer held right) while the
      port's stays positive (`0.77 → 0.14`) — worth `0.05` rad of the
      rotation figure and left as read; it is the next diagnosis,
      ahead of step (c).
    - **Tests.** `7` new in `wheels.rs`: `an_empty_curve_reads_one_and_a_
      single_point_reads_that_point_everywhere`, `the_analog_handbrake_
      ramps_up_in_a_fifth_of_a_second_and_down_in_half_a_second`,
      `a_half_engaged_handbrake_blends_halfway_between_the_two_steer_
      curves_and_factors`, `lateral_grip_falls_with_the_mounts_slip_
      ratio_and_ignores_slip_under_the_threshold` (`1` rolling straight,
      `0.2` sliding sideways, `0.6` at half slip, the `5` uu/s
      threshold), `the_handbrakes_longitudinal_factor_rises_with_slip_
      and_the_lateral_stays_a_tenth`, `a_coasting_car_on_a_wall_keeps_a_
      tenth_of_its_grip_and_a_driving_one_all_of_it`, `a_wheel_in_the_
      air_keeps_its_last_friction_factors`; the world handbrake test
      re-pinned to the ramp (`0.9625` one tick in, `0.1` / `0.5` after
      `0.2` s, back to `1` `0.5` s after release); the `drive.rs`
      handbrake tests re-pinned with `step_on_wheels` holding the value at
      its rail and the gripping car's own slip on the curve. `rb_physics_
      bullet` `389 → 396`, the workspace `450 → 457`; the ratchet
      tightens `< 125 → < 110` uu on the car (the ball stays `< 85`).
      Full workspace `fmt`/`clippy`/`test` green.
  - **Step (c), implemented.** (1) **The rest of the arena.**
    `collision::StaticScene` (moved out of `world.rs`, now `pub`) and
    `collision::raycast_static(origin, direction, max, &scene)` — the
    nearest hit across `ray_vs_plane` (the ground and the walls),
    `ray_vs_quarter_pipe` (the far root of the ray-cylinder
    intersection from inside the fillet, accepted within its sector,
    normal toward the axis), `ray_vs_corner_fillet` (the same for the
    sphere, within its `bounds`), `ray_vs_goal_wall` (the plane, a hit
    inside the window discarded) and `ray_vs_bounded_wall` (the plane,
    a hit outside the bound discarded); `raycast_wheels` takes the
    scene, and `PhysicsWorld` builds it for the rays and the priming
    from the same six fields `static_contact_manifolds` reads. A car
    over the standard arena's side-wall floor fillet now rests its
    wheels on the curve with the normal leaning toward the axis. (2)
    **The wall jump's normal from the wheels.**
    `wheels::wall_contact_normal`: the averaged contact normal when one
    or two wheels touch a wall-like surface (`z <
    WALL_CONTACT_MAX_NORMAL_Z = 1/√2`, RocketSim's own
    `CAR_AUTOFLIP_NORMZ_THRESH` read the other way round), `None`
    otherwise; it replaces the chassis-against-a-wall-plane probe as
    `drive`'s `wall_normal`. A car with three or more wheels on a wall
    is `on_ground` there and jumps along its own up — which *is* the
    wall's normal, the real mechanism `FR-067` found — so the composite
    push-off (`FR-013`/`FR-017`) is now what remains for a partial
    touch, and `FR-039`'s corner blend is the averaged normal. A car
    pressed sideways against a wall in mid-air, wheels in the air, gets
    a double jump, as RocketSim's would. (3) **Auto-roll.**
    `wheels::apply_auto_roll` (`Car::_UpdateAutoRoll`): with the raw
    throttle held and one to three wheels touching, a central force of
    `CAR_AUTOROLL_FORCE = 100 · mass` along the averaged contact normal
    into the surface and an angular acceleration of `CAR_AUTOROLL_TORQUE
    = 80` times the two misalignment factors (`1 - clamp(right ·
    (groundUp × forward))` about the forward axis, `1 - clamp(forward ·
    (groundDown × (groundUp × forward)))` about the right axis, each
    signed toward the surface), run after the driven forces and before
    the suspension impulses. The chassis's own world contact,
    RocketSim's fallback when no wheel touches, is not tracked (the
    Non-goals below stand); the auto-flip is not ported.
    - **Measured.** The isolated fixture never leaves the flat floor:
      `114.38 uu / 0.51 rad / 238.41 uu/s → 114.17 / 0.51 / 238.02`,
      ball `42.19` unchanged. The auto-roll acts on its two three-wheel
      landing ticks (`5.642`, `5.650`, throttle held) with the small
      factors RocketSim's formula gives at a `6°` roll — worth `0.2`
      uu. The step is measurable on wall-driving and curve-landing
      scenarios, which no capture yet holds.
    - **Tests.** `collision.rs`: `a_ray_from_inside_a_fillet_hits_its_
      concave_face_with_the_normal_toward_the_axis`, `a_ray_leaving_the_
      fillets_sector_or_starting_outside_its_cylinder_misses`, `a_ray_
      from_inside_a_corner_fillet_hits_its_face_within_its_triangle_
      only`, `a_ray_through_the_goal_window_misses_the_goal_wall_and_a_
      ray_beside_it_hits`, `a_ray_hits_a_bounded_wall_inside_its_bound_
      only`, `raycast_static_returns_the_nearest_hit_across_shapes`;
      `wheels.rs`: `the_wall_contact_normal_is_the_wheels_average_for_a_
      partial_wall_touch_only`, `auto_roll_presses_a_partially_landed_
      throttling_car_into_the_surface_and_levels_it`; `world.rs`: `the_
      wheels_see_the_standard_arenas_floor_fillet_with_its_tilted_
      normal`, and the wall-jump tests rewritten onto the wheels — `a_
      car_with_all_four_wheels_on_a_wall_jumps_along_its_own_up_which_
      is_the_walls_normal`, `a_car_with_two_wheels_on_a_wall_pushes_off_
      along_the_wheels_averaged_normal`, the wall-jump dodge and its
      flip-cancel re-posed on two wheels (the corner test's blend now
      the `wheels.rs` unit test). `rb_physics_bullet` `398 → 407`, the
      workspace `459 → 468`. Full workspace `fmt`/`clippy`/`test` green.
      `THIRD_PARTY_NOTICES.md`'s RocketSim table gains the auto-roll and
      the wall threshold; the ray tests are this port's own geometry.
  - **Non-goals (this requirement).** The
    implementation steps do not model three-wheel or non-Octane presets
    (descriptors are Octane-only; other presets are a data change later),
    do not raycast against other bodies (RocketSim allows a wheel to rest
    on the ball or a car; this port's wheels see only the static scene),
    do not track the chassis's own world-contact normal for auto-roll's
    fallback, and do not calibrate any of the replaced constants — they
    are deleted, not tuned. Does not touch
    `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on
    `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** This entry records the complete real
    mechanism with its symbols and constants, the rest-height and
    post-jump-contact derivations and their match to the capture, the
    proposed design, the blast radius, and the three-step sequencing;
    `FR-081` finding 1's wording is corrected here; `PROJECT-STATUS.md`'s
    Next item points at step (a).
  - **Verification plan.** No new tests (documentation-only, matching
    `FR-080`'s scoping precedent); the full workspace stays green (420
    tests). For the steps: (a) a `standard_car` seeded at `z = 17.0`
    stays within `0.1` uu of it, a car dropped from `z = 41` settles with
    no upward `vz`, four wheels report contact for `≥ 4` ticks after a
    ground jump; (b) `1600`/`3500` uu/s² throttle/brake totals from the
    per-wheel model, a full-lock steady-state turn radius at `500` uu/s
    from the curve, the handbrake's lateral/longitudinal ratio; (c) a car
    driving up a wall holds it with the sticky force; plus the
    re-measured `--self` / `--self-growth 0.05` numbers and the ratchet
    at every step.
- `RB-PHYSICS-001-FR-083` (post-hit divergence — diagnosis; findings 1–5
  implemented, closing `FR-063`): `FR-082` step (a) left the isolated `dodge-derailment` fixture
  matching the recording through the grounded ticks, the whole flight
  (`0.04` rad), and the landing, with the port's car hitting the ball at
  `t = 5.758` for the first time — and a car velocity error that steps to
  `220` uu/s at the hit and to `660` uu/s at `t ≈ 6.05` s. This pass
  traced that segment tick by tick against the recording (car position,
  velocity, angular velocity, forward and up axes, inputs, wheels in
  contact, and the ball) from the seed to `t = 6.2` s. The post-hit
  divergence is not a post-hit mechanism: it is the consequence of a
  `45` uu lag the port's car carries into the hit, plus the hit itself.
  Seven findings, ranked by cost. No physics changed; this entry is the
  plan.
  1. **Throttle accelerates the car in the air, and this port has no such
     force.** RocketSim's `_UpdateAirTorque` (which runs whenever fewer
     than three wheels touch) ends with `applyCentralForce(forward ·
     throttle · THROTTLE_AIR_ACCEL · mass)`, `THROTTLE_AIR_ACCEL = 200 /
     3 ≈ 66.7` uu/s². The recording shows it: with boost off and throttle
     held through the flight, the recorded horizontal velocity climbs
     `+0.36` uu/s per tick along the car's forward (`vx` `1008.6 →
     1013.1`, `vy` `1111.5 → 1117.8` over `t = 4.325`–`4.45`), `≈45`–`55`
     uu/s² horizontal — `66.7` times the pitched forward's horizontal
     component — while the port's stays at exactly `(988.1, 1100.6)`.
     Over the `1.4` s flight that is the `≈25` uu/s the port is slower at
     the landing (`(1657, 1460)` vs `(1664, 1486)`) and the `45` uu it is
     behind at `t = 5.75`. **That lag is the whole post-hit story**: the
     recorded car meets the ball at `t = 5.758` on the ground (`z =
     15.4`) with its jump press on the same tick, so the hit lands first
     and the jump fires the next tick (`vz` `+14 → -66 → +229`); the
     port's car, `45` uu (`2.4` ticks at `2240` uu/s) behind, fires its
     jump at `5.767` and meets the ball three ticks late at `5.783`, `z =
     22`, rising at `300` uu/s. Hitting from below and mid-jump, it gives
     the ball more vertical and less horizontal velocity (`(1521, 1969,
     901)` vs the recorded `(1602, 2148, 790)`) and loses `(-271, -313,
     -152)` where the recording loses `(-183, -158, -80)`; the `+300`
     uu/s of jump is spent into the ball. Everything after — the `+150`
     vs `+230` uu/s climb, the lower and shorter arc, the `660` uu/s step
     at `6.05` (finding 6) — follows. One line in the air branch.
  2. **The jump hold applies `JUMP_ACCEL` in full from its first tick;
     `JUMP_PRE_MIN_ACCEL_SCALE = 0.62` is wrong.** Recorded `vz` after the
     jump: `295.9, 299.9, 304.0, 308.0, 312.0, 316.1, 320.1` — `+4.0` per
     tick for seven ticks, which is exactly `JUMP_ACCEL / 120 (12.15) -
     gravity (5.42) - the sticky half-g (2.71) = 4.02`, then `+6.7` per
     tick (`326.8, 333.6, 340.3, …`) once the sticky force stops — the
     full `1458.3` uu/s² hold from the very first tick. The port's `vz`
     over the same ticks: `291.8, 291.3, 290.7, 290.1, 289.5` — `-0.6`
     per tick, exactly `0.62 · 12.15 - 5.42 - 2.71`, then `+6.7`. RocketSim
     carries the `0.62` under a "TODO: preferably don't use this system
     at all"; RLUtilities' `Jump` has no such scale; the capture has
     none. `FR-064`'s adoption is reversed on capture evidence: delete the
     constant. Worth `≈14` uu/s of `vz` — the port flies `≈4` uu low
     through the flight (`z` `96.2` vs `100.6` at `t = 4.40`).
  3. **The flip torque acts on the press tick.** Recorded `ω_y` on the
     dodge tick: `2.57 → 4.75` (`+2.18` = the `1.87` rad/s flip step plus
     that tick's air-control pitch), then `5.26, 5.38, 5.44, 5.46, 5.47,
     5.48` up against the `5.5` cap. The port's: `2.57 → 2.80` on the
     press tick, `4.97` a tick later — because `apply_driven_forces`
     applies the flip's torque from the tick *after* the dodge sets it,
     which is also RocketSim's order (`_UpdateAirTorque` runs before
     `_UpdateDoubleJumpOrFlip`). The recording says RL applies it on the
     press tick. One tick of phase at `5.5` rad/s is `0.046` rad — the
     residual `0.04` rad flight rotation error, and the reason the port's
     `ω_z` decays a tick late under the clamp (`-2.12` vs `-1.54` at `t =
     4.333`). Apply the first torque step inside the dodge block; a
     capture-over-reference correction with `FR-080` step (c)'s two as
     precedent.
  4. **A seeded car should start with its drive fields primed.** The port's
     wheels carry RocketSim's one-tick lag on engine force and steer
     angle, which is right mid-run but wrong at the seed: the recorded car
     is mid-maneuver (throttle `1`, steer `-1`) with those fields already
     set. First tick after the seed: recorded `vx` `+6.4`, port `+0.0`;
     recorded `ω_z` `-1.35 → -1.49`, port `-1.35 → -1.35`. `from_frame`
     should run `wheels::update_wheels`'s field half once from the seed
     frame's own input (no sticky force). `≈6` uu/s and `0.15` rad/s at
     the seed.
  5. **The car-ball hit itself: per-pair material and the extra impulse.**
     RocketSim's `Ball::_OnHit` overrides the contact's friction and
     restitution to `CARBALL_COLLISION_FRICTION = 2.0` /
     `CARBALL_COLLISION_RESTITUTION = 0.0` (`FR-063`'s finding) and, at
     most once per two ticks, adds a velocity to the ball: `hitDir =
     normalize((ball - car) ⊙ (1, 1, BALL_CAR_EXTRA_IMPULSE_Z_SCALE =
     0.35))`, then `hitDir = normalize(hitDir - forward · (hitDir ·
     forward) · (1 - BALL_CAR_EXTRA_IMPULSE_FORWARD_SCALE = 0.65))`,
     `addedVel = hitDir · relSpeed · BALL_CAR_EXTRA_IMPULSE_FACTOR_CURVE
     (relSpeed)` with `relSpeed = min(|v_ball - v_car|, 4600)` and the
     curve `{0: 0.65, 500: 0.65, 2300: 0.55, 4600: 0.30}` — a flattened,
     forward-biased kick worth up to `0.65` of the relative speed. The
     recorded ball leaves at `2795` uu/s with `vz = 790`; the port's, with
     the default materials and no extra impulse, at `2645` with `vz =
     901` — less along the ground and more up, the shape the `0.35`
     z-scale predicts. The angular kick to the car matches already
     (`(-0.47, -3.21, 4.44)` vs `(-0.36, -3.41, 4.30)`). But the port's
     hit is three ticks late and mid-jump (finding 1), so the linear
     comparison is not clean until findings 1–4 land. Medium cost: a
     per-pair material hook in the ball-car manifold and a post-solve
     added velocity on the ball; closes `FR-063`.
  6. **A fixture caveat, not a physics finding: the capture's pitch input
     is missing at the second dodge.** At `t = 6.05` the recording jumps
     with `pitch = 0, yaw = +1` recorded — a pure right dodge — but the
     recorded `Δv` `(-518, +466, 0)` decomposes on the car's flattened
     axes to `352` forward and `601` right, which is exactly `500 · 0.707`
     and `500 · 0.707 · 1.70` (the side scale at `1793` uu/s): a
     forward-right diagonal, `pitch = -1, yaw = +1`. The first dodge's
     inputs were exact (`FR-081` finding 2); this one's pitch is not in
     the capture. The port dodges pure right at `809` uu/s, so the `660`
     uu/s velocity step at `6.05`–`6.07` and the rotation that follows
     cannot be matched by any physics change on this fixture. That is the
     ratchet's floor here, and a note for `RB-VERIFY-001`/`ADR-0005` on
     capture input fidelity.
  7. **RocketSim vs. RL: the recording's wheels keep acting one to two
     ticks longer after the jump than RocketSim's ray allows.** The
     sticky-force deficit of finding 2 persists through the tick that
     starts with the origin at `z = 32.3` (mounts `53` uu up), and the
     recorded `vx` gains through six post-jump ticks where the port's,
     whose `48.755` uu ray lets go at `z = 28.0`, gains through five. The
     per-tick gains themselves match (`+9.6, +9.8, +10.4, +10.9, +11.2,
     +11.5` recorded vs `+10.3, +10.4, +10.9, +11.4, +11.6` in the port).
     Worth `≈11` uu/s of `vx`; no reference value to adopt, so it is
     recorded and left.
  - **What each finding is worth.** Findings 1–4 are a few lines each,
    each measurable on its own tick: 1 on the flight's horizontal
    velocity and the hit tick (`5.783 → 5.758`), 2 on `vz` over the seven
    post-jump ticks and the apex, 3 on the dodge tick's `ω_y`, 4 on the
    first tick after the seed. Together they should move the hit onto the
    recorded tick and geometry, after which finding 5 can be compared
    cleanly and implemented as its own step. Finding 6 bounds what this
    fixture can show after `6.05`; finding 7 is a known residual.
  - **Suggested sequencing.** Findings 1, 2, 3, 4 in one implementation
    pass with a re-measure after each; then finding 5 (closing `FR-063`)
    as its own pass; then `FR-082` steps (b) and (c). Do not tune
    anything against the segment after `6.05` s (finding 6).
  - **Findings 1–4, implemented.** (1) `drive::THROTTLE_AIR_ACCEL = 200 /
    3`, applied in the air branch as `forward · throttle · 66.7 · mass`
    from the raw throttle input (boost does not force it, as in
    `_UpdateAirTorque`). (2) `JUMP_PRE_MIN_ACCEL_SCALE` deleted; the hold
    is the full `JUMP_HOLD_ACCELERATION` inside the mandatory window —
    and, since RocketSim's `_UpdateJump` sets `isJumping` on the press
    and applies `jumpAccel` in the same call, the press tick now gets its
    first hold tick too (the armed window is one tick shorter, keeping
    `24` ticks of force in total). (3) `apply_flip_torque`, called on the
    press tick right after the dodge's impulse in both dodge blocks as
    well as on every flipping tick after; the flip-cancel argument cannot
    fire on the press tick because a dodge's own pitch is the opposite
    sign of its torque. (4) `PhysicsWorld::prime_car_wheels`, run by
    `from_frame` for every car whose seed frame carries an input: casts
    the rays, runs `wheels::update_wheels` for the drive fields, and
    discards the sticky force.
    - **Measured, each on its own tick** (the same tick-by-tick trace as
      the diagnosis). Finding 4: the first tick after the seed reads
      `vx` `314.2` vs the recorded `314.0` (was `307.7`) and `ω_z` `-1.50`
      vs `-1.49` (was `-1.35`). Finding 2: `vz` climbs `+4.0` per tick
      through the post-jump ticks exactly as recorded (`304.0, 308.0,
      312.0, 316.1, 320.1` vs `295.9, 299.9, 304.0, 308.0, 312.0`) — the
      same shape, `8.1` uu/s high throughout: the port's springs, still
      compressed at the start of the press tick, push for one more tick
      (`+8.1`, RocketSim's order too), where the recorded press tick reads
      the impulse plus one hold tick less gravity and the sticky half-g
      and no spring push at all. That is a second RocketSim-vs-RL
      residual around the jump, alongside finding 7 (the port also lets
      go a tick earlier, `+6.7` where the recording still reads `+4.0`),
      and is left as read. Finding 3: the dodge tick's `ω_y` reads `4.75`
      vs the recorded `4.75`, `5.26` vs `5.26` the tick after; the flight's
      rotation error fell `0.04 → 0.02` rad. Finding 1: the flight's
      horizontal velocity now climbs (`(376.5, 1131.6) → (377.8,
      1134.7)`, recorded `(385.1, 1130.6) → (386.1, 1133.1)`); the car is
      `16` uu behind at `t = 5.758` instead of `45`, and hits the ball
      at `5.775` instead of `5.783` — one tick late, not three. The last
      tick is the `≈11` uu/s of finding 7 over the flight (`≈15` uu);
      the trace shows the port's box `4.6` uu short of the ball's radius
      at the recorded detection tick (closest-point distance `93.0` vs
      `91.25`, the recording's own `88.4`). The hit is still after the
      port's jump. Isolated fixture: `160.19 uu / 0.44 rad / 264.09 uu/s
      → 139.52 uu / 0.47 rad / 253.03 uu/s` (max `741 / 1.78 / 691 →
      714 / 1.95 / 693`); `mean_ball_distance` `79.55 → 91.16` uu (max
      `320.66 → 405.66`) — worse, and expected: the earlier, higher-
      relative-speed hit under the default car-ball material gives the
      ball `vz = 1057` (recorded `790`) where before it gave `901`;
      finding 5's `0.35` z-scale and `CARBALL` material own that. The
      growth diagnostic: the whole flight `0.5`–`3.4` uu / `0.02` rad
      (from `1`–`45` uu / `0.04` rad), the landing `24` uu / `0.02` rad
      (from `53`), the hit `24` uu / `0.05` rad. After `6.05` s finding
      6's missing pitch still drives the `690` uu/s velocity step.
    - **Tests.** `throttle_accelerates_an_airborne_car_along_its_forward_
      at_the_real_air_rate` (replacing the no-effect test; `+66.7` uu/s
      per second, reverse pulls back, a grounded step adds none); the
      two mandatory-window tests re-pinned to the full hold; the four
      press-tick jump tests re-pinned to `JUMP_SPEED` plus one hold tick;
      seven flip tests re-pinned to the press-tick torque (the cap is
      reached on the third tick counting the press, as the capture
      shows); `from_frame_primes_a_seeded_cars_drive_fields_from_its_
      recorded_input` and the recorded-input test re-pinned to the
      primed first tick; the live-world double-jump tolerance widened
      `0.01 → 0.02` rad/s for the flip's earlier cap. `rb_physics_bullet`
      `382 → 383`, the workspace `443 → 444`; the ratchet tightens
      `< 165 → < 145` uu on the car (the ball stays `< 100`). Full
      workspace `fmt`/`clippy`/`test` green.
  - **Finding 5, implemented.** Three pieces, each where RocketSim has
    it. (i) `solver::PairMaterial { restitution, friction }` and
    `resolve_manifolds`'s dynamic manifolds now carry an
    `Option<PairMaterial>`: `Some` takes the pair's own values, `None`
    keeps `combine_restitution`/`combine_friction`'s per-body average
    (still Bullet's own path for every pair that has no override).
    `PhysicsWorld::step` hands the ball-car manifold
    `CARBALL_COLLISION_FRICTION = 2.0` / `CARBALL_COLLISION_RESTITUTION
    = 0.0` and the car-car manifold `CARCAR_COLLISION_FRICTION = 0.09` /
    `CARCAR_COLLISION_RESTITUTION = 0.1` (both from `RLConst.h`, both
    already recorded by `FR-063`; RocketSim sets them in
    `_BulletContactAddedCallback` and `Ball::_OnHit` before the solve
    runs, which is the same point). (ii) `hit::ball_car_extra_impulse
    (car, ball)`: `hitDir = normalize((ball.pos - car.pos) ⊙ (1, 1,
    BALL_CAR_EXTRA_IMPULSE_Z_SCALE = 0.35))`, then `hitDir = normalize
    (hitDir - forward · (hitDir · forward) · (1 -
    BALL_CAR_EXTRA_IMPULSE_FORWARD_SCALE = 0.65))`, times `min(|v_ball -
    v_car|, BALL_CAR_EXTRA_IMPULSE_MAXDELTAVEL = 4600)` times
    `BALL_CAR_EXTRA_IMPULSE_FACTOR_CURVE {0: 0.65, 500: 0.65, 2300:
    0.55, 4600: 0.30}` — computed from the pre-solve state on the tick
    the manifold appears (RocketSim computes it inside the contact
    callback, before the solver moves either body) and added to the
    ball's linear velocity after the solve and the nets, immediately
    before `clamp_ball_velocity` (RocketSim's `_velocityImpulseCache`,
    applied in `_FinishPhysicsTick` and clamped there). (iii) The
    cooldown: `PhysicsWorld` counts ticks (`tick_count`) and keeps, per
    car, the tick the impulse last fired; a second contact fires only
    when `tick_count > applied + 1`, RocketSim's `tickCount >
    lastHitTick + 1` — at most once per two ticks per car, so a ball
    resting on a roof gets one kick, not one every tick.
    - **Measured.** The port's ball now leaves the hit at `(1566, 2407,
      957)` uu/s, `3027` in magnitude, against the recorded `(1602,
      2148, 790)`, `2794` — flatter than the `(1548, 1983, 1057)` the
      default material gave (the `0.35` z-scale's shape), `8%` fast in
      magnitude and still `y`-heavy. The hit is still one tick late
      (`t = 5.775`, finding 7's residual): the port's car is one tick
      further into its jump and one tick faster when it meets the ball,
      and both `hitDir` and the relative speed read from that later
      state, which is where the overshoot lives. The car's post-hit path
      moves with the ball's: isolated fixture `139.52 uu / 0.47 rad /
      253.03 uu/s → 117.41 uu / 0.46 rad / 228.81 uu/s` (max `714.05 /
      1.95 / 692.89 → 615.60 / 1.91 / 636.01`); `mean_ball_distance`
      `91.16 → 75.22` uu (max `405.66 → 361.25`) — the ball figure is
      back below the wheels' `79.55` for the first time since findings
      1–4 moved the hit earlier. The growth diagnostic: the flight and
      landing unchanged (`0.5`–`3.4` uu and `24` uu), the hit window at
      `t = 5.77` reads `31` uu ball / `20` uu, `0.06` rad car (from `24`
      uu / `0.05` rad), and the post-`6.05` velocity step is still
      finding 6's `636` uu/s (`t = 6.07`).
    - **Tests.** Five in `hit.rs`: `a_ball_straight_above_a_still_car_
      is_popped_straight_up_at_the_curves_fraction` (`0.65` of the
      relative speed, straight up, no forward bias when `hitDir ⊥
      forward`); `the_kick_is_flattened_and_biased_away_from_the_cars_
      forward` (a ball ahead and above at `45°` leaves flatter than
      `45°`, its forward component scaled `0.65` before renormalizing);
      `the_relative_speed_is_capped_and_the_curve_falls_to_a_third_at_
      the_cap` (`9000` uu/s reads as `4600 · 0.30`); `no_relative_
      motion_means_no_kick`; `the_factor_curve_matches_rocketsims_
      breakpoints`. One in `world.rs`: `a_ball_dropped_on_a_still_car_
      pops_back_up_at_the_extra_impulses_fraction` — with zero
      restitution the pop is the extra impulse alone, `0.65` less the
      plastic contact's `30 / (30 + 180)` share, so the ratio sits at
      `≈0.45`–`0.5` and is bounded `0.4`–`0.6`. The wall-wedge symmetry
      test rewritten to give the wall the same zero-restitution /
      `2.0`-friction material as the pair and to subtract the extra
      impulse before comparing the two sides. `rb_physics_bullet` `383 →
      389`, the workspace `444 → 450`; the ratchet tightens `< 145 →
      < 125` uu on the car and `< 100 → < 85` uu on the ball. Full
      workspace `fmt`/`clippy`/`test` green. `THIRD_PARTY_NOTICES.md`'s
      RocketSim table gains rows for `hit.rs` and `PairMaterial`.
    - **Closes `FR-063`.** The per-pair-type override that requirement
      found and could not represent is now represented: the solver takes
      the pair's material when the world names one. `combine_restitution`
      /`combine_friction` are unchanged and still own every pair without
      an override.
  - **Non-goals (this requirement).** Does not implement findings 6 and
    7 (nothing to implement). Does not port RocketSim's car-car
    `_OnHit` bumps/demos (the `CARCAR` material is applied; the bump
    impulse is a separate mechanic with no fixture to measure it
    against). Does not touch
    `RB-PHYSICS-001-FR-005`'s real-data calibration, no longer blocked on
    `PHASE-0-EXIT` (now closed), but not itself started.
  - **Acceptance criteria.** This entry records the seven findings with
    their tick-level evidence, what each is worth, and the sequencing;
    findings 1–5 are implemented, each measured on its own tick or on
    the hit, with `FR-063` closed; `PROJECT-STATUS.md`'s Next item
    points at `FR-082` step (b).
  - **Verification plan.** The diagnosis itself added no tests (443);
    findings 1–4 brought the workspace to 444 and finding 5 to 450, with
    the fixture ratchet at `< 125` uu car / `< 85` uu ball.
- `RB-PHYSICS-001-FR-084` (landing and jump-exit contact — diagnosis;
  findings 1–3 implemented): `FR-082` step (b) left the isolated
  `dodge-derailment` fixture with two residuals around wheel contact: the
  recording's wheels touch at the landing one tick before the port's
  rays and let go one tick after them at the jump exit (`FR-083` finding
  7), and through the landing the recording's yaw rate falls where the
  port's holds. This pass added a second instrument to the tick trace:
  a *one-tick-from-recorded-state* comparison, which seeds the port from
  each recorded frame in turn (`PhysicsWorld::from_frame`), steps it
  once on that frame's input, and compares the step's `Δv`/`Δω` with the
  recording's — so a mechanism difference shows on its own tick, free
  of the pose the full run has accumulated. (It is blind inside a held
  jump, where re-seeding re-fires the press; the full run covers those
  ticks.) Six findings.
  1. **The rays reach `2.5` uu further than RocketSim's.** From the
     recorded pose at `t = 5.567` (origin `z = 43.5`, nose `13°` down,
     rolled `16°`) the recorded car's front-left wheel touches during the
     next tick (`Δω_x = +0.22`); the port's `48.755` uu ray misses that
     pose by `0.68` uu and the pose one tick earlier by `3.60`. At the
     jump exit the recorded tires still drive the car through the tick
     that starts at `z = 29.7` (mounts `50.5` uu up) and not the one at
     `32.3`. Together: the real reach is in `50.5..52.4` uu. RocketSim's
     `rayCast` subtracts `SUSPENSION_SUBTRACTION` (`2.5` uu) from the ray
     as well as from the pushback threshold; `rest + travel + radius`
     without it — `51.255` front, `52.055` back — sits inside the bound
     and the subtracted length does not. Cheap: `wheels::ray_length`.
  2. **The stick needs no wheel touching, not fewer than three.**
     RocketSim's `_PreTickUpdate` calls `_UpdateAirTorque(...,
     updateAirControl = numWheelsInContact == 0)`: with one or two wheels
     down the flip torque and the air throttle still apply, but
     `doAirControl &= updateAirControl` kills the stick torque *and* its
     damping. The port gated both on `!on_ground` (fewer than three).
     Through the six one- and two-wheel landing ticks (`5.583`–`5.633`)
     the port kept adding the held yaw's `+0.078` rad/s per tick: its
     single-wheel yaw change read `-0.04` where the recording's read
     `-0.15`, and `-0.12` once the stick is removed. Cheap: a
     `stick_control` argument to `apply_driven_forces`.
  3. **The stick gate reads last tick's wheel count.** Two ticks the
     fresh count cannot explain: at `t = 4.192`, the first tick after the
     jump exit with no wheel in reach, the recording holds `ω_z = -2.02`
     and `vx = 385` exactly — no tire, and no stick either, though the
     yaw stick is held and the port applies `+0.08`; and at `5.575`, the
     first tick with a wheel down, the recording's `Δω_z = -0.074` is the
     wheel's `-0.15` *plus* the stick's `+0.078`, which finding 2 alone
     would remove. Both fit one rule: the gate reads the previous tick's
     count — the stick stops one tick after the last wheel leaves and
     starts one tick after the first wheel lands. The candidate mechanism
     is the game's car logic reading the last physics tick's contact
     flags; RocketSim reads the fresh raycast. Implemented for the stick
     gate only (`PhysicsWorld::car_prev_wheels_in_contact`), not for
     `isOnGround`, the engine's quarter under three wheels, or the sticky
     force, none of which this fixture measures.
  4. **Open — the post-hit suspension slam.** The recorded car jumps on
     the hit tick (`5.758`, `j` pressed as the ball pushes it to `z =
     15.4`, `vz = -66`, nose lifting at `ω_y = -3.2`). In the next tick
     the recording gains `+295` uu/s of `vz` (the jump's `292` plus a
     spring-sized `≈7`) and keeps `ω_y = -3.22`. The port's back wheels,
     still within reach and driven down at `≈170` uu/s by the lift, fire
     the compression damping and the pushback: `+362` and `ω_y -3.35 →
     -1.64` (`+334` / `-2.46` with the pushback's velocity term dropped).
     RocketSim would do what the port does (its `updateSuspension` and
     `resolveSingleCollision` are the port's). The recording then loses
     `≈1.1` uu/s of `vz` per tick through the held jump, which neither
     `hold - g` (`+6.7`) nor `hold - g - sticky` (`+4.0`) predicts — as
     if its wheels were off *and* the hold were not applying. A variant
     with the pushback's velocity term removed measured `97.50 uu / 0.42
     rad / 205 uu/s` on the fixture but sinks the landing to `15.2`
     against the recorded `15.54` (`15.46` with it); not adopted. This
     needs a second fixture with a hit-tick jump, or one without.
  5. **Open — the single-wheel tire.** From the identical recorded pose
     one front wheel pushes the recorded car `5.3`–`5.6` uu/s per tick
     sideways and `1.0` uu/s backward; the port's `3.5` sideways, `0.3`
     forward. Two wheels: `9`–`10` vs `7`, with `2.7` of forward loss the
     port lacks; three: `19` vs `17`; four: `25.7` vs `25.5` and the yaw
     `+0.24` vs `+0.22` — the model matches once the car is down. The
     recorded single-wheel push points `11°` clockwise of the car's right
     where the port's points `4°` counter-clockwise (the steer). No
     RocketSim mechanism produces it (the friction circle Bullet's own
     vehicle applies would *reduce* a lightly loaded wheel's grip). Left
     as read.
  6. **A residual, recorded.** The press tick's spring push (`FR-083`
     finding 2) still leaves the port `+8` uu/s high in `vz` through the
     flight (`+11` at `4.242`, with finding 1's extra contact tick).
  - **Findings 1–3, implemented.** (1) `wheels::ray_length` is `rest +
    travel + radius`; the subtraction stays in the pushback threshold.
    (2) `drive::apply_driven_forces` takes `stick_control`; the stick's
    torque and damping run only when it is true. The wheel-less test
    helpers pass `!on_ground`; `step_on_wheels` and the world pass the
    contact count's zero test. (3) `PhysicsWorld` keeps each car's
    previous tick's count, primed at the seed, and passes `prev == 0`.
    - **Measured.** After findings 1 and 2: `102.64 uu / 0.40 rad /
      200.35 uu/s → 100.11 / 0.45 / 200.12`, ball `79.28 → 91.98` uu —
      the landing's contacts now arrive tick for tick (first wheel
      `5.575`, second `5.600`, third `5.633`, fourth `5.650`), the jump
      exit gains through `29.7` and reads `385`, but the first-touch tick
      lost its stick and the `4.192` tick gained one. After finding 3:
      `114.38 uu / 0.51 rad / 238.41 uu/s` (max `627.60 / 2.04 /
      682.23`), ball `42.19` uu (max `184.46`). The whole approach now
      matches to `4.4`–`4.8` uu and `0.05` rad (from `11.5`–`12.6`), the
      `4.192` tick reads `-2.02` / `386` exactly, the first landing touch
      `0.66` vs `0.64` rad/s (was `0.77`), and the car meets the ball *on
      the recorded tick* with the recorded geometry: at `5.758` the car
      reads `(1471, 1411, -72)` uu/s and `ω_z 4.33` against `(1488,
      1403, -66)` and `4.44`, and the ball leaves at `(1628, 2287, 815)`
      against `(1602, 2148, 790)` — `4.5%` fast, `3°` off (from `(1788,
      2347, 954)`, `11%` and `7°`, a tick late). The car figure *rose*,
      the first rise in the ratchet's history: finding 4's slam now
      lands on a car placed where the recording's is, and throws its
      rotation (`0.07` rad at the hit window, `0.24` by `5.87`) further
      than it threw the late, misplaced car of step (b); `FR-083`
      finding 6's missing pitch input then carries the larger angle
      through the rest of the run. The ratchet is loosened `< 110 → <
      120` uu on the car, once and for this reason, and tightened `< 85
      → < 50` on the ball.
    - **Tests.** `the_stick_does_nothing_while_any_wheel_touches`
      (`drive.rs`: the same one-wheel spinning car with the yaw stick
      held and neutral ends at the same yaw rate; with no wheel the stick
      yaws it); `the_stick_gate_reads_last_ticks_wheel_count`
      (`world.rs`: a car falling rolled `20°` keeps one tick of stick
      yaw on its first touching tick and none on the next); the reach
      tests re-pinned (`51.255` / `52.055`; contact at `29.7`, the front
      gone by `31.0`, both by `32.3`). `rb_physics_bullet` `396 → 398`,
      the workspace `457 → 459`. Full workspace `fmt`/`clippy`/`test`
      green.
  - **Non-goals (this requirement).** Does not change the pushback or
    the suspension (finding 4 is open), the tire model (finding 5), or
    any other reader of the wheel count (finding 3 is the stick gate
    only). Does not touch `RB-PHYSICS-001-FR-005`'s real-data
    calibration, no longer blocked on `PHASE-0-EXIT` (now closed), but
    not itself started.
  - **Acceptance criteria.** This entry records the six findings with
    their tick-level evidence, the one-tick instrument, and what was
    adopted; findings 1–3 are implemented and measured; findings 4 and 5
    name what a second fixture must show; `PROJECT-STATUS.md`'s Next
    item points at `FR-082` step (c) with finding 4 as the open item.
  - **Verification plan.** The two new tests and the re-pinned reach
    tests (459 in the workspace); the fixture ratchet at `< 120` uu car
    / `< 50` uu ball.
- `RB-PHYSICS-001-FR-085` (second capture session — findings A–K; A, C,
  D and E implemented): the second recording session `FR-084`'s Next
  item asked for — six clips of the owner's own real Rocket League, on
  the same plugin (`RB-VERIFY-002-FR-001`), driven through the port with
  `rb_verify_cli`'s seed rule and a per-tick trace (temporary examples,
  since removed), each divergence run to a mechanism. Three excerpts are
  vendored as fixtures with ratchet tests (`throttle-jump`,
  `boost-wall-entry`, `airborne-hit`; see `rb_capture_ingest/fixtures/
  README.md`); the whole clips are not.
  - **A. The car's speed is capped at `2300` uu/s, whatever put it
    there (implemented).** `walldrive04`'s boost run holds exactly
    `|v| = 2300`, and the jump pressed at that speed reads `(−2281, 292)`
    the tick after — still `2300` long: the kick's `vz` *costs*
    horizontal speed. The port read `(−2305, 304)`: `MAX_CAR_SPEED` only
    gated new boost force. RocketSim's `Car::_FinishPhysicsTick`
    "Limit velocities" block rescales the whole vector
    (`vel.normalized() * CAR_MAX_SPEED`) next to the angular cap
    `clamp_angular_speed` already carries; `drive::clamp_linear_speed`
    is that, called from the same place. Two tests; the goal-mouth
    car tests re-pinned to launch at the cap.
  - **B. Straight driving, ground jumps and landings match (fixture).**
    `groundjumpthrottle03`: a throttle-only drive from rest, a `0.18` s
    tapped jump, its landing, a `0.76` s held jump — `3.3` uu mean over
    `4.6` s (`throttle-jump`, `558` frames). The two-second stop, the
    brake, and the handbrake turn in the same clip track to a few uu
    too. `walldrive04`'s straight at the cap: `< 1` uu over `1.8` s.
  - **C. The goal-side edge fillets are withdrawn (implemented).**
    The port's car, drifted `15` uu left of the recording, struck a
    surface `340` uu in front of the back wall at `(921, 4779, 126)`,
    airborne — `standard_arena`'s `curves[24]`, one of `FR-024`'s six
    goal-cutout edge fillets. Built from the back wall's field-facing
    plane and the post's inward-facing plane, each of those was a
    concave `292` uu gutter standing *in front of* the wall beside the
    goal, at `x ∈ [893, 1185]`, `y ∈ [4828, 5120]`; the recording's car
    drives through `x ∈ [894, 992]`, `y ∈ [4838, 5120]`, `z ∈ [30, 160]`
    onto a flat wall at `y = 5120` and up it to `z = 305`. The six edge
    fillets and `FR-026`'s four post-crossbar corner fillets (the same
    premise) are removed; the goal window stays a clean cut. The
    recording bounds any real rounding there to `≤ 10` uu or a gutter
    of `71–259` uu; the flat wall the car climbs says the former.
  - **D. The corner arch radius is `FILLET_RADIUS`, not `750`
    (implemented).** `curverun05` rides the `+X` wall's floor fillet at
    `y ≈ −3300` (`z = 22–25`, flat); the port at the same spot sat `26`
    uu up a ramp and, reseeded at `18.40`, lifted off it (`vz 0 → 603`
    in `0.2` s, wheel normals turning to `(−0.17, 0.17, 0.97)`): the
    `+X/−Y` corner arch, `FR-025`'s `750` uu guess, reaches `1060` uu
    into the field. The recording crosses the real arch at
    `(3827, −4075, z 69)` and `(3631, −4346, z 120)` — `115` / `62` uu
    in from the `|x| + |y| = 8064` corner line, `52` / `103` uu up —
    which fit a circle of `≈ 277` (`≈ 285` with `5` uu of compression
    at `1900` uu/s), and sits flat at `(3886, −3677)` where `750` puts
    the floor `113` uu high. `CORNER_ARCH_RADIUS = FILLET_RADIUS`; the
    16 compound-corner fillets follow it. The reseeded ride now holds
    to `6` uu through the corner approach.
  - **E. The jump press tick has no suspension push (implemented).**
    Every jump in every clip: the tick after the press reads `296` uu/s
    recorded, `304` in the port; then `+4.0` a tick (hold `12.15` −
    gravity `5.42` − sticky `2.71`) in both while the wheels still
    reach, `+6.7` once they don't — except the port's wheels let go a
    tick early, being `0.4` uu higher, so the `+8` becomes `+11` and
    stays (`16` uu of height by the landing). `296 = 291.7 + 12.15 −
    5.42 − 2.71`: the real press tick is the impulse, the hold, gravity
    and the sticky force and *not* the springs that were holding the
    car up (`+8.1` at rest, exactly gravity plus sticky). RocketSim
    applies its `updateSuspension` after `_UpdateJump` from the same
    tick's ray hits and would push; the recording says the real game
    does not. `drive_and_integrate_velocities` now reads the ground
    press before `apply_driven_forces` consumes it and skips
    `apply_suspension_impulses` on that tick. One test. This is
    `FR-083`'s finding 6/7 residual ("`+8` uu/s press-tick spring
    push"), and it moved the `dodge-derailment` fixture `114.17 →
    73.76` uu (ball `42.19 → 41.80`); the one-wheel-landing clip's
    isolated hops (`8.2`, `12.7`, `16.4` s) lose their `+11` — a `+2.7`
    residual remains, the port's wheels still letting go one tick
    before the recording's.
  - **F. The floor-to-wall curve sheds speed the port does not
    (open).** `walldrive04` at `2300` into the `+X` curve: recorded
    `2300 → 1839` by the top of the curve, gravity explains `→ 2227`,
    so `~380` uu/s is lost in the transition; the port loses `~100`.
    Descending the same wall backwards: recorded `1521 → 1411` through
    the curve, the port `1508 → 1626` (gravity's gain, nothing lost),
    and the recording then holds `1411.83` for `4.7` s of reverse
    throttle — the taper's zero, reached from above. The recording's
    origin path through the curve fits `R ≈ 270` with the suspension
    bottomed (`5.6` uu clearance) or `292` with the origin `3` uu
    *inside* the surface; the port's hitbox never touches the curve
    (one `0.1` uu contact), its springs hold at `14.8` (the travel
    stop). The candidate: the real suspension bottoms harder and the
    chassis scrapes the mesh (`CARWORLD_COLLISION_FRICTION = 0.3` at
    `~30 g` of normal load is `~1000` uu/s of friction budget per
    `0.2` s), which the port's `resolveSingleCollision` pushback
    prevents. `boost-wall-entry` (`271` frames) pins the run at `3.9`
    uu mean, `64` uu max at the curve.
  - **G. The airborne hit matches (fixture).** `hittickjump01b`: boost
    from rest, jump at the cap, the ball met in the air `7` ticks after
    the press with the wheels already off — ball `4.8` uu mean, car
    `5.7` uu over `4.3` s (`airborne-hit`, `517` frames); the ball's
    exit `(−1, 3092, 408)` vs `(0, 3008, 372)`, the car's pitch kick
    within `7 %`. It is the jump-before-hit control `FR-084` finding 4
    wanted, not the wheels-down hit-tick jump it still needs.
  - **H. No drag above `1410` with the throttle held (confirmed).**
    Boost released at `2300` with throttle down: the recording holds
    `2297` for `1.8` s to the wall; so does the port. The `1626` vs
    `1411` reverse plateau above is finding F's, not a drag term.
  - **I. Capture defects (to the recorder).** `hittickjump01` and
    `01b` record `steer`/`pitch`/`yaw`/`roll` as `0` on every frame
    while the car turns through `194°` and flips forward at `20.283`
    with an all-zero stick; `walldrive04` records a dodge at `30.175`
    (`ω_y −0.87 → −5.22` in one tick, `1218 → 933` uu/s) with `jump =
    false` on every frame from `29.825` to `31.5`, its `pitch = 1`
    appearing one tick *after* the flip begins; `jumpbeforehit02` is a
    byte-identical upload of `hittickjump01`. The other three clips
    carry full analog data. The owner plays on keyboard and mouse, one
    device for all
    six (`jumpbeforehit02` was a rename of the first take), which
    points at the recorder: plugin 1.0 wrote each tick's line at the
    *first* `SetVehicleInput` firing and read every car's input back
    through `CarWrapper::GetInput()` then — fresh only if that car's
    own firing had already run. Plugin 1.1 (`RB-VERIFY-002` 0.5.0)
    records the hook's own `ControllerInput` per car and flushes the
    line when the next tick begins; it awaits its first real capture.
  - **J. The dodge residuals remain (`FR-083`).** `onewheellanding06`'s
    diagonal-dodge hops (`4.575`, `14.008`) diverge as the
    `dodge-derailment` fixture does (`ω_x 1.57` recorded vs `−0.47`,
    `vy −274` vs `−214` at `4.8`); after the first the port lands on
    its roof at `6.4` and stays (`z = 40.1`, no wheel down) while the
    recording drives off. The clip's plain hops and air rolls track to
    the second decimal in `ω`.
  - **K. The ball's goal entry diverges (open).** `hittickjump01b` at
    `10.0–10.25`: the recorded ball enters the `+Y` goal (`|y| > 5000`
    at `10.042`); the port's parts from it by `~990` uu there — the
    goal mouth / net interaction, outside `airborne-hit`'s cut.
  - **Non-goals (this requirement).** Finding F's mechanism, finding
    K, the dodge (`FR-083`), `FR-084` finding 4 (still without a
    wheels-down hit-tick jump) and finding 5 (the clip's one-wheel
    landings all follow dodges the port already misses).
  - **Acceptance criteria.** Findings A, C, D and E implemented with
    tests; the three fixtures vendored with ratchets; the
    `dodge-derailment` ratchet tightened `< 120 → < 85`; F, I, J and K
    recorded with their evidence; `PROJECT-STATUS.md`'s Next item
    points at F.
  - **Verification plan.** `468` tests in the workspace (`404` in
    `rb_physics_bullet`: `+2` speed cap, `+1` press tick, `−6` goal
    fillets; `13` in `rb_verify_cli`: `+3` fixtures); the four fixture
    ratchets.
- `RB-PHYSICS-001-NFR-001` (implemented): The physics core doesn't force
  Bullet-specific data modeling into `rb_domain` — `rb_domain::state`
  stays a plain state DTO plus general-purpose vector/quaternion algebra;
  `rb_physics_bullet` owns all rigid-body/solver-specific types.

## Architecture and interfaces

`rb_physics_bullet` (new crate, depends only on `rb_domain`):
- `mat3`: `Mat3`, a general 3x3 matrix — needed because a box's inertia
  tensor is anisotropic (unlike a sphere's scalar/isotropic inertia).
- `body`: `RigidBody` (dynamic; a `Shape` enum — `Sphere` or `Box` —
  picks the collision geometry and local inertia formula), `StaticPlane`
  (immovable). One `RigidBody` type serves both shapes, matching Bullet's
  own architecture (`btRigidBody` + a polymorphic `btCollisionShape`)
  rather than a separate rigid-body type per shape. `StaticQuarterPipe`
  (also immovable, since `RB-PHYSICS-001-FR-020`) is a second static shape
  alongside `StaticPlane` — a partial-cylinder fillet, with its own
  `between_planes` constructor deriving its geometry from two flat planes.
  `StaticCornerFillet` (also immovable, since `RB-PHYSICS-001-FR-023`) is a
  third static shape — a sphere blending three flat planes at a single
  vertex, with its own `between_three_planes` constructor deriving its
  center and containment `bounds` from those three planes directly.
  `StaticGoalWall` (also immovable, since `RB-PHYSICS-001-FR-024`) is a
  fourth static shape — a `StaticPlane` plus a rectangular window in the
  plane's own local `u_axis`/`v_axis` frame, with `contains_in_window`
  testing a point's projection onto that frame directly.
  `StaticBoundedWall` (also immovable, since `RB-PHYSICS-001-FR-029`) is a
  fifth static shape — a `StaticPlane` plus a rectangular *bound* in the
  plane's own local `u_axis`/`v_axis` frame (`bound_center`/`half_u`/
  `half_v`), with `contains_in_bound` testing a point's projection onto
  that frame directly, the opposite gate convention from
  `StaticGoalWall`'s window (collides only *inside* the bound, not
  everywhere *except* inside a window).
- `integrate`: force accumulation, velocity integration, transform
  integration — pure functions over `RigidBody`, shape-agnostic.
- `collision`: `contacts_vs_plane` — analytic body-vs-static-plane contact
  generation (any plane, not just the ground — an arena wall is the exact
  same test with a different normal), dispatching to a sphere- or
  box-specific test and returning a manifold (`Vec<Contact>`, 0 to 4
  points); `contacts_vs_quarter_pipe` (since FR-020) — analytic
  sphere-vs-fillet contact generation for a sphere (always 0 or 1 points),
  dispatching to `box_vs_quarter_pipe`'s corner-testing (exact, not an
  approximation, per `RB-PHYSICS-001-FR-032`; 0-8 points) for a box since
  `RB-PHYSICS-001-FR-027` (a box always
  returned no contact through FR-026, see FR-020's original Non-goals);
  `contacts_vs_corner_fillet` (since
  FR-023) — the same analytic sphere-only contact generation against a
  `StaticCornerFillet` instead, using its spherical-triangle containment
  test in place of a `StaticQuarterPipe`'s 2-sided sector one, and the same
  `box_vs_corner_fillet` corner-testing dispatch for a box since FR-027;
  `contacts_vs_goal_wall` (since FR-024) — dispatches by shape instead of
  always analytic sphere-only: a sphere gets `sphere_vs_goal_wall`'s
  windowed treatment (no contact for a center inside the window), and,
  since `RB-PHYSICS-001-FR-028`, a box gets the equivalent per-corner
  windowed treatment via `box_vs_goal_wall` instead of falling straight
  through to an unwindowed `contacts_vs_plane` the way it did through
  FR-027 — a car can now actually drive into a goal through the same
  window the ball already could pass through;
  `contacts_vs_bounded_wall` (since `RB-PHYSICS-001-FR-029`) — dispatches by
  shape against a `StaticBoundedWall` instead: a sphere gets
  `sphere_vs_bounded_wall`'s bounded treatment (no contact for a center
  outside the bound), and a box gets `box_vs_bounded_wall`'s per-corner
  bounded treatment, the same "test every corner" technique
  FR-027/FR-028 established, but skipping a corner *outside* the bound
  instead of *inside* a window;
  `contacts_between` — dispatches to `sphere_vs_box` (0 or 1 points) or the
  separating-axis `box_vs_box` (0 to 4 points), covering every
  two-dynamic-body shape pairing this crate has.
- `solver`: `resolve_contacts` — sequential-impulse contact + friction
  resolution over an entire manifold against one static body, identified
  only by its `restitution`/`friction` (since FR-020, this serves a
  `StaticQuarterPipe` fillet exactly as it already served a `StaticPlane` —
  the static shape's actual geometry is irrelevant here, already baked into
  the caller's own `Contact` list); `resolve_contacts_between` — the same
  sequential-impulse math generalized to two dynamic bodies' shared contact
  manifold; `resolve_dynamic_manifolds` (since `RB-PHYSICS-001-FR-030`) —
  the scene-wide entry point `PhysicsWorld::step` actually calls: every
  ball-vs-car and car-vs-car manifold touching in a step shares one
  interleaved `SOLVER_ITERATIONS`-iteration budget instead of each pair
  getting its own fully independent `resolve_contacts_between`-style
  solve, via a new `delta_pair_mut` disjoint-borrow helper letting
  multiple manifolds that share a body index share that body's single
  `DeltaVelocity` accumulator too.
- `drive`: `apply_driven_forces` — couples a car's `ControllerInput` into
  ground throttle/steering forces and torques, a boost force/resource
  drain, a handbrake-driven temporary friction adjustment, a
  rising-edge-triggered ground jump impulse (with a continuous
  `JUMP_HOLD_ACCELERATION` hold-window bonus for variable height, driven by
  `jump_hold_time_remaining`), airborne pitch/yaw/roll torque, a second
  rising-edge-triggered airborne jump impulse (double
  jump, gated on and consuming a `double_jump_available` flag rather than
  ground contact — either a plain vertical `JUMP_SPEED` kick or, when
  `pitch`/`roll` exceed `DODGE_DEADZONE` at the moment of the press, a
  directional dodge: a horizontal `DODGE_SPEED` impulse plus an
  instantaneous `DODGE_ANGULAR_SPEED` spin written directly to
  `RigidBody.angular_velocity`, also arming a `dodge_flip_active` flag a
  further fresh press can spend to flip-cancel — zero the spin outright —
  before landing or a wall touch re-arms the double jump), and a third jump
  variant fired instead of the double-jump-or-dodge branch when a
  `wall_normal` (the outward normal of a touched wall, if any) is present —
  a plain outward-plus-upward impulse that restores rather than consumes
  `double_jump_available` below `DODGE_DEADZONE`, or — at or above it — a
  wall-jump dodge combining that same push-off with a `DODGE_SPEED`
  horizontal component and `DODGE_ANGULAR_SPEED` spin, which *does* consume
  `double_jump_available` and arms `dodge_flip_active` exactly like a
  ground dodge, and — whenever airborne with no active `pitch`/`roll` and
  no fresh jump press this step — a gentle continuous `LANDING_AUTO_UPRIGHT_TORQUE`-scaled restoring torque nudging the car's local up axis
  back toward world up (not a Bullet3 port — this project's own model of
  Rocket League's driving mechanics, since the real numbers aren't public;
  see the module's own doc comment for which constants are commonly-cited
  community estimates vs. uncalibrated placeholders).
- `world`: `PhysicsWorld::step`/`frame`, and `simulate()` — the
  composition root Bullet's `btDiscreteDynamicsWorld::stepSimulation`
  corresponds to, run in the same staged order (integrate every body's
  velocity — for cars, including `drive::apply_driven_forces` — then
  resolve every contact — ground, every wall, every curve, every
  corner fillet, every goal wall, and every bounded wall for every body,
  every ball-vs-car
  pair, then every
  car-vs-car pair — then integrate
  every body's transform). `PhysicsWorld` carries one ball (`RigidBody`,
  always present), `walls: Vec<StaticPlane>` (any number, via repeated
  `with_wall` calls, empty by default), `curves: Vec<StaticQuarterPipe>`
  (since FR-020; any number, via repeated `with_curve` calls, empty by
  default — deflected the ball only, a no-op for every car, through
  FR-026; since `RB-PHYSICS-001-FR-027`, a car is deflected too, via the
  8-corner testing approximation `collision::box_vs_quarter_pipe`
  provides), `corner_fillets: Vec<StaticCornerFillet>` (since FR-023; any
  number, via
  repeated `with_corner_fillet` calls, empty by default — same
  ball-and-since-FR-027-car deflection convention as `curves`, via
  `collision::box_vs_corner_fillet`), `goal_walls: Vec<StaticGoalWall>`
  (since FR-024; any number, via repeated `with_goal_wall` calls, empty by
  default — unlike `curves`/`corner_fillets`, resolved for *every* body
  from the start, and, since `RB-PHYSICS-001-FR-028`, a car passes through
  the window too, via `collision::box_vs_goal_wall`'s own per-corner
  window test), `bounded_walls: Vec<StaticBoundedWall>` (since
  `RB-PHYSICS-001-FR-029`; any number, via repeated `with_bounded_wall`
  calls, empty by default — like `goal_walls`, resolved for every body
  from the start, via `collision::contacts_vs_bounded_wall`), and
  `cars: Vec<RigidBody>` (any number, via repeated `with_car` calls) with a
  parallel
  `car_inputs: Vec<ControllerInput>` set via `set_car_input`, a parallel
  `car_boost: Vec<f32>` set via `set_car_boost`, a parallel
  `car_base_friction: Vec<f32>` snapshotted from each car's own friction by
  `with_car` (handbrake's restore target), a parallel
  `car_jump_held: Vec<bool>` (jump's rising-edge state, starting `false`),
  a parallel `car_double_jump_available: Vec<bool>` (starting `true`,
  restored on landing or wall contact, consumed by an airborne double
  jump), a parallel `car_jump_hold_time_remaining: Vec<f32>` (starting
  `0.0`, the ground jump's variable-height hold window), and a parallel
  `car_dodge_flip_active: Vec<bool>` (starting `false`, whether the most
  recent double-jump-or-dodge press left a cancelable flip active); each
  car's current wall contact (if any), like its ground contact, is computed
  fresh at the start of every `step` from its position at the time.
  `frame()` assigns each car's `player_id` as its index in `cars` and
  reports its current input and boost amount.
- `arena`: `standard_ground`/`standard_walls` — Rocket League's real
  standard-arena field dimensions and a 7-`StaticPlane` boundary (2 side
  walls, a ceiling, 4 diagonal corner walls — the back walls moved out as
  of FR-024, see below) built from `body::StaticPlane` alone (no new
  collision code); `standard_curves` (since FR-020, extended to all 9
  walls' floor/ceiling seams by FR-021, and to the 8 corner-wall vertical
  edges by FR-022) — 24 `StaticQuarterPipe` fillets total: 16 floor-side/
  ceiling-side fillets (one pair per wall, all 9 walls including the 4
  diagonal corner walls since FR-021, and still built from `back_wall_plane`
  directly even though the back walls themselves left `standard_walls`)
  plus 8 vertical-edge fillets (one per corner-wall endpoint, since
  FR-022), all built via `StaticQuarterPipe::between_planes` from those
  same flat planes — a corner wall's own floor/ceiling-seam
  `axis_direction` is computed via a cross product rather than
  hand-picked, since (unlike a cardinal wall's) it isn't a coordinate
  axis, while a vertical-edge fillet's own `axis_direction` is simply
  `(0, 0, 1)` (the edge itself is vertical); since FR-025, the 8 of those
  16 floor/ceiling-seam fillets that bridge a corner wall (rather than a
  cardinal one) to the floor or ceiling are built with the distinctly
  larger `CORNER_ARCH_RADIUS` in place of `FILLET_RADIUS`, while the other
  8 cardinal-wall seam fillets and all 8 vertical-edge fillets keep
  `FILLET_RADIUS` unchanged; `standard_corner_fillets`
  (since FR-023) — 16 `StaticCornerFillet`s, one per compound corner (4 per
  corner wall — floor+side, floor+back, ceiling+side, ceiling+back — times
  the 4 corner walls), all built via `StaticCornerFillet::between_three_planes`
  from those same three flat planes directly, not from the two
  `StaticQuarterPipe`s `standard_curves` builds at that vertex, and, since
  FR-025, all built with `CORNER_ARCH_RADIUS` rather than `FILLET_RADIUS`
  — every one of them touches one of the 8 now-larger arches, and
  `between_three_planes` needs one shared radius across the three planes it
  blends to still meet an arch exactly where their axes cross;
  `standard_goal_walls` (since FR-024) — 2 `StaticGoalWall`s, one per back
  wall, each wrapping the same `back_wall_plane` `standard_curves` already
  uses, windowed at `GOAL_HALF_WIDTH`/`GOAL_HEIGHT`;
  `standard_goal_cutout_fillets` (since FR-024) — 6 more
  `StaticQuarterPipe`s (two posts and a crossbar per goal), built via
  `StaticQuarterPipe::between_planes` from the real back-wall plane and a
  purely-geometric post/crossbar plane (`goal_post_plane`/
  `goal_crossbar_plane`, never themselves added as real collision walls);
  `standard_goal_corner_fillets` (since FR-026) — 4 more
  `StaticCornerFillet`s (one per goal post per goal), built via
  `StaticCornerFillet::between_three_planes` directly from those same
  back-wall/post/crossbar planes, rounding off the two compound corners per
  goal where a post's own edge fillet meets the crossbar's, added to the
  same `corner_fillets` list `standard_corner_fillets`'s 16 already
  populate (20 total) and reusing `FILLET_RADIUS` unchanged;
  `standard_goal_back_walls` (since `RB-PHYSICS-001-FR-029`) — 2 plain,
  unbounded `StaticPlane`s (one per goal), `GOAL_DEPTH` behind the real
  back wall, added to `PhysicsWorld.walls` (now 9 real entries, up from 7,
  once `standard_arena` is built);
  `standard_goal_side_walls` (since FR-029) — 4 `StaticBoundedWall`s total
  (2 per goal), each reusing `goal_post_plane` unchanged, bounded to the
  goal's own depth and height; `standard_goal_roofs` (since FR-029) — 2
  `StaticBoundedWall`s total (1 per goal), each reusing
  `goal_crossbar_plane` unchanged, bounded to the goal's own width and
  depth — all 6 of these `StaticBoundedWall`s go into a new
  `PhysicsWorld.bounded_walls` field via `with_bounded_wall`, not the
  `walls`/`curves`/`corner_fillets` lists any prior static shape used;
  `PhysicsWorld::standard_arena` (in `world`)
  wires all of these into a new `PhysicsWorld` in one call, an alternative to
  `PhysicsWorld::new` plus manual `with_wall`/`with_curve`/
  `with_corner_fillet`/`with_goal_wall`/`with_bounded_wall` calls for a
  caller that wants the real field rather than a custom test arena.

No `PhysicsStateSource`-style trait exists yet for "the physics engine"
specifically — `rb_verify_cli` calls `rb_physics_bullet::simulate`
directly. A trait is worth introducing once a second physics core
implementation actually exists to justify it (per the "no speculative
abstraction before two real call sites" convention this project follows
throughout) — not before.

## Data/state and invariants

World convention: +Z is up (matching Unreal Engine, which Rocket League
runs on). Sphere inertia is isotropic (`I = 2/5 m r^2`, same value on all
three axes); box inertia is anisotropic (`I = m/12 * (b^2 + c^2)` per
axis, from the box's full dimensions). Both are stored as
`RigidBody.inv_inertia_local` (a diagonal, in the body's own local frame)
and combined with the body's current orientation into
`inv_inertia_world` (a full `Mat3`) via `update_inertia_tensor` — called
once per step, after the transform integrates. A sphere's `inv_inertia_world`
is mathematically orientation-independent (`R * kI * R^T == kI` for any
rotation `R`), so this generalization doesn't change sphere behavior from
the previous scalar-only representation (see
`body::tests::sphere_inertia_tensor_is_orientation_independent`).

## Errors, failure, recovery, and observability

No fallible operations — `RigidBody::new` panics on non-physical input
(zero/negative mass, or a zero/negative radius or half-extent), matching
"trust internal callers, validate at real boundaries" (a physics body's
own constructor is such a boundary; a malformed body is a programming
error, not a recoverable runtime condition).

## Security, privacy, and compatibility

None beyond `THIRD_PARTY_NOTICES.md`'s zlib attribution obligations.

## Acceptance criteria

- Sphere (met): free-fall matches semi-implicit Euler kinematics before
  impact; an inelastic resting contact stays at rest; a dropped ball
  settles near the ground; restitution produces a bounce proportional to
  the combined coefficient; friction decelerates a sliding sphere and
  couples into spin.
- Box/FR-004 (met): free-fall matches the same kinematics as a sphere
  (shape-independent integration); box-vs-plane contact generation
  produces the correct point count for flat (4), edge-tilted (2), and
  embedded (4, positive penetration) cases; a box's inertia tensor changes
  with orientation while a sphere's doesn't; a box dropped flat settles on
  the ground without tipping onto an edge or corner (multi-contact
  resolution keeping symmetric contacts symmetric); a box resting flat
  with a small downward velocity settles to zero net rotation (no spurious
  torque from resolving 4 contacts one at a time). Sphere-vs-box contact
  generation is correct at the surface, under overlap, and for a sphere
  center embedded inside the box (pushed out via the nearest face); the
  two-body solver conserves linear momentum, produces no residual closing
  speed for an inelastic collision, and leaves a much heavier body
  (the car) barely moving from a much lighter body's (the ball's) impact;
  an end-to-end `PhysicsWorld::step` test confirms a ball shot at a
  stationary car actually bounces off it rather than tunnelling through.
- FR-006 (met): `box_vs_box` correctly reports no contact for far-apart
  boxes, a 4-point manifold with correct depth and normal for a symmetric
  flat overlap, a normal/depth pair antisymmetric in argument order
  (matching the sphere-vs-box case), and a partial (fewer-than-4-point)
  manifold for a non-flat rotated overlap; the generalized
  `resolve_contacts_between` settles two colliding boxes' face-to-face
  manifold without spurious net rotation, the same property already
  verified for the one-body ground-manifold case. `PhysicsWorld` builds a
  multi-car scene from repeated `with_car` calls, assigns each car a
  sequential `player_id`, and — the real end-to-end proof — two cars shot
  head-on at each other in a live `PhysicsWorld::step` loop actually
  bounce off each other instead of tunnelling through.
- FR-007 (met, ground throttle/steering): a neutral input applies no
  force or torque (so a car with no input set is unaffected); throttle
  accelerates a grounded car forward, has no effect while airborne, stops
  accelerating at `MAX_CAR_SPEED`, and reverse throttle accelerates
  backward; steering has no effect on a stationary car but yaws a moving
  one, in the opposite direction for opposite `steer` sign; an end-to-end
  `PhysicsWorld::step` loop with `set_car_input` set to full throttle
  drives a car forward across the ground, and `frame()` reports that same
  input back.
- FR-008 (met, boost): boost accelerates a car regardless of ground
  contact (an end-to-end `PhysicsWorld::step` loop with gravity zeroed and
  full boost input drives a car forward while airborne); the boost tank
  drains over time while held and clamps at zero; boost has no effect once
  the tank is empty; boost still drains the tank even once the car is at
  `MAX_CAR_SPEED` and the forward force stops applying; a new car starts
  with a full tank (`MAX_BOOST`), and `frame()` reports the live
  `boost_amount` instead of a hardcoded `0.0`.
- FR-009 (met, handbrake): handbrake reduces friction while grounded, has
  no effect on friction while airborne, and releasing it restores the
  car's own base friction (not a hardcoded default, verified with a
  car constructed with a non-default friction); an end-to-end
  `PhysicsWorld::step` loop confirms a car already sliding sideways
  retains more of that slide under handbrake's reduced friction than
  under normal grip.
- FR-010 (met, single ground jump): jump gives a grounded car upward
  velocity, has no effect while airborne, doesn't re-fire on a second call
  while still held, and fires again after a release-then-re-press; an
  end-to-end `PhysicsWorld::step` loop confirms a car with jump input
  actually leaves the ground, and — holding jump for the car's entire
  flight, never released — confirms it lands and settles instead of being
  relaunched every time it touches back down.
- FR-011 (met, air control): pitch/yaw/roll each produce angular velocity
  about the correct local axis for a stationary airborne car (no speed
  requirement, unlike ground steering); air control has no effect while
  grounded; a `None` analog value behaves like neutral input; opposite-sign
  yaw spins the opposite way. An end-to-end `PhysicsWorld::step` loop
  (gravity zeroed) confirms a car with yaw input actually reorients itself
  mid-air, and a regression test confirms a grounded car stays level
  despite stray pitch/yaw/roll input.
- FR-012 (met, double jump): a fresh airborne jump press gives upward
  velocity when the double jump is available, has no effect when it isn't,
  is consumed after firing once (a release-then-re-press while still
  airborne doesn't refire it), and touching the ground restores
  availability. An end-to-end `PhysicsWorld::step` loop (gravity zeroed)
  confirms a double jump fired after a ground jump adds a second
  `JUMP_SPEED` kick on top of the first, and a regression test confirms a
  spent double jump doesn't fire again mid-air no matter how many more
  times jump is released and re-pressed before landing.
- FR-013 (met, arena walls and wall jump): a fresh jump press while
  airborne and touching a wall pushes the car outward along the wall's
  normal and upward; has no effect while grounded even if `wall_normal` is
  `Some`; takes priority over the double jump without consuming it; and
  merely touching a wall (no jump press needed) restores the double jump.
  An end-to-end `PhysicsWorld::step` test confirms a car resting against a
  wall wall-jumps outward and upward on a fresh press; a second end-to-end
  test confirms a ball shot at a wall bounces off it instead of tunnelling
  through — the same physical proof `ball_bounces_off_a_stationary_car_instead_of_passing_through` already gives for cars, now for the generic
  plane-collision machinery walls reuse; a regression test confirms a car
  not actually touching an existing wall still gets a plain double jump,
  not a wall jump.
- FR-014 (met, dodge): a fresh double-jump press with `pitch` (forward) or
  `roll` (sideways) held gives horizontal velocity along the matching axis
  plus a visible spin, in the opposite direction for opposite stick sign; a
  deflection below `DODGE_DEADZONE` still gives a plain double jump;
  either way the press spends `double_jump_available`; a diagonal
  (`pitch`+`roll`) press combines both axes; dodge logic never fires while
  grounded (the ground jump owns that branch entirely) or while touching a
  wall (the wall jump fires its own fixed push-off regardless of stick
  input). An end-to-end `PhysicsWorld::step` test confirms a car dodges
  forward with a visible flip after a ground jump, and a regression test
  confirms a car touching a wall with directional stick input still gets
  the wall jump's own (smaller, purely horizontal-plus-vertical) push-off
  rather than the dodge's (larger, purely horizontal) one. All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014 behavior covered
  by `rb_physics_bullet`'s unit tests (120 tests as of the 0.14.0 version).
- FR-015 (met, variable jump height): holding jump after a fresh ground
  jump adds more upward velocity than tapping it; releasing jump early
  stops the extra acceleration immediately, even with hold-window time
  left; the extra acceleration stops accruing once
  `JUMP_HOLD_MAX_DURATION` has elapsed, even if still held; a double jump
  fired after holding the ground jump through its whole window still adds
  exactly one more `JUMP_SPEED` kick, not an extra variable-height boost.
  An end-to-end `PhysicsWorld::step` test confirms a held ground jump
  reaches a greater peak height than a tapped one; a regression test
  confirms the same double-jump-unaffected property holds through a live
  `PhysicsWorld::step` loop, not just in `drive.rs` isolation; a second
  regression test (`holding_jump_does_not_repeatedly_relaunch_the_car`,
  extended for the longer flight time variable height now produces) still
  confirms holding jump for a car's entire flight lands and settles it
  instead of relaunching it every touchdown. All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014/FR-015 behavior
  covered by `rb_physics_bullet`'s unit tests (126 tests as of the 0.15.0
  version).
- FR-016 (met, flip-cancel): a dodge leaves the car spinning and sets a
  cancelable-flip flag; a further fresh jump press while airborne, not
  touching a wall, with the double jump already spent, zeroes the spin
  outright and spends the flag; flip-cancel touches neither the dodge's own
  linear velocity nor `double_jump_available`; a plain double jump (no
  stick input) explicitly clears any stale cancelable-flip flag left over
  from an earlier, already-landed-from dodge, so a later unrelated press
  can't spuriously flip-cancel nothing; a wall jump still takes priority
  over flip-cancel on a fresh press while touching a wall. An end-to-end
  `PhysicsWorld::step` test confirms a second jump press cancels a dodge's
  spin in a live scene; a regression test confirms landing and a later
  plain double jump clear a stale cancelable-flip flag there too, not just
  in `drive.rs` isolation — verified by confirming both the `drive.rs` and
  `world.rs` versions of that regression actually fail without the
  explicit-clear fix, not just that they pass with it. All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014/FR-015/FR-016
  behavior covered by `rb_physics_bullet`'s unit tests (132 tests as of
  the 0.16.0 version).
- FR-017 (met, wall-jump dodge): a fresh wall-jump press with directional
  stick input at or above `DODGE_DEADZONE` fires a combined wall-push-plus-
  dodge impulse and a visible spin instead of the plain fixed push-off;
  below the deadzone the plain wall jump fires unchanged; the dodge variant
  consumes `double_jump_available` while the plain variant still doesn't;
  its spin can be flip-cancelled by a further press, exactly like a ground
  dodge's; opposite stick sign dodges the opposite direction; a diagonal
  (pitch+roll) wall-jump dodge combines both axes. An end-to-end
  `PhysicsWorld::step` test confirms the wall-jump dodge fires in a live
  scene; a second end-to-end test confirms its spin is flip-cancelable
  there too. Two pre-existing tests whose premise this requirement
  deliberately reverses (`drive::wall_jump_fires_instead_of_a_dodge_when_touching_a_wall`, `world::wall_jump_still_fires_instead_of_a_dodge_when_touching_a_wall`) were repurposed, not silently deleted, to assert the
  new behavior. All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014/FR-015/FR-016/FR-017
  behavior covered by `rb_physics_bullet`'s unit tests (138 tests as of
  the 0.17.0 version).
- FR-018 (met, landing auto-orientation assist): a tilted airborne car with
  no pitch/roll input gets a corrective torque; an already-upright airborne
  car gets none; the assist has no effect while grounded; and it doesn't
  fire while pitch or roll air control is actively held (checked via a
  tilt whose own correction axis is orthogonal to full pitch's own torque
  axis, so the two contributions can be cleanly told apart). An end-to-end
  `PhysicsWorld::step` test (gravity zeroed) confirms a car tilted 90
  degrees with no input trends back toward level over repeated steps rather
  than staying tilted or drifting further away. A pre-existing regression
  test (`landing_and_a_new_double_jump_clears_a_stale_dodge_flip_flag_in_a_live_world`) was loosened from exact equality to a small tolerance, since
  the assist now legitimately nudges angular velocity by a tiny amount on
  the test's intervening neutral-input step — still tight enough to catch a
  real regression (a spurious flip-cancel zeroing ~1.5 rad/s), which would
  dwarf the assist's own per-step contribution. All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014/FR-015/FR-016/FR-017/FR-018
  behavior covered by `rb_physics_bullet`'s unit tests (143 tests as of
  the 0.18.0 version).
- FR-019 (met, modeled arena footprint): `standard_walls` returns exactly
  9 planes; the arena's center is on the playable side of every one of
  them; opposing side/back walls share one offset magnitude by
  construction; a point just past a side wall is no longer on the
  playable side; the ceiling bounds from above (playable below `CEILING_Z`,
  not above); a corner wall actually cuts off the true rectangular corner
  (that point is not on the playable side of its corresponding corner
  wall); all four corner walls share one offset magnitude. An end-to-end
  `PhysicsWorld::standard_arena` test confirms it carries exactly 9 walls
  and the standard ground; a second confirms a ball shot at the standard
  arena's side wall bounces off it rather than escaping (the same physical
  proof FR-013 already gave for an ad-hoc test wall, now for the real field
  dimension); a third confirms a ball fired straight at the true
  rectangular corner is stopped by the diagonal corner wall well before its
  x or y individually reaches either the side or back wall's own position —
  proof the corner cut is real physical geometry, not decoration. All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014/FR-015/FR-016/FR-017/FR-018/FR-019
  behavior covered by `rb_physics_bullet`'s unit tests (153 tests as of
  the 0.19.0 version).
- FR-020 (met, curved wall-to-floor/wall-to-ceiling transitions):
  `StaticQuarterPipe::between_planes` derives an axis sitting exactly
  `radius` units in from both bridged planes, with `sector_start`/
  `sector_end` pointing toward each plane's own tangent point (both unit
  vectors, perpendicular to each other), and those tangent points lying
  exactly on their respective planes. `sphere_vs_quarter_pipe`: a sphere
  deep inside the pipe has no contact; a sphere touching the pipe's own
  radius (from inside) has zero penetration; a sphere pushed past that
  radius has positive penetration with the correction pointing back toward
  the axis (not away from it, unlike a flat plane); a sphere outside the
  fillet's 90-degree sector has no contact regardless of absolute distance;
  a box always gets no contact (the documented deferred case). An
  end-to-end `PhysicsWorld` test confirms a ball resting at ordinary
  flat-floor height within a curve's footprint — already overlapping the
  fillet's own material — gets pushed up off that flat height instead of
  staying embedded, the real proof the curve is live physical geometry, not
  a detection hack; a regression test confirms a car (box) sitting in the
  exact same position is completely unaffected, staying at its ordinary
  flat-floor resting height. All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014/FR-015/FR-016/FR-017/FR-018/FR-019/FR-020
  behavior covered by `rb_physics_bullet`'s unit tests (168 tests as of
  the 0.20.0 version).
- FR-021 (met, curved corner-wall-to-floor/wall-to-ceiling transitions):
  `standard_curves` returns exactly 16 fillets instead of 8; every fillet's
  axis sits exactly `FILLET_RADIUS` in from some vertical wall — a side
  wall, a back wall, or a diagonal corner wall — not just a cardinal one;
  a corner wall's own derived fillet axis sits exactly `FILLET_RADIUS` in
  from both the corner wall and the floor, with the same perpendicular
  unit-vector sector properties FR-020 already proved for the cardinal-wall
  case; the cross product computing a corner wall's `axis_direction` is
  exactly unit length for every one of the 4 quadrants, confirming the
  production code's `.normalize()`-free assumption actually holds rather
  than merely compiling. An end-to-end `PhysicsWorld` test, built around a
  wall with a diagonal (non-axis-aligned) normal rather than going through
  `arena::standard_curves` directly, confirms `between_planes` genuinely
  generalizes to a non-cardinal wall: a ball resting at ordinary flat-floor
  height within that diagonal wall's fillet footprint gets pushed up off it,
  the same real physical-geometry proof FR-020 gave for a cardinal wall,
  now for one whose normal isn't a coordinate axis. All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014/FR-015/FR-016/FR-017/FR-018/FR-019/FR-020/FR-021
  behavior covered by `rb_physics_bullet`'s unit tests (172 tests as of
  the 0.21.0 version).
- FR-022 (met, curved corner-wall vertical-edge fillets): `standard_curves`
  returns exactly 24 fillets instead of 16; every fillet's axis (all 24,
  floor/ceiling-seam and vertical-edge alike) sits exactly `FILLET_RADIUS`
  in from some vertical wall; every vertical-edge fillet's own
  `axis_direction` runs purely along Z, unlike a floor/ceiling-seam
  fillet's horizontal one; a corner wall's own derived vertical-edge fillet
  axis sits exactly `FILLET_RADIUS` in from both the corner wall and its
  neighboring side wall, with a sector spanning exactly the 45-degree angle
  between their two normals (not the floor-seam fillets' 90 degrees).
  `between_planes`'s generalization is independently verified with a
  synthetic non-perpendicular fixture (a wall meeting a second wall at 45
  degrees, unrelated to the arena's own geometry): the derived axis still
  sits exactly `radius` in from both planes with tangent points exactly on
  each; the derived sector angle matches the angle between the two planes'
  normals exactly; the sharp corner the fillet replaces sits outside its
  own radius but within its sector (the real proof the generalized sector
  orientation actually faces the missing material, not away from it); and
  passing either of the two opposite directions as `axis_direction`
  produces the same correctly-oriented sector either way, confirming the
  self-correction is real and not an artifact of a particular input sign.
  An end-to-end `PhysicsWorld` test confirms a ball embedded past a
  vertical-edge fillet's own radius (deep in what would otherwise be the
  sharp corner sliver, at a wall-to-wall angle that isn't a right angle)
  gets pushed meaningfully back toward the axis — not a claim that it
  settles and stays at the exact resting distance, since (like every other
  fillet in this port) its contact stops firing once the overlap resolves,
  after which nothing cancels whatever residual velocity the correction
  left the ball with; `RB-PHYSICS-001-FR-020`'s and `FR-021`'s own
  equivalent tests make the same "moved meaningfully," not
  "settled-and-stayed," claim for exactly this reason. All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014/FR-015/FR-016/FR-017/FR-018/FR-019/FR-020/FR-021/FR-022
  behavior covered by `rb_physics_bullet`'s unit tests (181 tests as of
  the 0.22.0 version).
- FR-023 (met, compound-corner fillets): `StaticCornerFillet::between_three_planes` places its center exactly `radius` units in from
  all three bridged planes, with each plane's own tangent point lying
  exactly on that plane; its 3 `bounds` correctly include the direction
  toward the sharp corner it replaces and exclude the direction pointing
  straight away from that corner. `standard_corner_fillets` returns exactly
  16 fillets; every one of their centers sits exactly `FILLET_RADIUS` in
  from some floor-or-ceiling plane, some side-or-back wall, and some corner
  wall simultaneously — proof `between_three_planes` solved the real triple
  intersection this arena's geometry produces, not an arbitrary point.
  `sphere_vs_corner_fillet`: a sphere deep inside the fillet has no
  contact; a sphere touching the fillet's own radius (from inside) has zero
  penetration; a sphere pushed past that radius has positive penetration
  with the correction pointing back toward the center (not away from it,
  unlike a flat plane); a sphere outside the fillet's spherical-triangle
  bounds has no contact regardless of absolute distance; a box always gets
  no contact (the documented deferred case, same as every other fillet
  here). `PhysicsWorld::standard_arena` carries exactly 16 corner fillets.
  An end-to-end `PhysicsWorld` test confirms a ball embedded past a
  compound-corner fillet's own radius (deep in what would otherwise be the
  sharp, unrounded corner where three planes meet) gets pushed meaningfully
  back toward the center — the same "moved meaningfully," not
  "settled-and-stayed," claim `RB-PHYSICS-001-FR-020`'s/`FR-021`'s/`FR-022`'s
  own equivalent tests make, for exactly the same reason (a single-fire
  contact stops firing once the overlap resolves, and nothing cancels
  whatever residual velocity the correction left the ball with). All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014/FR-015/FR-016/FR-017/FR-018/FR-019/FR-020/FR-021/FR-022/FR-023
  behavior covered by `rb_physics_bullet`'s unit tests (194 tests as of
  the 0.23.0 version).
- FR-024 (met, goal cutouts): `StaticGoalWall::contains_in_window`
  correctly reports a point at the window's own center, and points just
  inside/outside each of its four edges (checked independently of the
  point's own depth from the plane, proving the test really is
  distance-along-the-plane-independent as designed). `sphere_vs_goal_wall`:
  a sphere embedded in the goal window has no contact at all (passes
  through); a sphere outside the window behaves exactly like an ordinary
  plane contact, both embedded (positive penetration) and resting exactly
  at the surface (zero penetration). `contacts_vs_goal_wall`'s box path is
  bit-for-bit identical to plain `contacts_vs_plane` against the same
  wrapped plane, proving a car really does see the same wall as before,
  window or not. `standard_walls` returns exactly 7 planes (down from 9);
  `standard_goal_walls` returns exactly 2, sharing one offset magnitude,
  each window centered exactly on its own wall at half the goal's own
  height with the documented `GOAL_HALF_WIDTH`/`GOAL_HEIGHT` half-extents.
  `standard_goal_cutout_fillets` returns exactly 6 fillets, each sitting
  exactly `FILLET_RADIUS` in from both a real back wall and a post/crossbar
  plane. `PhysicsWorld::standard_arena` carries exactly 7 walls, 30 curves
  (24 edge/corner-wall fillets plus the 6 new goal-cutout ones), and 2 goal
  walls. Two end-to-end `PhysicsWorld` tests give the real live-physics
  proof: a ball fired straight through the center of a goal-mouth window
  keeps going past the back wall's own position instead of bouncing off
  it, while a car aimed at the exact same spot is still stopped by the
  wall, completely unaffected by the window; a third end-to-end test
  confirms a ball embedded past a goal-post fillet's own radius gets
  pushed meaningfully back toward the axis, the same "moved meaningfully,"
  not "settled-and-stayed," claim every other fillet's own equivalent test
  makes, for the same residual-velocity reason. All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014/FR-015/FR-016/FR-017/FR-018/FR-019/FR-020/FR-021/FR-022/FR-023/FR-024
  behavior covered by `rb_physics_bullet`'s unit tests (211 tests as of
  this version).
- FR-025 (met, corner-wall floor/ceiling arch radius): a compile-time
  `const _: () = assert!(CORNER_ARCH_RADIUS > FILLET_RADIUS);` right after
  `CORNER_ARCH_RADIUS`'s own definition proves the "distinctly larger"
  relationship at build time rather than at runtime. `standard_curves`
  still returns exactly 24 fillets, but `every_floor_or_ceiling_seam_curve_bridges_a_wall_to_the_floor_or_ceiling` now checks the first 8 of those
  24 entries (the cardinal-wall seams) against `FILLET_RADIUS` and the
  next 8 (the corner-wall seams) against `CORNER_ARCH_RADIUS` separately,
  instead of checking all 16 floor/ceiling entries against one shared
  radius as before; `every_standard_curve_sits_radius_in_from_a_vertical_wall` now accepts either radius, since a curve's distance from its own
  bridged vertical wall is `FILLET_RADIUS` for a cardinal-wall or
  vertical-edge fillet and `CORNER_ARCH_RADIUS` for a corner-wall
  floor/ceiling seam. `standard_corner_fillets` still returns exactly 16
  fillets, but `every_standard_corner_fillets_center_sits_radius_in_from_a_floor_or_ceiling_a_side_or_back_wall_and_a_corner_wall` now checks
  `CORNER_ARCH_RADIUS` instead of `FILLET_RADIUS`, since all 16 switched
  together. A new end-to-end `PhysicsWorld` test,
  `a_ball_embedded_in_a_corner_walls_floor_arch_footprint_is_pushed_toward_the_axis`, gives the real live-physics proof: a ball embedded past
  `arena::CORNER_ARCH_RADIUS` at a diagonal corner wall's own floor seam —
  deep enough that it would sit outside a plain `arena::FILLET_RADIUS`
  fillet's own footprint entirely — still gets pushed meaningfully back
  toward the axis, the same "moved meaningfully," not
  "settled-and-stayed," claim every other fillet's own equivalent test
  makes, for the same residual-velocity reason, and the test additionally
  asserts `CORNER_ARCH_RADIUS > FILLET_RADIUS` directly. While validating
  this change, the pre-existing `world.rs` end-to-end test
  `a_ball_shot_through_the_goal_mouth_passes_the_standard_arenas_back_wall`
  (FR-024) started failing: `StaticQuarterPipe` is documented as infinite
  along its own axis, not clipped to a corner wall's real, finite span, so
  a ball fired dead down the arena's own center line eventually re-enters
  *some* corner-wall floor-seam arch's resting shell far past the goal —
  already true before FR-025 with the smaller `FILLET_RADIUS` (verified
  directly against the pre-FR-025 code, where the ball drifts into this
  zone around y≈7650-7930 and gets a mild, harmless correction that still
  leaves it past the wall), but FR-025's bigger `CORNER_ARCH_RADIUS` moves
  that zone closer in (~y=6300-7700) and turns the same brush into a much
  sharper, solver-destabilizing correction (velocity spikes to tens of
  thousands of units/sec, throwing the ball back past the wall and failing
  the test's assertion). This is a discovered-and-fixed test-scoping
  issue, not a new capability or a new documented Non-goal —
  `StaticQuarterPipe`'s "infinite along its own axis" simplification was
  already documented in `body.rs` before this increment. The fix:
  shortened that one test's simulated flight duration from 3.0s to 1.8s,
  comfortably long enough to prove the ball clears the back wall (needs
  y > 5121, reaches y=5400 unobstructed by 1.8s) while stopping well short
  of re-entering the infinite-fillet zone, with a code comment in the test
  explaining why. All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014/FR-015/FR-016/FR-017/FR-018/FR-019/FR-020/FR-021/FR-022/FR-023/FR-024/FR-025
  behavior covered by `rb_physics_bullet`'s unit tests (212 tests as of
  this version — net +1 over 0.24.0's 211, since one new `arena.rs` test
  idea was implemented as the compile-time const-assert above instead of a
  runtime test, alongside the one new `world.rs` end-to-end test).
- FR-026 (met, goal post-crossbar corner fillets):
  `standard_goal_corner_fillets` returns exactly 4 fillets; every one of
  their centers sits exactly `FILLET_RADIUS` in from some back wall, some
  post plane, and the crossbar plane simultaneously — proof
  `between_three_planes` solved the real triple intersection this goal's
  own geometry produces, not an arbitrary point, the same "prove the real
  triple intersection, not an arbitrary point" style test FR-023's own
  arena-corner test already used. `PhysicsWorld::standard_arena` carries
  exactly 20 corner fillets (16 arena-corner plus 4 goal-corner). An
  end-to-end `PhysicsWorld` test,
  `a_ball_embedded_in_a_goal_corner_fillets_footprint_is_pushed_toward_the_center`,
  gives the real live-physics proof: a ball embedded past a goal
  corner fillet's own radius, at a synthetic back-wall/post/crossbar
  3-plane fixture (not the real arena's own numbers, matching this test
  file's established convention for fillet unit tests), gets pushed
  meaningfully back toward the center — the same "moved meaningfully," not
  "settled-and-stayed," claim every other fillet's own equivalent test
  makes, for the same residual-velocity reason. All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014/FR-015/FR-016/FR-017/FR-018/FR-019/FR-020/FR-021/FR-022/FR-023/FR-024/FR-025/FR-026
  behavior covered by `rb_physics_bullet`'s unit tests (215 tests as of
  this version — net +3 over 0.25.0's 212: 2 new `arena.rs` tests
  (`standard_goal_corner_fillets_has_four_fillets` and
  `every_goal_corner_fillets_center_sits_radius_in_from_a_back_wall_a_post_and_the_crossbar`)
  plus the 1 new `world.rs` end-to-end test above).
- FR-027 (met, car deflection by curved fillets): `collision.rs` gained
  two pairs of tests, one pair per curved shape, proving the corner-testing
  approximation is real physical geometry for a box, not a detection hack
  or a silent no-op: `box_embedded_in_the_quarter_pipes_footprint_has_contact`
  centers a car box directly on a quarter-pipe's own surface (along its
  sector bisector) and confirms at least one real contact comes back, with
  every contact's normal pointing back toward the axis (the radial vector
  from the axis to the contact point has a negative dot product with the
  contact normal); `box_far_from_the_quarter_pipe_has_no_contact` places a
  box deep on the opposite side of the fillet's own angular sector — not
  just far away radially, since a point far along the sector's own bisector
  direction is still angularly "inside" the wedge regardless of distance —
  and confirms no contact at all; `box_embedded_in_the_corner_fillets_footprint_has_contact`
  and `box_outside_the_corner_fillets_bounds_has_no_contact` give the same
  two proofs for the sphere-shaped compound-corner fillet
  (`StaticCornerFillet`) instead of the cylindrical quarter-pipe. `world.rs`
  gained two live end-to-end `PhysicsWorld` proofs:
  `a_car_resting_within_a_curved_transitions_footprint_is_pushed_up_off_the_flat_floor_height`
  replaces the old `a_car_is_not_deflected_by_a_curved_transition` regression
  test (whose entire premise this requirement reverses) — a car resting at
  flat-floor height, at the exact position the analogous ball test already
  uses (already overlapping a wall-to-floor curve's own material), gets
  pushed up off that height, mirroring the ball's own equivalent proof;
  `a_car_embedded_in_a_compound_corner_fillets_footprint_has_its_penetration_reduced`
  checks, for a car embedded in a compound-corner fillet, that the worst
  (maximum) corner penetration this fillet reports shrinks meaningfully
  after the solver runs — deliberately not that the box's own center of
  mass moves closer to the fillet's center, the way the equivalent ball
  test does, since a sphere is a single point but an oriented box has
  multiple corners at different depths simultaneously: resolving one
  corner's contact can rotate the box in a way that moves its center of
  mass *away* from the fillet's center even as every individual corner's
  own overlap shrinks, so "distance from center-of-mass to fillet center"
  isn't the right invariant for a box the way it is for a sphere. This was
  found empirically while writing the test — an earlier version asserting
  center-of-mass distance shrank actually failed (the car ended up farther
  from the fillet's center even though the fillet was correctly resolving
  penetration), which is what led to writing the assertion this more
  careful, per-corner way instead. All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014/FR-015/FR-016/FR-017/FR-018/FR-019/FR-020/FR-021/FR-022/FR-023/FR-024/FR-025/FR-026/FR-027
  behavior covered by `rb_physics_bullet`'s unit tests (218 tests as of
  this version — net +3 over 0.26.0's 215: the 4 `collision.rs` tests above
  replace the 2 pre-existing "always empty for a box" regression tests
  FR-027's behavior change made obsolete, net +2, plus the 2 `world.rs`
  tests above replace the 1 pre-existing `a_car_is_not_deflected_by_a_curved_transition`
  regression test, net +1).
- FR-028 (met, car actually driving into a goal): `collision.rs` replaced
  the old `box_vs_goal_wall_ignores_the_window_entirely` regression test,
  whose entire premise this requirement reverses, with three new tests —
  `box_squarely_inside_the_goal_window_has_no_contact` (every corner
  inside the window gives an empty manifold, the box equivalent of the
  pre-existing `sphere_embedded_in_the_goal_window_has_no_contact`),
  `box_straddling_the_goal_window_edge_only_collides_on_the_corners_still_outside_it`
  (a car centered exactly on the window's own edge gets exactly 2
  contacts, only on the corners whose x-coordinate falls outside the
  window — the real proof of the partial-block behavior this
  requirement's own entry describes), and
  `box_entirely_outside_the_goal_window_behaves_like_an_ordinary_plane`
  (a car nowhere near the window collides identically to plain
  `contacts_vs_plane`, the box equivalent of the pre-existing
  `sphere_outside_the_goal_window_behaves_like_an_ordinary_plane`) — net
  +2 (3 new replacing 1 old). `world.rs` replaced
  `a_car_is_still_stopped_by_the_standard_arenas_back_wall_at_the_goal_mouth`,
  whose entire premise this requirement reverses, with
  `a_car_shot_through_the_goal_mouth_passes_the_standard_arenas_back_wall`
  — a live end-to-end `PhysicsWorld` proof: a car fired at the exact same
  goal-mouth-center position/velocity the pre-existing ball test
  `a_ball_shot_through_the_goal_mouth_passes_the_standard_arenas_back_wall`
  uses ends up past `BACK_WALL_Y` instead of being stopped, reusing that
  test's same 1.8s flight-duration bound for the same already-documented
  reason (see FR-025's own entry, unrelated to FR-028 itself) — and added
  a new regression guard,
  `a_car_aimed_away_from_the_goal_mouth_is_still_stopped_by_the_back_wall`
  (a car aimed well outside `GOAL_HALF_WIDTH`, at the solid part of the
  wall, is still stopped by it after 3.0s, proving this requirement only
  opens the window itself, not the rest of the wall) — net +1 (2 new
  replacing 1 old). All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014/FR-015/FR-016/FR-017/FR-018/FR-019/FR-020/FR-021/FR-022/FR-023/FR-024/FR-025/FR-026/FR-027/FR-028
  behavior covered by `rb_physics_bullet`'s unit tests (221 tests as of
  this version — net +3 over 0.27.0's 218: 2 net in `collision.rs` plus 1
  net in `world.rs`).
- FR-029 (met, modeled goal interior): `body.rs` gained 4 new tests for
  `StaticBoundedWall::contains_in_bound`
  (`contains_in_bound_is_true_for_the_bounds_own_center`,
  `contains_in_bound_is_true_just_inside_each_edge`,
  `contains_in_bound_is_false_just_outside_each_edge`,
  `contains_in_bound_ignores_distance_from_the_plane_itself`) — mirroring
  the pre-existing `StaticGoalWall::contains_in_window` tests exactly, just
  with the boolean gate meaning inverted. `collision.rs` gained 5 new tests
  against a synthetic fixture
  (`sphere_inside_the_bound_behaves_like_an_ordinary_plane`,
  `sphere_outside_the_bound_has_no_contact`,
  `box_squarely_inside_the_bound_behaves_like_an_ordinary_plane`,
  `box_straddling_the_bounds_edge_only_collides_on_the_corners_still_inside_it`,
  `box_entirely_outside_the_bound_has_no_contact`), mirroring the
  `StaticGoalWall` collision tests with the gate inverted. `arena.rs`
  gained 8 new tests proving the new geometry functions place things
  correctly (`standard_goal_back_walls_has_two_walls`,
  `every_goal_back_wall_sits_goal_depth_behind_the_real_back_wall`,
  `standard_goal_side_walls_has_four_walls`,
  `every_goal_side_walls_plane_matches_some_goal_post_plane`,
  `every_goal_side_walls_bound_covers_the_real_goal_depth_and_height`,
  `standard_goal_roofs_has_two_roofs`,
  `every_goal_roofs_plane_is_the_goal_crossbar_plane`,
  `every_goal_roofs_bound_covers_the_real_goal_width`) — the same "prove
  the real geometry, not an arbitrary point" discipline this crate's other
  arena tests already use. `world.rs` gained 1 new wiring-count test
  (`standard_arena_has_six_bounded_walls`) and 3 new live end-to-end
  `PhysicsWorld` proofs —
  `a_ball_shot_into_the_goal_is_stopped_by_the_goal_back_wall`,
  `a_ball_shot_sideways_inside_the_goal_is_stopped_by_a_goal_side_wall`,
  `a_ball_shot_upward_inside_the_goal_is_stopped_by_the_goal_roof` —
  each deliberately isolated to a minimal `PhysicsWorld` built from just
  the specific new wall(s) under test rather than the full
  `PhysicsWorld::standard_arena` every other end-to-end goal test in this
  file uses (see Verification plan for the two real test-design findings —
  a sector-membership isolation issue and a wall-restitution-zeroing fix —
  this discovery led to); `PhysicsWorld.walls` growing from 7 to 9 real
  entries also renamed the pre-existing
  `standard_arena_has_seven_walls_and_the_standard_ground` to
  `standard_arena_has_nine_walls_and_the_standard_ground` (a test-count
  correction, not a new test). All
  FR-007/FR-008/FR-009/FR-010/FR-011/FR-012/FR-013/FR-014/FR-015/FR-016/FR-017/FR-018/FR-019/FR-020/FR-021/FR-022/FR-023/FR-024/FR-025/FR-026/FR-027/FR-028/FR-029
  behavior covered by `rb_physics_bullet`'s unit tests (242 tests as of
  this version — net +21 over 0.28.0's 221: 4 in `body.rs`, 5 in
  `collision.rs`, 8 in `arena.rs`, 4 in `world.rs` — the renamed
  `world.rs` wall-count test is not counted, since it's a rename, not a
  new test).
- FR-005 (open): acceptance criteria defined when that work starts.

## Verification plan

Unit tests (existing) for physical sanity; `RB-VERIFY-003` divergence
scoring against real replay/BakkesMod ball *and car* trajectories once
`RB-VERIFY-001`/`RB-VERIFY-002` exist — that comparison is what actually
validates (or invalidates) the placeholder constants and this port's
fidelity to Rocket League's real ball/car behavior, not the unit tests
alone. In particular, no real data has yet exercised the box/multi-contact,
ball-vs-car, or car-vs-car collision paths at all — the unit tests confirm
internal physical consistency (a level box stays level, an anisotropic
inertia tensor behaves correctly, a collision conserves momentum), not
fidelity to a real car's actual resting/tumbling/hitting behavior, or to
how many real cars are ever mutually touching at once — this port's
combined solve (`RB-PHYSICS-001-FR-030`) shares its iteration budget
across every dynamic-vs-dynamic manifold touching in a step rather than
solving each pair fully independently, but is itself only proven against
a synthetic symmetric-pinch scenario, not real recorded multi-car
contact data.
`drive::apply_driven_forces`'s constants are even further from validated:
`MAX_CAR_SPEED`, `MAX_BOOST`,
`BOOST_ACCELERATION_GROUND`/`BOOST_ACCELERATION_AIR` (the single flat
`BOOST_ACCELERATION` this bullet used to name, split by
`RB-PHYSICS-001-FR-056` into the two distinct values RocketSim's own
source actually cites), and `JUMP_SPEED` are
commonly-cited community numbers, but `THROTTLE_ACCELERATION`,
`BOOST_CONSUMPTION_RATE`, `STEER_TORQUE`, `HANDBRAKE_FRICTION_MULTIPLIER`,
`AIR_CONTROL_TORQUE`, `WALL_JUMP_HORIZONTAL_SPEED`, `DODGE_SPEED`,
`DODGE_ANGULAR_SPEED`, `JUMP_HOLD_MAX_DURATION`,
`JUMP_HOLD_ACCELERATION`, and `LANDING_AUTO_UPRIGHT_TORQUE` are this
project's own simplifications (or, for
`STEER_TORQUE`/`HANDBRAKE_FRICTION_MULTIPLIER`/`AIR_CONTROL_TORQUE`/
`WALL_JUMP_HORIZONTAL_SPEED`/`DODGE_SPEED`/`DODGE_ANGULAR_SPEED`/
`JUMP_HOLD_MAX_DURATION`/`JUMP_HOLD_ACCELERATION`/
`LANDING_AUTO_UPRIGHT_TORQUE`, uncalibrated
placeholders with no public reference at all) — the unit
tests confirm the *shape* of the response (accelerates, caps at max speed,
yaws when moving not when parked, boosts regardless of ground contact,
drains the tank at a constant rate even once the force itself stops
applying, slides more under reduced handbrake friction than under normal
grip, jumps once per fresh press, spins about the correct axis from a
standing start in the air, can spend exactly one extra airborne jump per
airborne period, pushes outward from a touched wall with no such limit,
dodges in the stick's direction with a visible flip when that jump is
spent with pitch or roll held (a wall jump included, when that press's
stick input exceeds the same deadzone), climbs higher the longer the
ground jump button stays held up to a cap, stops a dodge's spin
outright — a wall-jump dodge's included — on a further press before
landing or a wall touch, and gently nudges a tilted airborne car back
toward level when the player isn't otherwise steering it), not that a real
car's throttle/steer/boost/handbrake/jump/double-jump/wall-jump/
wall-jump-dodge/dodge/air-control/hold-height/flip-cancel/landing-assist
response actually matches these curves.
Flip-cancel itself introduces no new constant to calibrate — it's
a state-flag-gated zeroing action, not a magnitude, so it inherits no
validation burden beyond `DODGE_SPEED`/`DODGE_ANGULAR_SPEED`'s own (the
spin it cancels). The double jump reuses `JUMP_SPEED` rather than
introducing a second speed constant, so it inherits that constant's
validation status as-is; the wall jump reuses `JUMP_SPEED` for its
vertical component but introduces `WALL_JUMP_HORIZONTAL_SPEED` for its
horizontal one; the dodge introduces its own `DODGE_SPEED` (horizontal)
and `DODGE_ANGULAR_SPEED` (spin) rather than reusing either — this port
has no public reference for a double-jump-, wall-jump-, or dodge-specific
number to reuse instead of inventing its own — real Rocket League's actual
impulses for these may differ from the ground jump's and from each other,
which this port doesn't model. Variable jump height introduces its own
`JUMP_HOLD_MAX_DURATION` (the hold window's length) and
`JUMP_HOLD_ACCELERATION` (the continuous force applied within it) —
likewise this port's own invention, with no public reference for real
Rocket League's actual hold-window length or acceleration curve.
`AIR_CONTROL_TORQUE` is additionally a
per-axis simplification: real Rocket League's pitch/yaw/roll rates differ
from each other, and this port shares one constant across all three; the
dodge reuses those same axis/sign conventions for its own direction, but
not `AIR_CONTROL_TORQUE`'s magnitude. The wall-jump dodge introduces no new
constant either — it reuses `DODGE_SPEED`/`DODGE_ANGULAR_SPEED`/
`WALL_JUMP_HORIZONTAL_SPEED` outright, so it inherits exactly those
constants' existing (unvalidated) status; its one behavioral choice this
port made up rather than measured — that it consumes
`double_jump_available` while the plain wall jump doesn't — is a structural
simplification, not a magnitude, and is called out in FR-017 and the
`drive` module doc comment. The landing auto-orientation assist introduces
its own `LANDING_AUTO_UPRIGHT_TORQUE` — this port's own invention (chosen
only to read as visibly gentler than `AIR_CONTROL_TORQUE` in tests, one
full order of magnitude smaller), since this port has no public reference
for real Rocket League's actual landing-assist strength or trigger
condition either; unlike every other jump-family constant, this one also
has no ground-proximity signal behind its trigger at all (see FR-018 and
Open questions). The modeled arena footprint's `SIDE_WALL_X`/`BACK_WALL_Y`
are, like `MAX_CAR_SPEED`/`JUMP_SPEED`, commonly-cited community-measured
field dimensions; `CEILING_Z` was one too, until `RB-PHYSICS-001-FR-036`
independently confirmed it (correcting its value from an earlier `2044`
to `2048` in the process) by directly reading RocketSim's and RLUtilities'
own source and by reconstructing it from real extracted collision-mesh
geometry, rather than merely citing another community project's constant
— the same FR-036 pass cross-confirmed `SIDE_WALL_X`/`BACK_WALL_Y` too.
`CORNER_LENGTH` (the octagon corner-cut inset) is, since
`RB-PHYSICS-001-FR-036`, confirmed exact against real extracted
collision-mesh data — this project's earlier claim that it was an
uncalibrated invention with no public reference at all was itself
incorrect (see FR-036's own entry). The flat corner-wall plane
`CORNER_LENGTH` positions is exact; what remains unmodeled is only the
real field mesh's own curved blend from that flat plane into the
floor/ceiling ramps this port doesn't model either, a genuinely different
(curved) collision geometry question, not a "better number" one (see
Open questions). FR-020's `arena::FILLET_RADIUS` does NOT share
`CORNER_LENGTH`'s newly-confirmed status — it remains this port's own
invention, no public reference, and only governed the ball through FR-026
(see FR-020's own Non-goals). Since
FR-027, `FILLET_RADIUS` also governs a car's own corner-testing
approximation — no real recorded car-vs-curve data has exercised either
the ball's or the car's response to it, so the constant's validation
status is unchanged by FR-027, only its reach. FR-022's,
FR-024's, and FR-026's own fillets (the 8 vertical corner-wall edges, the 6
goal-cutout edges, and the 4 goal post-crossbar compound corners) reuse
this same `FILLET_RADIUS` constant rather than
introducing a separate one each — a documented simplification, since this
port has no reason to believe a vertical-edge fillet's own radius, or a
goal post's, should match a cardinal wall's floor/ceiling-seam radius —
FR-022's own edges are visibly shallower (45 degrees) than a floor/ceiling
seam's (90), yet shared the same radius regardless even before FR-025.
FR-021's and FR-023's own fillets (the 4 corner walls' floor/ceiling seams
and all 16 compound corners) instead reuse `CORNER_ARCH_RADIUS` as of
FR-025, not `FILLET_RADIUS` — see FR-025's own entry above and Open
questions for why. `CORNER_ARCH_RADIUS` has exactly the same unvalidated
status as `FILLET_RADIUS`/`CORNER_LENGTH`: this port's own invention, no
public reference, chosen only to read as visibly larger than
`FILLET_RADIUS` in tests (enforced at compile time, not calibrated), and
governs both the ball and, since FR-027, a car's own corner-testing
approximation too, same as every other fillet radius in this crate —
unvalidated for either.
FR-026's own 4 goal post-crossbar compound-corner fillets reuse
`FILLET_RADIUS` unchanged rather than `CORNER_ARCH_RADIUS` — unlike the
arena's own compound corners (FR-023/FR-025), both edge fillets meeting at
a goal's post-crossbar vertex already share `FILLET_RADIUS`, so there is no
mismatched-radius concern here requiring a dedicated, larger arch radius.
`StaticCornerFillet::between_three_planes`'s own general
three-plane-intersection center solve and spherical-triangle `bounds`
derivation were already independently verified against a synthetic
fixture by FR-023 — FR-026 doesn't re-derive that machinery, only applies
it to a new triple of real planes (a back wall, a post, and the crossbar),
proven here by
`arena.rs`'s `every_goal_corner_fillets_center_sits_radius_in_from_a_back_wall_a_post_and_the_crossbar`
(the same "prove the real
triple intersection, not an arbitrary point" discipline FR-023's own
arena-corner test used), plus a new end-to-end `world.rs` test giving the
same live-physics "pushed meaningfully back toward the center" proof every
other fillet in this port gets.
The unit
tests confirm the fillet's *shape* of response (pushes back toward the
axis once the sphere's surface crosses the fillet's own radius from
inside, respects its own sector — 90 degrees for a floor/ceiling seam, 45
for a corner wall's own vertical edge — and, since FR-027, deflects a box
the same way via its own 8-corner testing, confirmed mathematically exact
for this containment question by `RB-PHYSICS-001-FR-032`), not that a
real ball's or car's actual wall-to-floor/wall-to-ceiling or wall-to-wall
transition behavior matches this radius or this trigger condition, at a cardinal wall, a
corner wall's floor/ceiling seam, or a corner wall's own vertical edge
alike. `StaticQuarterPipe::between_planes`'s own generalization (FR-022) —
solving the axis point as a real 2x2 linear system, and testing sector
membership via signed cross products rather than the old
perpendicular-only two-dot-products shortcut — is itself unit-tested
directly against a synthetic non-perpendicular fixture, independent of the
arena's own geometry, so its correctness doesn't rest solely on the
corner-wall numbers happening to work out. `StaticCornerFillet::between_three_planes`'s three-plane-intersection center solve and its
spherical-triangle `bounds` derivation (FR-023) are likewise independently
unit-tested against a synthetic fixture (a perpendicular floor combined
with the same 45-degree non-perpendicular wall pair `between_planes`'s own
fixture uses), not just proven out against the arena's own 4 corner-wall
quadrants. `arena::GOAL_HALF_WIDTH`/`GOAL_HEIGHT` (FR-024) are
commonly-cited community numbers, not independently confirmed by this
project — unlike `SIDE_WALL_X`/`BACK_WALL_Y`/`CEILING_Z`, which
`RB-PHYSICS-001-FR-036` did independently confirm — and only govern the
ball, same as `FILLET_RADIUS` (a car can't drive into a goal in this port
at all — see FR-024's Non-goals).
`StaticGoalWall::contains_in_window`'s own containment logic is
independently unit-tested against a synthetic fixture (not the arena's
own goal dimensions), the same "prove the general mechanism, not just
that the specific arena numbers happen to work out" discipline every
other new shape's own constructor/containment logic in this port already
gets.
`collision::box_vs_quarter_pipe`/`box_vs_corner_fillet` (FR-027) are
independently unit-tested against a synthetic fixture, not just proven
out against the standard arena's own fillets: a box centered directly on a
quarter-pipe's own surface (along its sector bisector) reports at least
one contact with every normal pointing back toward the axis, a box placed
deep on the opposite side of the fillet's own angular sector (not merely
far away radially — a point far along the sector's own bisector direction
is still angularly "inside" the wedge regardless of distance) reports no
contact, and the same two proofs hold for a `StaticCornerFillet`. The unit
tests confirm the corner-testing's own *shape* of response
(a box embedded in a fillet's footprint gets at least one real contact, a
box outside the fillet's own bounds gets none). This port still has no
GJK/EPA support-mapping machinery, but `RB-PHYSICS-001-FR-032` subsequently
proved (both analytically and via a dedicated dense-sampling test,
`no_point_on_a_boxs_face_is_ever_farther_from_a_quarter_pipes_axis_than_its_own_corners`)
that none is needed here: the once-suspected "face resting flush against a
shallow curve, with every corner just clear of the fillet while the face's
middle already overlaps it" case cannot actually occur, since this
contact's containment question is a convex-maximum one always attained at
a corner. The two live end-to-end `world.rs`
proofs take deliberately different shapes for the same reason a sphere and
a box need different invariants:
`a_car_resting_within_a_curved_transitions_footprint_is_pushed_up_off_the_flat_floor_height`
mirrors the ball's own equivalent `StaticQuarterPipe` proof directly (a
car resting at ordinary flat-floor height, already overlapping the curve's
own material, gets pushed up off that height), since a resting height is
just as well-defined for a box as for a sphere; but
`a_car_embedded_in_a_compound_corner_fillets_footprint_has_its_penetration_reduced`
deliberately checks that the *worst (maximum) corner penetration* this
fillet reports shrinks meaningfully after the solver runs, not that the
box's own center of mass moves closer to the fillet's center the way the
equivalent ball test checks distance-to-center — a real, documented
test-design decision, not just an implementation detail: an oriented box
has multiple corners at different depths simultaneously, so resolving one
corner's contact can rotate the box in a way that moves its center of mass
*away* from the fillet's center even as every individual corner's own
overlap shrinks, making "distance from center-of-mass to fillet center"
the wrong invariant for a box the way it's the right one for a sphere (a
single point). This was found empirically while writing the test — an
earlier version asserting center-of-mass distance shrank actually failed,
with the car ending up farther from the fillet's center even though the
fillet was correctly resolving every corner's penetration — which is what
led to writing the assertion the more careful, per-corner way instead.

`collision::box_vs_goal_wall` (FR-028) is independently unit-tested
against the same synthetic `goal_wall()` fixture
`sphere_vs_goal_wall`'s own tests already use, not the standard arena's
real goal dimensions: a car with every corner inside the window gets no
contact at all, a car straddling the window's own edge gets contacts on
exactly the corners still outside it (the box equivalent of a sphere's
single all-or-nothing center-point test, but resolved per-corner
instead), and a car entirely outside the window collides bit-for-bit
identically to plain `contacts_vs_plane` against the same wrapped plane.
The unit tests confirm the corner-testing's own *shape* of
response for a goal wall (a car with every corner inside the window
passes clean through, a car straddling the edge gets a real partial
block, a car outside the window is unaffected), not that it matches a
real car's actual goal-line-crossing behavior; whether a car's face
resting flush against the window's own edge, with every corner still just
clear of it while the face's middle already overlaps it, can actually
occur was a distinct question from the curved-fillet containment question
`RB-PHYSICS-001-FR-032` investigated and resolved (the window's boundary
is a flat rectangle, not a curve) — left open and unverified for the goal
wall specifically at the time, not covered by FR-032's own finding.
`RB-PHYSICS-001-FR-054` has since closed it: see that entry's own convex-hull
argument for why it collides exactly like an unwindowed plane, and its
own `box_vs_bounded_wall` finding for the one place this same question's
mirror image turned out to be a genuine, if currently unreachable, gap.
The live end-to-end `world.rs` proof,
`a_car_shot_through_the_goal_mouth_passes_the_standard_arenas_back_wall`,
mirrors the pre-existing ball test's own live-physics proof directly (a
car fired at the same goal-mouth-center position/velocity ends up past
`BACK_WALL_Y` instead of being stopped), reusing that test's already-solved
1.8s flight-duration bound rather than re-deriving it, since the
underlying `StaticQuarterPipe`-infinite-axis timing concern (see FR-025's
own entry) is unrelated to FR-028 itself; a second end-to-end test,
`a_car_aimed_away_from_the_goal_mouth_is_still_stopped_by_the_back_wall`,
confirms a car aimed at the wall's still-solid portion (outside
`GOAL_HALF_WIDTH`) is unaffected by this requirement.

`body::StaticBoundedWall::contains_in_bound` (FR-029) is independently
unit-tested against a synthetic fixture, the same discipline every other
new shape's own containment logic in this port gets: true at the bound's
own center and just inside each of its four edges, false just outside
them, and unaffected by a point's distance from the plane — mirroring
`StaticGoalWall::contains_in_window`'s own tests exactly, with the
boolean gate's meaning inverted (inside the bound instead of outside the
window). `collision::sphere_vs_bounded_wall`/`box_vs_bounded_wall` are
likewise independently unit-tested against a synthetic fixture, not just
proven out against the standard arena's own goal geometry: a sphere
inside the bound behaves exactly like an ordinary plane contact, a sphere
outside the bound has no contact at all, a box squarely inside the bound
behaves like an ordinary plane contact, a box straddling the bound's own
edge collides only on the corners still inside it (the bounded-wall
mirror of `box_vs_goal_wall`'s own straddling-the-window test, with the
gate inverted), and a box entirely outside the bound has no contact. The
unit tests confirm the bounded-wall's own *shape* of response (collides
only within its own rectangular footprint, an ordinary plane contact
everywhere inside it), not that a real ball or car's actual behavior
against a real net's back/side/roof structure matches this exactly —
`arena::GOAL_DEPTH` in particular has no public reference at all (see
Non-goals and Open questions).

Two real test-design findings came out of writing this requirement's own
3 live end-to-end `world.rs` proofs
(`a_ball_shot_into_the_goal_is_stopped_by_the_goal_back_wall`,
`a_ball_shot_sideways_inside_the_goal_is_stopped_by_a_goal_side_wall`,
`a_ball_shot_upward_inside_the_goal_is_stopped_by_the_goal_roof`), both
worth keeping here rather than only in code comments, matching this
spec's established practice of recording "why the test looks like this"
stories (see FR-025's and FR-027's own entries above). First: an early
version of these tests built the scene from the full
`PhysicsWorld::standard_arena` (the way every other end-to-end goal test
in this file does), but a ball fired sideways or upward from deep inside
the goal box got flung to bizarre, wildly wrong positions (e.g. ending up
at x=-687 after being fired only in +x). Root cause: the standard arena's
own goal-cutout post/crossbar fillets (`arena::standard_goal_cutout_fillets`,
FR-024) sit right at the window's edge, close enough to a point deep
inside the goal box to spuriously trigger the pre-existing, already-documented `StaticQuarterPipe` limitation that a fillet's sector-membership
test only checks angular position around its own axis, not radial
distance — the same category of issue the FR-025 test-writing notes above
already describe for a different corner-wall fillet. The fix: isolate
each of these 3 tests to a minimal `PhysicsWorld` built from just the
specific new wall(s) under test (`PhysicsWorld::new` plus
`with_wall`/`with_bounded_wall`, not `PhysicsWorld::standard_arena`),
sidestepping the unrelated fillet interaction entirely — the correct fix,
not a bug in `StaticBoundedWall` or `goal_back_wall_plane` themselves.
Second: an earlier version of these same 3 tests set only the ball's own
`restitution = 0.0` and got nondeterministic results for the roof test
specifically (the ball ending up *below* its own starting height). Root
cause: the wall's own default `StaticPlane::new` restitution (0.5) still
applies in the solver's contact resolution regardless of the ball's own
value, so the ball bounced back down with enough remaining simulation
time left to travel well past its start. Fixed by also explicitly
zeroing the specific wall(s)' own `plane.restitution` in each of these 3
tests before adding them to the scene, so the ball damps out
deterministically instead of bouncing indefinitely.

## Traceability

See [docs/traceability/TRACEABILITY.md](../../traceability/TRACEABILITY.md).

## Open questions

- Both genuine ambiguities `RB-PHYSICS-001-FR-031`'s constant-calibration
  audit surfaced but deliberately didn't act on for lack of confidence are
  now resolved by `RB-PHYSICS-001-FR-036`'s own dedicated follow-up (real
  source-level research, not a guess) — see its own Requirements entry for
  the full reasoning: (1) the ball radius question was itself reframed, not
  simply "92.75 is wrong, use 91.25" — real Rocket League splits an inertia
  radius (`91.25`) from a separately larger collision radius (`93.15`, the
  mesh's own collision margin), and since this port has no Bullet-style
  collision margin of its own, its single unified radius field's correct
  analog is the collision radius, so `92.75` became `93.15`, not `91.25`;
  (2) `arena::CEILING_Z` (`2044.0`) is now confirmed to share the same
  reference point as RocketSim's `ARENA_HEIGHT = 2048.f`, corrected to
  `2048.0`. Neither used real `RB-VERIFY-002` data (still blocked); both
  instead used direct RocketSim/RLUtilities source-code reading plus an
  independent real-mesh geometric reconstruction, which this project
  judged sufficient confidence to act on without waiting further.
- A combined multi-body solve for bodies simultaneously touching more than
  one other body is now implemented (see `RB-PHYSICS-001-FR-030`):
  `solver::resolve_dynamic_manifolds` shares one interleaved
  `SOLVER_ITERATIONS`-iteration budget across every dynamic-vs-dynamic
  manifold touching in a step, instead of resolving each pair with its
  own fully independent solve. `RB-PHYSICS-001-FR-041` investigated
  whether anything short of real recorded data could narrow this
  requirement's own documented extreme mass-ratio "sandwiched"
  under-convergence gap at this crate's fixed `SOLVER_ITERATIONS = 10`:
  a naive global SOR-style relaxation factor was tried first and rejected
  — any factor above `1.0` made the exact scenario FR-030's own tests
  measure genuinely *diverge* (worse than the pre-FR-030 independent-
  pairwise approach), matching standard PGS/SOR theory for a tightly-
  coupled multi-constraint body. A parameter-free `1 / k` impulse scale
  (`k` = the number of manifolds sharing a body this step) is now
  implemented instead — mathematically dominant rather than a tuned
  magic number, since it can only reduce, never increase, a shared
  body's per-iteration overshoot — narrowing the gap from ~89.5 to ~32
  units/s on FR-030's own symmetric-pinch scenario at zero added
  iteration cost, with zero effect on the overwhelming majority
  single-manifold-per-body case (`k == 1` is a mathematical no-op,
  confirmed by a dedicated bit-for-bit-equivalence test). What's still
  genuinely open: even with this fix, the sandwiched case still doesn't
  fully converge to the true simultaneous-solve answer within one call's
  fixed iteration budget — real recorded multi-car contact data would
  still be needed to know whether that residual error actually matters
  for fidelity in practice, or whether raising `SOLVER_ITERATIONS`
  itself (confirmed manually to converge much closer at 300 iterations,
  at obvious extra per-step cost — a real cost/benefit trade-off `1 / k`
  is not) is worth it before such data exists; not started.
- ~~Replicating real Rocket League's actual landing-assist trigger
  condition (proximity to the ground, via some raycast or distance query
  this port doesn't have) instead of the current continuous-whenever-
  airborne stand-in (see FR-018)~~ — resolved by `RB-PHYSICS-001-FR-060`:
  fetching RocketSim's real `Car.cpp` found there is no real "landing
  assist" gated on ground proximity to replicate in the first place. Real
  Rocket League's two closest systems, auto-flip and auto-roll, are both
  grounded and input-gated (a jump press past a roll threshold; a held
  throttle with wheel contact), not an airborne proximity check — a
  different shape of mechanic this port's own placeholder doesn't
  correspond to, not merely an unconfirmed guess about its distance
  threshold. Implementing either real system for real remains open, now a
  substantially larger feature (new grounded state machinery) rather than
  a stand-in to compare against — see FR-060's own Non-goals. (The double
  jump's own dodge — a
  directional flip off the ground/air, no wall involved — is now
  implemented as FR-014; variable jump height for the ground jump is now
  implemented as FR-015; canceling a dodge's rotation early — flip-cancel —
  is now implemented as FR-016; a dodge variant of the wall jump is now
  implemented as FR-017; a gentle landing auto-orientation assist is now
  implemented as FR-018.)
- ~~A car (box) actually being deflected by a curved fillet through genuine
  convex-vs-curved-surface narrow-phase collision machinery~~ —
  investigated and resolved, see `RB-PHYSICS-001-FR-032`: the specific
  concern motivating this (a face resting flush against a shallow curve
  under-detecting because none of its own corners individually register)
  is mathematically impossible for this contact's actual question
  (distance-from-an-axis/point is convex, so its maximum over a box is
  always at a corner — see FR-032's own entry for the full argument and
  the empirical test proving it). `box_vs_quarter_pipe`/
  `box_vs_corner_fillet`'s per-corner technique is exact for detecting
  whether *any* part of the box violates the fillet's radius, not an
  approximation of it; a genuine GJK/EPA convex-vs-convex narrow phase was
  actually built during this investigation and found to *regress* two
  real end-to-end tests, because it answered a different (nearest-point)
  question than the one this contact needs (farthest-point/containment).
  (Deflecting a car at
  all — previously this bullet's entire subject, when a car drove straight
  through every curve's or corner fillet's footprint completely unaffected
  — is now implemented via corner-testing, see FR-027.
  The compound corner where a vertical-edge fillet meets a
  floor-seam or ceiling-seam fillet, near a corner wall's own top/bottom
  endpoint, is now modeled with its own `StaticCornerFillet` sphere, as
  FR-023 — though it, the edge fillet, and the seam fillet it meets remain
  independent, additive contact sources, per `RB-PHYSICS-001`'s "single
  flat plane, single-radius edge fillet, or single-radius corner fillet
  per boundary segment" Non-goal — not a single continuously-blended
  surface across the whole octagon. The two compound corners per goal where
  a post's own edge fillet meets the crossbar's are likewise now modeled
  with a `StaticCornerFillet`, as FR-026 — built directly via
  `StaticCornerFillet::between_three_planes` on the back-wall/post/crossbar
  planes rather than derived from the two edge fillets meeting there,
  reusing `FILLET_RADIUS` unchanged since both edge fillets meeting at that
  vertex already share it, unlike the arena's own compound corners, which
  needed the distinctly larger `CORNER_ARCH_RADIUS` (see FR-025) — with the
  same independent-additive-fillets caveat as above, not a single
  continuously-blended surface. The goal's other two corners, where a post
  meets the floor, still need no such treatment: the window's own bottom
  edge sits exactly at floor level, so a post's own fillet there simply
  ends flush with the ground rather than leaving a sharp vertex to round
  off.)
- A car actually driving into a goal (now implemented, see
  `RB-PHYSICS-001-FR-028`) and a modeled bounded interior volume behind
  the goal window (now implemented too, see `RB-PHYSICS-001-FR-029`) — the
  goal-mouth window now opens onto a solid bounding box (2 back-of-net
  planes, 4 side walls, 2 roofs) a ball or car settles against instead of
  the open, unbounded space it opened onto through FR-028.
  `collision::box_vs_goal_wall`'s own per-corner approximation carries
  the same "exact per test point, an approximation of the whole shape"
  caveat the question above's fillet corner-testing already has — it
  isn't necessarily precise enough for a car actually clearing a
  goal-cutout edge fillet's own boundary right at the window's rim, only
  the flat window itself, which is what FR-028 actually tests; the same
  is true of FR-029's `box_vs_bounded_wall`, for its own bounded walls.
  ~~Still genuinely open: a **net mesh** — cloth/soft-body simulation,
  visual net sag, or a "ball tangles in netting" behavior — which FR-029
  deliberately didn't attempt; its interior is a solid bounding box, not
  a springy/catching net.~~ Implemented for the ball, see
  `RB-PHYSICS-001-FR-033`'s `net::NetMesh` (a real mass-spring grid, since
  the concrete motivation this bullet asked for turned out not to be
  needed to justify it — a "ball tangles in netting" behavior is itself
  the kind of qualitative, visually-checkable fidelity gap worth closing
  independent of a specific divergence-scoring signal). A car's own
  contact against a net (scoped out of FR-033, see its own Non-goals) is
  no longer open either — `RB-PHYSICS-001-FR-038` closed it by
  generalizing `net::NetMesh::step` to take every body that can touch the
  net, not the ball alone. Still open: a full 3D "sock" shape/visual net
  sag/bending stiffness beyond FR-033's flat structural-plus-shear-spring
  panel — see FR-038's own Non-goals for what it did and didn't touch.
- ~~Disambiguating or blending a car's simultaneous contact with two walls
  at a corner for wall-jump purposes (see FR-019's Non-goals) — physical
  collision resolution already handles this correctly regardless; only
  the wall-jump push-off direction picker (`PhysicsWorld::step`'s
  "first wall in `self.walls`" rule) isn't. FR-019's corner walls make this
  case reachable in the standard arena for the first time; still not
  exercised by any test here. Not started.~~ Implemented, see
  `RB-PHYSICS-001-FR-039`: the picker now sums every touched wall's normal
  and normalizes the result instead of picking whichever wall comes first.
- Sourcing or verifying `arena::FILLET_RADIUS`/`CORNER_ARCH_RADIUS`
  against real field mesh data (see
  FR-020/FR-021/FR-022/FR-023/FR-024/FR-025/FR-026/FR-027) — still open.
  `RB-PHYSICS-001-FR-040` looked, specifically, using this port's
  established reference tier (RocketSim/RLUtilities source, the RLBot
  wiki) and came back with no *reliable* reference for either constant —
  the one candidate it found (the RLBot wiki's uncited "wall bottom ramp
  radius: approx. 256, not circular") was deliberately not adopted: it
  doesn't distinguish `FILLET_RADIUS` from the corner walls' own bigger
  `CORNER_ARCH_RADIUS`, it explicitly disclaims being a true circular
  radius at all, and its own numeral is suspiciously identical to RLGym's
  unrelated `RAMP_HEIGHT` constant (the corner ramp's vertical height from
  the ground, a different geometric quantity), suggesting the wiki entry
  may be a mixed-up cross-reference rather than an independent
  measurement. Genuinely closing this gap needs real extracted
  collision-mesh geometry (e.g. via `ZealanL/RLArenaCollisionDumper`'s
  triangle-mesh dump), which needs the owner's own Windows/Rocket League
  environment — the same blocker `RB-VERIFY-002-FR-001` already documents,
  not something further wiki research alone can resolve. See
  `arena::FILLET_RADIUS`'s own doc comment for the full finding.
  `arena::CORNER_LENGTH` no longer belongs
  in this bullet: `RB-PHYSICS-001-FR-036` confirmed it exact against real
  extracted collision-mesh data, the same sourcing status as `SIDE_WALL_X`/
  `BACK_WALL_Y`/`CEILING_Z` (see FR-036's own entry and FR-019's). Even a
  sourced value for `FILLET_RADIUS`/`CORNER_ARCH_RADIUS` would only
  approximate the real corner/transition, which isn't a single flat plane,
  single-radius edge fillet, or single-radius corner fillet in the actual
  game. `FILLET_RADIUS` governs the 4 cardinal walls'
  floor/ceiling-seam fillets (FR-020), all 8 vertical-edge fillets
  (FR-022), all 6 goal-cutout-edge fillets (FR-024), and all 4 goal
  post-crossbar compound-corner fillets (FR-026); since FR-025, the
  4 corner walls' own floor/ceiling-seam fillets (FR-021) and all 16
  compound-corner fillets (FR-023) instead reuse the distinctly larger
  `CORNER_ARCH_RADIUS`, chosen only to read as visibly bigger than
  `FILLET_RADIUS` (enforced at compile time), not measured against real
  field mesh data either — whether the real game even uses two uniform
  transition radii at all (as opposed to a genuinely different
  corner-specific curve, a third, differently-shaped curve at the
  vertical edges, a fourth at the compound corners, and a fifth at the
  goal posts/crossbar — real Rocket League's actual goal-post radius is
  visually quite different from a wall-to-floor transition's) is itself
  unconfirmed. ~~`arena::GOAL_HALF_WIDTH`/`GOAL_HEIGHT` (FR-024) likewise
  have no independently-confirmed source, though they're commonly-cited
  community numbers like `SIDE_WALL_X`, not this port's own inventions
  like `CORNER_LENGTH`. `arena::GOAL_DEPTH` (FR-029) is a further step
  removed even from `GOAL_HALF_WIDTH`/`GOAL_HEIGHT`: this port has no
  commonly-cited community reference for the real net's depth at all, so
  it's this project's own uncalibrated invention, same status as
  `CORNER_LENGTH`, chosen only to be a visibly real interior volume
  comparable in scale to the goal mouth's own dimensions.~~ This bullet
  had gone stale: `arena::GOAL_DEPTH` (FR-029) was already confirmed
  against the current RLBot wiki's own cited value by
  `RB-PHYSICS-001-FR-036` — this passage's own "uncalibrated invention"
  framing for it directly contradicted FR-036's own Requirements entry
  and this same section's earlier paragraph above, simply never updated
  when FR-036 shipped. `RB-PHYSICS-001-FR-055` fixed the stale text and,
  while at it, closed `arena::GOAL_HALF_WIDTH`/`GOAL_HEIGHT`'s own
  genuinely-still-open question the same way: fetching that same wiki
  page directly confirmed both values exact against its own cited "Goal
  center-to-post"/"Goal height" numbers. All three goal-geometry
  constants (`GOAL_HALF_WIDTH`, `GOAL_HEIGHT`, `GOAL_DEPTH`) are now
  confirmed; only `net::NetMesh`'s own uncalibrated `arena::NET_DEPTH`
  (how far into that confirmed depth the net panel itself sits, not the
  goal box's own total depth) remains this project's own invention here.
- Calibrating `drive`'s constants (`THROTTLE_ACCELERATION`, `STEER_TORQUE`,
  `BOOST_CONSUMPTION_RATE`, `HANDBRAKE_FRICTION_MULTIPLIER`,
  `AIR_CONTROL_TORQUE`, `WALL_JUMP_HORIZONTAL_SPEED`, `DODGE_DEADZONE`,
  `DODGE_SPEED`, `DODGE_ANGULAR_SPEED`, `JUMP_HOLD_MAX_DURATION`,
  `JUMP_HOLD_ACCELERATION`, `LANDING_AUTO_UPRIGHT_TORQUE`, and re-checking
  `MAX_CAR_SPEED`/`MAX_BOOST`/`BOOST_ACCELERATION_GROUND`/
  `BOOST_ACCELERATION_AIR`/`JUMP_SPEED`) against
  real recorded driving data — needs `RB-VERIFY-002` capture data; not
  started. `RB-PHYSICS-001-FR-056` already went further than a
  re-check for the boost pair specifically: it found the single flat
  `BOOST_ACCELERATION` this bullet used to name was itself wrong (real
  Rocket League's own airborne value is distinctly higher than its
  grounded one, per RocketSim's own fetched source) and fixed it — a real
  behavioral change, not merely a confirmation, closing that half of this
  bullet's own scope; `MAX_CAR_SPEED`/`JUMP_SPEED` remain merely
  commonly-cited pending real recorded data. `RB-PHYSICS-001-FR-058`
  did the same for `THROTTLE_ACCELERATION`'s own *shape*: fetching
  RocketSim's own `Car.cpp` found its real throttle force is scaled by a
  confirmed piecewise-linear speed-taper curve, not applied flat, and
  modeled that curve directly (`drive::drive_speed_taper`) — but
  `THROTTLE_ACCELERATION`'s own peak magnitude (`1600.0`) remains
  uncalibrated, since the real reference constant it would otherwise
  come from (`THROTTLE_TORQUE_AMOUNT`) is expressed in Bullet-internal
  units that don't transfer to this port's own car body the same clean
  way the curve's unitless shape does — see FR-058's own entry for the
  full finding. That peak magnitude is still fully in scope for this
  bullet. `RB-PHYSICS-001-FR-059` did the same again for `DODGE_SPEED`'s
  own per-direction *scaling*: fetching RocketSim's own `Car.cpp` found a
  backward or side dodge's real impulse grows with current speed
  (confirmed exact ratios `2.5x`/`1.9x` at `MAX_CAR_SPEED`), modeled
  directly (`drive::dodge_speed_scale`/`dodge_pitch_is_backward`) — but
  `DODGE_SPEED`'s own base magnitude (`1400.0`) remains uncalibrated,
  since the real reference constant it would otherwise come from
  (`FLIP_INITIAL_VEL_SCALE = 500.f`) was deliberately not substituted in
  (see FR-059's own entry, including its further Non-goals: RocketSim's
  own direction-normalization for diagonal dodges — since adopted as a
  genuine fix by `RB-PHYSICS-001-FR-072` — and its continuous-torque-
  over-time spin model, still unaddressed). That
  base magnitude is still fully in scope for this bullet. This paragraph
  had gone stale in several places, corrected here:
  `DODGE_DEADZONE`
  is no longer "no public reference at all" either —
  `RB-PHYSICS-001-FR-075` confirmed it exact against RocketSim's own real
  dodge-cancellation threshold (`0.1f`), the same value this port already
  used; see that requirement's own entry for the full finding. `STEER_TORQUE`, `AIR_CONTROL_TORQUE`,
  `HANDBRAKE_FRICTION_MULTIPLIER`, and `WALL_JUMP_HORIZONTAL_SPEED` are a
  different case, not "no reference at all": `RB-PHYSICS-001-FR-065`,
  `RB-PHYSICS-001-FR-057`, `RB-PHYSICS-001-FR-066`, and
  `RB-PHYSICS-001-FR-067` respectively found real reference values (or,
  for the last, a real reference confirming no distinct value exists at
  all) for each, but none transfers onto this port's own model (a
  wheeled-vehicle raycast/tire-slip system for real steering; a
  car-mass/inertia-calibrated torque for real air control; an
  anisotropic lateral-vs-longitudinal friction split for real handbrake;
  a wheel/suspension-driven surface-tracking orientation system for the
  real wall jump's identical-to-ground-jump impulse) — see each
  requirement's own entry for the full finding.
  `DODGE_ANGULAR_SPEED` (still uncalibrated, no public reference of its
  own — its numeric equality to `MAX_CAR_ANGULAR_SPEED` is a documented
  coincidence, see that constant's own doc comment) remains fully in
  scope for this bullet. `JUMP_HOLD_MAX_DURATION`/
  `JUMP_HOLD_ACCELERATION` are no longer open at all — `RB-PHYSICS-001-FR-031`'s
  own audit already confirmed both exact against real source, this
  paragraph simply never updated when that landed. `LANDING_AUTO_UPRIGHT_TORQUE`
  likewise isn't "no reference found yet" — `RB-PHYSICS-001-FR-060` found
  real Rocket League has no matching mechanic at all to reference (see
  that requirement's own entry).
- Splitting `AIR_CONTROL_TORQUE` into distinct per-axis constants (pitch,
  yaw, roll) once real recorded air-control data exists to calibrate them
  separately — real Rocket League's three rates genuinely differ (roll
  fastest); sharing one constant is a documented simplification, not a
  claim they're actually equal. `RB-PHYSICS-001-FR-057`'s own fetch of
  RocketSim's `RLConst.h` confirmed RocketSim does define exactly such a
  split (`CAR_AIR_CONTROL_TORQUE`, a per-axis pitch/yaw/roll vector) —
  but explicitly didn't adopt it, since a torque constant (unlike
  `MAX_CAR_ANGULAR_SPEED`, the one constant that same fetch did adopt)
  is calibrated against RocketSim's own specific car mass/inertia tensor,
  which this port's placeholder car body doesn't match, the same "false
  precision" problem `RB-PHYSICS-001-FR-031`'s audit already found for
  this constant. Real recorded air-control data (not just RocketSim's own
  torque numbers) is still the real path to closing this one.
- Handbrake's real mechanic (reduced rear-wheel grip enabling a
  steering-assisted drift) doesn't map cleanly onto this port's one-box,
  uniform-friction car model (see Non-goals) — worth revisiting whether a
  front/rear friction split, or a genuine slip-angle-driven lateral force,
  is warranted once real recorded drift behavior exists to compare
  against; the current uniform temporary friction reduction is a
  deliberately simple stand-in, not a claim of mechanistic fidelity.
- Real Rocket League doesn't share one speed ceiling between throttle and
  boost (a boosting car can exceed unboosted top speed); this port reuses
  `MAX_CAR_SPEED` as boost's cap too, a documented simplification — worth
  splitting into a separate boost speed cap once real recorded top-speed
  data exists to calibrate one.
- FR-005 above.
- ~~Restitution/friction combine mode (`rb_physics_bullet::solver`
  currently averages) — `RB-PHYSICS-001-FR-043` checked this bullet's own
  prior claim that Bullet's actual default is `max` directly against
  `btManifoldResult`'s real source and found it wrong: the real default
  for both is an unclamped **product** (`a * b`), not `max`, with no `max`
  mode anywhere in the reference. This port's average is kept anyway, now
  for a correct reason: it preserves the identity `combine(a, a) == a`,
  which the reference's own product does not (`0.5 * 0.5 == 0.25`), and
  most bodies here currently share the same uncalibrated placeholder
  coefficient (see `body.rs`'s `Default` impls) — see
  `RB-PHYSICS-001-FR-043`'s own entry for the full finding. Which formula
  (if either) actually matches real Rocket League itself is unchanged by
  this correction and still needs real recorded ball/ground behavior to
  calibrate against~~ — `RB-PHYSICS-001-FR-063` found this framing itself
  was the wrong question: real Rocket League's own gameplay layer doesn't
  compute restitution/friction from any generic per-body combine formula
  at all for its own named contact-pair types (car-vs-world, car-vs-car,
  car-vs-ball) — it hardcodes a distinct value per pair, overriding
  whatever either body's own material would combine to. Most strikingly,
  `CARBALL_COLLISION_RESTITUTION = 0.0f` (a car hitting the ball has zero
  restitution-driven bounce in real Rocket League, regardless of either
  body's own material) and `CARBALL_COLLISION_FRICTION = 2.0f` (a
  friction coefficient above `1.0`, which no combine of two bodies' own
  sane per-material values could produce). This port's own
  `combine_restitution`/`combine_friction` architecture — two `f32`
  material values in, one combined value out, with no notion of *which
  kind* of pair produced them — can't represent a per-pair-type override
  without a substantially larger change (see that requirement's own
  Non-goals) — and `RB-PHYSICS-001-FR-083` finding 5 has since made that
  change: `solver::PairMaterial`, carried per dynamic manifold, lets the
  world name the pair's own values while every other pair keeps the
  per-body combine. Closed.
- Sleeping is no longer an open item — `RB-PHYSICS-001-FR-037` implemented
  it, and with it the actual fix for the *bouncy* (restitution > 0) resting
  contact that used to never settle (`RB-PHYSICS-001-FR-034`'s split
  impulse and `RB-PHYSICS-001-FR-035`'s warm-starting closed adjacent gaps
  but couldn't substitute for it, since restitution re-triggers off a fresh
  gravity-induced closing velocity every frame regardless of where the
  solver's iteration starts or how it got there). Warm-starting itself
  remains scoped to `resolve_dynamic_manifolds` only (static contacts and
  `resolve_contacts`/`resolve_contacts_between` stay un-warm-started, a
  deliberate scoping choice — see FR-035's own Non-goals); that's a
  genuinely separate, still-open item, tracked in this spec's own Non-goals
  rather than here now that it no longer shares a bullet with sleeping.
  `LINEAR_SLEEP_VELOCITY_THRESHOLD`/`ANGULAR_SLEEP_VELOCITY_THRESHOLD`/
  `SLEEP_TIME_THRESHOLD` (`body.rs`) are FR-037's own uncalibrated
  placeholders, worth revisiting once real recorded ball/car-hit behavior
  exists to compare against — see FR-037's own entry.
- `box_vs_box`'s edge-edge contact point (the midpoint of the two closest
  points on the involved edges) and its face-contact clipping's fallback
  to a single clamped-center point when clipping ever yields zero points
  are both now validated directly against Bullet's own `btBoxBoxDetector::
  dBoxBox` reference source, see `RB-PHYSICS-001-FR-042`: the edge-edge
  contact point derivation is confirmed strictly more rigorous than the
  reference's own (which uses unclamped infinite-line closest approach,
  not a proper finite-segment one); the face-clipping fallback's
  "defensive branch, never proven unreachable" framing matches the
  reference author's own identical, equally unproven judgment call, with
  this port's own choice to synthesize a contact rather than drop it (as
  the reference does) confirmed a deliberate, favorable divergence. Still
  genuinely open: `edge_contact`'s tangent sign-selection heuristic (which
  of a box's 4 candidate parallel edges is "near") — `FR-042` built and
  empirically tested a candidate fix (matching the reference's own
  normal-based approach instead of this port's center-to-center-vector
  one) against a brute-force ground truth and found it genuinely mixed,
  not adopted; a rigorous, non-heuristic nearest-edge-pair selection
  remains an open item, needing either real recorded car-vs-car contact
  data or a concrete visible-artifact motivation to justify the added
  complexity over either heuristic.

## Change history

- 0.105.0 (2026-09-06): `RB-PHYSICS-001-FR-085` added — the second
  capture session, six clips: the `2300` uu/s whole-vector speed cap
  (`clamp_linear_speed`), the goal-side edge and corner fillets
  withdrawn (a real car drives through them), the corner arch radius
  measured at `FILLET_RADIUS` (`750` was a guess), and the jump press
  tick without its suspension push (`296` recorded vs `304`) — the
  `dodge-derailment` fixture `114.17 → 73.76` uu. Three new fixtures
  (`throttle-jump` `3.3` uu, `boost-wall-entry` `3.9` uu,
  `airborne-hit` `5.7` / ball `4.8` uu). Open: the floor-to-wall curve's
  speed loss (F), the goal entry (K), two capture defects (I).
- 0.104.0 (2026-09-06): `RB-PHYSICS-001-FR-082` step (c) implemented,
  completing the wheel model: `collision::raycast_static` over the whole
  arena (`ray_vs_quarter_pipe`, `ray_vs_corner_fillet`,
  `ray_vs_goal_wall`, `ray_vs_bounded_wall`; `StaticScene` now public),
  the composite wall jump's normal from the wheels
  (`wall_contact_normal`, RocketSim's `1/√2` wall threshold), and
  `Car::_UpdateAutoRoll` (`apply_auto_roll`: `100` uu/s² into the
  surface, `80` rad/s² of levelling). Fixture unchanged (`114.17` uu);
  measurable on wall and curve scenarios no capture yet holds.
- 0.103.0 (2026-09-06): `RB-PHYSICS-001-FR-084` added — the landing and
  jump-exit contact diagnosed with a one-tick-from-recorded-state
  instrument; findings 1–3 implemented: the rays' real reach (`rest +
  travel + radius`, no `2.5` uu subtraction), the stick dead while any
  wheel touches (RocketSim's `numWheelsInContact == 0` gate), and the
  stick gate reading last tick's count. The car meets the ball on the
  recorded tick (`(1628, 2287, 815)` vs `(1602, 2148, 790)`); ball
  `79.28 → 42.19` uu, car `102.64 → 114.38` uu (the open post-hit
  suspension slam, finding 4, now lands on a correctly placed car).
- 0.102.0 (2026-09-06): `RB-PHYSICS-001-FR-082` step (b) implemented:
  the analog `handbrakeVal` (`5`/s up, `2`/s down) blending the steer
  angle and the handbrake's lateral (`0.1`) and longitudinal (`0.5 →
  0.9`) factor curves, the slip-driven `LAT_FRICTION_CURVE` from each
  mount's velocity, and the non-sticky curve of the contact normal when
  no throttle is held; `HANDBRAKE_LAT_FRICTION_FACTOR` replaced by the
  curve, `piecewise_linear` returning `1` for an empty curve. Fixture
  `117.41 → 102.64` uu (`0.46 → 0.40` rad, `229 → 200` uu/s); the ball
  `75.22 → 79.28` uu. `FR-066` fully superseded.
- 0.101.0 (2026-09-06): `RB-PHYSICS-001-FR-083` finding 5 implemented,
  closing `RB-PHYSICS-001-FR-063`: `solver::PairMaterial` per dynamic
  manifold (`CARBALL` friction `2.0` / restitution `0`, `CARCAR` `0.09`
  / `0.1`), `hit::ball_car_extra_impulse` (RocketSim's `Ball::_OnHit`
  kick: flattened `0.35`, forward-biased `0.65`, `min(Δv, 4600)` times
  the `{0.65, 0.65, 0.55, 0.30}` curve) applied after the solve with
  the once-per-two-ticks cooldown. Ball exit `(1548, 1983, 1057) →
  (1566, 2407, 957)` against the recorded `(1602, 2148, 790)`; fixture
  `139.52 → 117.41` uu, ball `91.16 → 75.22` uu.
- 0.100.0 (2026-09-05): `RB-PHYSICS-001-FR-083` findings 1–4 implemented:
  `THROTTLE_AIR_ACCEL` (`200/3` forward while airborne with throttle),
  the full `JUMP_ACCEL` hold from the press tick (`JUMP_PRE_MIN_ACCEL_
  SCALE` deleted), the flip torque on the press tick, and
  `PhysicsWorld::prime_car_wheels` at the seed. Each lands on its tick;
  the flight matches to `0.02` rad and the hit is one tick late instead
  of three (`160.19 → 139.52` uu). Two RocketSim-vs-RL residuals around
  the jump recorded: the press tick's spring push (`+8` uu/s) and the
  tick of ground effect the port lets go early.
- 0.99.0 (2026-09-05): `RB-PHYSICS-001-FR-083` added (documentation
  only): the post-hit divergence traced tick by tick to seven findings —
  the port lacks RocketSim's `THROTTLE_AIR_ACCEL` (`66.7` uu/s² forward
  while airborne with throttle), which is the `45` uu lag that makes
  the port's car hit the ball three ticks late and mid-jump; the jump
  hold applies `JUMP_ACCEL` in full from its first tick (the capture
  contradicts `JUMP_PRE_MIN_ACCEL_SCALE = 0.62`); the flip torque acts
  on the press tick; a seeded car should start with its drive fields
  primed; the car-ball hit needs `FR-063`'s per-pair material and
  `Ball::_OnHit`'s extra impulse; the capture's pitch is missing at the
  second dodge (a fixture caveat, the ratchet's floor after `6.05` s);
  and RL's wheels act one to two ticks longer after a jump than
  RocketSim's ray allows. Ranked by cost with a sequencing: findings
  1–4 in one pass, then 5, then `FR-082` (b)/(c).
- 0.98.0 (2026-09-05): `RB-PHYSICS-001-FR-082` step (a) implemented: the
  `wheels` module — four raycast wheels on the real spring-damper
  suspension with the sticky force and the `extraPushback` hard stop,
  the tire friction impulses (bilateral lateral grip, engine/brake/
  coast rolling term) with the real steer-angle curve on the front
  wheels and the real handbrake lateral factor, `on_ground` from the
  wheel count, the jump along the car's up, and the chassis meeting
  the arena at its real mount. `THROTTLE_ACCELERATION`, `STEER_TORQUE`,
  and `HANDBRAKE_FRICTION_MULTIPLIER` retired; `FR-065`/`FR-066`
  superseded. Three scoping corrections: the tire mechanism and the
  steer curve belong to step (a), and `SUSPENSION_SUBTRACTION` is `2.5`
  uu (`0.05` Bullet units), which makes the pushback a hard stop rather
  than a rest-height term. Fixture `239.55 → 160.19` uu; the port's car
  hits the ball for the first time (`mean_ball_distance` `729.95 →
  79.55` uu).
- 0.97.0 (2026-09-05): `RB-PHYSICS-001-FR-082` added (documentation
  only): the wheel/suspension/tire model scoped from RocketSim's
  `btVehicleRL` and `Car::_UpdateWheels` — tick order, the four Octane
  wheel mounts and radii, the `26.755`/`25.055` uu spring rests (the
  declared rests minus the `12` uu travel; `FR-081` finding 1's
  "compressed `≈13` uu" wording corrected), the `51.2` uu raycast,
  the spring-damper with its front/back force scales, the tire
  friction impulses with their lateral/longitudinal curves, the analog
  handbrake, throttle/brake/coast logic, the steer-angle curves, the
  sticky force, the car-up jump on a three-wheel ground test, and
  auto-roll. The constants reproduce the recorded rest height (`17.03`
  vs `17.0`, only with the half-g sticky force) and the four ticks of
  post-jump contact. Proposed design (per-car wheel state, a static-
  scene raycast, the chassis on `hitbox_center()` for static contact,
  `STEER_TORQUE`/`HANDBRAKE_FRICTION_MULTIPLIER`/the central throttle
  force retired), blast radius, and a three-step sequencing: (a)
  flat-ground wheels, (b) tire friction, (c) the rest of the arena.
- 0.96.0 (2026-09-05): `RB-PHYSICS-001-FR-081` finding 5 implemented for
  body-vs-body contact, its scoping corrected first: the real car rests on
  its wheels with the hitbox `18.4` uu clear of the ground, so the offset
  cannot apply against static surfaces until the suspension exists (a
  seeded car would fall `18` uu and miss its own jump). New
  `body::CAR_HITBOX_OFFSET`, `RigidBody::hitbox_offset`/`hitbox_center`;
  `collision::contacts_between` meets each shape at its mount, static
  routines keep the unoffset box as the wheel-support stand-in. Six new
  tests, three re-based (420 total); fixture unchanged, as expected.
- 0.95.0 (2026-09-05): `RB-PHYSICS-001-FR-081` finding 2 implemented:
  the dodge's translation impulse is applied along the car's flattened,
  horizontal forward/right (new `drive::dodge_axes_2d`, RocketSim's
  `forwardDir2D`/`rightDir2D`) in both dodge paths instead of its tilted
  3D axes. Isolated fixture: dodge-tick velocity window `121 → 88` uu/s,
  whole-run mean velocity `≈337 → ≈303` uu/s, mean rotation `0.77 → 0.68`
  rad; position unchanged (finding 1). Three new tests (414 total).
- 0.94.0 (2026-09-05): `RB-PHYSICS-001-FR-081` added (documentation
  only): the isolated fixture's remaining, post-airborne divergence
  traced tick by tick to five findings — the through-flight velocity gap
  is born in the four ticks after the ground jump, where the real car's
  wheels stay in contact while its suspension extends; the dodge impulse
  is applied along tilted 3D axes where RocketSim flattens them (`-75`
  uu/s of spurious vertical velocity); the recorded car hits the ball at
  `t = 5.758` while the port's, `172` uu behind, never does (why
  `mean_ball_distance` has been exactly `729.95` uu through every fix);
  the landing is a spring-damper suspension in the recording and a
  bouncing rigid box in the port, which then reads airborne and fires a
  dodge where the recording ground-jumps; and the real hitbox is centred
  `(13.9, 0, 20.8)` uu from the recorded position where the port centres
  it on the position. Ranked by cost with a sequencing: the 2D dodge axes
  first, the hitbox offset next, then a wheel/suspension model as its
  own entry folding in `FR-065`/`FR-066`.
- 0.93.0 (2026-09-05): `RB-PHYSICS-001-FR-071` implemented: real Rocket
  League's per-axis air-control damping (`AIR_CONTROL_PITCH_DAMPING =
  30`, `AIR_CONTROL_YAW_DAMPING = 20`, `AIR_CONTROL_ROLL_DAMPING = 50`,
  via `drive::air_control_damping`) applies every airborne step, the
  pitch and yaw terms scaled by `1 - |stick|`, inertia-cancelled through
  `CAR_TORQUE_SCALE`, mid-flip and under the pitch lock included. The
  placeholder landing auto-orientation assist (`FR-018`'s
  `LANDING_AUTO_UPRIGHT_TORQUE`) is removed — real Rocket League has no
  such mechanic (`FR-060`), and the fixture measured a wash with it.
  Isolated fixture: the rotation gap now stays under `0.1` rad through
  the whole airborne phase (the post-flip decay included); whole-run mean
  rotation `1.51` → `0.77` rad, `cars.mean_position_distance` `≈237` →
  `≈240` uu, with the divergence now starting at the landing. `FR-018`
  marked superseded, `FR-060`/`FR-068`/`FR-080` cross-referenced. 4
  tests removed, 4 added, 1 replaced, 19 re-pinned (350 in
  `rb_physics_bullet`, 411 total).
- 0.92.0 (2026-09-05): `RB-PHYSICS-001-FR-080` step (c) implemented: the
  real pitch-hold flip cancel (`FR-070`'s `1 - |pitch|` scale on the
  flip's pitch component when the signs match) replaces `FR-016`'s
  jump-press cancel, which is removed — a further press mid-flip now does
  nothing. Two corrections the real capture forced, each confirmed at the
  tick: yaw/roll stick air control stays live during the flip with only
  pitch locked (both RocketSim and RLUtilities lock all three out; the
  fixture's 77 in-window ticks fit to `0.0025` rad/s rms only with
  yaw/roll torque and the `FR-071` damping active), and
  `drive::clamp_angular_speed` moved to the end of `PhysicsWorld::step`,
  after the transform integration, where RocketSim's `_FinishPhysicsTick`
  runs (the recording turns `7.58` rad/s per tick mid-flip at a reported
  `|ω| = 5.50`; this port turned `5.50`). The flip window now matches to
  within `0.1` rad (from a `1.33` rad gap); whole-run
  `cars.mean_position_distance` `≈259` → `≈237` uu, mean velocity `≈339`
  → `≈254` uu/s, while mean rotation rose `1.14` → `1.51` rad because the
  post-window decay (`FR-071`, next) is now the unmasked gap. `FR-016`,
  `FR-057`, `FR-070`, `FR-071` entries cross-referenced; `FR-061`'s ball
  clamp placement noted as an adjacent finding. `rb_physics_bullet` 345 →
  350, ratchet `< 250` uu, 411 tests.
- 0.91.0 (2026-09-05): `RB-PHYSICS-001-FR-080` step (b) implemented: the
  real continuous flip replaces the instantaneous `DODGE_ANGULAR_SPEED`
  kick. New per-car `drive::DodgeFlip { rel_torque, elapsed }` state
  (replacing the `dodge_flip_active` flag in `apply_driven_forces` and
  `PhysicsWorld`); every airborne step for `FLIP_TORQUE_TIME = 0.65` s
  applies `FLIP_TORQUE_X = 260`/`FLIP_TORQUE_Y = 224` inertia-cancelled,
  per-tick, without `CAR_TORQUE_SCALE`, so `clamp_angular_speed` holds the
  car at `MAX_CAR_ANGULAR_SPEED` from the third tick; stick air control and
  the landing assist are locked out meanwhile, pitch for
  `FLIP_PITCHLOCK_EXTRA_TIME = 0.3` s more; `FLIP_Z_DAMP_120 = 0.35` bleeds
  `vz` per tick between `0.15` s and the window's end; landing clears the
  state. `FR-016`'s jump-press cancel stays as the interim (now ending the
  real flip too), pending step (c). Measured alone on the isolated
  fixture: `cars.mean_position_distance` `≈573` → `≈259` uu (`-55%`), max
  `≈2005` → `≈528` uu, mean velocity `≈744` → `≈377` uu/s; the remaining
  rotation gap grows inside the flip window at a pinned `|ω|` — an axis
  mismatch pointing at the fixture's held pitch and the real flip cancel,
  step (c). `rb_physics_bullet` 337 → 345, ratchet `< 300` uu, 406 tests.
- 0.90.0 (2026-09-04): `RB-PHYSICS-001-FR-080` step (a) implemented:
  `drive::DODGE_SPEED` is now RocketSim's real `FLIP_INITIAL_VEL_SCALE =
  500.0` (from the `1400.0` placeholder), and the backward dodge's
  forward-axis component carries the real `FLIP_BACKWARD_IMPULSE_SCALE_X =
  16/15` (`DODGE_BACKWARD_SCALE_X`, the one scale `FR-059` hadn't adopted).
  Measured alone on the isolated fixture: `cars.mean_position_distance`
  `≈937` → `≈573` uu (`-39%`), mean velocity distance `≈1369` → `≈744`
  uu/s, and the dodge-tick window's velocity gap `≈1032` → `≈126` uu/s —
  the jump `FR-079` left at the dodge was almost entirely the placeholder.
  One new test; `rb_verify_cli`'s ratchet tightened to `< 600` uu. Full
  workspace green (398 tests). Steps (b)/(c) unchanged, still scoped.
- 0.89.0 (2026-09-04): `RB-PHYSICS-001-FR-080` added — the real continuous
  flip torque (`FR-069`'s finding, `FR-079`'s dominant remaining gap)
  scoped for implementation, doc-only. Records the complete real
  mechanism from RocketSim's `Car.cpp`/`RLConst.h`: the flip torque is
  applied inertia-independently (`FR-079`'s path) but *without*
  `CAR_TORQUE_SCALE`, so it is a per-tick `Δω` of `≈1.87`/`2.17` rad/s
  that reaches `CAR_MAX_ANG_SPEED` in three ticks and is held there for
  `0.65` s; stick air control and damping are off while flipping; flip
  cancel is `FR-070`'s pitch-hold scale; `FLIP_Z_DAMP` bleeds vertical
  speed `×0.65`/tick in a `0.15`–`0.65` s window; pitch stays locked
  `0.3` s after. The isolated fixture confirms each piece to the tick
  (`|ω|` pinned at exactly `5.50` through `t ≈ 4.967`; `vel.z` holding at
  `-15.5` uu/s, the damping's gravity equilibrium). Also confirmed from
  the same data: the real initial dodge velocity is `FLIP_INITIAL_VEL_SCALE
  = 500` (matches the recorded dodge-tick `Δv` to `~1%`), so this port's
  `DODGE_SPEED = 1400` is `2.8x` too large — a mass-independent velocity
  change the old "false precision" objection never applied to. Proposes
  `Option<DodgeFlip { rel_torque, elapsed }>` replacing
  `dodge_flip_active`, threaded like `jump_hold_time_remaining`; lists the
  blast radius (3 dodge-spin tests, 8 flip-cancel tests, `DODGE_SPEED`/
  `DODGE_ANGULAR_SPEED` removed) and a three-step sequencing. No code
  changed.
- 0.88.0 (2026-09-04): `RB-PHYSICS-001-FR-079`'s pitch/roll sign fix
  implemented, for air control and the dodge together. The dodge was
  checked against RocketSim's `_UpdateDoubleJumpOrFlip` in the same pass
  and had the same bug three ways (pitch translation, pitch spin, and roll
  spin all inverted; only the roll translation already matched) — the
  earlier "Translation" finding's dodge-frame velocity mismatch was
  primarily this, not accumulated drift. `drive.rs` now applies air
  control's pitch about `-right_axis` and roll about `-forward`, and both
  dodge blocks form `dodge_forward = -norm_pitch` (RocketSim's own
  `dodgeDir.x`) for impulse, spin, and the backward classification
  (`dodge_pitch_is_backward` → `dodge_is_backward`), with the roll spin
  about `-forward`. Real-data effect on the isolated fixture: the last
  pre-dodge window's orientation gap `~0.13` → `~0.03` rad (`~0.22` →
  `~0.13` → `~0.03` across the three fixes — the pre-dodge divergence is
  closed), and the whole-fixture car position divergence moved for the
  first time, `≈2792` → `≈937` uu (`-66%`; max `≈5919` → `≈2606`). What
  remains is post-dodge (`FR-069`'s continuous flip torque, `DODGE_SPEED`'s
  own placeholder magnitude). 14 tests switched to real Rocket League's
  own stick convention; `rb_verify_cli`'s baseline became a ratchet
  (`< 1000` uu). Full workspace green (397 tests).
- 0.87.0 (2026-09-04): `RB-PHYSICS-001-FR-079`'s residual pre-dodge gap
  (left open by the inertia-cancellation fix) traced to a further,
  separate bug: real Rocket League's `Car.cpp::_UpdateAirTorque` applies
  pitch and roll about the *negative* of the car's own right/forward axes
  (`dirPitch_right = -GetRightDir()`, `dirRoll_forward = -GetForwardDir()`
  — only yaw's `dirYaw_up` is unnegated), while this port's `drive.rs`
  applies both about the *positive* axes. Isolated via per-tick
  angular-velocity comparison during the fixture's own second pre-dodge
  sub-phase (`pitch=-1, roll=-1` held, `t≈4.24`–`4.32`s): at `1.54°`
  orientation distance (too small to explain a sign flip via accumulated
  drift), the candidate's angular-velocity delta is almost exactly the
  negative of the recorded car's own (`(-0.047,-0.332)` vs. `(+0.056,
  +0.331)`) — confirmed against RocketSim's real source, not just
  inferred from the mismatch. Yaw, unaffected by this bug in both real and
  this port, already tracked well post-fix. No production code changed;
  the fix (negate `right_axis`/`forward` for pitch/roll specifically) is
  scoped as a Non-goal pending explicit confirmation, since it flips
  visible pitch/roll behavior for every existing air-control test.
- 0.86.0 (2026-09-04): `RB-PHYSICS-001-FR-079`'s inertia-cancellation fix
  implemented: `RigidBody` gained `apply_angular_acceleration`/
  `total_angular_accel`, integrated with no inverse-inertia multiply
  (`integrate.rs`), and `drive.rs`'s air control now applies real
  RocketSim constants (`AIR_CONTROL_PITCH_TORQUE = 130.0`,
  `AIR_CONTROL_YAW_TORQUE = 95.0`, `AIR_CONTROL_ROLL_TORQUE = 400.0`,
  `CAR_TORQUE_SCALE ≈ 0.095882`) through this new path instead of the old
  placeholder-plus-ratio scheme through `apply_torque`. A second,
  independent quantitative check (real constants alone, no reference to
  this port's own model) confirms the predicted yaw acceleration
  (`≈9.109` rad/s²) against the recorded car's own measured value
  (`≈9.12` rad/s²) even more tightly than before. Real-data effect on the
  isolated `dodge-derailment.capture.jsonl` fixture: the specific
  pre-dodge orientation gap this investigation targeted shrank `~40%`
  (`~12.5°` to `~7.4°`), but the fixture's own whole-trajectory divergence
  didn't shrink to match, since a residual gap still gets amplified by the
  dodge's own orientation-relative impulse and `FR-069`'s separate,
  still-unfixed post-dodge spin-rate mismatch continues to dominate the
  aggregate score — documented as expected, not a regression. All 336
  pre-existing `rb_physics_bullet` tests pass unchanged (qualitative
  assertions, not tied to the old model's exact values); `rb_verify_cli`'s
  own regression baseline test still passes against its existing loose
  bounds, now backed by different real numbers. Full workspace green (397
  tests).
- 0.85.0 (2026-09-04): `RB-PHYSICS-001-FR-079`'s own entry extended with a
  mechanism-level root cause for the pre-dodge orientation-rate
  divergence it left open: RocketSim's real `Car.cpp::_UpdateAirTorque`
  (and its dodge-torque/autoroll-torque call sites) pre-multiply by the
  car's own actual inertia tensor to cancel Bullet's inverse-inertia
  integration step, making `AIR_CONTROL_TORQUE` an inertia-independent
  direct angular-acceleration input in real Rocket League — while this
  port's own `apply_torque`/`integrate.rs` divide by the car's actual
  moment of inertia as usual, silently under-applying it. Confirmed
  quantitatively (predicted `≈2.211` rad/s² vs. measured `≈2.2` rad/s² for
  the candidate, vs. `≈9.12` rad/s² for the real car). A naive uniform
  `≈4.15x` scale-up of the constant was tried and rejected (helps yaw,
  hurts pitch/roll) as documented negative evidence that the fix must be
  architectural, not a scalar tweak. No production code changed (the
  scaling experiment was reverted before commit); the actual fix — an
  inertia-independent torque-application path — is scoped as a Non-goal
  pending explicit go-ahead given its likely broad test impact.
- 0.84.0 (2026-09-04): `RB-PHYSICS-001-FR-079` implemented — an isolated
  replay of `FR-077`'s own abrupt-derailment dodge, seeded fresh from the
  exact real state right before it (a new 347-frame real fixture,
  `dodge-derailment.capture.jsonl`), confirming the maneuver as the
  proximate cause and refining the leading hypothesis: an orientation-rate
  divergence begins smoothly during the grounded jump hold, *before* the
  dodge fires, which the dodge's own orientation-relative impulse then
  amplifies into a dramatically different translation kick, on top of a
  likely-separate post-dodge spin-rate mismatch matching `FR-069`. 1 new
  `rb_verify_cli` test (10 total; 396 workspace-wide). No production code
  changed — `FR-005` itself still hasn't started; see `FR-079`'s own
  entry for the full evidence chain.
- 0.83.0 (2026-09-04): `RB-VERIFY-003-FR-004`'s diagnostic actually ran
  against `FR-077`'s own real capture — the divergence is abrupt, not
  gradual: near-perfect for ~4 seconds, then a sharp derailment
  coinciding with a diagonal dodge in the recorded input. `FR-005`'s
  entry now names a concrete, falsifiable leading hypothesis (this port's
  instantaneous dodge-spin kick vs. `FR-069`'s already-documented,
  unimplemented continuous flip torque) and a starting point (replay the
  dodge in isolation and compare). See `RB-VERIFY-003`'s Verification
  plan for the full numbers and reasoning. No code change — a reading of
  real data, not a fix; `FR-005` itself still hasn't started.
- 0.82.0 (2026-09-04): `RB-VERIFY-003-FR-004`'s divergence-growth
  diagnostic (referenced from `FR-005`'s own entry and `FR-077`'s
  Non-goals) is now implemented — `rb_domain::divergence::score_windows`
  and `rb-verify --self-growth`, sanity-checked against the synthetic
  capture fixture. It still needs to run against `FR-077`'s own real
  capture before `FR-005` can start; that run is pending the owner's own
  machine. No change to this spec's own code (the diagnostic lives in
  `RB-VERIFY-003`); cross-references updated from "scoped" to
  "implemented" accordingly.
- 0.81.0 (2026-09-04): `FR-005`'s own entry, and `FR-077`'s Non-goals,
  updated to name the divergence-growth diagnostic they'd both flagged as
  a follow-up — now scoped concretely as `RB-VERIFY-003-FR-004` (a
  windowed variant of `rb_domain::divergence::score`, plus a new
  `rb-verify --self-growth` CLI mode; see that spec's Requirements for
  the full design). No code change here; the diagnostic itself is not yet
  implemented.
- 0.80.0 (2026-09-04): FR-077's real-capture run, done — the owner ran
  `cargo run -p rb_verify_cli -- --self test2.jsonl` on their own machine
  against the real capture from `RB-VERIFY-002-FR-001` (2,818 frames) and
  reported this project's first genuine fidelity number: `frames
  compared: 2818, mean ball distance: 2206.08 uu, max ball distance:
  5673.98 uu, car pairs compared: 2818, mean car position/rotation/
  velocity distance: 4508.71 uu / 2.12 rad / 1421.73 uu/s, max car
  position/rotation/velocity distance: 8798.56 uu / 3.14 rad / 3643.64
  uu/s`. A large divergence — consistent with essentially total
  trajectory decorrelation over the run's ~23-second span rather than a
  small, directly-calibratable gap (see FR-077's own Interpretation note
  for the full reasoning, including why this doesn't yet tell us whether
  the divergence is gradual or abrupt). `RB-PHYSICS-001-FR-005` remains
  not started: this single whole-run number isn't yet the right shape of
  evidence to calibrate individual constants from — a follow-up
  diagnostic into divergence growth over time is the recommended next
  step, not blind curve-fitting. No code change; `RB-VERIFY-003` updated
  in the same pass.
- 0.79.0 (2026-09-03): FR-078 implemented — retuned every existing
  `car_box`-style test helper across `rb_physics_bullet` that models a
  real car (`body.rs`/`collision.rs`/`drive.rs`/`net.rs`/`solver.rs`/
  `world.rs`) from the old placeholder half-extents (`Vec3::new(60.0,
  30.0, 18.0)`) to the confirmed real `body::CAR_HALF_EXTENTS`
  `RB-PHYSICS-001-FR-076` introduced but deliberately left every
  pre-existing call site untouched. Downstream assertions that duplicated
  the exact half-extents as a bare literal were refactored to reference
  the actual half-extents used to construct that test's own car instead,
  rather than hand-recomputing each one; the small number of assertions
  genuinely dependent on the exact resting-height value (`position.z`
  settling on the car's own half-extent) were switched to
  `CAR_HALF_EXTENTS.z`. Two solver-level tests' doc comments citing
  specific measured velocities for a symmetric ball-vs-two-cars pinch
  were re-measured and confirmed unchanged (a purely 1D, mass/velocity-
  driven collision has no dependency on the absolute half-extent value).
  No new tests (a constant-correctness change with no new behavior to
  characterize, matching `FR-036`'s own precedent) — all 335 pre-existing
  `rb_physics_bullet` tests pass unchanged; full workspace `fmt`/
  `clippy`/`test` green (388 tests workspace-wide).
- 0.78.0 (2026-09-03): FR-077 implemented (pending a real-capture run) —
  `rb_verify_cli` gained `score_capture_against_candidate`, a composition
  path that seeds a `PhysicsWorld` from a capture's own first grounded,
  neutral frame (`is_grounded_and_neutral`, a heuristic proxy for
  `FR-076`'s unset hidden jump/dodge state being accurate there) and scores
  a candidate `rb_physics_bullet` actually simulated from that capture's
  own recorded input against the capture's own recorded outcome — this
  project's first fidelity comparison with a genuine physical reason to be
  small if the physics core is accurate, unlike `score_replay_against_capture`'s
  existing mechanical-only comparison of two unrelated matches. A new
  `rb-verify --self <capture-file>` CLI mode exposes it. 3 new unit tests
  (happy path against the synthetic capture fixture, missing-file, and
  no-qualifying-frame `Malformed` cases); full workspace `cargo fmt`/
  `clippy`/`test` all green (388 tests). The one manual run this
  requirement's own scope calls for — against the real capture from
  `RB-VERIFY-002-FR-001` — still needs a real Rocket League/BakkesMod
  environment this sandbox doesn't have; numbers will be recorded here
  once it happens.
- 0.77.0 (2026-09-02): FR-076 implemented — `rb_physics_bullet` can now
  seed a `PhysicsWorld` from a recorded `PhysicsFrame`
  (`PhysicsWorld::from_frame`) and simulate it forward using a recorded
  per-tick controller-input sequence (`world::simulate_recorded`),
  producing a candidate trajectory `rb_domain::divergence::score` can
  compare against the real recording it came from — the prerequisite
  plumbing `FR-005`'s real-data calibration needs. Along the way, fetched
  and adopted RocketSim's own real car mass/hitbox (`body::CAR_MASS`,
  `body::CAR_HALF_EXTENTS`, `RigidBody::standard_car`) and ball mass
  (`body::BALL_MASS`, `RigidBody::standard_ball`, reusing `FR-036`'s own
  already-confirmed radius) — surfacing a substantial, previously-unnoticed
  discrepancy in this crate's own long-standing car hitbox test
  placeholder (real width ~44% wider than the placeholder), deliberately
  left uncorrected at every existing call site (a separate calibration FR
  of its own). 13 new unit tests (335 total, up from 322); no existing
  test's own expectations changed. See Requirements for the full scope,
  including where implementation diverged from the original scoping.
- 0.76.0 (2026-09-02): Scoped (not implemented) `FR-076`/`FR-077`, the
  prerequisite plumbing `FR-005`'s real-data calibration needs now that
  `PHASE-0-EXIT` is closed: `FR-076` extends `rb_physics_bullet` to seed a
  `PhysicsWorld` from a recorded `PhysicsFrame` and simulate it forward
  using a recorded per-tick input sequence (the exact next step
  `world::simulate`'s own doc comment already named); `FR-077` wires that
  into `rb_verify_cli` and runs it once against the real capture from
  `RB-VERIFY-002-FR-001`, producing this project's first genuine fidelity
  number. `FR-005` itself updated to note its blocker (`PHASE-0-EXIT`) is
  resolved but it doesn't start until `FR-076`/`FR-077` land. Also
  corrected 35 stale "still blocked on `PHASE-0-EXIT`" Non-goals bullets
  across earlier FR entries, now that gate is closed. No code changes; all
  existing tests unaffected.
- 0.75.0 (2026-09-01): FR-075 added and investigated (confirm
  `DODGE_DEADZONE` matches RocketSim's own real cancellation threshold —
  audit finding, documentation only) — this spec's own Open Questions
  section claimed `DODGE_DEADZONE` "still has no public reference at
  all... so it may be off by a large factor," and `FR-074`'s own Non-goals
  (mirroring `FR-073`'s identical earlier claim) separately framed
  RocketSim's all-or-nothing dodge-cancellation check as "a real but
  separate architectural difference" from this port's own independent
  per-axis trigger. Both were wrong: RocketSim's own confirmed check
  (already quoted verbatim during `FR-072`/`FR-073`/`FR-074`'s own
  investigations) is `if (abs(yaw + roll) < 0.1f && abs(pitch) < 0.1f)`,
  i.e. fires iff `abs(yaw + roll) >= 0.1 || abs(pitch) >= 0.1`. Since
  `FR-073` already folds yaw into this port's own `dodge_roll`, this
  port's own trigger (`dodge_pitch.abs() > DODGE_DEADZONE ||
  dodge_roll.abs() > DODGE_DEADZONE`) is the identical boolean decision
  once `DODGE_DEADZONE == 0.1` — the same real value, differing only in an
  unobservable strict-vs-non-strict boundary comparison. Corrected
  `DODGE_DEADZONE`'s own doc comment, the module doc's dodge paragraph,
  this spec's stale Open Questions bullet, and added forward citations
  from `FR-073`'s and `FR-074`'s own Non-goals correcting their framing.
  No code change: this port's dodge trigger already matched real Rocket
  League exactly. No new tests; all 322 pre-existing tests pass unchanged.
- 0.74.0 (2026-09-01): FR-074 added and implemented (snap a near-axis-
  aligned dodge to a pure single axis, genuine behavioral fix) —
  `FR-073`'s own Non-goals had flagged RocketSim's post-normalization
  small-component zeroing as "a separate, independent simplification," a
  mis-scoping this requirement corrects: it's a further pure
  post-processing step on `normalize_dodge_direction`'s own already-
  computed normalized pair, needing no new machinery, exactly like
  normalization itself (`FR-072`). Re-confirmed via RocketSim's `Car.cpp`:
  after `dodgeDir.safeNormalized()`, `if (abs(dodgeDir.x()) < 0.1f)
  dodgeDir.x() = 0; if (abs(dodgeDir.y()) < 0.1f) dodgeDir.y() = 0;` — not
  re-normalized afterward. Added `drive::DODGE_DIRECTION_SNAP_THRESHOLD =
  0.1` (a distinct constant from `DODGE_DEADZONE` despite sharing the same
  real value, since they serve different real purposes) and wired the
  zeroing into `normalize_dodge_direction`'s own return path — both dodge
  call sites already route through it, so no call-site changes were
  needed. Effect: a near-axis-aligned diagonal stick input now snaps to a
  clean single-axis dodge instead of a slightly diagonal one. Added 2 new
  tests pinning the snap behavior at both sides of the threshold; crate
  grows from 320 to 322, all passing.
- 0.73.0 (2026-09-01): FR-073 added and implemented (fold yaw input into
  the dodge/wall-jump-dodge direction, genuine behavioral fix) —
  `FR-059`'s own Non-goals (and `FR-072`'s own doc comment) had already
  found and flagged that this port's dodge direction reads `pitch`/`roll`
  only, never `yaw`, unlike real Rocket League's own `dodgeDir = (-pitch,
  yaw + roll, 0)`. Confirmed via RocketSim's `Car.cpp`
  (`_UpdateDoubleJumpOrFlip`) that `controls.yaw` feeds nowhere else in the
  function — only `dodgeDir`'s own combined axis. Unlike a wheeled-vehicle
  model or a continuous-torque timing state, this needed no new machinery:
  this port already reads `input.yaw` in this same function for air
  control, so folding it into the dodge's roll-axis stick value
  (`roll + yaw`, each clamped to `[-1.0, 1.0]` individually first) at both
  the ground-dodge and wall-jump-dodge call sites was a pure additive
  combination of an already-available input. The existing
  `DODGE_DEADZONE` trigger, `normalize_dodge_direction`, and speed scaling
  are otherwise unchanged; `dodge_pitch_is_backward` still reads raw pitch
  only. Not adopted: RocketSim's own all-or-nothing cancellation check
  (combined `yaw + roll` and `pitch` both under `0.1` zeroes the whole
  direction, vs. this port's independent per-axis trigger) and its
  post-normalization small-component zeroing — both separate architectural
  differences left open. Added 3 new tests exercising a yaw-only dodge, a
  yaw-and-roll cancellation, and a yaw-only wall-jump-dodge; crate grows
  from 317 to 320, all passing.
- 0.72.0 (2026-09-01): FR-072 added and implemented (normalized
  diagonal-dodge direction, genuine behavioral fix) — `FR-059`'s own
  Non-goals had already found and flagged that this port sums each dodge
  axis' own full-strength contribution independently, making a diagonal
  dodge `sqrt(2)`-ish times faster than an axis-aligned one, unlike real
  Rocket League. Fetched RocketSim's own `Car.cpp`
  (`_UpdateDoubleJumpOrFlip`) and confirmed the real mechanism: `dodgeDir
  = btVector3(-pitch, yaw + roll, 0).safeNormalized()`, normalized to unit
  length before any further speed-based scaling. Unlike a wheeled-vehicle
  model or a continuous-torque timing state, normalizing a direction
  vector needs no new machinery this port lacks, so it transfers cleanly.
  Added `drive::normalize_dodge_direction`, wired into both the ground-
  dodge and wall-jump-dodge code paths — the per-axis `DODGE_DEADZONE`
  trigger and `dodge_pitch_is_backward`'s sign check still read raw stick
  values; only the scaled magnitude changes. This port's own sign
  convention is kept and yaw isn't folded in, both already-documented,
  separate simplifications. Updated the two existing diagonal-dodge tests
  to assert the corrected magnitude and added 3 new tests for
  `normalize_dodge_direction` directly; crate grows from 314 to 317 tests,
  all passing.
- 0.71.0 (2026-09-01): FR-071 added and implemented (real air-control
  damping mechanism — audit finding, documentation only) — `FR-068`'s own
  Non-goals had already found RocketSim's `CAR_AIR_CONTROL_DAMPING =
  Vec(30, 20, 50)` exists but left it as "a separate, independent addition
  left for a future requirement" without examining the mechanism. Fetched
  RocketSim's own `Car.cpp` again (the same fetch `FR-070` used for
  `pitchTorqueScale`) and found the full mechanism: for each axis, real air
  control subtracts a damping torque `(angular velocity along that axis) *
  CAR_AIR_CONTROL_DAMPING[axis] * (1 - abs(analog input on that axis))`
  from the applied torque before scaling by inertia — releasing the stick
  gives full damping strength, continuously bleeding off spin; holding it
  fully zeroes the damping, granting full torque authority. Corrected the
  `drive` module's air-control doc comment and `AIR_CONTROL_ROLL_SCALE`'s
  own doc comment, and added a forward citation from `FR-068`'s own
  Non-goals. Not adopted: unlike `AIR_CONTROL_TORQUE`'s own pitch/yaw/roll
  ratio, this port has no existing damping quantity to apply a ratio to —
  introducing one is a genuinely new mechanism, not a multiplier transfer —
  and its absolute coefficients are calibrated against real Rocket League's
  own specific inertia tensor, the same "false precision" reasoning that
  already keeps `AIR_CONTROL_TORQUE` a placeholder. Zero production behavior
  changed, no new tests; all 314 pre-existing tests pass unchanged.
- 0.70.0 (2026-09-01): FR-070 added and implemented (real flip-cancel is
  continuous, pitch-stick-driven, and pitch-axis-only, not
  jump-press-triggered and all-axis — audit finding, documentation only) —
  `FR-069`'s own fetch of `_UpdateAirTorque` surfaced a `pitchTorqueScale`
  factor scoped out as "an additional speed- or state-dependent scale...
  didn't fully characterize." Fetched RocketSim's own `Car.cpp` again to
  close that thread and found real Rocket League's flip-cancel is driven by
  continuously *holding* pitch in the same direction as the flip's own
  pitch-torque component, scaling only that pitch-axis component by
  `1 - abs(controls.pitch)` every tick — not this port's own jump-press
  trigger that zeros every axis outright. A sideways (roll-only) dodge has
  no pitch-torque component, so real Rocket League can't pitch-cancel it at
  all. Corrected the `drive` module's flip-cancel doc comment (which had
  inaccurately claimed to match real Rocket League) and added a forward
  citation from `FR-016`'s own entry. Not adopted: this port's dodge has no
  per-axis torque split to partially cancel (`FR-069`'s own architecture
  gap applies identically here), and reproducing the real continuous-hold
  trigger and pitch-only scope would need the same per-axis torque and
  elapsed-flip-time state `FR-059`'s own Non-goals already flagged as out
  of scope. Zero production behavior changed, no new tests; all 314
  pre-existing tests pass unchanged.
- 0.69.0 (2026-09-01): FR-069 added and implemented (real dodge spin is a
  continuous per-axis torque over a fixed window, not an instantaneous
  kick — audit finding, documentation only) — `FR-031`'s own original
  audit had already found the real mechanism is a torque, not a flat kick
  (`RLConst.h`'s `FLIP_TORQUE_X = 260.f`/`FLIP_TORQUE_Y = 224.f` for
  `FLIP_TORQUE_TIME = 0.65f` seconds), but only had the constants, not the
  exact mechanism. Fetched RocketSim's own `Car.cpp`
  (`_UpdateDoubleJumpOrFlip`/`_UpdateAirTorque`, matching
  `FR-058`/`FR-059`/`FR-064`/`FR-065`/`FR-066`/`FR-067`/`FR-068`'s own
  method, continuing FR-059's own investigation) and confirmed it:
  `_UpdateDoubleJumpOrFlip` stores a per-dodge relative torque direction
  at the moment a flip begins; `_UpdateAirTorque` applies it as a
  continuous torque every step, gated by `flipTime < FLIP_TORQUE_TIME` —
  a hard cutoff at exactly `0.65` seconds with no decay or ramp
  beforehand. Also confirmed `FLIP_TORQUE_X`/`FLIP_TORQUE_Y` genuinely
  differ from each other, a second axis-shaped divergence this port's own
  single shared `DODGE_ANGULAR_SPEED` doesn't model. Corrected
  `DODGE_ANGULAR_SPEED`'s own doc comment, the module doc's dodge
  section, and the "commonly-cited constants" paragraph; not adopted as a
  fix, since the resulting spin rate depends on real Rocket League's own
  specific hitbox inertia tensor this port doesn't match, and adopting the
  real shape for real would also mean threading new per-car
  elapsed-flip-time state through `PhysicsWorld` — a substantially larger
  redesign `FR-059`'s own Non-goals already flagged as out of scope. Zero
  production behavior changed, no new tests; all 314 pre-existing tests
  pass unchanged.
- 0.68.0 (2026-09-01): FR-068 added and implemented (real per-axis
  air-control torque ratio) — `FR-031`'s own audit had found real
  air-control torque coefficients exist but didn't adopt them (absolute
  torques calibrated against real Rocket League's own specific
  mass/inertia, the same "false precision" reasoning that kept
  `AIR_CONTROL_TORQUE` a placeholder). Fetched RocketSim's own `Car.cpp`
  (`_UpdateAirTorque`, matching `FR-058`/`FR-059`/`FR-064`/`FR-065`/
  `FR-066`/`FR-067`'s own method) and found the real mechanism —
  `torque = pitch * CAR_AIR_CONTROL_TORQUE.x + yaw *
  CAR_AIR_CONTROL_TORQUE.y + roll * CAR_AIR_CONTROL_TORQUE.z` — is
  structurally identical to this port's own (a direct per-axis torque
  scaled by analog input), unlike steering or handbrake's own architecture
  mismatches. `RLConst.h` confirms `CAR_AIR_CONTROL_TORQUE = Vec(130, 95,
  400)` (pitch-yaw-roll order). Because the mechanism matches, the
  confirmed per-axis *ratio* (unlike the real absolute values) is
  adoptable the same way `FR-058`'s throttle taper and `FR-059`'s dodge
  scale ratios are: added `AIR_CONTROL_YAW_SCALE = 95.0/130.0` and
  `AIR_CONTROL_ROLL_SCALE = 400.0/130.0`, redefined `AIR_CONTROL_TORQUE`
  (value unchanged) as pitch's own magnitude specifically, and wired both
  scales into `apply_driven_forces`'s yaw/roll torque application. A
  genuine behavioral change, not a doc correction: yaw is now measurably
  weaker and roll measurably stronger than pitch for equal analog input.
  Corrected the module doc's air-control paragraph, the "commonly-cited
  constants" paragraph, and `AIR_CONTROL_TORQUE`'s own doc comment. 2 new
  tests (`yaw_air_control_is_scaled_down_from_pitch_by_the_confirmed_real_ratio`,
  `roll_air_control_is_scaled_up_from_pitch_by_the_confirmed_real_ratio`)
  pin the exact expected angular velocity in closed form; all 312
  pre-existing tests pass unchanged (none asserted cross-axis magnitude
  equality), bringing the crate to 314.
- 0.67.0 (2026-09-01): FR-067 added and implemented (real Rocket League has
  no distinct wall-jump mechanic or constant at all — audit finding,
  documentation only) — `drive::WALL_JUMP_HORIZONTAL_SPEED` had no public
  reference at all. Fetched RocketSim's own `Car.cpp` (`_UpdateJump`,
  matching `FR-058`/`FR-059`/`FR-064`/`FR-065`/`FR-066`'s own method) and
  found real Rocket League's `_UpdateJump` applies exactly one impulse,
  `GetUpDir() * mutatorConfig.jumpImmediateForce` (the same real value
  this port's own `JUMP_SPEED` already matches), gated only on
  `isOnGround`, itself defined purely by wheel-contact count with no
  floor-vs-wall distinction; a dedicated search of `RLConst.h` found no
  `WALL_JUMP`-named constant anywhere. Since `FR-065` already confirmed
  real cars ride Bullet's own raycast vehicle system (`btVehicleRL`), a
  car driving on a wall has its own orientation continuously tipped by
  ordinary wheel/suspension contact forces to match that wall, the same
  way a real car tilts to match a ramp — so `GetUpDir()` already points
  along the wall's outward normal by the time a wall jump fires. Real
  Rocket League's "wall jump" is thus the identical single grounded-jump
  impulse, not a distinct horizontal-plus-vertical composite — closing a
  thread `FR-031`'s original audit only briefly noted ("a wall jump
  reusing the plain jump impulse rather than its own faster speed")
  without confirming the exact mechanism. Corrected
  `WALL_JUMP_HORIZONTAL_SPEED`'s own doc comment, the module doc's own
  wall-jump section, and the "commonly-cited constants" paragraph; not
  adopted as a fix, since this port's car has no wheels, raycasting, or
  surface-tracking orientation system at all (the same architecture gap
  `FR-065` found for steering) — applying only `JUMP_SPEED` straight up on
  a wall touch would produce no push-off at all in this port's own model,
  so its own two-component composite substitute remains deliberate and
  necessary, not an unfilled gap. Zero production behavior changed, no new
  tests; all 312 pre-existing tests pass unchanged.
- 0.66.0 (2026-09-01): FR-066 added and implemented (real handbrake
  friction reduction is anisotropic, not a single uniform multiplier —
  audit finding, documentation only) — `drive::HANDBRAKE_FRICTION_MULTIPLIER`
  had no public reference at all. Fetched RocketSim's own `Car.cpp`
  (`_UpdateWheels`, continuing FR-065's own investigation) and found real
  Rocket League's handbrake applies two separate confirmed real curves,
  `HANDBRAKE_LAT_FRICTION_FACTOR_CURVE` (a constant `0.1`) and
  `HANDBRAKE_LONG_FRICTION_FACTOR_CURVE` (`0.5` at a standstill, `0.9`
  at real driving speeds), to lateral and longitudinal tire friction
  independently — not one shared multiplier. This port's own pre-existing
  `HANDBRAKE_FRICTION_MULTIPLIER = 0.1` happens to match the real
  lateral-only factor exactly, a striking coincidence, not a confirmation:
  applied to this port's single isotropic friction scalar, it also wrongly
  crushes longitudinal grip to a tenth, where real Rocket League keeps it
  near `0.9`, understating real forward-momentum retention during a
  drift. Corrected `HANDBRAKE_FRICTION_MULTIPLIER`'s own doc comment and
  the module doc's "Handbrake" and "commonly-cited constants" paragraphs;
  not adopted as a fix, since `solver::friction_directions`' own two
  tangent rows currently share one combined-friction scalar, and giving
  handbrake a genuinely different lateral-vs-longitudinal factor would
  mean threading a second friction coefficient through every one of
  `solver.rs`'s several row-limit call sites plus a way to know which
  body is handbraking — the same architecture-mismatch category
  `FR-063`/`FR-065` already established. Zero production behavior
  changed, no new tests; all 312 pre-existing tests pass unchanged.
- 0.65.0 (2026-09-01): FR-065 added and implemented (real steering is a
  wheeled-vehicle raycast model, not a torque, with an inverted
  speed-vs-turning-ability curve — audit finding, documentation only) —
  `drive::STEER_TORQUE` had no public reference at all. Fetched
  RocketSim's own `Car.cpp` (`_UpdateWheels`, matching
  `FR-058`/`FR-059`/`FR-064`'s own method) and found real Rocket League's
  steering is not a direct yaw-torque model: a wheel's *steer angle* (from
  a confirmed `STEER_ANGLE_FROM_SPEED_CURVE`) feeds Bullet's own raycast
  vehicle system (`btVehicleRL`), whose per-wheel lateral tire friction
  (a further confirmed `LAT_FRICTION_CURVE`) is what actually turns the
  car — a fundamentally different architecture this port's single-rigid-
  box car has no way to represent, the same category `FR-063` already
  established. The confirmed curve's own shape is also strikingly the
  opposite of this port's own `speed_factor`: real turning ability is
  highest at a standstill and decreases with speed, while this port's
  `speed_factor` is zero at a standstill and scales up with speed.
  Corrected `STEER_TORQUE`'s and `MAX_CAR_SPEED`'s own doc comments and
  the `speed_factor` call site's own comment to state this finding
  directly; not adopted as a fix, since the real curve maps speed to a
  wheel angle whose translation to yaw torque depends on tire-slip
  friction this port doesn't model, leaving no principled way to carry
  even the curve's shape onto this port's own direct-torque model. Zero
  production behavior changed, no new tests; all 312 pre-existing tests
  pass unchanged.
- 0.64.0 (2026-09-01): FR-064 added and implemented (real mandatory
  minimum-hold window for a ground jump's variable-height acceleration) —
  `drive::JUMP_HOLD_MAX_DURATION`'s own doc comment had flagged, since
  `RB-PHYSICS-001-FR-031`'s original audit, that real Rocket League scales
  its jump-hold acceleration down during a `JUMP_MIN_TIME` (0.025s)
  mandatory window rather than applying it flat, unmodeled here. Fetched
  RocketSim's own `Car.cpp` (`_UpdateJump`, matching `FR-058`/`FR-059`'s
  own real-implementation-file method) and confirmed the exact mechanism:
  the hold force keeps applying (scaled by `JUMP_PRE_MIN_ACCEL_SCALE =
  0.62f`) for the first `JUMP_MIN_TIME` seconds regardless of whether
  `jump` is still held, not merely a slower ramp — even an instantaneous
  tap gets a small amount of extra height in real Rocket League. Added
  `drive::JUMP_MIN_TIME`/`JUMP_PRE_MIN_ACCEL_SCALE` and reworked
  `apply_driven_forces`'s hold-acceleration check to derive elapsed time
  since the press from the existing `jump_hold_time_remaining` state
  (`JUMP_HOLD_MAX_DURATION - *jump_hold_time_remaining`) rather than
  adding a second field, so no caller (`PhysicsWorld`, any existing test)
  needed to change; 3 new tests pin the mandatory window's own scaled
  acceleration, its immunity to an early release, and its on-schedule
  closure even when jump is never held. All 309 pre-existing tests pass
  unchanged, bringing the crate to 312.
- 0.63.0 (2026-09-01): FR-063 added and implemented (real Rocket League
  uses per-contact-pair-type restitution/friction overrides, not a
  per-body combine — audit finding, documentation only) —
  `RB-PHYSICS-001-FR-043` had left open "which formula (if either)
  actually matches real Rocket League itself" for
  `solver::combine_restitution`/`combine_friction`. Fetched RocketSim's
  own `RLConst.h` (matching `FR-057`/`FR-060`/`FR-061`/`FR-062`'s own
  method) and found the real answer isn't a different formula at all:
  real Rocket League hardcodes a distinct restitution/friction value per
  named contact-pair type (`CARWORLD_COLLISION_FRICTION/RESTITUTION =
  0.3f`/`0.3f`, `CARCAR_COLLISION_FRICTION/RESTITUTION = 0.09f`/`0.1f`,
  `CARBALL_COLLISION_FRICTION/RESTITUTION = 2.0f`/`0.0f`), overriding
  whatever a generic per-body combine would produce. Two findings are
  individually striking: `CARBALL_COLLISION_RESTITUTION = 0.0f` means a
  car hitting the ball has zero restitution-driven bounce in real Rocket
  League regardless of either body's own material (a stark contrast with
  this port's own combine, which since `FR-062` averages the ball's
  confirmed real `0.6` against the car's generic `0.5` to a real `~0.55`
  bounce for exactly that pairing); `CARBALL_COLLISION_FRICTION = 2.0f`
  is a friction coefficient above `1.0`, which no combine of two bodies'
  own sane per-material values could ever produce, confirming this is a
  genuinely different model, not merely an uncalibrated magnitude.
  Corrected `combine_restitution`/`combine_friction`'s own doc comments
  and this spec's stale Open Questions bullet to state this finding
  directly. Not adopted: implementing real per-pair-type overrides, since
  `combine_restitution`/`combine_friction`'s own two-`f32`-in-one-out
  signature has no way to know which kind of pair produced its inputs —
  doing so for real would mean threading body/shape identity into every
  one of `solver.rs`'s several call sites, a substantially larger
  architecture change left for a future, dedicated requirement. Also not
  adopted: setting the car's own generic default restitution/friction to
  any of these values (mirroring `FR-062`'s `RigidBody::ball`) — unlike
  the ball, real Rocket League has no single "the car's own" value here,
  every real number found is contact-pair-specific, so picking one for a
  generic default would be arbitrary. No behavioral change and no new
  tests (documentation-only, matching `RB-PHYSICS-001-FR-044`/`FR-060`'s
  own precedent); all 309 of `rb_physics_bullet`'s pre-existing tests
  pass unchanged.
- 0.62.0 (2026-09-01): FR-062 added and implemented (real ball material
  properties via a new `RigidBody::ball` constructor) —
  `RB-PHYSICS-001-FR-061`'s own Non-goals had explicitly deferred adopting
  `BALL_DRAG` for lack of a dedicated ball-construction API: `sphere`
  gives every caller an identical generic `0.5`/`0.5`/`0.0` placeholder
  with no way to say "this one is a real ball." Fetched RocketSim's own
  `RLConst.h` (matching `FR-057`/`FR-060`/`FR-061`'s own method) and
  confirmed `BALL_RESTITUTION = 0.6f`, `BALL_FRICTION = 0.35f`, and
  `BALL_DRAG = 0.03f` — none a torque/force calibrated against a specific
  mass/inertia, so all three transfer cleanly the same way `FR-061`'s
  speed caps did. Added `body::RigidBody::ball(radius, mass, position)`,
  new additive API alongside the existing `sphere`/`car_box`: identical
  for `radius`/`mass`/`position`, but sets `restitution = 0.6`, `friction
  = 0.35`, `linear_damping = 0.03` instead of the generic defaults;
  `sphere` itself unchanged. Deliberately not adopted: `BALL_MASS_BT =
  CAR_MASS_BT / 6.f`, since this project has no canonical "real" car
  construction site yet to keep a `1:6` ratio against (every `car_box`
  call site today is test-only) — left for a future requirement. 3 new
  tests (`ball_sets_confirmed_real_material_properties`,
  `ball_otherwise_behaves_identically_to_sphere`, and a regression pin
  confirming `sphere`'s own default stayed untouched); all 306
  pre-existing tests pass unchanged; bringing the crate to 309 total (+3
  over `FR-061`'s 306).
- 0.61.0 (2026-09-01): FR-061 added and implemented (hard caps on ball
  linear/angular speed) — the ball had no linear or angular speed cap of
  any kind (`RigidBody.linear_damping`/`angular_damping` both default to
  `0.0`, and nothing else ever bounded its velocity), unlike a car, which
  has had a hard angular-speed ceiling since `FR-057`. Fetched RocketSim's
  own `RLConst.h` and `Ball.cpp` (matching `FR-057`/`FR-060`'s own method)
  and found two confirmed real hard caps — `BALL_MAX_SPEED = 6000.f` and
  `BALL_MAX_ANG_SPEED = 6.f` — enforced via a hard clamp
  (`if (vel.length2() > max*max) vel = vel.normalized() * max`) inside
  `_FinishPhysicsTick()`, after collision resolution, at the end of the
  physics tick. Both are pure velocity caps, not torque/force constants
  calibrated against a specific mass/inertia, so they transfer cleanly
  regardless of this port's own ball not being calibrated to real Rocket
  League's — the same category `FR-057` established. Added
  `world::BALL_MAX_SPEED`/`BALL_MAX_ANG_SPEED` and a new
  `world::clamp_ball_velocity`, generalizing `drive::clamp_angular_speed`'s
  own shape to both linear and angular speed (placed in `world.rs` rather
  than `drive.rs` since the ball has no drive-input-gated mechanic of its
  own), wired into `PhysicsWorld::step` right after this step's contact
  resolution (including any net) and before sleep evaluation or transform
  integration — matching real RocketSim's own placement more precisely
  than `drive::clamp_angular_speed`'s own placement for the car (which
  runs *before* that same step's own contact resolution, an earlier point
  in this port's own pipeline). Deliberately not adopted: `BALL_DRAG =
  0.03f`, since real RocketSim sets it once at ball construction as a
  per-match mutator-config default (`constructionInfo.m_linearDamping =
  mutatorConfig.ballDrag`), not a hardcoded system invariant like the two
  speed caps — this port's own `RigidBody::sphere` constructor takes no
  opinion on a "real" ball's own damping default, and changing that is a
  separate, deliberate design decision left for a future requirement. 4
  new tests (2 unit tests of `clamp_ball_velocity` directly, one each for
  linear and angular; 1 integration test through `PhysicsWorld::step`
  confirming a ball launched far past `BALL_MAX_SPEED` never exceeds it
  after a step; 1 no-op-below-both-caps test); no existing test ever set
  the ball's speed or angular speed anywhere near either cap (highest
  directly-assigned ball speed in the crate's own tests: `3000.0` uu/s;
  no test assigns ball angular velocity directly at all), an explicit
  zero-regression-risk property confirmed by inspection before
  implementation and by all 302 pre-existing tests passing unchanged
  afterward; bringing the crate to 306 total (+4 over `FR-060`'s 302).
- 0.60.0 (2026-09-01): FR-060 added and implemented (landing
  auto-orientation vs. real auto-flip/auto-roll — audit finding,
  documentation only) — `RB-PHYSICS-001-FR-057`'s own Non-goals had
  flagged RocketSim's auto-flip constants
  (`CAR_AUTOFLIP_IMPULSE/TORQUE/TIME/NORMZ_THRESH/ROLL_THRESH`) as a
  possible reference for `drive::LANDING_AUTO_UPRIGHT_TORQUE`, but left
  open whether real auto-flip's conditional/threshold shape could map onto
  this port's continuous-torque assist "without further investigation."
  Fetched and read RocketSim's real `Car.cpp` (the same technique
  `FR-058`/`FR-059` used) and found the answer is no: real Rocket League
  has no mechanic matching "continuously nudge an airborne car upright
  with no player input" at all. It has two distinct, real, grounded,
  input-gated systems instead — auto-flip (a turtle-recovery flip firing
  only on a jump press while grounded on a roughly-horizontal surface
  (`CAR_AUTOFLIP_NORMZ_THRESH`) with roll already past a threshold
  (`CAR_AUTOFLIP_ROLL_THRESH`), timed over `CAR_AUTOFLIP_TIME`) and
  auto-roll (a continuous ground-alignment torque active only while
  throttle is held with wheel contact) — neither airborne nor input-free,
  the opposite shape from this port's own placeholder. Corrected the
  `drive` module's doc comments, this spec's stale Open Questions bullet,
  and `FR-057`'s own Non-goals bullet to state this finding directly
  rather than leave it an open "may not map... without further
  investigation" question. No behavioral change and no new tests
  (documentation-only, matching `RB-PHYSICS-001-FR-044`'s own precedent);
  all 302 of `rb_physics_bullet`'s pre-existing tests pass unchanged.
- 0.59.0 (2026-09-01): FR-059 added and implemented (real
  forward-speed-dependent dodge impulse scaling) — `RB-PHYSICS-001-FR-031`'s
  own audit had already found real Rocket League's dodge impulse has
  "direction/speed-dependent scaling" but couldn't adopt it without the
  actual formula, only `RLConst.h`'s bare constants. Fetched RocketSim's
  own `Car.cpp` (`_UpdateDoubleJumpOrFlip`, the same file/technique
  `FR-058` used) and found the real mechanism: a dodge's base impulse
  scales per-axis by `((maxSpeedScale - 1) * forwardSpeedRatio) + 1`,
  where `maxSpeedScale` is `1.f` for a forward dodge (no change, ever),
  `2.5f` for a backward dodge (opposing current velocity, per
  `shouldDodgeBackwards`), or `1.9f` for any side dodge. Adopted the
  confirmed real *ratios* (`2.5`, `1.9`) via two new functions
  (`drive::dodge_speed_scale`/`dodge_pitch_is_backward`, the second
  re-derived in this port's own sign convention), wired into both the
  ground-dodge and wall-jump-dodge blocks — but deliberately not the real
  base magnitude (`FLIP_INITIAL_VEL_SCALE = 500.f`, vs this port's own
  still-uncalibrated `DODGE_SPEED = 1400.0`, unchanged), since the real
  forward-dodge scale of exactly `1.0` means `DODGE_SPEED` already stands
  in for that case — the same "shape confirmed, magnitude not" split
  `FR-058` established for `THROTTLE_ACCELERATION`. Also explicitly not
  adopted: RocketSim's own diagonal-dodge direction normalization (a
  separate, already-documented simplification) and its
  continuous-torque-over-`FLIP_TORQUE_TIME` spin model (a substantially
  larger redesign, left for a future requirement). 5 new `drive.rs` tests
  (two unit tests of the new functions, three integration tests
  confirming exact scaled magnitudes at `MAX_CAR_SPEED`); every existing
  dodge test dodges from a standing start, where the new scale evaluates
  to `1.0` regardless of direction — an explicit zero-regression-risk
  property confirmed by inspection before implementation, then by the
  full suite passing unchanged; bringing the crate to 302 total (+5 over
  `FR-058`'s 297).
- 0.58.0 (2026-09-01): FR-058 added and implemented (real speed-dependent
  throttle taper) — `THROTTLE_ACCELERATION`'s own doc comment had named
  this exact gap since it was introduced: applying full flat acceleration
  right up to a hard cutoff at `UNBOOSTED_MAX_CAR_SPEED`, not a genuine
  taper. Fetched RocketSim's own `Car.cpp` (not just `RLConst.h`'s
  constants) to find exactly how its own `THROTTLE_TORQUE_AMOUNT` is
  used, surfacing the real mechanism: drive force is scaled by
  `DRIVE_SPEED_TORQUE_FACTOR_CURVE`, a confirmed 3-point piecewise-linear
  curve (`{0, 1.0}, {1400, 0.1}, {1410, 0.0}`), not applied flat.
  `THROTTLE_TORQUE_AMOUNT` itself is expressed in Bullet-internal units
  that don't transfer to this port's own car body the same clean way
  (repeating `FR-031`'s/`FR-057`'s own "false precision" finding), but
  the curve's *shape* is a pure, unitless ratio that transfers regardless
  — the same reasoning `FR-057` used for `MAX_CAR_ANGULAR_SPEED`. Added
  `drive::DRIVE_SPEED_TAPER_BREAKPOINTS`/`drive_speed_taper`, replaced the
  hard cutoff with the real taper (evaluated against this port's own
  pre-existing signed, direction-aware speed quantity — not RocketSim's
  own direction-agnostic `abs(forward speed)`, a separate behavioral
  question left out of scope), and corrected doc comments describing this
  as unmodeled. `THROTTLE_ACCELERATION`'s own peak magnitude remains an
  uncalibrated placeholder — only the curve's shape is now confirmed and
  modeled. 2 new `drive.rs` tests (a direct unit test of the interpolator,
  and a regression test confirming a car at 1400 uu/s now gains only ~10%
  of a full-strength step's velocity delta); bringing the crate to 297
  total (+2 over `FR-057`'s 295).
- 0.57.0 (2026-09-01): FR-057 added and implemented (hard cap on car
  angular speed) — nothing in this port previously bounded how fast
  sustained air control torque (or a dodge's own kick, or the
  landing-orientation assist) could spin a car; holding full pitch/yaw/roll
  indefinitely spun a car arbitrarily fast, unlike real Rocket League.
  Fetched RocketSim's own `RLConst.h` a second time (`FR-056` proved the
  first fetch's technique — targeting this port's own "no public
  reference at all" constants — could surface genuine findings), this
  time targeting `STEER_TORQUE`, `HANDBRAKE_FRICTION_MULTIPLIER`,
  `AIR_CONTROL_TORQUE`, `WALL_JUMP_HORIZONTAL_SPEED`, `DODGE_SPEED`,
  `DODGE_ANGULAR_SPEED`, `JUMP_HOLD_MAX_DURATION`,
  `JUMP_HOLD_ACCELERATION`, and `LANDING_AUTO_UPRIGHT_TORQUE`. Found
  `CAR_MAX_ANG_SPEED = 5.5f` (rad/s), a hard "can never exceed" ceiling
  this port had no equivalent for. Several other real constants the same
  fetch surfaced (dodge per-direction impulse scaling, auto-flip
  thresholds, a ramping powerslide model, a steering-torque mapping, and
  RocketSim's own per-axis `CAR_AIR_CONTROL_TORQUE`) were considered and
  explicitly not adopted — the torque-based ones repeat
  `RB-PHYSICS-001-FR-031`'s own "false precision" finding (a torque
  constant is calibrated against RocketSim's own car mass/inertia tensor,
  which this port's placeholder body doesn't match), while
  `CAR_MAX_ANG_SPEED` bounds the *result* (a rad/s quantity) rather than
  the torque producing it, so it transfers cleanly regardless. Added
  `drive::MAX_CAR_ANGULAR_SPEED` and `drive::clamp_angular_speed` (a
  genuine clamp, unlike `MAX_CAR_SPEED`'s force-gating), wired in right
  after `integrate::integrate_velocities` in both `world.rs`'s production
  path and `drive.rs`'s own test helper. Also noted, as a coincidence, that
  the pre-existing uncalibrated `DODGE_ANGULAR_SPEED` placeholder is
  numerically equal to this same 5.5 value — flagged in both constants'
  own doc comments, not treated as a second confirmation. 3 new `drive.rs`
  tests (two unit tests for the clamp function, one proving sustained full
  roll input caps out rather than growing unbounded); bringing the crate
  to 295 total (+3 over `FR-056`'s 292).
- 0.56.0 (2026-09-01): FR-056 added and implemented (boost acceleration
  ground/air split) — `drive::BOOST_ACCELERATION` was a single flat
  constant applied identically whether a car was grounded or airborne,
  and this port's own doc comments (including `RB-PHYSICS-001-FR-008`'s
  own Requirements entry) explicitly claimed boost "works identically
  airborne". Fetched RocketSim's own `RLConst.h` directly and found that
  claim wrong: the reference defines `BOOST_ACCEL_GROUND = 2975.f / 3.f`
  (≈991.667, exactly matching this port's own existing value) and a
  distinctly higher `BOOST_ACCEL_AIR = 3175.f / 3.f` (≈1058.333, about
  6.5% more) — a genuine ground/air split this port didn't model, so
  every airborne boost this crate ever applied understated real boost
  strength. Split into `BOOST_ACCELERATION_GROUND`/`BOOST_ACCELERATION_AIR`,
  wired `apply_driven_forces`'s existing `on_ground` parameter to select
  between them (no new gating logic — boost already applied in both
  cases, only the magnitude changed), and corrected every doc comment
  that claimed the two were identical. Also confirmed, as a byproduct of
  the same fetch, that `BOOST_CONSUMPTION_RATE`/`MAX_BOOST` already match
  RocketSim's own `BOOST_USED_PER_SECOND = BOOST_MAX / 3` — no change
  needed there. 1 new `drive.rs` test confirming the exact ratio between
  grounded and airborne boost acceleration matches the reference's own
  ratio; bringing the crate to 292 total (+1 over `FR-055`'s 291).
- 0.55.0 (2026-09-01): FR-055 added and implemented
  (`GOAL_HALF_WIDTH`/`GOAL_HEIGHT` reference confirmation, stale doc
  correction) — `arena::GOAL_HALF_WIDTH`/`GOAL_HEIGHT` carried a
  "commonly-cited, not independently confirmed" caveat since FR-024
  introduced them. Fetched the current RLBot wiki's "Useful Game Values"
  page directly (the same page FR-036's own research already used for
  `GOAL_DEPTH`) and confirmed both values exact against its own cited
  "Goal center-to-post: 892.755"/"Goal height: z=642.775" numbers — no
  value change, a sourcing-status upgrade only. Also found and fixed a
  stale "Open questions" passage that still described `GOAL_DEPTH` as an
  unconfirmed "uncalibrated invention", directly contradicting FR-036's
  own already-shipped Requirements entry and this spec's own Non-goals
  section (never updated when FR-036 shipped) — rewritten to state all
  three goal-geometry constants are now confirmed, leaving only
  `arena::NET_DEPTH` open in that vicinity. No new tests (pure
  constant-sourcing/doc correction, no behavioral change, matching
  FR-031/FR-036's own precedent); `cargo test --workspace` re-run clean
  at 291 total (unchanged from FR-054).
- 0.54.0 (2026-09-01): FR-054 added and implemented (goal-wall/
  bounded-wall corner-testing overlap investigation) — closes the one
  question `RB-PHYSICS-001-FR-028`'s own doc comment left open: whether
  `collision::box_vs_goal_wall`'s per-corner window test could
  under-detect a car's face resting flush against the window's own edge,
  every corner just clear of it while the face's middle already
  overlapped it, the same category of concern `RB-PHYSICS-001-FR-032`
  investigated for a curved fillet but explicitly didn't cover for a flat
  rectangle. Resolved via a convex-hull argument: a box's touching face is
  the convex hull of whichever corners penetrate the plane, so "every
  corner outside the (convex) window" is exactly equivalent to "the face
  doesn't fully fit through it," the correct block condition — no bug,
  matching FR-032's own "further investigation found the suspected gap
  doesn't exist" precedent, via a distinct argument (convex containment,
  not a convex scalar maximum). Investigating `box_vs_bounded_wall`
  (`RB-PHYSICS-001-FR-029`) alongside it, since it shares the identical
  corner-testing technique with the opposite gate, found the mirror image
  is a genuine gap this time: a face larger than a bound and centered on
  it has no corner touching solid material even though the bound's own
  rectangle sits entirely within the face's interior, so it reports zero
  contacts despite genuinely resting on real material — confirmed
  unreachable given this project's own car (`60x30x18` half-extents) and
  ball (`93.15` radius) against the standard arena's own two
  `StaticBoundedWall`s (hundreds of units on their shortest side), so
  documented as an explicit Non-goals item rather than fixed with a
  heavier 2D convex-polygon overlap test no constructible scene needs. 2
  new `collision.rs` tests, bringing the crate to 291 total (+2 over
  `FR-053`'s 289).
- 0.53.0 (2026-09-01): FR-053 added and implemented (`combine_friction`
  defensive clamp) — `RB-PHYSICS-001-FR-043` fetched and read real
  Bullet's own `btManifoldResult::calculateCombinedFriction`/
  `calculateCombinedRestitution` source to correct this spec's wrong
  claim that the reference's default combine mode is `btMax`, but never
  separately examined one more detail in that same source: real Bullet's
  own `calculateCombinedFriction` additionally clamps its product result
  to `[-10.0, 10.0]` (`calculateCombinedRestitution` has no such clamp).
  This requirement re-fetched and re-read `btManifoldResult.cpp` directly
  to confirm the clamp's exact mechanics, found it currently inert for
  every friction coefficient this crate itself ever sets (all positive
  placeholders in `0.1..=0.9`), and adopted it anyway for reference
  conformance — every `RigidBody`/`StaticPlane`/`StaticQuarterPipe`/
  `StaticCornerFillet`/`StaticGoalWall`/`StaticBoundedWall`'s own
  `friction` field is a public, unvalidated `f32`, so the clamp costs
  nothing and closes a genuinely uninvestigated gap. `combine_friction`
  now clamps its average result to `[-10.0, 10.0]`, keeping the average
  formula `FR-043` already decided to keep; `combine_restitution` stays
  unclamped, matching the reference's own choice. 1 new test
  (`combine_friction_clamps_to_the_same_bound_real_bullet_uses`), bringing
  `rb_physics_bullet` to 289 tests (+1 over FR-052's 288).
- 0.52.0 (2026-09-01): FR-052 added and implemented (static-vs-dynamic
  combined-solve ordering investigation) — `PhysicsWorld::step` resolved a
  body's now-combined static contacts (FR-051) and its combined dynamic
  manifolds (FR-030) as two separate solves, one fully resolved and
  applied before the other's own setup for that same body ever read the
  result — the same independent-pairwise gap FR-030/FR-050/FR-051 already
  proved under-converges, just at the boundary between the two existing
  combined solves instead of inside either one. A dedicated single-shot
  test reused FR-051's own symmetric two-wall corner setup, replacing one
  wall with a very-heavy dynamic body (`mass = 1e9`, geometrically
  identical contact) routed through the dynamic-manifold code path
  instead of the static one, and confirmed resolving the static wall then
  the dynamic body (`step`'s own pre-fix order) is genuinely
  order-dependent (mirror-image results depending on which channel
  resolves first), not merely slow to converge. A new
  `solver::resolve_manifolds` folds a step's static and dynamic manifolds
  into one shared solve, sharing one `DeltaVelocity`/push-delta
  accumulator per body index across both channels for the whole
  `SOLVER_ITERATIONS` loop; `RB-PHYSICS-001-FR-041`'s own `1 / k`
  relaxation keeps counting `k` purely from dynamic manifolds (extending
  it to a body's static rows was tried and found to regress FR-051's own
  two-static-wall test, not adopted). `PhysicsWorld::step` was rewired to
  use it: `resolve_static_contacts` became `static_contact_manifolds`
  (now returning gathered manifolds instead of resolving them), and
  `step` makes one `solver::resolve_manifolds` call instead of two
  separate ones. A `PhysicsWorld::step`-level test (a ball fired into a
  real wall-and-heavy-car corner) confirmed the fix at the public API,
  verified to fail under the old two-call sequence first. 2 new tests,
  bringing `rb_physics_bullet` to 288 tests (+2 over FR-051's 286).
- 0.51.0 (2026-09-01): FR-051 added and implemented (static multi-surface
  contact combined-solve investigation) — `PhysicsWorld::step` resolved a
  body's contact against each static shape type (ground, then every wall,
  curve, corner fillet, goal wall, bounded wall) via one independent
  `solver::resolve_contacts` call per shape, the same independent-pairwise
  gap FR-030/FR-050 already proved under-converges for a shared body
  touched by 2+ others in the same step. A dedicated single-shot test
  confirmed a ball wedged into a symmetric two-wall corner is genuinely
  order-dependent (mirror-image results depending on which wall resolves
  first), not merely slow to converge. A new `solver::resolve_static_manifolds`
  generalizes `resolve_contacts` to combine every static-shape manifold a
  body touches into one shared solve; `PhysicsWorld::step` was rewired to
  use it via a new `resolve_static_contacts` (bundling the six static-shape
  slices into a `StaticScene` to stay under clippy's argument-count limit),
  replacing the old five-function-per-body call sequence
  (`resolve_plane_contact`/`resolve_curve_contact`/
  `resolve_corner_fillet_contact`/`resolve_goal_wall_contact`/
  `resolve_bounded_wall_contact`, all removed). A `PhysicsWorld::step`-level
  test (a ball fired into a real two-wall corner) confirmed the fix at the
  public API, verified to fail under the old sequential loop first. 2 new
  tests, bringing `rb_physics_bullet` to 286 tests (+2 over FR-050's 284).
- 0.50.0 (2026-09-01): FR-050 added and implemented (net-point contact
  combined-solve investigation) — `net::NetMesh::step` used to resolve every
  body-vs-net-point contact independently and sequentially via
  `solver::resolve_contacts_between`, one pair at a time, waving off
  `RB-PHYSICS-001-FR-030`'s own documented independent-pairwise gap as
  irrelevant because a net point's mass is "tiny enough" — an untested
  claim found false (`NET_POINT_MASS = 0.5` is half a typical ball's own
  mass). A dedicated single-shot test confirmed the old sequential loop is
  genuinely order-dependent (not merely slow to converge) for a perfectly
  symmetric double-point impact; a `NetMesh::step`-level test measured the
  real residual at ~0.25 units/s out of a 2000 units/s impact. Adopted
  `solver::resolve_dynamic_manifolds`'s combined solve for every
  body-vs-point contact within a sub-step, reducing that residual roughly
  15-fold to ~0.016 units/s; warm-starting deliberately left out of scope.
  2 new tests, bringing `rb_physics_bullet` to 284 tests (+2 over FR-049's
  282).
- 0.49.0 (2026-09-01): FR-049 added and implemented (velocity-aligned
  friction direction selection) — closes the genuine, significant
  divergence `RB-PHYSICS-001-FR-048` found and explicitly left open: a new
  `friction_directions` helper in `solver.rs` now aligns friction
  direction 1 with the tangential component of the current relative
  sliding velocity, matching real Bullet's actual default, with direction
  2 completing a right-handed basis via `dir1.cross(normal)`. Falls back
  to `plane_space`'s fixed basis both for negligible tangential velocity
  (matching real Bullet's own `SIMD_EPSILON` threshold) and for a
  second, genuinely new case found while implementing this: near-head-on
  collisions where catastrophic floating-point cancellation can leave a
  degenerate tangential residual that real Bullet's own unguarded
  `normalize()` would silently mishandle but this crate's own
  `Option`-returning `Vec3::normalize()` instead falls back gracefully
  from. Wired into both `setup_rows` and `setup_two_body_rows`. Confirmed
  via a dedicated isotropic-friction regression test, verified to fail
  under the old fixed-basis behavior. 3 new tests, bringing
  `rb_physics_bullet` to 282 tests (+3 over FR-048's 279).
- 0.48.0 (2026-08-31): FR-048 added and investigated (`solver.rs`
  constraint-row setup/resolve reference validation) — fetched and read
  Bullet's real `btSequentialImpulseConstraintSolver.cpp`/`.h`,
  `btContactSolverInfo.h`, and `btVector3.h` to check every Bullet-reference
  claim `restitution_curve`, `plane_space`, `setup_rows`, and `resolve_row`
  make. Confirmed `plane_space` byte-for-byte exact against real
  `btPlaneSpace1`; `restitution_curve` behaviorally exact (its `.max(0.0)`
  folds in a clamp real Bullet applies at its one call site instead, not a
  divergence); `setup_rows`'s normal/friction row formulas exact against
  real `setupContactConstraint`/`setupFrictionConstraint` (correcting a
  stale citation to an unrelated, differently-named function); `resolve_row`'s
  single unified two-bound resolver behaviorally equivalent to Bullet's own
  two separate resolvers, given the normal row's effectively-infinite upper
  limit; and all 6 of `btContactSolverInfo`'s cited default constants exact.
  Found one genuine, significant divergence, not adopted: this port always
  derives both friction directions from a fixed, velocity-independent
  `plane_space` basis, while real Bullet's actual default aligns friction
  direction 1 with the tangential component of the current relative sliding
  velocity, falling back to `btPlaneSpace1` only when that velocity is
  negligible — a fixed two-axis friction limit can over/under-estimate the
  true circular friction cone by up to `sqrt(2)` relative to the real slide
  direction, so this is flagged as open follow-up work (a dedicated future
  requirement, the same scoping already used for `RB-PHYSICS-001-FR-030`/
  `FR-034`/`FR-035`/`FR-037`) rather than folded into this pass. 1 new test,
  bringing `rb_physics_bullet` to 279 tests (+1 over FR-047's 278);
  investigated, doc-only changes to production code plus the 1 new test —
  no other runtime behavior changed.
- 0.47.0 (2026-08-31): FR-047 added and investigated (`collision.rs`
  remaining closed-form shape pairings reference validation) — fetched and
  read Bullet's real `btConvexPlaneCollisionAlgorithm.cpp`/`.h`,
  `btSphereBoxCollisionAlgorithm.cpp`, `btSphereSphereCollisionAlgorithm.cpp`,
  and `btManifoldPoint.h` to check every Bullet-reference claim
  `sphere_vs_plane`, `box_vs_plane`, `sphere_vs_box`, and `sphere_vs_sphere`
  make (`box_vs_box` was already checked this way,
  `RB-PHYSICS-001-FR-042`). Confirmed `sphere_vs_plane` and
  `sphere_vs_sphere` exact, and `sphere_vs_box`'s deep-penetration face
  selection confirmed to reproduce Bullet's own exact
  `+x, -x, +y, -y, +z, -z` tie-break check order, not just a
  mathematically-equivalent alternative — pinned by a new test using a
  deliberately non-symmetric tied case. Found one genuine, deliberate
  divergence: real `btConvexPlaneCollisionAlgorithm` generates only one
  contact point per frame via a single GJK support query (its own
  multi-point "perturbation" path is off by Bullet's own real default),
  relying on several frames of persistent-manifold accumulation to reach a
  resting box's full 4-corner manifold, where `box_vs_plane` computes all
  4 corners exactly in one pass — not adopted, confirmed a favorable
  divergence in the same spirit as `box_vs_box`'s own FR-042 finding. 1
  new test, bringing `rb_physics_bullet` to 278 tests (+1 over FR-046's
  277); investigated, doc-only changes to production code plus the 1 new
  test — no other runtime behavior changed.
- 0.46.0 (2026-08-31): FR-046 added and investigated (`body.rs`/`mat3.rs`
  reference validation) — fetched and read Bullet's real
  `btSphereShape.cpp`, `btBoxShape.cpp`, `btRigidBody.cpp`/`.h`, and
  `btMatrix3x3.h` to check every Bullet-reference claim
  `body.rs`'s `Shape::local_inertia`/`RigidBody::update_inertia_tensor`
  and `mat3.rs`'s `Mat3::scaled_columns`/`Mat3::from_quat` make. Confirmed
  the sphere/box local-inertia formulas, `update_inertia_tensor`'s
  `basis.scaled(invInertiaLocal) * basis.transpose()`, and
  `Mat3::scaled_columns`'s per-column scaling all byte-for-byte accurate.
  Found one genuine difference: `Mat3::from_quat` hardcodes an `s = 2`
  factor assuming an exactly unit-length input quaternion, while the
  reference's own `btMatrix3x3::setRotation` computes `s = 2 /
  q.length2()` to self-correct for a non-unit-length input — not adopted,
  since this function's only production call site
  (`RigidBody::update_inertia_tensor`) always receives an
  already-renormalized orientation (see FR-045's own
  `integrate_transform` finding), making the reference's own
  self-correction unreachable defensive theater here. Added 1 new
  `mat3.rs` test pinning this exact distinction
  (`from_quat_does_not_self_correct_a_non_unit_length_quaternion`). All
  276 of `rb_physics_bullet`'s pre-existing tests (as of `FR-045`) pass
  unchanged; 277 total (+1 over `FR-045`'s 276).
- 0.45.0 (2026-08-31): FR-045 added and investigated (`integrate.rs`
  reference validation) — fetched and read Bullet's real
  `btRigidBody.cpp`/`.h`, `btTransformUtil.h`, `btQuaternion.h`, and
  `btScalar.h` to check every Bullet-reference claim `integrate.rs`'s own
  doc comments make. Confirmed `apply_damping`'s "Bullet's default"
  claim (`BT_USE_OLD_DAMPING_METHOD` is never `#define`d anywhere in the
  reference) and its exact formula; confirmed `integrate_velocities`'s
  `MAX_ANGVEL` (`SIMD_HALF_PI`) and clamp formula byte-for-byte; confirmed
  `integrate_transform`'s `ANGULAR_MOTION_THRESHOLD`, small-angle Taylor
  coefficient (`1 / 48`), and sinc-based rotation-axis formula
  byte-for-byte. Found one minor numeric difference (this port's
  degenerate-quaternion guard uses `1e-12`, the reference's own
  `SIMD_EPSILON` is `FLT_EPSILON` — about `1.19e-7` for `f32`, ~5 orders
  of magnitude larger) — not adopted, both are far below any physically
  realistic quaternion magnitude and behaviorally indistinguishable for
  every reachable scenario. Found one more significant thing: this
  function's own check-then-normalize fallback isn't defensive theater —
  it's necessary to match Bullet's real fallback choice (preserve the
  prior orientation on a degenerate result, never reset to identity),
  which an unconditional `Quat::normalize` call would have silently gotten
  wrong (that function's own generic guard substitutes `IDENTITY`
  instead). Added 1 new `integrate.rs` test pinning this exact distinction
  (`integrate_transform_preserves_a_degenerate_orientation_instead_of_snapping_to_identity`).
  All 275 of `rb_physics_bullet`'s pre-existing tests (as of `FR-044`)
  pass unchanged; 276 total (+1 over `FR-044`'s 275).
- 0.44.0 (2026-08-31): FR-044 added and investigated (stale Non-goals
  correction) — this spec's own top-level "Non-goals (this increment)"
  section still carried a "Split impulse. This port always takes Bullet's
  non-split contact-resolution branch" bullet, contradicted by
  `RB-PHYSICS-001-FR-034`'s own already-shipped implementation (its own
  Requirements entry, the version 0.34.0 Change History entry, and
  `rb_physics_bullet::solver`'s own module doc comment all already
  correctly describe split impulse as implemented — only this one
  Non-goals bullet had never been updated to match). Confirmed the
  implementation is genuinely present by locating `solver::
  resolve_push_row`/`resolve_two_body_push_row`/`apply_push_delta`
  directly in `solver.rs`, and confirmed via a repo-wide `grep` that this
  was the only stale occurrence anywhere in code or docs. Corrected the
  bullet to a strikethrough-and-close note, matching the same convention
  this section already uses for its own two other resolved Non-goals items
  (the wall-jump-corner disambiguation closed via FR-039; the
  curved-geometry Non-goal closed progressively via FR-026 through
  FR-033). Zero production code changed. No new tests (documentation-only,
  no value or behavior changed, the same precedent FR-032/FR-040/FR-042
  established for a documentation-only finding being real, valuable work).
  All 275 pre-existing tests pass unchanged (total unchanged from FR-043).
- 0.43.0 (2026-08-31): FR-043 added and investigated (restitution/friction
  combine-mode reference validation) — this spec's own "Restitution/
  friction combine mode" Open Question claimed, without ever having
  checked, that Bullet's actual default combine mode is `btMax` for both.
  Fetched and read `btManifoldResult.h`/`btManifoldResult.cpp` in full
  (matching FR-036/FR-042's own method) and found that claim wrong: the
  real default for both `calculateCombinedRestitution` and
  `calculateCombinedFriction` is an unclamped **product** (`a * b`;
  friction additionally clamps to `[-10, 10]`), with no `max` mode, no
  geometric mean, and no per-pair override anywhere in the reference short
  of a custom `gContactAddedCallback`. This port's own `solver::
  combine_restitution`/`combine_friction` already use average
  (`(a + b) * 0.5`), previously justified by the now-corrected wrong claim;
  re-examined against the real default and kept anyway, now for a genuine
  reason — average preserves the identity `combine(a, a) == a`
  (`0.5` and `0.5` average to `0.5`), which the reference's own product
  does not (`0.5 * 0.5 == 0.25`), and most bodies in this port currently
  share the same uncalibrated placeholder `0.5` for both coefficients (see
  `body.rs`'s `Default` impls), so the reference's real default would
  silently combine the overwhelming majority of this port's own contacts
  to `0.25` — a value nobody chose. Whether either formula matches real
  Rocket League itself is unaffected by this correction and remains
  genuinely open, needing `RB-VERIFY-001`/`RB-VERIFY-002` data — only which
  of the two known quantities (this port's choice, and Bullet's real
  default) was being compared got corrected. Updated the wrong claim
  everywhere it appeared: this spec's Open Questions, `solver.rs`'s module
  doc comment, and `body.rs`'s field doc comment. 2 new `solver.rs` tests
  (`combine_restitution_preserves_a_uniform_coefficients_identity`,
  `combine_friction_preserves_a_uniform_coefficients_identity`) pin
  `combine_restitution`/`combine_friction`'s own behavior directly,
  independent of any full contact-resolution scenario, asserting both the
  identity property and that it differs from the reference's own product.
  All 273 of `rb_physics_bullet`'s pre-existing tests (as of FR-042) pass
  unchanged; 275 total (+2 over FR-042's 273).
- 0.42.0 (2026-08-31): FR-042 added and investigated (box-vs-box reference
  validation) — fetched and read Bullet's own `btBoxBoxDetector::dBoxBox`
  reference source directly to validate two "reasonable, tested choices,
  never validated against the reference" this spec's own Open Questions
  flagged, plus one further heuristic found during that reading. (1)
  Edge-edge contact point: the reference uses `dLineClosestApproach`
  (unclamped infinite-line closest approach, confirmed no bounds check on
  `alpha`/`beta` in the fetched source), while this port's own
  `closest_points_on_segments` implements Ericson's proper finite-segment
  closest-point construction — strictly more rigorous than the reference,
  confirmed rather than assumed; no change needed. (2) Face-clipping
  degenerate ("zero points") fallback: the reference contains the same
  undocumented "this should never happen" judgment call (twice, with zero
  geometric justification given), confirming this port's own framing
  wasn't a weaker position — but the reference's own fallback drops the
  collision entirely (`return 0`) while this port synthesizes a
  clamped-center contact instead, a deliberate, favorable divergence (SAT
  already confirmed real overlap by that point, so dropping it risks
  tunneling); kept as-is. (3) Edge-edge tangent sign-selection heuristic:
  this port picks which of a box's 4 candidate parallel edges is nearest
  via the raw center-to-center vector, while the reference uses the
  actual resolved collision normal instead — a candidate fix swapping to
  the normal was built and empirically tested against a brute-force
  ground truth across 50,000 randomized configurations, found genuinely
  mixed (the current heuristic wins for large/arbitrary penetration
  depths, ~11.6% vs. ~8.7% optimal-match rate; the candidate wins for
  realistic near-first-contact depths, ~93% vs. ~77%; neither is reliably
  optimal, both have large outliers), so not adopted — kept as-is,
  documented as a still-open item needing a non-heuristic algorithm or
  real recorded contact data to responsibly improve. No new tests
  (documentation-only, no value or behavior changed, the same precedent
  FR-032/FR-040 established for a rigorously investigated negative result
  being real work); the temporary probe test built to run the brute-force
  comparison was not shipped, the same "confirmed during test design, not
  shipped" precedent FR-030's own 300-iteration manual check established.
  All 273 pre-existing tests pass unchanged (total unchanged from FR-041).
- 0.41.0 (2026-08-31): FR-041 added and implemented (sandwiched-solve
  convergence) — investigated whether anything short of real recorded data
  could narrow FR-030's own documented extreme-mass-ratio "sandwiched"
  under-convergence gap at this crate's fixed `SOLVER_ITERATIONS = 10`. A
  naive global SOR-style relaxation factor was tried first and rejected:
  factors above 1.0 made FR-030's own symmetric-pinch scenario measurably
  *diverge* (worse than the pre-FR-030 independent-pairwise approach),
  while factors below 1.0 monotonically improved it, matching standard
  PGS/SOR theory for a tightly-coupled multi-constraint body.
  `solver::resolve_dynamic_manifolds` now scales each manifold's
  velocity-row impulse by a parameter-free `1 / k` instead, where `k` is
  the largest number of manifolds either of that manifold's two bodies
  takes part in this step — the same "fair share" weighting position-
  based-dynamics solvers use for a point mass under several simultaneous
  constraints. Mathematically dominant rather than a tuned magic number
  (it can only reduce, never increase, a shared body's per-iteration
  overshoot), so unlike raising `SOLVER_ITERATIONS` itself it needed no
  real recorded data to justify adopting. Narrows FR-030's own
  symmetric-pinch result from ~89.5 to ~32 units/s (independent-pairwise
  stays ~98.9) at zero added iteration cost; a body touched by only one
  other body this step (`k == 1`, the overwhelming majority of contacts)
  is a mathematical no-op, confirmed by a dedicated bit-for-bit-equivalence
  test against `resolve_contacts_between`. Does not achieve full
  convergence within one call's fixed `SOLVER_ITERATIONS` — the gap is
  narrowed, not closed; real recorded multi-car contact data would still
  be needed to know whether the residual error matters for fidelity in
  practice, or whether raising `SOLVER_ITERATIONS` itself (a real added
  cost, unlike this fix) is worth it before such data exists. 2 new
  `solver.rs` tests; all pre-existing tests pass unchanged. Bringing the
  crate to 273 total (+2 over FR-040's 271).
- 0.40.0 (2026-08-31): FR-040 added and investigated (fillet-radius
  calibration research) — a dedicated research pass, matching FR-036's own
  real-source-research method, specifically targeting the two remaining
  uncalibrated placeholder constants FR-036 itself deliberately left
  untouched: `arena::FILLET_RADIUS` and `arena::CORNER_ARCH_RADIUS`.
  Searched this port's established reference tier (RocketSim/RLUtilities
  source, the RLBot wiki, RLGym's game-value list) and found exactly one
  candidate: the RLBot wiki's uncited "Wall bottom ramp radius: Aprox. 256
  (but they are not circular)". Deliberately not adopted for either
  constant: it carries no citation, doesn't distinguish the two constants'
  distinctly different radii, explicitly disclaims being a true circular
  arc, and shares its numeral with RLGym's own unrelated `RAMP_HEIGHT`
  constant (a ramp's height from the ground, not a curve's radius),
  suggesting the wiki entry may be a conflation rather than an independent
  measurement. Both constants remain unchanged (`292.0`/`750.0`) and
  genuinely uncalibrated. Doc comments on both, plus this spec's Non-goals
  and Open Questions, updated to record the finding so a future
  contributor doesn't re-tread the same search — genuinely closing this
  gap needs actual extracted collision-mesh geometry (e.g. via
  `ZealanL/RLArenaCollisionDumper`), the same "requires the owner's own
  Windows/Rocket League environment" blocker `RB-VERIFY-002-FR-001`
  already documents. No new tests (documentation-only, no value changed,
  the same precedent FR-031/FR-036 established); all 271 pre-existing
  tests pass unchanged (total unchanged from FR-038).
- 0.39.0 (2026-08-31): FR-038 added and implemented (car-vs-net contact) —
  closes this port's own former Non-goal that a car passes straight
  through a `net::NetMesh`'s spatial footprint untouched
  (`RB-PHYSICS-001-FR-033`'s own entry). `net::NetMesh::step` changed from
  a single `&mut RigidBody` (the ball alone) to `&mut [RigidBody]` (every
  body that can touch the net); its inner contact-resolution loop now
  iterates every body in the slice against each free point instead of just
  one parameter. No new collision code needed: `collision::contacts_between`
  already dispatches to `sphere_vs_box` for a car (box) against a net point
  (sphere) the same way it always has for ball-vs-car. `PhysicsWorld::step`
  reuses the same ball-plus-cars snapshot `solver::resolve_dynamic_manifolds`
  already resolved that step for the net-step call too, deferring the sync
  back to `self.ball`/`self.cars` until after every net has had its turn.
  All of `net.rs`'s pre-existing tests updated only their call syntax
  (`std::slice::from_mut(&mut ball)`), not their own assertions — a
  single-element slice behaves identically to the old signature. 3 new
  tests: 2 in `net.rs` (`a_car_shot_into_the_net_is_measurably_slowed_compared_to_free_flight`,
  the direct car analog of the pre-existing ball version, and
  `a_ball_and_a_car_are_both_resolved_against_the_same_net_step`, proving
  the slice's own iteration resolves every element, not just the first)
  and 1 in `world.rs`
  (`a_car_shot_at_a_goal_net_is_caught_instead_of_passing_through_untouched`,
  the live-`PhysicsWorld` "caught vs. free flight" proof mirroring the
  ball's own version) — floated near the net panel's own vertical center
  rather than resting on the ground, since a car sized to rest at ground
  height would only ever reach the panel's anchored bottom row, which
  `NetMesh::step`'s own contact-resolution loop deliberately skips — a
  real trap this test's own first draft fell into before being corrected.
  Bringing the crate to 271 total (+3 over FR-039's 268).
- 0.38.0 (2026-08-31): FR-039 added and implemented (wall-jump corner
  disambiguation) — closes the "first wall in `self.walls`" simplification
  FR-013 originally documented and FR-019's new diagonal corner walls made
  reachable in the standard arena for the first time.
  `PhysicsWorld::step`'s per-car wall-normal computation now collects every
  wall a car is touching this step, sums their normals, and normalizes the
  result, instead of `Iterator::find`-ing the first match — a car touching
  exactly one wall gets that wall's own normal back unchanged (summing a
  single unit vector and normalizing it is a no-op), so the far more common
  single-wall case is bit-for-bit unaffected; a car touching two walls at
  once now pushes off diagonally along the sum of both, instead of firing
  along only one of them depending on iteration order. No new collision
  code was needed — `resolve_plane_contact` already resolved simultaneous
  multi-wall contact correctly; only the wall-jump push-off direction
  picker, `drive::apply_driven_forces`'s own input, was affected. 1 new
  `world.rs` test,
  `a_car_touching_two_walls_at_a_corner_wall_jumps_diagonally_outward`
  (two perpendicular walls, a car touching both at once, asserting both
  horizontal velocity components come out positive and roughly equal after
  a wall jump), bringing the crate to 268 total (+1 over FR-037's 267).
- 0.37.0 (2026-08-31): FR-037 added and implemented (sleeping) — closes
  the "no sleeping" half of `solver`'s own documented gap FR-035 left open,
  and with it the actual fix for a *bouncy* resting contact never
  settling, since restitution re-triggers off a fresh gravity-induced
  closing velocity every frame regardless of where the solver's iteration
  starts, so nothing about warm-starting or split impulse could ever stop
  the residual bounce. New `body::RigidBody` fields `is_sleeping: bool`
  (public) and `sleep_timer: f32` (private), plus
  `update_sleep_state(&mut self, dt: f32)` (accumulates `sleep_timer` while
  both `linear_velocity.length()`/`angular_velocity.length()` stay under
  new `LINEAR_SLEEP_VELOCITY_THRESHOLD`/`ANGULAR_SLEEP_VELOCITY_THRESHOLD`
  constants, setting `is_sleeping` and forcibly zeroing both velocities
  once `sleep_timer` reaches `SLEEP_TIME_THRESHOLD`, repeated every
  subsequent call while still under threshold; crossing either threshold
  resets the timer and clears `is_sleeping` immediately) and `wake(&mut
  self)` (the same reset, unconditionally, independent of velocity).
  `PhysicsWorld::step` calls `update_sleep_state` for the ball and every
  car right after every other contact this step resolves but before the
  transform integrates. `drive::apply_driven_forces` calls `car.wake()`
  unconditionally, before anything else in that call runs, whenever a new
  `input_is_active` helper finds the car's `ControllerInput` genuinely
  active — necessary because a resultant-velocity-only wake check isn't
  enough for a driven body: a car accelerating from rest under a small
  per-frame driving force whose own one-frame velocity delta is itself
  smaller than the sleep threshold would otherwise have that delta zeroed
  right back out every frame, permanently stranding it.
  `input_is_active` treats an unrecovered analog channel (`None`) the same
  as a recovered-but-literally-neutral one (`Some(0.0)`), rather than the
  simpler `!= ControllerInput::default()`, which would keep a car fed a
  real recorded input stream that always resolves every channel from ever
  sleeping at all. All three new threshold constants are this project's
  own uncalibrated placeholders — no public reference exists for what, if
  any, real Rocket League's own physics engine uses internally for this
  purely implementation-internal stabilization detail. 8 new tests (5 in
  `body.rs` exercising `update_sleep_state`/`wake` directly, 3 in
  `world.rs` proving the fix through a live `PhysicsWorld` — a
  nonzero-restitution ball/ground pair actually falling asleep at exactly
  zero velocity, a car seeded already asleep waking the instant throttle
  is applied, and a sleeping ball waking when a moving car hits it),
  bringing the crate to 267 total (+8 over FR-036's 259). All pre-existing
  tests pass unchanged, including `resting_ball_stays_at_rest` (whose own
  comment, describing the bouncy-resting-contact limitation this
  requirement fixes, was corrected to point at the new test that now
  demonstrates the fix instead of only documenting the gap) and
  `dropped_ball_eventually_settles_on_the_ground` (already used a bouncy
  restitution of 0.3, unaffected since it only ever checked landing height,
  not whether the ball kept bouncing forever).
- 0.36.0 (2026-08-31): FR-036 added and implemented (ball radius /
  `CEILING_Z` constant-ambiguity resolution) — a dedicated follow-up to
  FR-031's own audit, using real source-level research (RocketSim's and
  RLUtilities' own source, and the current RLBot wiki, read directly rather
  than guessed at) instead of leaving both ambiguities open indefinitely.
  Ball radius: FR-031 had framed this as "`92.75` vs. `91.25`", but the
  real games actually split the ball into a smaller inertia radius
  (`91.25`) and a distinctly larger collision radius (`93.15`, the mesh's
  own collision margin) — a split this port's single unified radius field
  can't represent, and since this port has no separate Bullet-style
  collision margin, the collision radius is the correct single-constant
  analog, not the inertia radius. Every `92.75` literal across
  `solver.rs`/`world.rs`/`net.rs`/`collision.rs` became `93.15`, not
  `91.25`. `arena::CEILING_Z`: confirmed, via both RocketSim's
  `ARENA_HEIGHT = 2048.f` and an independent reconstruction from real
  extracted collision-mesh geometry, to share the same reference point,
  so `2044.0` became `2048.0`. Two mis-documented claims were also
  corrected as a low-risk byproduct: `arena::CORNER_LENGTH` and
  `arena::GOAL_DEPTH` were wrongly described (by FR-019/FR-031 and FR-029
  respectively) as uncalibrated placeholders with no public reference —
  both are actually confirmed exact, so only their doc comments changed,
  not their values. `arena::FILLET_RADIUS`/`CORNER_ARCH_RADIUS` remain
  untouched and still genuinely uncalibrated — no analytic reference exists
  for either, a separate, more involved mesh-ingestion follow-up
  deliberately left for later. Before the mechanical substitution, a
  targeted `grep -c "892\.75"` across the four non-`arena.rs` files
  confirmed zero matches in each, ruling out corruption of
  `arena::GOAL_HALF_WIDTH`'s unrelated `892.755` literal (edited by hand in
  `arena.rs` instead, which was excluded from the substitution). No new
  tests: a constant-only correction with no new behavior to characterize,
  the same precedent FR-031 established; `cargo test --workspace` re-run
  clean at 259 total (unchanged from FR-035), confirming the change is
  behavior-preserving everywhere the old values were exercised.
- 0.35.0 (2026-08-31): FR-035 added and implemented (warm-starting,
  `resolve_dynamic_manifolds` only) — a new `solver::ContactCache` carries
  a manifold's converged real-channel impulses (normal plus both friction
  rows) from one call to the next, matched by each contact's approximate
  world position (`CONTACT_MATCH_DISTANCE`, an uncalibrated placeholder).
  A new `warm_start_two_body_row` applies each row's cached impulse
  directly to the manifold's shared `DeltaVelocity` accumulators *before*
  iterating — critically not just setting `TwoBodyRow::applied_impulse`,
  which (with `GLOBAL_CFM` always `0.0`) would leave the first iteration's
  correction identical to a cold start; the seed has to be baked into the
  starting delta itself, mirroring Bullet's own warm-start applying the
  cached impulse to the solver body's temporary velocity at setup, before
  any iteration runs. `resolve_dynamic_manifolds` gained a new `caches: &mut
  HashMap<(usize, usize), ContactCache>` parameter (key normalized so
  either argument order finds the same entry); every call rebuilds
  `caches` from only that call's manifolds, so a pair no longer touching
  is dropped automatically, no separate eviction pass needed.
  `PhysicsWorld` gains one `dynamic_manifold_caches` field, persisted
  across steps. Deliberately scoped to this one call site:
  `resolve_contacts`/`resolve_contacts_between` (every static-geometry
  contact, for both the ball and every car) stay un-warm-started, since
  this port's fixed `SOLVER_ITERATIONS` already fully converges every
  one-body/two-body scenario this crate tests — warm-starting has no
  scenario to demonstrate value against there yet, while
  `resolve_dynamic_manifolds` already had one:
  `RB-PHYSICS-001-FR-030`'s own documented extreme-mass-ratio "sandwiched"
  case, which doesn't fully converge within one call's iteration budget. 1
  new `solver.rs` test,
  `warm_starting_a_sandwiched_ball_across_two_calls_converges_closer_than_a_repeated_cold_start`,
  reuses that exact scenario: call 1 (cold) partially converges and
  populates a cache; from that identical post-call-1 state, call 2 runs
  twice on independent copies — once warm (call 1's cache), once cold (a
  fresh map) — with identical positions, contacts, velocities, and
  iteration budget both times, isolating exactly what the warm seed
  contributes; the warm run lands measurably closer to the true
  zero-velocity equilibrium. All 14 of `solver.rs`'s pre-existing tests
  pass unchanged when given an empty cache, confirming this requirement is
  behavior-preserving for every case they already covered. This
  requirement does NOT fix this spec's own documented "bouncy resting
  contact never settles" limitation — that symptom comes from restitution
  re-triggering off a fresh gravity-induced closing velocity every frame,
  independent of where the solver's iteration starts; warm-starting
  converges the same wrong-looking bounce faster, it doesn't stop it from
  recurring. Sleeping (still unimplemented) is the actual fix for that,
  and remains the sole open item under the old combined
  "no-warm-starting-or-sleeping" bullet this requirement splits. Bringing
  the crate to 259 tests total (+1 over FR-034's 258).
- 0.34.0 (2026-08-31): FR-034 added and implemented (split impulse) —
  closes the "no split impulse" half of this spec's own documented
  solver-simplification gap (see Non-goals/Open questions), leaving only
  warm-starting/sleeping open. `ConstraintRow`/`TwoBodyRow` (`solver.rs`)
  each gained `rhs_penetration`/`applied_push_impulse` fields, splitting
  the normal row's combined `rhs = (positional_error + velocity_error) *
  jac_diag_ab_inv` into two independent terms (a friction row's
  `rhs_penetration` is always `0.0`). Two new resolve functions,
  `resolve_push_row`/`resolve_two_body_push_row`, run the same
  projected-Gauss-Seidel iteration as `resolve_row`/`resolve_two_body_row`
  but against `rhs_penetration`/`applied_push_impulse` and a separate
  `push_delta`/`push_delta_a`/`push_delta_b` accumulator, always clamped
  to `[0, UPPER_LIMIT]`. A new `apply_push_delta` applies that accumulated
  push delta directly to a body's position/orientation via the existing
  `integrate::integrate_transform` — mirroring Bullet's own
  `btSolverBody::writebackVelocity`'s second, independent
  `integrateTransform` call. `resolve_contacts`, `resolve_contacts_between`,
  and `resolve_dynamic_manifolds` (the last via a new per-body-index
  `push_deltas` vector, reusing `delta_pair_mut`) each
  gained the push-channel resolve/apply calls alongside their pre-existing
  real-channel ones; no other module or call site changed. 2 new
  `solver.rs` tests
  (`split_impulse_corrects_deep_penetration_via_position_not_velocity`/
  `..._between_two_bodies`) directly prove a deeply-penetrating, at-rest
  contact leaves near-zero real velocity while the body/bodies' positions
  measurably separate. This requirement also surfaced (via failing tests,
  not by design) that 4 pre-existing `world.rs` live end-to-end fillet
  tests
  (`a_ball_embedded_in_a_vertical_corner_edges_fillet_footprint_is_pushed_toward_the_axis`,
  `a_ball_embedded_in_a_compound_corner_fillets_footprint_is_pushed_toward_the_center`,
  `a_ball_embedded_in_a_goal_posts_fillet_footprint_is_pushed_toward_the_axis`,
  `a_ball_embedded_in_a_goal_corner_fillets_footprint_is_pushed_toward_the_center`)
  had encoded the *old*, pre-split-impulse behavior in their own
  assertions — each expected the ball to keep coasting past its resting
  distance under residual velocity the old combined `rhs` term left
  behind, and each now instead settles almost exactly at that resting
  distance with no such residual velocity to coast on. Their assertions
  and comments were updated to check settling at (not past) the resting
  distance, a strictly stronger and more physically correct proof than
  the "moved meaningfully" one they replace — a direct sign this
  requirement's fix is real and not just internally self-consistent.
  Bringing the crate to 258 tests total (+2 over FR-033's 256, plus the 4
  fillet-test assertion corrections).
- 0.33.0 (2026-08-31): FR-033 added and implemented (genuine net mesh,
  ball only) — closes the "genuine net mesh" Non-goal `FR-029`'s own doc
  comment left open. New module `net` (`net::NetMesh`): a rectangular
  mass-spring grid of point masses (each a real `RigidBody::sphere`, tiny
  and light), every perimeter point anchored (fixed, representing
  attachment to the rigid goal frame) and every interior point free,
  connected by structural (horizontal/vertical) and shear (diagonal)
  springs (Hooke's law plus velocity damping). `NetMesh::step` sub-steps
  its own internal physics for stability and resolves the ball's contact
  against every free point it overlaps via a new `collision::sphere_vs_sphere`
  (this crate's first real sphere-vs-sphere contact — previously an
  unimplemented, callerless placeholder) plus the *existing*
  `solver::resolve_contacts_between` two-body path — no new solver code,
  reusing the same machinery every other dynamic-vs-dynamic contact in
  this crate already uses. New `arena::standard_nets` builds one
  `net::NetMesh` per goal, `NET_DEPTH` behind the real back wall and well
  in front of `FR-029`'s own rigid back-of-net plane (unchanged, still a
  car's real backstop, since a car isn't tested against the net at all —
  a documented Non-goal, not an oversight). `PhysicsWorld` gains
  `nets`/`with_net`, resolved after every other contact each step. Every
  new constant (`net::NET_POINT_MASS`/`NET_POINT_RADIUS`/
  `NET_SPRING_CONSTANT`/`NET_SPRING_DAMPING`/`NET_LINEAR_DAMPING`/
  `NET_RESTITUTION`/`NET_FRICTION`, `arena::NET_DEPTH`) is an uncalibrated
  placeholder — real Rocket League net material properties have never
  been published. 10 net new tests: 5 in `net.rs` (perimeter anchoring,
  zero-stretch springs at rest, anchored points immovable under gravity,
  an undisturbed net settling instead of oscillating forever, and the
  real catching proof — a ball fired at the net's own center loses over
  half its speed within 1 second compared to free flight); `collision.rs`
  replaced the old `contacts_between_two_spheres_is_empty` regression test
  (whose entire premise this requirement reverses) with 2 tests proving
  `sphere_vs_sphere`'s own correctness (net +1); 2 in `arena.rs`
  (`standard_nets` returns exactly 2 nets, each sitting `NET_DEPTH` behind
  the real back wall and spanning exactly the goal mouth's own footprint);
  2 in `world.rs` (a wiring-count test, plus the real live end-to-end
  proof — a ball fired at a lone net panel in an isolated minimal scene
  loses at least half its speed compared to the identical shot with no
  net present). Bringing the crate to 256 tests total (+10 over FR-032's
  246). Still not modeled: a car's own contact against a net, a full 3D
  "sock" shape, bending stiffness, and everything else FR-024's own
  Non-goals already cover.
- 0.32.0 (2026-08-31): FR-032 added and resolved (genuine
  convex-vs-curved-surface narrow phase investigation — no change to the
  narrow phase itself). Set out to replace `box_vs_quarter_pipe`/
  `box_vs_corner_fillet`'s per-corner technique with a real GJK/EPA
  convex-vs-convex narrow phase, on the strength of a limitation FR-027's
  own doc comments claimed: a face resting flush against a shallow curve
  could have every corner still clear of the fillet while the face's
  middle already overlapped it, under-detecting that case. Built a
  from-scratch GJK closest-points implementation (`gjk::closest_points`)
  and wired it in — doing so broke two pre-existing, previously-passing
  end-to-end tests
  (`a_car_resting_within_a_curved_transitions_footprint_is_pushed_up_off_the_flat_floor_height`,
  `a_car_embedded_in_a_compound_corner_fillets_footprint_has_its_penetration_reduced`),
  because closest-point is the wrong question: a quarter-pipe/corner-fillet
  contact is a *containment* question (is the box's farthest point from the
  axis/center at or beyond radius), and distance-from-a-line/point is a
  convex function whose maximum over a convex polytope (the box) is always
  attained at a corner — so the original per-corner technique is
  mathematically exact for this question, not an approximation. Reverted
  `box_vs_quarter_pipe`/`box_vs_corner_fillet` to their original FR-027
  implementations, deleted the now-unused `gjk` module entirely (no
  remaining consumer), and corrected every doc comment across this crate
  and this spec that had inherited FR-027's unverified "approximation"/
  "under-detection" claim (`lib.rs`'s crate doc, this spec's own Purpose
  and scope, Non-goals, Requirements, and Verification plan sections, and
  the FR-027/FR-020 Open questions entry) to reflect the now-verified
  state. Added one new regression test,
  `no_point_on_a_boxs_face_is_ever_farther_from_a_quarter_pipes_axis_than_its_own_corners`,
  which densely samples (50×50 grid per face) all 6 faces of a car-sized
  box positioned exactly like the two broken end-to-end tests' scenario and
  confirms no sampled face-interior point ever exceeds the box's own 8
  corners' maximum distance from the axis. The goal wall's own analogous
  window-edge concern (`collision::box_vs_goal_wall`, FR-028) is a distinct
  question — the window boundary is a flat rectangle, not a curve — and
  remains open and unverified, not resolved by this investigation. 1 new
  test (246 total in `rb_physics_bullet`, +1 over FR-031's 245); no other
  test changes, since `box_vs_quarter_pipe`/`box_vs_corner_fillet`'s actual
  behavior is unchanged from FR-027.
- 0.31.0 (2026-08-31): FR-031 added and implemented (constant-calibration
  audit — does NOT close FR-005). Sourced every uncalibrated placeholder
  constant in `drive.rs`/`arena.rs`/`world.rs` against the RocketSim
  (`ZealanL/RocketSim`) and RLUtilities (`samuelpmish/RLUtilities`) source
  code plus the RLBot community wiki's "Useful Game Values" page — three
  independently-written reverse-engineering references, agreement across
  all three treated as high confidence. Corrected with code changes:
  `drive::JUMP_SPEED` (`292.0` → `875.0/3.0`, ≈291.667) and
  `drive::JUMP_HOLD_ACCELERATION` (`1400.0` → `4375.0/3.0`, ≈1458.33) to
  their precise real values; split `drive::MAX_CAR_SPEED` (2300, boost's
  cap, confirmed correct) from a new `drive::UNBOOSTED_MAX_CAR_SPEED`
  (1410, throttle's own cap) — a real behavioral fix, since throttle alone
  previously could reach the boosted top speed. Confirmed correct with no
  change: `drive::JUMP_HOLD_MAX_DURATION` (0.2), `drive::BOOST_ACCELERATION`
  (991.667), `drive::MAX_BOOST` (100), gravity (-650), `arena::GOAL_DEPTH`
  (880). Explicitly flagged as audited-but-still-uncalibrated (a real
  reference exists but doesn't safely port into this port's own unit
  system/mechanic shape, or no reference exists at all): `drive::DODGE_SPEED`,
  `drive::DODGE_ANGULAR_SPEED`, `drive::WALL_JUMP_HORIZONTAL_SPEED`,
  `drive::STEER_TORQUE`, `drive::AIR_CONTROL_TORQUE`,
  `drive::HANDBRAKE_FRICTION_MULTIPLIER`,
  `drive::LANDING_AUTO_UPRIGHT_TORQUE`, `arena::FILLET_RADIUS`,
  `arena::CORNER_ARCH_RADIUS`. Surfaced two open ambiguities without acting
  on them (ball radius 91.25 vs. this port's 92.75; `arena::CEILING_Z`
  2044 vs. RocketSim's cited 2048) — recorded in Open questions rather
  than guessed at. 1 new test (`drive::tests::throttle_alone_cannot_reach_the_boosted_top_speed`,
  245 total in `rb_physics_bullet`, +1 over FR-030's 244); the `JUMP_SPEED`/
  `JUMP_HOLD_ACCELERATION` precision refinements needed no new tests since
  every existing assertion already used a tolerance wide enough to absorb
  the small precision differences.
- 0.30.0 (2026-08-31): FR-030 added and implemented (combined multi-body
  solve) — closes the "combined multi-body solve" Non-goal/Open question
  this spec carried since FR-004: every ball-vs-car and car-vs-car
  contact manifold touching in the same step used to be resolved by its
  own independent call to `solver::resolve_contacts_between`, so a body
  touching two others at once (e.g. a car pinned between the ball and
  another car) never had both contacts reasoned about together — the
  second-resolved pair's setup used the first pair's already-finished
  result as its starting velocity, discarding almost all of the first
  contact's effect. New `solver::resolve_dynamic_manifolds(bodies,
  manifolds, dt)` shares one interleaved `SOLVER_ITERATIONS`-iteration
  budget across every manifold at once — every iteration processes every
  manifold once, each reading and updating the shared per-body-index
  `DeltaVelocity` accumulator a new `solver::delta_pair_mut` helper
  hands out (a `split_at_mut`-based disjoint-borrow generalization of the
  `b == a + 1` trick `PhysicsWorld::step`'s car-vs-car loop already used)
  — with velocities only applied once, after all iterations finish. The
  old `TwoBodyDelta` struct is removed in favor of two separate
  `DeltaVelocity` parameters on `resolve_two_body_row`, which is what
  makes sharing an accumulator across manifolds possible;
  `resolve_contacts`/`resolve_contacts_between`'s own public behavior is
  unchanged (the latter's internals were adjusted to match, a pure
  refactor). `PhysicsWorld::step` now builds one `Vec<RigidBody>` (ball
  plus every car), collects every non-empty ball-vs-car/car-vs-car
  manifold into one list, and resolves the whole scene's dynamic contacts
  with a single `resolve_dynamic_manifolds` call per step, replacing the
  two old independent loops and their now-deleted private
  `resolve_dynamic_contact` helper. Static contacts (ground, arena walls,
  curves, corner fillets, goal walls, bounded walls) are deliberately
  untouched — each depends on only one body, so independent resolution
  loses no cross-body information there. Measured via two new tests (1 in
  `solver.rs`, 1 in `world.rs`, bringing the crate to 244 total): a
  symmetric ball-between-two-closing-cars "pinch" whose true answer is
  every body near zero velocity. The old independent-pairwise approach
  left the ball at ~98-99% of a closing car's own speed (~98.9 units/s) —
  the first contact's effect almost entirely discarded; the new combined
  solve leaves it noticeably slower (~89.5 units/s at this crate's
  existing `SOLVER_ITERATIONS = 10`) — a real, measurable improvement,
  but not full convergence (confirmed, during test design only, to
  converge much closer at 300 iterations, while the old approach's result
  doesn't change at all regardless of iteration count — proving its error
  is structural, not an iteration-count shortfall). Both tests
  deliberately assert the direction/magnitude of the improvement, not
  exact convergence to zero, matching this spec's practice of not
  overclaiming what a test proves. Non-goals unaffected: split impulse,
  warm-starting/sleeping, and the average-not-max restitution/friction
  combine mode are exactly as documented before this requirement.
- 0.29.0 (2026-08-31): FR-029 added and implemented (modeled goal
  interior) — closes the last remaining goal-cutout Non-goal repeated
  across FR-024 through FR-028: the goal-mouth window opened onto
  completely open, unbounded space, so a ball or car passing through (the
  ball since FR-024, a car since FR-028) sailed forever with nothing
  behind the window to stop it. This closes that gap with a bounded
  interior volume behind each goal-mouth window — explicitly a solid
  bounding box, **not** a springy/catching net mesh (no cloth/soft-body
  simulation was added); a deliberate, honest scoping decision, not an
  oversight, with a genuine net mesh remaining a real, separate,
  not-yet-implemented Non-goal. Three pieces, all in
  `crates/rb_physics_bullet`: a new constant `arena::GOAL_DEPTH: f32 =
  880.0` — an uncalibrated placeholder, since this port has no verified
  reference for Rocket League's real net depth at all (unlike
  `GOAL_HALF_WIDTH`/`GOAL_HEIGHT`), chosen only to be a visibly real
  interior volume comparable in scale to the goal mouth's own dimensions;
  a new shape type, `body::StaticBoundedWall` — a flat `StaticPlane` that
  only collides *within* a rectangular bound in the plane's own local 2D
  frame (`bound_center`/`u_axis`/`v_axis`/`half_u`/`half_v`, plus a
  `contains_in_bound` method) — the opposite gate convention from
  `StaticGoalWall`'s window (collides everywhere *except* inside a
  rectangle), needed because the goal's own side walls and roof can't be
  plain unbounded `StaticPlane`s (an infinite plane at, say, `x =
  GOAL_HALF_WIDTH` would incorrectly wall off the entire main field at
  that x coordinate — the exact same problem `arena::goal_post_plane`'s
  own pre-existing doc comment already documented for a different
  purely-geometric plane used only to derive fillets), with new dispatch
  functions in `collision.rs` (`sphere_vs_bounded_wall`/
  `box_vs_bounded_wall`/`contacts_vs_bounded_wall` — `box_vs_bounded_wall`
  uses the same "test every corner" technique established by
  FR-027/FR-028, but a corner *outside* the bound is skipped, the opposite
  of `box_vs_goal_wall`'s per-corner window test); and new arena geometry
  functions in `arena.rs` — `goal_back_wall_plane`/
  `standard_goal_back_walls` (2 plain, unbounded `StaticPlane`s per goal,
  `GOAL_DEPTH` behind the real back wall, added to `PhysicsWorld.walls` via
  the existing `with_wall` builder, since nothing can ever reach this
  plane except by first passing through the goal-mouth window, so an
  unbounded plane here is exact, not an approximation), `goal_side_wall`/
  `standard_goal_side_walls` (4 `StaticBoundedWall`s total, each reusing
  `goal_post_plane` completely unchanged, bounded to the goal's own depth
  and height), and `goal_roof`/`standard_goal_roofs` (2
  `StaticBoundedWall`s total, each reusing `goal_crossbar_plane`
  completely unchanged, bounded to the goal's own width and depth).
  `PhysicsWorld` (in `world.rs`) gains a new field
  `bounded_walls: Vec<StaticBoundedWall>` and a `with_bounded_wall` builder
  (mirroring `with_goal_wall`), plus a new `resolve_bounded_wall_contact`
  (mirroring `resolve_goal_wall_contact`), resolved for the ball and every
  car in `PhysicsWorld::step`, exactly like `goal_walls`.
  `PhysicsWorld::standard_arena` wires in `standard_goal_back_walls` via
  `with_wall`, and `standard_goal_side_walls`/`standard_goal_roofs` via
  `with_bounded_wall`. No changes to the actual step-loop resolution
  pattern were needed beyond adding the new loop — unlike FR-027/FR-028,
  this isn't a "silent no-op that later got activated" story; it's
  straightforwardly new geometry wired in from the start.
  `PhysicsWorld.walls` grew from 7 to 9 real entries once `standard_arena`
  is built (the 2 new back-of-net planes), so the pre-existing `world.rs`
  test `standard_arena_has_seven_walls_and_the_standard_ground` was
  renamed `standard_arena_has_nine_walls_and_the_standard_ground` and its
  assertion updated — a test-count correction, not new capability. New
  tests: 4 in `body.rs` for `StaticBoundedWall::contains_in_bound`
  (`contains_in_bound_is_true_for_the_bounds_own_center`,
  `contains_in_bound_is_true_just_inside_each_edge`,
  `contains_in_bound_is_false_just_outside_each_edge`,
  `contains_in_bound_ignores_distance_from_the_plane_itself`), mirroring
  the pre-existing `StaticGoalWall::contains_in_window` tests exactly with
  the boolean gate meaning inverted; 5 in `collision.rs`
  (`sphere_inside_the_bound_behaves_like_an_ordinary_plane`,
  `sphere_outside_the_bound_has_no_contact`,
  `box_squarely_inside_the_bound_behaves_like_an_ordinary_plane`,
  `box_straddling_the_bounds_edge_only_collides_on_the_corners_still_inside_it`,
  `box_entirely_outside_the_bound_has_no_contact`) against a synthetic
  fixture, mirroring the `StaticGoalWall` collision tests with the gate
  inverted; 8 in `arena.rs` proving the geometry functions place things
  correctly (`standard_goal_back_walls_has_two_walls`,
  `every_goal_back_wall_sits_goal_depth_behind_the_real_back_wall`,
  `standard_goal_side_walls_has_four_walls`,
  `every_goal_side_walls_plane_matches_some_goal_post_plane`,
  `every_goal_side_walls_bound_covers_the_real_goal_depth_and_height`,
  `standard_goal_roofs_has_two_roofs`,
  `every_goal_roofs_plane_is_the_goal_crossbar_plane`,
  `every_goal_roofs_bound_covers_the_real_goal_width`); and 4 in
  `world.rs` — 1 wiring-count test (`standard_arena_has_six_bounded_walls`)
  and 3 new live end-to-end `PhysicsWorld` proofs
  (`a_ball_shot_into_the_goal_is_stopped_by_the_goal_back_wall`,
  `a_ball_shot_sideways_inside_the_goal_is_stopped_by_a_goal_side_wall`,
  `a_ball_shot_upward_inside_the_goal_is_stopped_by_the_goal_roof`). Net
  +21 tests, bringing the crate to 242 tests total. These 3 end-to-end
  tests are deliberately isolated to a minimal `PhysicsWorld` built from
  just the specific new wall(s) under test (`PhysicsWorld::new` plus
  `with_wall`/`with_bounded_wall`, not `PhysicsWorld::standard_arena`) —
  discovered empirically while writing them: using the full
  `standard_arena`, a ball fired sideways or upward from deep inside the
  goal box got flung to bizarre, wildly wrong positions (e.g. ending up
  at x=-687 after being fired only in +x), root-caused to the
  pre-existing, already-documented `StaticQuarterPipe` limitation that a
  fillet's sector-membership test only checks angular position around its
  own axis, not radial distance (the same category of issue as the
  earlier FR-025 test-writing discovery), which the standard arena's own
  goal-cutout post/crossbar fillets (sitting right at the window's edge)
  could spuriously trigger for a point deep inside the goal box; isolating
  the test scene to just the wall(s) actually under test sidesteps this
  entirely and is the correct fix, not a bug in the new
  `StaticBoundedWall`/`goal_back_wall_plane` code itself. Additionally, an
  early version of these 3 tests set only the ball's own `restitution =
  0.0` and got nondeterministic results for the roof test specifically
  (the ball ending up below its own starting height), root-caused to the
  wall's own default `StaticPlane::new` restitution (0.5) still applying
  in the solver's contact resolution regardless of the ball's own value,
  causing it to bounce back down with enough remaining simulation time to
  travel well past its start; fixed by also explicitly zeroing the
  specific wall(s)' own `plane.restitution` in each of these 3 tests
  before adding them to the scene, so the ball damps out deterministically.
  Still not modeled: a genuine net *mesh* — no cloth/soft-body simulation,
  no visual net sag, no "ball tangles in netting" behavior; this is a
  solid bounding volume standing in for the net's functional role
  (stopping the ball/car), nothing more.
- 0.28.0 (2026-08-31): FR-028 added and implemented (car actually driving
  into a goal) — closes the last remaining half of the goal-cutout
  Non-goal FR-024 through FR-027 kept repeating:
  `collision::contacts_vs_goal_wall` dispatched a `Shape::Box` straight through to
  plain `contacts_vs_plane` against the wrapped `StaticPlane`, completely
  ignoring the goal-mouth window `StaticGoalWall` carries, even though a
  sphere (the ball) already passed through the window via
  `sphere_vs_goal_wall`. New `collision::box_vs_goal_wall(position,
  orientation, half_extents, wall) -> Vec<Contact>` iterates the box's 8
  corners exactly like `box_vs_plane` does, but for each corner first
  checks `wall.contains_in_window(&world_corner)` — a corner inside the
  window contributes no contact at all (`continue`), regardless of how
  deep it might be penetrating along the plane's normal, exactly
  mirroring `sphere_vs_goal_wall`'s own distance-along-normal-independent
  window test (`contains_in_window` needed no changes); a corner outside
  the window falls through to an ordinary `box_vs_plane`-style corner
  test. This is the exact same "test every corner" approximation
  technique `RB-PHYSICS-001-FR-027` established for curved fillets,
  applied here to a flat windowed plane instead. One real behavioral
  consequence worth documenting: because each corner is tested
  independently, a car only partially lined up with the window gets a
  *partial* block — contacts register on whichever corners are still
  outside it — rather than the all-or-nothing result a single-point
  sphere test necessarily produces. `contacts_vs_goal_wall`'s dispatch
  changed from `Shape::Box { .. } => contacts_vs_plane(body,
  &wall.plane)` to `Shape::Box { half_extents } =>
  box_vs_goal_wall(body.position, body.orientation, half_extents,
  wall)`. No `world.rs` step-loop changes were needed — exactly like
  FR-027's own discovery: `PhysicsWorld::step`'s
  `resolve_goal_wall_contact` was already being called for every car in
  the scene (it always needed the wall's plain-plane collision even
  before this fix), so this is a pure dispatch-function change; only
  doc-comment updates were needed (`goal_walls`' field doc comment,
  `with_goal_wall`'s doc comment, and `resolve_goal_wall_contact`'s doc
  comment in `world.rs`; `body.rs`'s `StaticGoalWall` doc comment;
  `arena.rs`'s module-level and `standard_goal_walls`'s own doc
  comments; and `lib.rs`'s crate-level module doc comment), removing
  "unwindowed"/"falls straight through to an ordinary plane contact,
  ignoring the window" language now that a car passes through too. New
  tests: `collision.rs` replaced the old
  `box_vs_goal_wall_ignores_the_window_entirely` regression test, whose
  entire premise this requirement reverses, with three new tests —
  `box_squarely_inside_the_goal_window_has_no_contact` (every corner
  inside the window gives an empty manifold, the box equivalent of
  `sphere_embedded_in_the_goal_window_has_no_contact`),
  `box_straddling_the_goal_window_edge_only_collides_on_the_corners_still_outside_it`
  (a car centered on the window's own edge gets exactly 2 contacts, only
  on the corners outside it — the real proof of the partial-block
  behavior above), and
  `box_entirely_outside_the_goal_window_behaves_like_an_ordinary_plane`
  (a car nowhere near the window collides identically to plain
  `contacts_vs_plane`, the box equivalent of
  `sphere_outside_the_goal_window_behaves_like_an_ordinary_plane`) — net
  +2. `world.rs` replaced
  `a_car_is_still_stopped_by_the_standard_arenas_back_wall_at_the_goal_mouth`,
  whose entire premise this requirement reverses, with
  `a_car_shot_through_the_goal_mouth_passes_the_standard_arenas_back_wall`
  (a live end-to-end `PhysicsWorld` proof: a car fired at the exact same
  goal-mouth-center position/velocity the pre-existing ball test uses
  ends up past `BACK_WALL_Y` instead of being stopped, reusing that
  test's already-solved 1.8s flight-duration bound) and added a new
  regression guard,
  `a_car_aimed_away_from_the_goal_mouth_is_still_stopped_by_the_back_wall`
  (a car aimed well outside `GOAL_HALF_WIDTH`, at the solid part of the
  wall, is still stopped by it after 3.0s) — net +1. Net +3 tests,
  bringing the crate to 221 tests total (net +2 in `collision.rs`, net +1
  in `world.rs`). Still not modeled: a modeled goal interior/net beyond
  the cutout itself, a genuine convex-vs-curved-surface narrow phase for
  a car (support-mapping/SAT-style, e.g. GJK/EPA) in place of this
  port's corner-testing approximations, and everything else FR-024's own
  Non-goals already cover.
- 0.27.0 (2026-08-31): FR-027 added and implemented (car deflection by
  curved fillets) — closes the long-repeated Non-goal, documented across
  FR-020 through FR-026, that a car (box) drove straight through every
  curved fillet in this port unaffected while the ball was properly
  deflected. New `collision::box_vs_quarter_pipe(position, orientation,
  half_extents, pipe)` tests each of the box's 8 corners as a zero-radius
  sphere via the existing `sphere_vs_quarter_pipe(world_corner, 0.0, pipe)`
  call and collects every corner that reports a contact — exactly the same
  "test every corner" technique `box_vs_plane` already uses for a box
  against a flat plane (already in this codebase, generating up to 4
  contacts for a box resting on a plane), generalized here to a curved
  surface instead of a flat one; new `box_vs_corner_fillet` does the
  identical thing against `sphere_vs_corner_fillet` for the sphere-shaped
  compound-corner fillets. `contacts_vs_quarter_pipe`/
  `contacts_vs_corner_fillet` now dispatch a `Shape::Box` to these new
  functions instead of returning `Vec::new()`. Each surviving contact's
  `point` field is overwritten to the corner's own world position rather
  than the fillet-surface point `sphere_vs_quarter_pipe`/
  `sphere_vs_corner_fillet` itself computes, for the same rel_pos/
  torque-accuracy reason `box_vs_plane`'s own doc comment already
  documents (a tilted box's corner isn't generally "below" the body center
  along the surface normal, so the solver needs the true contact-to-center
  offset). Both new functions' own doc comments document this explicitly
  as an approximation, not a full convex-vs-curved-surface narrow phase —
  no GJK/EPA support-mapping machinery was added, and this port still
  doesn't have any: a face of the box resting flush against a shallow
  curve (a radius large relative to the box) could have every one of its
  own corners still just clear of the fillet while the face's middle
  already overlaps it, under-detecting that case — the same category of
  "exact per test point, an approximation of the whole shape" caveat this
  crate has always carried for curved geometry. No new shape or
  fundamentally new collision primitive was needed — this is a
  generalization of existing dispatch, not new physics/math machinery.
  `PhysicsWorld::step` already called `resolve_curve_contact`/
  `resolve_corner_fillet_contact` for every car in the scene (wired in
  since FR-023), just always a no-op before now, since
  `contacts_vs_quarter_pipe`/`contacts_vs_corner_fillet` always returned
  empty for a box — so no `world.rs` step-loop changes were needed, only
  doc-comment updates reflecting the new real behavior (the "no-op for a
  box" language in `resolve_curve_contact`/`resolve_corner_fillet_contact`'s
  own doc comments, the `curves`/`corner_fillets` field doc comments, the
  `with_curve`/`with_corner_fillet` builder doc comments, and `arena.rs`'s
  own module-level "Still not modeled" list). Explicitly unaffected:
  `StaticGoalWall`/`contacts_vs_goal_wall` — a car still sees the exact
  same solid, full-width back wall it always has, deliberately ignoring
  the goal-mouth window, since a goal wall isn't a curved fillet at all; a
  car actually driving into a goal remains a real, separate,
  not-yet-implemented capability. New tests: `collision.rs` renamed/
  replaced the 2 pre-existing "always empty for a box" regression tests
  (`box_vs_quarter_pipe_is_always_empty`, `box_vs_corner_fillet_is_always_empty`)
  with 4 new tests proving the corner-testing approximation actually works
  both ways — `box_embedded_in_the_quarter_pipes_footprint_has_contact` and
  `box_far_from_the_quarter_pipe_has_no_contact` for the cylindrical
  quarter-pipe, `box_embedded_in_the_corner_fillets_footprint_has_contact`
  and `box_outside_the_corner_fillets_bounds_has_no_contact` for the
  sphere-shaped compound-corner fillet (net +2); `world.rs` replaced the
  pre-existing `a_car_is_not_deflected_by_a_curved_transition` regression
  test, whose entire premise this requirement reverses, with
  `a_car_resting_within_a_curved_transitions_footprint_is_pushed_up_off_the_flat_floor_height`
  (mirroring the ball's own equivalent live-physics proof) and added
  `a_car_embedded_in_a_compound_corner_fillets_footprint_has_its_penetration_reduced`
  (net +1) — the latter deliberately asserts that the fillet's own worst
  (maximum) corner penetration shrinks after the solver runs, not that the
  box's center of mass moves closer to the fillet's center the way the
  equivalent ball test does, since an earlier version asserting
  center-of-mass distance actually failed empirically (a box's multiple
  simultaneously-penetrating corners can rotate the box away from the
  fillet's center even as every individual corner's own overlap shrinks —
  see the spec's own Verification plan for the full reasoning). Net +3
  tests, bringing the crate to 218 tests total (2 in `collision.rs` net
  above the 2 replaced, plus 1 in `world.rs` net above the 1 replaced).
  Still not modeled: a car actually driving into a goal, a modeled goal
  interior/net beyond the cutout itself, a genuine convex-vs-curved-surface
  narrow phase for a car (support-mapping/SAT-style, e.g. GJK/EPA) in place
  of this increment's corner-testing approximation, and everything else
  FR-024's own Non-goals already cover.
- 0.26.0 (2026-08-31): FR-026 added and implemented (goal post-crossbar
  corner fillets) — rounds off the two remaining sharp compound corners per
  goal (4 total, 2 per goal), where a post's own vertical-edge fillet
  (FR-024) meets the crossbar's own horizontal-edge fillet, an explicitly
  documented gap FR-024's own doc comment left open. New
  `arena::standard_goal_corner_fillets` builds all 4 by calling
  `StaticCornerFillet::between_three_planes` directly on the three real
  flat planes that meet at each vertex (the back wall, that post's own
  plane, and the crossbar) — the same direct-from-real-planes approach
  FR-023 used for the arena's own 16 compound corners, rather than deriving
  from the two edge fillets already built at that vertex, since a corner
  fillet's center is already exactly those two edge fillets' own common
  axis intersection. No new shape or collision code needed —
  `StaticCornerFillet`/`sphere_vs_corner_fillet` (FR-023) are already fully
  generic to any three non-parallel planes. Unlike FR-025, this increment
  reuses `FILLET_RADIUS` unchanged for all 4 new fillets rather than
  introducing a new radius constant, since both edge fillets meeting at a
  goal's post-crossbar corner already share `FILLET_RADIUS` — no
  mismatched-radius concern exists here. `PhysicsWorld::standard_arena`
  wires these 4 in via the same generic `with_corner_fillet` builder
  `standard_corner_fillets`'s 16 already use, so `PhysicsWorld.corner_fillets`
  now holds 20 total; the pre-existing test
  `standard_arena_has_sixteen_compound_corner_fillets` was renamed
  `standard_arena_has_twenty_compound_corner_fillets` and its assertion
  updated to match. New tests: `arena.rs` gained
  `standard_goal_corner_fillets_has_four_fillets` and
  `every_goal_corner_fillets_center_sits_radius_in_from_a_back_wall_a_post_and_the_crossbar`
  (the same "prove the real triple intersection, not an arbitrary point"
  style test FR-023's own arena-corner test used); `world.rs` gained
  `a_ball_embedded_in_a_goal_corner_fillets_footprint_is_pushed_toward_the_center`
  (the same live-physics "ball embedded past the fillet's own radius gets
  pushed back toward the center" proof already given for every other
  fillet type in this port, using a synthetic back-wall/post/crossbar
  3-plane fixture rather than the real arena's own numbers, matching this
  test file's established convention for fillet unit tests). Net +3 tests,
  bringing the crate to 215 tests total (2 in `arena.rs`, 1 in `world.rs`).
  Explicitly still out of scope: the goal's other two corners, where a post
  meets the floor — the window's own bottom edge sits exactly at floor
  level (`z = 0`), so a post's own fillet there simply ends flush with the
  ground, with no sharp, unrounded vertex requiring a blend, unlike the top
  post-crossbar corner this increment addresses. Still not modeled: a car
  actually being deflected by any fillet or driving into a goal, a modeled
  goal interior/net beyond the cutout itself, and everything else FR-024's
  own Non-goals already cover.
- 0.25.0 (2026-08-30): FR-025 added and implemented (corner-wall
  floor/ceiling arch radius) — gives a diagonal corner wall's own
  floor-seam and ceiling-seam fillets a distinctly larger, dedicated
  radius instead of reusing the cardinal walls' own `arena::FILLET_RADIUS`
  (292.0), matching real Rocket League's corner-boost area reading as a
  noticeably bigger, more swept curve than a cardinal wall's small
  rounding, not just a scaled-down copy of the same shape. New constant
  `arena::CORNER_ARCH_RADIUS` (750.0), documented as an uncalibrated
  placeholder same as every other arena dimension in this module, governs
  8 of `standard_curves`' 24 entries — the ones bridging one of the 4
  corner walls to the floor or ceiling; a compile-time
  `const _: () = assert!(CORNER_ARCH_RADIUS > FILLET_RADIUS);` enforces
  the "distinctly larger" relationship at build time. Because
  `StaticCornerFillet::between_three_planes` needs one shared radius
  across all three planes it blends — a mismatched radius wouldn't
  produce a geometrically valid single sphere satisfying all three
  radius-in conditions, and would also break FR-023's established
  "meets its adjoining edge fillets exactly where their axes cross"
  no-gap property — all 16 of `standard_corner_fillets`'s
  compound-corner fillets switch to `CORNER_ARCH_RADIUS` too, since every
  one of them touches one of the 8 now-larger arches. Unaffected, still
  using `FILLET_RADIUS` exactly as before: the 8 cardinal-wall
  floor/ceiling-seam fillets, the 8 vertical corner-edge fillets
  (`RB-PHYSICS-001-FR-022`), and the 6 goal-cutout edge fillets
  (`RB-PHYSICS-001-FR-024`) — independent, additive contact sources next
  to the bigger arches, not blended with them, per this port's established
  "no blended 3D corner" convention. New end-to-end `world.rs` test
  `a_ball_embedded_in_a_corner_walls_floor_arch_footprint_is_pushed_toward_the_axis` gives the live-physics proof for the new radius, the same
  "moved meaningfully" claim every other fillet test in this port makes,
  and additionally asserts `CORNER_ARCH_RADIUS > FILLET_RADIUS`.
  `arena.rs`'s existing tests were updated to match: `every_floor_or_ceiling_seam_curve_bridges_a_wall_to_the_floor_or_ceiling` now checks
  the first 8 of `standard_curves()`'s 24 entries against `FILLET_RADIUS`
  and the next 8 against `CORNER_ARCH_RADIUS` separately (previously all
  16 floor/ceiling entries were checked against one radius);
  `every_standard_curve_sits_radius_in_from_a_vertical_wall` now accepts
  either radius; `every_standard_corner_fillets_center_sits_radius_in_from_a_floor_or_ceiling_a_side_or_back_wall_and_a_corner_wall` now
  checks `CORNER_ARCH_RADIUS` instead of `FILLET_RADIUS`. Net +1 test (one
  new `arena.rs` test idea was implemented as the compile-time
  const-assert above instead of a runtime test, alongside the one new
  `world.rs` end-to-end test), bringing the crate to 212 tests total.
  While validating this change, the pre-existing `world.rs` test
  `a_ball_shot_through_the_goal_mouth_passes_the_standard_arenas_back_wall`
  (FR-024) started failing — a discovered-and-fixed test regression, not a
  new feature or Non-goal. Root cause: `StaticQuarterPipe` is documented
  (in `body.rs`, unchanged by this increment) as infinite along its own
  axis, not clipped to a corner wall's real, finite span, so a ball fired
  dead down the arena's own center line eventually re-enters *some*
  corner-wall floor-seam arch's resting shell far past the goal — already
  true before FR-025 with the smaller `FILLET_RADIUS` (verified against
  the pre-FR-025 code, where the ball drifts into this zone around
  y≈7650-7930 and gets a mild, harmless correction that still leaves it
  past the wall), but FR-025's bigger `CORNER_ARCH_RADIUS` moves that zone
  closer in (~y=6300-7700) and turns the same brush into a much sharper,
  solver-destabilizing correction (velocity spikes to tens of thousands of
  units/sec, throwing the ball back past the wall and failing the test's
  assertion). Fixed by shortening that one test's simulated flight
  duration from 3.0s to 1.8s — comfortably long enough to prove the ball
  clears the back wall (needs y > 5121, reaches y=5400 unobstructed by
  1.8s) while stopping well short of re-entering the infinite-fillet zone,
  with a code comment in the test explaining why; `StaticQuarterPipe`'s
  own documented scope is otherwise unchanged. Still not modeled: a car
  actually being deflected by any fillet, and everything else FR-024's own
  Non-goals already cover.
- 0.24.0 (2026-08-30): FR-024 added and implemented (goal cutouts) —
  opens an actual goal-mouth window in each back wall, rounded at its own
  rim, where every prior increment had a single solid, flat plane spanning
  the full width. New static shape `body::StaticGoalWall` — a `StaticPlane`
  plus a rectangular window in the plane's own local `u_axis`/`v_axis`
  frame (`window_center`, `half_width`, `half_height`), with
  `contains_in_window` testing a point's projection onto that frame
  directly, independent of the point's own depth from the plane. New
  `collision::sphere_vs_goal_wall`/`contacts_vs_goal_wall`: a sphere (the
  ball) gets no contact at all when its center falls inside the window,
  letting it pass through; a box (car) falls straight through to the
  ordinary `contacts_vs_plane` against the wrapped plane, deliberately
  ignoring the window — a zero-regression choice, since a car now sees
  literally the same contact-generation call it always did. `arena::standard_walls` drops its 2 back-wall `StaticPlane`s (now 7 planes
  instead of 9); new `arena::standard_goal_walls` returns them instead as
  2 `StaticGoalWall`s, windowed at new commonly-cited constants
  `GOAL_HALF_WIDTH`/`GOAL_HEIGHT`. New `arena::standard_goal_cutout_fillets`
  rounds each window's 3 edges (two posts, one crossbar, per goal — 6
  `StaticQuarterPipe`s, added to the same `curves` list `standard_curves`'s
  24 already populate), each derived via the existing
  `StaticQuarterPipe::between_planes` from the real back-wall plane and a
  second, purely-geometric plane (`goal_post_plane`/`goal_crossbar_plane`)
  representing the post's or crossbar's own inward-/downward-facing
  surface — positioned at exactly the window's own edge, so the fillet's
  tangent point lands exactly on the window boundary with no gap or
  overlap. Unlike `standard_walls`' real walls, these post/crossbar planes
  are never themselves added as collision geometry — an infinite plane
  facing straight along X (or capping Z) would incorrectly wall off the
  entire rest of the field at that coordinate, unlike a diagonal corner
  wall's own orientation, which stays non-binding everywhere except right
  at the true corner. `PhysicsWorld` gains a parallel
  `goal_walls: Vec<StaticGoalWall>` field and `with_goal_wall` builder,
  resolved for the ball *and* every car (unlike `curves`/`corner_fillets`'s
  ball-only resolution) — safe precisely because the box path is a no-op
  change from the prior plain-`StaticPlane` behavior; `PhysicsWorld::standard_arena` wires in both the goal walls and the goal-cutout fillets
  automatically. 17 new unit tests (4 in `body.rs` for
  `StaticGoalWall::contains_in_window`'s correctness against a synthetic
  fixture, 4 in `collision.rs` for `sphere_vs_goal_wall`'s contact
  generation and `contacts_vs_goal_wall`'s box-path equivalence to plain
  `contacts_vs_plane`, 5 in `arena.rs` for `standard_walls`'s reduced
  count and the new `standard_goal_walls`/`standard_goal_cutout_fillets`
  functions' counts and geometric correctness, and 4 in `world.rs` — two
  end-to-end tests proving a ball passes through the goal mouth while a
  car aimed at the same spot is still stopped by the wall, one proving a
  ball embedded past a goal-post fillet's own radius gets pushed
  meaningfully back toward the axis, and one confirming `standard_arena`
  carries exactly 2 goal walls) bring the crate to 211 tests total. Still
  not modeled: a car actually being deflected by any fillet or driving
  into a goal, a modeled goal interior/net beyond the cutout itself, and
  the goal's own two compound top corners where a post's fillet meets the
  crossbar's (see Non-goals).
- 0.23.0 (2026-08-30): FR-023 added and implemented (compound-corner
  fillets) — rounds off the last 16 sharp vertices in the standard arena's
  vertical boundary: the compound corners where a corner wall's own
  vertical-edge fillet (FR-022) meets a floor- or ceiling-seam fillet
  (FR-020/FR-021), near that corner wall's own top or bottom endpoint. A
  compound corner is where three planes meet at once, which no existing
  cylindrical `StaticQuarterPipe` can blend, so this version introduces a
  new static shape, `body::StaticCornerFillet` — an immovable sphere riding
  the concave inside of the vertex. Its `between_three_planes` constructor
  exploits the same "radius-in from every bridged plane" invariant
  `StaticQuarterPipe::between_planes` already relies on: the fillet's
  center must sit exactly `radius` in from all three planes, so it's also
  exactly `radius` in from each pair — meaning it already lies on all
  three of that vertex's own pairwise `between_planes` axis lines
  simultaneously, so the center is just those three lines' common
  intersection, solved directly via the classic three-plane-intersection
  cross-product form of Cramer's rule rather than from scratch.
  Containment (new `sphere_vs_corner_fillet` in `collision.rs`)
  generalizes a `StaticQuarterPipe`'s 2-sided sector test to a "spherical
  triangle": inside iff a direction's dot product with each of 3 `bounds`
  is non-negative, each bound the raw (non-normalized — only its sign is
  used) cross product of a pair of normals, sign-corrected against the
  third plane's own normal to always point toward the sharp corner —
  provably correct since that dot product is exactly the derivative of
  the third plane's signed distance along a candidate direction. No
  `.normalize()`/`.unwrap()` is needed or used anywhere in this new
  production code, consistent with `between_planes`'s own FR-022
  precedent. `arena::standard_corner_fillets` builds all 16 (4 per corner
  wall — floor+side, floor+back, ceiling+side, ceiling+back — times the 4
  corner walls) directly from the same three flat planes `standard_walls`
  already builds, reusing `FILLET_RADIUS` once again. `PhysicsWorld` gains
  a parallel `corner_fillets: Vec<StaticCornerFillet>` field and
  `with_corner_fillet` builder, resolved for the ball and every car
  exactly like `curves` (a no-op for a car, same deferred case as every
  other fillet); `PhysicsWorld::standard_arena` wires in all 16
  automatically. 13 new unit tests (4 in `body.rs` for
  `between_three_planes`'s center/tangent-point/containment correctness
  against a synthetic fixture, 5 in `collision.rs` for
  `sphere_vs_corner_fillet`'s contact generation, 2 in `arena.rs` for
  `standard_corner_fillets`' count and per-fillet radius-in property, and
  2 in `world.rs` — a `standard_arena` fillet-count check and an
  end-to-end test proving a ball embedded past a compound-corner fillet's
  own radius gets pushed meaningfully back toward the center, the same
  "moved meaningfully" — not "settled-and-stayed" — claim FR-020/FR-021/
  FR-022's own equivalent tests make, for the same residual-velocity
  reason) bring the crate to 194 tests total. Still not modeled: a car
  actually being deflected by any fillet, and goal cutouts (see
  Non-goals).
- 0.22.0 (2026-08-30): FR-022 added and implemented (curved corner-wall
  vertical-edge fillets) — rounds off the arena's last remaining sharp
  edges: the 8 vertical edges where each of the 4 diagonal corner walls
  meets its neighboring side or back wall. `arena::standard_curves` now
  builds 24 `StaticQuarterPipe`s (the 16 floor/ceiling-seam fillets
  FR-020/FR-021 already built, plus 8 vertical-edge fillets). Unlike every
  prior fillet, the two planes a vertical-edge fillet bridges aren't
  perpendicular — a corner wall meets its neighbor at 135 degrees (given
  `standard_walls`' 45-degree corner cut), not 90 — which exposed a real
  gap in `StaticQuarterPipe::between_planes`: it previously only computed
  the correct axis point for perpendicular planes, via a shortcut (summing
  the two scaled normals) that silently gives the wrong point for any other
  angle. `between_planes` is now fully general: it solves the axis point as
  an actual 2x2 linear system in the (possibly non-orthogonal) basis the
  two normals form, and its own sector angle comes out to exactly
  `arccos(dot(plane_a.normal, plane_b.normal))` — a right angle for
  perpendicular planes as before, or (for this requirement's own geometry)
  a shallow 45 degrees, the supplement of the walls' 135-degree dihedral
  angle. `sphere_vs_quarter_pipe`'s sector-membership test is likewise
  generalized from the old two-dot-products check (only correct for a
  90-degree sector, since its two edges happen to be perpendicular) to a
  signed-cross-product test against `axis_direction`, exact for any sector
  up to 180 degrees. Since the general test depends on `axis_direction`'s
  own sign/handedness — unlike the old test, which never used
  `axis_direction` at all — `between_planes` now self-corrects a
  "backwards" `axis_direction` internally, so a caller can pass either of
  the two opposite directions along the shared edge line without reasoning
  about which is correct. The vertical-edge fillets' own `axis_direction`
  is simply `(0, 0, 1)` (the edge itself is vertical) — no cross product
  needed, unlike the corner-wall floor/ceiling-seam case. `FILLET_RADIUS` is
  reused as-is once again rather than a separate, smaller radius for these
  visibly shallower edges. Still not modeled: a car actually being
  deflected by any fillet, the compound corner where a vertical-edge
  fillet meets a floor- or ceiling-seam fillet, and goal cutouts (see
  Non-goals). 9 new unit tests across `body.rs`/`arena.rs`/`world.rs` in
  `rb_physics_bullet` (181 total): 5 in `body.rs`, using a synthetic
  non-perpendicular fixture independent of the arena's own geometry — the
  axis still sits exactly `radius` in from both planes with tangent points
  exactly on each; the derived sector angle matches the angle between the
  two planes' normals (45 degrees for this fixture); the sharp corner the
  fillet replaces sits outside its own radius but within its sector (the
  real proof the generalized sector orientation actually faces the missing
  material); and passing either of the two opposite `axis_direction`
  choices produces the same correctly-oriented sector; 3 in `arena.rs` —
  `standard_curves` returns exactly 24 fillets, every vertical-edge
  fillet's `axis_direction` runs purely along Z, and a corner wall's own
  vertical-edge fillet sits radius-in from both the corner wall and its
  neighboring side wall with a 45-degree sector; 1 in `world.rs` — the real
  end-to-end proof, a ball embedded past a vertical-edge fillet's own
  radius (at a wall-to-wall angle that isn't a right angle) gets pushed
  meaningfully back toward the axis (not a claim that it settles and stays
  at the exact resting distance — like every other fillet here, its
  contact stops firing once the overlap resolves, so nothing cancels
  whatever residual velocity the correction left the ball with, the same
  reason FR-020's and FR-021's own equivalent tests make the same weaker,
  "moved meaningfully" claim rather than an exact-settling one).
- 0.21.0 (2026-08-30): FR-021 added and implemented (curved
  corner-wall-to-floor/wall-to-ceiling transitions) — extends FR-020's
  fillet treatment to the 4 diagonal corner walls `FR-019` introduced.
  `arena::standard_curves` now builds 16 `StaticQuarterPipe`s (still one
  floor-side and one ceiling-side fillet per wall, now for all 9 walls)
  instead of 8. `StaticQuarterPipe::between_planes` itself needed no code
  changes: its real correctness requirement was never "axis-aligned
  planes" (as FR-020's own doc comment had incorrectly claimed) but only
  that the two bridged planes' normals, plus `axis_direction`, form an
  orthonormal basis — which only needs the two planes to be mutually
  *perpendicular*, true for a corner wall meeting the floor or ceiling
  regardless of the corner wall's own horizontal rotation (a vertical
  wall's normal always has zero Z component, and the floor/ceiling's is
  always purely Z). The only new work is in `arena.rs`'s
  `standard_curves`: a cardinal wall's fillet axis direction was always
  hand-picked as a coordinate axis, but a corner wall's own "along the
  wall" direction isn't one, so it's instead computed via a cross product
  (`floor.normal.cross(&wall.normal)`, and the ceiling equivalent) —
  already exactly unit length by construction (the two operands are always
  exactly perpendicular unit vectors), so no `.normalize()`/`.unwrap()` is
  needed, avoiding a `clippy::unwrap_used` violation the workspace's lint
  config promotes to a hard CI error in production code. A new
  `corner_wall_plane(sx, sy)` helper in `arena.rs` factors out the existing
  (behavior-unchanged) corner-wall plane construction `standard_walls`
  already did inline, so `standard_curves` can reuse it rather than
  duplicating the math. `PhysicsWorld::standard_arena` picks up the extra 8
  curves automatically, since it already loops over every curve
  `arena::standard_curves()` returns — no changes needed there.
  `FILLET_RADIUS` is reused as-is for the corner-wall fillets rather than
  introducing a second, independently chosen radius (see Verification
  plan). Still not modeled: a car actually being deflected by any fillet
  (unchanged from FR-020), a fillet at a corner wall's own *vertical* edges
  — where it meets its neighboring side/back wall at other than 90 degrees,
  a materially different problem `between_planes` doesn't address, since it
  only handles two perpendicular planes — and goal cutouts (see Non-goals
  and Open questions). 8 new unit tests across `arena.rs`/`world.rs` in
  `rb_physics_bullet` (172 total): `standard_curves` returns exactly 16
  fillets; every fillet's axis sits exactly `FILLET_RADIUS` in from some
  vertical wall, cardinal or corner; a corner wall's own derived fillet
  axis sits exactly `FILLET_RADIUS` in from both the corner wall and the
  floor, with correctly perpendicular unit sector vectors; the cross
  product computing each of the 4 corner walls' `axis_direction` is exactly
  unit length, confirming the production code's `.normalize()`-free
  assumption actually holds; plus — the real end-to-end proof — a new
  `PhysicsWorld` test built around a wall with a diagonal (non-axis-aligned)
  normal, rather than going through `arena::standard_curves` directly,
  confirms a ball resting at ordinary flat-floor height within that
  diagonal wall's fillet footprint gets pushed up off it, the same physical
  proof FR-020 gave for a cardinal wall, now for one whose normal isn't a
  coordinate axis.
- 0.20.0 (2026-08-30): FR-020 added and implemented (curved
  wall-to-floor/wall-to-ceiling transitions) — a new `body::StaticQuarterPipe`
  shape (an immovable partial-cylinder fillet, infinite along its own axis
  like `StaticPlane`) and `collision::contacts_vs_quarter_pipe` (sphere-only
  — a box always returns no contact, deliberately deferred). The playable
  side is the *inside* of the fillet's concave face (the geometry a
  skateboard quarter-pipe is named after and ridden on the inside of): a
  point is governed by a fillet only within the 90-degree sector from
  `sector_start` to `sector_end`, and contact fires as the sphere's surface
  approaches or crosses the fillet's own radius from the inside, pushing
  the sphere back toward the axis — the opposite direction convention from
  `sphere_vs_plane`'s always-away-from-the-plane push.
  `StaticQuarterPipe::between_planes(plane_a, plane_b, radius,
  axis_direction)` derives a fillet's axis/sector automatically from the
  two flat planes it bridges, exact only when both planes' normals and
  `axis_direction` form an orthonormal basis (true for every cardinal
  arena wall's own floor/ceiling seam, not for a diagonal corner wall's).
  `PhysicsWorld` gains `curves: Vec<StaticQuarterPipe>` and a `with_curve`
  builder (mirroring `walls`/`with_wall`), resolved via a new
  `resolve_curve_contact` for the ball and every car (a no-op for cars).
  `solver::resolve_contacts`'s second parameter changed from `&StaticPlane`
  to plain `restitution: f32, friction: f32` — the only two fields it ever
  used — so the same solver path serves a `StaticQuarterPipe` fillet
  exactly as it already served a `StaticPlane`, with no new solver code.
  `arena::standard_curves` builds the 8 fillets (floor-side and
  ceiling-side, for each of the 4 cardinal walls) the standard arena needs,
  using a new uncalibrated placeholder `FILLET_RADIUS` (this port has no
  verified reference for the real transition radius, same status as
  `arena::CORNER_LENGTH`); `PhysicsWorld::standard_arena` now adds these 8
  curves alongside its existing 9 walls. Still not modeled: a car actually
  being deflected by a fillet, fillets at the 4 diagonal corner walls
  (their non-axis-aligned normals don't satisfy `between_planes`'
  orthonormal-basis assumption), and goal cutouts. 15 new unit tests across
  `body.rs`/`collision.rs`/`arena.rs`/`world.rs` in `rb_physics_bullet`
  (168 total): the derived fillet geometry sits exactly `radius` in from
  both bridged planes with correctly-directed, perpendicular unit sector
  vectors and tangent points exactly on each plane; a sphere deep inside a
  fillet has no contact, touching it has zero penetration, pushed past it
  has positive penetration pushing back toward the axis, and outside the
  90-degree sector has no contact regardless of absolute distance; a box
  against a fillet always returns no contact; `standard_curves` returns
  exactly 8 fillets, each sitting radius-in from the floor/ceiling and a
  cardinal wall; `PhysicsWorld::standard_arena` carries exactly 8 curves,
  plus — the real end-to-end proof — a ball resting at ordinary flat-floor
  height within a curve's footprint (already overlapping the fillet's own
  material) gets pushed up off that flat height instead of staying
  embedded, while a car in the exact same position stays completely
  unaffected at its ordinary flat-floor resting height.
- 0.19.0 (2026-08-30): FR-019 added and implemented (modeled arena
  footprint) — a new `arena` module builds Rocket League's real
  standard-arena boundary entirely from FR-013's existing generic
  `StaticPlane`/`with_wall` machinery: no new collision code, since a
  ceiling and a corner-cut wall are each just another flat plane.
  `arena::standard_ground` is the flat floor at `z = 0` (identical to the
  `flat_ground()` test helper this crate has used since v0);
  `arena::standard_walls` returns 9 `StaticPlane`s — 2 side walls
  (`x = ±SIDE_WALL_X`), 2 back walls (`y = ±BACK_WALL_Y`), a ceiling
  (`z = CEILING_Z`), and 4 diagonal corner walls (one per quadrant) cutting
  off the true rectangular corner where a side wall would otherwise meet a
  back wall at 90 degrees, giving the field its real octagonal footprint.
  `SIDE_WALL_X` (4096), `BACK_WALL_Y` (5120), and `CEILING_Z` (2044) are
  commonly-cited community-measured field dimensions, matching the sourcing
  convention `drive::MAX_CAR_SPEED`/`JUMP_SPEED` already established; the
  corner walls' inset distance (`CORNER_LENGTH`, equal along both axes,
  giving a 45-degree cut) is this project's own uncalibrated
  placeholder — this port has no verified reference for the real arena's
  actual corner-wall geometry, which isn't even a single flat plane in the
  real field mesh (it's curved, and blends into ramps this port doesn't
  model either). New `PhysicsWorld::standard_arena` convenience
  constructor wires both into a `PhysicsWorld` in one call — offered
  alongside, not replacing, `PhysicsWorld::new`/`with_wall`'s existing
  ad-hoc-wall capability, which this crate's own tests keep using for
  non-standard scenes. Still not modeled: curved wall-to-floor/
  wall-to-ceiling transitions, goal cutouts in the back walls, and
  disambiguating or blending a car's simultaneous contact with two walls
  at a corner for wall-jump purposes — physical collision resolution
  already handles a car touching two walls at once correctly regardless
  (each wall is resolved independently every step), only the wall-jump
  push-off direction picker still isn't, and FR-019's corner walls make
  that case reachable in the standard arena for the first time (still
  untested here). 10 new unit tests across `arena.rs`/`world.rs` in
  `rb_physics_bullet` (153 total): `standard_walls` returns exactly 9
  planes; the arena's center is on the playable side of every one of them;
  opposing side/back walls share one offset magnitude by construction; a
  point just past a side wall is no longer on the playable side; the
  ceiling bounds from above; a corner wall actually cuts off the true
  rectangular corner; all four corner walls share one offset magnitude,
  plus — the real end-to-end proof — `PhysicsWorld::standard_arena` carries
  exactly 9 walls and the standard ground, a ball shot at the standard
  arena's side wall bounces off it rather than escaping, and a ball fired
  straight at the true rectangular corner is stopped by the diagonal
  corner wall well before its x or y individually reaches either the side
  or back wall's own position.
- 0.18.0 (2026-08-30): FR-018 added and implemented (landing
  auto-orientation assist) — `drive::apply_driven_forces` gains a gentle
  continuous restoring torque, applied while airborne, nudging the car's
  local up axis back toward world up. Real Rocket League triggers this on
  approach to the ground; this port has no raycast or distance query to
  replicate that condition, so the assist instead applies continuously
  whenever airborne, gated on two conditions so it never fights the player:
  no active `pitch`/`roll` air-control input this step, and no fresh
  `ControllerInput.jump` press this step (avoiding a same-step conflict
  between this torque's accumulation into `total_torque` and a
  dodge's/wall-jump-dodge's/double-jump's/flip-cancel's own direct
  `angular_velocity` mutation, both resolved by the same
  `integrate_velocities` call). The correction is
  `up_axis(car).cross(&world_up) * LANDING_AUTO_UPRIGHT_TORQUE`: since both
  vectors are unit length, the cross product's magnitude is already
  proportional to the sine of the car's tilt off level, so a level car
  earns no correction and a heavily tilted one earns a proportionally
  stronger nudge, with no separate angle computation needed. New constant
  `LANDING_AUTO_UPRIGHT_TORQUE` is an uncalibrated placeholder, deliberately
  one full order of magnitude smaller than `AIR_CONTROL_TORQUE` so the
  assist reads as gentle assistance, not full control; this port has no
  public reference for the real assist's actual strength or trigger
  condition either. Known, accepted, unaddressed limitation: a car resting
  exactly upside-down gives an exactly antiparallel `up_axis`/`world_up`
  pair, whose cross product is also zero, so no correction is computed in
  that unlikely exact singularity. No new `PhysicsWorld` state — the assist
  is a pure function of the car's current orientation, input, and ground
  contact, all already in scope. Drive.rs's own test-helper chain never
  calls `integrate::integrate_transform`, so a car's `orientation` never
  actually changes step-to-step there; the new `drive.rs` tests instead set
  a known tilted orientation directly (a new `tilted_car()` helper, calling
  `RigidBody::update_inertia_tensor` afterward for consistency) and check a
  single step's resulting torque, a pattern reusable for any future
  orientation-dependent test there. A pre-existing regression test
  (`world::tests::landing_and_a_new_double_jump_clears_a_stale_dodge_flip_flag_in_a_live_world`) was loosened from an exact `assert_eq!` to a small
  tolerance, since the assist now legitimately nudges angular velocity by a
  tiny amount on the test's intervening neutral-input step — the tolerance
  stays far tighter than a real spurious flip-cancel (which zeroes ~1.5
  rad/s) would need to slip through undetected. 5 new unit tests across
  `drive.rs`/`world.rs` in `rb_physics_bullet` (143 total): a tilted
  airborne car with no input gets a corrective torque; an already-upright
  airborne car gets none; the assist has no effect while grounded; it
  doesn't fire while pitch air control is actively held (isolated via a
  tilt whose own correction axis is orthogonal to full pitch's own torque
  axis); and — the real end-to-end proof — a car tilted 90 degrees with no
  input trends back toward level over 120 steps of a live
  `PhysicsWorld::step` loop (gravity zeroed) rather than staying tilted or
  drifting further away. This closes out the last item that had been
  tracked in `drive.rs`'s own module doc "Not implemented" list since the
  dodge (FR-014) increment — that list is now empty.
- 0.17.0 (2026-08-30): FR-017 added and implemented (wall-jump dodge) —
  the wall jump's own fresh press (FR-013) now checks
  `ControllerInput.pitch`/`roll` against `DODGE_DEADZONE`, the same check
  the ground double jump's press already uses (FR-014): at or above it on
  either axis, a **wall-jump dodge** fires instead of the plain fixed
  push-off — the same outward-plus-upward impulse combined with a
  horizontal `DODGE_SPEED` component and `DODGE_ANGULAR_SPEED` spin
  (identical axis/sign conventions to the ground dodge), also arming
  `dodge_flip_active` so its spin is flip-cancelable (FR-016) exactly like
  a ground dodge's. Below the deadzone, the plain wall jump fires exactly
  as before, still never touching `double_jump_available`. Unlike the
  plain wall jump, the dodge variant *does* consume `double_jump_available`
  — the same resource a ground dodge spends. This is a deliberate
  simplification: since touching a wall unconditionally restores
  `double_jump_available` before this check ever runs, gating the dodge
  variant on it would be vacuous (always true there); having the dodge
  variant spend it instead keeps the existing invariant
  "`dodge_flip_active` is only ever true while `double_jump_available` is
  false" intact with zero changes to flip-cancel's own branch ordering or
  any new landing/wall-touch-clearing logic — this port has no way to
  separately account for "a wall touch refilled the double jump, then the
  wall-jump dodge spent it" versus a genuinely independent wall-dash
  resource, and real Rocket League's precise accounting here isn't public
  to the precision this project would need to model that distinction. No
  new physics constants — reuses `DODGE_SPEED`/`DODGE_ANGULAR_SPEED`/
  `WALL_JUMP_HORIZONTAL_SPEED`/`JUMP_SPEED` outright. Two pre-existing
  tests whose entire premise this requirement deliberately reverses
  (`drive::wall_jump_fires_instead_of_a_dodge_when_touching_a_wall`,
  `world::wall_jump_still_fires_instead_of_a_dodge_when_touching_a_wall`,
  both of which asserted "wall jump always ignores stick input") were
  repurposed in place — not silently deleted — to assert the new
  wall-jump-dodge behavior instead, keeping the same scenario (touching a
  wall with directional stick input) but updating the expected outcome. 6
  new unit tests across `drive.rs`/`world.rs` in `rb_physics_bullet` (138
  total): a wall-jump dodge consumes the double jump unlike a plain wall
  jump; its spin can be flip-cancelled; a below-deadzone stick deflection
  still gives a plain wall jump; opposite stick sign dodges the opposite
  direction; a diagonal (pitch+roll) wall-jump dodge combines both axes,
  plus — the real end-to-end proof — a wall-jump dodge firing in a live
  `PhysicsWorld::step` loop, and a second end-to-end test confirming its
  spin is flip-cancelable there too.
- 0.16.0 (2026-08-30): FR-016 added and implemented (flip-cancel) — a
  dodge's spin (FR-014) can now be canceled early: a further fresh
  `ControllerInput.jump` press while airborne, not touching a wall, with
  `double_jump_available` already spent by that dodge, zeroes
  `RigidBody.angular_velocity` outright instead of leaving the flip to spin
  indefinitely. A new per-car `dodge_flip_active: bool`
  (`PhysicsWorld`'s parallel `car_dodge_flip_active: Vec<bool>`, starting
  `false`) tracks this: the directional-dodge branch sets it `true`; the
  plain-double-jump branch explicitly sets it `false` rather than leaving
  it alone, closing off a real staleness bug this port's own regression
  tests were written to catch and did catch (verified by temporarily
  removing the fix and confirming both the `drive.rs` and `world.rs`
  regression tests fail without it) — without that explicit clear, a
  much-later, completely unrelated plain double jump (after landing from
  the dodge and taking off again) would leave the flag `true`, letting a
  further press spuriously flip-cancel a flip that no longer exists.
  Flip-cancel touches neither the dodge's own linear velocity nor
  `double_jump_available` (already spent by the dodge that set the flag).
  Wall jump keeps its existing priority — checked first in the airborne
  branch, unchanged. This port has no timed flip animation to interrupt (a
  dodge is one instantaneous angular-velocity kick, not a sustained torque
  over a fixed duration), so "mid-flip" here means "any time before
  landing or a wall touch re-arms the double jump," a documented
  simplification of real Rocket League's actual flip-duration window. No
  new physics constants — this is a state-flag-gated zeroing action, not a
  magnitude to calibrate. 6 new unit tests across `drive.rs`/`world.rs` in
  `rb_physics_bullet` (132 total): a second jump press cancels a dodge's
  spin outright and spends the flag; flip-cancel leaves the dodge's own
  translation and `double_jump_available` untouched; a plain double jump
  clears a stale `dodge_flip_active` left over from an earlier dodge so a
  later press can't spuriously cancel nothing; a wall jump still takes
  priority over flip-cancel when touching a wall; an end-to-end test
  confirms a second jump press cancels a dodge's spin in a live
  `PhysicsWorld::step` loop; a second end-to-end regression test confirms
  landing and a later plain double jump clear a stale flag there too, not
  just in `drive.rs` isolation. Deliberately excludes a dodge variant of
  the wall jump and landing auto-orientation assistance — see Non-goals.
- 0.15.0 (2026-08-30): FR-015 added and implemented (variable jump
  height) — the ground jump (FR-010) gains a hold window: continuing to
  hold `ControllerInput.jump` after the fresh press that fires it adds a
  continuous `JUMP_HOLD_ACCELERATION` upward force, for up to
  `JUMP_HOLD_MAX_DURATION` seconds, on top of the press's own fixed
  `JUMP_SPEED` impulse. A new per-car `jump_hold_time_remaining: f32`
  (`PhysicsWorld`'s parallel `car_jump_hold_time_remaining: Vec<f32>`,
  starting `0.0`, threaded into `apply_driven_forces` and
  `drive_and_integrate_velocities` alongside `jump_held`/
  `double_jump_available`) is checked and decremented against the
  *previous* call's value at the very top of `apply_driven_forces`, before
  that same call's own `on_ground`/`jump_pressed` handling can re-arm it to
  `JUMP_HOLD_MAX_DURATION` — so a fresh ground-jump press's own step always
  fires only the plain impulse, and only continued holding into later
  calls earns the extra height. Releasing `jump` zeroes the window
  immediately, even with time left, stopping the extra acceleration right
  away. Scoped to the ground jump alone: firing the double jump, a dodge,
  or the wall jump all require releasing jump first (a fresh press), which
  itself unconditionally zeroes the ground jump's hold window before that
  press's own branch ever runs, so none of the three can be boosted by a
  leftover hold window. `JUMP_HOLD_MAX_DURATION` and
  `JUMP_HOLD_ACCELERATION` are both uncalibrated placeholders — this port
  has no public reference for real Rocket League's actual hold-window
  length or acceleration the way `JUMP_SPEED` does. The pre-existing
  `holding_jump_does_not_repeatedly_relaunch_the_car` regression test's run
  duration was extended (1.5s → 3.0s) since a continuously held jump now
  also earns the variable-height bonus, climbing higher and taking longer
  to land than a bare `JUMP_SPEED` impulse alone. 6 new unit tests across
  `drive.rs` and `world.rs` in `rb_physics_bullet` (126 total): holding
  jump after a ground jump adds more upward velocity than tapping it,
  releasing jump early stops the extra acceleration immediately, the extra
  acceleration stops accruing once the hold window has expired even if
  still held, and a double jump fired after holding the ground jump
  through its whole window still adds exactly one more `JUMP_SPEED` kick
  rather than an extra variable-height boost; an end-to-end test confirms
  a held ground jump reaches a greater peak height than a tapped one in a
  live `PhysicsWorld::step` loop, and a second end-to-end regression test
  confirms the double-jump-unaffected property holds there too, not just
  in `drive.rs` isolation.
- 0.14.0 (2026-08-30): FR-014 added and implemented (dodge) — the double
  jump's own fresh press (see FR-012) now checks `ControllerInput.pitch`/
  `roll` at the moment it fires: at or above a new `DODGE_DEADZONE` on
  either axis, it fires a directional dodge instead of the plain vertical
  double jump — a purely horizontal `DODGE_SPEED` impulse (along
  `forward_axis` for `pitch`, `right_axis` for `roll`) plus an
  instantaneous `DODGE_ANGULAR_SPEED` spin added directly to
  `RigidBody.angular_velocity` (mirroring how `apply_impulse` already
  directly changes `linear_velocity`, rather than `apply_torque`'s
  continuous accumulation, since `RigidBody` has no "angular impulse"
  helper and none was warranted for this one call site) about the
  perpendicular axis (`right_axis` for `pitch`, `forward_axis` for
  `roll`) — reusing air control's own pitch/roll axis and sign
  conventions for direction, though not its `AIR_CONTROL_TORQUE`
  magnitude. Both axes can contribute at once (a diagonal dodge), simply
  summed rather than normalized — a documented simplification, since real
  Rocket League normalizes the stick direction so a diagonal dodge isn't
  faster than an axis-aligned one. Below `DODGE_DEADZONE` on both axes, the
  plain vertical double jump fires exactly as before this requirement;
  either way the press still spends the shared `double_jump_available`
  resource. Wall jump is untouched — it never checks `pitch`/`roll` at
  all, so touching a wall always gets the fixed wall-jump push-off, never
  a dodge. `DODGE_SPEED` and `WALL_JUMP_HORIZONTAL_SPEED` are now `pub`
  (mirroring `JUMP_SPEED`) so `world.rs`'s end-to-end tests can assert
  against, and distinguish between, all three jump variants' distinct
  magnitudes. Deliberately excludes a dodge variant of the wall jump,
  canceling a dodge's rotation early (flip-cancel), landing
  auto-orientation assistance, and variable jump height — see Non-goals.
  10 new unit tests across `drive.rs` and `world.rs` in `rb_physics_bullet`
  (120 total): a forward (pitch) dodge and a lateral (roll) dodge each
  give the expected horizontal velocity and spin, a below-deadzone
  deflection still gives a plain double jump, a dodge spends
  `double_jump_available` the same as a plain double jump, opposite pitch
  dodges the opposite direction, a diagonal (pitch+roll) dodge combines
  both axes, dodge logic has no effect while grounded, and a wall jump
  still fires its own (smaller) push-off instead of a dodge when touching
  a wall; an end-to-end test confirms a car dodges forward with a visible
  flip after a ground jump in a live `PhysicsWorld::step` loop, and a
  second end-to-end test confirms a car touching a wall with directional
  stick input still gets the wall jump, not a dodge.
- 0.13.0 (2026-08-30): FR-013 added and implemented (arena walls and wall
  jump) — `PhysicsWorld` gains `walls: Vec<StaticPlane>` and a `with_wall`
  builder (mirroring `with_car`); `resolve_ground_contact` is renamed
  `resolve_plane_contact` (no behavior change — it already had no
  ground-specific logic, only a ground-specific name) and is now called
  once per wall in addition to the ground, for both the ball and every
  car, so arena walls are real physical geometry every body collides with,
  not just an input-detection hack. On top of that, `drive::apply_driven_forces` gains a `wall_normal: Option<Vec3>` parameter (a per-step fact
  computed by `PhysicsWorld` the same way `on_ground` is, not `&mut` state)
  and a wall jump: a fresh airborne `jump_pressed` press while touching a
  wall fires an impulse combining a new `WALL_JUMP_HORIZONTAL_SPEED`
  (uncalibrated placeholder) outward along the wall's normal with
  `JUMP_SPEED` upward, checked before the double jump so it takes priority
  on that press. Wall contact — independent of whether jump is pressed —
  unconditionally restores `double_jump_available`, the same "any surface
  contact refills your second jump" rule landing already uses, so wall
  jump doesn't cost a player their double jump and has no
  once-per-airborne-period limit of its own (unlike the double jump).
  Deliberately excludes the directional "dodge" a real wall jump can pair
  with, variable jump height, and any modeled arena footprint beyond
  generic flat walls (no octagonal shape, curved transitions, ceiling, or
  multi-wall-corner disambiguation) — see Non-goals. 7 new unit tests
  across `drive.rs` and `world.rs` in `rb_physics_bullet` (110 total):
  wall jump gives outward-and-upward velocity when available, has no
  effect while grounded, takes priority over the double jump without
  consuming it, and mere wall contact restores double-jump availability;
  an end-to-end test confirms a car resting against a wall wall-jumps
  outward and upward in a live `PhysicsWorld::step` loop; a second
  end-to-end test confirms a ball shot at a wall bounces off it instead of
  tunnelling through — the same physical proof already given for
  ball-vs-car, now for the generic plane-collision machinery walls reuse;
  and a regression test confirms a car near, but not touching, an existing
  wall still gets a plain double jump instead of a wall jump.
- 0.12.0 (2026-08-29): FR-012 added and implemented (double jump) —
  `drive::apply_driven_forces` fires one more, identical `JUMP_SPEED`
  instantaneous upward velocity change on a fresh airborne press of
  `ControllerInput.jump`, reusing the ground jump's own rising-edge
  detection (`jump_pressed`) and the `JUMP_SPEED` constant itself rather
  than introducing a second edge-detector or a separately-calibrated
  speed. Gated on a new per-car `double_jump_available` flag: landing
  unconditionally restores it, and a fresh airborne press that spends it
  sets it back to `false` until the next landing, so it fires at most once
  per airborne period regardless of how many more times jump is released
  and re-pressed before then. `PhysicsWorld` gains a parallel
  `car_double_jump_available: Vec<bool>` (starting `true`, kept in
  lockstep with `cars` by `with_car`), threaded through
  `drive_and_integrate_velocities` and `step`'s per-car loop alongside
  `jump_held`. Deliberately excludes the directional "dodge" impulse/torque
  a real double jump pairs with, variable jump height, and wall jump — see
  Non-goals. `JUMP_SPEED` is now `pub` so `world.rs`'s end-to-end tests can
  assert against it directly. 6 new unit tests across `drive.rs` and
  `world.rs` in `rb_physics_bullet`, minus one pre-existing `drive.rs`
  test — `jump_has_no_effect_while_airborne` — removed because this
  feature deliberately supersedes its premise (a fresh airborne jump press
  can now have an effect); net +5, 103 total, including an end-to-end
  test confirming a double jump fired after a ground jump adds a second
  `JUMP_SPEED` kick on top of the first in a live `PhysicsWorld::step` loop
  (gravity zeroed), and a regression test confirming a spent double jump
  doesn't refire mid-air no matter how many more times jump is released and
  re-pressed before landing.
- 0.11.0 (2026-08-29): FR-011 added and implemented (air control) —
  `drive::apply_driven_forces` applies torque about the car's local
  right/up/forward axes, scaled by `ControllerInput.pitch`/`yaw`/`roll`
  (each an `Option<f32>`, `None` treated as zero) times one shared
  `AIR_CONTROL_TORQUE` constant, gated on the car *not* touching the
  ground — the mirror image of throttle/steering/handbrake/jump's
  ground-only gating. Unlike ground steering, not speed-scaled: a car can
  spin from a standing start in the air. New `right_axis` helper completes
  the local (forward, right, up) basis `forward_axis`/`up_axis` already
  provided. `AIR_CONTROL_TORQUE` is a shared, uncalibrated placeholder
  across all three axes — a documented simplification, since real Rocket
  League's pitch/yaw/roll rates differ from each other. Double jump/dodge,
  variable jump height, and wall jump remain explicitly not implemented —
  see Non-goals. 6 new unit tests across `drive.rs` and `world.rs` in
  `rb_physics_bullet` (98 total), including an end-to-end test confirming
  a car with yaw input actually reorients itself mid-air (gravity zeroed)
  in a live `PhysicsWorld::step` loop, and a regression test confirming a
  grounded car stays level despite stray pitch/yaw/roll input.
- 0.10.0 (2026-08-29): FR-010 added and implemented (single ground jump) —
  `drive::apply_driven_forces` applies a fixed `JUMP_SPEED` instantaneous
  upward velocity change (via `RigidBody::apply_impulse`) on the rising
  edge of `ControllerInput.jump` while the car is grounded — a fresh
  press, not merely held; a continued press through the resulting
  airborne period doesn't re-fire it, and releasing then re-pressing while
  still airborne doesn't fire it either (no double jump in this scope).
  `PhysicsWorld` gains a parallel `car_jump_held: Vec<bool>` (starting
  `false`, kept in lockstep with `cars` by `with_car`) carrying the
  rising-edge state across steps, the same pattern `boost_amount` already
  uses. Double jump/dodge, variable jump height (holding for a higher
  jump), wall jump, and air control remain explicitly not implemented —
  see Non-goals. 6 new unit tests across `drive.rs` and `world.rs` in
  `rb_physics_bullet` (92 total), including an end-to-end test confirming
  a car with jump input actually leaves the ground in a live
  `PhysicsWorld::step` loop, and a regression test confirming that holding
  jump for a car's entire flight (never released) lets it land and settle
  instead of being relaunched on touchdown.
- 0.9.0 (2026-08-29): FR-009 added and implemented (handbrake) —
  `drive::apply_driven_forces` temporarily multiplies the car's
  `RigidBody.friction` by a new `HANDBRAKE_FRICTION_MULTIPLIER`
  (uncalibrated placeholder) while `ControllerInput.handbrake` is held and
  the car is grounded, restoring it otherwise — modeling handbrake as a
  temporary grip reduction that lets existing momentum carry the car into
  a slide, reusing the ground-contact solver's existing friction machinery
  rather than a new lateral-slip system (this port has no per-wheel tire
  model to build a real rear-grip-loss mechanic on). `PhysicsWorld` gains
  a parallel `car_base_friction: Vec<f32>`, snapshotted from each car's own
  constructed `friction` by `with_car`, so handbrake restores the car's
  own value rather than a hardcoded default. Jump and air control remain
  explicitly not implemented — see Non-goals. 5 new unit tests across
  `drive.rs` and `world.rs` in `rb_physics_bullet` (86 total), including
  an end-to-end test confirming a car already sliding sideways retains
  more of that slide under handbrake's reduced friction than under normal
  grip in a live `PhysicsWorld::step` loop, and a regression test
  confirming handbrake restores a car's own non-default base friction, not
  a crate-wide constant.
- 0.8.0 (2026-08-29): FR-008 added and implemented (boost) —
  `drive::apply_driven_forces` gains a boost force: a flat forward force
  (`BOOST_ACCELERATION * mass`, not speed-tapered like throttle) along the
  car's local forward axis, applied whenever `ControllerInput.boost` is set
  and the car has boost remaining, capped at `MAX_CAR_SPEED`. Unlike
  throttle and steering, boost is *not* gated on ground contact — it works
  identically airborne, matching real Rocket League's rocket-based (not
  wheel-based) boost. `PhysicsWorld` gains a parallel `car_boost: Vec<f32>`
  (kept in lockstep with `cars` by `with_car`, initialized to a full tank —
  `drive::MAX_BOOST`) and `set_car_boost` to set it directly; holding boost
  drains the tank at `BOOST_CONSUMPTION_RATE` per second whenever held,
  even once the forward force itself stops applying at `MAX_CAR_SPEED`
  (matching real Rocket League's "holding boost drains fuel regardless");
  the tank clamps at zero. `frame()` now reports each car's live
  `boost_amount` instead of a hardcoded `0.0`. Jump, air control, and
  handbrake remain explicitly not implemented — see Non-goals. 6 new unit
  tests across `drive.rs` and `world.rs` in `rb_physics_bullet` (81 total),
  including an end-to-end test confirming a car with boost input actually
  drives forward while airborne (gravity zeroed) in a live
  `PhysicsWorld::step` loop, and a regression test confirming a new car
  starts with a full boost tank.
- 0.7.0 (2026-08-29): FR-007 added and implemented (ground throttle and
  steering only) — new `drive` module, `apply_driven_forces` couples
  `rb_domain::ControllerInput` into a throttle force (along the car's
  local forward axis, capped at `MAX_CAR_SPEED`) and a steering torque
  (about the car's local up axis, scaled by current speed), both gated on
  ground contact. `PhysicsWorld` gains `car_inputs: Vec<ControllerInput>`
  (kept in lockstep with `cars` by `with_car`, defaulting to neutral) and
  `set_car_input` to update a car's persistent input; `step` computes each
  car's ground-contact state up front and applies its driven forces
  alongside gravity, before integrating velocities; `frame()` now reports
  each car's actual input instead of always `None`. Boost, jump, air
  control, and handbrake remain explicitly not implemented — see
  Non-goals. 10 new unit tests in `rb_physics_bullet` (75 total),
  including an end-to-end test confirming a car with throttle input
  actually drives forward across the ground in a live `PhysicsWorld::step`
  loop, and a regression test confirming a car with no input set behaves
  exactly as before this requirement existed.
- 0.6.0 (2026-08-29): Multi-car `PhysicsWorld` support — `car: Option<RigidBody>` is replaced by `cars: Vec<RigidBody>` (a breaking
  field rename); `with_car` now appends, so it's callable any number of
  times to build a scene with any number of cars. `PhysicsWorld::step`
  resolves every car's ground contact, every ball-vs-car pair, and every
  car-vs-car pair (via `collision::box_vs_box`, now running for real in a
  live scene instead of only under a unit test) each step, one pair at a
  time; `frame()` assigns each car's `player_id` as its index in `cars`.
  This completes `RB-PHYSICS-001-FR-006` — car-vs-car collision detection
  (0.5.0) is now actually wired up, not just unit-tested in isolation. 3
  new unit tests in `rb_physics_bullet` (65 total), including an
  end-to-end test confirming two cars shot head-on at each other in a live
  `PhysicsWorld` actually bounce off instead of tunnelling through.
- 0.5.0 (2026-08-29): FR-006 added and implemented (detection only) —
  `collision::box_vs_box`, a 15-axis separating-axis test (3+3 face axes,
  9 edge-pair axes) between two oriented boxes, producing a clipped face
  manifold (`face_contact`, 0-4 points) or a single edge-edge point
  (`edge_contact`, via a standard closest-point-between-segments
  construction). `collision::contact_between` is generalized to
  `contacts_between` (returning `Vec<Contact>` uniformly, since box-vs-box
  can now return a manifold where sphere-vs-box always returned at most
  one point), and `solver::resolve_contact_between` is generalized to
  `resolve_contacts_between` (a manifold, mirroring `resolve_contacts`'
  existing multi-contact structure) to match. `box_vs_box` has no live
  caller through `PhysicsWorld` yet — this scope still has exactly one
  car, so multi-car wiring is deliberate, tracked follow-up work, not this
  change's scope (see Non-goals). 4 new unit tests in `rb_physics_bullet`
  (62 total).
- 0.4.0 (2026-08-28): FR-004 completed — sphere-vs-box (ball-vs-car)
  contact generation (`collision::sphere_vs_box`/`contact_between`,
  handling both the ordinary exterior case and a deep-penetration interior
  case) and a two-dynamic-body sequential-impulse solver path
  (`solver::resolve_contact_between`, generalizing the existing
  body-vs-static-plane rows to carry both bodies' mass/inertia
  contributions). `PhysicsWorld::step` was restructured into Bullet's
  actual staged pipeline (integrate every body's velocity, then resolve
  every contact, then integrate every body's transform) so ball-vs-car
  resolution sees the same pre-integration state ground contacts do.
  `rb_domain::Quat` gains `conjugate` (needed to transform a world point
  into the box's local frame). Box-vs-box collision remains explicitly not
  implemented — see Non-goals. 11 new unit tests in `rb_physics_bullet`
  (58 total), 1 in `rb_domain`.
- 0.3.0 (2026-08-28): FR-004 substantially implemented — box-shaped
  bodies via a unified `RigidBody`/`Shape` design (matching Bullet's own
  rigid-body-plus-collision-shape architecture), a general 3x3 inverse
  inertia tensor (`Mat3`, shared by sphere and box), analytic box-vs-plane
  contact generation (1-4 points), and multi-contact manifold resolution
  in the solver. `PhysicsWorld` gains an optional car body stepped
  alongside the ball. Box-vs-sphere collision remains explicitly not
  implemented — see Non-goals. 21 new unit tests (47 total).
- 0.2.0 (2026-08-28): v0 implemented — sphere-vs-static-plane rigid body
  integration and sequential-impulse contact solver, ported from Bullet3
  per ADR-0004. Resolves the "build-vs-integrate" framing from this spec's
  0.1.0 open questions in favor of a direct source port.
- 0.1.0 (2026-08-28): Placeholder created at bootstrap; full spec deferred
  to Phase 1 start.
