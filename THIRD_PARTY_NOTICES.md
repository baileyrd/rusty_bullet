# Third-Party Notices

## Bullet Physics (Bullet3) — algorithm port

`crates/rb_physics_bullet` is a from-scratch Rust **translation of specific
algorithms** from [Bullet Physics (bullet3)](https://github.com/bulletphysics/bullet3),
not a binding, a vendored copy, or a build of Bullet's C++ source. No
Bullet3 source file is copied, compiled, or linked into this repository —
the Rust code was written independently, informed by reading the referenced
functions to reproduce their math and control flow faithfully.

This notice exists to satisfy zlib license condition 1 (the origin of this
software must not be misrepresented) and to comply with condition 2
(altered source versions must be plainly marked as such) for the portions
of `rb_physics_bullet` derived from Bullet3.

### Bullet3's license (as it applies to the original work this ports)

```
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from
the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software in
   a product, an acknowledgment in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
```

Full text: <https://github.com/bulletphysics/bullet3/blob/main/LICENSE.txt>.

### What was ported, from where

Each `rb_physics_bullet` module names its Bullet3 source file(s) in its
module doc comment. Summary, with the exact functions this port's
algorithms are derived from:

| `rb_physics_bullet` module | Bullet3 source | Function(s) ported |
|---|---|---|
| `integrate::apply_gravity`, `apply_damping`, `integrate_velocities` | `src/BulletDynamics/Dynamics/btRigidBody.cpp` | `applyGravity`, `applyDamping`, `integrateVelocities` |
| `integrate::integrate_transform` | `src/LinearMath/btTransformUtil.h` | `btTransformUtil::integrateTransform` |
| `solver::setup_rows`, `resolve_row` | `src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp` | `setupContactConstraint`, `setupFrictionConstraint`, `resolveSingleConstraintRowGeneric`, `resolveSingleConstraintRowLowerLimit`, `restitutionCurve` |
| `solver::plane_space` | `src/LinearMath/btVector3.h` | `btPlaneSpace1` |
| `body::RigidBody::update_inertia_tensor` | `src/BulletDynamics/Dynamics/btRigidBody.cpp` | `btRigidBody::updateInertiaTensor` |
| `body::Shape::local_inertia` | `src/BulletCollision/CollisionShapes/btSphereShape.h`, `btBoxShape.h` | `btSphereShape::calculateLocalInertia`, `btBoxShape::calculateLocalInertia` |
| `mat3::Mat3::from_quat` | `src/LinearMath/btMatrix3x3.h` | `btMatrix3x3::setRotation` |
| `mat3::Mat3::scaled_columns` | `src/LinearMath/btMatrix3x3.h` | `btMatrix3x3::scaled` |
| `state::Quat::conjugate` (in `rb_domain`, used by `collision::sphere_vs_box`) | `src/LinearMath/btQuaternion.h` | `btQuaternion::inverse` |
| `collision::sphere_vs_box` | `src/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp` (closest-point-on-box structure) | closed-form equivalent of the closest-point query `btBoxSphereCollisionAlgorithm` runs via general support mapping — see the function's own doc comment for why a box's closest point to an external point reduces to a plain per-axis clamp |
| `solver::setup_two_body_rows`, `resolve_two_body_row`, `resolve_contacts_between` | `src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp` | the same `setupContactConstraint`/`resolveSingleConstraintRowGeneric` family as `solver::setup_rows`/`resolve_row`, generalized to carry both bodies' mass/inertia contributions (Bullet's solver always does; `resolve_contacts`' one-body-only version only worked because a static plane's side is always zero) |
| `collision::box_vs_box` (SAT axis test) | `src/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp` | `btBoxBoxDetector::dBoxBox`'s separating-axis loop (15 candidate axes: 3+3 face normals, 9 edge-pair cross products) — itself derived from ODE's `dBoxBox`, credited in Bullet3's own source comments |
| `collision::face_contact` | `src/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp` | a box-specific closed form of `dBoxBox`'s incident-face-vs-reference-face polygon clipping, direct rather than a line-for-line port of its (ODE-derived) general clipping code — see the function's own doc comment |
| `collision::edge_contact`'s closest-point step | *(not from Bullet3)* | standard closest-point-between-two-segments construction (e.g. Ericson, *Real-Time Collision Detection*, §5.1.9), used for `dBoxBox`'s edge-edge contact case |
| `wheels::compute_friction_impulses`'s side impulse | `src/BulletDynamics/ConstraintSolver/btContactConstraint.cpp`, `btJacobianEntry.h` | `resolveSingleBilateral` (its `contactDamping = 0.2` velocity impulse through the contact Jacobian's diagonal) |
| `wheels` (raycast-vehicle structure: wheel mounts, the per-wheel ray, `suspensionLength`, `clippedInvContactDotSuspension`, `suspensionRelativeVelocity`, the spring-damper `updateSuspension`, the friction-impulse split into a side impulse and a rolling term) | `src/BulletDynamics/Vehicle/btRaycastVehicle.cpp` | `btRaycastVehicle::rayCast`, `updateSuspension`, `updateFriction`, `updateWheelTransform` — as modified by RocketSim's `btVehicleRL` (next section), which is the form actually ported |

Deliberate, documented deviations from the original algorithms (no split
impulse, no SIMD, no warm-starting/sleeping, `box_vs_box`'s clipping and
edge-edge steps implemented directly rather than as a line-for-line port of
`dBoxBox`'s own (ODE-derived) code, a different restitution/friction
combine mode) are noted in the relevant Rust module's doc comments and in
`RB-PHYSICS-001`/ADR-0004 — this port does not claim behavioral equivalence
with upstream Bullet3, only that its core integration and contact-solving
math is derived from it.

### Not from Bullet3

Rocket League's own modified/forked Bullet integration is **not** available
publicly and is not used, referenced, or reverse-engineered by this port —
see `docs/architecture/SYSTEM-ARCHITECTURE.md`'s "Legal and IP boundary".
The Bullet material in this file concerns the public, zlib-licensed
upstream Bullet3 project only; the wheel model below is ported from
RocketSim, a public, MIT-licensed reimplementation.

## RocketSim — algorithm port

`crates/rb_physics_bullet/src/wheels.rs` (`RB-PHYSICS-001-FR-082`) is a
from-scratch Rust translation of the wheel, suspension, and tire-friction
algorithms of [RocketSim](https://github.com/ZealanL/RocketSim) — its
`btVehicleRL` (a modified copy of Bullet3's `btRaycastVehicle`, shipped
inside RocketSim under RocketSim's license) and the wheel half of
`Car::_UpdateWheels` — together with the constants those algorithms take
from RocketSim's `RLConst.h` and `CarConfig.cpp`;
`crates/rb_physics_bullet/src/hit.rs` (`RB-PHYSICS-001-FR-083` finding
5) likewise translates `Ball::_OnHit`'s car-ball extra impulse. Earlier requirements
(`RB-PHYSICS-001-FR-056` onward) adopted individual constants and curve
shapes from the same files; this is the first port of RocketSim *control
flow*. No RocketSim source file is copied, compiled, or linked into this
repository — the Rust code was written independently, informed by reading
the referenced functions to reproduce their math and control flow
faithfully.

### RocketSim's license

```
MIT License

Copyright (c) 2022 ZealanL

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

Full text: <https://github.com/ZealanL/RocketSim/blob/main/LICENSE>.

### What was ported, from where

| `rb_physics_bullet` module | RocketSim source | Function(s) / data ported |
|---|---|---|
| `wheels::WHEELS` and the `SUSPENSION_*`, `WHEELS_DAMPING_*`, `MAX_SUSPENSION_TRAVEL`, `SUSPENSION_SUBTRACTION`, `THROTTLE_TORQUE_AMOUNT`, `BRAKE_TORQUE_AMOUNT`, `STOPPING_FORWARD_VEL`, `COASTING_BRAKE_FACTOR`, `BRAKING_NO_THROTTLE_SPEED_THRESH`, `THROTTLE_DEADZONE`, `STEER_ANGLE_FROM_SPEED_CURVE`, `POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE` constants | `src/RLConst.h`, `src/CarConfig.cpp`, `src/Sim/Car/Car.cpp` (`_BulletSetup`) | the Octane wheel mounts, radii, and suspension rests; the `BTVehicle` namespace; the drive/brake/steer constants and curves |
| `wheels::raycast_wheels` | `src/Sim/btVehicleRL/btVehicleRL.cpp` | `btVehicleRL::rayCast`, `updateWheelTransformsWS` |
| `wheels::apply_suspension_impulses` | `src/Sim/btVehicleRL/btVehicleRL.cpp` | `btVehicleRL::updateSuspension` and its impulse loop |
| `wheels::compute_friction_impulses`, `apply_friction_impulses` | `src/Sim/btVehicleRL/btVehicleRL.cpp` | `btVehicleRL::calcFrictionImpulses` (`ROLLING_FRICTION_SCALE_MAGIC` included), `applyFrictionImpulses` |
| `wheels::upwards_dir_from_contacts` | `src/Sim/btVehicleRL/btVehicleRL.cpp` | `btVehicleRL::getUpwardsDirFromWheelContacts` |
| `wheels::update_wheels` | `src/Sim/Car/Car.cpp` | `Car::_UpdateWheels` (throttle/brake/coast logic, steer angle, friction factors, sticky force) and `_PreTickUpdate`'s `isOnGround` rule |
| `wheels::piecewise_linear` | `src/Sim/MutatorConfig` / `src/Math/LinearPieceCurve.cpp` | `LinearPieceCurve::GetOutput` |
| `PhysicsWorld::step`'s wheel ordering | `src/Sim/Car/Car.cpp` | `Car::_PreTickUpdate`'s `updateVehicleFirst` → `_UpdateWheels` → jump/air → `updateVehicleSecond` order |
| `hit::ball_car_extra_impulse` and the `BALL_CAR_EXTRA_IMPULSE_*` constants | `src/Sim/Ball/Ball.cpp`, `src/RLConst.h` | `Ball::_OnHit`'s `hitDir` flattening, forward bias, `relSpeed` cap, and `BALL_CAR_EXTRA_IMPULSE_FACTOR_CURVE`; `_velocityImpulseCache` applied in `_FinishPhysicsTick` |
| `PhysicsWorld::step`'s once-per-two-ticks hit cooldown | `src/Sim/Ball/Ball.cpp` | `Ball::_OnHit`'s `tickCount > lastHitTick + 1` guard |
| `solver::PairMaterial` and the `CARBALL_*`/`CARCAR_*` constants | `src/Sim/Arena/Arena.cpp`, `src/Sim/Ball/Ball.cpp`, `src/RLConst.h` | the contact callback's per-pair `m_combinedFriction`/`m_combinedRestitution` overrides (`CARCAR` set in `Arena.cpp`, `CARBALL` returned by `Ball::_OnHit`) |

Deliberate, documented deviations (rays against the scene's flat planes
only, the auto-roll not yet ported, the analog
`handbrakeVal` and the slip-driven friction curves not yet ported) are
noted in `wheels.rs`'s module doc comment and in `RB-PHYSICS-001-FR-082`'s
own step sequencing.
