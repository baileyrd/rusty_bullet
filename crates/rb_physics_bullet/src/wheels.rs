//! The car's wheels: four raycast wheels on spring-damper suspension, the
//! sticky force that presses the car into whatever its wheels touch, and
//! the tire-friction impulses that drive, brake, and grip the ground —
//! `RB-PHYSICS-001-FR-082`, a port of RocketSim's `btVehicleRL` (itself a
//! modified `btRaycastVehicle`) and the wheel half of `Car::_UpdateWheels`.
//!
//! Step (a) of that entry ports the *mechanism*: the mounts, the raycast,
//! the suspension, the sticky force, and the per-wheel friction impulses
//! with their engine/brake/coast logic. The *curves* that shape those
//! impulses — the speed-to-steer-angle curve, the analog handbrake with its
//! lateral/longitudinal factor curves, the slip-driven lateral friction
//! curve, and the non-sticky curve — are here too since step (b): each
//! touching wheel's lateral factor follows `LAT_FRICTION_CURVE` of its
//! mount's slip ratio, the handbrake's two factor curves blend in by the
//! analog `handbrakeVal` (ramped `5`/s up, `2`/s down), the longitudinal
//! factor is `1` unless powersliding, and both scale down by the
//! non-sticky curve of the contact normal whenever no throttle is held.
//! The steer-angle curve itself is here
//! in step (a): the real capture's grounded ticks yaw faster and faster
//! under full steer, and unsteered tires fight any torque that tries to
//! imitate that, so steering had to become the real mechanism the moment
//! the tires did. Raycasts see the flat planes of the scene (the ground
//! and the walls); the curved fillets, goal walls, and bounded walls and
//! the auto-roll are step (c). The `extraPushback` hard stop is here in
//! step (a): read with `SUSPENSION_SUBTRACTION` in its real units it is
//! what stops a hard landing, not a rest-height term.
//!
//! # Per-tick order (RocketSim `Car::_PreTickUpdate`)
//!
//! 1. [`raycast_wheels`] — each wheel's mount, ray, contact, suspension
//!    length, and suspension relative velocity, from the car's
//!    start-of-step transform (`btVehicleRL::updateVehicleFirst`, the
//!    `rayCast` half).
//! 2. [`compute_friction_impulses`] — each touching wheel's friction
//!    impulse from the car's start-of-step velocity and the engine, brake,
//!    steer, and friction factors the *previous* tick's [`update_wheels`]
//!    left on it (`calcFrictionImpulses`; the one-tick lag is RocketSim's,
//!    since `updateVehicleFirst` runs before `_UpdateWheels`).
//! 3. `wheels_in_contact >= 3` is the car's grounded state
//!    (`isOnGround`) for `drive::apply_driven_forces`.
//! 4. [`update_wheels`] — this tick's engine force, brake force, and
//!    friction factors from the input, plus the sticky force
//!    (`_UpdateWheels`).
//! 5. `drive::apply_driven_forces` — jump, air control, flip, boost.
//! 6. [`apply_suspension_impulses`] then [`apply_friction_impulses`]
//!    (`updateVehicleSecond`), as velocity impulses before the contact
//!    solve sees the body — exactly where RocketSim applies them, before
//!    Bullet's `stepSimulation`.
//!
//! # Units
//!
//! RocketSim runs these in Bullet units (`uu / 50`) with the same `180`
//! mass. Every stiffness, damping, force-scale, and friction number here
//! is used unchanged in uu: each enters the body as `a = k · x / m`, and
//! both `x` and `a` scale by the same `50`, so the numbers are unit-
//! invariant as accelerations per unit of length or velocity per unit
//! mass. The two force amounts that RocketSim declares in uu-mass units
//! (`THROTTLE_TORQUE_AMOUNT`, `BRAKE_TORQUE_AMOUNT`) are likewise used as
//! declared.

use crate::body::{RigidBody, StaticPlane, CAR_MASS};
use crate::collision;
use crate::drive;
use rb_domain::Vec3;

/// One wheel's mount on the Octane chassis (RocketSim `CarConfig.cpp`,
/// `_BulletSetup`'s `addWheel`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelMount {
    /// The ray's start, in the car's frame (`FRONT/BACK_WHEELS_OFFSET`).
    pub mount: Vec3,
    /// `FRONT/BACK_WHEEL_RADS`.
    pub radius: f32,
    /// The spring's rest length: RocketSim subtracts `MAX_SUSPENSION_TRAVEL`
    /// from the declared `FRONT/BACK_WHEEL_SUS_REST` before `addWheel`, so
    /// this is `26.755` / `25.055` uu, not the declared `38.755` /
    /// `37.055`.
    pub rest_length: f32,
    /// `SUSPENSION_FORCE_SCALE_FRONT/BACK`.
    pub force_scale: f32,
    /// Front wheels steer; back wheels don't.
    pub is_front: bool,
}

/// `RLConst::BTVehicle::SUSPENSION_STIFFNESS`.
pub const SUSPENSION_STIFFNESS: f32 = 500.0;
/// `RLConst::BTVehicle::WHEELS_DAMPING_COMPRESSION` — the damping while
/// the spring is compressing (suspension relative velocity negative).
pub const WHEELS_DAMPING_COMPRESSION: f32 = 25.0;
/// `RLConst::BTVehicle::WHEELS_DAMPING_RELAXATION` — the damping while
/// the spring is extending.
pub const WHEELS_DAMPING_RELAXATION: f32 = 40.0;
/// `RLConst::BTVehicle::MAX_SUSPENSION_TRAVEL` — how far past its rest
/// length a spring can compress or extend, and how far past rest the ray
/// still looks for the ground.
pub const MAX_SUSPENSION_TRAVEL: f32 = 12.0;
/// Bullet units to uu: RocketSim runs Bullet at `uu / 50`.
pub const BT_TO_UU: f32 = 50.0;
/// `RLConst::BTVehicle::SUSPENSION_SUBTRACTION` — `0.05`, shaved off every
/// ray and off the pushback threshold. RocketSim subtracts it from lengths
/// that are already in Bullet units, so it is `0.05` *Bullet* units:
/// `2.5` uu. (The `FR-082` scoping read it as `0.05` uu and derived a
/// `51.2` uu ray and a pushback that engaged at rest from that; both are
/// corrected here.)
pub const SUSPENSION_SUBTRACTION: f32 = 0.05 * BT_TO_UU;
/// Bullet's `btContactSolverInfo::m_erp` (`0.2`), the error-reduction
/// fraction the pushback's positional term uses
/// (`resolveSingleCollision`, `positionalError = erp * -distance /
/// timeStep`).
pub const PUSHBACK_ERP: f32 = 0.2;
/// `RLConst::BTVehicle::SUSPENSION_FORCE_SCALE_FRONT` (`36 - 1/4`).
pub const SUSPENSION_FORCE_SCALE_FRONT: f32 = 36.0 - 1.0 / 4.0;
/// `RLConst::BTVehicle::SUSPENSION_FORCE_SCALE_BACK` (`54 + 1/4 + 1.5/100`).
pub const SUSPENSION_FORCE_SCALE_BACK: f32 = 54.0 + 1.0 / 4.0 + 1.5 / 100.0;

/// `FRONT_WHEELS_OFFSET[OCTANE]` — `(51.25, 25.90, 20.755)`; the left wheel
/// mirrors `y`.
pub const FRONT_WHEEL_OFFSET: Vec3 = Vec3::new(51.25, 25.90, 20.755);
/// `BACK_WHEELS_OFFSET[OCTANE]` — `(-33.75, 29.50, 20.755)`.
pub const BACK_WHEEL_OFFSET: Vec3 = Vec3::new(-33.75, 29.50, 20.755);
/// `FRONT_WHEEL_RADS[OCTANE]`.
pub const FRONT_WHEEL_RADIUS: f32 = 12.5;
/// `BACK_WHEEL_RADS[OCTANE]`.
pub const BACK_WHEEL_RADIUS: f32 = 15.0;
/// `FRONT_WHEEL_SUS_REST[OCTANE]`, as declared (before the travel is
/// subtracted).
pub const FRONT_WHEEL_SUSPENSION_REST: f32 = 38.755;
/// `BACK_WHEEL_SUS_REST[OCTANE]`, as declared.
pub const BACK_WHEEL_SUSPENSION_REST: f32 = 37.055;

/// The four Octane wheels in RocketSim's order: front right, front left,
/// back right, back left (`i < 2` is front, `i % 2` is left with `y`
/// mirrored). `+y` is the car's right.
pub const WHEELS: [WheelMount; 4] = [
    WheelMount {
        mount: FRONT_WHEEL_OFFSET,
        radius: FRONT_WHEEL_RADIUS,
        rest_length: FRONT_WHEEL_SUSPENSION_REST - MAX_SUSPENSION_TRAVEL,
        force_scale: SUSPENSION_FORCE_SCALE_FRONT,
        is_front: true,
    },
    WheelMount {
        mount: Vec3::new(
            FRONT_WHEEL_OFFSET.x,
            -FRONT_WHEEL_OFFSET.y,
            FRONT_WHEEL_OFFSET.z,
        ),
        radius: FRONT_WHEEL_RADIUS,
        rest_length: FRONT_WHEEL_SUSPENSION_REST - MAX_SUSPENSION_TRAVEL,
        force_scale: SUSPENSION_FORCE_SCALE_FRONT,
        is_front: true,
    },
    WheelMount {
        mount: BACK_WHEEL_OFFSET,
        radius: BACK_WHEEL_RADIUS,
        rest_length: BACK_WHEEL_SUSPENSION_REST - MAX_SUSPENSION_TRAVEL,
        force_scale: SUSPENSION_FORCE_SCALE_BACK,
        is_front: false,
    },
    WheelMount {
        mount: Vec3::new(
            BACK_WHEEL_OFFSET.x,
            -BACK_WHEEL_OFFSET.y,
            BACK_WHEEL_OFFSET.z,
        ),
        radius: BACK_WHEEL_RADIUS,
        rest_length: BACK_WHEEL_SUSPENSION_REST - MAX_SUSPENSION_TRAVEL,
        force_scale: SUSPENSION_FORCE_SCALE_BACK,
        is_front: false,
    },
];

/// Three or more wheels touching is "on the ground" (RocketSim
/// `isOnGround = numWheelsInContact >= 3`): the jump's precondition, the
/// end of any flip, and the switch between the ground and air branches
/// of `drive::apply_driven_forces`.
pub const WHEELS_FOR_GROUNDED: usize = 3;

/// `RLConst::THROTTLE_TORQUE_AMOUNT = CAR_MASS_BT * 400` — the engine
/// force per wheel at full throttle and full `drive_speed_taper`; over
/// four wheels it is the `1600` uu/s² this port's throttle always had.
pub const THROTTLE_TORQUE_AMOUNT: f32 = CAR_MASS * 400.0;
/// `RLConst::BRAKE_TORQUE_AMOUNT = CAR_MASS_BT * (14.25 + 1/3)` — the
/// brake's clamp per wheel; through `friction_scale` and over four wheels
/// it is the real `3500` uu/s² brake deceleration.
pub const BRAKE_TORQUE_AMOUNT: f32 = CAR_MASS * (14.25 + 1.0 / 3.0);
/// `RLConst::STOPPING_FORWARD_VEL` — below this forward speed a coasting
/// car brakes fully, and throttle against the direction of travel only
/// counts as braking above it.
pub const STOPPING_FORWARD_VEL: f32 = 25.0;
/// `RLConst::COASTING_BRAKE_FACTOR` — the brake applied while coasting
/// (no throttle) above `STOPPING_FORWARD_VEL`.
pub const COASTING_BRAKE_FACTOR: f32 = 0.15;
/// `RLConst::BRAKING_NO_THROTTLE_SPEED_THRESH` — while braking above this
/// speed the engine is cut ("we can't throttle and brake at the same
/// time, even backwards").
pub const BRAKING_NO_THROTTLE_SPEED_THRESH: f32 = 0.01;
/// `RLConst::THROTTLE_DEADZONE`.
pub const THROTTLE_DEADZONE: f32 = 0.001;
/// `btVehicleRL::calcFrictionImpulses`'s `ROLLING_FRICTION_SCALE_MAGIC` —
/// the brake force per unit of forward contact velocity below the brake's
/// clamp (RocketSim: "No idea where this number comes from"). Its
/// proportional band ends at `BRAKE_TORQUE_AMOUNT / 113.74 ≈ 23` uu/s.
pub const ROLLING_FRICTION_SCALE_MAGIC: f32 = 113.73963;
/// Bullet's `resolveSingleBilateral` `contactDamping` — the fraction of
/// the lateral contact velocity one wheel's side impulse removes.
pub const BILATERAL_CONTACT_DAMPING: f32 = 0.2;
/// `calcFrictionImpulses`'s `frictionScale = mass / 3`.
pub const FRICTION_SCALE_DIVISOR: f32 = 3.0;
/// `RLConst::STEER_ANGLE_FROM_SPEED_CURVE` — the front wheels' maximum
/// steer angle in radians against the car's forward speed, largest at a
/// standstill (`0.534` rad, `30.6°`) and shrinking with speed (`0.035` rad
/// at `3000` uu/s) — `RB-PHYSICS-001-FR-065`'s curve, now the steering
/// mechanism: the angled front wheels' lateral grip turns the car.
pub const STEER_ANGLE_FROM_SPEED_CURVE: [(f32, f32); 6] = [
    (0.0, 0.53356),
    (500.0, 0.31930),
    (1000.0, 0.18203),
    (1500.0, 0.10570),
    (1750.0, 0.08507),
    (3000.0, 0.03454),
];
/// `RLConst::POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE` — the steer angle
/// curve the handbrake blends toward.
pub const POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE: [(f32, f32); 2] =
    [(0.0, 0.39235), (2500.0, 0.12610)];
/// `RLConst::POWERSLIDE_RISE_RATE`: the analog handbrake value climbs at
/// this rate per second while the handbrake is held (`0 → 1` in `0.2` s).
pub const POWERSLIDE_RISE_RATE: f32 = 5.0;
/// `RLConst::POWERSLIDE_FALL_RATE`: and falls at this rate per second once
/// released (`1 → 0` in `0.5` s).
pub const POWERSLIDE_FALL_RATE: f32 = 2.0;
/// `_UpdateWheels`' lateral-slip threshold: below this much sideways
/// contact velocity (uu/s) at the wheel's mount, the friction curves read
/// at zero slip.
pub const LATERAL_SLIP_THRESHOLD: f32 = 5.0;
/// `RLConst::LAT_FRICTION_CURVE` — the lateral friction factor against
/// the slip ratio `lateral / (longitudinal + lateral)` of the mount's
/// velocity: full grip rolling straight, a fifth sliding straight
/// sideways. `RB-PHYSICS-001-FR-066`'s slip-driven curve.
pub const LAT_FRICTION_CURVE: [(f32, f32); 2] = [(0.0, 1.0), (1.0, 0.2)];
/// `RLConst::LONG_FRICTION_CURVE` — empty in RocketSim, so
/// `LinearPieceCurve::GetOutput`'s default of `1` at every slip ratio.
pub const LONG_FRICTION_CURVE: [(f32, f32); 0] = [];
/// `RLConst::HANDBRAKE_LAT_FRICTION_FACTOR_CURVE` — a single point, so a
/// constant `0.1` at every slip ratio: the handbrake cuts lateral grip to
/// a tenth (`RB-PHYSICS-001-FR-066`'s finding), blended in by the analog
/// handbrake value.
pub const HANDBRAKE_LAT_FRICTION_FACTOR_CURVE: [(f32, f32); 1] = [(0.0, 0.1)];
/// `RLConst::HANDBRAKE_LONG_FRICTION_FACTOR_CURVE` — the handbrake's
/// longitudinal factor, `0.5` rolling straight rising to `0.9` sliding
/// straight sideways; without the handbrake longitudinal friction is `1`.
pub const HANDBRAKE_LONG_FRICTION_FACTOR_CURVE: [(f32, f32); 2] = [(0.0, 0.5), (1.0, 0.9)];
/// `RLConst::NON_STICKY_FRICTION_FACTOR_CURVE` — with no throttle held,
/// both friction factors scale by this curve of the contact normal's `z`:
/// `1` on the floor, `0.5` at `45°`, a tenth on a vertical wall, so a
/// coasting car slides down a wall.
pub const NON_STICKY_FRICTION_FACTOR_CURVE: [(f32, f32); 3] =
    [(0.0, 0.1), (0.7075, 0.5), (1.0, 1.0)];
/// The sticky force's base scale: half a g into the surface whenever any
/// wheel touches the world (`_UpdateWheels`, `stickyForceScale = 0.5`).
pub const STICKY_FORCE_BASE_SCALE: f32 = 0.5;
/// RocketSim rounds tiny forward contact velocities to zero in the brake
/// below this tick rate, to hide stuttering; at this port's `120` Hz the
/// branch is never taken, but it is ported for fidelity at other rates.
const ROLLING_FRICTION_ROUNDING_TICK_RATE: f32 = 80.0;

/// One wheel's state. The geometric fields are rewritten every step by
/// [`raycast_wheels`]; the drive fields are written by [`update_wheels`]
/// and consumed by the *next* step's [`compute_friction_impulses`], one
/// tick later, as in RocketSim.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelState {
    /// The ray hit something within its length (`m_isInContact`).
    pub in_contact: bool,
    /// Where the ray hit, in world space (`m_contactPointWS`); the ray's
    /// end when it hit nothing.
    pub contact_point: Vec3,
    /// The surface normal at the hit (`m_contactNormalWS`); the car's up
    /// when it hit nothing.
    pub contact_normal: Vec3,
    /// The spring's current length (`m_suspensionLength`), clamped to
    /// `rest ± MAX_SUSPENSION_TRAVEL`; `rest + travel` when airborne.
    pub suspension_length: f32,
    /// The contact point's velocity along the normal, divided by
    /// `normal · up` (`m_suspensionRelativeVelocity`); negative while
    /// compressing.
    pub relative_velocity: f32,
    /// `1 / (normal · up)`, or `10` when the ray hit at a grazing angle
    /// (`m_clippedInvContactDotSuspension`).
    pub inv_normal_dot_up: f32,
    /// The hard-stop impulse (`m_extraPushback`) for a spring compressed
    /// more than `SUSPENSION_SUBTRACTION` past its rest: Bullet's
    /// `resolveSingleCollision` against the surface with that overshoot as
    /// the penetration — its `erp`-scaled positional error plus the
    /// contact's approach velocity, through the contact's effective mass,
    /// floored at zero and divided by the wheel count. Zero at rest (the
    /// springs sit `≈1.5` uu compressed, inside the `2.5` uu margin) and
    /// what stops a hard landing without a bounce.
    pub extra_pushback: f32,
    /// This wheel's engine force (`m_engineForce`), set by
    /// [`update_wheels`].
    pub engine_force: f32,
    /// This wheel's brake clamp (`m_brake`), set by [`update_wheels`].
    pub brake_force: f32,
    /// Lateral friction factor (`m_latFriction`), set by [`update_wheels`].
    pub lat_friction: f32,
    /// Longitudinal friction factor (`m_longFriction`), set by
    /// [`update_wheels`].
    pub long_friction: f32,
    /// Steer angle in radians about the car's up (`m_steerAngle`); `0` on
    /// the back wheels.
    pub steer_angle: f32,
    /// The friction impulse per second (`m_impulse`) computed by
    /// [`compute_friction_impulses`] and applied `× dt` by
    /// [`apply_friction_impulses`].
    pub friction_impulse: Vec3,
}

impl WheelState {
    /// A wheel that has never touched anything (`resetSuspension`).
    pub fn airborne(mount: &WheelMount) -> WheelState {
        WheelState {
            in_contact: false,
            contact_point: Vec3::ZERO,
            contact_normal: Vec3::new(0.0, 0.0, 1.0),
            suspension_length: mount.rest_length + MAX_SUSPENSION_TRAVEL,
            relative_velocity: 0.0,
            inv_normal_dot_up: 1.0,
            extra_pushback: 0.0,
            engine_force: 0.0,
            brake_force: 0.0,
            lat_friction: 0.0,
            long_friction: 0.0,
            steer_angle: 0.0,
            friction_impulse: Vec3::ZERO,
        }
    }
}

/// The four wheels of a car that has never touched anything.
pub fn initial_wheels() -> [WheelState; 4] {
    [
        WheelState::airborne(&WHEELS[0]),
        WheelState::airborne(&WHEELS[1]),
        WheelState::airborne(&WHEELS[2]),
        WheelState::airborne(&WHEELS[3]),
    ]
}

/// How many wheels currently touch something.
pub fn wheels_in_contact(wheels: &[WheelState; 4]) -> usize {
    wheels.iter().filter(|wheel| wheel.in_contact).count()
}

/// Whether the car is on the ground for driving purposes (three or more
/// wheels touching).
pub fn is_on_ground(wheels: &[WheelState; 4]) -> bool {
    wheels_in_contact(wheels) >= WHEELS_FOR_GROUNDED
}

/// The length of a wheel's ray: `rest + travel + radius -
/// SUSPENSION_SUBTRACTION` (`btVehicleRL::rayCast`'s `realRayLength`) —
/// `48.755` uu for a front wheel, `49.555` for a back wheel.
pub fn ray_length(mount: &WheelMount) -> f32 {
    mount.rest_length + MAX_SUSPENSION_TRAVEL + mount.radius - SUSPENSION_SUBTRACTION
}

/// Casts each wheel's ray from its mount along the car's down axis against
/// `planes` (the ground and the walls) and rewrites the wheel's geometric
/// fields (`btVehicleRL::rayCast`): the contact point and normal, the
/// spring length (`trace - radius`, clamped to `rest ± travel`), the
/// suspension relative velocity (`normal · velocity_at_contact / (normal
/// · up)`, zeroed at a grazing angle), and the `extra_pushback` hard stop
/// when the trace is shorter than `rest + radius - SUSPENSION_SUBTRACTION`
/// (`dt` is the step the pushback's positional term is scaled by, Bullet's
/// `solverInfo.m_timeStep`). The drive fields are left alone.
pub fn raycast_wheels(
    car: &RigidBody,
    wheels: &mut [WheelState; 4],
    planes: &[&StaticPlane],
    dt: f32,
) {
    let up = drive::up_axis(car);
    let down = -up;
    for (mount, wheel) in WHEELS.iter().zip(wheels.iter_mut()) {
        let origin = car.position + car.orientation.rotate(&mount.mount);
        let length = ray_length(mount);
        let mut nearest: Option<collision::RayHit> = None;
        for plane in planes {
            if let Some(hit) = collision::ray_vs_plane(origin, down, length, plane) {
                if nearest.is_none_or(|best| hit.distance < best.distance) {
                    nearest = Some(hit);
                }
            }
        }
        match nearest {
            Some(hit) => {
                wheel.in_contact = true;
                wheel.contact_point = hit.point;
                wheel.contact_normal = hit.normal;
                // The ray runs exactly along `down`, so the trace length
                // along `up` is the hit distance.
                let trace = hit.distance;
                wheel.suspension_length = (trace - mount.radius).clamp(
                    mount.rest_length - MAX_SUSPENSION_TRAVEL,
                    mount.rest_length + MAX_SUSPENSION_TRAVEL,
                );
                let denominator = hit.normal.dot(&up);
                let rel_pos = hit.point - car.position;
                let projected_velocity = hit.normal.dot(&car.velocity_at_point(&rel_pos));
                if denominator > 0.1 {
                    let inv = 1.0 / denominator;
                    wheel.relative_velocity = projected_velocity * inv;
                    wheel.inv_normal_dot_up = inv;
                } else {
                    wheel.relative_velocity = 0.0;
                    wheel.inv_normal_dot_up = 10.0;
                }
                // `resolveSingleCollision` (RocketSim's variant, which
                // returns the impulse without applying it) with the
                // overshoot past the threshold as the penetration and
                // zero restitution, shared over the four wheels.
                let pushback_threshold = mount.rest_length + mount.radius - SUSPENSION_SUBTRACTION;
                wheel.extra_pushback = if trace < pushback_threshold {
                    let penetration = pushback_threshold - trace;
                    let positional_error = PUSHBACK_ERP * penetration / dt;
                    let velocity_error = -projected_velocity;
                    let (_, _, denominator) =
                        crate::solver::effective_mass_denom(car, &rel_pos, &hit.normal);
                    ((positional_error + velocity_error) / denominator).max(0.0)
                        / WHEELS.len() as f32
                } else {
                    0.0
                };
            }
            None => {
                wheel.in_contact = false;
                wheel.contact_point = origin + down * length;
                wheel.contact_normal = up;
                wheel.suspension_length = mount.rest_length + MAX_SUSPENSION_TRAVEL;
                wheel.relative_velocity = 0.0;
                wheel.inv_normal_dot_up = 1.0;
                wheel.extra_pushback = 0.0;
            }
        }
    }
}

/// `calcFrictionImpulses`: from the car's current velocity and each
/// touching wheel's drive fields (the previous tick's), the per-second
/// friction impulse the wheel will apply — a lateral bilateral impulse
/// that removes `BILATERAL_CONTACT_DAMPING` of the contact's sideways
/// velocity through the contact's effective mass, plus a rolling term
/// along the wheel's forward: `-engine / friction_scale` while the engine
/// drives, otherwise the brake's `clamp(-v_forward · 113.74, ±brake)`,
/// otherwise nothing. The sum, weighted by the lateral/longitudinal
/// friction factors and scaled by `friction_scale = mass / 3`, is stored
/// on the wheel for [`apply_friction_impulses`].
pub fn compute_friction_impulses(car: &RigidBody, wheels: &mut [WheelState; 4], dt: f32) {
    let friction_scale = car.mass() / FRICTION_SCALE_DIVISOR;
    let up = drive::up_axis(car);
    let right = drive::right_axis(car);
    for wheel in wheels.iter_mut() {
        if !wheel.in_contact {
            wheel.friction_impulse = Vec3::ZERO;
            continue;
        }
        let normal = wheel.contact_normal;
        // The axle direction, including the steer angle (a rotation of
        // the car's right about its up), projected onto the surface.
        let steered_right = rotate_about(&right, &up, wheel.steer_angle);
        let axle = steered_right - normal * steered_right.dot(&normal);
        let Some(axle) = axle.normalize() else {
            wheel.friction_impulse = Vec3::ZERO;
            continue;
        };
        let Some(forward_dir) = normal.cross(&axle).normalize() else {
            wheel.friction_impulse = Vec3::ZERO;
            continue;
        };

        let rel_pos = wheel.contact_point - car.position;
        let contact_velocity = car.velocity_at_point(&rel_pos);

        // Bullet `resolveSingleBilateral` against a static ground.
        let side_impulse = {
            let (_, _, denominator) = crate::solver::effective_mass_denom(car, &rel_pos, &axle);
            -BILATERAL_CONTACT_DAMPING * axle.dot(&contact_velocity) / denominator
        };

        let rolling_friction = if wheel.engine_force != 0.0 {
            // "Engine force already accounts for our mass, so we will
            // cancel out the friction scale multiplication at the end."
            -wheel.engine_force / friction_scale
        } else if wheel.brake_force != 0.0 {
            let mut relative_velocity = contact_velocity.dot(&forward_dir);
            if dt > 1.0 / ROLLING_FRICTION_ROUNDING_TICK_RATE {
                // RocketSim compares this threshold in Bullet units.
                let threshold = (-(1.0 / (dt * 150.0)) + 0.8) * BT_TO_UU;
                if relative_velocity.abs() < threshold {
                    relative_velocity = 0.0;
                }
            }
            (-relative_velocity * ROLLING_FRICTION_SCALE_MAGIC)
                .clamp(-wheel.brake_force, wheel.brake_force)
        } else {
            0.0
        };

        let total = forward_dir * (rolling_friction * wheel.long_friction)
            + axle * (side_impulse * wheel.lat_friction);
        wheel.friction_impulse = total * friction_scale;
    }
}

/// `Car::_UpdateWheels`, the wheel half: this tick's engine and brake
/// force on every wheel from the throttle/boost/handbrake input and the
/// forward speed (full brake when coasting below `STOPPING_FORWARD_VEL`
/// or throttling against the direction of travel, `COASTING_BRAKE_FACTOR`
/// when coasting faster, the engine cut while braking), the front wheels'
/// steer angle (`steer · STEER_ANGLE_FROM_SPEED_CURVE(|forward speed|)`,
/// blended toward the powerslide curve by the analog handbrake value),
/// each touching wheel's friction factors (step (b): the slip-driven
/// `LAT_FRICTION_CURVE`, the handbrake's lateral and longitudinal factor
/// curves blended in by `handbrake_val`, and the non-sticky curve of the
/// contact normal's `z` whenever no throttle is held), and the sticky
/// force into the averaged contact normal whenever any wheel touches:
/// `0.5 g`, plus `(1 - |up.z|) g` when driving (throttle held or faster
/// than `STOPPING_FORWARD_VEL`), so a car on a vertical wall is pressed
/// into it with a full g.
///
/// `handbrake_val` is RocketSim's `handbrakeVal`, the car's analog
/// handbrake: ramped here first (`+POWERSLIDE_RISE_RATE · dt` while held,
/// `-POWERSLIDE_FALL_RATE · dt` otherwise, clamped to `0..=1`) and then
/// read by the steering and friction blocks. `gravity_z` is the world's
/// vertical gravity (`-650`); `boost_amount` is read only, since boosting
/// with boost left drives the wheels at full throttle.
#[allow(clippy::too_many_arguments)]
pub fn update_wheels(
    car: &mut RigidBody,
    wheels: &mut [WheelState; 4],
    throttle: f32,
    steer: f32,
    boost_pressed: bool,
    boost_amount: f32,
    handbrake: bool,
    handbrake_val: &mut f32,
    gravity_z: f32,
    dt: f32,
) {
    let forward_speed = car.linear_velocity.dot(&drive::forward_axis(car));
    let abs_forward_speed = forward_speed.abs();
    let in_contact = wheels_in_contact(wheels);

    *handbrake_val = if handbrake {
        *handbrake_val + POWERSLIDE_RISE_RATE * dt
    } else {
        *handbrake_val - POWERSLIDE_FALL_RATE * dt
    }
    .clamp(0.0, 1.0);

    let real_throttle = if boost_pressed && boost_amount > 0.0 {
        1.0
    } else {
        throttle.clamp(-1.0, 1.0)
    };
    let mut engine_throttle = real_throttle;
    let mut real_brake = 0.0;
    if !handbrake {
        if real_throttle.abs() >= THROTTLE_DEADZONE {
            if abs_forward_speed > STOPPING_FORWARD_VEL
                && sign(real_throttle) != sign(forward_speed)
            {
                // Full brake when trying to drive the other way.
                real_brake = 1.0;
                if abs_forward_speed > BRAKING_NO_THROTTLE_SPEED_THRESH {
                    engine_throttle = 0.0;
                }
            }
        } else {
            // Coasting: brake gently, or fully when nearly stopped.
            engine_throttle = 0.0;
            real_brake = if abs_forward_speed < STOPPING_FORWARD_VEL {
                1.0
            } else {
                COASTING_BRAKE_FACTOR
            };
        }
    }

    let mut drive_speed_scale = drive::drive_speed_taper(abs_forward_speed);
    if in_contact < WHEELS_FOR_GROUNDED {
        drive_speed_scale /= 4.0;
    }
    let engine_force = engine_throttle * THROTTLE_TORQUE_AMOUNT * drive_speed_scale;
    let brake_force = real_brake * BRAKE_TORQUE_AMOUNT;

    let mut steer_angle = piecewise_linear(&STEER_ANGLE_FROM_SPEED_CURVE, abs_forward_speed);
    if *handbrake_val > 0.0 {
        // `steerAngle += (powerslide - steerAngle) * handbrakeVal`.
        let powerslide =
            piecewise_linear(&POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE, abs_forward_speed);
        steer_angle += (powerslide - steer_angle) * *handbrake_val;
    }
    steer_angle *= steer.clamp(-1.0, 1.0);

    for (mount, wheel) in WHEELS.iter().zip(wheels.iter_mut()) {
        wheel.engine_force = engine_force;
        wheel.brake_force = brake_force;
        wheel.steer_angle = if mount.is_front { steer_angle } else { 0.0 };
    }

    // The friction factors, per touching wheel (a wheel in the air keeps
    // its last values, as RocketSim's do; nothing reads them there).
    let up = drive::up_axis(car);
    let right = drive::right_axis(car);
    for (mount, wheel) in WHEELS.iter().zip(wheels.iter_mut()) {
        if !wheel.in_contact {
            continue;
        }
        // `latDir` is the wheel transform's own axle — the car's right
        // steered about its up, *not* flattened onto the surface —
        // and `longDir = latDir × contactNormal`.
        let lat_dir = rotate_about(&right, &up, wheel.steer_angle);
        let long_dir = lat_dir.cross(&wheel.contact_normal);
        // The mount's velocity (`hardPointWS`, not the contact point).
        let mount_rel = car.orientation.rotate(&mount.mount);
        let mount_velocity = car.velocity_at_point(&mount_rel);
        let lateral = mount_velocity.dot(&lat_dir).abs();
        let slip = if lateral > LATERAL_SLIP_THRESHOLD {
            lateral / (mount_velocity.dot(&long_dir).abs() + lateral)
        } else {
            0.0
        };
        let mut lat_friction = piecewise_linear(&LAT_FRICTION_CURVE, slip);
        let mut long_friction = piecewise_linear(&LONG_FRICTION_CURVE, slip);
        if *handbrake_val > 0.0 {
            lat_friction *= (piecewise_linear(&HANDBRAKE_LAT_FRICTION_FACTOR_CURVE, slip) - 1.0)
                * *handbrake_val
                + 1.0;
            long_friction *= (piecewise_linear(&HANDBRAKE_LONG_FRICTION_FACTOR_CURVE, slip) - 1.0)
                * *handbrake_val
                + 1.0;
        } else {
            // "If we aren't powersliding, it's not scaled down."
            long_friction = 1.0;
        }
        if real_throttle == 0.0 {
            let non_sticky =
                piecewise_linear(&NON_STICKY_FRICTION_FACTOR_CURVE, wheel.contact_normal.z);
            lat_friction *= non_sticky;
            long_friction *= non_sticky;
        }
        wheel.lat_friction = lat_friction;
        wheel.long_friction = long_friction;
    }

    if in_contact > 0 {
        let upwards = upwards_dir_from_contacts(car, wheels);
        let full_stick = real_throttle != 0.0 || abs_forward_speed > STOPPING_FORWARD_VEL;
        let mut scale = STICKY_FORCE_BASE_SCALE;
        if full_stick {
            scale += 1.0 - upwards.z.abs();
        }
        car.apply_central_force(upwards * (scale * gravity_z * car.mass()));
    }
}

/// `updateSuspension` and its impulse loop: each touching wheel's spring
/// force `(rest - length) · stiffness · inv - damping · relative_velocity`
/// (compression damping while compressing, relaxation damping while
/// extending), times the wheel's force scale, floored at zero ("RL never
/// uses downwards suspension forces"), applied as `normal · (force · dt +
/// extra_pushback)` at the contact point — the pushback rides along only
/// when the spring force is nonzero, as in RocketSim's loop.
pub fn apply_suspension_impulses(car: &mut RigidBody, wheels: &[WheelState; 4], dt: f32) {
    for (mount, wheel) in WHEELS.iter().zip(wheels.iter()) {
        if !wheel.in_contact {
            continue;
        }
        let spring = (mount.rest_length - wheel.suspension_length)
            * SUSPENSION_STIFFNESS
            * wheel.inv_normal_dot_up;
        let damping = if wheel.relative_velocity < 0.0 {
            WHEELS_DAMPING_COMPRESSION
        } else {
            WHEELS_DAMPING_RELAXATION
        };
        let force = ((spring - damping * wheel.relative_velocity) * mount.force_scale).max(0.0);
        if force > 0.0 {
            let rel_pos = wheel.contact_point - car.position;
            car.apply_impulse(
                wheel.contact_normal * (force * dt + wheel.extra_pushback),
                rel_pos,
            );
        }
    }
}

/// `applyFrictionImpulses`: each wheel's stored friction impulse `× dt`,
/// applied at the contact point with its offset's component along the
/// car's up removed, so tire forces never pitch the body about the
/// contact height.
pub fn apply_friction_impulses(car: &mut RigidBody, wheels: &[WheelState; 4], dt: f32) {
    let up = drive::up_axis(car);
    for wheel in wheels.iter() {
        if wheel.friction_impulse == Vec3::ZERO {
            continue;
        }
        let offset = wheel.contact_point - car.position;
        let flattened = offset - up * up.dot(&offset);
        car.apply_impulse(wheel.friction_impulse * dt, flattened);
    }
}

/// `getUpwardsDirFromWheelContacts`: the normalized sum of the touching
/// wheels' contact normals, or the car's up when none touch.
pub fn upwards_dir_from_contacts(car: &RigidBody, wheels: &[WheelState; 4]) -> Vec3 {
    let mut sum = Vec3::ZERO;
    for wheel in wheels.iter().filter(|wheel| wheel.in_contact) {
        sum += wheel.contact_normal;
    }
    sum.normalize().unwrap_or_else(|| drive::up_axis(car))
}

/// RocketSim's `LinearPieceCurve::GetOutput`: linear between the points,
/// clamped to the first and last values outside them.
pub fn piecewise_linear(points: &[(f32, f32)], x: f32) -> f32 {
    // `GetOutput`'s `defaultOutput = 1` for an empty curve.
    let Some(first) = points.first() else {
        return 1.0;
    };
    if x <= first.0 {
        return points[0].1;
    }
    for window in points.windows(2) {
        let (x0, y0) = window[0];
        let (x1, y1) = window[1];
        if x <= x1 {
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
        }
    }
    points[points.len() - 1].1
}

/// RocketSim's `RS_SGN`: `-1`, `0`, or `1`.
fn sign(value: f32) -> i8 {
    (value > 0.0) as i8 - (value < 0.0) as i8
}

/// Rotates `v` about the unit `axis` by `angle` radians (Rodrigues).
fn rotate_about(v: &Vec3, axis: &Vec3, angle: f32) -> Vec3 {
    if angle == 0.0 {
        return *v;
    }
    let (sin, cos) = angle.sin_cos();
    *v * cos + axis.cross(v) * sin + *axis * (axis.dot(v) * (1.0 - cos))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::CAR_HITBOX_OFFSET;

    const DT: f32 = 1.0 / 120.0;

    fn ground() -> StaticPlane {
        StaticPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0)
    }

    fn car_at(z: f32) -> RigidBody {
        RigidBody::standard_car(Vec3::new(0.0, 0.0, z))
    }

    #[test]
    fn the_spring_rests_are_the_declared_lengths_minus_the_travel() {
        assert!((WHEELS[0].rest_length - 26.755).abs() < 1e-4);
        assert!((WHEELS[2].rest_length - 25.055).abs() < 1e-4);
        assert!((SUSPENSION_SUBTRACTION - 2.5).abs() < 1e-6);
        assert!((ray_length(&WHEELS[0]) - 48.755).abs() < 1e-3);
        assert!((ray_length(&WHEELS[2]) - 49.555).abs() < 1e-3);
    }

    #[test]
    fn the_wheels_mount_at_the_hitbox_height_and_mirror_left_to_right() {
        for mount in &WHEELS {
            assert_eq!(mount.mount.z, CAR_HITBOX_OFFSET.z);
        }
        assert_eq!(WHEELS[0].mount.y, -WHEELS[1].mount.y);
        assert_eq!(WHEELS[2].mount.y, -WHEELS[3].mount.y);
        assert!(WHEELS[0].is_front && WHEELS[1].is_front);
        assert!(!WHEELS[2].is_front && !WHEELS[3].is_front);
    }

    #[test]
    fn a_car_at_the_recorded_rest_height_has_all_four_wheels_touching_slightly_compressed() {
        let car = car_at(17.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        assert_eq!(wheels_in_contact(&wheels), 4);
        assert!(is_on_ground(&wheels));
        // Mount at z = 37.755; trace 37.755; front spring 25.255 against a
        // rest of 26.755, back 22.755 against 25.055.
        assert!((wheels[0].suspension_length - 25.255).abs() < 1e-3);
        assert!((wheels[2].suspension_length - 22.755).abs() < 1e-3);
        assert_eq!(wheels[0].contact_normal, Vec3::new(0.0, 0.0, 1.0));
        assert!((wheels[0].contact_point.z).abs() < 1e-4);
        assert_eq!(wheels[0].inv_normal_dot_up, 1.0);
        assert_eq!(wheels[0].relative_velocity, 0.0);
    }

    #[test]
    fn the_ray_keeps_contact_until_the_car_has_risen_past_its_length() {
        // Front ray 48.755 from a mount 20.755 above the origin: contact
        // ends once the origin is above 28.0 — 11 uu of rise from rest.
        let mut wheels = initial_wheels();
        raycast_wheels(&car_at(27.0), &mut wheels, &[&ground()], DT);
        assert!(wheels[0].in_contact);
        // Mount at 47.755, trace 47.755, spring 35.255: extended past rest
        // but inside the travel.
        assert!((wheels[0].suspension_length - 35.255).abs() < 1e-3);
        assert!(wheels[0].suspension_length > WHEELS[0].rest_length);
        assert_eq!(wheels[0].extra_pushback, 0.0);
        raycast_wheels(&car_at(28.5), &mut wheels, &[&ground()], DT);
        assert!(!wheels[0].in_contact);
        assert!(wheels[2].in_contact, "the back ray is 0.8 uu longer");
        raycast_wheels(&car_at(29.0), &mut wheels, &[&ground()], DT);
        assert_eq!(wheels_in_contact(&wheels), 0);
    }

    #[test]
    fn the_pushback_hard_stop_engages_only_past_the_subtraction_margin() {
        // At the recorded rest height the springs sit 1.5 / 2.3 uu
        // compressed, inside the 2.5 uu margin: no pushback.
        let mut wheels = initial_wheels();
        raycast_wheels(&car_at(17.0), &mut wheels, &[&ground()], DT);
        assert!(wheels.iter().all(|wheel| wheel.extra_pushback == 0.0));
        // 5 uu lower the front springs are 6.5 uu compressed: the
        // positional term alone (no approach velocity) pushes back by
        // erp * (6.5 - 2.5) / dt through the contact's effective mass,
        // shared over four wheels.
        let car = car_at(12.0);
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        let rel_pos = wheels[0].contact_point - car.position;
        let (_, _, denominator) =
            crate::solver::effective_mass_denom(&car, &rel_pos, &Vec3::new(0.0, 0.0, 1.0));
        let expected = PUSHBACK_ERP * (6.5 - 2.5) / DT / denominator / 4.0;
        assert!(
            (wheels[0].extra_pushback - expected).abs() < 1e-2,
            "{} vs {expected}",
            wheels[0].extra_pushback
        );
        // A separating contact (moving up) is subtracted, never below zero.
        let mut rising = car_at(12.0);
        rising.linear_velocity = Vec3::new(0.0, 0.0, 1000.0);
        raycast_wheels(&rising, &mut wheels, &[&ground()], DT);
        assert_eq!(wheels[0].extra_pushback, 0.0);
        // An approaching contact adds its whole approach velocity.
        let mut falling = car_at(12.0);
        falling.linear_velocity = Vec3::new(0.0, 0.0, -300.0);
        raycast_wheels(&falling, &mut wheels, &[&ground()], DT);
        let expected_falling = (PUSHBACK_ERP * 4.0 / DT + 300.0) / denominator / 4.0;
        assert!((wheels[0].extra_pushback - expected_falling).abs() < 1e-2);
    }

    #[test]
    fn suspension_length_clamps_to_the_travel_on_both_sides() {
        let mut wheels = initial_wheels();
        // Mount 5 uu above the floor: trace 5, far below rest - travel.
        raycast_wheels(&car_at(5.0 - 20.755), &mut wheels, &[&ground()], DT);
        assert!((wheels[0].suspension_length - (26.755 - 12.0)).abs() < 1e-3);
    }

    #[test]
    fn a_descending_car_reads_a_negative_suspension_relative_velocity() {
        let mut car = car_at(17.0);
        car.linear_velocity = Vec3::new(0.0, 0.0, -100.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        assert!((wheels[0].relative_velocity + 100.0).abs() < 1e-3);
    }

    #[test]
    fn the_suspension_impulse_is_the_spring_minus_the_damping_times_the_force_scale() {
        let mut car = car_at(17.0);
        car.linear_velocity = Vec3::new(0.0, 0.0, -100.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        let before = car.linear_velocity;
        apply_suspension_impulses(&mut car, &wheels, DT);
        // Front: (26.755 - 25.255) * 500 + 25 * 100 = 3250, * 35.75;
        // back: (25.055 - 22.755) * 500 + 25 * 100 = 3650, * 54.265;
        // total over four wheels, / mass / 120.
        let expected_force = 2.0 * (3250.0 * SUSPENSION_FORCE_SCALE_FRONT)
            + 2.0 * (3650.0 * SUSPENSION_FORCE_SCALE_BACK);
        let expected_dv = expected_force / CAR_MASS * DT;
        let dv = car.linear_velocity.z - before.z;
        assert!(
            (dv - expected_dv).abs() < 1e-2,
            "dv {dv} vs expected {expected_dv}"
        );
    }

    #[test]
    fn extended_springs_never_pull_the_car_down() {
        let mut car = car_at(28.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        assert_eq!(wheels_in_contact(&wheels), 4);
        assert!(wheels[0].suspension_length > WHEELS[0].rest_length);
        apply_suspension_impulses(&mut car, &wheels, DT);
        assert_eq!(car.linear_velocity, Vec3::ZERO);
    }

    #[test]
    fn full_throttle_over_four_wheels_is_sixteen_hundred_per_second_squared() {
        let mut car = car_at(17.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        update_wheels(
            &mut car,
            &mut wheels,
            1.0,
            0.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        assert_eq!(wheels[0].engine_force, THROTTLE_TORQUE_AMOUNT);
        assert_eq!(wheels[0].brake_force, 0.0);
        compute_friction_impulses(&car, &mut wheels, DT);
        apply_friction_impulses(&mut car, &wheels, DT);
        let dv = car.linear_velocity.x;
        assert!((dv - 1600.0 * DT).abs() < 1e-3, "dv {dv}");
        assert!(car.linear_velocity.y.abs() < 1e-4);
        assert!(car.linear_velocity.z.abs() < 1e-4);
    }

    #[test]
    fn the_brake_clamps_at_thirty_five_hundred_per_second_squared_over_four_wheels() {
        let mut car = car_at(17.0);
        car.linear_velocity = Vec3::new(1000.0, 0.0, 0.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        // Throttle against the direction of travel at speed: full brake,
        // engine cut.
        update_wheels(
            &mut car,
            &mut wheels,
            -1.0,
            0.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        assert_eq!(wheels[0].engine_force, 0.0);
        assert_eq!(wheels[0].brake_force, BRAKE_TORQUE_AMOUNT);
        compute_friction_impulses(&car, &mut wheels, DT);
        apply_friction_impulses(&mut car, &wheels, DT);
        let dv = car.linear_velocity.x - 1000.0;
        assert!((dv + 3500.0 * DT).abs() < 1e-2, "dv {dv}");
    }

    #[test]
    fn coasting_brakes_at_fifteen_percent_and_fully_below_the_stopping_speed() {
        let mut car = car_at(17.0);
        car.linear_velocity = Vec3::new(500.0, 0.0, 0.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        update_wheels(
            &mut car,
            &mut wheels,
            0.0,
            0.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        assert_eq!(wheels[0].engine_force, 0.0);
        assert!((wheels[0].brake_force - COASTING_BRAKE_FACTOR * BRAKE_TORQUE_AMOUNT).abs() < 1e-3);

        car.linear_velocity = Vec3::new(10.0, 0.0, 0.0);
        update_wheels(
            &mut car,
            &mut wheels,
            0.0,
            0.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        assert_eq!(wheels[0].brake_force, BRAKE_TORQUE_AMOUNT);
        // Below the proportional band the brake is proportional to the
        // forward contact velocity, not the clamp: 113.74 * 10 * 60 * 4
        // over the mass.
        compute_friction_impulses(&car, &mut wheels, DT);
        apply_friction_impulses(&mut car, &wheels, DT);
        let expected_dv =
            -ROLLING_FRICTION_SCALE_MAGIC * 10.0 * (CAR_MASS / 3.0) * 4.0 / CAR_MASS * DT;
        let dv = car.linear_velocity.x - 10.0;
        assert!((dv - expected_dv).abs() < 1e-3, "dv {dv} vs {expected_dv}");
    }

    #[test]
    fn boost_drives_the_wheels_at_full_throttle_and_handbrake_keeps_the_input_throttle() {
        let mut car = car_at(17.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        update_wheels(
            &mut car,
            &mut wheels,
            0.0,
            0.0,
            true,
            50.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        assert_eq!(wheels[0].engine_force, THROTTLE_TORQUE_AMOUNT);
        update_wheels(
            &mut car,
            &mut wheels,
            0.0,
            0.0,
            true,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        assert_eq!(wheels[0].engine_force, 0.0, "no boost left");

        // Handbrake: no coasting brake, the real lateral factor.
        car.linear_velocity = Vec3::new(500.0, 0.0, 0.0);
        update_wheels(
            &mut car,
            &mut wheels,
            0.0,
            0.0,
            false,
            0.0,
            true,
            &mut 1.0,
            -650.0,
            DT,
        );
        assert_eq!(wheels[0].brake_force, 0.0);
        // Rolling straight (zero slip): the handbrake's lateral `0.1` and
        // longitudinal `0.5`, fully in at `handbrake_val = 1`.
        assert!((wheels[0].lat_friction - 0.1).abs() < 1e-6);
        assert!((wheels[0].long_friction - 0.5).abs() < 1e-6);
    }

    #[test]
    fn the_lateral_impulse_removes_a_fifth_of_the_sideways_contact_velocity_per_wheel_scaled() {
        let mut car = car_at(17.0);
        car.linear_velocity = Vec3::new(0.0, 300.0, 0.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        update_wheels(
            &mut car,
            &mut wheels,
            1.0,
            0.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        compute_friction_impulses(&car, &mut wheels, DT);
        for wheel in &wheels {
            assert!(wheel.friction_impulse.y < 0.0, "grip opposes the slide");
        }
        apply_friction_impulses(&mut car, &wheels, DT);
        assert!(car.linear_velocity.y < 300.0);
        assert!(
            car.linear_velocity.y > 200.0,
            "one tick does not stop a slide"
        );
        // Handbrake cuts the lateral grip to a tenth.
        let mut sliding = car_at(17.0);
        sliding.linear_velocity = Vec3::new(0.0, 300.0, 0.0);
        let mut handbrake_wheels = initial_wheels();
        raycast_wheels(&sliding, &mut handbrake_wheels, &[&ground()], DT);
        update_wheels(
            &mut sliding,
            &mut handbrake_wheels,
            1.0,
            0.0,
            false,
            0.0,
            true,
            &mut 1.0,
            -650.0,
            DT,
        );
        compute_friction_impulses(&sliding, &mut handbrake_wheels, DT);
        apply_friction_impulses(&mut sliding, &handbrake_wheels, DT);
        let grip = 300.0 - car.linear_velocity.y;
        let handbrake_grip = 300.0 - sliding.linear_velocity.y;
        assert!((handbrake_grip - grip * 0.1).abs() < 1e-3);
    }

    #[test]
    fn the_sticky_force_is_half_a_g_on_the_floor_and_a_full_g_more_on_a_wall_when_driving() {
        let mut car = car_at(17.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        car.clear_forces();
        update_wheels(
            &mut car,
            &mut wheels,
            0.0,
            0.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        let resting = car.total_force();
        assert!(
            (resting.z + 0.5 * 650.0 * CAR_MASS).abs() < 1e-2,
            "{resting:?}"
        );
        car.clear_forces();
        update_wheels(
            &mut car,
            &mut wheels,
            1.0,
            0.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        let driving = car.total_force();
        assert!(
            (driving.z + 0.5 * 650.0 * CAR_MASS).abs() < 1e-2,
            "on the floor |up.z| = 1 adds nothing: {driving:?}"
        );
        // A car with its wheels on a vertical wall (contact normal along
        // -x) gets a full g more into the wall while driving.
        let mut wall_wheels = initial_wheels();
        for wheel in wall_wheels.iter_mut() {
            wheel.in_contact = true;
            wheel.contact_normal = Vec3::new(-1.0, 0.0, 0.0);
        }
        car.clear_forces();
        update_wheels(
            &mut car,
            &mut wall_wheels,
            1.0,
            0.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        let on_wall = car.total_force();
        assert!(
            (on_wall.x - 1.5 * 650.0 * CAR_MASS).abs() < 1e-2,
            "{on_wall:?}"
        );
        // No wheel touching: no sticky force at all.
        let mut airborne = initial_wheels();
        car.clear_forces();
        update_wheels(
            &mut car,
            &mut airborne,
            1.0,
            0.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        assert_eq!(car.total_force(), Vec3::ZERO);
    }

    #[test]
    fn engine_force_is_quartered_under_three_wheels() {
        let mut car = car_at(17.0);
        let mut wheels = initial_wheels();
        wheels[0].in_contact = true;
        wheels[0].contact_normal = Vec3::new(0.0, 0.0, 1.0);
        wheels[1].in_contact = true;
        wheels[1].contact_normal = Vec3::new(0.0, 0.0, 1.0);
        update_wheels(
            &mut car,
            &mut wheels,
            1.0,
            0.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        assert!((wheels[0].engine_force - THROTTLE_TORQUE_AMOUNT / 4.0).abs() < 1e-3);
        assert!(!is_on_ground(&wheels));
    }

    #[test]
    fn the_steer_angle_follows_the_real_curve_on_the_front_wheels_only() {
        let mut car = car_at(17.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        update_wheels(
            &mut car,
            &mut wheels,
            0.0,
            1.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        assert!((wheels[0].steer_angle - 0.53356).abs() < 1e-5);
        assert!((wheels[1].steer_angle - 0.53356).abs() < 1e-5);
        assert_eq!(wheels[2].steer_angle, 0.0);
        car.linear_velocity = Vec3::new(1250.0, 0.0, 0.0);
        update_wheels(
            &mut car,
            &mut wheels,
            0.0,
            -0.5,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        let expected = -0.5 * (0.18203 + 0.10570) / 2.0;
        assert!(
            (wheels[0].steer_angle - expected).abs() < 1e-5,
            "{}",
            wheels[0].steer_angle
        );
        // Handbrake: the powerslide curve.
        update_wheels(
            &mut car,
            &mut wheels,
            0.0,
            1.0,
            false,
            0.0,
            true,
            &mut 1.0,
            -650.0,
            DT,
        );
        let powerslide = 0.39235 + (0.12610 - 0.39235) * 1250.0 / 2500.0;
        assert!((wheels[0].steer_angle - powerslide).abs() < 1e-5);
        // Past the curve's end it clamps.
        car.linear_velocity = Vec3::new(4000.0, 0.0, 0.0);
        update_wheels(
            &mut car,
            &mut wheels,
            0.0,
            1.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        assert!((wheels[0].steer_angle - 0.03454).abs() < 1e-5);
    }

    #[test]
    fn a_steered_moving_car_yaws_through_its_front_wheels_lateral_grip() {
        let mut car = car_at(17.0);
        car.linear_velocity = Vec3::new(1000.0, 0.0, 0.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        update_wheels(
            &mut car,
            &mut wheels,
            0.0,
            1.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        compute_friction_impulses(&car, &mut wheels, DT);
        assert!(
            wheels[0].friction_impulse.y > 0.0,
            "front wheels pull right"
        );
        assert!(
            wheels[2].friction_impulse.y.abs() < 1e-3,
            "unsteered back wheels have no slip"
        );
        apply_friction_impulses(&mut car, &wheels, DT);
        assert!(car.angular_velocity.z > 0.0, "{:?}", car.angular_velocity);
    }

    #[test]
    fn a_wall_plane_is_seen_by_the_rays_of_a_car_driving_on_it() {
        // A car rotated to lie on a wall at x = 100 (its up along -x): the
        // rays run along +x and find the wall.
        let wall = StaticPlane::new(Vec3::new(-1.0, 0.0, 0.0), -100.0);
        let mut car = RigidBody::standard_car(Vec3::new(100.0 - 17.0, 0.0, 500.0));
        // Rotate so the car's up (+z) points along -x: a rotation of +90°
        // about y takes +z to +x, so use -90°.
        let half = -std::f32::consts::FRAC_PI_4;
        car.orientation = rb_domain::Quat::new(0.0, half.sin(), 0.0, half.cos()).normalize();
        car.update_inertia_tensor();
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground(), &wall], DT);
        assert_eq!(wheels_in_contact(&wheels), 4, "{wheels:?}");
        assert_eq!(wheels[0].contact_normal, Vec3::new(-1.0, 0.0, 0.0));
    }

    // RB-PHYSICS-001-FR-082 step (b): the curves.

    #[test]
    fn an_empty_curve_reads_one_and_a_single_point_reads_that_point_everywhere() {
        assert_eq!(piecewise_linear(&LONG_FRICTION_CURVE, 0.0), 1.0);
        assert_eq!(piecewise_linear(&LONG_FRICTION_CURVE, 0.7), 1.0);
        assert_eq!(
            piecewise_linear(&HANDBRAKE_LAT_FRICTION_FACTOR_CURVE, 0.0),
            0.1
        );
        assert_eq!(
            piecewise_linear(&HANDBRAKE_LAT_FRICTION_FACTOR_CURVE, 1.0),
            0.1
        );
    }

    #[test]
    fn the_analog_handbrake_ramps_up_in_a_fifth_of_a_second_and_down_in_half_a_second() {
        let mut car = car_at(17.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        let mut value = 0.0;
        update_wheels(
            &mut car,
            &mut wheels,
            0.0,
            0.0,
            false,
            0.0,
            true,
            &mut value,
            -650.0,
            DT,
        );
        assert!((value - POWERSLIDE_RISE_RATE * DT).abs() < 1e-6, "{value}");
        for _ in 0..23 {
            update_wheels(
                &mut car,
                &mut wheels,
                0.0,
                0.0,
                false,
                0.0,
                true,
                &mut value,
                -650.0,
                DT,
            );
        }
        assert!((value - 1.0).abs() < 1e-5, "full after 24 ticks: {value}");
        update_wheels(
            &mut car,
            &mut wheels,
            0.0,
            0.0,
            false,
            0.0,
            true,
            &mut value,
            -650.0,
            DT,
        );
        assert_eq!(value, 1.0, "clamped at one");
        update_wheels(
            &mut car,
            &mut wheels,
            0.0,
            0.0,
            false,
            0.0,
            false,
            &mut value,
            -650.0,
            DT,
        );
        assert!(
            (value - (1.0 - POWERSLIDE_FALL_RATE * DT)).abs() < 1e-6,
            "{value}"
        );
        for _ in 0..59 {
            update_wheels(
                &mut car,
                &mut wheels,
                0.0,
                0.0,
                false,
                0.0,
                false,
                &mut value,
                -650.0,
                DT,
            );
        }
        assert!(value.abs() < 1e-5, "gone after 60 ticks: {value}");
        update_wheels(
            &mut car,
            &mut wheels,
            0.0,
            0.0,
            false,
            0.0,
            false,
            &mut value,
            -650.0,
            DT,
        );
        assert_eq!(value, 0.0, "clamped at zero");
    }

    #[test]
    fn a_half_engaged_handbrake_blends_halfway_between_the_two_steer_curves_and_factors() {
        let mut car = car_at(17.0);
        car.linear_velocity = Vec3::new(1250.0, 0.0, 0.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        // Held with the value at `0.5 - rise`, so it reads `0.5` this tick.
        let mut value = 0.5 - POWERSLIDE_RISE_RATE * DT;
        update_wheels(
            &mut car,
            &mut wheels,
            1.0,
            1.0,
            false,
            0.0,
            true,
            &mut value,
            -650.0,
            DT,
        );
        let normal = piecewise_linear(&STEER_ANGLE_FROM_SPEED_CURVE, 1250.0);
        let powerslide = piecewise_linear(&POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE, 1250.0);
        assert!((wheels[0].steer_angle - (normal + powerslide) / 2.0).abs() < 1e-5);
        // Rolling straight on the unsteered back wheels (the steered front
        // wheels slip against the axle they are turned to): lateral `1 →
        // 0.1` half blended is `0.55`, longitudinal `1 → 0.5` half blended
        // is `0.75`.
        assert!(
            (wheels[2].lat_friction - 0.55).abs() < 1e-5,
            "{}",
            wheels[2].lat_friction
        );
        assert!(
            (wheels[2].long_friction - 0.75).abs() < 1e-5,
            "{}",
            wheels[2].long_friction
        );
    }

    #[test]
    fn lateral_grip_falls_with_the_mounts_slip_ratio_and_ignores_slip_under_the_threshold() {
        let mut wheels = initial_wheels();
        let mut factors = |velocity: Vec3| {
            let mut car = car_at(17.0);
            car.linear_velocity = velocity;
            raycast_wheels(&car, &mut wheels, &[&ground()], DT);
            update_wheels(
                &mut car,
                &mut wheels,
                1.0,
                0.0,
                false,
                0.0,
                false,
                &mut 0.0,
                -650.0,
                DT,
            );
            (wheels[0].lat_friction, wheels[0].long_friction)
        };
        assert_eq!(
            factors(Vec3::new(1000.0, 0.0, 0.0)),
            (1.0, 1.0),
            "rolling straight"
        );
        let (lat, long) = factors(Vec3::new(0.0, 300.0, 0.0));
        assert!(
            (lat - 0.2).abs() < 1e-6 && long == 1.0,
            "sliding straight sideways: {lat}"
        );
        let (lat, long) = factors(Vec3::new(300.0, 300.0, 0.0));
        assert!((lat - 0.6).abs() < 1e-5, "half slip: {lat}");
        assert_eq!(long, 1.0, "longitudinal is one without the handbrake");
        assert_eq!(
            factors(Vec3::new(1000.0, LATERAL_SLIP_THRESHOLD - 0.5, 0.0)),
            (1.0, 1.0),
            "under the threshold reads as no slip"
        );
        let (lat, _) = factors(Vec3::new(1000.0, LATERAL_SLIP_THRESHOLD + 0.5, 0.0));
        assert!(lat < 1.0 && lat > 0.99, "just over it: {lat}");
    }

    #[test]
    fn the_handbrakes_longitudinal_factor_rises_with_slip_and_the_lateral_stays_a_tenth() {
        let mut car = car_at(17.0);
        car.linear_velocity = Vec3::new(0.0, 300.0, 0.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        update_wheels(
            &mut car,
            &mut wheels,
            1.0,
            0.0,
            false,
            0.0,
            true,
            &mut 1.0,
            -650.0,
            DT,
        );
        // Sliding straight sideways: `0.2 · 0.1` laterally, `1 · 0.9`
        // longitudinally.
        assert!(
            (wheels[0].lat_friction - 0.02).abs() < 1e-6,
            "{}",
            wheels[0].lat_friction
        );
        assert!(
            (wheels[0].long_friction - 0.9).abs() < 1e-6,
            "{}",
            wheels[0].long_friction
        );
    }

    #[test]
    fn a_coasting_car_on_a_wall_keeps_a_tenth_of_its_grip_and_a_driving_one_all_of_it() {
        let mut car = car_at(17.0);
        let mut wall_wheels = initial_wheels();
        for wheel in wall_wheels.iter_mut() {
            wheel.in_contact = true;
            wheel.contact_normal = Vec3::new(-1.0, 0.0, 0.0);
        }
        update_wheels(
            &mut car,
            &mut wall_wheels,
            0.0,
            0.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        assert!(
            (wall_wheels[0].lat_friction - 0.1).abs() < 1e-6,
            "{}",
            wall_wheels[0].lat_friction
        );
        assert!((wall_wheels[0].long_friction - 0.1).abs() < 1e-6);
        update_wheels(
            &mut car,
            &mut wall_wheels,
            1.0,
            0.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        assert_eq!(
            wall_wheels[0].lat_friction, 1.0,
            "throttle makes the contact sticky"
        );
        assert_eq!(wall_wheels[0].long_friction, 1.0);
        // Boost with boost left is full throttle: sticky too.
        update_wheels(
            &mut car,
            &mut wall_wheels,
            0.0,
            0.0,
            true,
            10.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        assert_eq!(wall_wheels[0].lat_friction, 1.0);
        // A 45° surface, coasting: the curve's midpoint.
        for wheel in wall_wheels.iter_mut() {
            wheel.contact_normal = Vec3::new(-0.7075, 0.0, 0.7075);
        }
        update_wheels(
            &mut car,
            &mut wall_wheels,
            0.0,
            0.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        assert!(
            (wall_wheels[0].lat_friction - 0.5).abs() < 1e-5,
            "{}",
            wall_wheels[0].lat_friction
        );
    }

    #[test]
    fn a_wheel_in_the_air_keeps_its_last_friction_factors() {
        let mut car = car_at(17.0);
        car.linear_velocity = Vec3::new(0.0, 300.0, 0.0);
        let mut wheels = initial_wheels();
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        update_wheels(
            &mut car,
            &mut wheels,
            1.0,
            0.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        let sliding = wheels[0].lat_friction;
        assert!((sliding - 0.2).abs() < 1e-6);
        car.position.z = 200.0;
        raycast_wheels(&car, &mut wheels, &[&ground()], DT);
        assert_eq!(wheels_in_contact(&wheels), 0);
        car.linear_velocity = Vec3::new(1000.0, 0.0, 0.0);
        update_wheels(
            &mut car,
            &mut wheels,
            1.0,
            0.0,
            false,
            0.0,
            false,
            &mut 0.0,
            -650.0,
            DT,
        );
        assert_eq!(wheels[0].lat_friction, sliding, "untouched while airborne");
    }
}
