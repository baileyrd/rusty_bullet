//! The car-ball hit's extra impulse — `RB-PHYSICS-001-FR-083` finding 5, a
//! port of RocketSim's `Ball::_OnHit`: on top of the ordinary contact
//! solve (which the pair's own material, `body::CARBALL_COLLISION_FRICTION`
//! / `CARBALL_COLLISION_RESTITUTION`, governs), real Rocket League adds a
//! velocity to the ball that depends only on where the ball is relative to
//! the car and how fast the two approach — a flattened, forward-biased
//! kick that is most of why a car "pops" the ball. RocketSim computes it in
//! the contact-added callback (from the pre-solve velocities) and adds it
//! to the ball's velocity at the end of the tick, after the solver; the
//! world does the same. At most one such impulse per car every other
//! tick (`tickCount > tickCountWhenExtraImpulseApplied + 1`).

use crate::body::RigidBody;
use crate::drive;
use crate::wheels::piecewise_linear;
use rb_domain::Vec3;

/// `RLConst::BALL_CAR_EXTRA_IMPULSE_Z_SCALE` — the relative position's `z`
/// is scaled by this before the hit direction is normalized, flattening
/// the kick.
pub const BALL_CAR_EXTRA_IMPULSE_Z_SCALE: f32 = 0.35;
/// `RLConst::BALL_CAR_EXTRA_IMPULSE_FORWARD_SCALE` — the hit direction's
/// component along the car's forward is kept at this fraction (the rest is
/// removed) before renormalizing.
pub const BALL_CAR_EXTRA_IMPULSE_FORWARD_SCALE: f32 = 0.65;
/// `RLConst::BALL_CAR_EXTRA_IMPULSE_MAXDELTAVEL_UU` — the relative speed
/// the impulse is computed from is capped here.
pub const BALL_CAR_EXTRA_IMPULSE_MAXDELTAVEL: f32 = 4600.0;
/// `RLConst::BALL_CAR_EXTRA_IMPULSE_FACTOR_CURVE` — the fraction of the
/// (capped) relative speed added to the ball, against that speed.
pub const BALL_CAR_EXTRA_IMPULSE_FACTOR_CURVE: [(f32, f32); 4] =
    [(0.0, 0.65), (500.0, 0.65), (2300.0, 0.55), (4600.0, 0.30)];

/// The velocity `Ball::_OnHit` adds to the ball for a hit by `car`, from
/// the two bodies' current (pre-solve) states: `hitDir = normalize((ball -
/// car) ⊙ (1, 1, Z_SCALE))`, then `normalize(hitDir - forward · (hitDir ·
/// forward) · (1 - FORWARD_SCALE))`, times `min(|v_ball - v_car|,
/// MAXDELTAVEL)` times the factor curve at that speed. Zero when the two
/// bodies do not move relative to each other (or sit on top of each
/// other). The soccar case only: the hoops ground scale is not modeled.
pub fn ball_car_extra_impulse(car: &RigidBody, ball: &RigidBody) -> Vec3 {
    let rel_pos = ball.position - car.position;
    let rel_vel = ball.linear_velocity - car.linear_velocity;
    let rel_speed = rel_vel.length().min(BALL_CAR_EXTRA_IMPULSE_MAXDELTAVEL);
    if rel_speed <= 0.0 {
        return Vec3::ZERO;
    }
    let flattened = Vec3::new(
        rel_pos.x,
        rel_pos.y,
        rel_pos.z * BALL_CAR_EXTRA_IMPULSE_Z_SCALE,
    );
    let Some(hit_dir) = flattened.normalize() else {
        return Vec3::ZERO;
    };
    let forward = drive::forward_axis(car);
    let forward_adjustment =
        forward * (hit_dir.dot(&forward) * (1.0 - BALL_CAR_EXTRA_IMPULSE_FORWARD_SCALE));
    let Some(hit_dir) = (hit_dir - forward_adjustment).normalize() else {
        return Vec3::ZERO;
    };
    hit_dir * (rel_speed * piecewise_linear(&BALL_CAR_EXTRA_IMPULSE_FACTOR_CURVE, rel_speed))
}

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use super::*;

    fn car_facing_x() -> RigidBody {
        RigidBody::standard_car(Vec3::new(0.0, 0.0, 17.0))
    }

    #[test]
    fn a_ball_straight_above_a_still_car_is_popped_straight_up_at_the_curves_fraction() {
        let car = car_facing_x();
        let mut ball = RigidBody::standard_ball(Vec3::new(0.0, 0.0, 130.0));
        ball.linear_velocity = Vec3::new(0.0, 0.0, -400.0);
        let added = ball_car_extra_impulse(&car, &ball);
        // Straight up: no forward component to bias, the z-scale only
        // shortens a vector that is renormalized anyway; 400 uu/s sits on
        // the curve's flat 0.65.
        assert!((added.x).abs() < 1e-4 && (added.y).abs() < 1e-4);
        assert!((added.z - 400.0 * 0.65).abs() < 1e-2, "{added:?}");
    }

    #[test]
    fn the_kick_is_flattened_and_biased_away_from_the_cars_forward() {
        let car = car_facing_x();
        let mut ball = RigidBody::standard_ball(Vec3::new(100.0, 0.0, 117.0));
        ball.linear_velocity = Vec3::new(-1000.0, 0.0, 0.0);
        let added = ball_car_extra_impulse(&car, &ball);
        // Raw direction (100, 0, 100)/√2 = 45° up; z scaled to 35 → 19.3°
        // up; then 35% of the forward (x) component removed → steeper
        // again but well under 45°, and the magnitude is the curve's
        // fraction of the relative speed.
        let expected_magnitude =
            1000.0 * piecewise_linear(&BALL_CAR_EXTRA_IMPULSE_FACTOR_CURVE, 1000.0);
        assert!(
            (added.length() - expected_magnitude).abs() < 1e-2,
            "{added:?}"
        );
        let raw = Vec3::new(100.0, 0.0, 35.0).normalize().expect("non-zero");
        let adjusted = Vec3::new(raw.x * 0.65, 0.0, raw.z)
            .normalize()
            .expect("non-zero");
        let direction = added.normalize().expect("non-zero");
        assert!((direction.x - adjusted.x).abs() < 1e-4 && (direction.z - adjusted.z).abs() < 1e-4);
        assert!(
            direction.z < 0.707,
            "flatter than the raw 45°: {direction:?}"
        );
        assert!(added.x > 0.0 && added.z > 0.0);
    }

    #[test]
    fn the_relative_speed_is_capped_and_the_curve_falls_to_a_third_at_the_cap() {
        let car = car_facing_x();
        let mut ball = RigidBody::standard_ball(Vec3::new(0.0, 100.0, 60.0));
        ball.linear_velocity = Vec3::new(0.0, 9000.0, 0.0);
        let added = ball_car_extra_impulse(&car, &ball);
        assert!((added.length() - 4600.0 * 0.30).abs() < 1e-1, "{added:?}");
        // Sideways: no forward component at all, so only the z-scale
        // shapes the direction (the ball sits 43 uu above the car's
        // origin at z = 17).
        let expected = Vec3::new(0.0, 100.0, 43.0 * 0.35)
            .normalize()
            .expect("non-zero");
        let direction = added.normalize().expect("non-zero");
        assert!((direction.y - expected.y).abs() < 1e-4 && (direction.z - expected.z).abs() < 1e-4);
    }

    #[test]
    fn no_relative_motion_means_no_kick() {
        let car = car_facing_x();
        let ball = RigidBody::standard_ball(Vec3::new(120.0, 0.0, 100.0));
        assert_eq!(ball_car_extra_impulse(&car, &ball), Vec3::ZERO);
        let mut moving_together = car;
        moving_together.linear_velocity = Vec3::new(500.0, 0.0, 0.0);
        let mut ball_along = ball;
        ball_along.linear_velocity = Vec3::new(500.0, 0.0, 0.0);
        assert_eq!(
            ball_car_extra_impulse(&moving_together, &ball_along),
            Vec3::ZERO
        );
    }

    #[test]
    fn the_factor_curve_matches_rocketsims_breakpoints() {
        let curve = &BALL_CAR_EXTRA_IMPULSE_FACTOR_CURVE;
        assert_eq!(piecewise_linear(curve, 0.0), 0.65);
        assert_eq!(piecewise_linear(curve, 500.0), 0.65);
        assert!((piecewise_linear(curve, 2300.0) - 0.55).abs() < 1e-6);
        assert!((piecewise_linear(curve, 4600.0) - 0.30).abs() < 1e-6);
        assert!((piecewise_linear(curve, 1400.0) - 0.60).abs() < 1e-6);
    }
}
