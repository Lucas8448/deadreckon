//! 3D missile guidance simulator built on top of `physics_sandbox`.
//!
//! `physics_sandbox` provides the rigid-body dynamics + RK4 integrator +
//! environment. We layer:
//!   * Proportional-navigation guidance (3D true PN)
//!   * Target maneuvering models
//!   * Seeker model with sensor noise
//!   * Engagement bookkeeping (range, closing speed, LOS rate, kill check)
//!   * Monte Carlo sweeps (see [`monte_carlo`]) and scenario presets
//!     (see [`scenario`]).
//!
//! World convention (matches `physics_sandbox`): Y is up, X is downrange,
//! Z is cross-range. Distances in meters, time in seconds, mass in kg.

use physics_sandbox::{
    World,
    dynamics::RigidBody,
    environment::Environment,
    integrator::RK4Integrator,
    math::Vec3,
};

pub mod monte_carlo;
pub mod scenario;
pub use monte_carlo::{Perturbations, SweepConfig};
pub use scenario::Scenario;

/// Re-export so callers don't need to depend on `physics_sandbox` directly.
pub use physics_sandbox::math::Vec3 as V3;

#[derive(Clone, Copy, Debug, Default)]
pub enum TargetManeuver {
    #[default]
    ConstantVelocity,
    /// Steady turn around `axis` at angular rate `omega` (rad/s). Velocity
    /// vector rotates; speed is preserved.
    ConstantTurn { axis: Vec3, omega: f64 },
    /// Constant acceleration vector `a` applied between `start_t` and `end_t`.
    AccelBurst { start_t: f64, end_t: f64, a: Vec3 },
    /// Sinusoidal maneuver perpendicular to the current velocity, in the
    /// plane defined by velocity × `axis`. Peak |a| = `amplitude` m/s².
    Weave { axis: Vec3, amplitude: f64, freq: f64 },
}

#[derive(Clone, Copy, Debug)]
pub struct Missile {
    pub p: Vec3,
    pub v: Vec3,
    pub mass: f64,
    /// Lateral acceleration cap (m/s²).
    pub a_max: f64,
    /// Max look-angle off the velocity vector before the seeker breaks lock (rad).
    pub gimbal_limit: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Target {
    pub p: Vec3,
    pub v: Vec3,
    pub mass: f64,
    pub maneuver: TargetManeuver,
}

#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub dt: f64,
    pub nav_const: f64,
    pub kill_radius: f64,
    pub max_time: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SensorNoise {
    /// 1σ of per-step gaussian white noise on observed range (m). Resampled
    /// every `step()`.
    pub range_std: f64,
    /// 1σ of per-step gaussian white noise on observed closing speed (m/s).
    pub range_rate_std: f64,
    /// 1σ of per-step gaussian white noise applied as a perpendicular
    /// displacement to the LOS unit vector before re-normalization.
    /// Note: the perpendicular kick uses two independent gaussians (one per
    /// basis axis), so the realised angular error has standard deviation
    /// ≈ √2 · `angle_std` for small angles.
    pub angle_std: f64,
    /// 1σ of per-step gaussian white noise on each component of the LOS
    /// rate vector (rad/s). The radial component is harmless (gets killed
    /// by the subsequent × r̂ in PN) but inflates apparent variance.
    pub angle_rate_std: f64,
}

impl SensorNoise {
    pub fn perfect() -> Self {
        Self::default()
    }

    pub fn realistic() -> Self {
        Self {
            range_std: 10.0,
            range_rate_std: 2.0,
            angle_std: 0.002,
            angle_rate_std: 0.001,
        }
    }

    pub fn degraded() -> Self {
        Self {
            range_std: 50.0,
            range_rate_std: 10.0,
            angle_std: 0.01,
            angle_rate_std: 0.005,
        }
    }

    pub fn extreme() -> Self {
        Self {
            range_std: 800.0,
            range_rate_std: 150.0,
            angle_std: 0.2,
            angle_rate_std: 0.1,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ObservedState {
    pub range: f64,
    pub closing_speed: f64,
    /// Observed unit vector along the line of sight (missile -> target).
    pub r_hat: Vec3,
    /// Observed LOS rate vector (rad/s, world frame).
    pub los_rate: Vec3,
}

impl Default for ObservedState {
    fn default() -> Self {
        Self {
            range: 0.0,
            closing_speed: 0.0,
            r_hat: Vec3::new(1.0, 0.0, 0.0),
            los_rate: Vec3::zero(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Telemetry {
    pub t: f64,
    pub range: f64,
    pub closing_speed: f64,
    /// Magnitude of the true LOS rate vector (rad/s).
    pub los_rate: f64,
    /// Magnitude of the commanded lateral acceleration this step (m/s²).
    pub a_cmd: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Running,
    Hit,
    Timeout,
    LostLock,
}

pub struct Sim {
    world: World,
    missile_id: usize,
    target_id: usize,
    missile_def: Missile,
    target_def: Target,
    pub params: Params,
    pub sensor_noise: SensorNoise,
    pub t: f64,
    pub last: Telemetry,
    pub observed: ObservedState,
    rng_state: u64,
    /// Number of consecutive steps the seeker has been out of gimbal limit.
    /// Triggers `LostLock` after a small persistence window.
    out_of_gimbal_steps: u32,
}

const LOST_LOCK_PERSISTENCE_STEPS: u32 = 10;

fn vec3_zero() -> Vec3 {
    Vec3::zero()
}

/// Pick an arbitrary unit vector perpendicular to `v`. Robust for any input.
fn any_perp(v: Vec3) -> Vec3 {
    let abs_x = v.x.abs();
    let abs_y = v.y.abs();
    let abs_z = v.z.abs();
    let helper = if abs_x <= abs_y && abs_x <= abs_z {
        Vec3::new(1.0, 0.0, 0.0)
    } else if abs_y <= abs_z {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(0.0, 0.0, 1.0)
    };
    let perp = v.cross(helper);
    if perp.magnitude_sq() < 1e-18 {
        // v was (anti-)parallel to helper — fall back to another axis.
        v.cross(Vec3::new(0.0, 0.0, 1.0))
            .normalize()
    } else {
        perp.normalize()
    }
}

impl Sim {
    pub fn new(missile: Missile, target: Target, params: Params) -> Self {
        Self::with_sensor_noise(missile, target, params, SensorNoise::perfect(), 0)
    }

    pub fn with_sensor_noise(
        missile: Missile,
        target: Target,
        params: Params,
        sensor_noise: SensorNoise,
        seed: u64,
    ) -> Self {
        // No gravity, no atmosphere — preserves the original "ballistic frames
        // ignored over short engagement" semantics. Scenarios that want
        // gravity/drag can be added later by swapping the environment.
        let env = Environment::space();
        let mut world = World::new(env, RK4Integrator);

        let m_body = RigidBody::new(missile.mass)
            .with_position(missile.p)
            .with_velocity(missile.v);
        let t_body = RigidBody::new(target.mass)
            .with_position(target.p)
            .with_velocity(target.v);

        let missile_id = world.add_body(m_body);
        let target_id = world.add_body(t_body);

        let mut sim = Self {
            world,
            missile_id,
            target_id,
            missile_def: missile,
            target_def: target,
            params,
            sensor_noise,
            t: 0.0,
            last: Telemetry {
                t: 0.0,
                range: 0.0,
                closing_speed: 0.0,
                los_rate: 0.0,
                a_cmd: 0.0,
            },
            observed: ObservedState::default(),
            rng_state: seed.wrapping_add(0x9E37_79B9_7F4A_7C15),
            out_of_gimbal_steps: 0,
        };
        sim.recompute_telemetry(0.0);
        sim
    }

    pub fn missile_pos(&self) -> Vec3 {
        self.world.body(self.missile_id).position
    }

    pub fn missile_vel(&self) -> Vec3 {
        self.world.body(self.missile_id).velocity
    }

    pub fn target_pos(&self) -> Vec3 {
        self.world.body(self.target_id).position
    }

    pub fn target_vel(&self) -> Vec3 {
        self.world.body(self.target_id).velocity
    }

    fn next_uniform(&mut self) -> f64 {
        // SplitMix-ish LCG, same constants as original 2D version.
        self.rng_state = self
            .rng_state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.rng_state >> 11) as f64) / ((1u64 << 53) as f64)
    }

    fn next_gaussian(&mut self) -> f64 {
        let u1 = self.next_uniform().max(1e-12);
        let u2 = self.next_uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }

    fn next_gaussian_vec3(&mut self) -> Vec3 {
        Vec3::new(self.next_gaussian(), self.next_gaussian(), self.next_gaussian())
    }

    fn observe(&mut self) -> ObservedState {
        let m = self.missile_pos();
        let mv = self.missile_vel();
        let tp = self.target_pos();
        let tv = self.target_vel();

        let r = tp - m;
        let range_true = r.magnitude().max(1e-6);
        let r_hat_true = r * (1.0 / range_true);
        let v_rel = tv - mv;
        let closing_true = -v_rel.dot(r_hat_true);
        let los_rate_true = r.cross(v_rel) * (1.0 / (range_true * range_true));

        // Range / range-rate: scalar gaussian noise.
        let range = (range_true + self.next_gaussian() * self.sensor_noise.range_std).max(1.0);
        let closing_speed = closing_true + self.next_gaussian() * self.sensor_noise.range_rate_std;

        // Bearing noise: kick r_hat by a random vector in the plane perpendicular
        // to it, with magnitude ~ N(0, angle_std). Re-normalize to stay a unit vector.
        let perp_basis_a = any_perp(r_hat_true);
        let perp_basis_b = r_hat_true.cross(perp_basis_a).normalize();
        let kick = perp_basis_a * (self.next_gaussian() * self.sensor_noise.angle_std)
            + perp_basis_b * (self.next_gaussian() * self.sensor_noise.angle_std);
        // Guard against catastrophic cancellation: if the noise sample puts us
        // near the origin (extreme σ or unlucky draw), fall back to the true LOS
        // rather than NaN-poisoning the rest of the step.
        let kicked = r_hat_true + kick;
        let r_hat = if kicked.magnitude_sq() > 1e-12 {
            kicked.normalize()
        } else {
            r_hat_true
        };

        // LOS-rate noise: add gaussian to each component (rad/s).
        let los_rate = los_rate_true
            + self.next_gaussian_vec3() * self.sensor_noise.angle_rate_std;

        ObservedState {
            range,
            closing_speed,
            r_hat,
            los_rate,
        }
    }

    pub fn step(&mut self) -> Status {
        if self.t >= self.params.max_time {
            return Status::Timeout;
        }

        // Kill check on true geometry (sensor noise must not cause / prevent hits).
        let range_true = (self.target_pos() - self.missile_pos()).magnitude();
        if range_true <= self.params.kill_radius {
            return Status::Hit;
        }

        // ------------------- Seeker observation -------------------
        self.observed = self.observe();

        // ------------------- Guidance -------------------
        let mv = self.missile_vel();
        let speed = mv.magnitude().max(1e-6);
        let v_hat = mv * (1.0 / speed);

        // Look-angle: between missile velocity and observed LOS.
        let cos_look = v_hat.dot(self.observed.r_hat).clamp(-1.0, 1.0);
        let look_angle = cos_look.acos();

        let mut a_cmd_vec = vec3_zero();
        let mut a_cmd_mag = 0.0;
        let mut lost_lock_this_step = false;

        if look_angle > self.missile_def.gimbal_limit {
            self.out_of_gimbal_steps = self.out_of_gimbal_steps.saturating_add(1);
            if self.out_of_gimbal_steps >= LOST_LOCK_PERSISTENCE_STEPS {
                lost_lock_this_step = true;
            }
        } else {
            self.out_of_gimbal_steps = 0;

            // True PN (3D): a = N · Vc · (ω_los × r_hat). Always perpendicular
            // to the observed LOS. We then project onto the plane perpendicular
            // to missile velocity so the command can't change speed (the body
            // has no thrust model; lateral force only).
            let raw = self.observed.los_rate.cross(self.observed.r_hat)
                * (self.params.nav_const * self.observed.closing_speed);

            // Project out any along-velocity component.
            let along = v_hat * raw.dot(v_hat);
            let perp = raw - along;

            let mag = perp.magnitude();
            if mag > 1e-9 {
                let clamped = mag.min(self.missile_def.a_max);
                a_cmd_vec = perp * (clamped / mag);
                a_cmd_mag = clamped;
            }
        }

        // Apply lateral force on the missile body (force = mass · accel).
        // physics_sandbox::Integrator::step calls body.clear_forces() at the
        // end of every step, so this command is transient (single-step load),
        // not accumulating. See physics_sandbox/src/integrator/mod.rs.
        let missile_force = a_cmd_vec * self.missile_def.mass;
        self.world
            .body_mut(self.missile_id)
            .apply_force(missile_force, None);

        // ------------------- Target maneuver -------------------
        // For ConstantTurn we directly rotate velocity (analytic, no force).
        // For AccelBurst / Weave we apply forces and let the integrator do it.
        match self.target_def.maneuver {
            TargetManeuver::ConstantVelocity => {}
            TargetManeuver::ConstantTurn { axis, omega } => {
                let axis_mag_sq = axis.magnitude_sq();
                if axis_mag_sq < 1e-12 {
                    // Zero-axis is a degenerate config; behave as ConstantVelocity
                    // rather than NaN-ing the body.
                } else {
                    let axis_n = axis * (1.0 / axis_mag_sq.sqrt());
                    let theta = omega * self.params.dt;
                    let (s, c) = theta.sin_cos();
                    let v = self.target_vel();
                    // Rodrigues: v' = v cosθ + (k×v) sinθ + k (k·v)(1−cosθ).
                    // NB: this directly mutates body.velocity, bypassing the
                    // integrator. It's a kinematic override — documented and
                    // intentional (a centripetal-force formulation would also
                    // need to track radius and is overkill for this sim).
                    let v_rot = v * c
                        + axis_n.cross(v) * s
                        + axis_n * (axis_n.dot(v) * (1.0 - c));
                    let body = self.world.body_mut(self.target_id);
                    body.velocity = v_rot;
                }
            }
            TargetManeuver::AccelBurst { start_t, end_t, a } => {
                if self.t >= start_t && self.t <= end_t {
                    let force = a * self.target_def.mass;
                    self.world
                        .body_mut(self.target_id)
                        .apply_force(force, None);
                }
            }
            TargetManeuver::Weave {
                axis,
                amplitude,
                freq,
            } => {
                let axis_mag_sq = axis.magnitude_sq();
                if axis_mag_sq >= 1e-12 {
                    let tv = self.target_vel();
                    let speed = tv.magnitude();
                    if speed > 1e-6 {
                        let v_hat = tv * (1.0 / speed);
                        let axis_n = axis * (1.0 / axis_mag_sq.sqrt());
                        // Component of the maneuver axis perpendicular to velocity:
                        let perp_dir_raw = axis_n - v_hat * axis_n.dot(v_hat);
                        let len = perp_dir_raw.magnitude();
                        if len > 1e-6 {
                            let perp_dir = perp_dir_raw * (1.0 / len);
                            let mag = amplitude
                                * (2.0 * std::f64::consts::PI * freq * self.t).sin();
                            let force = perp_dir * (mag * self.target_def.mass);
                            self.world
                                .body_mut(self.target_id)
                                .apply_force(force, None);
                        }
                    }
                }
                // Zero-axis: silently degenerates to ConstantVelocity.
            }
        }

        // Advance physics.
        self.world.step(self.params.dt);
        self.t += self.params.dt;

        self.recompute_telemetry(a_cmd_mag);

        if lost_lock_this_step {
            return Status::LostLock;
        }

        // Re-check kill after the step (we may have crossed inside the radius
        // during the integration).
        let range_after = (self.target_pos() - self.missile_pos()).magnitude();
        if range_after <= self.params.kill_radius {
            return Status::Hit;
        }

        Status::Running
    }

    fn recompute_telemetry(&mut self, a_cmd_mag: f64) {
        let r = self.target_pos() - self.missile_pos();
        let range = r.magnitude().max(1e-6);
        let r_hat = r * (1.0 / range);
        let v_rel = self.target_vel() - self.missile_vel();
        let closing_speed = -v_rel.dot(r_hat);
        let los_rate_vec = r.cross(v_rel) * (1.0 / (range * range));

        self.last = Telemetry {
            t: self.t,
            range,
            closing_speed,
            los_rate: los_rate_vec.magnitude(),
            a_cmd: a_cmd_mag,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn baseline_engagement() -> (Missile, Target, Params) {
        (
            Missile {
                p: Vec3::new(0.0, 0.0, 0.0),
                v: Vec3::new(250.0, 0.0, 0.0),
                mass: 100.0,
                a_max: 80.0,
                gimbal_limit: 0.7,
            },
            Target {
                p: Vec3::new(6000.0, 1500.0, 0.0),
                v: Vec3::new(-180.0, 0.0, 0.0),
                mass: 5_000.0,
                maneuver: TargetManeuver::ConstantVelocity,
            },
            Params {
                dt: 0.02,
                nav_const: 4.0,
                kill_radius: 15.0,
                max_time: 60.0,
            },
        )
    }

    #[test]
    fn constant_velocity_target_is_intercepted() {
        let (m, t, p) = baseline_engagement();
        let mut sim = Sim::new(m, t, p);
        let mut status = Status::Running;
        while status == Status::Running {
            status = sim.step();
        }
        assert_eq!(status, Status::Hit, "got {:?} (final range {:.2}m)", status, sim.last.range);
    }

    #[test]
    fn perp_helper_returns_orthogonal_unit_vector() {
        for v in [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0).normalize(),
            Vec3::new(-3.0, 2.0, 0.5).normalize(),
        ] {
            let p = any_perp(v);
            assert!((p.magnitude() - 1.0).abs() < 1e-9, "not unit: {:?}", p);
            assert!(v.dot(p).abs() < 1e-9, "not perp: v={:?} p={:?}", v, p);
        }
    }
}
