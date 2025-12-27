use glam::Vec2;

pub mod monte_carlo;
pub mod scenario;
pub use scenario::Scenario;
pub use monte_carlo::{SweepConfig, Perturbations};

#[derive(Clone, Copy, Debug, Default)]
pub enum TargetManeuver {
    #[default]
    ConstantVelocity,
    ConstantTurn { omega: f32 },
    AccelBurst { start_t: f32, end_t: f32, a: Vec2 },
    Weave { amplitude: f32, freq: f32 },
}

#[derive(Clone, Copy, Debug)]
pub struct Missile {
    pub p: Vec2,
    pub v: Vec2,
    pub a_max: f32,
    pub gimbal_limit: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Target {
    pub p: Vec2,
    pub v: Vec2,
    pub maneuver: TargetManeuver,
}

#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub dt: f32,
    pub nav_const: f32,
    pub kill_radius: f32,
    pub max_time: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SensorNoise {
    pub range_std: f32,
    pub range_rate_std: f32,
    pub angle_std: f32,
    pub angle_rate_std: f32,
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

#[derive(Clone, Copy, Debug, Default)]
pub struct ObservedState {
    pub range: f32,
    pub closing_speed: f32,
    pub los_angle: f32,
    pub los_rate: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Telemetry {
    pub t: f32,
    pub range: f32,
    pub closing_speed: f32,
    pub los_rate: f32,
    pub a_cmd: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Sim {
    pub missile: Missile,
    pub target: Target,
    pub params: Params,
    pub sensor_noise: SensorNoise,
    pub t: f32,
    pub last: Telemetry,
    pub observed: ObservedState,
    rng_state: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Running,
    Hit,
    Timeout,
    LostLock,
}

fn perp(v: Vec2) -> Vec2 {
    // 90° rotation (right-hand)
    Vec2::new(-v.y, v.x)
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
        let mut sim = Self {
            missile,
            target,
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
            rng_state: seed,
        };
        sim.recompute_telemetry(0.0);
        sim
    }

    fn next_gaussian(&mut self) -> f32 {
        self.rng_state = self.rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let u1 = ((self.rng_state >> 32) as u32) as f32 / (u32::MAX as f32);
        self.rng_state = self.rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let u2 = ((self.rng_state >> 32) as u32) as f32 / (u32::MAX as f32);
        let u1 = u1.max(1e-10);
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos()
    }

    fn observe_target(&mut self) -> ObservedState {
        let r = self.target.p - self.missile.p;
        let range_true = r.length().max(1e-6);
        let r_hat = r / range_true;
        let v_rel = self.target.v - self.missile.v;
        let closing_speed_true = -(v_rel.dot(r_hat));
        let los_angle_true = r.y.atan2(r.x);
        let los_rate_true = (r.x * v_rel.y - r.y * v_rel.x) / (range_true * range_true);

        let range = range_true + self.next_gaussian() * self.sensor_noise.range_std;
        let closing_speed = closing_speed_true + self.next_gaussian() * self.sensor_noise.range_rate_std;
        let los_angle = los_angle_true + self.next_gaussian() * self.sensor_noise.angle_std;
        let los_rate = los_rate_true + self.next_gaussian() * self.sensor_noise.angle_rate_std;

        ObservedState {
            range: range.max(1.0),
            closing_speed,
            los_angle,
            los_rate,
        }
    }

    pub fn step(&mut self) -> Status {
        if self.t >= self.params.max_time {
            return Status::Timeout;
        }

        let dt = self.params.dt;

        let r = self.target.p - self.missile.p;
        let range = r.length();

        if range <= self.params.kill_radius {
            return Status::Hit;
        }

        self.observed = self.observe_target();

        let r = self.target.p - self.missile.p;
        let v_hat = self.missile.v.normalize_or_zero();
        let r_hat = r.normalize_or_zero();
        let look_angle = v_hat.dot(r_hat).acos();

        let a_cmd = if look_angle > self.missile.gimbal_limit {
            0.0
        } else {
            let cmd = self.params.nav_const * self.observed.closing_speed * self.observed.los_rate;
            cmd.clamp(-self.missile.a_max, self.missile.a_max)
        };

        let a_vec = perp(v_hat) * a_cmd;

        self.missile.v += a_vec * dt;
        self.missile.p += self.missile.v * dt;

        // Apply target maneuver
        match self.target.maneuver {
            TargetManeuver::ConstantVelocity => {}
            TargetManeuver::ConstantTurn { omega } => {
                // Rotate velocity by omega * dt
                let (sin, cos) = (omega * dt).sin_cos();
                let vx = self.target.v.x * cos - self.target.v.y * sin;
                let vy = self.target.v.x * sin + self.target.v.y * cos;
                self.target.v = Vec2::new(vx, vy);
            }
            TargetManeuver::AccelBurst { start_t, end_t, a } => {
                if self.t >= start_t && self.t <= end_t {
                    self.target.v += a * dt;
                }
            }
            TargetManeuver::Weave { amplitude, freq } => {
                // a(t) = amplitude * sin(2π * freq * t)
                let a_mag = amplitude * (2.0 * std::f32::consts::PI * freq * self.t).sin();
                let v_hat = self.target.v.normalize_or_zero();
                let a_perp = perp(v_hat) * a_mag;
                self.target.v += a_perp * dt;
            }
        }

        self.target.p += self.target.v * dt;

        self.t += dt;
        self.recompute_telemetry(a_cmd);

        Status::Running
    }

    fn recompute_telemetry(&mut self, a_cmd: f32) {
        let r = self.target.p - self.missile.p;
        let range = r.length().max(1e-6);
        let r_hat = r / range;

        let v_rel = self.target.v - self.missile.v;
        let closing_speed = -(v_rel.dot(r_hat));
        let los_rate = (r.x * v_rel.y - r.y * v_rel.x) / (range * range);

        self.last = Telemetry {
            t: self.t,
            range,
            closing_speed,
            los_rate,
            a_cmd,
        };
    }
}