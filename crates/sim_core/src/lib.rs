use glam::Vec2;

pub mod scenario;
pub use scenario::Scenario;

#[derive(Clone, Copy, Debug)]
pub struct Missile {
    pub p: Vec2,      // position (m)
    pub v: Vec2,      // velocity (m/s)
    pub a_max: f32,   // max lateral acceleration (m/s^2)
}

#[derive(Clone, Copy, Debug)]
pub struct Target {
    pub p: Vec2,
    pub v: Vec2,
}

#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub dt: f32,          // seconds
    pub nav_const: f32,   // PN constant N
    pub kill_radius: f32, // meters
    pub max_time: f32,    // seconds
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
    pub t: f32,
    pub last: Telemetry,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Running,
    Hit,
    Timeout,
}

fn perp(v: Vec2) -> Vec2 {
    // 90° rotation (right-hand)
    Vec2::new(-v.y, v.x)
}

impl Sim {
    pub fn new(missile: Missile, target: Target, params: Params) -> Self {
        let mut sim = Self {
            missile,
            target,
            params,
            t: 0.0,
            last: Telemetry {
                t: 0.0,
                range: 0.0,
                closing_speed: 0.0,
                los_rate: 0.0,
                a_cmd: 0.0,
            },
        };
        sim.recompute_telemetry(0.0);
        sim
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

        let r_len = range.max(1e-6);
        let r2 = r_len * r_len;
        let r_hat = r / r_len;

        let v_rel = self.target.v - self.missile.v;

        // Vc = -v_rel · r̂
        let closing_speed = -(v_rel.dot(r_hat));

        // λ̇ = (r × v_rel) / |r|²
        let los_rate = (r.x * v_rel.y - r.y * v_rel.x) / r2;

        // a = N · Vc · λ̇
        let mut a_cmd = self.params.nav_const * closing_speed * los_rate;
        a_cmd = a_cmd.clamp(-self.missile.a_max, self.missile.a_max);

        let v_hat = self.missile.v.normalize_or_zero();
        let a_vec = perp(v_hat) * a_cmd;

        self.missile.v += a_vec * dt;
        self.missile.p += self.missile.v * dt;

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