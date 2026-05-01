//! Monte Carlo dispersion sweep over a [`Scenario`]. Same statistical surface
//! as the original 2D version, ported to 3D perturbations.

use physics_sandbox::math::Vec3;
use rand::Rng;

use crate::{Scenario, SensorNoise, Sim, Status};

#[derive(Clone, Copy, Debug)]
pub struct TrialResult {
    pub hit: bool,
    pub miss_distance: f64,
    pub time: f64,
}

#[derive(Clone, Debug)]
pub struct MonteCarloResult {
    pub trials: usize,
    pub hits: usize,
    pub hit_rate: f64,
    pub miss_distances: Vec<f64>,
    pub mean_miss: f64,
    pub std_miss: f64,
    pub min_miss: f64,
    pub max_miss: f64,
    pub p50_miss: f64,
    pub p90_miss: f64,
    pub p99_miss: f64,
}

#[derive(Clone, Debug)]
pub struct SweepConfig {
    pub scenario: Scenario,
    pub trials: usize,
    pub seed: u64,
    pub perturb: Perturbations,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Perturbations {
    /// 1σ box half-width on each axis of the target initial position (m).
    pub target_pos: f64,
    /// 1σ box half-width on the target speed magnitude (m/s).
    pub target_vel: f64,
    /// 1σ box half-width on the target heading direction (rad). Applied as
    /// a rotation around a random axis perpendicular to current velocity.
    pub target_heading: f64,
    /// ± window on `params.nav_const`.
    pub nav_const: f64,
    /// ± window on `missile.a_max`.
    pub a_max: f64,
    pub sensor_noise: SensorNoise,
}

impl SweepConfig {
    pub fn new(scenario: Scenario, trials: usize) -> Self {
        Self {
            scenario,
            trials,
            seed: 42,
            perturb: Perturbations {
                target_pos: 500.0,
                target_vel: 30.0,
                target_heading: 0.2,
                nav_const: 0.0,
                a_max: 0.0,
                sensor_noise: SensorNoise::default(),
            },
        }
    }

    pub fn with_seed(mut self, seed: u64) -> Self {
        self.seed = seed;
        self
    }

    pub fn with_nav_const_sweep(mut self, delta: f64) -> Self {
        self.perturb.nav_const = delta;
        self
    }

    pub fn with_a_max_sweep(mut self, delta: f64) -> Self {
        self.perturb.a_max = delta;
        self
    }

    pub fn with_sensor_noise(mut self, noise: SensorNoise) -> Self {
        self.perturb.sensor_noise = noise;
        self
    }
}

/// Pick a random unit vector perpendicular to `v`. Used to sample an
/// arbitrary axis when rotating heading by a small angle so the perturbation
/// covers the full 3D heading cone, not a fixed plane.
fn random_perp<R: Rng>(v: Vec3, rng: &mut R) -> Vec3 {
    let v_mag_sq = v.magnitude_sq();
    if v_mag_sq < 1e-18 {
        return Vec3::new(0.0, 1.0, 0.0);
    }
    let v_hat = v * (1.0 / v_mag_sq.sqrt());
    // Try up to a few times to draw a random vector that's not (nearly)
    // parallel to v_hat. Practically always succeeds on the first try.
    for _ in 0..8 {
        let r = Vec3::new(
            rng.random_range(-1.0..1.0),
            rng.random_range(-1.0..1.0),
            rng.random_range(-1.0..1.0),
        );
        let perp = r - v_hat * r.dot(v_hat);
        let mag_sq = perp.magnitude_sq();
        if mag_sq > 1e-6 {
            return perp * (1.0 / mag_sq.sqrt());
        }
    }
    // Deterministic fallback (extremely unlikely path):
    any_perp(v_hat)
}

/// Pick an arbitrary unit vector perpendicular to `v`. Deterministic; used
/// only as a last-ditch fallback in [`random_perp`].
fn any_perp(v: Vec3) -> Vec3 {
    let helper = if v.x.abs() <= v.y.abs() && v.x.abs() <= v.z.abs() {
        Vec3::new(1.0, 0.0, 0.0)
    } else if v.y.abs() <= v.z.abs() {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(0.0, 0.0, 1.0)
    };
    let p = v.cross(helper);
    if p.magnitude_sq() < 1e-18 {
        v.cross(Vec3::new(0.0, 0.0, 1.0)).normalize()
    } else {
        p.normalize()
    }
}

/// Rodrigues rotation of `v` around unit `axis` by `theta` radians.
fn rotate_around(v: Vec3, axis: Vec3, theta: f64) -> Vec3 {
    let (s, c) = theta.sin_cos();
    v * c + axis.cross(v) * s + axis * (axis.dot(v) * (1.0 - c))
}

fn run_trial<R: Rng>(scenario: &Scenario, perturb: &Perturbations, rng: &mut R) -> TrialResult {
    let mut missile = scenario.missile;
    let mut target = scenario.target;
    let mut params = scenario.params;

    if perturb.target_pos > 0.0 {
        target.p = Vec3::new(
            target.p.x + rng.random_range(-perturb.target_pos..perturb.target_pos),
            target.p.y + rng.random_range(-perturb.target_pos..perturb.target_pos),
            target.p.z + rng.random_range(-perturb.target_pos..perturb.target_pos),
        );
    }

    if perturb.target_vel > 0.0 {
        let speed = target.v.magnitude();
        if speed > 1e-6 {
            let new_speed = (speed
                + rng.random_range(-perturb.target_vel..perturb.target_vel))
            .max(10.0);
            target.v = target.v.normalize() * new_speed;
        }
    }

    if perturb.target_heading > 0.0 {
        let speed = target.v.magnitude();
        if speed > 1e-6 {
            let axis = random_perp(target.v, rng);
            let theta = rng.random_range(-perturb.target_heading..perturb.target_heading);
            target.v = rotate_around(target.v, axis, theta);
        }
    }

    if perturb.nav_const > 0.0 {
        params.nav_const += rng.random_range(-perturb.nav_const..perturb.nav_const);
        params.nav_const = params.nav_const.max(1.0);
    }

    if perturb.a_max > 0.0 {
        missile.a_max += rng.random_range(-perturb.a_max..perturb.a_max);
        missile.a_max = missile.a_max.max(10.0);
    }

    let sensor_seed = rng.random::<u64>();
    let mut sim = Sim::with_sensor_noise(missile, target, params, perturb.sensor_noise, sensor_seed);

    let mut status = Status::Running;
    while status == Status::Running {
        status = sim.step();
    }

    TrialResult {
        hit: status == Status::Hit,
        miss_distance: sim.last.range,
        time: sim.t,
    }
}

pub fn run_sweep(config: &SweepConfig) -> MonteCarloResult {
    use rand::SeedableRng;
    if config.trials == 0 {
        // Empty sweep: return a zeroed result instead of panicking on
        // `0 / 0` and out-of-bounds percentile indexing.
        return MonteCarloResult {
            trials: 0,
            hits: 0,
            hit_rate: 0.0,
            miss_distances: Vec::new(),
            mean_miss: 0.0,
            std_miss: 0.0,
            min_miss: 0.0,
            max_miss: 0.0,
            p50_miss: 0.0,
            p90_miss: 0.0,
            p99_miss: 0.0,
        };
    }
    let mut rng = rand::rngs::StdRng::seed_from_u64(config.seed);

    let mut results: Vec<TrialResult> = Vec::with_capacity(config.trials);
    for _ in 0..config.trials {
        results.push(run_trial(&config.scenario, &config.perturb, &mut rng));
    }

    let hits = results.iter().filter(|r| r.hit).count();
    let hit_rate = hits as f64 / config.trials as f64;

    let mut miss_distances: Vec<f64> = results.iter().map(|r| r.miss_distance).collect();
    miss_distances.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let mean_miss = miss_distances.iter().sum::<f64>() / miss_distances.len() as f64;
    let variance = miss_distances
        .iter()
        .map(|d| (d - mean_miss).powi(2))
        .sum::<f64>()
        / miss_distances.len() as f64;
    let std_miss = variance.sqrt();

    let percentile = |p: f64| -> f64 {
        let idx = ((miss_distances.len() as f64 - 1.0) * p).round() as usize;
        miss_distances[idx.min(miss_distances.len() - 1)]
    };

    MonteCarloResult {
        trials: config.trials,
        hits,
        hit_rate,
        mean_miss,
        std_miss,
        min_miss: miss_distances.first().copied().unwrap_or(0.0),
        max_miss: miss_distances.last().copied().unwrap_or(0.0),
        p50_miss: percentile(0.5),
        p90_miss: percentile(0.9),
        p99_miss: percentile(0.99),
        miss_distances,
    }
}

impl MonteCarloResult {
    pub fn print_report(&self) {
        println!("╔═══════════════════════════════════════╗");
        println!("║       MONTE CARLO RESULTS             ║");
        println!("╠═══════════════════════════════════════╣");
        println!("║ Trials:     {:>6}                    ║", self.trials);
        println!("║ Hits:       {:>6}                    ║", self.hits);
        println!(
            "║ Hit Rate:   {:>6.1}%                   ║",
            self.hit_rate * 100.0
        );
        println!("╠═══════════════════════════════════════╣");
        println!("║ Miss Distance Statistics (m)          ║");
        println!("║   Mean:     {:>8.2}                  ║", self.mean_miss);
        println!("║   Std:      {:>8.2}                  ║", self.std_miss);
        println!("║   Min:      {:>8.2}                  ║", self.min_miss);
        println!("║   Max:      {:>8.2}                  ║", self.max_miss);
        println!("║   P50:      {:>8.2}                  ║", self.p50_miss);
        println!("║   P90:      {:>8.2}                  ║", self.p90_miss);
        println!("║   P99:      {:>8.2}                  ║", self.p99_miss);
        println!("╚═══════════════════════════════════════╝");
    }

    pub fn print_histogram(&self, bins: usize) {
        if self.miss_distances.is_empty() {
            return;
        }

        let min = self.min_miss;
        let max = self.max_miss.max(min + 1.0);
        let bin_width = (max - min) / bins as f64;

        let mut counts = vec![0usize; bins];
        for &d in &self.miss_distances {
            let idx = ((d - min) / bin_width).floor() as usize;
            let idx = idx.min(bins - 1);
            counts[idx] += 1;
        }

        let max_count = *counts.iter().max().unwrap_or(&1);
        let bar_width = 40;

        println!("\nMiss Distance Distribution:");
        println!("{:>8} {:>8} {}", "Range", "Count", "");
        for (i, &count) in counts.iter().enumerate() {
            let lo = min + i as f64 * bin_width;
            let hi = lo + bin_width;
            let bar_len = (count as f64 / max_count as f64 * bar_width as f64) as usize;
            let bar: String = "█".repeat(bar_len);
            println!("{:>4.0}-{:<4.0} {:>5} │{}", lo, hi, count, bar);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenario::baseline;

    #[test]
    fn monte_carlo_runs() {
        let config = SweepConfig::new(baseline(), 100);
        let result = run_sweep(&config);
        assert_eq!(result.trials, 100);
        assert!(result.hit_rate > 0.5, "expected >50% hits, got {}", result.hit_rate);
    }

    #[test]
    fn deterministic_with_seed() {
        let config = SweepConfig::new(baseline(), 50).with_seed(12345);
        let r1 = run_sweep(&config);
        let r2 = run_sweep(&config);
        assert_eq!(r1.hits, r2.hits);
        assert_eq!(r1.miss_distances, r2.miss_distances);
    }
}
