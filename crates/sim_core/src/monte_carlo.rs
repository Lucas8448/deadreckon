use glam::Vec2;
use rand::Rng;

use crate::{Scenario, Sim, Status};

#[derive(Clone, Copy, Debug)]
pub struct TrialResult {
    pub hit: bool,
    pub miss_distance: f32,
    pub time: f32,
}

#[derive(Clone, Debug)]
pub struct MonteCarloResult {
    pub trials: usize,
    pub hits: usize,
    pub hit_rate: f32,
    pub miss_distances: Vec<f32>,
    pub mean_miss: f32,
    pub std_miss: f32,
    pub min_miss: f32,
    pub max_miss: f32,
    pub p50_miss: f32,
    pub p90_miss: f32,
    pub p99_miss: f32,
}

#[derive(Clone, Debug)]
pub struct SweepConfig {
    pub scenario: Scenario,
    pub trials: usize,
    pub seed: u64,
    pub perturb: Perturbations,
}

/// What to randomize in each trial.
#[derive(Clone, Copy, Debug, Default)]
pub struct Perturbations {
    pub target_pos: f32,
    pub target_vel: f32,
    pub target_heading: f32,
    pub nav_const: f32,
    pub a_max: f32,
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
            },
        }
    }

    pub fn with_seed(mut self, seed: u64) -> Self {
        self.seed = seed;
        self
    }

    pub fn with_nav_const_sweep(mut self, delta: f32) -> Self {
        self.perturb.nav_const = delta;
        self
    }

    pub fn with_a_max_sweep(mut self, delta: f32) -> Self {
        self.perturb.a_max = delta;
        self
    }
}

fn run_trial<R: Rng>(scenario: &Scenario, perturb: &Perturbations, rng: &mut R) -> TrialResult {
    let mut missile = scenario.missile;
    let mut target = scenario.target;
    let mut params = scenario.params;

    if perturb.target_pos > 0.0 {
        target.p.x += rng.random_range(-perturb.target_pos..perturb.target_pos);
        target.p.y += rng.random_range(-perturb.target_pos..perturb.target_pos);
    }

    if perturb.target_vel > 0.0 {
        let speed = target.v.length();
        let new_speed = speed + rng.random_range(-perturb.target_vel..perturb.target_vel);
        if speed > 1e-6 {
            target.v = target.v.normalize() * new_speed.max(10.0);
        }
    }

    if perturb.target_heading > 0.0 {
        let angle = rng.random_range(-perturb.target_heading..perturb.target_heading);
        let (sin, cos) = angle.sin_cos();
        let vx = target.v.x * cos - target.v.y * sin;
        let vy = target.v.x * sin + target.v.y * cos;
        target.v = Vec2::new(vx, vy);
    }

    if perturb.nav_const > 0.0 {
        params.nav_const += rng.random_range(-perturb.nav_const..perturb.nav_const);
        params.nav_const = params.nav_const.max(1.0);
    }

    if perturb.a_max > 0.0 {
        missile.a_max += rng.random_range(-perturb.a_max..perturb.a_max);
        missile.a_max = missile.a_max.max(10.0);
    }

    let mut sim = Sim::new(missile, target, params);
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

// Run a Monte Carlo sweep and return aggregated statistics.
pub fn run_sweep(config: &SweepConfig) -> MonteCarloResult {
    use rand::SeedableRng;
    let mut rng = rand::rngs::StdRng::seed_from_u64(config.seed);

    let mut results: Vec<TrialResult> = Vec::with_capacity(config.trials);

    for _ in 0..config.trials {
        results.push(run_trial(&config.scenario, &config.perturb, &mut rng));
    }

    let hits = results.iter().filter(|r| r.hit).count();
    let hit_rate = hits as f32 / config.trials as f32;

    let mut miss_distances: Vec<f32> = results.iter().map(|r| r.miss_distance).collect();
    miss_distances.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let mean_miss = miss_distances.iter().sum::<f32>() / miss_distances.len() as f32;
    let variance = miss_distances
        .iter()
        .map(|d| (d - mean_miss).powi(2))
        .sum::<f32>()
        / miss_distances.len() as f32;
    let std_miss = variance.sqrt();

    let percentile = |p: f32| -> f32 {
        let idx = ((miss_distances.len() as f32 - 1.0) * p).round() as usize;
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
        let bin_width = (max - min) / bins as f32;

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
            let lo = min + i as f32 * bin_width;
            let hi = lo + bin_width;
            let bar_len = (count as f32 / max_count as f32 * bar_width as f32) as usize;
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
        assert!(result.hit_rate > 0.5);
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
