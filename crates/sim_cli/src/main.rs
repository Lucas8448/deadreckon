use sim_core::monte_carlo::{SweepConfig, run_sweep};
use sim_core::{Scenario, SensorNoise, Sim, Status};

fn print_usage() {
    eprintln!("Usage:");
    eprintln!("  sim_cli [scenario]                       Run a single 3D scenario");
    eprintln!("  sim_cli [scenario] --noise=<level>       Run with sensor noise (perfect|realistic|degraded|extreme)");
    eprintln!("  sim_cli sweep [scenario] [N]             Run N Monte Carlo trials (default: 500)");
    eprintln!("  sim_cli sweep [scenario] [N] --nav=<delta>    Sweep nav_const ± delta");
    eprintln!("  sim_cli sweep [scenario] [N] --amax=<delta>   Sweep a_max ± delta");
    eprintln!("  sim_cli sweep [scenario] [N] --noise=<level>  Add sensor noise");
    eprintln!("  sim_cli sweep [scenario] [N] --seed=<u64>     Set RNG seed");
    eprintln!();
    eprintln!("Available scenarios:");
    for s in Scenario::all() {
        eprintln!("  - {}", s.name);
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();

    if args.len() > 1 && args[1] == "sweep" {
        run_monte_carlo(&args[2..]);
    } else if args.len() > 1 && (args[1] == "-h" || args[1] == "--help") {
        print_usage();
    } else {
        run_single(&args[1..]);
    }
}

fn parse_noise(args: &[String]) -> SensorNoise {
    for arg in args {
        if let Some(val) = arg.strip_prefix("--noise=") {
            return match val {
                "realistic" => SensorNoise::realistic(),
                "degraded" => SensorNoise::degraded(),
                "extreme" => SensorNoise::extreme(),
                _ => SensorNoise::perfect(),
            };
        }
    }
    SensorNoise::perfect()
}

fn lookup_scenario(name: &str) -> Scenario {
    match Scenario::by_name(name) {
        Some(s) => s,
        None => {
            eprintln!("Unknown scenario: '{}'", name);
            print_usage();
            std::process::exit(1);
        }
    }
}

fn run_single(args: &[String]) {
    let scenario_name = args.first().map(|s| s.as_str()).unwrap_or("baseline");
    let scenario = lookup_scenario(scenario_name);

    let noise = parse_noise(args);
    let noise_label = if noise.range_std > 0.0 { "with noise" } else { "perfect" };
    println!("=== Scenario: {} (3D, {}) ===", scenario.name, noise_label);

    let mut sim = Sim::with_sensor_noise(scenario.missile, scenario.target, scenario.params, noise, 42);

    let mut status = Status::Running;
    while status == Status::Running {
        status = sim.step();
    }

    let mp = sim.missile_pos();
    let mv = sim.missile_vel();
    let tp = sim.target_pos();
    let tv = sim.target_vel();

    println!("status: {:?}", status);
    println!(
        "t: {:.2}s  range: {:.2}m  closing: {:.2}m/s  los_rate: {:.6}rad/s  a_cmd: {:.2}m/s^2",
        sim.last.t, sim.last.range, sim.last.closing_speed, sim.last.los_rate, sim.last.a_cmd
    );
    println!(
        "missile.p: ({:.1}, {:.1}, {:.1})  v: ({:.1}, {:.1}, {:.1})",
        mp.x, mp.y, mp.z, mv.x, mv.y, mv.z
    );
    println!(
        "target.p:  ({:.1}, {:.1}, {:.1})  v: ({:.1}, {:.1}, {:.1})",
        tp.x, tp.y, tp.z, tv.x, tv.y, tv.z
    );
}

fn run_monte_carlo(args: &[String]) {
    let scenario_name = args.first().map(|s| s.as_str()).unwrap_or("baseline");
    let scenario = lookup_scenario(scenario_name);

    // Trial count: any positional non-flag arg after the scenario name. This
    // avoids the bug where `sim_cli sweep baseline --noise=realistic` (no N)
    // silently dropped the flag because we did `args.skip(2)`.
    let trials: usize = args
        .iter()
        .skip(1)
        .find(|a| !a.starts_with("--"))
        .and_then(|s| s.parse().ok())
        .unwrap_or(500);
    if trials == 0 {
        eprintln!("sweep: trial count must be > 0");
        std::process::exit(1);
    }

    let mut config = SweepConfig::new(scenario.clone(), trials);

    for arg in args.iter().filter(|a| a.starts_with("--")) {
        if let Some(val) = arg.strip_prefix("--nav=") {
            if let Ok(delta) = val.parse::<f64>() {
                config = config.with_nav_const_sweep(delta);
            }
        } else if let Some(val) = arg.strip_prefix("--amax=") {
            if let Ok(delta) = val.parse::<f64>() {
                config = config.with_a_max_sweep(delta);
            }
        } else if let Some(val) = arg.strip_prefix("--seed=") {
            if let Ok(seed) = val.parse::<u64>() {
                config = config.with_seed(seed);
            }
        } else if let Some(val) = arg.strip_prefix("--noise=") {
            let noise = match val {
                "realistic" => SensorNoise::realistic(),
                "degraded" => SensorNoise::degraded(),
                "extreme" => SensorNoise::extreme(),
                _ => SensorNoise::perfect(),
            };
            config = config.with_sensor_noise(noise);
        }
    }

    println!("=== Monte Carlo: {} (3D, {} trials) ===", scenario.name, trials);
    if config.perturb.nav_const > 0.0 {
        println!("  nav_const sweep: ±{:.1}", config.perturb.nav_const);
    }
    if config.perturb.a_max > 0.0 {
        println!("  a_max sweep: ±{:.1}", config.perturb.a_max);
    }
    if config.perturb.sensor_noise.range_std > 0.0 {
        println!(
            "  sensor noise: range_std={:.1}m angle_rate_std={:.4}rad/s",
            config.perturb.sensor_noise.range_std, config.perturb.sensor_noise.angle_rate_std
        );
    }
    println!();

    let result = run_sweep(&config);
    result.print_report();
    result.print_histogram(12);
}
