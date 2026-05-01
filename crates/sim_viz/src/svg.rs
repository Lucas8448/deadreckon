//! Headless SVG exporter — runs each scenario and writes an animated SVG
//! (SMIL) of the missile + target trajectories into `docs/`. Used to embed
//! engagement previews in the README.
//!
//! Usage:
//!     cargo run -p sim_viz --bin sim_svg                # all scenarios
//!     cargo run -p sim_viz --bin sim_svg baseline       # one scenario
//!
//! Output:
//!     docs/svg/<scenario>.svg

use physics_sandbox::viz::MultiRecorder;
use sim_core::{Scenario, Sim, Status};

const OUT_DIR: &str = "docs/svg";
const SAMPLE_EVERY_N_STEPS: usize = 5;

fn run_scenario(scenario: &Scenario) -> std::io::Result<String> {
    let mut sim = Sim::new(scenario.missile, scenario.target, scenario.params);

    let mut rec = MultiRecorder::new(format!(
        "{} — missile (red) vs target (blue)",
        scenario.name
    ));
    let missile_track = rec.add_track("missile", "#e63946");
    let target_track = rec.add_track("target", "#1d3557");

    // Initial sample.
    rec.push(missile_track, sim.t, sim.missile_pos());
    rec.push(target_track, sim.t, sim.target_pos());

    let mut step_idx = 0usize;
    let mut status = Status::Running;
    while status == Status::Running {
        status = sim.step();
        step_idx += 1;
        if step_idx % SAMPLE_EVERY_N_STEPS == 0 || status != Status::Running {
            rec.push(missile_track, sim.t, sim.missile_pos());
            rec.push(target_track, sim.t, sim.target_pos());
        }
    }

    std::fs::create_dir_all(OUT_DIR)?;
    let path = format!("{}/{}.svg", OUT_DIR, scenario.name);
    rec.export_svg_animated(&path)?;
    Ok(format!(
        "{} → {} ({:?}, t={:.2}s, miss={:.2}m)",
        scenario.name, path, status, sim.t, sim.last.range
    ))
}

fn main() -> std::io::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let scenarios: Vec<Scenario> = if args.is_empty() {
        Scenario::all()
    } else {
        args.iter()
            .filter_map(|n| {
                Scenario::by_name(n).or_else(|| {
                    eprintln!("unknown scenario: {}", n);
                    None
                })
            })
            .collect()
    };

    if scenarios.is_empty() {
        eprintln!("no scenarios to render");
        std::process::exit(1);
    }

    for s in &scenarios {
        match run_scenario(s) {
            Ok(line) => println!("{}", line),
            Err(e) => eprintln!("{}: {}", s.name, e),
        }
    }
    Ok(())
}
