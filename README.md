# Deadreckon

Deterministic missile guidance simulation with proportional navigation.

## Quick Start

```bash
# Run a single scenario
cargo run -p sim_cli -- baseline
cargo run -p sim_cli -- turning
cargo run -p sim_cli -- weaving

# Visualize
cargo run -p sim_viz -- turning

# Monte Carlo analysis (500 trials)
cargo run -p sim_cli -- sweep baseline 500
cargo run -p sim_cli -- sweep weaving 500 --nav=1.5
```

## Scenarios

| Name | Description |
|------|-------------|
| `baseline` | Standard engagement, target approaching |
| `head_on` | Target flying directly at missile |
| `crossing` | Target perpendicular to LOS |
| `fast_target` | Supersonic target |
| `turning` | Constant turn rate maneuver |
| `weaving` | Sinusoidal evasive maneuver |

## Workspace

| Crate | Purpose |
|-------|---------|
| `sim_core` | Dynamics, guidance laws, Monte Carlo |
| `sim_cli` | Command-line runner |
| `sim_viz` | Real-time visualization (macroquad) |