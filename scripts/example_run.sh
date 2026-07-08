#!/usr/bin/env bash
set -euo pipefail

cargo run -q -p sim_cli -- baseline

printf '\n--- Monte Carlo smoke test (10 trials) ---\n'
cargo run -q -p sim_cli -- sweep baseline 10 --seed=7
