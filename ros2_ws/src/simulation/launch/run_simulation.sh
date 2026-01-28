#!/bin/bash
# Wrapper script to run Unity Simulation from the correct directory
# This ensures Simulation_Data is found

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../" && pwd)"
INSTALL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../install/simulation/lib/simulation" && pwd)"

# Change to the installation directory where Simulation_Data is located
cd "$INSTALL_DIR"

# Run the simulator with all arguments passed through
exec ./Simulation.x86_64 "$@"
