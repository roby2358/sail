# Sail - Square-Rigged Vessel Simulation

A Python package for simulating square-rigged sailing vessels with realistic aerodynamics and ship dynamics.

## Features

- **SailForces**: Comprehensive aerodynamic force calculations for sails with stall modeling
- **Ship**: 2D ship dynamics simulation with realistic physics
- **Interactive Visualization**: Pygame-based real-time sailing simulator
- **Comprehensive Testing**: Full unit test suite for physics validation

## Installation

This project uses `uv` for dependency management. Install with:

```bash
# Install dependencies
uv sync

# Install in development mode
uv pip install -e .
```

## Usage

### Interactive Simulation

Run the interactive sailing simulator:

```bash
# Using uv
uv run sail-sim

# Or after installation
sail-sim
```

**Controls:**
- **A/D or Arrow Keys**: Control rudder (steering)
- **Q/E**: Trim sails (adjust sail angle)
- **Z/X**: Adjust wind speed (0-30 m/s)
- **C/V**: Change wind direction
- **SPACE**: Pause/unpause
- **R**: Reset ship
- **1/2**: Switch between "Full Press" and "Battle Sails"
- **ESC**: Quit

### Programmatic Usage

```python
from sail import SailForces, SailParams, Ship, SAILBOAT, MANOWAR_FULL_PRESS

# Option 1: Create sail calculator with custom parameters
params = SailParams(A=100.0, AR=3.0)  # 100 m² sail, aspect ratio 3
calc = SailForces(params)

# Option 2: Use predefined ship configurations
sailboat_calc = SailForces(SAILBOAT)      # Modern sailboat
manowar_calc = SailForces(MANOWAR_FULL_PRESS)  # Historical man-of-war

# Calculate forces for given conditions
Va = 15.0  # Apparent wind speed (m/s)
gamma = 45.0  # Apparent wind angle (degrees)
delta = 20.0  # Sail angle (degrees)

forces = calc.forces(Va, gamma, delta)
print(f"Thrust: {forces['T']:.1f} N")
print(f"Side force: {forces['S']:.1f} N")

# Create ship simulation using predefined configurations
from sail import Ship, Sails, SAILBOAT, MANOWAR_FULL_PRESS

# Use predefined sailboat configuration
sailboat_sails = Sails(SAILBOAT)
sailboat_ship = Ship(sailboat_sails)

# Or use man-of-war configuration for heavier ship
manowar_sails = Sails(MANOWAR_FULL_PRESS) 
manowar_ship = Ship(manowar_sails)

# Simulate one time step
dt = 0.1  # 0.1 second time step
VT = 12.0  # True wind speed
wind_from_global = 30.0  # Wind from 30° (global frame)

aero, Va, gamma = ship.step(dt, VT, wind_from_global)
print(f"Ship position: ({ship.x:.1f}, {ship.y:.1f})")
print(f"Ship heading: {ship.psi:.2f} rad")
```

### Testing

Run the comprehensive test suite:

```bash
# Run all tests
uv run pytest

# Run with coverage
uv run pytest --cov=sail

# Run specific test file
uv run pytest test/test_sail.py
```

## Physics Model

### Sail Aerodynamics

The sail model uses:
- **Lift/Drag Coefficients**: Based on finite-wing theory with aspect ratio effects
- **Stall Modeling**: Smooth transition from linear to post-stall behavior
- **Induced Drag**: Parabolic drag polar with Oswald efficiency factor
- **Zero-Lift Angle**: Accounts for sail shape and trim

### Ship Dynamics

The ship model includes:
- **6-DOF Integration**: Position, velocity, and heading
- **Damping**: Realistic water resistance
- **Rudder Control**: Yaw moment generation
- **Side Force Lever**: Sail forces create yawing moment

### Wind Model

- **True Wind**: Global wind speed and direction
- **Apparent Wind**: Calculated considering ship velocity
- **Force Decomposition**: Lift and drag resolved into thrust and side force

## Project Structure

```
sail/
├── src/sail/
│   ├── __init__.py          # Package exports
│   ├── sail_params.py       # Parameter definitions & ship configs
│   ├── sails.py             # Game-tuned aerodynamics
│   ├── ship.py              # Ship dynamics simulation
│   ├── main.py              # Interactive pygame application
│   └── reference/
│       └── sail_forces.py   # Comprehensive aerodynamics
├── test/
│   ├── __init__.py
│   ├── test_rudder_values.py
│   └── reference/
│       ├── test_sail.py     # Core physics tests
│       └── ...              # Additional reference tests
├── pyproject.toml           # Project configuration
└── README.md               # This file
```

## Development

The project follows these principles:
- **Functional Programming**: Objects contain functional methods internally
- **Guard Conditions**: Early returns to reduce nesting
- **Granular Functions**: Small, focused functions over deep nesting
- **Comprehensive Testing**: All functionality covered by unit tests
- **Type Hints**: Full type annotations for better code clarity

## License

See LICENSE file for details.
