# Sail - Square-Rigged Vessel Simulation

A comprehensive Python package for simulating sailing vessels with realistic aerodynamics and ship dynamics. Features both detailed physics calculations for research/analysis and an interactive pygame-based sailing simulator.

## Overview

This project provides a modular sailing simulation framework with three main use cases:

- **🔬 Research & Analysis**: Comprehensive aerodynamic calculations with optimization tools
- **🎮 Interactive Simulation**: Real-time sailing game with different ship types
- **🛠️ Engineering Applications**: Modular components for building sailing simulations

## Features

### Core Classes

- **`SailForces`**: Comprehensive aerodynamic calculations with pandas integration for analysis
- **`Sails`**: Game-tuned aerodynamics optimized for real-time interactive simulation  
- **`Ship`**: 2D ship dynamics with realistic physics (rudder, damping, inertia)
- **`SailParams`**: Parameter definitions with predefined ship configurations

### Ship Configurations

- **`SAILBOAT`**: Modern sailboat (50m² sail, 500kg mass)
- **`MANOWAR_FULL_PRESS`**: Historical man-of-war with full sails (800m² sail, 175t mass)
- **`MANOWAR_BATTLE_SAILS`**: Man-of-war with battle sails (500m² sail, 175t mass)

### Interactive Features

- Real-time pygame visualization with multiple ship types
- Realistic sailing physics (apparent wind, sail trim, rudder control)
- Built-in sail optimization for close-hauled sailing
- Comprehensive controls for wind, sails, and rudder

## Installation

### Using uv (Recommended)

```bash
# Clone the repository
git clone <repository-url>
cd sail

# Install dependencies and package
uv sync
uv pip install -e .
```

### Using pip

```bash
# Install from source
pip install -e .

# Or install dependencies manually
pip install pygame pandas pytest
```

## Quick Start

### Interactive Sailing Simulator

Launch the interactive simulator:

```bash
# Using uv
uv run sail-sim

# After installation
sail-sim
```

**Controls:**
- **A/D** or **←/→**: Rudder control (steering)
- **Q/E**: Sail trim (adjust sail angle)
- **Z/C**: Wind direction
- **X**: Increase wind speed
- **S**: Center rudder
- **O**: Optimize sail for close-hauled sailing
- **1/2/3**: Switch ship types (Sailboat/Full Press/Battle Sails)
- **R**: Reset ship position
- **T**: Move target buoy
- **SPACE**: Pause/unpause
- **ESC**: Quit

### Programmatic Usage

#### Analysis with Comprehensive Physics

```python
from sail import SailForces, SAILBOAT, MANOWAR_FULL_PRESS

# Use predefined configurations
sailboat_calc = SailForces(SAILBOAT)
manowar_calc = SailForces(MANOWAR_FULL_PRESS)

# Calculate forces for sailing conditions
Va = 15.0      # Apparent wind speed (m/s)
gamma = 45.0   # Apparent wind angle (degrees) 
delta = 20.0   # Sail angle (degrees)

forces = sailboat_calc.forces(Va, gamma, delta)
print(f"Thrust: {forces['T']:.1f} N")
print(f"Side force: {forces['S']:.1f} N")
print(f"Lift: {forces['L']:.1f} N")
print(f"Drag: {forces['D']:.1f} N")

# Optimize sail trim for maximum drive
optimal = sailboat_calc.optimize_trim_for_drive(Va, gamma)
print(f"Optimal sail angle: {optimal['delta_deg']:.1f}°")
print(f"Maximum thrust: {optimal['T']:.1f} N")
```

#### Interactive Simulation

```python
from sail import Ship, Sails, SAILBOAT

# Create ship with game-tuned physics
sails = Sails(SAILBOAT)
ship = Ship(sails)

# Simulation loop
dt = 0.1  # Time step (seconds)
VT = 12.0  # True wind speed (m/s)
wind_direction = 30.0  # Wind from 30° (global frame)

# Step simulation
aero_forces, Va, gamma = ship.step(dt, VT, wind_direction)

print(f"Ship position: ({ship.x:.1f}, {ship.y:.1f})")
print(f"Ship heading: {ship.psi:.2f} rad")
print(f"Apparent wind: {Va:.1f} m/s at {gamma:.1f}°")
```

#### Custom Ship Configuration

```python
from sail import SailParams, SailForces

# Create custom ship parameters
custom_ship = SailParams(
    A=75.0,              # Sail area (m²)
    AR=4.0,              # Aspect ratio
    mass=2000.0,         # Ship mass (kg)
    CD0=0.012,           # Parasitic drag coefficient
    alpha_stall_deg=16.0, # Stall angle (degrees)
    ship_type="custom"
)

# Use in calculations
calc = SailForces(custom_ship)
```

## Architecture

The project uses a clean modular architecture:

```
┌─────────────────────────────────────────────────┐
│                    sail                         │
│  ┌─────────────┐ ┌─────────────┐ ┌──────────────┐│
│  │ SailForces  │ │    Sails    │ │     Ship     ││
│  │(analysis)   │ │(game-tuned) │ │ (dynamics)   ││
│  └─────────────┘ └─────────────┘ └──────────────┘│
│  ┌─────────────────────────────────────────────┐ │
│  │              SailParams                     │ │
│  │     (shared parameter definitions)          │ │
│  └─────────────────────────────────────────────┘ │
│  ┌─────────────────────────────────────────────┐ │
│  │                main.py                      │ │
│  │        (interactive pygame app)             │ │
│  └─────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────┘
```

### Class Responsibilities

- **`SailForces`**: Comprehensive aerodynamic calculations, optimization tools, pandas integration
- **`Sails`**: Game-tuned aerodynamics with force scaling for interactive simulation
- **`Ship`**: 2D ship dynamics, rudder control, speed-dependent effects
- **`SailParams`**: Parameter definitions and predefined ship configurations
- **`main.py`**: Interactive pygame visualization and user interface

## Physics Model

### Aerodynamics

- **Finite Wing Theory**: Lift curve slope with aspect ratio effects
- **Stall Modeling**: Smooth transition from linear to post-stall regime
- **Drag Polar**: Parabolic induced drag with Oswald efficiency factor
- **Force Decomposition**: Lift/drag resolved into thrust/side force components

### Ship Dynamics  

- **6-DOF State**: Position (x,y), heading (ψ), velocities (u,v), yaw rate (r)
- **Damping Models**: Surge, sway, and yaw damping with speed dependencies
- **Rudder Physics**: Speed-dependent effectiveness with realistic limits
- **Integration**: Euler integration with body-to-world frame transforms

### Wind Model

- **True Wind**: Global wind speed and direction
- **Apparent Wind**: Vector addition of true wind and ship velocity
- **Reference Frames**: Consistent "from" angle conventions throughout

## Testing

Run the comprehensive test suite:

```bash
# Run all tests
uv run pytest

# Run with coverage report
uv run pytest --cov=sail --cov-report=html

# Run specific test categories
uv run pytest test/reference/              # Physics tests
uv run pytest test/test_rudder_values.py   # Parameter validation
```

## Project Structure

```
sail/
├── src/sail/
│   ├── __init__.py          # Package exports
│   ├── sail_params.py       # Parameters & ship configurations
│   ├── sails.py             # Game-tuned aerodynamics  
│   ├── ship.py              # Ship dynamics simulation
│   ├── main.py              # Interactive pygame application
│   └── reference/
│       └── sail_forces.py   # Comprehensive aerodynamics
├── test/
│   ├── test_rudder_values.py
│   └── reference/
│       ├── test_sail.py     # Core physics tests
│       ├── test_sail_cycling.py
│       └── test_sail_force_direction.py
├── pyproject.toml           # Project configuration
└── README.md
```

## Development Principles

- **Modular Design**: Clean separation between physics, visualization, and configuration
- **Functional Approach**: Objects expose functional methods like `map` and `optimize`
- **Guard Conditions**: Early returns to reduce nesting complexity
- **Comprehensive Testing**: Unit tests for all physics and edge cases
- **Type Safety**: Full type annotations for better development experience

## Ship Configuration Details

### SAILBOAT (Modern Performance Sailboat)
- **Sail Area**: 50 m² (manageable for single-handed sailing)
- **Aspect Ratio**: 6.0 (high efficiency upwind)
- **Mass**: 500 kg (lightweight construction)
- **Stall Angle**: 18° (forgiving characteristics)
- **Use Case**: Fast, responsive sailing for recreation/racing

### MANOWAR_FULL_PRESS (Historical Warship - All Sails)
- **Sail Area**: 800 m² (reduced from historical ~5000m² for gameplay)
- **Aspect Ratio**: 3.5 (square-rigged characteristics)
- **Mass**: 175,000 kg (1st rate ship of the line)
- **Stall Angle**: 15° (earlier stall, less efficient upwind)
- **Use Case**: Powerful reaching/running, historical accuracy

### MANOWAR_BATTLE_SAILS (Historical Warship - Reduced Sail)
- **Sail Area**: 500 m² (topsails and courses only)
- **Aspect Ratio**: 3.0 (stubbier with less canvas aloft)
- **Mass**: 175,000 kg (same hull as full press)
- **Stall Angle**: 12° (reduced upwind performance)  
- **Use Case**: Heavy weather sailing, combat configuration

## Contributing

1. Follow the established modular architecture
2. Add comprehensive unit tests for new features
3. Use type hints throughout
4. Keep functions small and focused
5. Test with multiple ship configurations

## License

This project is licensed under the MIT License - see the LICENSE file for details.