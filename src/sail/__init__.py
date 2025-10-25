"""
Sail - Square-rigged vessel simulation and sail force calculations.

This package provides:
- SailForces: Comprehensive aerodynamic force calculations for analysis/research
- Sails: Game-tuned aerodynamic calculations for interactive simulation
- Ship: 2D ship dynamics simulation
- SailParams: Parameter definitions and predefined ship configurations
- Interactive pygame visualization (main.py entry point)

Predefined ship configurations:
- SAILBOAT: Modern sailboat (50m² sail, 500kg mass)
- MANOWAR_FULL_PRESS: Man-of-war with full sails (800m² sail, 175t mass)
- MANOWAR_BATTLE_SAILS: Man-of-war with battle sails (500m² sail, 175t mass)
"""

from .reference import SailForces
from .sails import Sails
from .sail_params import SailParams, SAILBOAT, MANOWAR_FULL_PRESS, MANOWAR_BATTLE_SAILS
from .ship import Ship

__all__ = ["SailForces", "Sails", "SailParams", "Ship", "SAILBOAT", "MANOWAR_FULL_PRESS", "MANOWAR_BATTLE_SAILS"]

__version__ = "0.1.0"
