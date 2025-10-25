"""
Sail - Square-rigged vessel simulation and sail force calculations.

This package provides:
- SailForceCalculator: Aerodynamic force calculations for sails
- Ship: 2D ship dynamics simulation
- Interactive pygame visualization for sailing simulation
"""

from .sail_forces import SailForceCalculator, SailParams
from .manowar import Ship

__version__ = "0.1.0"
__all__ = ["SailForceCalculator", "SailParams", "Ship"]
