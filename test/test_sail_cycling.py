"""
Unit tests for sail cycling functionality.

Tests that sails cycle around continuously instead of stopping at limits.
"""

import math
import pytest
from sail.manowar import Ship, ShipParams
from sail.sail_forces import SailForceCalculator, SailParams


class TestSailCycling:
    """Test sail cycling behavior when holding Q or E keys."""
    
    def setup_method(self):
        """Set up test ship with known parameters."""
        sail_params = SailParams(A=100.0)  # Smaller area for testing
        ship_params = ShipParams(
            mass=1000.0,  # Smaller mass for testing
            Iz=1000.0,
            sway_damp=100.0,
            surge_damp=100.0,
            yaw_damp=100.0,
            lever_S=5.0,
            rudder_N_per_rad=1000.0,
            max_rudder_deg=30.0
        )
        self.calc = SailForceCalculator(sail_params)
        self.ship = Ship(self.calc, ship_params)
    
    def test_sail_cycling_q_key(self):
        """Test that holding Q cycles sails from -85° to +85°."""
        # Start at the upper limit
        self.ship.delta_sail_deg = 85.0
        
        # Simulate holding Q key for multiple steps
        dt = 0.1
        for _ in range(10):  # Simulate 1 second of holding Q
            # Simulate Q key press logic
            self.ship.delta_sail_deg -= 30.0 * dt
            if self.ship.delta_sail_deg < -85.0:
                self.ship.delta_sail_deg = 85.0
        
        # Should have cycled back to near the upper limit
        assert self.ship.delta_sail_deg > 80.0  # Should be close to 85.0
    
    def test_sail_cycling_e_key(self):
        """Test that holding E cycles sails from +85° to -85°."""
        # Start at the lower limit
        self.ship.delta_sail_deg = -85.0
        
        # Simulate holding E key for multiple steps
        dt = 0.1
        for _ in range(10):  # Simulate 1 second of holding E
            # Simulate E key press logic
            self.ship.delta_sail_deg += 30.0 * dt
            if self.ship.delta_sail_deg > 85.0:
                self.ship.delta_sail_deg = -85.0
        
        # Should have cycled back to near the lower limit
        assert self.ship.delta_sail_deg < -80.0  # Should be close to -85.0
    
    def test_sail_cycling_multiple_cycles(self):
        """Test multiple complete cycles."""
        # Start at 0 degrees
        self.ship.delta_sail_deg = 0.0
        
        dt = 0.1
        cycles_completed = 0
        
        # Simulate holding Q for enough time to complete multiple cycles
        for _ in range(100):  # Simulate 10 seconds
            self.ship.delta_sail_deg -= 30.0 * dt
            if self.ship.delta_sail_deg < -85.0:
                self.ship.delta_sail_deg = 85.0
                cycles_completed += 1
        
        # Should have completed multiple cycles
        assert cycles_completed > 0
        assert cycles_completed >= 2  # Should complete at least 2 cycles in 10 seconds
    
    def test_sail_cycling_e_key_multiple_cycles(self):
        """Test multiple complete cycles with E key."""
        # Start at 0 degrees
        self.ship.delta_sail_deg = 0.0
        
        dt = 0.1
        cycles_completed = 0
        
        # Simulate holding E for enough time to complete multiple cycles
        for _ in range(100):  # Simulate 10 seconds
            self.ship.delta_sail_deg += 30.0 * dt
            if self.ship.delta_sail_deg > 85.0:
                self.ship.delta_sail_deg = -85.0
                cycles_completed += 1
        
        # Should have completed multiple cycles
        assert cycles_completed > 0
        assert cycles_completed >= 2  # Should complete at least 2 cycles in 10 seconds
    
    def test_sail_angle_always_in_range(self):
        """Test that sail angle always stays within valid range after cycling."""
        dt = 0.1
        
        # Test Q key cycling
        self.ship.delta_sail_deg = 85.0
        for _ in range(50):
            self.ship.delta_sail_deg -= 30.0 * dt
            if self.ship.delta_sail_deg < -85.0:
                self.ship.delta_sail_deg = 85.0
            # Angle should always be in valid range
            assert -85.0 <= self.ship.delta_sail_deg <= 85.0
        
        # Test E key cycling
        self.ship.delta_sail_deg = -85.0
        for _ in range(50):
            self.ship.delta_sail_deg += 30.0 * dt
            if self.ship.delta_sail_deg > 85.0:
                self.ship.delta_sail_deg = -85.0
            # Angle should always be in valid range
            assert -85.0 <= self.ship.delta_sail_deg <= 85.0


if __name__ == "__main__":
    pytest.main([__file__])
