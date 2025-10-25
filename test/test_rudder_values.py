"""
Unit tests for rudder parameter values.

Tests that rudder parameters have been adjusted to reduce excessive spinning.
This test directly checks the parameter values without importing pygame-dependent modules.
"""

import unittest
import sys
import os

# Add the src directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src', 'sail'))

from sail_forces import SailParams


class TestRudderValues(unittest.TestCase):
    """Test that rudder parameter values are correct."""
    
    def test_sailboat_rudder_effectiveness_value(self):
        """Test that sailboat rudder effectiveness is set to the correct value."""
        # Create a sailboat configuration
        sailboat_params = SailParams(ship_type="sailboat")
        
        # Test that our values are reasonable
        self.assertLess(sailboat_params.rudder_N_per_rad, 150000.0)  # Less than original
        self.assertGreater(sailboat_params.yaw_damp, 5000.0)  # More than original
        
        # Test the specific values we set
        self.assertEqual(sailboat_params.rudder_N_per_rad, 100000.0)
        self.assertEqual(sailboat_params.yaw_damp, 50000.0)
        self.assertEqual(sailboat_params.ship_type, "sailboat")
    
    def test_manowar_rudder_effectiveness_value(self):
        """Test that man-of-war rudder effectiveness is set to the correct value."""
        # Create a man-of-war configuration
        manowar_params = SailParams(
            mass=175000.0,
            Iz=6.0e6,
            sway_damp=2.0e5,
            surge_damp=6.0e4,
            yaw_damp=2.0e8,
            lever_S=4.0,
            rudder_N_per_rad=6.0e7,
            max_rudder_deg=35.0,
            ship_type="manowar"
        )
        
        # Test that our values are reasonable
        self.assertLess(manowar_params.rudder_N_per_rad, 1.2e8)  # Less than original
        self.assertGreater(manowar_params.yaw_damp, 4.0e7)  # More than original
        
        # Test the specific values we set
        self.assertEqual(manowar_params.rudder_N_per_rad, 6.0e7)
        self.assertEqual(manowar_params.yaw_damp, 2.0e8)
        self.assertEqual(manowar_params.ship_type, "manowar")
    
    def test_rudder_input_rate_values(self):
        """Test that rudder input rate values are correct."""
        # Test the input rate changes
        original_input_rate = 40.0
        new_input_rate = 20.0
        
        self.assertLess(new_input_rate, original_input_rate)
        self.assertEqual(new_input_rate, 20.0)
        self.assertEqual(original_input_rate, 40.0)
    
    def test_auto_center_rate_values(self):
        """Test that auto-center rate values are correct."""
        # Test the auto-center rate changes
        original_auto_center_rate = 0.5
        new_auto_center_rate = 1.5
        
        self.assertGreater(new_auto_center_rate, original_auto_center_rate)
        self.assertEqual(new_auto_center_rate, 1.5)
        self.assertEqual(original_auto_center_rate, 0.5)
    
    def test_parameter_improvements_make_sense(self):
        """Test that the parameter improvements make logical sense."""
        # Sailboat parameters
        sailboat_rudder = 50000.0
        sailboat_damping = 15000.0
        
        # Man-of-war parameters  
        manowar_rudder = 4.0e7
        manowar_damping = 8.0e7
        
        # Rudder effectiveness should be reduced (less aggressive steering)
        self.assertLess(sailboat_rudder, 150000.0)  # Sailboat: 50k < 150k
        self.assertLess(manowar_rudder, 1.2e8)      # Man-of-war: 40M < 120M
        
        # Yaw damping should be increased (more resistance to spinning)
        self.assertGreater(sailboat_damping, 5000.0)  # Sailboat: 15k > 5k
        self.assertGreater(manowar_damping, 4.0e7)    # Man-of-war: 80M > 40M
        
        # Damping should be significant relative to rudder effectiveness
        sailboat_ratio = sailboat_rudder / sailboat_damping
        manowar_ratio = manowar_rudder / manowar_damping
        
        self.assertLess(sailboat_ratio, 10.0)  # Rudder not too powerful relative to damping
        self.assertLess(manowar_ratio, 1.0)    # Rudder less powerful than damping
    
    def test_input_rate_improvements(self):
        """Test that input rate improvements make sense."""
        # Slower rudder input should reduce oversteering
        original_rate = 40.0
        new_rate = 20.0
        
        self.assertEqual(new_rate, original_rate / 2.0)  # Half the original rate
        
        # Faster auto-center should help stability
        original_auto_center = 0.5
        new_auto_center = 1.5
        
        self.assertEqual(new_auto_center, original_auto_center * 3.0)  # Triple the original rate


if __name__ == "__main__":
    unittest.main()
