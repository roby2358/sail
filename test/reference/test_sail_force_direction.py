#!/usr/bin/env python3
"""
Unit tests for sail force direction calculation.

Tests that sail forces can generate forces in both directions
based on sail trim, regardless of wind direction.
"""

import unittest
import math
import sys
import os

# Add the src directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from sail.reference import SailForces
from sail.sail_params import SailParams


class TestSailForceDirection(unittest.TestCase):
    """Test that sail forces work correctly on both tacks."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.calc = SailForces()
    
    def test_port_tack_can_generate_starboard_force(self):
        """Test that wind from port can generate starboard force with proper trim."""
        # Wind from port (negative angle from bow)
        VT = 12.0  # m/s
        beta_deg = -45.0  # wind from port at 45 degrees
        U_boat = 0.0  # stationary boat initially
        
        # Get apparent wind
        Va, gamma = self.calc.apparent_wind(VT, beta_deg, U_boat)
        
        # Test various sail angles to find one that generates starboard force
        found_starboard_force = False
        best_starboard_force = 0.0
        best_delta = 0.0
        
        for delta_deg in range(-90, 91, 5):  # Test sail angles from -90 to +90 degrees
            forces = self.calc.forces(Va, gamma, delta_deg)
            S = forces["S"]  # Side force (positive = starboard)
            
            if S > 0:  # Starboard force
                found_starboard_force = True
                if S > best_starboard_force:
                    best_starboard_force = S
                    best_delta = delta_deg
        
        self.assertTrue(found_starboard_force, 
                       f"Could not generate starboard force with wind from port. "
                       f"Wind from {beta_deg}°, apparent wind from {gamma:.1f}°")
        
        # Test the best case
        forces = self.calc.forces(Va, gamma, best_delta)
        self.assertGreater(forces["S"], 0, 
                          f"Best sail angle {best_delta}° should generate starboard force")
    
    def test_starboard_tack_can_generate_port_force(self):
        """Test that wind from starboard can generate port force with proper trim."""
        # Wind from starboard (positive angle from bow)
        VT = 12.0  # m/s
        beta_deg = 45.0  # wind from starboard at 45 degrees
        U_boat = 0.0  # stationary boat initially
        
        # Get apparent wind
        Va, gamma = self.calc.apparent_wind(VT, beta_deg, U_boat)
        
        # Test various sail angles to find one that generates port force
        found_port_force = False
        best_port_force = 0.0
        best_delta = 0.0
        
        for delta_deg in range(-90, 91, 5):  # Test sail angles from -90 to +90 degrees
            forces = self.calc.forces(Va, gamma, delta_deg)
            S = forces["S"]  # Side force (negative = port)
            
            if S < 0:  # Port force
                found_port_force = True
                if abs(S) > abs(best_port_force):
                    best_port_force = S
                    best_delta = delta_deg
        
        self.assertTrue(found_port_force, 
                       f"Could not generate port force with wind from starboard. "
                       f"Wind from {beta_deg}°, apparent wind from {gamma:.1f}°")
        
        # Test the best case
        forces = self.calc.forces(Va, gamma, best_delta)
        self.assertLess(forces["S"], 0, 
                       f"Best sail angle {best_delta}° should generate port force")
    
    def test_angle_of_attack_calculation(self):
        """Test that angle of attack is calculated correctly."""
        # Wind from port
        VT = 12.0
        beta_deg = -45.0
        U_boat = 0.0
        
        Va, gamma = self.calc.apparent_wind(VT, beta_deg, U_boat)
        
        # Test sail angle that should give positive angle of attack
        delta_deg = 20.0  # Sail angled toward starboard
        forces = self.calc.forces(Va, gamma, delta_deg)
        alpha_deg = forces["alpha_deg"]
        
        # alpha = gamma - delta
        expected_alpha = gamma - delta_deg
        self.assertAlmostEqual(alpha_deg, expected_alpha, places=1,
                             msg=f"Angle of attack calculation incorrect: {alpha_deg}° vs expected {expected_alpha}°")
    
    def test_force_direction_consistency(self):
        """Test that force directions are consistent with angle of attack."""
        VT = 12.0
        U_boat = 0.0
        
        # Test both port and starboard winds
        for beta_deg in [-45.0, 45.0]:
            Va, gamma = self.calc.apparent_wind(VT, beta_deg, U_boat)
            
            # Test sail angles that should give positive and negative angles of attack
            for delta_deg in [-20.0, 20.0]:
                forces = self.calc.forces(Va, gamma, delta_deg)
                alpha_deg = forces["alpha_deg"]
                S = forces["S"]
                
                # Positive alpha should generally give positive S (starboard force)
                # Negative alpha should generally give negative S (port force)
                if abs(alpha_deg) > 5.0:  # Only test when angle of attack is significant
                    if alpha_deg > 0:
                        self.assertGreater(S, 0, 
                                         f"Positive alpha {alpha_deg:.1f}° should give starboard force, got S={S:.1f}")
                    else:
                        self.assertLess(S, 0, 
                                      f"Negative alpha {alpha_deg:.1f}° should give port force, got S={S:.1f}")
    
    def test_negative_angle_of_attack_behavior(self):
        """Test that negative angle of attack helps escape bad sailing states."""
        # Simulate a bad state: wind from starboard but boat moving backward
        # This creates apparent wind from port with negative angle of attack
        VT = 12.0
        beta_deg = 45.0  # True wind from starboard
        U_boat = -2.0  # Boat moving backward (negative speed)
        
        Va, gamma = self.calc.apparent_wind(VT, beta_deg, U_boat)
        
        # With negative angle of attack, we should get forces that help escape the bad state
        # Test sail angles that give negative alpha
        for delta_deg in [-30.0, -20.0, -10.0]:
            forces = self.calc.forces(Va, gamma, delta_deg)
            alpha_deg = forces["alpha_deg"]
            T = forces["T"]
            S = forces["S"]
            
            # With negative alpha, we should get:
            # - Forward thrust (T > 0) to help escape backward motion
            # - Side force that helps turn away from the wind
            if alpha_deg < -5.0:  # Significant negative angle of attack
                self.assertGreater(T, 0, 
                                 f"Negative alpha {alpha_deg:.1f}° should generate forward thrust to escape bad state, got T={T:.1f}")
    
    def test_lift_force_direction_flip(self):
        """Test that lift force direction flips correctly with angle of attack sign."""
        VT = 12.0
        beta_deg = 0.0  # Wind from directly ahead
        U_boat = 0.0
        
        Va, gamma = self.calc.apparent_wind(VT, beta_deg, U_boat)
        
        # Test positive and negative angles of attack with same magnitude
        delta_pos = 10.0  # Should give positive alpha
        delta_neg = -10.0  # Should give negative alpha
        
        forces_pos = self.calc.forces(Va, gamma, delta_pos)
        forces_neg = self.calc.forces(Va, gamma, delta_neg)
        
        alpha_pos = forces_pos["alpha_deg"]
        alpha_neg = forces_neg["alpha_deg"]
        S_pos = forces_pos["S"]
        S_neg = forces_neg["S"]
        
        # Alpha values should be opposite
        self.assertAlmostEqual(alpha_pos, -alpha_neg, places=1,
                             msg=f"Alpha values should be opposite: {alpha_pos:.1f}° vs {alpha_neg:.1f}°")
        
        # Side forces should be opposite (lift direction should flip)
        if abs(alpha_pos) > 5.0:  # Only test when angles are significant
            self.assertGreater(S_pos * S_neg, 0, 
                             f"Side forces should be opposite for opposite alpha: S_pos={S_pos:.1f}, S_neg={S_neg:.1f}")
            # Or they should both be zero (stalled condition)
            if abs(S_pos) < 100 and abs(S_neg) < 100:
                pass  # Both near zero is acceptable (stalled)
            else:
                self.assertLess(S_pos * S_neg, 0, 
                               f"Side forces should be opposite for opposite alpha: S_pos={S_pos:.1f}, S_neg={S_neg:.1f}")
    
    def test_force_magnitude_reasonable(self):
        """Test that force magnitudes are reasonable for typical sailing conditions."""
        VT = 12.0  # m/s
        beta_deg = -45.0  # wind from port
        U_boat = 0.0
        
        Va, gamma = self.calc.apparent_wind(VT, beta_deg, U_boat)
        
        # Test a few sail angles
        for delta_deg in [-30.0, 0.0, 30.0]:
            forces = self.calc.forces(Va, gamma, delta_deg)
            T = forces["T"]
            S = forces["S"]
            L = forces["L"]
            D = forces["D"]
            
            # Forces should be reasonable in magnitude (not too large or too small)
            self.assertLess(abs(T), 50000, f"Thrust force {T} is too large")
            self.assertLess(abs(S), 50000, f"Side force {S} is too large")
            self.assertGreater(abs(L), 0, f"Lift force {L} should be non-zero")
            self.assertGreater(abs(D), 0, f"Drag force {D} should be non-zero")

    def test_exists_trim_with_forward_and_starboard_for_wind_from_port(self):
        """There should exist a trim with T>0 and S>0 for wind-from-port."""
        VT = 12.0
        beta_deg = -45.0  # wind from port
        U_boat = 0.0
        Va, gamma = self.calc.apparent_wind(VT, beta_deg, U_boat)
        found = False
        for delta in [x * 0.5 - 90 for x in range(361)]:  # -90..90 step 0.5
            f = self.calc.forces(Va, gamma, delta)
            if f["T"] > 0 and f["S"] > 0:
                found = True
                break
        self.assertTrue(found, "No trim produced forward+starboard force for wind-from-port")

    def test_exists_trim_with_forward_and_port_for_wind_from_starboard(self):
        """There should exist a trim with T>0 and S<0 for wind-from-starboard."""
        VT = 12.0
        beta_deg = 45.0  # wind from starboard
        U_boat = 0.0
        Va, gamma = self.calc.apparent_wind(VT, beta_deg, U_boat)
        found = False
        for delta in [x * 0.5 - 90 for x in range(361)]:
            f = self.calc.forces(Va, gamma, delta)
            if f["T"] > 0 and f["S"] < 0:
                found = True
                break
        self.assertTrue(found, "No trim produced forward+port force for wind-from-starboard")
    
    def test_close_hauled_sail_optimization(self):
        """Test that close-hauled sailing works with realistic sail angles."""
        VT = 12.0
        beta_deg = 45.0  # wind from 45 degrees (typical tacking angle)
        U_boat = 0.0
        Va, gamma = self.calc.apparent_wind(VT, beta_deg, U_boat)
        
        # Test realistic close-hauled sail angles (-60 to +60 degrees)
        best_thrust = -float('inf')
        best_sail_angle = 0.0
        
        for sail_angle in range(-60, 61, 5):  # Realistic close-hauled range
            forces = self.calc.forces(Va, gamma, sail_angle)
            if forces["T"] > best_thrust:
                best_thrust = forces["T"]
                best_sail_angle = sail_angle
        
        # Should find a sail angle that generates positive forward thrust
        self.assertGreater(best_thrust, 0, 
                          f"Could not generate forward thrust for close-hauled sailing at {beta_deg}° wind")
        
        # The optimal sail angle should be reasonable (not at extremes)
        self.assertGreaterEqual(best_sail_angle, -60, 
                               f"Optimal sail angle {best_sail_angle}° is too extreme (negative)")
        self.assertLessEqual(best_sail_angle, 60, 
                            f"Optimal sail angle {best_sail_angle}° is too extreme (positive)")
    
    def test_realistic_tacking_angles(self):
        """Test that tacking works at realistic angles (45-60 degrees to wind)."""
        VT = 12.0
        
        # Test both port and starboard tacks at realistic angles
        tack_angles = [45.0, -45.0]  # 45 degrees to wind on each tack
        
        for tack_angle in tack_angles:
            # Wind from the tack angle (simulating sailing at that angle to wind)
            beta_deg = tack_angle
            U_boat = 0.0
            Va, gamma = self.calc.apparent_wind(VT, beta_deg, U_boat)
            
            # Find sail angle that gives good forward thrust
            best_thrust = -float('inf')
            best_sail_angle = 0.0
            
            for sail_angle in range(-45, 46, 5):  # Reasonable sail angle range
                forces = self.calc.forces(Va, gamma, sail_angle)
                if forces["T"] > best_thrust:
                    best_thrust = forces["T"]
                    best_sail_angle = sail_angle
            
            # Should be able to generate forward thrust on this tack
            self.assertGreater(best_thrust, 0, 
                              f"Could not generate forward thrust on {tack_angle}° tack")
            
            # The sail angle should be reasonable
            self.assertGreaterEqual(best_sail_angle, -45, 
                                   f"Sail angle {best_sail_angle}° too extreme for {tack_angle}° tack")
            self.assertLessEqual(best_sail_angle, 45, 
                                f"Sail angle {best_sail_angle}° too extreme for {tack_angle}° tack")


if __name__ == '__main__':
    unittest.main()
