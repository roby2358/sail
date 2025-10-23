#!/usr/bin/env python3
"""Direct test of sail force calculation without external dependencies."""

import math
from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass
class SailParams:
    # Geometry & air
    rho: float = 1.225         # kg/m^3
    A: float = 20.0            # m^2 (reference area)
    AR: float = 4.5            # aspect ratio (span^2 / area), rough for a main+jib effective
    e: float = 0.85            # Oswald efficiency factor

    # Polar / section
    CD0: float = 0.015         # zero-lift (parasitic) drag
    alpha0_deg: float = -2.0   # zero-lift AoA (deg)

    # Stall model
    alpha_stall_deg: float = 15.0  # onset of stall (deg)
    alpha_max_deg: float = 35.0    # fully separated (deg) where model caps out
    min_poststall_CL_frac: float = 0.30  # CL fraction retained far past stall
    CD_surge_max: float = 0.8      # extra CD added at alpha_max


class SailForceCalculator:
    def __init__(self, params: SailParams = SailParams()):
        self.p = params

        # Geometry-driven induced-drag factor k = 1/(pi e AR)
        self.k = 1.0 / (math.pi * self.p.e * self.p.AR)

        # Finite-wing lift-curve slope (Helmbold-like approx): a = 2π AR / (2 + AR)
        self.CL_alpha = 2.0 * math.pi * self.p.AR / (2.0 + self.p.AR)  # per rad

    @staticmethod
    def deg2rad(deg: float) -> float:
        return math.radians(deg)

    @staticmethod
    def rad2deg(rad: float) -> float:
        return math.degrees(rad)

    def _stall_blend(self, eff_alpha_deg: float) -> Tuple[float, float]:
        """
        Returns (CL_mult, extra_CD) for post-stall handling.
        - CL_mult smoothly tapers from 1 at stall to min_poststall_CL_frac at alpha_max.
        - extra_CD rises from 0 at stall to CD_surge_max at alpha_max.
        """
        a = abs(eff_alpha_deg)
        if a <= self.p.alpha_stall_deg:
            return 1.0, 0.0

        # Normalize beyond stall
        denom = max(1e-6, self.p.alpha_max_deg - self.p.alpha_stall_deg)
        x = min(1.0, (a - self.p.alpha_stall_deg) / denom)

        # Smooth ease (cosine) from 1 -> min_CL and 0 -> surge
        cl_end = self.p.min_poststall_CL_frac
        CL_mult = cl_end + (1.0 - cl_end) * 0.5 * (1.0 + math.cos(math.pi * x))  # 1 at x=0 -> cl_end at x=1
        extra_CD = self.p.CD_surge_max * (x ** 2)  # gentle then strong rise

        return CL_mult, extra_CD

    def cl(self, alpha_deg: float) -> float:
        """
        Lift coefficient with zero-lift shift, finite-wing slope, and soft post-stall decay.
        """
        effective_alpha_deg = alpha_deg - self.p.alpha0_deg
        alpha_rad = self.deg2rad(effective_alpha_deg)

        cl_linear = self.CL_alpha * alpha_rad

        # Post-stall taper
        CL_mult, _ = self._stall_blend(effective_alpha_deg)
        CL = cl_linear * CL_mult

        # Hard cap for pathological angles
        alpha_cap = self.p.alpha_max_deg * 1.25
        if abs(effective_alpha_deg) > alpha_cap:
            CL *= 0.0

        return CL

    def cd(self, CL: float, alpha_deg: float) -> float:
        """
        Parabolic polar + post-stall drag surge.
        """
        CD = self.p.CD0 + self.k * CL * CL

        effective_alpha_deg = alpha_deg - self.p.alpha0_deg
        _, extra_CD = self._stall_blend(effective_alpha_deg)
        CD += extra_CD

        return CD

    @staticmethod
    def _polar_to_cart(V: float, ang_deg: float) -> Tuple[float, float]:
        th = math.radians(ang_deg)
        return (V * math.cos(th), V * math.sin(th))

    @staticmethod
    def _cart_to_polar(x: float, y: float) -> Tuple[float, float]:
        V = math.hypot(x, y)
        ang = math.degrees(math.atan2(y, x))
        return V, ang

    def apparent_wind(self, VT: float, beta_deg: float, U_boat: float) -> Tuple[float, float]:
        """
        beta_deg: true-wind *from* angle measured from bow (starboard positive).
        Returns Va and gamma_deg, where gamma is the apparent-wind *from* angle.
        """
        # Unit vector pointing toward the wind *source* ("from" direction)
        nx, ny = math.cos(math.radians(beta_deg)), math.sin(math.radians(beta_deg))
        # True wind velocity in Earth frame points opposite the "from" direction
        VTx, VTy = (-VT * nx, -VT * ny)

        # Boat velocity is +x (forward)
        VAx, VAy = (VTx - U_boat, VTy)

        # Apparent wind magnitude
        Va = math.hypot(VAx, VAy)

        # Convert relative-flow vector (VA) into a "from" angle by flipping its sign
        gamma = math.degrees(math.atan2(-VAy, -VAx))
        return Va, gamma

    def forces(self, Va: float, gamma_deg: float, delta_deg: float) -> Dict[str, float]:
        """
        Forces in boat axes (+x forward, +y starboard). gamma_deg is a "from" angle.
        """
        alpha_deg = gamma_deg - delta_deg
        CL = self.cl(alpha_deg)
        CD = self.cd(CL, alpha_deg)
        q = 0.5 * self.p.rho * Va * Va
        L = q * self.p.A * CL
        D = q * self.p.A * CD
        g = math.radians(gamma_deg)

        # Wind flows FROM gamma_deg, so it goes TO the opposite direction
        # Drag acts in the flow direction (downwind): (-cos g, -sin g)
        # Lift acts perpendicular to the flow direction
        # The lift direction depends on the angle of attack:
        # - Positive alpha (wind from starboard) creates lift to starboard
        # - Negative alpha (wind from port) creates lift to port
        alpha_rad = math.radians(alpha_deg)
        lift_sign = 1.0 if alpha_rad >= 0 else -1.0
        
        # Lift force components: perpendicular to wind direction
        # For wind FROM gamma_deg, lift is perpendicular in the direction determined by alpha
        lift_x = lift_sign * L * math.sin(g)  # lift component in x (forward/aft)
        lift_y = -lift_sign * L * math.cos(g)  # lift component in y (starboard/port)
        
        # Drag force components: along wind direction
        drag_x = -D * math.cos(g)  # drag component in x
        drag_y = -D * math.sin(g)  # drag component in y
        
        T = lift_x + drag_x   # + forward
        S = lift_y + drag_y   # + starboard

        return {"T": T, "S": S, "L": L, "D": D, "alpha_deg": alpha_deg, "CL": CL, "CD": CD, "q": q}


def test_port_tack_forces():
    """Test that wind from port can generate starboard force with proper trim."""
    calc = SailForceCalculator()
    
    # Wind from port (negative angle from bow)
    VT = 12.0  # m/s
    beta_deg = -45.0  # wind from port at 45 degrees
    U_boat = 0.0  # stationary boat initially
    
    # Get apparent wind
    Va, gamma = calc.apparent_wind(VT, beta_deg, U_boat)
    print(f"Wind from port {beta_deg}° → Apparent wind: {Va:.2f} m/s from {gamma:.1f}°")
    
    # Test various sail angles to find one that generates starboard force
    found_starboard_force = False
    best_starboard_force = 0.0
    best_delta = 0.0
    
    print("\nTesting sail angles for starboard force:")
    for delta_deg in range(-90, 91, 10):  # Test sail angles from -90 to +90 degrees
        forces = calc.forces(Va, gamma, delta_deg)
        S = forces["S"]  # Side force (positive = starboard)
        alpha_deg = forces["alpha_deg"]
        
        print(f"  Sail {delta_deg:3.0f}° → α={alpha_deg:5.1f}°, S={S:8.1f} N")
        
        if S > 0:  # Starboard force
            found_starboard_force = True
            if S > best_starboard_force:
                best_starboard_force = S
                best_delta = delta_deg
    
    if found_starboard_force:
        print(f"\n✓ SUCCESS: Found starboard force with sail at {best_delta}° (S = {best_starboard_force:.1f} N)")
        return True
    else:
        print(f"\n✗ FAILED: Could not generate starboard force with wind from port")
        return False


def test_starboard_tack_forces():
    """Test that wind from starboard can generate port force with proper trim."""
    calc = SailForceCalculator()
    
    # Wind from starboard (positive angle from bow)
    VT = 12.0  # m/s
    beta_deg = 45.0  # wind from starboard at 45 degrees
    U_boat = 0.0  # stationary boat initially
    
    # Get apparent wind
    Va, gamma = calc.apparent_wind(VT, beta_deg, U_boat)
    print(f"Wind from starboard {beta_deg}° → Apparent wind: {Va:.2f} m/s from {gamma:.1f}°")
    
    # Test various sail angles to find one that generates port force
    found_port_force = False
    best_port_force = 0.0
    best_delta = 0.0
    
    print("\nTesting sail angles for port force:")
    for delta_deg in range(-90, 91, 10):  # Test sail angles from -90 to +90 degrees
        forces = calc.forces(Va, gamma, delta_deg)
        S = forces["S"]  # Side force (negative = port)
        alpha_deg = forces["alpha_deg"]
        
        print(f"  Sail {delta_deg:3.0f}° → α={alpha_deg:5.1f}°, S={S:8.1f} N")
        
        if S < 0:  # Port force
            found_port_force = True
            if abs(S) > abs(best_port_force):
                best_port_force = S
                best_delta = delta_deg
    
    if found_port_force:
        print(f"\n✓ SUCCESS: Found port force with sail at {best_delta}° (S = {best_port_force:.1f} N)")
        return True
    else:
        print(f"\n✗ FAILED: Could not generate port force with wind from starboard")
        return False


def main():
    print("Testing sail force fix...")
    print("=" * 50)
    
    port_success = test_port_tack_forces()
    print("\n" + "=" * 50)
    starboard_success = test_starboard_tack_forces()
    
    print("\n" + "=" * 50)
    if port_success and starboard_success:
        print("✓ ALL TESTS PASSED: Sail forces work correctly on both tacks!")
    else:
        print("✗ SOME TESTS FAILED: Sail force fix needs more work")
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())
