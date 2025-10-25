import math
from .sails import Sails


def wrap_deg(a: float) -> float:
    """Wrap angle to [-180, 180] degrees."""
    a = (a + 180.0) % 360.0 - 180.0
    return a


class Ship:
    """
    2D ship dynamics simulation with realistic physics.
    
    Handles:
    - 6-DOF ship state (position, heading, velocities, yaw rate)
    - Sail force integration via Sails class
    - Rudder dynamics with speed-dependent effectiveness
    - Water resistance and keel damping
    - Sail optimization for close-hauled sailing
    """
    
    def __init__(self, sail_calc: Sails):
        self.sc = sail_calc
        self.p = sail_calc.p  # Use the same params from the sail calculator
        # World state (meters, radians)
        self.x = 400.0
        self.y = 300.0
        self.psi = 0.0  # heading rad (0 = east, +CCW)
        self.u = 0.0    # forward speed m/s (body x)
        self.v = 0.0    # lateral speed m/s (body y, +starboard)
        self.r = 0.0    # yaw rate rad/s
        # Controls
        self.delta_sail_deg = 30.0  # yard angle from centerline (deg)
        self.delta_rudder_deg = 0.0

    def step(self, dt: float, VT: float, wind_from_global_deg: float):
        """
        Advance ship simulation by one time step.
        
        Args:
            dt: Time step in seconds
            VT: True wind speed (m/s)
            wind_from_global_deg: Global wind direction (degrees, "from" angle)
            
        Returns:
            Tuple of (aero_forces_dict, apparent_wind_speed, apparent_wind_angle)
        """
        # Convert global wind-from to boat-relative from angle beta
        boat_from_wind_deg = wrap_deg(wind_from_global_deg - math.degrees(self.psi))

        # Apparent wind using forward speed only (simple):
        Va, gamma_from_deg = self.sc.apparent_wind(VT, boat_from_wind_deg, self.u)

        # Aero forces in body axes
        aero = self.sc.forces(Va, gamma_from_deg, self.delta_sail_deg)
        T, S = aero["T"], aero["S"]

        # Add simple quadratic water drag already modeled as linear-ish damping for stability
        Fx = T - self.p.surge_damp * self.u
        Fy = S - self.p.sway_damp * self.v

        # Rudder yaw moment and side-force lever arm yaw
        delta_r = math.radians(max(-self.p.max_rudder_deg, min(self.p.max_rudder_deg, self.delta_rudder_deg)))
        
        # Speed-dependent rudder effectiveness based on water speed (|velocity|)^2
        # Use combined body speed so rudder stays effective when there is sway as well as surge
        reference_speed = 3.0  # m/s reference speed for nominal rudder effectiveness
        boat_speed = math.hypot(self.u, self.v)
        speed_factor = max(0.1, min(3.0, (boat_speed * boat_speed) / (reference_speed * reference_speed)))
        rudder_force = self.p.rudder_N_per_rad * delta_r * speed_factor

        # Keel-like yaw damping grows with water speed to prevent spinning in place
        # Stronger keel damping at high speeds: cubic growth with a higher cap
        keel_ratio = (boat_speed / reference_speed)
        keel_damp_factor = min(10.0, max(0.0, keel_ratio ** 3))
        yaw_damp_eff = self.p.yaw_damp * (1.0 + keel_damp_factor)

        N = self.p.lever_S * S + rudder_force - yaw_damp_eff * self.r

        # Integrate body velocities
        ax = Fx / self.p.mass
        ay = Fy / self.p.mass
        self.u += ax * dt
        self.v += ay * dt
        self.r += (N / self.p.Iz) * dt

        # Integrate pose: convert body velocities to world frame
        c, s = math.cos(self.psi), math.sin(self.psi)
        Vx = c * self.u - s * self.v
        Vy = s * self.u + c * self.v
        self.x += Vx * dt * 8.0  # 8x the distance travelled for faster movement
        self.y += Vy * dt * 8.0  # 8x the distance travelled for faster movement
        self.psi += self.r * dt

        # Keep heading wrapped
        if self.psi > math.pi:
            self.psi -= 2 * math.pi
        elif self.psi < -math.pi:
            self.psi += 2 * math.pi

        return aero, Va, gamma_from_deg

    def optimize_sail_for_close_hauled(self, VT: float, wind_from_global_deg: float) -> float:
        """
        Optimize sail angle for close-hauled sailing (upwind performance).
        Returns the optimal sail angle for maximum forward thrust.
        """
        boat_from_wind_deg = wrap_deg(wind_from_global_deg - math.degrees(self.psi))
        Va, gamma_from_deg = self.sc.apparent_wind(VT, boat_from_wind_deg, self.u)
        
        best_thrust = -float('inf')
        best_sail_angle = 0.0
        
        # Test sail angles in realistic close-hauled range
        for sail_angle in range(-60, 61, 5):  # -60 to +60 degrees in 5-degree steps
            forces = self.sc.forces(Va, gamma_from_deg, sail_angle)
            if forces["T"] > best_thrust:
                best_thrust = forces["T"]
                best_sail_angle = sail_angle
        
        return best_sail_angle
