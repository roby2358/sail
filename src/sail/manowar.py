import math
import pygame
from dataclasses import dataclass
from typing import Dict, Tuple
from sail.sail_forces import SailParams

# -----------------------------
# Aero model (single equivalent "wing")
# -----------------------------
# SailParams is now imported from sail_forces.py


class SailForceCalculator:
    def __init__(self, params: SailParams = SailParams()):
        self.p = params
        self.k = 1.0 / (math.pi * self.p.e * self.p.AR)  # induced-drag factor
        self.CL_alpha = 2.0 * math.pi * self.p.AR / (2.0 + self.p.AR)  # per rad (Helmbold approx)
        # Magnify only lift (not drag) to improve upwind drive without increasing drag
        self.lift_gain = 1.8

    @staticmethod
    def deg2rad(deg: float) -> float:
        return math.radians(deg)

    @staticmethod
    def rad2deg(rad: float) -> float:
        return math.degrees(rad)

    def _stall_blend(self, eff_alpha_deg: float) -> Tuple[float, float]:
        a = abs(eff_alpha_deg)
        if a <= self.p.alpha_stall_deg:
            return 1.0, 0.0
        denom = max(1e-6, self.p.alpha_max_deg - self.p.alpha_stall_deg)
        x = min(1.0, (a - self.p.alpha_stall_deg) / denom)
        cl_end = self.p.min_poststall_CL_frac
        CL_mult = cl_end + (1.0 - cl_end) * 0.5 * (1.0 + math.cos(math.pi * x))
        extra_CD = self.p.CD_surge_max * (x ** 2)
        return CL_mult, extra_CD

    def cl(self, alpha_deg: float) -> float:
        effective_alpha_deg = alpha_deg - self.p.alpha0_deg
        alpha_rad = self.deg2rad(effective_alpha_deg)
        cl_linear = self.CL_alpha * alpha_rad
        CL_mult, _ = self._stall_blend(effective_alpha_deg)
        CL = cl_linear * CL_mult
        return CL

    def cd(self, CL: float, alpha_deg: float) -> float:
        CD = self.p.CD0 + self.k * CL * CL
        effective_alpha_deg = alpha_deg - self.p.alpha0_deg
        _, extra_CD = self._stall_blend(effective_alpha_deg)
        CD += extra_CD
        return CD

    def apparent_wind(self, VT: float, beta_from_deg: float, U_boat_forward: float) -> Tuple[float, float]:
        """
        beta_from_deg: true-wind *from* angle relative to boat bow (starboard positive).
        Returns Va and gamma_from_deg (apparent-wind *from* angle).
        Only the forward speed U_boat_forward is considered here for simplicity.
        """
        nx, ny = math.cos(math.radians(beta_from_deg)), math.sin(math.radians(beta_from_deg))
        VTx, VTy = (-VT * nx, -VT * ny)  # wind velocity points TO opposite of the from-angle
        VAx, VAy = (VTx - U_boat_forward, VTy)  # subtract boat velocity (forward only)
        Va = math.hypot(VAx, VAy)
        gamma_from = math.degrees(math.atan2(-VAy, -VAx))  # convert to a from-angle
        return Va, gamma_from

    def forces(self, Va: float, gamma_from_deg: float, delta_deg: float) -> Dict[str, float]:
        """Forces in boat axes (+x forward, +y starboard). gamma is a FROM angle."""
        alpha_deg = gamma_from_deg - delta_deg
        CL = self.cl(alpha_deg)
        CD = self.cd(CL, alpha_deg)
        q = 0.5 * self.p.rho * Va * Va
        L = q * self.p.A * CL
        L *= self.lift_gain  # apply lift-only gain
        D = q * self.p.A * CD
        g = math.radians(gamma_from_deg)
        # Use same decomposition as sail_forces: CL carries AoA sign already
        T = L * math.sin(g) - D * math.cos(g)
        S = -L * math.cos(g) - D * math.sin(g)
        return {"T": T, "S": S, "L": L, "D": D, "alpha_deg": alpha_deg, "CL": CL, "CD": CD, "q": q}


# -----------------------------
# Toy ship dynamics (2D top-down)
# -----------------------------
# Ship dynamics parameters are now part of SailParams


class Ship:
    def __init__(self, sail_calc: SailForceCalculator):
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



# -----------------------------
# Utility
# -----------------------------

def wrap_deg(a: float) -> float:
    a = (a + 180.0) % 360.0 - 180.0
    return a


# -----------------------------
# Pygame viz
# -----------------------------

WIDTH, HEIGHT = 1000, 700
BG = (12, 18, 28)
WHITE = (230, 230, 230)
CYAN = (90, 200, 220)
GREEN = (120, 200, 120)
DARK_GREEN = (60, 120, 60)
RED = (230, 100, 100)
YELLOW = (240, 220, 120)
ORANGE = (255, 165, 0)


def draw_arrow(surf, x, y, ang_deg, length, color, width=2):
    th = math.radians(ang_deg)
    x2 = x + length * math.cos(th)
    y2 = y + length * math.sin(th)
    pygame.draw.line(surf, color, (x, y), (x2, y2), width)
    # head
    h = 10
    left = (x2 + -h * math.cos(th) - h * math.sin(th), y2 + -h * math.sin(th) + h * math.cos(th))
    right = (x2 + -h * math.cos(th) + h * math.sin(th), y2 + -h * math.sin(th) - h * math.cos(th))
    pygame.draw.polygon(surf, color, [(x2, y2), left, right])


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Man o' War — square‑rig toy sim")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 18)

    # Sailboat configuration (default)
    params_sailboat = SailParams(
        A=50.0,              # m^2 (reduced sail area for more manageable power)
        AR=6.0,              # High aspect ratio for efficiency
        e=0.90,              # Excellent efficiency with slot effect
        CD0=0.008,           # Very low parasitic drag
        alpha0_deg=-3.0,     # Better zero-lift angle for upwind
        alpha_stall_deg=18.0, # Later stall for better upwind performance
        alpha_max_deg=45.0,   # More forgiving stall characteristics
        min_poststall_CL_frac=0.40,  # Better post-stall performance
        CD_surge_max=0.6,    # Less drag surge when overtrimmed
        ship_type="sailboat"  # Ship type
    )
    
    # Man-of-war full press configuration
    params_manowar_full = SailParams(
        A=800.0,             # m^2 effective projected area (reduced for more manageable power)
        AR=3.5,              # aspect ratio (slightly higher)
        e=0.80,              # Oswald efficiency factor (improved)
        CD0=0.025,           # zero-lift drag (reduced)
        alpha0_deg=-2.0,     # zero-lift AoA (better for upwind)
        alpha_stall_deg=15.0, # onset of stall (later stall)
        alpha_max_deg=60.0,   # fully separated (more forgiving)
        min_poststall_CL_frac=0.30,  # CL fraction retained (better post-stall)
        CD_surge_max=1.20,   # extra CD added (reduced drag surge)
        # Ship dynamics for man-of-war
        mass=175000.0,       # kg (ballpark for a 1st rate fully loaded)
        Iz=6.0e6,            # yaw inertia kg·m^2
        sway_damp=2.0e5,     # N·(m/s) for lateral velocity
        surge_damp=6.0e4,    # N·(m/s) for forward velocity
        yaw_damp=2.0e8,      # N·m per rad/s (increased significantly)
        lever_S=4.0,         # m: side-force lever arm
        rudder_N_per_rad=6.0e7,  # N·m per rad rudder effectiveness (restored to reasonable level)
        max_rudder_deg=35.0, # Full rudder range
        ship_type="manowar"  # Ship type
    )
    
    # Man-of-war battle sails configuration
    params_manowar_battle = SailParams(
        A=500.0,             # m^2 reduced effective area (reduced for more manageable power)
        AR=3.0,              # slightly higher aspect ratio
        e=0.75,              # improved efficiency
        CD0=0.030,           # reduced parasitic drag
        alpha0_deg=-2.0,     # zero-lift AoA (better for upwind)
        alpha_stall_deg=12.0, # onset of stall (later than before)
        alpha_max_deg=55.0,   # fully separated (more forgiving)
        min_poststall_CL_frac=0.25,  # CL fraction retained (better post-stall)
        CD_surge_max=1.10,   # extra CD added (reduced drag surge)
        # Ship dynamics for man-of-war (same as full press)
        mass=175000.0,       # kg
        Iz=6.0e6,            # yaw inertia kg·m^2
        sway_damp=2.0e5,     # N·(m/s) for lateral velocity
        surge_damp=6.0e4,    # N·(m/s) for forward velocity
        yaw_damp=2.0e8,      # N·m per rad/s (increased significantly)
        lever_S=4.0,         # m: side-force lever arm
        rudder_N_per_rad=6.0e7,  # N·m per rad rudder effectiveness (restored to reasonable level)
        max_rudder_deg=35.0, # Full rudder range
        ship_type="battle"   # Ship type
    )

    # Create current ship (sailboat by default)
    current_ship = Ship(SailForceCalculator(params_sailboat))
    
    # Store ship configurations for switching
    ship_configs = {
        "sailboat": SailForceCalculator(params_sailboat),
        "manowar_full": SailForceCalculator(params_manowar_full),
        "manowar_battle": SailForceCalculator(params_manowar_battle)
    }

    # Environment — fix wind FROM due East (0°) permanently
    wind_from_global_deg = 0.0  # FROM angle in world frame (0° = East)
    VT = 8.0  # m/s (increased wind speed for better upwind performance)

    # Destination buoy
    import random
    buoy_x = random.randint(100, WIDTH - 100)
    buoy_y = random.randint(100, HEIGHT - 100)
    buoy_radius = 7.5
    buoy_capture_distance = 20
    buoy_captures = 0

    paused = False

    while True:
        dt = clock.tick(60) / 1000.0
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                pygame.quit()
                return
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    pygame.quit(); return
                if e.key == pygame.K_SPACE:
                    paused = not paused
                if e.key == pygame.K_r:
                    # Reset current ship
                    calc = ship_configs[current_ship.p.ship_type]
                    current_ship.__init__(calc)
                if e.key == pygame.K_RETURN:
                    current_ship.x = WIDTH // 2
                    current_ship.y = HEIGHT // 2
                if e.key == pygame.K_s:
                    # Instantly center the rudder
                    current_ship.delta_rudder_deg = 0.0
                if e.key == pygame.K_t:
                    # Move buoy to new random location
                    buoy_x = random.randint(100, WIDTH - 100)
                    buoy_y = random.randint(100, HEIGHT - 100)
                if e.key == pygame.K_1:
                    # Switch to sailboat
                    current_ship = Ship(ship_configs["sailboat"])
                if e.key == pygame.K_2:
                    # Switch to man-of-war (full press)
                    current_ship = Ship(ship_configs["manowar_full"])
                if e.key == pygame.K_3:
                    # Switch to man-of-war (battle sails)
                    current_ship = Ship(ship_configs["manowar_battle"])
                if e.key == pygame.K_o:
                    # Optimize sail for close-hauled sailing
                    optimal_sail = current_ship.optimize_sail_for_close_hauled(VT, wind_from_global_deg)
                    current_ship.delta_sail_deg = optimal_sail
        
        keys = pygame.key.get_pressed()
        # Rudder
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            current_ship.delta_rudder_deg = max(-current_ship.p.max_rudder_deg, current_ship.delta_rudder_deg - 60.0 * dt)
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            current_ship.delta_rudder_deg = min(current_ship.p.max_rudder_deg, current_ship.delta_rudder_deg + 60.0 * dt)
        # Rudder stays where you put it (no auto-centering)

        # Sail trim - realistic close-hauled sailing (more responsive)
        if keys[pygame.K_q]:
            current_ship.delta_sail_deg -= 120.0 * dt  # Faster trim response
            # Limit to realistic close-hauled range for upwind sailing
            if current_ship.delta_sail_deg < -60.0:
                current_ship.delta_sail_deg = -60.0
        if keys[pygame.K_e]:
            current_ship.delta_sail_deg += 120.0 * dt  # Faster trim response
            # Limit to realistic close-hauled range for upwind sailing
            if current_ship.delta_sail_deg > 60.0:
                current_ship.delta_sail_deg = 60.0

        # Wind controls
        if keys[pygame.K_x]:
            VT = min(15.0, VT + 1.0 * dt)  # Slower wind increase, lower max
        if keys[pygame.K_z]:
            wind_from_global_deg = wrap_deg(wind_from_global_deg - 20.0 * dt)
        if keys[pygame.K_c]:
            wind_from_global_deg = wrap_deg(wind_from_global_deg + 20.0 * dt)

        if not paused:
            aero, Va, gamma = current_ship.step(dt, VT, wind_from_global_deg)
            
            # Check if ship reached the buoy
            distance_to_buoy = math.hypot(current_ship.x - buoy_x, current_ship.y - buoy_y)
            if distance_to_buoy < buoy_capture_distance:
                # Move buoy to new random location
                buoy_x = random.randint(100, WIDTH - 100)
                buoy_y = random.randint(100, HEIGHT - 100)
                buoy_captures += 1
        else:
            aero, Va, gamma = {"T":0,"S":0,"L":0,"D":0,"alpha_deg":0,"CL":0,"CD":0,"q":0}, 0, 0

        # ----- Render
        screen.fill(BG)

        # Draw wind arrows at top-left (from and apparent-from)
        draw_arrow(screen, 60, 60, math.radians(wind_from_global_deg) and wind_from_global_deg, 50, CYAN, 3)
        draw_arrow(screen, 60, 120, math.radians(gamma) and gamma + math.degrees(current_ship.psi), 50, YELLOW, 3)
        pygame.draw.circle(screen, WHITE, (60, 60), 4)
        pygame.draw.circle(screen, WHITE, (60, 120), 4)

        # Draw destination buoy
        pygame.draw.circle(screen, ORANGE, (int(buoy_x), int(buoy_y)), buoy_radius)
        pygame.draw.circle(screen, WHITE, (int(buoy_x), int(buoy_y)), buoy_radius, 3)
        # Draw capture zone (dashed circle)
        pygame.draw.circle(screen, (255, 255, 255, 100), (int(buoy_x), int(buoy_y)), buoy_capture_distance, 2)
        
        # Draw ship (triangle) - different size based on ship type
        if current_ship.p.ship_type == "sailboat":
            ship_len = 25  # Much bigger for sailboat visibility
            ship_wid = 8
            ship_color = (120, 200, 255)  # Light blue for sailboat
        else:
            ship_len = 30  # Bigger for man-of-war too
            ship_wid = 10
            ship_color = (180, 180, 200)  # Gray for man-of-war
            
        c, s = math.cos(current_ship.psi), math.sin(current_ship.psi)
        p1 = (current_ship.x + c * ship_len * 0.6,               current_ship.y + s * ship_len * 0.6)
        p2 = (current_ship.x - c * ship_len * 0.4 - s * ship_wid, current_ship.y - s * ship_len * 0.4 + c * ship_wid)
        p3 = (current_ship.x - c * ship_len * 0.4 + s * ship_wid, current_ship.y - s * ship_len * 0.4 - c * ship_wid)
        pygame.draw.polygon(screen, ship_color, [p1, p2, p3])

        # Draw sails - different for sailboat vs man-of-war
        if current_ship.p.ship_type == "sailboat":
            # Draw main sail as a simple white line - in ship body coordinates
            sail_ang = math.radians(current_ship.delta_sail_deg)  # Only use sail trim angle
            sail_len = 20
            # Mast position in ship body coordinates (4 units forward from center)
            mast_x_body = 4
            mast_y_body = 0
            # Convert mast position to world coordinates
            sx1 = current_ship.x + c * mast_x_body - s * mast_y_body
            sy1 = current_ship.y + s * mast_x_body + c * mast_y_body
            # Sail end position in ship body coordinates
            sail_x_body = -math.cos(sail_ang) * sail_len  # Negative for port/starboard mirroring
            sail_y_body = math.sin(sail_ang) * sail_len
            # Convert sail end to world coordinates
            sx2 = current_ship.x + c * sail_x_body - s * sail_y_body
            sy2 = current_ship.y + s * sail_x_body + c * sail_y_body
            pygame.draw.line(screen, WHITE, (sx1, sy1), (sx2, sy2), 3)
        else:
            # Draw yards as a line across the mast (man-of-war) - in ship body coordinates
            yard_ang = math.radians(current_ship.delta_sail_deg)  # Only use sail trim angle
            ylen = 17.5
            # Yard end positions in ship body coordinates
            yard1_x_body = math.cos(yard_ang) * ylen  # Positive for port/starboard mirroring
            yard1_y_body = -math.sin(yard_ang) * ylen
            yard2_x_body = -math.cos(yard_ang) * ylen  # Negative for port/starboard mirroring
            yard2_y_body = math.sin(yard_ang) * ylen
            # Convert to world coordinates
            x1 = current_ship.x + c * yard1_x_body - s * yard1_y_body
            y1 = current_ship.y + s * yard1_x_body + c * yard1_y_body
            x2 = current_ship.x + c * yard2_x_body - s * yard2_y_body
            y2 = current_ship.y + s * yard2_x_body + c * yard2_y_body
            pygame.draw.line(screen, (210, 210, 230), (x1, y1), (x2, y2), 4)

        # Force vectors in body axes mapped to world frame at ship position
        scale = 0.0006  # pixels per Newton for drawing
        # Forward (x) and starboard (y) forces to world axes
        Fx_body, Fy_body = aero.get("T", 0.0), aero.get("S", 0.0)
        Fxw = math.cos(current_ship.psi) * Fx_body - math.sin(current_ship.psi) * Fy_body
        Cyw = math.sin(current_ship.psi) * Fx_body + math.cos(current_ship.psi) * Fy_body
        draw_arrow(screen, current_ship.x, current_ship.y, math.degrees(math.atan2(Cyw, Fxw)), scale * math.hypot(Fxw, Cyw), GREEN, 3)
        
        # Momentum arrow (dark green) - shows ship's velocity vector
        if abs(current_ship.u) > 1e-6 or abs(current_ship.v) > 1e-6:
            # Convert ship body velocities to world frame
            uw = math.cos(current_ship.psi) * current_ship.u - math.sin(current_ship.psi) * current_ship.v
            vw = math.sin(current_ship.psi) * current_ship.u + math.cos(current_ship.psi) * current_ship.v
            momentum_mag = math.hypot(uw, vw)
            momentum_ang = math.degrees(math.atan2(vw, uw))
            momentum_scale = 20.0  # pixels per m/s for drawing
            draw_arrow(screen, current_ship.x, current_ship.y, momentum_ang, momentum_scale * momentum_mag, DARK_GREEN, 2)

        # HUD
        distance_to_buoy = math.hypot(current_ship.x - buoy_x, current_ship.y - buoy_y)
        ship_type_display = {
            "sailboat": "Sailboat",
            "manowar": "Full Press", 
            "battle": "Battle Sails"
        }.get(current_ship.p.ship_type, "Unknown")
        lines = [
            f"Ship: {ship_type_display} (1/2/3 to toggle) |  Mass: {current_ship.p.mass/1000:.0f}k kg",
            f"Wind FROM: {wind_from_global_deg:5.1f}°  |  VT: {VT:4.1f} m/s",
            f"Heading: {math.degrees(current_ship.psi)%360:5.1f}°  Speed u: {current_ship.u:5.2f} m/s  v: {current_ship.v:5.2f} m/s",
            f"Sail δ: {current_ship.delta_sail_deg:5.1f}°   Rudder: {current_ship.delta_rudder_deg:5.1f}°",
            f"Va: {Va:5.2f} m/s   γ_app FROM: {gamma:5.1f}°   α: {aero.get('alpha_deg',0):5.1f}°",
            f"T: {aero.get('T',0):8.0f} N   S: {aero.get('S',0):8.0f} N   L: {aero.get('L',0):8.0f} N   D: {aero.get('D',0):8.0f} N",
            f"Rudder: {current_ship.delta_rudder_deg:5.1f}°  Yaw rate: {current_ship.r:6.3f} rad/s  Heading: {math.degrees(current_ship.psi):6.1f}°",
            f"Speed factor: {max(0.1, min(3.0, (math.hypot(current_ship.u, current_ship.v)**2) / 9.0)):.2f}  Rudder force: {current_ship.p.rudder_N_per_rad * math.radians(current_ship.delta_rudder_deg) * max(0.1, min(3.0, (math.hypot(current_ship.u, current_ship.v)**2) / 9.0)):.0f} N",
            f"Distance to buoy: {distance_to_buoy:5.1f} pixels  |  Capture: {buoy_capture_distance} pixels  |  Captures: {buoy_captures}",
            "Controls — A/D or ←/→: rudder  |  S: center rudder  |  Q/E: trim  |  X: wind speed+  |  Z/C: wind dir −/+  |  SPACE: pause  |  R: reset  |  T: move buoy  |  O: optimize sail",
            "Tacking: Sail at 45-60° to wind, trim sails (Q/E) for close-hauled sailing, use rudder to zigzag upwind"
        ]
        for i, txt in enumerate(lines):
            surf = font.render(txt, True, WHITE)
            screen.blit(surf, (20, HEIGHT - 20 * (len(lines) - i)))

        pygame.display.flip()


if __name__ == "__main__":
    main()
