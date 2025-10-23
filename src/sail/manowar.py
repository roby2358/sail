import math
import pygame
from dataclasses import dataclass
from typing import Dict, Tuple

# -----------------------------
# Aero model (single equivalent "wing")
# -----------------------------
@dataclass
class SailParams:
    # Geometry & air
    rho: float = 1.225         # kg/m^3
    A: float = 1900.0          # m^2 effective projected area (Full Press default)
    AR: float = 1.6            # aspect ratio (span^2 / area)
    e: float = 0.65            # Oswald efficiency factor

    # Polar / section
    CD0: float = 0.055         # zero-lift (parasitic) drag (rigging + hull windage baked in)
    alpha0_deg: float = +0.5   # zero-lift AoA (deg)

    # Stall model
    alpha_stall_deg: float = 12.0  # onset of stall (deg)
    alpha_max_deg: float = 65.0    # fully separated (deg)
    min_poststall_CL_frac: float = 0.20  # CL fraction retained far past stall
    CD_surge_max: float = 1.35      # extra CD added at alpha_max


class SailForceCalculator:
    def __init__(self, params: SailParams = SailParams()):
        self.p = params
        self.k = 1.0 / (math.pi * self.p.e * self.p.AR)  # induced-drag factor
        self.CL_alpha = 2.0 * math.pi * self.p.AR / (2.0 + self.p.AR)  # per rad (Helmbold approx)

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
        D = q * self.p.A * CD
        g = math.radians(gamma_from_deg)
        # Drag along (cos g, sin g) as FROM direction → force on boat is toward the FROM: (+cos g, +sin g)
        # Using established decomposition (see discussion):
        T = L * math.sin(g) - D * math.cos(g)   # + forward
        S = -L * math.cos(g) - D * math.sin(g)  # + starboard
        return {"T": T, "S": S, "L": L, "D": D, "alpha_deg": alpha_deg, "CL": CL, "CD": CD, "q": q}


# -----------------------------
# Toy ship dynamics (2D top-down)
# -----------------------------
@dataclass
class ShipParams:
    mass: float = 175000.0     # kg (ballpark for a 1st rate fully loaded)
    Iz: float = 6.0e6          # yaw inertia kg·m^2 (very rough)
    sway_damp: float = 2.0e5   # N·(m/s) for lateral velocity
    surge_damp: float = 6.0e4  # N·(m/s) for forward velocity
    yaw_damp: float = 4.0e7    # N·m per rad/s
    lever_S: float = 4.0       # m: side-force lever arm creating yaw
    rudder_N_per_rad: float = 6.0e7  # N·m per rad rudder effectiveness
    max_rudder_deg: float = 35.0


class Ship:
    def __init__(self, sail_calc: SailForceCalculator, ship_params: ShipParams = ShipParams()):
        self.sc = sail_calc
        self.p = ship_params
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
        N = self.p.lever_S * S + self.p.rudder_N_per_rad * delta_r - self.p.yaw_damp * self.r

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
        self.x += Vx * dt
        self.y += Vy * dt
        self.psi += self.r * dt

        # Keep heading wrapped
        if self.psi > math.pi:
            self.psi -= 2 * math.pi
        elif self.psi < -math.pi:
            self.psi += 2 * math.pi

        return aero, Va, gamma_from_deg


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

    params_full = SailParams()
    params_battle = SailParams(A=1100.0, AR=1.4, e=0.58, CD0=0.060,
                               alpha_stall_deg=10.0, alpha_max_deg=60.0,
                               min_poststall_CL_frac=0.18, CD_surge_max=1.25)

    calc = SailForceCalculator(params_full)
    ship = Ship(calc)

    # Environment
    wind_from_global_deg = 45.0  # FROM angle in world frame
    VT = 12.0  # m/s

    # Destination buoy
    import random
    buoy_x = random.randint(100, WIDTH - 100)
    buoy_y = random.randint(100, HEIGHT - 100)
    buoy_radius = 30
    buoy_capture_distance = 80
    buoy_captures = 0

    paused = False
    using_battle = False

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
                    ship.__init__(calc)  # reset
                if e.key == pygame.K_RETURN:
                    ship.x = WIDTH // 2
                    ship.y = HEIGHT // 2
                if e.key == pygame.K_t:
                    # Move buoy to new random location
                    buoy_x = random.randint(100, WIDTH - 100)
                    buoy_y = random.randint(100, HEIGHT - 100)
                if e.key == pygame.K_1:
                    calc.p = params_full; calc.__init__(calc.p); using_battle = False
                if e.key == pygame.K_2:
                    calc.p = params_battle; calc.__init__(calc.p); using_battle = True

        keys = pygame.key.get_pressed()
        # Rudder
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            ship.delta_rudder_deg = max(-ship.p.max_rudder_deg, ship.delta_rudder_deg - 40.0 * dt)
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            ship.delta_rudder_deg = min(ship.p.max_rudder_deg, ship.delta_rudder_deg + 40.0 * dt)
        else:
            # auto-center rudder
            ship.delta_rudder_deg *= (1.0 - min(1.0, 3.0 * dt))

        # Sail trim
        if keys[pygame.K_q]:
            ship.delta_sail_deg = max(-85.0, ship.delta_sail_deg - 30.0 * dt)
        if keys[pygame.K_e]:
            ship.delta_sail_deg = min(85.0, ship.delta_sail_deg + 30.0 * dt)

        # Wind controls
        if keys[pygame.K_z]:
            VT = max(0.0, VT - 2.0 * dt)
        if keys[pygame.K_x]:
            VT = min(30.0, VT + 2.0 * dt)
        if keys[pygame.K_c]:
            wind_from_global_deg = wrap_deg(wind_from_global_deg - 20.0 * dt)
        if keys[pygame.K_v]:
            wind_from_global_deg = wrap_deg(wind_from_global_deg + 20.0 * dt)

        if not paused:
            aero, Va, gamma = ship.step(dt, VT, wind_from_global_deg)
            
            # Check if ship reached the buoy
            distance_to_buoy = math.hypot(ship.x - buoy_x, ship.y - buoy_y)
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
        draw_arrow(screen, 60, 120, math.radians(gamma) and gamma + math.degrees(ship.psi), 50, YELLOW, 3)
        pygame.draw.circle(screen, WHITE, (60, 60), 4)
        pygame.draw.circle(screen, WHITE, (60, 120), 4)

        # Draw destination buoy
        pygame.draw.circle(screen, ORANGE, (int(buoy_x), int(buoy_y)), buoy_radius)
        pygame.draw.circle(screen, WHITE, (int(buoy_x), int(buoy_y)), buoy_radius, 3)
        # Draw capture zone (dashed circle)
        pygame.draw.circle(screen, (255, 255, 255, 100), (int(buoy_x), int(buoy_y)), buoy_capture_distance, 2)
        
        # Draw ship (triangle)
        ship_len = 40
        ship_wid = 13
        c, s = math.cos(ship.psi), math.sin(ship.psi)
        p1 = (ship.x + c * ship_len * 0.6,               ship.y + s * ship_len * 0.6)
        p2 = (ship.x - c * ship_len * 0.4 - s * ship_wid, ship.y - s * ship_len * 0.4 + c * ship_wid)
        p3 = (ship.x - c * ship_len * 0.4 + s * ship_wid, ship.y - s * ship_len * 0.4 - c * ship_wid)
        pygame.draw.polygon(screen, (180, 180, 200), [p1, p2, p3])

        # Draw yards as a line across the mast, orientation = ship heading + delta_sail
        yard_ang = ship.psi + math.radians(ship.delta_sail_deg)
        ylen = 35
        x1 = ship.x - math.cos(yard_ang) * ylen
        y1 = ship.y - math.sin(yard_ang) * ylen
        x2 = ship.x + math.cos(yard_ang) * ylen
        y2 = ship.y + math.sin(yard_ang) * ylen
        pygame.draw.line(screen, (210, 210, 230), (x1, y1), (x2, y2), 4)

        # Force vectors in body axes mapped to world frame at ship position
        scale = 0.0012  # pixels per Newton for drawing
        # Forward (x) and starboard (y) forces to world axes
        Fx_body, Fy_body = aero.get("T", 0.0), aero.get("S", 0.0)
        Fxw = math.cos(ship.psi) * Fx_body - math.sin(ship.psi) * Fy_body
        Cyw = math.sin(ship.psi) * Fx_body + math.cos(ship.psi) * Fy_body
        draw_arrow(screen, ship.x, ship.y, math.degrees(math.atan2(Cyw, Fxw)), scale * math.hypot(Fxw, Cyw), GREEN, 3)
        
        # Momentum arrow (dark green) - shows ship's velocity vector
        if abs(ship.u) > 1e-6 or abs(ship.v) > 1e-6:
            momentum_mag = math.hypot(ship.u, ship.v)
            momentum_ang = math.degrees(math.atan2(ship.v, ship.u))
            momentum_scale = 40.0  # pixels per m/s for drawing
            # Reverse the angle to show where the ship is going (not where it came from)
            draw_arrow(screen, ship.x, ship.y, momentum_ang + 180, momentum_scale * momentum_mag, DARK_GREEN, 2)

        # HUD
        distance_to_buoy = math.hypot(ship.x - buoy_x, ship.y - buoy_y)
        lines = [
            f"Preset: {'Battle Sails' if using_battle else 'Full Press'} (1/2 to toggle)",
            f"Wind FROM: {wind_from_global_deg:5.1f}°  |  VT: {VT:4.1f} m/s",
            f"Heading: {math.degrees(ship.psi)%360:5.1f}°  Speed u: {ship.u:5.2f} m/s  v: {ship.v:5.2f} m/s",
            f"Sail δ: {ship.delta_sail_deg:5.1f}°   Rudder: {ship.delta_rudder_deg:5.1f}°",
            f"Va: {Va:5.2f} m/s   γ_app FROM: {gamma:5.1f}°   α: {aero.get('alpha_deg',0):5.1f}°",
            f"T: {aero.get('T',0):8.0f} N   S: {aero.get('S',0):8.0f} N   L: {aero.get('L',0):8.0f} N   D: {aero.get('D',0):8.0f} N",
            f"Distance to buoy: {distance_to_buoy:5.1f} pixels  |  Capture: {buoy_capture_distance} pixels  |  Captures: {buoy_captures}",
            "Controls — A/D or ←/→: rudder  |  Q/E: trim  |  Z/X: wind speed  |  C/V: wind dir  |  SPACE: pause  |  R: reset  |  T: move buoy"
        ]
        for i, txt in enumerate(lines):
            surf = font.render(txt, True, WHITE)
            screen.blit(surf, (20, HEIGHT - 20 * (len(lines) - i)))

        pygame.display.flip()


if __name__ == "__main__":
    main()
