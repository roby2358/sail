import math
import pygame
from dataclasses import dataclass
from typing import Dict, Tuple
from .sail_params import SailParams, SAILBOAT, MANOWAR_FULL_PRESS, MANOWAR_BATTLE_SAILS
from .sails import Sails
from .ship import Ship, wrap_deg


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

    # Create current ship (sailboat by default)
    current_ship = Ship(Sails(SAILBOAT))
    
    # Store ship configurations for switching
    ship_configs = {
        "sailboat": Sails(SAILBOAT),
        "manowar_full": Sails(MANOWAR_FULL_PRESS),
        "manowar_battle": Sails(MANOWAR_BATTLE_SAILS)
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
        # Rudder — momentary deflection: A/D (or ←/→) deflect while held, snap to 0 when released
        left_held = keys[pygame.K_a] or keys[pygame.K_LEFT]
        right_held = keys[pygame.K_d] or keys[pygame.K_RIGHT]
        if left_held and not right_held:
            current_ship.delta_rudder_deg = max(-current_ship.p.max_rudder_deg, current_ship.delta_rudder_deg - 60.0 * dt)
        elif right_held and not left_held:
            current_ship.delta_rudder_deg = min(current_ship.p.max_rudder_deg, current_ship.delta_rudder_deg + 60.0 * dt)
        else:
            current_ship.delta_rudder_deg = 0.0

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
