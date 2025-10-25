from dataclasses import dataclass


@dataclass
class SailParams:
    # Geometry & air
    rho: float = 1.225         # kg/m^3
    A: float = 25.0            # m^2 (reduced sail area for more manageable power)
    AR: float = 6.0            # aspect ratio (higher for better efficiency)
    e: float = 0.90            # Oswald efficiency factor (higher for better performance)

    # Polar / section
    CD0: float = 0.008         # zero-lift (parasitic) drag (lower for better performance)
    alpha0_deg: float = -3.0   # zero-lift AoA (deg) (better for upwind)

    # Stall model
    alpha_stall_deg: float = 18.0  # onset of stall (deg) (later stall for better upwind)
    alpha_max_deg: float = 45.0    # fully separated (deg) (more forgiving)
    min_poststall_CL_frac: float = 0.40  # CL fraction retained far past stall (better post-stall)
    CD_surge_max: float = 0.6      # extra CD added at alpha_max (less drag surge)
    
    # Ship dynamics
    mass: float = 500.0        # kg (ship mass)
    Iz: float = 5000.0         # yaw inertia kg·m^2
    sway_damp: float = 10000.0 # N·(m/s) for lateral velocity
    surge_damp: float = 800.0 # N·(m/s) for forward velocity (reduced for more momentum persistence)
    yaw_damp: float = 50000.0  # N·m per rad/s (reduced to restore rudder authority)
    lever_S: float = 1.0       # m: side-force lever arm creating yaw
    rudder_N_per_rad: float = 300000.0   # N·m per rad rudder effectiveness (boosted for better turning)
    max_rudder_deg: float = 35.0  # Full rudder range
    ship_type: str = "sailboat"  # Ship type for visualization

# Predefined ship configurations

# Modern sailboat configuration (default)
SAILBOAT = SailParams(
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

# Man-of-war full press configuration (all plain sail)
MANOWAR_FULL_PRESS = SailParams(
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

# Man-of-war battle sails configuration (reefed/shortened sail)
MANOWAR_BATTLE_SAILS = SailParams(
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
