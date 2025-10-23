import math
try:
    import pandas as pd  # optional; only needed for sweep/optimize helpers
except Exception:  # pragma: no cover - optional dependency
    pd = None
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

# Man o' War — Full Press (all plain sail, fair wind, reaching/running bias)
# SailParams(
#     rho=1.225,            # sea-level standard; 1.20–1.25 covers warm/cool marine air
#     A=3800.0,             # m^2 effective projected area (≈ 0.7–0.8 of ~5000–5500 m^2 total canvas)
#     AR=1.6,               # low aspect ratio for stacked square sails
#     e=0.65,               # Oswald efficiency knocked down by mast/yard losses & interference
#
#     CD0=0.055,            # hull + rigging windage baked into the polar
#     alpha0_deg=+0.5,      # square canvas doesn’t make lift at tiny α like a foil
#
#     alpha_stall_deg=12.0, # stalls early when pinching
#     alpha_max_deg=65.0,   # but stays “parachute-useful” far past stall
#     min_poststall_CL_frac=0.20,
#     CD_surge_max=1.35     # strong drag rise when squared off — downwind is drag-driven
# )

# Man o' War — Battle Sails (reefed/shortened sail, closer to weather, less showy)
# SailParams(
#     rho=1.225,
#     A=2200.0,             # reduced effective area for topsails/fore-and-main only
#     AR=1.4,               # slightly “stubbier” with less aloft
#     e=0.58,               # more interference and leakage when canvas is cut down
#
#     CD0=0.060,            # proportionally more parasitic drag (rigging dominates)
#     alpha0_deg=+0.5,
#
#     alpha_stall_deg=10.0, # earlier stall reflects poor upwind bite
#     alpha_max_deg=60.0,
#     min_poststall_CL_frac=0.18,
#     CD_surge_max=1.25
# )

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

        # Wind flows FROM gamma_deg, so the relative flow is TO (-cos g, -sin g)
        # Drag acts along the flow direction: (-cos g, -sin g)
        # Lift is +90° from the flow: (sin g, -cos g)
        # The sign of CL (hence L) already encodes the side (from α), so don't flip again.
        T = L * math.sin(g) - D * math.cos(g)
        S = -L * math.cos(g) - D * math.sin(g)

        return {"T": T, "S": S, "L": L, "D": D, "alpha_deg": alpha_deg, "CL": CL, "CD": CD, "q": q}

    def sweep_trim(self, Va: float, gamma_deg: float, delta_min: float = -90.0,
                   delta_max: float = 90.0, step: float = 0.5):
        if pd is None:
            raise ImportError("pandas is required for sweep_trim; install pandas or skip this helper")
        rows = []
        delta = delta_min
        while delta <= delta_max + 1e-12:
            res = self.forces(Va, gamma_deg, delta)
            res.update({"delta_deg": delta, "Va": Va, "gamma_deg": gamma_deg})
            rows.append(res)
            delta += step
        return pd.DataFrame(rows)

    def optimize_trim_for_drive(self, Va: float, gamma_deg: float,
                                delta_min: float = -90.0, delta_max: float = 90.0,
                                step: float = 0.25) -> Dict[str, float]:
        if pd is None:
            raise ImportError("pandas is required for optimize_trim_for_drive; install pandas or skip this helper")
        df = self.sweep_trim(Va, gamma_deg, delta_min, delta_max, step)
        return df.loc[df["T"].idxmax()].to_dict()

    def forces_from_true_wind(self, VT: float, beta_deg: float, U_boat: float, delta_deg: float) -> Dict[str, float]:
        """
        Convenience wrapper: pass true wind (speed, angle from bow) + boat speed.
        """
        Va, gamma = self.apparent_wind(VT, beta_deg, U_boat)
        out = self.forces(Va, gamma, delta_deg)
        out.update({"Va": Va, "gamma_deg": gamma, "VT": VT, "beta_deg": beta_deg, "U_boat": U_boat})
        return out

if __name__ == "__main__":
    calc = SailForceCalculator()

    # Example: true wind 10 m/s from 45° off the bow, boat at 5 m/s forward
    Va, gamma = calc.apparent_wind(VT=10.0, beta_deg=45.0, U_boat=5.0)
    result = calc.forces(Va=Va, gamma_deg=gamma, delta_deg=20.0)
    print(f"Apparent wind: Va={Va:.2f} m/s, gamma={gamma:.1f}°")
    print("Forces:", {k: (f"{v:.1f}" if isinstance(v, float) else v) for k, v in result.items()})

    optimal = calc.optimize_trim_for_drive(Va=Va, gamma_deg=gamma)
    print(f"Optimal trim: {optimal['delta_deg']:.1f}°, T_max={optimal['T']:.1f} N")
