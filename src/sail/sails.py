import math
from typing import Dict, Tuple
from .sail_params import SailParams


class Sails:
    """
    Game-tuned sail force calculator with optimized parameters for interactive simulation.
    
    This implementation includes:
    - Lift gain factor (1.8x) for better upwind performance
    - Overall force scaling (0.5x) for manageable gameplay
    - Simplified calculations optimized for real-time use
    """
    
    def __init__(self, params: SailParams = SailParams()):
        self.p = params
        self.k = 1.0 / (math.pi * self.p.e * self.p.AR)  # induced-drag factor
        self.CL_alpha = 2.0 * math.pi * self.p.AR / (2.0 + self.p.AR)  # per rad (Helmbold approx)
        # Magnify only lift (not drag) to improve upwind drive without increasing drag
        self.lift_gain = 1.8
        # Global sail force gain to uniformly scale aero forces (lift and drag)
        self.force_gain = 0.5  # reduce overall sail force by half

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
        # Uniformly scale both lift and drag to reduce overall sail force
        L *= self.force_gain
        D *= self.force_gain
        g = math.radians(gamma_from_deg)
        # Use same decomposition as sail_forces: CL carries AoA sign already
        T = L * math.sin(g) - D * math.cos(g)
        S = -L * math.cos(g) - D * math.sin(g)
        return {"T": T, "S": S, "L": L, "D": D, "alpha_deg": alpha_deg, "CL": CL, "CD": CD, "q": q}
