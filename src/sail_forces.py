
from dataclasses import dataclass
import math
from typing import Dict
import pandas as pd

@dataclass
class SailParams:
    rho: float = 1.225
    A: float = 20.0
    CD0: float = 0.015
    k: float = 0.075
    alpha0_deg: float = -2.0

class SailForceCalculator:
    def __init__(self, params: SailParams = SailParams()):
        self.p = params

    @staticmethod
    def deg2rad(deg: float) -> float:
        return math.radians(deg)

    def cl(self, alpha_deg: float) -> float:
        return 2 * math.pi * self.deg2rad(alpha_deg)

    def cd(self, cl: float) -> float:
        return self.p.CD0 + self.p.k * cl * cl

    def forces(self, Va: float, gamma_deg: float, delta_deg: float) -> Dict[str, float]:
        alpha_deg = gamma_deg - delta_deg - self.p.alpha0_deg
        CL = self.cl(alpha_deg)
        CD = self.cd(CL)
        q = 0.5 * self.p.rho * Va * Va
        L = q * self.p.A * CL
        D = q * self.p.A * CD
        gamma_rad = self.deg2rad(gamma_deg)
        T = L * math.sin(gamma_rad) - D * math.cos(gamma_rad)
        S = L * math.cos(gamma_rad) + D * math.sin(gamma_rad)
        return {"T": T, "S": S, "L": L, "D": D, "alpha_deg": alpha_deg, "CL": CL, "CD": CD, "q": q}

    def sweep_trim(self, Va: float, gamma_deg: float, delta_min: float = -90.0, delta_max: float = 90.0, step: float = 0.5) -> pd.DataFrame:
        rows = []
        delta = delta_min
        while delta <= delta_max + 1e-12:
            res = self.forces(Va, gamma_deg, delta)
            res.update({"delta_deg": delta, "Va": Va, "gamma_deg": gamma_deg})
            rows.append(res)
            delta += step
        return pd.DataFrame(rows)

    def optimize_trim_for_drive(self, Va: float, gamma_deg: float, delta_min: float = -90.0, delta_max: float = 90.0, step: float = 0.1):
        df = self.sweep_trim(Va, gamma_deg, delta_min, delta_max, step)
        return df.loc[df["T"].idxmax()].to_dict()
