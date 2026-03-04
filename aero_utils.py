# aero_utils.py
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Tuple, Dict

def isa_atmosphere(h_m: float) -> tuple[float, float, float, float]:
    """
    ISA (semplificata):
    - Troposfera fino 11 km (gradiente -6.5 K/km)
    - Strato isoterma fino ~20 km
    Ritorna: (T [K], p [Pa], rho [kg/m^3], a [m/s])
    """
    T0 = 288.15
    p0 = 101325.0
    g  = 9.80665
    R  = 287.05287
    L  = -0.0065

    if h_m < 11000.0:
        T = T0 + L * h_m
        p = p0 * (T / T0) ** (-g / (L * R))
    else:
        T11 = T0 + L * 11000.0
        p11 = p0 * (T11 / T0) ** (-g / (L * R))
        T = T11
        p = p11 * math.exp(-g * (h_m - 11000.0) / (R * T))

    rho = p / (R * T)
    gamma = 1.4
    a = math.sqrt(gamma * R * T)
    return T, p, rho, a

def sutherland_mu(T: float) -> float:
    """Viscosità dinamica aria via legge di Sutherland."""
    mu0 = 1.716e-5
    T0  = 273.15
    S   = 110.4
    return mu0 * (T / T0) ** 1.5 * (T0 + S) / (T + S)

def reynolds_mach(h_m: float, V_ms: float, chord_m: float) -> tuple[float, float, Dict[str, float]]:
    """
    Calcola Reynolds e Mach per aria ISA:
    Re = rho V c / mu
    M  = V / a
    """
    T, p, rho, a = isa_atmosphere(h_m)
    mu = sutherland_mu(T)
    Re = rho * V_ms * chord_m / max(mu, 1e-12)
    M  = V_ms / max(a, 1e-9)
    return Re, M, {"T": T, "p": p, "rho": rho, "a": a, "mu": mu}

# -----------------------------
# Polar data + surrogate
# -----------------------------

@dataclass
class PolarPoint:
    alpha: float
    cl: float
    cd: float
    cm: float = 0.0

@dataclass
class Polar:
    source: str               # "XFOIL" | "EXPERIMENTAL"
    airfoil_id: str           # e.g. "NACA2412" or filename stem
    re: float
    mach: float
    points: List[PolarPoint]  # sweep alpha

    def cl_cd_pairs(self) -> List[Tuple[float, float]]:
        return [(p.cl, p.cd) for p in self.points]

@dataclass
class PolarSurrogate:
    """
    Tabella Cd(Cl, Re, M) su griglie:
      cd_table[i_cl][j_re][k_m]
    con interpolazione trilineare.
    """
    cl_grid: List[float]
    re_grid: List[float]
    m_grid: List[float]
    cd_table: List[List[List[float]]]

    def eval(self, cl: float, re: float, mach: float) -> float:
        def clamp(v, a, b): return max(a, min(b, v))

        def locate(grid: List[float], x: float) -> Tuple[int, float]:
            if len(grid) == 1:
                return 0, 0.0
            if x <= grid[0]:
                return 0, 0.0
            if x >= grid[-1]:
                return len(grid) - 2, 1.0
            for i in range(len(grid) - 1):
                if grid[i] <= x <= grid[i + 1]:
                    t = (x - grid[i]) / (grid[i + 1] - grid[i])
                    return i, t
            return len(grid) - 2, 1.0

        cl = clamp(cl, self.cl_grid[0], self.cl_grid[-1])
        re = clamp(re, self.re_grid[0], self.re_grid[-1])
        mach = clamp(mach, self.m_grid[0], self.m_grid[-1])

        ic, tc = locate(self.cl_grid, cl)
        ir, tr = locate(self.re_grid, re)
        im, tm = locate(self.m_grid, mach)

        def cd_interp(i_cl: int) -> float:
            c000 = self.cd_table[i_cl][ir][im]
            c010 = self.cd_table[i_cl][ir][im + 1]
            c100 = self.cd_table[i_cl][ir + 1][im]
            c110 = self.cd_table[i_cl][ir + 1][im + 1]
            c0 = c000 * (1 - tm) + c010 * tm
            c1 = c100 * (1 - tm) + c110 * tm
            return c0 * (1 - tr) + c1 * tr

        cd0 = cd_interp(ic)
        cd1 = cd_interp(ic + 1)
        return cd0 * (1 - tc) + cd1 * tc
