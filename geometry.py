from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Tuple

from avl_export import AirfoilRef, Bay as AvlBay
from models import BayModel, SectionModel


@dataclass
class GeometrySummary:
    area: float
    cma: float
    cmg: float
    mass: float
    cg: Tuple[float, float, float]


def update_default_sections_from_bay(b: BayModel) -> None:
    if not b.sections:
        b.sections = [
            SectionModel(name="Root", eta=0.0, airfoil=AirfoilRef("NACA", "2412", "")),
            SectionModel(name="Tip", eta=1.0, airfoil=AirfoilRef("NACA", "2412", "")),
        ]
    if len(b.sections) == 1:
        b.sections.append(SectionModel(name="Tip", eta=1.0, airfoil=b.sections[0].airfoil))
    b.sections.sort(key=lambda s: s.eta)

    tmp = AvlBay(
        x_le_root=b.x_le_root, y_le_root=b.y_le_root, z_le_root=b.z_le_root,
        c_root=b.c_root, tip_chord_mode=b.tip_chord_mode, c_tip=b.c_tip, taper=b.taper,
        span=b.span, dihedral_deg=b.dihedral_deg, span_def=b.span_def, span_l3d=(b.length_3d or 0.0), surface_kind=b.surface_kind, sweep_mode=b.sweep_mode, sweep_deg=b.sweep_deg,
        twist_root_deg=b.twist_root_deg, twist_tip_deg=b.twist_tip_deg, rigid_inc_deg=b.rigid_inc_deg,
        nchord=b.nchord, cspace=b.cspace, nspan=b.nspan, sspace=b.sspace,
    )
    dx = tmp.dx_le()
    dy, dz = tmp.dy_dz()

    def lerp(a, c, t):
        return a + (c - a) * t

    for s in b.sections:
        eta = max(0.0, min(1.0, float(s.eta)))
        s.x = b.x_le_root + eta * dx
        s.y = b.y_le_root + eta * dy
        s.z = b.z_le_root + eta * dz
        local_tw = lerp(b.twist_root_deg, b.twist_tip_deg, eta)
        s.theta_deg = b.rigid_inc_deg + local_tw


def baymodel_to_avlbay(b: BayModel) -> AvlBay:
    update_default_sections_from_bay(b)
    root_af = b.sections[0].airfoil
    tip_af = b.sections[-1].airfoil
    avb = AvlBay(
        x_le_root=b.x_le_root, y_le_root=b.y_le_root, z_le_root=b.z_le_root,
        c_root=b.c_root, tip_chord_mode=b.tip_chord_mode, c_tip=b.c_tip, taper=b.taper,
        span=b.span, dihedral_deg=b.dihedral_deg, span_def=b.span_def, span_l3d=(b.length_3d or 0.0), surface_kind=b.surface_kind, sweep_mode=b.sweep_mode, sweep_deg=b.sweep_deg,
        twist_root_deg=b.twist_root_deg, twist_tip_deg=b.twist_tip_deg, rigid_inc_deg=b.rigid_inc_deg,
        nchord=b.nchord, cspace=b.cspace, nspan=b.nspan, sspace=b.sspace,
        airfoil_root=root_af, airfoil_tip=tip_af,
    )
    avb.controls = list(b.controls)
    return avb


def compute_bay_geometry_summary(b: BayModel, density_kg_m2: float) -> GeometrySummary:
    avb = baymodel_to_avlbay(b)
    c_root = float(avb.c_root)
    c_tip = float(avb.c_tip_effective())
    dy, dz = avb.dy_dz()
    dy_abs = abs(dy)

    area = dy_abs * (c_root + c_tip) * 0.5
    cmg = (area / dy_abs) if dy_abs > 1e-12 else 0.0

    lam = (c_tip / c_root) if abs(c_root) > 1e-12 else 0.0
    if (1.0 + lam) > 1e-12:
        cma = (2.0 / 3.0) * c_root * (1.0 + lam + lam * lam) / (1.0 + lam)
        eta_bar = (1.0 + 2.0 * lam) / (3.0 * (1.0 + lam))
    else:
        cma = 0.0
        eta_bar = 0.0

    i1 = c_root * (1.0 + lam) * 0.5
    i_eta_c = c_root * (1.0 + 2.0 * lam) / 6.0
    i_c2 = (c_root * c_root) * (1.0 + lam + lam * lam) / 3.0

    x = avb.x_le_root
    if abs(i1) > 1e-12:
        x = (avb.x_le_root * i1 + avb.dx_le() * i_eta_c + 0.5 * i_c2) / i1
    y = avb.y_le_root + dy * eta_bar
    z = avb.z_le_root + dz * eta_bar

    mass = max(0.0, float(density_kg_m2)) * max(0.0, area)
    return GeometrySummary(area=max(0.0, area), cma=cma, cmg=cmg, mass=mass, cg=(x, y, z))


def combine_geometry_summaries(items: Iterable[GeometrySummary]) -> GeometrySummary:
    values = list(items)
    total_area = 0.0
    total_cma_weight = 0.0
    total_mass = 0.0
    sx = sy = sz = 0.0
    total_span = 0.0
    for it in values:
        total_area += it.area
        total_cma_weight += it.cma * it.area
        total_mass += it.mass
        sx += it.cg[0] * it.mass
        sy += it.cg[1] * it.mass
        sz += it.cg[2] * it.mass
        if it.cmg > 1e-12:
            total_span += it.area / it.cmg

    cmg = (total_area / total_span) if total_span > 1e-12 else 0.0
    cma = (total_cma_weight / total_area) if total_area > 1e-12 else 0.0
    cg = (sx / total_mass, sy / total_mass, sz / total_mass) if total_mass > 1e-12 else (0.0, 0.0, 0.0)
    return GeometrySummary(area=total_area, cma=cma, cmg=cmg, mass=total_mass, cg=cg)
