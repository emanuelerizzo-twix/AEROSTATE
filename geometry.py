from __future__ import annotations

from avl_export import AirfoilRef, Bay as AvlBay
from models import BayModel, SectionModel


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
        span=b.span, dihedral_deg=b.dihedral_deg, sweep_mode=b.sweep_mode, sweep_deg=b.sweep_deg,
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
        span=b.span, dihedral_deg=b.dihedral_deg, sweep_mode=b.sweep_mode, sweep_deg=b.sweep_deg,
        twist_root_deg=b.twist_root_deg, twist_tip_deg=b.twist_tip_deg, rigid_inc_deg=b.rigid_inc_deg,
        nchord=b.nchord, cspace=b.cspace, nspan=b.nspan, sspace=b.sspace,
        airfoil_root=root_af, airfoil_tip=tip_af,
    )
    avb.controls = list(b.controls)
    return avb
