from __future__ import annotations

from typing import List, Tuple

from avl_export import ConnectionType
from geometry import baymodel_to_avlbay, update_default_sections_from_bay
from models import BayModel, Project


def flatten_bays(project: Project) -> List[Tuple[int, int, BayModel]]:
    out = []
    for wi, w in enumerate(project.wings):
        for bi, b in enumerate(w.bays):
            out.append((wi, bi, b))
    return out


def flat_bay_names(project: Project) -> List[str]:
    names = []
    for wi, w in enumerate(project.wings):
        for bi, b in enumerate(w.bays):
            names.append(f"{w.name}/{b.name}")
    return names


def apply_constraints_to_project(project: Project) -> None:
    flats = flatten_bays(project)
    if not flats:
        return

    bay_by_flat = {k: flats[k][2] for k in range(len(flats))}
    cons = sorted(project.connections, key=lambda c: (c.slave_index, c.master_index, c.ctype))

    first_master_for_slave = {}
    for c in cons:
        if c.slave_index not in first_master_for_slave:
            first_master_for_slave[c.slave_index] = c.master_index

    for c in cons:
        if c.master_index not in bay_by_flat or c.slave_index not in bay_by_flat:
            continue
        m = bay_by_flat[c.master_index]
        s = bay_by_flat[c.slave_index]

        if c.ctype == ConnectionType.MIN_EDGE_ATTACH:
            tmpm = baymodel_to_avlbay(m)
            _, yt, zt = tmpm.tip_le()
            s.y_le_root = yt
            s.z_le_root = zt

        elif c.ctype == ConnectionType.LOCK_LE:
            tmpm = baymodel_to_avlbay(m)
            xt, yt, zt = tmpm.tip_le()
            s.x_le_root = xt
            s.y_le_root = yt
            s.z_le_root = zt

        elif c.ctype == ConnectionType.MATCH_TWIST:
            s.twist_root_deg = m.twist_tip_deg

        elif c.ctype == ConnectionType.MATCH_DIHEDRAL:
            s.dihedral_deg = m.dihedral_deg

        elif c.ctype == ConnectionType.MATCH_SWEEP:
            s.sweep_mode = m.sweep_mode
            s.sweep_deg = m.sweep_deg

        elif c.ctype == ConnectionType.MATCH_SWEEP_LE:
            tmpm = baymodel_to_avlbay(m)
            s.sweep_mode = "LE"
            s.sweep_deg = tmpm.sweep_le_deg()

        elif c.ctype == ConnectionType.MATCH_SWEEP_TE:
            tmpm = baymodel_to_avlbay(m)
            s.sweep_mode = "TE"
            s.sweep_deg = tmpm.sweep_te_deg()

        elif c.ctype == ConnectionType.MATCH_CROOT_CTIP:
            tmpm = baymodel_to_avlbay(m)
            s.c_root = tmpm.c_tip_effective()

        if s.tip_chord_mode.upper() == "CTIP":
            s.taper = (s.c_tip / s.c_root) if s.c_root != 0 else s.taper
        else:
            s.c_tip = s.taper * s.c_root

    for slave_idx, master_idx in first_master_for_slave.items():
        if master_idx in bay_by_flat and slave_idx in bay_by_flat:
            m = bay_by_flat[master_idx]
            s = bay_by_flat[slave_idx]
            update_default_sections_from_bay(m)
            update_default_sections_from_bay(s)
            s.sections[0].airfoil = m.sections[-1].airfoil

    for _, _, b in flats:
        update_default_sections_from_bay(b)
