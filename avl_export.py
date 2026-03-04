# avl_export.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple, Dict
from pathlib import Path
import math
from collections import defaultdict, deque


@dataclass
class AirfoilRef:
    kind: str = "NACA"   # "NACA" | "FILE"
    name: str = "2412"
    filepath: str = ""   # .dat

    def to_avl_lines(self) -> List[str]:
        k = self.kind.upper()
        if k == "NACA":
            return ["NACA", self.name]
        return ["AFIL", self.filepath]


@dataclass
class ControlSurface:
    name: str = "aileron"
    eta_start: float = 0.0
    eta_end: float = 1.0
    cf_over_c: float = 0.25
    gain: float = 1.0
    hinge_axis: Tuple[float, float, float] = (0.0, 1.0, 0.0)
    sgn_dup: int = +1

    def xhinge(self) -> float:
        # AVL uses Xhinge measured from LE in fraction of chord. If cf/c is flap chord fraction from TE:
        # Xhinge = 1 - cf/c
        return max(0.0, min(1.0, 1.0 - self.cf_over_c))

    def active_at_eta(self, eta: float) -> bool:
        a = min(self.eta_start, self.eta_end)
        b = max(self.eta_start, self.eta_end)
        return (a - 1e-12) <= eta <= (b + 1e-12)


@dataclass
class SectionAirfoilAssignment:
    eta: float
    airfoil: AirfoilRef


@dataclass
class Bay:
    x_le_root: float = 0.0
    y_le_root: float = 0.0
    z_le_root: float = 0.0

    c_root: float = 1.0
    tip_chord_mode: str = "CTIP"  # CTIP|TAPER
    c_tip: float = 1.0
    taper: float = 1.0

    span: float = 1.0
    dihedral_deg: float = 0.0
    span_def: str = "DY"  # DY|L3D
    span_l3d: float = 0.0
    surface_kind: str = "wing"  # wing|fin|winglet|bulk|fuselage_top|fuselage_lat

    sweep_mode: str = "LE"  # LE|C4|TE
    sweep_deg: float = 0.0

    twist_root_deg: float = 0.0
    twist_tip_deg: float = 0.0
    rigid_inc_deg: float = 0.0

    nchord: int = 8
    cspace: float = 1.0
    nspan: int = 20
    sspace: float = 1.0

    airfoil_root: AirfoilRef = field(default_factory=lambda: AirfoilRef("NACA", "2412", ""))
    airfoil_tip: AirfoilRef = field(default_factory=lambda: AirfoilRef("NACA", "2412", ""))
    overrides: List[SectionAirfoilAssignment] = field(default_factory=list)

    controls: List[ControlSurface] = field(default_factory=list)

    def c_tip_effective(self) -> float:
        if self.tip_chord_mode.upper() == "TAPER":
            return max(1e-9, self.taper) * self.c_root
        return self.c_tip

    def dy_dz(self) -> Tuple[float, float]:
        gamma = math.radians(self.dihedral_deg)
        if self.span_def.upper() == "L3D":
            l3d = self.span_l3d if self.span_l3d > 0.0 else self.span
            dy = l3d * math.cos(gamma)
            dz = l3d * math.sin(gamma)
            return dy, dz
        dy = self.span
        dz = dy * math.tan(gamma)
        return dy, dz

    def dihedral_from_yz_deg(self) -> float:
        """Dihedral angle gamma from YZ components: gamma = atan2(DZ, DY)."""
        dy, dz = self.dy_dz()
        return math.degrees(math.atan2(dz, dy))

    def dx_le(self) -> float:
        dy, _ = self.dy_dz()
        c_tip = self.c_tip_effective()
        dx_ref = dy * math.tan(math.radians(self.sweep_deg))
        mode = self.sweep_mode.upper()
        if mode == "LE":
            return dx_ref
        if mode == "C4":
            return dx_ref - 0.25 * (c_tip - self.c_root)
        if mode == "TE":
            return dx_ref - (c_tip - self.c_root)
        return dx_ref

    def _dx_te(self) -> float:
        return self.dx_le() + (self.c_tip_effective() - self.c_root)

    def sweep_from_xy_deg(self) -> float:
        """Sweep angle lambda from XY components: lambda = atan2(DX, DY)."""
        dy, _ = self.dy_dz()
        return math.degrees(math.atan2(self.dx_le(), dy))

    def length_3d(self) -> float:
        """True 3D bay length based on LE displacement vector."""
        if self.span_def.upper() == "L3D" and self.span_l3d > 0.0:
            return self.span_l3d
        dx = self.dx_le()
        dy, dz = self.dy_dz()
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def tip_le(self) -> Tuple[float, float, float]:
        dx = self.dx_le()
        dy, dz = self.dy_dz()
        return (self.x_le_root + dx, self.y_le_root + dy, self.z_le_root + dz)

    def sweep_le_deg(self) -> float:
        dy, _ = self.dy_dz()
        if abs(dy) < 1e-12:
            return 90.0 if self.dx_le() >= 0 else -90.0
        return math.degrees(math.atan2(self.dx_le(), dy))

    def sweep_te_deg(self) -> float:
        dy, _ = self.dy_dz()
        dx_te = self._dx_te()
        if abs(dy) < 1e-12:
            return 90.0 if dx_te >= 0 else -90.0
        return math.degrees(math.atan2(dx_te, dy))

    def airfoil_at_eta(self, eta: float) -> AirfoilRef:
        for ov in self.overrides:
            if abs(ov.eta - eta) < 1e-6:
                return ov.airfoil
        if eta >= 0.999999:
            return self.airfoil_tip
        return self.airfoil_root


class ConnectionType:
    MATCH_CROOT_CTIP = "MATCH_CROOT_CTIP"  # slave.c_root := master.c_tip_effective()
    MIN_EDGE_ATTACH = "Minimal Edge Attach"
    LOCK_LE = "Lock Leading Edge"
    MATCH_TWIST = "Match Twist Angle"
    MATCH_SWEEP = "Match Sweep Angle"
    MATCH_SWEEP_LE = "Match Sweep at LE"
    MATCH_SWEEP_TE = "Match Sweep at TE"
    MATCH_DIHEDRAL = "Match Dihedral Angle"


@dataclass
class BayConnection:
    master_index: int
    slave_index: int
    ctype: str = ConnectionType.MIN_EDGE_ATTACH


def _apply_single_constraint(master: Bay, slave: Bay, ctype: str) -> None:
    xt, yt, zt = master.tip_le()

    if ctype == ConnectionType.MIN_EDGE_ATTACH:
        # minimal: only force Y,Z coincidence at tip/root
        slave.y_le_root += (yt - slave.y_le_root)
        slave.z_le_root += (zt - slave.z_le_root)
        return

    if ctype == ConnectionType.LOCK_LE:
        slave.x_le_root = xt
        slave.y_le_root = yt
        slave.z_le_root = zt
        return

    if ctype == ConnectionType.MATCH_TWIST:
        slave.twist_root_deg = master.twist_tip_deg
        return

    if ctype == ConnectionType.MATCH_SWEEP:
        slave.sweep_mode = master.sweep_mode
        slave.sweep_deg = master.sweep_deg
        return

    if ctype == ConnectionType.MATCH_SWEEP_LE:
        slave.sweep_mode = "LE"
        slave.sweep_deg = master.sweep_le_deg()
        return

    if ctype == ConnectionType.MATCH_SWEEP_TE:
        slave.sweep_mode = "TE"
        slave.sweep_deg = master.sweep_te_deg()
        return

    if ctype == ConnectionType.MATCH_CROOT_CTIP:
        # slave root chord equals master's effective tip chord
        slave.c_root = master.c_tip_effective()
        return

    if ctype == ConnectionType.MATCH_DIHEDRAL:
        slave.dihedral_deg = master.dihedral_deg
        return


def apply_connections(bays: List[Bay], connections: List[BayConnection], max_iters: int = 10) -> None:
    if not connections or not bays:
        return

    # Toposort-ish; fallback to iter if cycles
    adj: Dict[int, List[int]] = defaultdict(list)
    indeg: Dict[int, int] = defaultdict(int)
    edge_types: Dict[Tuple[int, int], List[str]] = defaultdict(list)

    for c in connections:
        if c.master_index == c.slave_index:
            continue
        if not (0 <= c.master_index < len(bays) and 0 <= c.slave_index < len(bays)):
            continue
        adj[c.master_index].append(c.slave_index)
        indeg[c.slave_index] += 1
        edge_types[(c.master_index, c.slave_index)].append(c.ctype)

    q = deque([i for i in range(len(bays)) if indeg.get(i, 0) == 0])
    order: List[int] = []
    indeg2 = dict(indeg)

    while q:
        n = q.popleft()
        order.append(n)
        for m in adj.get(n, []):
            indeg2[m] = indeg2.get(m, 0) - 1
            if indeg2[m] == 0:
                q.append(m)

    acyclic = (len(order) == len({i for i in range(len(bays))}))

    def apply_all_once():
        for c in connections:
            if c.master_index == c.slave_index:
                continue
            if not (0 <= c.master_index < len(bays) and 0 <= c.slave_index < len(bays)):
                continue
            _apply_single_constraint(bays[c.master_index], bays[c.slave_index], c.ctype)

    if acyclic:
        for u in order:
            for v in adj.get(u, []):
                for ct in edge_types[(u, v)]:
                    _apply_single_constraint(bays[u], bays[v], ct)
    else:
        for _ in range(max_iters):
            apply_all_once()


def bay_to_sections_avl(b: Bay) -> List[str]:
    def lerp(a, c, t): return a + (c - a) * t

    etas = {0.0, 1.0}
    for c in b.controls:
        etas.add(max(0.0, min(1.0, c.eta_start)))
        etas.add(max(0.0, min(1.0, c.eta_end)))
    for ov in b.overrides:
        etas.add(max(0.0, min(1.0, ov.eta)))
    eta_list = sorted(etas)

    dx_total = b.dx_le()
    dy_total, dz_total = b.dy_dz()
    c_root, c_tip = b.c_root, b.c_tip_effective()

    out: List[str] = []
    for i, eta in enumerate(eta_list):
        x = b.x_le_root + eta * dx_total
        y = b.y_le_root + eta * dy_total
        z = b.z_le_root + eta * dz_total
        chord = lerp(c_root, c_tip, eta)
        twist = lerp(b.twist_root_deg, b.twist_tip_deg, eta)
        ainc = b.rigid_inc_deg + twist

        nspan = b.nspan if i == 0 else 0

        out += [
            "SECTION",
            f"{x:.6g} {y:.6g} {z:.6g} {chord:.6g} {ainc:.6g} {nspan:d} {b.sspace:.6g}"
        ]
        out += b.airfoil_at_eta(eta).to_avl_lines()

        for ctrl in b.controls:
            if ctrl.active_at_eta(eta):
                vx, vy, vz = ctrl.hinge_axis
                out.append(
                    f"CONTROL {ctrl.name} {ctrl.gain:.6g} {ctrl.xhinge():.6g} "
                    f"{vx:.6g} {vy:.6g} {vz:.6g} {ctrl.sgn_dup:+d}"
                )
    return out


def export_avl_file(
    bays: List[Bay],
    filepath: str,
    surface_name: str = "WING",
    header_name: str = "AEROSTATE",
    mach: float = 0.0,
    iYsym: int = 1,
    iZsym: int = 0,
    Zsym: float = 0.0,
    Sref: float = 1.0,
    Cref: float = 1.0,
    Bref: float = 1.0,
    Xref: float = 0.0,
    Yref: float = 0.0,
    Zref: float = 0.0,
) -> None:
    lines: List[str] = []
    lines += [header_name]
    lines += [f"{mach:.6g}"]
    lines += [f"{iYsym:d} {iZsym:d} {Zsym:.6g}"]
    lines += [f"{Sref:.6g} {Cref:.6g} {Bref:.6g}"]
    lines += [f"{Xref:.6g} {Yref:.6g} {Zref:.6g}"]
    lines += [""]

    lines += ["SURFACE", surface_name]
    if bays:
        b0 = bays[0]
        lines += [f"{b0.nchord:d} {b0.cspace:.6g} {b0.nspan:d} {b0.sspace:.6g}"]
    else:
        lines += ["8 1.0 20 1.0"]
    lines += [""]

    for b in bays:
        lines += bay_to_sections_avl(b)
        lines += [""]

    Path(filepath).write_text("\n".join(lines), encoding="utf-8")
