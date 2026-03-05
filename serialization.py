from __future__ import annotations

from dataclasses import asdict
from typing import Any, Dict

from avl_export import AirfoilRef, BayConnection, ConnectionType, ControlSurface
from models import AeroPlaceholder, BayModel, ConcentratedMass, InertialData, Project, SectionModel, VarMeta, WingModel


def _airfoil_to_dict(a: AirfoilRef) -> Dict[str, Any]:
    return {"kind": a.kind, "name": a.name, "filepath": a.filepath}


def _airfoil_from_dict(d: Dict[str, Any]) -> AirfoilRef:
    return AirfoilRef(d.get("kind", "NACA"), d.get("name", "2412"), d.get("filepath", ""))


def project_to_dict(p: Project) -> Dict[str, Any]:
    def inertial_to(x: InertialData) -> Dict[str, Any]:
        return asdict(x)

    def varopt_to(vo: Dict[str, VarMeta]) -> Dict[str, Any]:
        return {k: {"optimize": v.optimize} for k, v in vo.items()}

    def section_to(s: SectionModel) -> Dict[str, Any]:
        return {
            "name": s.name, "eta": s.eta,
            "x": s.x, "y": s.y, "z": s.z, "theta_deg": s.theta_deg,
            "airfoil": _airfoil_to_dict(s.airfoil),
            "inertial": inertial_to(s.inertial),
            "aero": {"load_cases": s.aero.load_cases},
            "var_opt": varopt_to(s.var_opt),
        }

    def control_to(c: ControlSurface) -> Dict[str, Any]:
        return {
            "name": c.name, "eta_start": c.eta_start, "eta_end": c.eta_end,
            "cf_over_c": c.cf_over_c, "gain": c.gain,
            "hinge_axis": list(c.hinge_axis), "sgn_dup": c.sgn_dup,
        }

    def bay_to(b: BayModel) -> Dict[str, Any]:
        return {
            "name": b.name,
            "x_le_root": b.x_le_root, "y_le_root": b.y_le_root, "z_le_root": b.z_le_root,
            "c_root": b.c_root,
            "tip_chord_mode": b.tip_chord_mode, "c_tip": b.c_tip, "taper": b.taper,
            "span": b.span, "dihedral_deg": b.dihedral_deg,
            "span_def": b.span_def, "length_3d": b.length_3d, "surface_kind": b.surface_kind,
            "sweep_mode": b.sweep_mode, "sweep_deg": b.sweep_deg,
            "twist_root_deg": b.twist_root_deg, "twist_tip_deg": b.twist_tip_deg,
            "rigid_inc_deg": b.rigid_inc_deg,
            "nchord": b.nchord, "cspace": b.cspace, "nspan": b.nspan, "sspace": b.sspace,
            "controls": [control_to(c) for c in b.controls],
            "use_wing_density": bool(b.use_wing_density),
            "density_kg_m2": float(b.density_kg_m2),
            "concentrated_masses": [
                {"x": float(cm.x), "y": float(cm.y), "z": float(cm.z), "mass": float(cm.mass)}
                for cm in b.concentrated_masses
            ],
            "sections": [section_to(s) for s in b.sections],
            "inertial": inertial_to(b.inertial),
            "aero": {"load_cases": b.aero.load_cases},
            "var_opt": varopt_to(b.var_opt),
        }

    def wing_to(w: WingModel) -> Dict[str, Any]:
        return {
            "name": w.name,
            "surface_kind": str(getattr(w, "surface_kind", "wing")),
            "winglet_attach_to_wing": getattr(w, "winglet_attach_to_wing", None),
            "bulk_attach_to_wings": [int(x) for x in getattr(w, "bulk_attach_to_wings", [])],
            "density_kg_m2": float(w.density_kg_m2),
            "bays": [bay_to(b) for b in w.bays],
            "inertial": inertial_to(w.inertial),
            "aero": {"load_cases": w.aero.load_cases},
            "var_opt": varopt_to(w.var_opt),
        }

    return {
        "schema": 2,
        "wings": [wing_to(w) for w in p.wings],
        "concentrated_masses": [
            {"x": float(cm.x), "y": float(cm.y), "z": float(cm.z), "mass": float(cm.mass)}
            for cm in (p.concentrated_masses or [])
        ],
        "connections": [
            {"master_index": c.master_index, "slave_index": c.slave_index, "ctype": c.ctype}
            for c in p.connections
        ],
    }


def project_from_dict(d: Dict[str, Any]) -> Project:
    def inertial_from(x: Dict[str, Any]) -> InertialData:
        keys = ["mass", "Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"]
        return InertialData(**{k: x.get(k, None) for k in keys})

    def varopt_from(x: Dict[str, Any], defaults: Dict[str, VarMeta]) -> Dict[str, VarMeta]:
        out = {k: VarMeta(v.optimize) for k, v in defaults.items()}
        for k, vv in (x or {}).items():
            if k in out:
                out[k].optimize = bool(vv.get("optimize", False))
        return out

    def control_from(c: Dict[str, Any]) -> ControlSurface:
        return ControlSurface(
            name=c.get("name", "ctrl"),
            eta_start=float(c.get("eta_start", 0.0)),
            eta_end=float(c.get("eta_end", 1.0)),
            cf_over_c=float(c.get("cf_over_c", 0.25)),
            gain=float(c.get("gain", 1.0)),
            hinge_axis=tuple(c.get("hinge_axis", [0.0, 1.0, 0.0])),
            sgn_dup=int(c.get("sgn_dup", +1)),
        )

    def section_from(s: Dict[str, Any]) -> SectionModel:
        sm = SectionModel(
            name=s.get("name", "Section"),
            eta=float(s.get("eta", 0.0)),
            x=float(s.get("x", 0.0)),
            y=float(s.get("y", 0.0)),
            z=float(s.get("z", 0.0)),
            theta_deg=float(s.get("theta_deg", 0.0)),
            airfoil=_airfoil_from_dict(s.get("airfoil", {})),
        )
        sm.inertial = inertial_from(s.get("inertial", {}))
        sm.aero = AeroPlaceholder(load_cases=dict(s.get("aero", {}).get("load_cases", {})))
        sm.var_opt = varopt_from(s.get("var_opt", {}), sm.var_opt)
        return sm

    def bay_from(b: Dict[str, Any]) -> BayModel:
        bm = BayModel(
            name=b.get("name", "Bay"),
            x_le_root=float(b.get("x_le_root", 0.0)),
            y_le_root=float(b.get("y_le_root", 0.0)),
            z_le_root=float(b.get("z_le_root", 0.0)),
            c_root=float(b.get("c_root", 1.0)),
            tip_chord_mode=b.get("tip_chord_mode", "CTIP"),
            c_tip=float(b.get("c_tip", 1.0)),
            taper=float(b.get("taper", 1.0)),
            span=float(b.get("span", 1.0)),
            dihedral_deg=float(b.get("dihedral_deg", 0.0)),
            span_def=str(b.get("span_def", "DY")),
            length_3d=(None if b.get("length_3d", None) is None else float(b.get("length_3d"))),
            surface_kind=str(b.get("surface_kind", "wing")),
            sweep_mode=b.get("sweep_mode", "LE"),
            sweep_deg=float(b.get("sweep_deg", 0.0)),
            twist_root_deg=float(b.get("twist_root_deg", 0.0)),
            twist_tip_deg=float(b.get("twist_tip_deg", 0.0)),
            rigid_inc_deg=float(b.get("rigid_inc_deg", 0.0)),
            nchord=int(b.get("nchord", 8)),
            cspace=float(b.get("cspace", 1.0)),
            nspan=int(b.get("nspan", 20)),
            sspace=float(b.get("sspace", 1.0)),
            use_wing_density=bool(b.get("use_wing_density", True)),
            density_kg_m2=float(b.get("density_kg_m2", 1.0)),
        )
        bm.controls = [control_from(c) for c in b.get("controls", [])]
        bm.concentrated_masses = [
            ConcentratedMass(
                x=float(cm.get("x", 0.0)),
                y=float(cm.get("y", 0.0)),
                z=float(cm.get("z", 0.0)),
                mass=float(cm.get("mass", 0.0)),
            )
            for cm in b.get("concentrated_masses", [])
        ]
        bm.sections = [section_from(s) for s in b.get("sections", [])]
        bm.inertial = inertial_from(b.get("inertial", {}))
        bm.aero = AeroPlaceholder(load_cases=dict(b.get("aero", {}).get("load_cases", {})))
        bm.var_opt = varopt_from(b.get("var_opt", {}), bm.var_opt)
        return bm

    def wing_from(w: Dict[str, Any]) -> WingModel:
        bays = [bay_from(b) for b in w.get("bays", [])]
        default_kind = str(bays[0].surface_kind if bays else "wing")
        wm = WingModel(
            name=w.get("name", "Wing"),
            surface_kind=str(w.get("surface_kind", default_kind)),
            winglet_attach_to_wing=(None if w.get("winglet_attach_to_wing", None) is None else int(w.get("winglet_attach_to_wing"))),
            bulk_attach_to_wings=[int(x) for x in (w.get("bulk_attach_to_wings", []) or [])],
            density_kg_m2=float(w.get("density_kg_m2", 1.0)),
        )
        wm.bays = bays
        wm.inertial = inertial_from(w.get("inertial", {}))
        wm.aero = AeroPlaceholder(load_cases=dict(w.get("aero", {}).get("load_cases", {})))
        wm.var_opt = varopt_from(w.get("var_opt", {}), wm.var_opt)
        return wm

    p = Project()
    p.wings = [wing_from(w) for w in d.get("wings", [])]
    p.concentrated_masses = [
        ConcentratedMass(
            x=float(cm.get("x", 0.0)),
            y=float(cm.get("y", 0.0)),
            z=float(cm.get("z", 0.0)),
            mass=float(cm.get("mass", 0.0)),
        )
        for cm in d.get("concentrated_masses", []) or []
    ]
    p.connections = []
    for c in d.get("connections", []) or []:
        try:
            p.connections.append(BayConnection(int(c.get("master_index", 0)), int(c.get("slave_index", 0)), str(c.get("ctype", ConnectionType.MIN_EDGE_ATTACH))))
        except Exception:
            pass
    return p
