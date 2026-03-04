from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from avl_export import AirfoilRef, BayConnection, ControlSurface


@dataclass
class VarMeta:
    optimize: bool = False


@dataclass
class InertialData:
    mass: Optional[float] = None
    Ixx: Optional[float] = None
    Iyy: Optional[float] = None
    Izz: Optional[float] = None
    Ixy: Optional[float] = None
    Ixz: Optional[float] = None
    Iyz: Optional[float] = None


@dataclass
class AeroPlaceholder:
    load_cases: Dict[str, Dict[str, Any]] = field(default_factory=dict)


@dataclass
class SectionModel:
    name: str = "Section"
    eta: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    theta_deg: float = 0.0
    airfoil: AirfoilRef = field(default_factory=lambda: AirfoilRef("NACA", "2412", ""))

    inertial: InertialData = field(default_factory=InertialData)
    aero: AeroPlaceholder = field(default_factory=AeroPlaceholder)
    var_opt: Dict[str, VarMeta] = field(default_factory=lambda: {
        "x": VarMeta(False), "y": VarMeta(False), "z": VarMeta(False), "theta_deg": VarMeta(False)
    })


@dataclass
class BayModel:
    name: str = "Bay"
    x_le_root: float = 0.0
    y_le_root: float = 0.0
    z_le_root: float = 0.0
    c_root: float = 1.0
    tip_chord_mode: str = "CTIP"
    c_tip: float = 1.0
    taper: float = 1.0
    span: float = 1.0
    dihedral_deg: float = 0.0
    span_def: str = "DY"  # DY|L3D
    length_3d: Optional[float] = None
    surface_kind: str = "wing"  # wing|fin|winglet|bulk|fuselage_top|fuselage_lat
    sweep_mode: str = "LE"
    sweep_deg: float = 0.0
    twist_root_deg: float = 0.0
    twist_tip_deg: float = 0.0
    rigid_inc_deg: float = 0.0
    nchord: int = 8
    cspace: float = 1.0
    nspan: int = 20
    sspace: float = 1.0

    controls: List[ControlSurface] = field(default_factory=list)
    use_wing_density: bool = True
    density_kg_m2: float = 1.0
    concentrated_masses: List["ConcentratedMass"] = field(default_factory=list)
    inertial: InertialData = field(default_factory=InertialData)
    aero: AeroPlaceholder = field(default_factory=AeroPlaceholder)
    sections: List[SectionModel] = field(default_factory=list)

    var_opt: Dict[str, VarMeta] = field(default_factory=lambda: {
        "x_le_root": VarMeta(False), "y_le_root": VarMeta(False), "z_le_root": VarMeta(False),
        "c_root": VarMeta(False), "c_tip": VarMeta(False), "taper": VarMeta(False),
        "span": VarMeta(False), "dihedral_deg": VarMeta(False),
        "span_def": VarMeta(False), "length_3d": VarMeta(False), "surface_kind": VarMeta(False),
        "sweep_deg": VarMeta(False), "twist_root_deg": VarMeta(False), "twist_tip_deg": VarMeta(False),
        "rigid_inc_deg": VarMeta(False), "nchord": VarMeta(False), "nspan": VarMeta(False),
    })


@dataclass
class WingModel:
    name: str = "Wing"
    surface_kind: str = "wing"  # wing|winglet|bulk|fin|fuselage_top|fuselage_lat
    winglet_attach_to_wing: Optional[int] = None
    bulk_attach_to_wings: List[int] = field(default_factory=list)
    density_kg_m2: float = 1.0
    bays: List[BayModel] = field(default_factory=list)
    inertial: InertialData = field(default_factory=InertialData)
    aero: AeroPlaceholder = field(default_factory=AeroPlaceholder)
    var_opt: Dict[str, VarMeta] = field(default_factory=lambda: {"name": VarMeta(False)})


@dataclass
class Project:
    wings: List[WingModel] = field(default_factory=list)
    connections: List[BayConnection] = field(default_factory=list)
    filepath: Optional[str] = None


@dataclass
class ConcentratedMass:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    mass: float = 0.0
