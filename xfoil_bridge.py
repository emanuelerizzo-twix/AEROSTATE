# xfoil_bridge.py
from __future__ import annotations

import subprocess
from pathlib import Path
from typing import List, Optional

from aero_utils import Polar, PolarPoint

def run_xfoil_polar(
    xfoil_exe: str,
    airfoil_dat: Path,
    re: float,
    mach: float,
    alpha_list: List[float],
    out_polar: Path,
    ncrit: Optional[float] = None,
) -> str:
    """
    Esegue XFOIL in modalità batch e salva la polare su out_polar.
    Ritorna stdout di XFOIL (utile per debug).
    """
    airfoil_dat = Path(airfoil_dat)
    out_polar = Path(out_polar)

    if not airfoil_dat.exists():
        raise FileNotFoundError(f"Airfoil file not found: {airfoil_dat}")

    cmds: List[str] = []
    cmds += [f"LOAD {airfoil_dat.name}", ""]
    cmds += ["PANE"]
    cmds += ["OPER"]
    cmds += [f"VISC {re:.0f}"]
    if mach > 0.0:
        cmds += [f"MACH {mach:.4f}"]
    if ncrit is not None:
        cmds += ["VPAR", f"N {ncrit:.3f}", "", ""]

    cmds += ["PACC", str(out_polar.name), ""]
    for a in alpha_list:
        cmds += [f"ALFA {a:.4f}"]
    cmds += ["PACC", ""]
    cmds += ["QUIT"]
    script = "\n".join(cmds) + "\n"

    p = subprocess.Popen(
        [xfoil_exe],
        cwd=str(airfoil_dat.parent),
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    out, _ = p.communicate(script)
    return out

def parse_xfoil_polar_file(polar_file: Path, airfoil_id: str, re: float, mach: float, source: str = "XFOIL") -> Polar:
    polar_file = Path(polar_file)
    if not polar_file.exists():
        raise FileNotFoundError(f"Polar file not found: {polar_file}")

    pts: List[PolarPoint] = []
    with open(polar_file, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.split()
            if len(parts) < 3:
                continue
            try:
                alpha = float(parts[0])
                cl = float(parts[1])
                cd = float(parts[2])
                cm = float(parts[4]) if len(parts) > 4 else 0.0
                pts.append(PolarPoint(alpha=alpha, cl=cl, cd=cd, cm=cm))
            except ValueError:
                continue

    return Polar(source=source, airfoil_id=airfoil_id, re=re, mach=mach, points=pts)

def import_experimental_polar_csv(path: Path, airfoil_id: str, re: float, mach: float) -> Polar:
    """
    Import polare sperimentale (CSV/TXT): alpha, cl, cd (cm opzionale).
    Separatori: virgola o spazio. Header tollerato.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")

    pts: List[PolarPoint] = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            parts = [p.strip() for p in s.split(",")] if "," in s else s.split()
            if len(parts) < 3:
                continue
            try:
                alpha = float(parts[0])
                cl = float(parts[1])
                cd = float(parts[2])
                cm = float(parts[3]) if len(parts) > 3 else 0.0
                pts.append(PolarPoint(alpha=alpha, cl=cl, cd=cd, cm=cm))
            except ValueError:
                continue

    return Polar(source="EXPERIMENTAL", airfoil_id=airfoil_id, re=re, mach=mach, points=pts)
