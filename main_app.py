# main_app.py
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any

from PySide6.QtCore import Qt, QPoint, QTimer, Signal
from PySide6.QtGui import QAction
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QDockWidget,
    QTreeWidget, QTreeWidgetItem, QFileDialog, QMessageBox, QFormLayout,
    QLineEdit, QComboBox, QPushButton, QLabel, QFrame, QMenu,
    QDialog, QDialogButtonBox, QSizeGrip,
    QSpinBox, QDoubleSpinBox, QCheckBox, QGroupBox,
    QTabWidget, QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QScrollArea
)

import vtk
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

from avl_export import (
    AirfoilRef, ControlSurface, Bay as AvlBay,
    BayConnection, ConnectionType, apply_connections,
    export_avl_file
)

# -----------------------------
# Data model / serialization / geometry helpers (modularized)
# -----------------------------
from models import VarMeta, InertialData, AeroPlaceholder, SectionModel, BayModel, WingModel, Project, ConcentratedMass
from serialization import project_to_dict, project_from_dict
from geometry import (
    update_default_sections_from_bay, baymodel_to_avlbay,
    compute_bay_geometry_summary, combine_geometry_summaries,
)
from constraints import flatten_bays as flatten_project_bays, flat_bay_names as project_flat_bay_names, apply_constraints_to_project

# -----------------------------
# VTK View + overlay manipulator
# -----------------------------
class CameraOverlay(QFrame):
    """A small draggable overlay window inside the 3D view."""
    def __init__(self, parent: QWidget):
        super().__init__(parent)
        self.setFrameShape(QFrame.StyledPanel)
        self.setAutoFillBackground(True)
        self.setWindowOpacity(0.95)
        self.setStyleSheet(
            "QFrame { background: rgba(245,245,255,235); border: 1px solid rgba(40,40,60,60); border-radius: 8px; }"
            "QLabel { font-weight: 600; }"
        )

        self._dragging = False
        self._drag_start = QPoint(0, 0)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(6)

        # Title bar
        title_row = QHBoxLayout()
        self.title = QLabel("View")
        self.title.setCursor(Qt.OpenHandCursor)
        title_row.addWidget(self.title)
        title_row.addStretch(1)
        lay.addLayout(title_row)

        # Views row
        rowv = QHBoxLayout()
        self.fit = QPushButton("FIT")
        self.top = QPushButton("TOP")
        self.bottom = QPushButton("BOT")
        self.left = QPushButton("LFT")
        self.right = QPushButton("RGT")
        self.iso = QPushButton("ISO")
        for b in (self.fit, self.top, self.bottom, self.left, self.right, self.iso):
            b.setFixedHeight(34)
            b.setMinimumWidth(54)
        rowv.addWidget(self.fit); rowv.addWidget(self.top); rowv.addWidget(self.bottom); rowv.addWidget(self.left); rowv.addWidget(self.right); rowv.addWidget(self.iso)
        lay.addLayout(rowv)

        # Zoom
        rowz = QHBoxLayout()
        self.zoom_out = QPushButton("–")
        self.zoom_in = QPushButton("+")
        for b in (self.zoom_out, self.zoom_in):
            b.setMinimumSize(54, 34)
        rowz.addWidget(QLabel("Zoom"))
        rowz.addWidget(self.zoom_out); rowz.addWidget(self.zoom_in)
        lay.addLayout(rowz)

        # Rotate
        self.yaw_l = QPushButton("⟲")
        self.yaw_r = QPushButton("⟳")
        self.pitch_u = QPushButton("↑")
        self.pitch_d = QPushButton("↓")
        self.roll_l = QPushButton("↶")
        self.roll_r = QPushButton("↷")
        for b in (self.yaw_l, self.yaw_r, self.pitch_u, self.pitch_d, self.roll_l, self.roll_r):
            b.setMinimumSize(54, 34)

        rowr1 = QHBoxLayout()
        rowr1.addWidget(QLabel("Rotate"))
        rowr1.addWidget(self.yaw_l); rowr1.addWidget(self.yaw_r); rowr1.addWidget(self.pitch_u); rowr1.addWidget(self.pitch_d)
        lay.addLayout(rowr1)

        rowr2 = QHBoxLayout()
        rowr2.addWidget(QLabel("Roll"))
        rowr2.addWidget(self.roll_l); rowr2.addWidget(self.roll_r)
        lay.addLayout(rowr2)

        self.setMinimumSize(360, 210)

        # Resize grip
        grip_row = QHBoxLayout()
        grip_row.addStretch(1)
        self._grip = QSizeGrip(self)
        grip_row.addWidget(self._grip)
        lay.addLayout(grip_row)

    def mousePressEvent(self, e):
        # Drag when clicking on title label area
        pos = e.position().toPoint()
        if e.button() == Qt.LeftButton and (self.title.geometry().contains(pos) or pos.y() <= 28):
            self._dragging = True
            self._drag_start = e.globalPosition().toPoint() - self.pos()
            self.title.setCursor(Qt.ClosedHandCursor)
            e.accept()
            return
        super().mousePressEvent(e)

    def mouseMoveEvent(self, e):
        if self._dragging:
            new_pos = e.globalPosition().toPoint() - self._drag_start
            # keep within parent
            p = self.parentWidget()
            if p is not None:
                new_pos.setX(max(0, min(new_pos.x(), p.width() - self.width())))
                new_pos.setY(max(0, min(new_pos.y(), p.height() - self.height())))
            self.move(new_pos)
            e.accept()
            return
        super().mouseMoveEvent(e)

    def mouseReleaseEvent(self, e):
        if self._dragging and e.button() == Qt.LeftButton:
            self._dragging = False
            self.title.setCursor(Qt.OpenHandCursor)
            e.accept()
            return
        super().mouseReleaseEvent(e)

class VTKCadView(QWidget):
    def __init__(self, get_ordered_bays_fn):
        super().__init__()
        self.get_ordered_bays_fn = get_ordered_bays_fn
        self._camera_initialized = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.vtkWidget = QVTKRenderWindowInteractor(self)
        layout.addWidget(self.vtkWidget)

        self.renderer = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.renderer)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.renderer.SetBackground(0.86, 0.86, 0.92)

        style = vtk.vtkInteractorStyleTrackballCamera()
        style.SetDefaultRenderer(self.renderer)
        style.SetMotionFactor(0.10)
        self.iren.SetInteractorStyle(style)

        self.axes_actor = vtk.vtkAxesActor()
        self.axes_actor.SetShaftTypeToLine()
        self.axes_actor.SetAxisLabels(1)
        self.orientation = vtk.vtkOrientationMarkerWidget()
        self.orientation.SetOrientationMarker(self.axes_actor)
        self.orientation.SetInteractor(self.iren)
        self.orientation.SetViewport(0.0, 0.0, 0.18, 0.18)
        self.orientation.SetEnabled(1)
        self.orientation.InteractiveOff()

        self.bay_actors: List[vtk.vtkActor] = []

        self.grid_actor = None
        self._ensure_grid(size=120.0, step=1.0, z=0.0)
        self.rebuild_scene(reset_camera=True)
        self.iren.Initialize()

        self.overlay = CameraOverlay(self)
        self._fit_cam_state = None  # (pos, fp, vu, parallel, pscale)

        self.overlay.zoom_in.clicked.connect(self.zoom_in)
        self.overlay.zoom_out.clicked.connect(self.zoom_out)
        self.overlay.fit.clicked.connect(self.restore_fit_view)
        self.overlay.yaw_l.clicked.connect(lambda: self.rotate_yaw(-5))
        self.overlay.yaw_r.clicked.connect(lambda: self.rotate_yaw(5))
        self.overlay.pitch_u.clicked.connect(lambda: self.rotate_pitch(5))
        self.overlay.pitch_d.clicked.connect(lambda: self.rotate_pitch(-5))
        self.overlay.roll_l.clicked.connect(lambda: self.rotate_roll(-5))
        self.overlay.roll_r.clicked.connect(lambda: self.rotate_roll(5))
        self.overlay.top.clicked.connect(lambda: self.set_ortho_view("TOP"))
        self.overlay.bottom.clicked.connect(lambda: self.set_ortho_view("BOTTOM"))
        self.overlay.left.clicked.connect(lambda: self.set_ortho_view("LEFT"))
        self.overlay.right.clicked.connect(lambda: self.set_ortho_view("RIGHT"))
        self.overlay.iso.clicked.connect(self.set_iso_view)
        # initial position
        self.overlay.move(max(0, self.width() - self.overlay.width() - 16), 16)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        # Keep overlay inside the view; do not force a fixed corner position
        x = min(self.overlay.x(), max(0, self.width() - self.overlay.width()))
        y = min(self.overlay.y(), max(0, self.height() - self.overlay.height()))
        self.overlay.move(max(0, x), max(0, y))

    def _ensure_grid(self, size: float, step: float, z: float = 0.0) -> None:
        """Create or update an XY grid large enough for current scene."""
        size = float(size)
        step = float(step)
        n = int(max(2, size / max(step, 1e-6)))
        pts = vtk.vtkPoints()
        lines = vtk.vtkCellArray()
        idx = 0

        def add_seg(x0, y0, x1, y1):
            nonlocal idx
            pts.InsertNextPoint(x0, y0, z)
            pts.InsertNextPoint(x1, y1, z)
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, idx)
            line.GetPointIds().SetId(1, idx + 1)
            lines.InsertNextCell(line)
            idx += 2

        for i in range(-n, n + 1):
            add_seg(-size, i * step, size, i * step)
            add_seg(i * step, -size, i * step, size)

        poly = vtk.vtkPolyData()
        poly.SetPoints(pts)
        poly.SetLines(lines)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly)

        if self.grid_actor is None:
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(0.55, 0.55, 0.55)
            actor.GetProperty().SetOpacity(0.22)
            actor.GetProperty().SetLineWidth(1.0)
            self.renderer.AddActor(actor)
            self.grid_actor = actor
        else:
            self.grid_actor.SetMapper(mapper)

    def zoom(self, factor: float):
        cam = self.renderer.GetActiveCamera()
        if cam.GetParallelProjection():
            s = cam.GetParallelScale()
            cam.SetParallelScale(max(1e-9, s / factor))
        else:
            cam.Dolly(factor)
            self.renderer.ResetCameraClippingRange()
        self.vtkWidget.GetRenderWindow().Render()

    def zoom_in(self): self.zoom(1.12)
    def zoom_out(self): self.zoom(1.0 / 1.12)

    def rotate_yaw(self, deg: float):
        cam = self.renderer.GetActiveCamera()
        cam.Azimuth(deg)
        self.renderer.ResetCameraClippingRange()
        self.vtkWidget.GetRenderWindow().Render()

    def rotate_pitch(self, deg: float):
        cam = self.renderer.GetActiveCamera()
        cam.Elevation(deg)
        cam.OrthogonalizeViewUp()
        self.renderer.ResetCameraClippingRange()
        self.vtkWidget.GetRenderWindow().Render()

    def rotate_roll(self, deg: float):
        cam = self.renderer.GetActiveCamera()
        cam.Roll(deg)
        cam.OrthogonalizeViewUp()
        self.renderer.ResetCameraClippingRange()
        self.vtkWidget.GetRenderWindow().Render()

    def _scene_bounds(self):
        if not self.bay_actors:
            return (-1, 1, -1, 1, -1, 1)
        b = [1e18, -1e18, 1e18, -1e18, 1e18, -1e18]
        for a in self.bay_actors:
            bb = a.GetBounds()
            b[0] = min(b[0], bb[0]); b[1] = max(b[1], bb[1])
            b[2] = min(b[2], bb[2]); b[3] = max(b[3], bb[3])
            b[4] = min(b[4], bb[4]); b[5] = max(b[5], bb[5])
        return tuple(b)

    def set_ortho_view(self, which: str):
        cam = self.renderer.GetActiveCamera()
        bounds = self._scene_bounds()
        cx = 0.5 * (bounds[0] + bounds[1])
        cy = 0.5 * (bounds[2] + bounds[3])
        cz = 0.5 * (bounds[4] + bounds[5])
        diag = max(bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4], 1.0)
        dist = 2.2 * diag

        which = which.upper().strip()
        cam.SetParallelProjection(True)

        if which == "TOP":
            cam.SetFocalPoint(cx, cy, cz)
            cam.SetPosition(cx, cy, cz + dist)
            cam.SetViewUp(0, 1, 0)
        elif which == "BOTTOM":
            cam.SetFocalPoint(cx, cy, cz)
            cam.SetPosition(cx, cy, cz - dist)
            cam.SetViewUp(0, 1, 0)
        elif which == "RIGHT":
            cam.SetFocalPoint(cx, cy, cz)
            cam.SetPosition(cx, cy + dist, cz)
            cam.SetViewUp(0, 0, 1)
        elif which == "LEFT":
            cam.SetFocalPoint(cx, cy, cz)
            cam.SetPosition(cx, cy - dist, cz)
            cam.SetViewUp(0, 0, 1)
        else:
            return

        self.renderer.ResetCameraClippingRange()
        self.vtkWidget.GetRenderWindow().Render()

    def set_iso_view(self):
        cam = self.renderer.GetActiveCamera()
        bounds = self._scene_bounds()
        cx = 0.5 * (bounds[0] + bounds[1])
        cy = 0.5 * (bounds[2] + bounds[3])
        cz = 0.5 * (bounds[4] + bounds[5])
        diag = max(bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4], 1.0)
        dist = 2.2 * diag

        cam.SetParallelProjection(False)
        cam.SetFocalPoint(cx, cy, cz)
        cam.SetPosition(cx + dist, cy + dist, cz + dist)
        cam.SetViewUp(0, 0, 1)
        self.renderer.ResetCameraClippingRange()
        self.vtkWidget.GetRenderWindow().Render()


    def fit_view(self, save_state: bool = False):
        """Fit camera to show the whole scene with a safety margin (1.2x)."""
        # Determine bounds from actors
        if not self.bay_actors:
            self.renderer.ResetCamera()
            self.vtkWidget.GetRenderWindow().Render()
            return

        bounds = self._scene_bounds()
        self.renderer.ResetCamera(bounds)
        cam = self.renderer.GetActiveCamera()

        # Apply margin: show 1.2x of the needed size
        if cam.GetParallelProjection():
            cam.SetParallelScale(cam.GetParallelScale() * 1.2)
        else:
            # Dolly < 1 zooms out
            cam.Dolly(1.0 / 1.2)

        cam.OrthogonalizeViewUp()
        self.renderer.ResetCameraClippingRange()

        if save_state:
            self._fit_cam_state = (
                cam.GetPosition(), cam.GetFocalPoint(), cam.GetViewUp(),
                bool(cam.GetParallelProjection()), cam.GetParallelScale()
            )

        self.vtkWidget.GetRenderWindow().Render()

    def restore_fit_view(self):
        """Fit to current geometry (1.2x margin) and save as the new FIT state."""
        self.fit_view(save_state=True)

    def clear_scene(self):
        for a in self.bay_actors:
            self.renderer.RemoveActor(a)
        self.bay_actors = []

    def rebuild_scene(self, reset_camera: bool = False):
        self.clear_scene()
        ordered = self.get_ordered_bays_fn()
        # Update grid to cover the scene comfortably
        if ordered:
            # approximate bounds from ordered bays
            xs, ys = [], []
            for b in ordered:
                dx = b.dx_le(); dy, dz = b.dy_dz(); ctip = b.c_tip_effective()
                xs += [b.x_le_root, b.x_le_root + b.c_root, b.x_le_root + dx, b.x_le_root + dx + ctip]
                ys += [b.y_le_root, b.y_le_root + dy]
            span_xy = max(max(xs) - min(xs), max(ys) - min(ys), 10.0)
            self._ensure_grid(size=max(200.0, span_xy * 5.0), step=1.0, z=0.0)
        else:
            self._ensure_grid(size=120.0, step=1.0, z=0.0)

        colors = [(1, 0.7, 0.2), (0.45, 0.45, 0.95), (0.2, 0.7, 0.35), (0.8, 0.2, 0.2)]
        for i, b in enumerate(ordered):
            actor = self._create_bay_actor(b, colors[i % len(colors)])
            actor.GetProperty().SetEdgeVisibility(True)
            actor.GetProperty().SetEdgeColor(0.15, 0.15, 0.15)
            actor.GetProperty().SetLineWidth(1.0)
            self.renderer.AddActor(actor)
            self.bay_actors.append(actor)

        if reset_camera and not self._camera_initialized:
            # Initial fit (slightly zoomed out: 1.2x of max extent)
            self.fit_view(save_state=True)
            self._camera_initialized = True

        self.vtkWidget.GetRenderWindow().Render()

    def _create_bay_actor(self, b: AvlBay, rgb):
        dx = b.dx_le()
        dy, dz = b.dy_dz()
        ctip = b.c_tip_effective()

        pts = vtk.vtkPoints()
        polys = vtk.vtkCellArray()
        pts.InsertNextPoint(b.x_le_root, b.y_le_root, b.z_le_root)
        pts.InsertNextPoint(b.x_le_root + b.c_root, b.y_le_root, b.z_le_root)
        pts.InsertNextPoint(b.x_le_root + dx + ctip, b.y_le_root + dy, b.z_le_root + dz)
        pts.InsertNextPoint(b.x_le_root + dx, b.y_le_root + dy, b.z_le_root + dz)

        quad = vtk.vtkQuad()
        for i in range(4):
            quad.GetPointIds().SetId(i, i)
        polys.InsertNextCell(quad)

        poly = vtk.vtkPolyData()
        poly.SetPoints(pts)
        poly.SetPolys(polys)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*rgb)
        actor.GetProperty().SetOpacity(0.92)
        return actor

# -----------------------------
# Tree widget
# -----------------------------
class ProjectTree(QTreeWidget):
    selectionChangedCustom = Signal()
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setColumnCount(3)
        self.setHeaderLabels(["Variable", "Value", "Opt"])
        self.setAlternatingRowColors(True)
        self.setEditTriggers(QTreeWidget.DoubleClicked | QTreeWidget.EditKeyPressed)



# -----------------------------
# Properties panel (CAD-like)
# -----------------------------
class PropertiesPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._current_tag = None
        self._current_obj = None

        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(8)

        self.title = QLabel("Properties")
        self.title.setStyleSheet("font-weight:700; font-size: 14px;")
        lay.addWidget(self.title)

        self.form = QFormLayout()
        self.form.setLabelAlignment(Qt.AlignLeft)
        form_host = QWidget()
        form_host.setLayout(self.form)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(form_host)
        lay.addWidget(scroll)

        self._editors = []  # (widget, on_change)
        self._block = False

    def clear(self):
        while self.form.rowCount():
            self.form.removeRow(0)
        self._editors = []
        self._current_tag = None
        self._current_obj = None

    def set_title(self, text: str):
        self.title.setText(text)

    def _add_row(self, label: str, w: QWidget):
        self.form.addRow(label, w)

    def _add_dspin(self, label: str, value: float, on_commit, step=0.05):
        sp = QDoubleSpinBox()
        sp.setDecimals(6)
        sp.setRange(-1e9, 1e9)
        sp.setSingleStep(step)
        sp.setValue(float(value))
        sp.valueChanged.connect(lambda v: (None if self._block else on_commit(float(v))))
        self._add_row(label, sp)
        return sp

    def _add_spin(self, label: str, value: int, on_commit, step=1):
        sp = QSpinBox()
        sp.setRange(0, 10**9)
        sp.setSingleStep(step)
        sp.setValue(int(value))
        sp.valueChanged.connect(lambda v: (None if self._block else on_commit(int(v))))
        self._add_row(label, sp)
        return sp

    def _add_combo(self, label: str, options, current, on_commit):
        cb = QComboBox()
        cb.addItems(list(options))
        cb.setCurrentText(str(current))
        cb.currentTextChanged.connect(lambda t: (None if self._block else on_commit(t)))
        self._add_row(label, cb)
        return cb

    def _add_opt_check(self, label: str, checked: bool, on_commit):
        chk = QCheckBox("Optimize")
        chk.setChecked(bool(checked))
        chk.stateChanged.connect(lambda st: (None if self._block else on_commit(st == Qt.Checked)))
        self._add_row(label, chk)
        return chk

# -----------------------------
# Dialogs

class VarConstraintDialog(QDialog):
    """Choose master bay for a specific constraint on a given slave bay."""
    def __init__(self, parent: QWidget, bay_names: List[str], default_master: int, slave_index: int, title: str):
        super().__init__(parent)
        self.setWindowTitle(title)
        lay = QVBoxLayout(self)
        form = QFormLayout()
        lay.addLayout(form)

        self.cb_master = QComboBox()
        self.cb_master.addItems(bay_names)
        self.cb_master.setCurrentIndex(max(0, min(default_master, len(bay_names) - 1)))

        self.lbl_slave = QLabel(bay_names[slave_index] if 0 <= slave_index < len(bay_names) else f"Slave {slave_index}")
        form.addRow("Slave", self.lbl_slave)
        form.addRow("Master", self.cb_master)

        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        lay.addWidget(bb)

    def master_index(self) -> int:
        return self.cb_master.currentIndex()

# -----------------------------
class ConnectBaysDialog(QDialog):
    def __init__(self, parent: QWidget, bay_names: List[str], default_master: int, default_slave: int, active_types: List[str]):
        super().__init__(parent)
        self.setWindowTitle("Connect bays (Master → Slave)")
        lay = QVBoxLayout(self)
        form = QFormLayout()
        lay.addLayout(form)

        self.cb_master = QComboBox()
        self.cb_master.addItems(bay_names)
        self.cb_master.setCurrentIndex(max(0, min(default_master, len(bay_names) - 1)))

        self.cb_slave = QComboBox()
        self.cb_slave.addItems(bay_names)
        self.cb_slave.setCurrentIndex(max(0, min(default_slave, len(bay_names) - 1)))

        form.addRow("Master bay", self.cb_master)
        form.addRow("Slave bay", self.cb_slave)

        # Constraints (multi-select)
        self.chk_min = QPushButton("✓  MIN_EDGE_ATTACH (root LE on master tip edge)")
        self.chk_min.setCheckable(True)

        self.chk_lock = QPushButton("✓  LOCK_LE (same root LE point)")
        self.chk_lock.setCheckable(True)

        self.chk_tw = QPushButton("✓  MATCH_TWIST")
        self.chk_tw.setCheckable(True)

        self.chk_dih = QPushButton("✓  MATCH_DIHEDRAL")
        self.chk_dih.setCheckable(True)

        self.chk_chord = QPushButton("✓  MATCH_CROOT_CTIP (slave c_root = master c_tip)")
        self.chk_chord.setCheckable(True)

        # Sweep (mutually exclusive): LE / C4 / TE / none
        self.sweep_none = QPushButton("Sweep: none")
        self.sweep_none.setCheckable(True)
        self.sweep_le = QPushButton("Sweep: match LE")
        self.sweep_le.setCheckable(True)
        self.sweep_c4 = QPushButton("Sweep: match C/4")
        self.sweep_c4.setCheckable(True)
        self.sweep_te = QPushButton("Sweep: match TE")
        self.sweep_te.setCheckable(True)

        self._sweep_buttons = [self.sweep_none, self.sweep_le, self.sweep_c4, self.sweep_te]
        for b in self._sweep_buttons:
            b.clicked.connect(lambda _=False, bb=b: self._select_sweep(bb))

        # Layout
        lay.addWidget(QLabel("Constraints (multi-select):"))
        for b in (self.chk_min, self.chk_lock, self.chk_chord, self.chk_tw, self.chk_dih):
            lay.addWidget(b)

        lay.addSpacing(8)
        lay.addWidget(QLabel("Sweep reference (mutually exclusive):"))
        row = QHBoxLayout()
        for b in self._sweep_buttons:
            row.addWidget(b)
        lay.addLayout(row)

        # Initialize from active_types
        active = set(active_types or [])
        self.chk_min.setChecked(ConnectionType.MIN_EDGE_ATTACH in active)
        self.chk_lock.setChecked(ConnectionType.LOCK_LE in active)
        self.chk_tw.setChecked(ConnectionType.MATCH_TWIST in active)
        self.chk_dih.setChecked(ConnectionType.MATCH_DIHEDRAL in active)
        self.chk_chord.setChecked(ConnectionType.MATCH_CROOT_CTIP in active)

        if ConnectionType.MATCH_SWEEP_LE in active:
            self._select_sweep(self.sweep_le)
        elif ConnectionType.MATCH_SWEEP in active:
            self._select_sweep(self.sweep_c4)
        elif ConnectionType.MATCH_SWEEP_TE in active:
            self._select_sweep(self.sweep_te)
        else:
            self._select_sweep(self.sweep_none)

        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        lay.addWidget(bb)

    def _select_sweep(self, which: QPushButton):
        for b in self._sweep_buttons:
            b.setChecked(b is which)

    def result_values(self) -> Tuple[int, int, List[str]]:
        master = self.cb_master.currentIndex()
        slave = self.cb_slave.currentIndex()
        out: List[str] = []
        if self.chk_min.isChecked(): out.append(ConnectionType.MIN_EDGE_ATTACH)
        if self.chk_lock.isChecked(): out.append(ConnectionType.LOCK_LE)
        if self.chk_chord.isChecked(): out.append(ConnectionType.MATCH_CROOT_CTIP)
        if self.chk_tw.isChecked(): out.append(ConnectionType.MATCH_TWIST)
        if self.chk_dih.isChecked(): out.append(ConnectionType.MATCH_DIHEDRAL)

        if self.sweep_le.isChecked(): out.append(ConnectionType.MATCH_SWEEP_LE)
        elif self.sweep_c4.isChecked(): out.append(ConnectionType.MATCH_SWEEP)
        elif self.sweep_te.isChecked(): out.append(ConnectionType.MATCH_SWEEP_TE)
        # else none

        return master, slave, out

# -----------------------------
# Main window
# -----------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AEROSTATE - Tree Project")
        self.resize(1700, 950)

        self.project = Project()
        self._build_default_project()

        # Remember last connect dialog selections
        self._last_connect_master = 0
        self._last_connect_slave = 0

        self.view3d = VTKCadView(self._ordered_avl_bays)
        self.setCentralWidget(self.view3d)

        self.tree_dock = QDockWidget("Project", self)
        self.tree = ProjectTree()
        self.tree_dock.setWidget(self.tree)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.tree_dock)

        # Properties (CAD-like). Dock can be closed to maximize 3D.
        self.props_dock = QDockWidget("Properties", self)
        self.props = PropertiesPanel()
        self.props_dock.setWidget(self.props)
        self.props_dock.setAllowedAreas(Qt.RightDockWidgetArea | Qt.LeftDockWidgetArea)
        self.addDockWidget(Qt.RightDockWidgetArea, self.props_dock)
        self.props_dock.setMinimumWidth(320)
        self.props_dock.setMaximumWidth(520)
        self.resizeDocks([self.tree_dock, self.props_dock], [260, 420], Qt.Horizontal)

        self._build_menu()
        self.rebuild_tree()
        self.refresh_scene()
        self.on_tree_selection_changed()
        self.statusBar().showMessage("Constraints updated", 2000)

        self.tree.setContextMenuPolicy(Qt.CustomContextMenu)
        self.tree.customContextMenuRequested.connect(self.on_tree_context_menu)
        self.tree.itemSelectionChanged.connect(self.on_tree_selection_changed)
        self.tree.itemChanged.connect(self.on_tree_item_changed)

    def _build_menu(self):
        mb = self.menuBar()
        m_file = mb.addMenu("File")
        a_open = QAction("Open...", self); a_open.triggered.connect(self.file_open)
        a_save = QAction("Save", self); a_save.triggered.connect(self.file_save)
        a_saveas = QAction("Save As...", self); a_saveas.triggered.connect(self.file_save_as)
        m_file.addAction(a_open); m_file.addAction(a_save); m_file.addAction(a_saveas)
        m_file.addSeparator()
        a_export = QAction("Export AVL...", self); a_export.triggered.connect(self.export_avl)
        m_file.addAction(a_export)

        m_edit = mb.addMenu("Edit")
        a_add_w = QAction("Add Wing", self); a_add_w.triggered.connect(self.add_wing)
        a_add_b = QAction("Add Bay to selected Wing", self); a_add_b.triggered.connect(self.add_bay_to_selected_wing)
        a_connect = QAction("Connect Bays...", self); a_connect.triggered.connect(self.open_connect_dialog)
        a_rm_w = QAction("Remove Wing", self); a_rm_w.triggered.connect(self.remove_selected_wing)
        a_rm_b = QAction("Remove Bay", self); a_rm_b.triggered.connect(self.remove_selected_bay)
        m_edit.addAction(a_add_w); m_edit.addAction(a_add_b); m_edit.addAction(a_connect)
        m_edit.addSeparator()
        m_edit.addAction(a_rm_w); m_edit.addAction(a_rm_b)

    def _build_default_project(self):
        wing = WingModel(name="ala1")
        bay1 = BayModel(
            name="Bay A",
            x_le_root=0, y_le_root=0, z_le_root=0,
            c_root=1.2, c_tip=0.9, span=2.0, dihedral_deg=5, sweep_mode="C4", sweep_deg=20,
            twist_root_deg=2, twist_tip_deg=1,
            controls=[ControlSurface(name="aileron", eta_start=0.6, eta_end=1.0, cf_over_c=0.30)],
        )
        update_default_sections_from_bay(bay1)
        bay1.sections[0].airfoil = AirfoilRef("NACA", "2412", "")
        bay1.sections[-1].airfoil = AirfoilRef("NACA", "2412", "")

        bay2 = BayModel(
            name="Bay B",
            x_le_root=0.8, y_le_root=2.0, z_le_root=0.2,
            c_root=0.9, c_tip=0.7, span=2.0, dihedral_deg=5, sweep_mode="C4", sweep_deg=20,
            twist_root_deg=1, twist_tip_deg=0,
        )
        update_default_sections_from_bay(bay2)
        bay2.sections[0].airfoil = AirfoilRef("NACA", "0012", "")
        bay2.sections[-1].airfoil = AirfoilRef("NACA", "0012", "")

        wing.bays = [bay1, bay2]
        self.project.wings = [wing]

    def _ordered_avl_bays(self) -> List[AvlBay]:
        # Ensure model-level constraints are applied so exported geometry matches UI
        self._apply_constraints_to_models()

        out: List[AvlBay] = []
        for w in self.project.wings:
            for b in w.bays:
                update_default_sections_from_bay(b)
                out.append(baymodel_to_avlbay(b))
        # Apply constraints (master->slave) on the flattened bay list (export-level)
        if self.project.connections:
            apply_connections(out, self.project.connections)
        return out

    def refresh_scene(self):
        self._apply_constraints_to_models()
        self.view3d.rebuild_scene(reset_camera=False)

    def _collect_expanded_tags(self) -> set:
        tags = set()
        def walk(it: QTreeWidgetItem):
            tag = it.data(0, Qt.UserRole)
            if it.isExpanded() and tag is not None:
                tags.add(tag)
            for i in range(it.childCount()):
                walk(it.child(i))
        if self.tree.topLevelItemCount():
            walk(self.tree.topLevelItem(0))
        return tags

    def _restore_expanded_tags(self, tags: set) -> None:
        def walk(it: QTreeWidgetItem):
            tag = it.data(0, Qt.UserRole)
            if tag in tags:
                it.setExpanded(True)
            for i in range(it.childCount()):
                walk(it.child(i))
        if self.tree.topLevelItemCount():
            walk(self.tree.topLevelItem(0))

    def rebuild_tree(self):
        expanded = self._collect_expanded_tags()
        cur = self.tree.currentItem()
        cur_tag = cur.data(0, Qt.UserRole) if cur else None

        self.tree.blockSignals(True)
        self.tree.clear()

        root = QTreeWidgetItem(self.tree, ["Project", "", ""])
        root.setExpanded(True)
        root.setFlags(root.flags() & ~Qt.ItemIsEditable)

        conn_root = QTreeWidgetItem(root, ["connections", str(len(self.project.connections)), ""])
        conn_root.setFlags(conn_root.flags() & ~Qt.ItemIsEditable)
        for ci, c in enumerate(self.project.connections):
            # names may be missing if bays removed; show indices
            item = QTreeWidgetItem(conn_root, [f"{ci}", f"{c.master_index}->{c.slave_index} {c.ctype}", ""])
            item.setData(0, Qt.UserRole, ("connection", ci))
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)

        for wi, w in enumerate(self.project.wings):
            w_item = QTreeWidgetItem(root, ["wing", w.name, ""])
            w_item.setData(0, Qt.UserRole, ("wing", wi))
            w_item.setFlags(w_item.flags() | Qt.ItemIsEditable)
            w_item.setExpanded(True)

            for bi, b in enumerate(w.bays):
                # Determine flat index for this bay
                flat_idx = 0
                for _w in self.project.wings[:wi]:
                    flat_idx += len(_w.bays)
                flat_idx += bi
                active_types = [c.ctype for c in self.project.connections if c.slave_index == flat_idx]
                suffix = ""
                if active_types:
                    suffix = "  ✓ " + ",".join(active_types)
                b_item = QTreeWidgetItem(w_item, ["bay", b.name + suffix, ""])
                b_item.setData(0, Qt.UserRole, ("bay", wi, bi))
                b_item.setFlags(b_item.flags() | Qt.ItemIsEditable)

                vars_item = QTreeWidgetItem(b_item, ["vars", "", ""])
                vars_item.setFlags(vars_item.flags() & ~Qt.ItemIsEditable)
                for varname in [
                        "x_le_root","y_le_root","z_le_root",
                        "c_root",
                        "tip_chord_mode",
                        "c_tip",
                        "taper",
                        "span",
                        "dihedral_deg",
                        "sweep_mode",
                        "sweep_deg",
                        "twist_root_deg","twist_tip_deg"
                    ]:
                    it = QTreeWidgetItem(vars_item, [varname, str(getattr(b, varname)), ""])
                    it.setData(0, Qt.UserRole, ("bay_var", wi, bi, varname))
                    it.setFlags(it.flags() & ~Qt.ItemIsEditable)
                    it.setCheckState(2, Qt.Checked if b.var_opt.get(varname, VarMeta(False)).optimize else Qt.Unchecked)

                # Constraints node (active constraints for this bay when it is a SLAVE)
                constraints_item = QTreeWidgetItem(b_item, ["constraints", "", ""])
                constraints_item.setData(0, Qt.UserRole, ("constraints", flat_idx))
                constraints_item.setFlags(constraints_item.flags() & ~Qt.ItemIsEditable)

                active_cs = [c for c in self.project.connections if c.slave_index == flat_idx]
                constraints_item.setText(1, str(len(active_cs)))

                for c in active_cs:
                    # show master -> slave and type
                    cit = QTreeWidgetItem(constraints_item, ["constraint", f"{c.master_index} → {c.slave_index}  {c.ctype}", ""])
                    cit.setData(0, Qt.UserRole, ("constraint_item", c.master_index, c.slave_index, c.ctype))
                    cit.setFlags(cit.flags() & ~Qt.ItemIsEditable)

                sec_item = QTreeWidgetItem(b_item, ["sections", str(len(b.sections)), ""])
                sec_item.setFlags(sec_item.flags() & ~Qt.ItemIsEditable)
                for si, s in enumerate(b.sections):
                    s_item = QTreeWidgetItem(sec_item, ["section", s.name, ""])
                    s_item.setData(0, Qt.UserRole, ("section", wi, bi, si))
                    s_item.setFlags(s_item.flags() | Qt.ItemIsEditable)

                    sv = QTreeWidgetItem(s_item, ["vars", "", ""])
                    sv.setFlags(sv.flags() & ~Qt.ItemIsEditable)
                    for vn in ["eta","x","y","z"]:
                        vit = QTreeWidgetItem(sv, [vn, str(getattr(s, vn)), ""])
                        vit.setData(0, Qt.UserRole, ("section_var", wi, bi, si, vn))
                        vit.setFlags(vit.flags() | Qt.ItemIsEditable)
                        opt = s.var_opt.get(vn, VarMeta(False)).optimize if vn in s.var_opt else False
                        vit.setCheckState(2, Qt.Checked if opt else Qt.Unchecked)

        self.tree.expandToDepth(1)
        self._restore_expanded_tags(expanded)
        # restore selection
        if cur_tag is not None:
            def find(it: QTreeWidgetItem):
                if it.data(0, Qt.UserRole) == cur_tag:
                    self.tree.setCurrentItem(it)
                    return True
                for i in range(it.childCount()):
                    if find(it.child(i)):
                        return True
                return False
            if self.tree.topLevelItemCount():
                find(self.tree.topLevelItem(0))
        self.tree.blockSignals(False)


    def _make_prop_row_widget(self, editor: QWidget, opt_checked: bool, on_opt, constr_checked: bool, on_constr) -> QWidget:
        w = QWidget()
        lay = QHBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        lay.addWidget(editor, 1)
        opt = QCheckBox("Opt")
        opt.setChecked(bool(opt_checked))
        opt.setTristate(False)
        opt.clicked.connect(lambda checked: (None if self.props._block else QTimer.singleShot(0, lambda: on_opt(bool(checked)))))
        lay.addWidget(opt)
        constr = QCheckBox("Constr")
        constr.setTristate(False)
        constr.setChecked(bool(constr_checked))
        constr.clicked.connect(lambda checked: (None if self.props._block else QTimer.singleShot(0, lambda: on_constr(bool(checked)))))
        lay.addWidget(constr)
        return w

    def on_tree_selection_changed(self):
        it = self.tree.currentItem()
        if not it:
            self.props.clear()
            self.props.set_title("Properties")
            return
        tag = it.data(0, Qt.UserRole)
        if not tag or not isinstance(tag, tuple):
            self.props.clear()
            self.props.set_title("Properties")
            return

        kind = tag[0]
        self.props._block = True
        try:
            self.props.clear()
            if kind == "wing":
                wi = tag[1]
                w = self.project.wings[wi]
                self.props.set_title(f"Wing: {w.name}")
                # Name editable
                name_edit = QLineEdit(w.name)
                name_edit.editingFinished.connect(lambda: self._set_wing_name_from_panel(wi, name_edit.text()))
                self.props.form.addRow("name", name_edit)
                sp_wd = QDoubleSpinBox(); sp_wd.setDecimals(6); sp_wd.setRange(0.0, 1e9); sp_wd.setSingleStep(0.05); sp_wd.setValue(float(w.density_kg_m2))
                sp_wd.valueChanged.connect(lambda v: (None if self.props._block else self._set_wing_density(wi, float(v))))
                self.props.form.addRow("Density [kg/m²]", sp_wd)
                # Add bay
                btn = QPushButton("Add bay to this wing")
                btn.clicked.connect(lambda: self._add_bay_to_wing_index(wi))
                self.props.form.addRow("", btn)

            elif kind in ("bay", "bay_var", "constraints", "constraint_item", "section", "section_var"):
                # map to bay indices if possible
                wi = None; bi = None; si = None
                if kind in ("bay", "bay_var", "section", "section_var"):
                    wi, bi = tag[1], tag[2]
                    if kind in ("section", "section_var"):
                        si = tag[3] if kind == "section" else tag[3]
                elif kind == "constraints":
                    flat = int(tag[1])
                    wi, bi = self._flat_to_wing_bay(flat)
                elif kind == "constraint_item":
                    flat = int(tag[2])
                    wi, bi = self._flat_to_wing_bay(flat)

                if wi is None or bi is None:
                    self.props.set_title("Properties")
                    return

                b = self.project.wings[wi].bays[bi]
                self.props.set_title(f"Bay: {self.project.wings[wi].name}/{b.name}")

                tabs = QTabWidget()
                tabs.setTabPosition(QTabWidget.North)
                self.props.form.addRow("", tabs)

                tab_inputs = QWidget()
                form_inputs = QFormLayout(tab_inputs)

                def add_input_dspin(label: str, value: float, on_commit, step: float = 0.05):
                    sp = QDoubleSpinBox()
                    sp.setDecimals(6)
                    sp.setRange(-1e9, 1e9)
                    sp.setSingleStep(step)
                    sp.setValue(float(value))
                    sp.valueChanged.connect(lambda v: (None if self.props._block else on_commit(float(v))))
                    form_inputs.addRow(label, sp)
                    return sp

                def add_input_combo(label: str, options, current, on_commit):
                    cb = QComboBox()
                    cb.addItems(list(options))
                    cb.setCurrentText(str(current))
                    cb.currentTextChanged.connect(lambda t: (None if self.props._block else on_commit(t)))
                    form_inputs.addRow(label, cb)
                    return cb

                # Name
                nm = QLineEdit(b.name)
                nm.editingFinished.connect(lambda: self._set_bay_name_from_panel(wi, bi, nm.text()))
                form_inputs.addRow("name", nm)

                # Tip chord mode (CTIP/TAPER)
                cb_tip = add_input_combo("tip_chord_mode", ["CTIP", "TAPER"], b.tip_chord_mode.upper(),
                                              lambda t: self._set_bay_tip_mode(wi, bi, t))
                # Root chord
                sp_cr = QDoubleSpinBox(); sp_cr.setDecimals(6); sp_cr.setRange(-1e9, 1e9); sp_cr.setSingleStep(0.05); sp_cr.setValue(float(b.c_root))
                sp_cr.valueChanged.connect(lambda v: (None if self.props._block else self._set_bay_num(wi, bi, "c_root", float(v))))
                slave_flat = self._wing_bay_to_flat(wi, bi)
                wrow = self._make_prop_row_widget(
                    sp_cr,
                    b.var_opt.get("c_root", VarMeta(False)).optimize,
                    lambda chk: self._set_opt_flag_bay(wi, bi, "c_root", chk),
                    self._slave_has_constraint(slave_flat, ConnectionType.MATCH_CROOT_CTIP),
                    lambda on: self._toggle_var_constraint(slave_flat, ConnectionType.MATCH_CROOT_CTIP, on, title="Constraint: c_root (slave)=c_tip (master)")
                )
                form_inputs.addRow("c_root [m]", wrow)

                # Depending on mode: enable c_tip or taper
                sp_ct = add_input_dspin("c_tip [m]", b.c_tip, lambda v: self._set_bay_num(wi, bi, "c_tip", v), step=0.05)
                sp_ta = add_input_dspin("taper (Ct/Cr) [-]", b.taper, lambda v: self._set_bay_num(wi, bi, "taper", v), step=0.02)

                if b.tip_chord_mode.upper() == "CTIP":
                    sp_ct.setEnabled(True)
                    sp_ta.setEnabled(False)
                else:
                    sp_ct.setEnabled(False)
                    sp_ta.setEnabled(True)

                # Sweep mode (LE/C4/TE) + sweep angle
                add_input_combo("sweep_mode", ["LE", "C4", "TE"], b.sweep_mode.upper(),
                                     lambda t: self._set_bay_mode(wi, bi, "sweep_mode", t))
                sp_sw = QDoubleSpinBox(); sp_sw.setDecimals(6); sp_sw.setRange(-1e9, 1e9); sp_sw.setSingleStep(0.5); sp_sw.setValue(float(b.sweep_deg))
                sp_sw.valueChanged.connect(lambda v: (None if self.props._block else self._set_bay_num(wi, bi, "sweep_deg", float(v))))
                # Constrained sweep depends on sweep_mode
                def sweep_ctype():
                    m = b.sweep_mode.upper()
                    if m == "LE":
                        return ConnectionType.MATCH_SWEEP_LE
                    if m == "TE":
                        return ConnectionType.MATCH_SWEEP_TE
                    return ConnectionType.MATCH_SWEEP
                slave_flat = self._wing_bay_to_flat(wi, bi)
                ctype_sw = sweep_ctype()
                constrained_sw = (
                    self._slave_has_constraint(slave_flat, ConnectionType.MATCH_SWEEP) or
                    self._slave_has_constraint(slave_flat, ConnectionType.MATCH_SWEEP_LE) or
                    self._slave_has_constraint(slave_flat, ConnectionType.MATCH_SWEEP_TE)
                )
                wrow = self._make_prop_row_widget(
                    sp_sw,
                    b.var_opt.get("sweep_deg", VarMeta(False)).optimize,
                    lambda chk: self._set_opt_flag_bay(wi, bi, "sweep_deg", chk),
                    constrained_sw,
                    lambda on: self._toggle_var_constraint(slave_flat, ctype_sw, on, title="Constraint: sweep_deg (match sweep)")
                )
                form_inputs.addRow("sweep_deg [deg]", wrow)

                # Position LE root
                sp_x = QDoubleSpinBox(); sp_x.setDecimals(6); sp_x.setRange(-1e9, 1e9); sp_x.setSingleStep(0.05); sp_x.setValue(float(b.x_le_root))
                sp_x.valueChanged.connect(lambda v: (None if self.props._block else self._set_bay_num(wi, bi, "x_le_root", float(v))))
                slave_flat = self._wing_bay_to_flat(wi, bi)
                wrow = self._make_prop_row_widget(
                    sp_x,
                    b.var_opt.get("x_le_root", VarMeta(False)).optimize,
                    lambda chk: self._set_opt_flag_bay(wi, bi, "x_le_root", chk),
                    self._slave_has_constraint(slave_flat, ConnectionType.LOCK_LE),
                    lambda on: self._toggle_var_constraint(slave_flat, ConnectionType.LOCK_LE, on, title="Constraint: LOCK_LE (same root LE point)")
                )
                form_inputs.addRow("x_le_root [m]", wrow)
                add_input_dspin("y_le_root [m]", b.y_le_root, lambda v: self._set_bay_num(wi, bi, "y_le_root", v), step=0.05)
                add_input_dspin("z_le_root [m]", b.z_le_root, lambda v: self._set_bay_num(wi, bi, "z_le_root", v), step=0.05)

                # Span/dihedral
                sp_span = add_input_dspin("span [m]", b.span, lambda v: self._set_bay_num(wi, bi, "span", v), step=0.05)
                sp_di = QDoubleSpinBox(); sp_di.setDecimals(6); sp_di.setRange(-1e9, 1e9); sp_di.setSingleStep(0.5); sp_di.setValue(float(b.dihedral_deg))
                sp_di.valueChanged.connect(lambda v: (None if self.props._block else self._set_bay_num(wi, bi, "dihedral_deg", float(v))))
                slave_flat = self._wing_bay_to_flat(wi, bi)
                wrow = self._make_prop_row_widget(
                    sp_di,
                    b.var_opt.get("dihedral_deg", VarMeta(False)).optimize,
                    lambda chk: self._set_opt_flag_bay(wi, bi, "dihedral_deg", chk),
                    self._slave_has_constraint(slave_flat, ConnectionType.MATCH_DIHEDRAL),
                    lambda on: self._toggle_var_constraint(slave_flat, ConnectionType.MATCH_DIHEDRAL, on, title="Constraint: dihedral (match)")
                )
                form_inputs.addRow("dihedral_deg [deg]", wrow)

                # Derived geometry (span interpreted as DY)
                lbl_l3d = QLabel()
                lbl_gamma = QLabel()
                lbl_lambda = QLabel()

                def update_derived_geometry_labels() -> None:
                    tmp = AvlBay(
                        x_le_root=b.x_le_root,
                        y_le_root=b.y_le_root,
                        z_le_root=b.z_le_root,
                        c_root=b.c_root,
                        tip_chord_mode=b.tip_chord_mode,
                        c_tip=b.c_tip,
                        taper=b.taper,
                        span=float(sp_span.value()),
                        dihedral_deg=float(sp_di.value()),
                        sweep_mode=b.sweep_mode,
                        sweep_deg=float(sp_sw.value()),
                        twist_root_deg=b.twist_root_deg,
                        twist_tip_deg=b.twist_tip_deg,
                        rigid_inc_deg=b.rigid_inc_deg,
                        nchord=b.nchord,
                        cspace=b.cspace,
                        nspan=b.nspan,
                        sspace=b.sspace,
                    )

                    dy, dz = tmp.dy_dz()
                    gamma = tmp.dihedral_from_yz_deg()
                    sweep_xy = tmp.sweep_from_xy_deg()
                    lbl_l3d.setText(f"{tmp.length_3d():.6g}")
                    lbl_gamma.setText(f"{gamma:.6g}  (atan2(DZ,DY), DY={dy:.6g}, DZ={dz:.6g})")
                    lbl_lambda.setText(f"{sweep_xy:.6g}  (atan2(DX,DY))")

                sp_span.valueChanged.connect(lambda _v: update_derived_geometry_labels())
                sp_di.valueChanged.connect(lambda _v: update_derived_geometry_labels())
                sp_sw.valueChanged.connect(lambda _v: update_derived_geometry_labels())
                form_inputs.addRow("L3D [m]", lbl_l3d)
                form_inputs.addRow("dihedral_yz_deg [deg]", lbl_gamma)
                form_inputs.addRow("sweep_xy_deg [deg]", lbl_lambda)
                update_derived_geometry_labels()

                # Twist
                sp_tr = QDoubleSpinBox(); sp_tr.setDecimals(6); sp_tr.setRange(-1e9, 1e9); sp_tr.setSingleStep(0.2); sp_tr.setValue(float(b.twist_root_deg))
                sp_tr.valueChanged.connect(lambda v: (None if self.props._block else self._set_bay_num(wi, bi, "twist_root_deg", float(v))))
                slave_flat = self._wing_bay_to_flat(wi, bi)
                wrow = self._make_prop_row_widget(
                    sp_tr,
                    b.var_opt.get("twist_root_deg", VarMeta(False)).optimize,
                    lambda chk: self._set_opt_flag_bay(wi, bi, "twist_root_deg", chk),
                    self._slave_has_constraint(slave_flat, ConnectionType.MATCH_TWIST),
                    lambda on: self._toggle_var_constraint(slave_flat, ConnectionType.MATCH_TWIST, on, title="Constraint: twist (match)")
                )
                form_inputs.addRow("twist_root_deg [deg]", wrow)
                add_input_dspin("twist_tip_deg [deg]", b.twist_tip_deg, lambda v: self._set_bay_num(wi, bi, "twist_tip_deg", v), step=0.2)
                add_input_dspin("rigid_inc_deg [deg]", b.rigid_inc_deg, lambda v: self._set_bay_num(wi, bi, "rigid_inc_deg", v), step=0.2)


                # Root/Tip sections (theta + airfoil) moved from tree to Properties
                update_default_sections_from_bay(b)
                root_s = b.sections[0]
                tip_s = b.sections[-1]

                # Computed section theta (rigid_inc + local twist). Editable: updates twists.
                theta_root = b.rigid_inc_deg + b.twist_root_deg
                theta_tip = b.rigid_inc_deg + b.twist_tip_deg

                sp_th_r = QDoubleSpinBox(); sp_th_r.setDecimals(6); sp_th_r.setRange(-360.0, 360.0); sp_th_r.setSingleStep(0.2); sp_th_r.setValue(float(theta_root))
                sp_th_t = QDoubleSpinBox(); sp_th_t.setDecimals(6); sp_th_t.setRange(-360.0, 360.0); sp_th_t.setSingleStep(0.2); sp_th_t.setValue(float(theta_tip))

                sp_th_r.valueChanged.connect(lambda v: (None if self.props._block else self._set_bay_num(wi, bi, "twist_root_deg", float(v) - b.rigid_inc_deg)))
                sp_th_t.valueChanged.connect(lambda v: (None if self.props._block else self._set_bay_num(wi, bi, "twist_tip_deg", float(v) - b.rigid_inc_deg)))

                form_inputs.addRow("section.theta_root_deg [deg]", sp_th_r)
                form_inputs.addRow("section.theta_tip_deg [deg]", sp_th_t)

                # Airfoil editors: format "NACA:2412" or "FILE:path"
                def airfoil_to_str(a: AirfoilRef) -> str:
                    if a.kind.upper() == "NACA":
                        return f"NACA:{a.name}"
                    if a.kind.upper() == "FILE":
                        return f"FILE:{a.filepath}"
                    return f"{a.kind}:{a.name}"

                def parse_airfoil(s: str) -> AirfoilRef:
                    t = (s or "").strip()
                    if ":" in t:
                        k, v = t.split(":", 1)
                        k = k.strip().upper()
                        v = v.strip()
                        if k == "NACA":
                            return AirfoilRef("NACA", v, "")
                        if k in ("FILE", "AFIL"):
                            return AirfoilRef("FILE", "", v)
                        return AirfoilRef(k, v, "")
                    # fallback assume NACA
                    return AirfoilRef("NACA", t, "")

                ed_af_r = QLineEdit(airfoil_to_str(root_s.airfoil))
                ed_af_t = QLineEdit(airfoil_to_str(tip_s.airfoil))

                # If this bay is a slave in any constraint, its root airfoil is constrained by master tip
                slave_flat = self._wing_bay_to_flat(wi, bi)
                is_connected_slave = any(c.slave_index == slave_flat for c in self.project.connections)
                if is_connected_slave:
                    ed_af_r.setEnabled(False)
                    ed_af_r.setToolTip("Root airfoil constrained: equals master tip airfoil.")
                else:
                    ed_af_r.editingFinished.connect(lambda: self._set_section_airfoil(wi, bi, True, ed_af_r.text()))
                ed_af_t.editingFinished.connect(lambda: self._set_section_airfoil(wi, bi, False, ed_af_t.text()))

                form_inputs.addRow("section.airfoil_root", ed_af_r)
                form_inputs.addRow("section.airfoil_tip", ed_af_t)

                sp_nchord = QSpinBox(); sp_nchord.setRange(0, 10**9); sp_nchord.setSingleStep(1); sp_nchord.setValue(int(b.nchord))
                sp_nspan = QSpinBox(); sp_nspan.setRange(0, 10**9); sp_nspan.setSingleStep(1); sp_nspan.setValue(int(b.nspan))
                sp_nchord.valueChanged.connect(lambda v: (None if self.props._block else self._set_bay_int(wi, bi, "nchord", int(v))))
                sp_nspan.valueChanged.connect(lambda v: (None if self.props._block else self._set_bay_int(wi, bi, "nspan", int(v))))
                form_inputs.addRow("NChord [-]", sp_nchord)
                form_inputs.addRow("NSpan [-]", sp_nspan)

                chk_use_wing_density = QCheckBox("Use wing density")
                chk_use_wing_density.setChecked(bool(b.use_wing_density))
                sp_bd = QDoubleSpinBox(); sp_bd.setDecimals(6); sp_bd.setRange(0.0, 1e9); sp_bd.setSingleStep(0.05)
                sp_bd.setValue(float(b.density_kg_m2))
                sp_bd.setEnabled(not b.use_wing_density)
                chk_use_wing_density.toggled.connect(lambda on: (None if self.props._block else self._set_bay_use_wing_density(wi, bi, bool(on))))
                sp_bd.valueChanged.connect(lambda v: (None if self.props._block else self._set_bay_density(wi, bi, float(v))))
                form_inputs.addRow("", chk_use_wing_density)
                form_inputs.addRow("Density [kg/m²]", sp_bd)

                tab_geom = QWidget()
                form_geom = QFormLayout(tab_geom)

                wing = self.project.wings[wi]
                bay_density = wing.density_kg_m2 if b.use_wing_density else b.density_kg_m2
                bay_geo = compute_bay_geometry_summary(b, bay_density)
                wing_geos = [
                    compute_bay_geometry_summary(wb, wing.density_kg_m2 if wb.use_wing_density else wb.density_kg_m2)
                    for wb in wing.bays
                ]
                wing_geo = combine_geometry_summaries(wing_geos)

                form_geom.addRow("--- Selected Bay ---", QLabel(""))
                form_geom.addRow("Area [m²]", QLabel(f"{bay_geo.area:.6g}"))
                form_geom.addRow("MAC [m]", QLabel(f"{bay_geo.cma:.6g}"))
                form_geom.addRow("MGC [m]", QLabel(f"{bay_geo.cmg:.6g}"))
                form_geom.addRow("Mass [kg]", QLabel(f"{bay_geo.mass:.6g}"))
                form_geom.addRow("CG X [m]", QLabel(f"{bay_geo.cg[0]:.6g}"))
                form_geom.addRow("CG Y [m]", QLabel(f"{bay_geo.cg[1]:.6g}"))
                form_geom.addRow("CG Z [m]", QLabel(f"{bay_geo.cg[2]:.6g}"))

                form_geom.addRow("--- Full Wing ---", QLabel(""))
                form_geom.addRow("Wing Area [m²]", QLabel(f"{wing_geo.area:.6g}"))
                form_geom.addRow("Wing MAC [m]", QLabel(f"{wing_geo.cma:.6g}"))
                form_geom.addRow("Wing MGC [m]", QLabel(f"{wing_geo.cmg:.6g}"))
                form_geom.addRow("Wing Mass [kg]", QLabel(f"{wing_geo.mass:.6g}"))
                form_geom.addRow("Wing CG X [m]", QLabel(f"{wing_geo.cg[0]:.6g}"))
                form_geom.addRow("Wing CG Y [m]", QLabel(f"{wing_geo.cg[1]:.6g}"))
                form_geom.addRow("Wing CG Z [m]", QLabel(f"{wing_geo.cg[2]:.6g}"))

                form_geom.addRow("--- Inertias ---", QLabel(""))
                for inert_attr, label in [
                    ("mass", "Mass override [kg]"),
                    ("Ixx", "Ixx [kg·m²]"),
                    ("Iyy", "Iyy [kg·m²]"),
                    ("Izz", "Izz [kg·m²]"),
                    ("Ixy", "Ixy [kg·m²]"),
                    ("Ixz", "Ixz [kg·m²]"),
                    ("Iyz", "Iyz [kg·m²]"),
                ]:
                    sp_in = QDoubleSpinBox()
                    sp_in.setDecimals(6)
                    sp_in.setRange(-1e9, 1e9)
                    sp_in.setSingleStep(0.1)
                    sp_in.setValue(float(getattr(b.inertial, inert_attr) or 0.0))
                    sp_in.valueChanged.connect(lambda v, a=inert_attr: (None if self.props._block else self._set_bay_inertial_num(wi, bi, a, float(v))))
                    form_geom.addRow(label, sp_in)

                tab_masses = QWidget()
                lay_mass = QVBoxLayout(tab_masses)
                table = QTableWidget(len(b.concentrated_masses), 4)
                table.setHorizontalHeaderLabels(["X [m]", "Y [m]", "Z [m]", "Mass [kg]"])
                table.verticalHeader().setVisible(False)
                table.setSelectionBehavior(QAbstractItemView.SelectRows)
                table.setSelectionMode(QAbstractItemView.SingleSelection)
                table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
                for r, cm in enumerate(b.concentrated_masses):
                    table.setItem(r, 0, QTableWidgetItem(f"{cm.x:.6g}"))
                    table.setItem(r, 1, QTableWidgetItem(f"{cm.y:.6g}"))
                    table.setItem(r, 2, QTableWidgetItem(f"{cm.z:.6g}"))
                    table.setItem(r, 3, QTableWidgetItem(f"{cm.mass:.6g}"))

                def _on_mass_cell_changed(_row: int, _col: int):
                    if self.props._block:
                        return
                    new_list: List[ConcentratedMass] = []
                    for rr in range(table.rowCount()):
                        vals = []
                        for cc in range(4):
                            itv = table.item(rr, cc)
                            try:
                                vals.append(float(itv.text()) if itv else 0.0)
                            except Exception:
                                vals.append(0.0)
                        new_list.append(ConcentratedMass(x=vals[0], y=vals[1], z=vals[2], mass=vals[3]))
                    self._set_bay_concentrated_masses(wi, bi, new_list)

                table.cellChanged.connect(_on_mass_cell_changed)
                lay_mass.addWidget(table)

                row_btn = QHBoxLayout()
                btn_add_mass = QPushButton("Add mass")
                btn_del_mass = QPushButton("Remove selected")
                row_btn.addWidget(btn_add_mass)
                row_btn.addWidget(btn_del_mass)
                row_btn.addStretch(1)
                lay_mass.addLayout(row_btn)

                btn_add_mass.clicked.connect(lambda: self._add_bay_concentrated_mass(wi, bi))
                btn_del_mass.clicked.connect(lambda: self._remove_selected_bay_concentrated_mass(wi, bi, table.currentRow()))

                tabs.addTab(tab_inputs, "INPUT")
                tabs.addTab(tab_geom, "GEO/Inertias")
                tabs.addTab(tab_masses, "Concentrated Masses")

        finally:
            self.props._block = False

    def _wing_bay_to_flat(self, wi: int, bi: int) -> int:
        k = 0
        for wj, w in enumerate(self.project.wings):
            for bj, _ in enumerate(w.bays):
                if (wj, bj) == (wi, bi):
                    return k
                k += 1
        return 0

    def _set_opt_flag_bay(self, wi: int, bi: int, varname: str, on: bool):
        b = self.project.wings[wi].bays[bi]
        if varname in b.var_opt:
            b.var_opt[varname].optimize = bool(on)
        self.rebuild_tree()

    def _toggle_var_constraint(self, slave_flat: int, ctype: str, on: bool, title: str = "Constraint"):
        """Toggle a specific constraint type for a given slave bay.
        If turning ON, opens a dialog to choose the master bay.
        """
        if not on:
            self._remove_constraint_type_for_slave(slave_flat, ctype)
            self._apply_constraints_to_models()
            self.rebuild_tree()
            self.refresh_scene()
            self.on_tree_selection_changed()
            return

        names = self._flat_bay_names()
        if len(names) < 2:
            QMessageBox.information(self, "Constraint", "Need at least 2 bays to constrain.")
            self.on_tree_selection_changed()
            return

        default_master = getattr(self, "_last_connect_master", 0)
        dlg = VarConstraintDialog(self, names, default_master, slave_flat, title)
        if dlg.exec() != QDialog.Accepted:
            # revert UI state
            self.on_tree_selection_changed()
            return

        master = dlg.master_index()
        if master == slave_flat:
            QMessageBox.warning(self, "Constraint", "Master and Slave must be different.")
            self.on_tree_selection_changed()
            return

        # Sweep exclusivity
        if ctype in (ConnectionType.MATCH_SWEEP, ConnectionType.MATCH_SWEEP_LE, ConnectionType.MATCH_SWEEP_TE):
            self._remove_constraint_type_for_slave(slave_flat, ConnectionType.MATCH_SWEEP)
            self._remove_constraint_type_for_slave(slave_flat, ConnectionType.MATCH_SWEEP_LE)
            self._remove_constraint_type_for_slave(slave_flat, ConnectionType.MATCH_SWEEP_TE)

        self._set_constraint_type_for_slave(slave_flat, ctype, master)
        self._last_connect_master = master
        self._last_connect_slave = slave_flat

        self._apply_constraints_to_models()
        self.rebuild_tree()
        self.refresh_scene()
        self.on_tree_selection_changed()

        names = self._flat_bay_names()
        default_master = getattr(self, "_last_connect_master", 0)
        dlg = VarConstraintDialog(self, names, default_master, slave_flat, title)
        if dlg.exec() != QDialog.Accepted:
            # revert UI state
            self.on_tree_selection_changed()
            return
        master = dlg.master_index()
        if master == slave_flat:
            QMessageBox.warning(self, "Constraint", "Master and Slave must be different.")
            self.on_tree_selection_changed()
            return

        # handle sweep exclusivity when setting sweep constraint
        if ctype in (ConnectionType.MATCH_SWEEP, ConnectionType.MATCH_SWEEP_LE, ConnectionType.MATCH_SWEEP_TE):
            self._remove_constraint_type_for_slave(slave_flat, ConnectionType.MATCH_SWEEP)
            self._remove_constraint_type_for_slave(slave_flat, ConnectionType.MATCH_SWEEP_LE)
            self._remove_constraint_type_for_slave(slave_flat, ConnectionType.MATCH_SWEEP_TE)

        self._set_constraint_type_for_slave(slave_flat, ctype, master)
        self._last_connect_master = master
        self._last_connect_slave = slave_flat
        self._apply_constraints_to_models()
        self.rebuild_tree()
        self.refresh_scene()
        self.on_tree_selection_changed()
        self.on_tree_selection_changed()

        k = 0
        for wi, w in enumerate(self.project.wings):
            for bi, _b in enumerate(w.bays):
                if k == flat_idx:
                    return wi, bi
                k += 1
        return None, None

    def _set_wing_name_from_panel(self, wi: int, name: str):
        self.project.wings[wi].name = (name.strip() or self.project.wings[wi].name)
        self.rebuild_tree()
        self.refresh_scene()

    def _set_bay_name_from_panel(self, wi: int, bi: int, name: str):
        b = self.project.wings[wi].bays[bi]
        b.name = (name.strip() or b.name)
        self.rebuild_tree()
        self.refresh_scene()

    def _set_wing_density(self, wi: int, density: float):
        self.project.wings[wi].density_kg_m2 = max(0.0, float(density))
        QTimer.singleShot(0, self.on_tree_selection_changed)

    def _set_bay_use_wing_density(self, wi: int, bi: int, on: bool):
        self.project.wings[wi].bays[bi].use_wing_density = bool(on)
        QTimer.singleShot(0, self.on_tree_selection_changed)

    def _set_bay_density(self, wi: int, bi: int, density: float):
        self.project.wings[wi].bays[bi].density_kg_m2 = max(0.0, float(density))
        QTimer.singleShot(0, self.on_tree_selection_changed)

    def _set_bay_inertial_num(self, wi: int, bi: int, attr: str, val: float):
        inert = self.project.wings[wi].bays[bi].inertial
        setattr(inert, attr, float(val))

    def _set_bay_concentrated_masses(self, wi: int, bi: int, masses: List[ConcentratedMass]):
        self.project.wings[wi].bays[bi].concentrated_masses = list(masses)
        self.on_tree_selection_changed()

    def _add_bay_concentrated_mass(self, wi: int, bi: int):
        self.project.wings[wi].bays[bi].concentrated_masses.append(ConcentratedMass())
        self.on_tree_selection_changed()

    def _remove_selected_bay_concentrated_mass(self, wi: int, bi: int, row: int):
        b = self.project.wings[wi].bays[bi]
        if 0 <= row < len(b.concentrated_masses):
            b.concentrated_masses.pop(row)
            self.on_tree_selection_changed()

    def _set_bay_tip_mode(self, wi: int, bi: int, mode: str):
        b = self.project.wings[wi].bays[bi]
        mode = mode.upper().strip()
        if mode not in ("CTIP", "TAPER"):
            return
        b.tip_chord_mode = mode
        # Keep coherence
        if mode == "CTIP":
            # derive taper
            b.taper = (b.c_tip / b.c_root) if b.c_root != 0 else b.taper
        else:
            # derive c_tip
            b.c_tip = b.taper * b.c_root
        update_default_sections_from_bay(b)
        self.rebuild_tree()
        self.refresh_scene()
        # refresh panel enable/disable
        self.on_tree_selection_changed()

    def _set_bay_num(self, wi: int, bi: int, attr: str, val: float):
        b = self.project.wings[wi].bays[bi]
        setattr(b, attr, float(val))
        # Maintain Ct/taper coherence
        if attr in ("c_root", "c_tip", "taper", "tip_chord_mode"):
            if b.tip_chord_mode.upper() == "CTIP":
                b.taper = (b.c_tip / b.c_root) if b.c_root != 0 else b.taper
            else:
                b.c_tip = b.taper * b.c_root
        update_default_sections_from_bay(b)
        self.rebuild_tree()
        self.refresh_scene()

    def _set_bay_int(self, wi: int, bi: int, attr: str, val: int):
        b = self.project.wings[wi].bays[bi]
        setattr(b, attr, int(val))
        update_default_sections_from_bay(b)
        self.rebuild_tree()
        self.refresh_scene()

    def _set_section_airfoil(self, wi: int, bi: int, is_root: bool, text: str):
        b = self.project.wings[wi].bays[bi]
        update_default_sections_from_bay(b)
        s = b.sections[0] if is_root else b.sections[-1]

        t = (text or "").strip()
        if ":" in t:
            k, v = t.split(":", 1)
            k = k.strip().upper(); v = v.strip()
            if k == "NACA":
                s.airfoil = AirfoilRef("NACA", v, "")
            elif k in ("FILE", "AFIL"):
                s.airfoil = AirfoilRef("FILE", "", v)
            else:
                s.airfoil = AirfoilRef(k, v, "")
        else:
            s.airfoil = AirfoilRef("NACA", t, "")

        # Re-apply constraints (airfoil rule may override root if connected)
        self._apply_constraints_to_models()
        self.rebuild_tree()
        self.refresh_scene()
        self.on_tree_selection_changed()

    def on_tree_item_changed(self, item: QTreeWidgetItem, col: int):
        tag = item.data(0, Qt.UserRole)
        if not tag:
            return
        kind = tag[0]

        if kind == "wing" and col == 1:
            wi = tag[1]
            self.project.wings[wi].name = item.text(1).strip() or self.project.wings[wi].name
            return

        if kind == "bay" and col == 1:
            wi, bi = tag[1], tag[2]
            self.project.wings[wi].bays[bi].name = item.text(1).strip() or self.project.wings[wi].bays[bi].name
            return

        if kind == "section" and col == 1:
            wi, bi, si = tag[1], tag[2], tag[3]
            self.project.wings[wi].bays[bi].sections[si].name = item.text(1).strip() or self.project.wings[wi].bays[bi].sections[si].name
            return

        if kind == "bay_var":
            wi, bi, varname = tag[1], tag[2], tag[3]
            b = self.project.wings[wi].bays[bi]
            if col == 1:
                txt = item.text(1).strip()
                try:
                    if varname in ("tip_chord_mode", "sweep_mode"):
                        setattr(b, varname, txt.upper())
                    else:
                        setattr(b, varname, float(txt.replace(",", ".")))
                except Exception:
                    pass
                update_default_sections_from_bay(b)
                self.refresh_scene()
                self.rebuild_tree()
            elif col == 2 and varname in b.var_opt:
                b.var_opt[varname].optimize = (item.checkState(2) == Qt.Checked)

        if kind == "section_var":
            wi, bi, si, varname = tag[1], tag[2], tag[3], tag[4]
            s = self.project.wings[wi].bays[bi].sections[si]
            if col == 1:
                if varname == "airfoil":
                    t = item.text(1).strip()
                    if ":" in t:
                        k, v = t.split(":", 1)
                        k = k.strip().upper(); v = v.strip()
                        if k == "NACA":
                            s.airfoil = AirfoilRef("NACA", v, "")
                        elif k in ("FILE","AFIL"):
                            s.airfoil = AirfoilRef("FILE", "", v)
                else:
                    try:
                        setattr(s, varname, float(item.text(1).strip().replace(",", ".")))
                    except Exception:
                        pass
                update_default_sections_from_bay(self.project.wings[wi].bays[bi])
                self.refresh_scene()
                self.rebuild_tree()
            elif col == 2 and varname in s.var_opt:
                s.var_opt[varname].optimize = (item.checkState(2) == Qt.Checked)


    # ----- Project edit (Add wing/bay + connections) -----
    def _flatten_bays(self) -> List[Tuple[int, int, BayModel]]:
        return flatten_project_bays(self.project)

    def _flat_bay_names(self) -> List[str]:
        return project_flat_bay_names(self.project)


    def _apply_constraints_to_models(self):
        """Apply constraints to BayModel objects so UI reflects dependent values."""
        apply_constraints_to_project(self.project)

    def _slave_has_constraint(self, slave_flat_idx: int, ctype: str) -> bool:
        return any(c.slave_index == slave_flat_idx and c.ctype == ctype for c in self.project.connections)

    def _remove_constraint_type_for_slave(self, slave_flat_idx: int, ctype: str):
        self.project.connections = [c for c in self.project.connections if not (c.slave_index == slave_flat_idx and c.ctype == ctype)]

    def _set_constraint_type_for_slave(self, slave_flat_idx: int, ctype: str, master_idx: int):
        # remove existing of that type for slave, then add
        self._remove_constraint_type_for_slave(slave_flat_idx, ctype)
        self.project.connections.append(BayConnection(master_index=master_idx, slave_index=slave_flat_idx, ctype=ctype))


    def _clear_constraints_for_slave(self, slave_flat_idx: int):
        self.project.connections = [c for c in self.project.connections if c.slave_index != slave_flat_idx]
        self._apply_constraints_to_models()
        self.rebuild_tree()
        self.refresh_scene()
        self.on_tree_selection_changed()

    def _remove_single_constraint(self, master_idx: int, slave_idx: int, ctype: str):
        self.project.connections = [
            c for c in self.project.connections
            if not (c.master_index == master_idx and c.slave_index == slave_idx and c.ctype == ctype)
        ]
        self._apply_constraints_to_models()
        self.rebuild_tree()
        self.refresh_scene()
        self.on_tree_selection_changed()


    def _remove_slave_connection(self, slave_flat_idx: int):
        self.project.connections = [c for c in self.project.connections if c.slave_index != slave_flat_idx]

    def _get_slave_connection(self, slave_flat_idx: int) -> Optional[BayConnection]:
        for c in self.project.connections:
            if c.slave_index == slave_flat_idx:
                return c
        return None


    def _reindex_constraints_after_remove(self):
        """Reindex flat bay constraints after bays list changes."""
        # Build mapping from old flat idx to new flat idx by current flatten order
        # We'll conservatively drop constraints with out-of-range indices.
        flats = self._flatten_bays()
        n = len(flats)
        self.project.connections = [
            c for c in self.project.connections
            if 0 <= c.master_index < n and 0 <= c.slave_index < n and c.master_index != c.slave_index
        ]

    def remove_selected_wing(self):
        it = self.tree.currentItem()
        if not it:
            return
        tag = it.data(0, Qt.UserRole)
        wi = None
        if tag and tag[0] == "wing":
            wi = tag[1]
        elif tag and tag[0] in ("bay", "bay_var", "section", "section_var", "constraints", "constraint_item"):
            wi = tag[1] if tag[0] in ("bay", "bay_var", "section", "section_var") else self._flat_to_wing_bay(int(tag[1] if tag[0]=="constraints" else tag[2]))[0]
        if wi is None or wi < 0 or wi >= len(self.project.wings):
            return
        if QMessageBox.question(self, "Remove Wing", f"Remove wing '{self.project.wings[wi].name}' and all its bays?") != QMessageBox.Yes:
            return
        # Remove wing
        self.project.wings.pop(wi)
        # Drop all constraints (indices will be wrong); easiest rebuild by filtering to range after removal
        self._reindex_constraints_after_remove()
        self._apply_constraints_to_models()
        self.rebuild_tree()
        self.refresh_scene()
        self.on_tree_selection_changed()

    def remove_selected_bay(self):
        it = self.tree.currentItem()
        if not it:
            return
        tag = it.data(0, Qt.UserRole)
        if not tag:
            return
        wi = bi = None
        if tag[0] in ("bay", "bay_var", "section", "section_var"):
            wi, bi = tag[1], tag[2]
        elif tag[0] in ("constraints",):
            wi, bi = self._flat_to_wing_bay(int(tag[1]))
        elif tag[0] in ("constraint_item",):
            wi, bi = self._flat_to_wing_bay(int(tag[2]))
        if wi is None or bi is None:
            return
        w = self.project.wings[wi]
        if bi < 0 or bi >= len(w.bays):
            return
        if QMessageBox.question(self, "Remove Bay", f"Remove bay '{w.bays[bi].name}' from wing '{w.name}'?") != QMessageBox.Yes:
            return
        w.bays.pop(bi)
        self._reindex_constraints_after_remove()
        self._apply_constraints_to_models()
        self.rebuild_tree()
        self.refresh_scene()
        self.on_tree_selection_changed()

    def add_wing(self):
        w = WingModel(name=f"wing{len(self.project.wings)+1}")
        self.project.wings.append(w)
        self.rebuild_tree()
        self.refresh_scene()

    def add_bay_to_selected_wing(self):
        # Determine current selected wing from tree; fallback to first wing
        wi = 0
        it = self.tree.currentItem()
        if it:
            tag = it.data(0, Qt.UserRole)
            if tag and tag[0] in ("wing", "bay", "bay_var", "section", "section_var"):
                wi = tag[1]
        if not self.project.wings:
            self.add_wing()
            wi = 0
        w = self.project.wings[wi]
        b = BayModel(name=f"Bay {len(w.bays)+1}")
        update_default_sections_from_bay(b)
        # If there is a previous bay in the same wing, do a minimal attach by default
        if w.bays:
            prev = w.bays[-1]
            # approximate: snap root to previous tip (min attach)
            tmp = baymodel_to_avlbay(prev)
            xt, yt, zt = tmp.tip_le()
            b.x_le_root, b.y_le_root, b.z_le_root = xt, yt, zt
        w.bays.append(b)
        self.rebuild_tree()
        self.refresh_scene()

    def open_connect_dialog(self):
        bays = self._flatten_bays()
        if len(bays) < 2:
            QMessageBox.information(self, "Connect", "Need at least 2 bays.")
            return
        names = self._flat_bay_names()

        # default slave: current selected bay if any
        slave_flat = getattr(self, "_last_connect_slave", 0)
        it = self.tree.currentItem()
        if it:
            tag = it.data(0, Qt.UserRole)
            if tag and tag[0] in ("bay", "bay_var", "section", "section_var"):
                wi, bi = tag[1], tag[2]
                k = 0
                for wj, w in enumerate(self.project.wings):
                    for bj, _ in enumerate(w.bays):
                        if (wj, bj) == (wi, bi):
                            slave_flat = k
                        k += 1
            elif tag and tag[0] == "constraints":
                slave_flat = int(tag[1])
            elif tag and tag[0] == "constraint_item":
                slave_flat = int(tag[2])

        active_list = [c.ctype for c in self.project.connections if c.slave_index == slave_flat]
        # if multiple masters exist, prefer the first one
        active_master = None
        for c in self.project.connections:
            if c.slave_index == slave_flat:
                active_master = c.master_index
                break
        default_master = active_master if active_master is not None else getattr(self, "_last_connect_master", (0 if slave_flat != 0 else 1))

        dlg = ConnectBaysDialog(self, names, default_master, slave_flat, active_list)
        if dlg.exec() != QDialog.Accepted:
            return
        m, s, ctypes = dlg.result_values()
        if m == s:
            QMessageBox.warning(self, "Connect", "Master and Slave must be different.")
            return

        # Remove all previous constraints on this slave
        self._remove_slave_connection(s)
        for ct in ctypes:
            self.project.connections.append(BayConnection(master_index=m, slave_index=s, ctype=ct))

        self._last_connect_master = m
        self._last_connect_slave = s

        self.rebuild_tree()
        self.refresh_scene()

    def on_tree_context_menu(self, pos: QPoint):
        it = self.tree.itemAt(pos)
        if not it:
            return
        tag = it.data(0, Qt.UserRole)
        menu = QMenu(self)

        if tag is None or (isinstance(tag, tuple) and tag and tag[0] == "wing"):
            a_add_w = QAction("Add Wing", self); a_add_w.triggered.connect(self.add_wing)
            menu.addAction(a_add_w)

        if isinstance(tag, tuple) and tag and tag[0] == "wing":
            a_add_b = QAction("Add Bay to this Wing", self)
            wi = tag[1]
            a_add_b.triggered.connect(lambda: self._add_bay_to_wing_index(wi))
            menu.addAction(a_add_b)
            a_rm_w = QAction("Remove Wing", self)
            a_rm_w.triggered.connect(self.remove_selected_wing)
            menu.addAction(a_rm_w)

        if isinstance(tag, tuple) and tag and tag[0] in ("bay", "bay_var", "section", "section_var"):
            a_add_b2 = QAction("Add Bay to selected Wing", self); a_add_b2.triggered.connect(self.add_bay_to_selected_wing)
            menu.addAction(a_add_b2)
            a_conn = QAction("Edit constraints (Connect Bays...)...", self); a_conn.triggered.connect(self.open_connect_dialog)
            menu.addAction(a_conn)
            a_rm_b = QAction("Remove Bay", self); a_rm_b.triggered.connect(self.remove_selected_bay)
            menu.addAction(a_rm_b)

        if isinstance(tag, tuple) and tag and tag[0] == "constraints":
            a_edit = QAction("Edit constraints...", self); a_edit.triggered.connect(self.open_connect_dialog)
            menu.addAction(a_edit)
            flat_idx = int(tag[1])
            a_clear = QAction("Remove all constraints for this bay", self)
            a_clear.triggered.connect(lambda: self._clear_constraints_for_slave(flat_idx))
            menu.addAction(a_clear)

        if isinstance(tag, tuple) and tag and tag[0] == "constraint_item":
            master_idx, slave_idx, ctype = int(tag[1]), int(tag[2]), str(tag[3])
            a_rm = QAction(f"Remove {ctype}", self)
            a_rm.triggered.connect(lambda: self._remove_single_constraint(master_idx, slave_idx, ctype))
            menu.addAction(a_rm)
            a_edit2 = QAction("Edit constraints...", self); a_edit2.triggered.connect(self.open_connect_dialog)
            menu.addAction(a_edit2)

        if menu.actions():
            menu.exec(self.tree.mapToGlobal(pos))

    def _add_bay_to_wing_index(self, wi: int):
        if wi < 0 or wi >= len(self.project.wings):
            return
        w = self.project.wings[wi]
        b = BayModel(name=f"Bay {len(w.bays)+1}")
        update_default_sections_from_bay(b)
        if w.bays:
            prev = w.bays[-1]
            tmp = baymodel_to_avlbay(prev)
            xt, yt, zt = tmp.tip_le()
            b.x_le_root, b.y_le_root, b.z_le_root = xt, yt, zt
        w.bays.append(b)
        self.rebuild_tree()
        self.refresh_scene()

    # ----- File IO -----
    def file_open(self):
        p, _ = QFileDialog.getOpenFileName(self, "Open project", "", "AEROSTATE (*.aer)")
        if not p:
            return
        try:
            data = json.loads(Path(p).read_text(encoding="utf-8"))
            pr = project_from_dict(data)
            pr.filepath = p
            self.project = pr
            for w in self.project.wings:
                for b in w.bays:
                    update_default_sections_from_bay(b)
            self.rebuild_tree()
            self.refresh_scene()
        except Exception as e:
            QMessageBox.warning(self, "Open error", str(e))

    def file_save(self):
        if not self.project.filepath:
            return self.file_save_as()
        try:
            Path(self.project.filepath).write_text(json.dumps(project_to_dict(self.project), indent=2), encoding="utf-8")
            self.statusBar().showMessage(f"Saved: {self.project.filepath}", 3000)
        except Exception as e:
            QMessageBox.warning(self, "Save error", str(e))

    def file_save_as(self):
        p, _ = QFileDialog.getSaveFileName(self, "Save project as", "", "AEROSTATE (*.aer)")
        if not p:
            return
        if not p.lower().endswith(".aer"):
            p += ".aer"
        self.project.filepath = p
        self.file_save()

    def export_avl(self):
        p, _ = QFileDialog.getSaveFileName(self, "Export AVL file", "", "AVL (*.avl);;All (*.*)")
        if not p:
            return
        try:
            export_avl_file(self._ordered_avl_bays(), p)
            QMessageBox.information(self, "OK", f"Exported:\n{p}")
        except Exception as e:
            QMessageBox.warning(self, "Export error", str(e))

def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
