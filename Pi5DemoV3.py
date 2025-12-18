# !/usr/bin/env python3
import sys, threading, serial, serial.tools.list_ports, platform
from pathlib import Path
from PySide6.QtCore import Qt, QThread, QObject, Signal, Slot, QTimer, QEvent
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QSizePolicy, QMessageBox,
    QGridLayout, QListWidget, QSpinBox, QDoubleSpinBox, QFormLayout
)
from DMMServoLib import DmmDriver

# Globals
serial_lock = threading.Lock()              # single global lock for ALL serial I/O (Linux-friendly)
availableDrives: list[DmmDriver] = []       # Connected drives
broadcastID = 127                           # default read/write id (safe per-port)
sampleTime = 50                             # time (ms) between polls in DMMWorker

portByAxis: list[str] = []                  # axis -> serial port string

def _isUSBserialPort(p) -> bool:
    """
    Accept only USB serial adapters.
    - Windows: COM*
    - Linux: /dev/ttyUSB* or /dev/ttyACM* (some adapters)
    """
    sysname = platform.system().lower()
    dev = (getattr(p, "device", "") or "").lower()
    if "windows" in sysname:
        return dev.startswith("com")
    if "linux" in sysname:
        return dev.startswith("/dev/ttyusb") or dev.startswith("/dev/ttyacm")
    return False

class DMMWorker(QObject):
    """
    Periodically polls a single axis for pos/spd/torque on a thread and displays
    values for the UI to display. One worker active per selected axis.
    """
    activeWorker = Signal(int, int, int, int)   # axis, pos, spd, trq
    finishedWorker = Signal(int)                # axis

    def __init__(self, axis: int = 0, parent=None):
        super().__init__(parent)
        self._active = False
        self.axis = axis

    @Slot()
    def startLoop(self):
        global availableDrives, sampleTime, broadcastID, portByAxis
        self._active = True
        while self._active:
            try:
                drive = availableDrives[self.axis]
                with serial_lock:
                    did = broadcastID
                    drive.readMotorPosition32(did)
                    drive.readMotorSpeed32(did)
                    drive.readMotorTorqueCurrent(did)
                    pos = getattr(drive, "motorPos32", 0)
                    spd = getattr(drive, "motorSpeed32", 0)
                    trq = getattr(drive, "motorTorqueCurrent", 0)
                self.activeWorker.emit(self.axis, int(pos), int(spd), int(trq))
            except Exception as e:
                print(f"\nDMM read error (axis {self.axis} on {portByAxis[self.axis] if self.axis < len(portByAxis) else '?'}): {e}", flush=True)
                break
            QThread.msleep(sampleTime)
        self.finishedWorker.emit(self.axis)

    @Slot()
    def stopLoop(self):
        self._active = False

class ParamReaderWorker(QObject):
    """
    One-shot read of static parameters for a given axis (ID and gains).
    Emits a dict payload for the UI to mirror into labels/spinboxes.
    """
    paramsReady = Signal(int, dict)   # axis, payload (dict)
    failed = Signal(int, str)
    finished = Signal(int)

    def __init__(self, axis: int, parent=None):
        super().__init__(parent)
        self.axis = axis

    @Slot()
    def run(self):
        global broadcastID
        try:
            drive = availableDrives[self.axis]
            with serial_lock:
                did = broadcastID
                drive.readDriveID(did)
                drive.readDriveStatus(did)
                drive.readMainGain(did)
                drive.readDerivGain(did)
                drive.readIntGain(did)
                drive.readGearNumber(did)
                payload = {
                    "driverIDNumber": getattr(drive, "driverIDNumber", None),
                    "driverMainGain": getattr(drive, "driverMainGain", None),
                    "driverSpeedGain": getattr(drive, "driverSpeedGain", None),
                    "driverIntGain": getattr(drive, "driverIntGain", None),
                    "driverGearNumber": getattr(drive, "driverGearNumber", None),
                    "driverStatus": getattr(drive, "driverStatus", None),
                }
            self.paramsReady.emit(self.axis, payload)
        except Exception as e:
            self.failed.emit(self.axis, str(e))
        finally:
            self.finished.emit(self.axis)

class TuningTab(QWidget):
    """
    Main GUI: connects to drives, shows live data, reads/writes parameters,
    and provides simple motion commands for the selected axis.
    """
    def __init__(self, axis: int = 0, parent=None):
        super().__init__(parent)
        self.axis = axis
        self.setWindowTitle("DMM: Demonstration GUI")
        self.boxBay = QGroupBox("Drive Bay")
        self.boxParams = QGroupBox("Drive Parameters")
        self.boxControl = QGroupBox("Control Panel")
        self._value_labels: list[QLabel] = []               # tiles showing values
        self._spinboxes: list[QSpinBox] = []                # inputs (QSpinBox)

        # Live worker (only one active per selected axis)
        self.workerThread: QThread | None = None
        self.worker: DMMWorker | None = None
        self.activeAxis: int | None = None

        # Parameter reader (one at a time; replaced on each click)
        self.paramThread: QThread | None = None
        self.paramWorker: ParamReaderWorker | None = None
        self.paramAxis: int = -1
        
        self.lastLive = {}                                  # Cache of last live values per axis
        self._gear_scale = 1                                # Track detected scaling for "gear number"
        self.setup_ui()
        self.uiTimer = QTimer(self)
        self.uiTimer.setInterval(150)
        self.uiTimer.timeout.connect(self._tickRefreshCurrentAxis)
        self.uiTimer.start()
        self.apply_theme_styles()                           # Initial theme application
        self.populateDriveBay()

    def _is_dark_palette(self) -> bool:
        pal = self.palette()
        c = pal.window().color()
        lum = (0.299 * c.redF() + 0.587 * c.greenF() + 0.114 * c.blueF())   # luminance
        return lum < 0.5

    def apply_theme_styles(self):
        """
        Used for dynamic UI styling. Computes light/dark colors from current palette and re-styles inputs/tiles.
        """
        dark = self._is_dark_palette()
        # SpinBoxes
        spin_fg = "white" if dark else "black"
        spin_bg = "rgba(255,255,255,0.10)" if dark else "rgba(0,0,0,0.05)"
        spin_border = "rgba(255,255,255,0.15)" if dark else "rgba(0,0,0,0.15)"
        spin_style = f"""
            QAbstractSpinBox {{
                background-color: {spin_bg};
                color: {spin_fg};
                border: 1px solid {spin_border};
                border-radius: 4px;
                padding: 4px 6px;
                font-size: 13px;
            }}
        """
        for sb in self._spinboxes:
            sb.setStyleSheet(spin_style)
        val_fg = "white" if dark else "black"               # Value tiles
        val_bg = "rgba(255,255,255,0.10)" if dark else "rgba(0,0,0,0.05)"
        val_border = "rgba(255,255,255,0.15)" if dark else "rgba(0,0,0,0.15)"
        val_style = f"""
            QLabel {{
                font-size: 14px;
                color: {val_fg};
                background-color: {val_bg};
                border: 1px solid {val_border};
                border-radius: 6px;
                padding: 4px 6px;
                min-height: 22px;
            }}
        """
        for lab in self._value_labels:
            lab.setStyleSheet(val_style)
        self.col2.setStyleSheet("""                         
            QWidget#col2 {
                border: 1px solid rgba(255,255,255,0.25);
                border-radius: 8px;
            }
        """)

    def changeEvent(self, ev):
        """
        Reacts to (Windows-safe) palette/theme changes without a global event filter.
        """
        if ev.type() in (QEvent.PaletteChange, QEvent.ApplicationPaletteChange):
            self.apply_theme_styles()
        super().changeEvent(ev)

    def setup_ui(self):
        ## BOX 1 - Drive Bay/List
        self.boxBay.setLayout(QVBoxLayout())
        self.connectedList = QListWidget()
        self.connectedList.setAlternatingRowColors(True)
        self.connectedList.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.connectedList.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.connectedList.setSelectionMode(QListWidget.SingleSelection)
        self.connectedList.itemClicked.connect(self.selectAxis)
        self.connectedList.currentRowChanged.connect(self.loadAxis)
        self.connectedList.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.connectedList.setStyleSheet("""
            QListWidget { font-size: 16px; }
            QListWidget::item { padding: 12px 8px; }
            QListWidget::item:selected { background-color: #0078d7; color: white; }
        """)
        self.boxBay.layout().addWidget(self.connectedList)
        btnRow = QHBoxLayout()                                          # 'Connect' and 'Disconnect' buttons
        self.connectBtn = QPushButton("Connect")
        self.connectBtn.clicked.connect(self.connectDrives)
        self.disconnectBtn = QPushButton("Disconnect")
        self.disconnectBtn.clicked.connect(self.disconnectDrives)
        self.disconnectBtn.setEnabled(False)
        self.connectBtn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.disconnectBtn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        btnRow.addWidget(self.connectBtn)
        btnRow.addWidget(self.disconnectBtn)
        btnRow.setSpacing(8)
        self.boxBay.layout().addLayout(btnRow)

        ## BOX 2 - Drive Parameters
        self.boxParams.setLayout(QVBoxLayout())
        refreshRow = QHBoxLayout()
        self.paramHeader = QLabel("No drive selected.")
        self.paramHeader.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        self.paramStatus = QLabel("Status: —")
        self.paramStatus.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        self.paramStatus.setStyleSheet("color:#aaa; font-size: 12px; margin-left: 12px;")
        self.refreshBtn = QPushButton("Refresh")
        self.refreshBtn.setFixedWidth(100)
        self.refreshBtn.clicked.connect(self.paramRefresh)
        self.refreshBtn.setEnabled(False)
        refreshRow.addWidget(self.paramHeader)
        refreshRow.addWidget(self.paramStatus)
        refreshRow.addStretch(1)
        refreshRow.addWidget(self.refreshBtn)
        self.boxParams.layout().addLayout(refreshRow)
        self.paramGrid = QGridLayout()
        self.paramGrid.setHorizontalSpacing(12)
        self.paramGrid.setVerticalSpacing(8)
        self.boxParams.layout().addLayout(self.paramGrid)
        def makeTiles(label_text: str):
            wrappedTile = QWidget()
            tile = QVBoxLayout(wrappedTile)
            tile.setContentsMargins(0, 0, 0, 0)
            tile.setSpacing(4)
            label = QLabel(label_text)
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("color: #bbb; font-size: 12px;")
            value = QLabel("—")
            value.setAlignment(Qt.AlignCenter)
            value.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
            value.setMinimumWidth(50)
            tile.addWidget(label)
            tile.addWidget(value)
            self._value_labels.append(value)
            return wrappedTile, value
        p_id,   self.drive_id   = makeTiles("Drive ID number")          # Left column tiles
        p_kp,   self.drive_kp   = makeTiles("Main gain (Kp)")
        p_kd,   self.drive_kd   = makeTiles("Speed gain (Kd)")
        p_ki,   self.drive_ki   = makeTiles("Integration gain (Ki)")
        p_pos,  self.drive_pos  = makeTiles("Position")                 # Right column tiles
        p_spd,  self.drive_spd  = makeTiles("Speed (RPM)")
        p_trq,  self.drive_trq  = makeTiles("Scaled Torque")
        p_gear, self.drive_gear = makeTiles("Gear number")
        self.paramGrid.addWidget(p_id,   0, 0)
        self.paramGrid.addWidget(p_kp,   1, 0)
        self.paramGrid.addWidget(p_kd,   2, 0)
        self.paramGrid.addWidget(p_ki,   3, 0)
        self.paramGrid.addWidget(p_pos,  0, 1)
        self.paramGrid.addWidget(p_spd,  1, 1)
        self.paramGrid.addWidget(p_trq,  2, 1)
        self.paramGrid.addWidget(p_gear, 3, 1)
        self.paramGrid.setColumnStretch(0, 1)
        self.paramGrid.setColumnStretch(1, 1)

        ## BOX 3 - Control Panel
        self.boxControl.setLayout(QVBoxLayout())
        ctrlPanel = QWidget()
        ctrlPanel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.MinimumExpanding)
        ctrlPanelLayout = QHBoxLayout(ctrlPanel)
        ctrlPanelLayout.setSpacing(12)
        ctrlPanelLayout.setContentsMargins(6, 6, 6, 6)
        ctrlPanelLayout.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        def makeCols(pairs):
            wrappedCols = QWidget()
            col = QVBoxLayout(wrappedCols)
            col.setContentsMargins(10, 10, 10, 10)
            col.setSpacing(8)
            col.setAlignment(Qt.AlignTop)
            form = QFormLayout()
            form.setHorizontalSpacing(8)
            form.setVerticalSpacing(6)
            form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
            form.setFormAlignment(Qt.AlignHCenter | Qt.AlignTop)
            form.setRowWrapPolicy(QFormLayout.DontWrapRows)
            form.setFieldGrowthPolicy(QFormLayout.ExpandingFieldsGrow)
            for text, w in pairs:
                label = QLabel(text)
                label.setStyleSheet("color:#aaa; font-size: 12px;")
                w.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
                w.setMinimumWidth(70)
                form.addRow(label, w)
            col.addLayout(form)
            return wrappedCols, form
        self.sb_id = QSpinBox();        self.sb_id.setRange(1, 127)     # Inputs + ranges
        self.sb_kp = QSpinBox();        self.sb_kp.setRange(1, 127)
        self.sb_kd = QSpinBox();        self.sb_kd.setRange(1, 127)
        self.sb_ki = QSpinBox();        self.sb_ki.setRange(1, 127)  
        self.sb_pos = QDoubleSpinBox(); self.sb_pos.setDecimals(0); self.sb_pos.setRange(-2000000, 2000000); self.sb_pos.setSingleStep(100)
        self.sb_spd = QSpinBox();       self.sb_spd.setRange(-3000, 3000); self.sb_spd.setSuffix(" RPM")
        self.sb_gear = QSpinBox();      self.sb_gear.setRange(500, 16384)
        for s in (self.sb_id, self.sb_kp, self.sb_kd, self.sb_ki, self.sb_pos, self.sb_spd, self.sb_gear):
            s.setButtonSymbols(QSpinBox.NoButtons)
            s.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
            s.setMinimumWidth(60)
            self._spinboxes.append(s)
        col1 = QWidget()                                                # Col1: enable/disable buttons
        col1v = QVBoxLayout(col1)
        col1v.setContentsMargins(0, 0, 0, 0)
        col1v.setSpacing(6)
        self.btnEnableDrive = QPushButton("Enable Drive")
        self.btnDisableDrive = QPushButton("Disable Drive")
        self.btnEnableDrive.setMinimumWidth(120)
        self.btnDisableDrive.setMinimumWidth(120)
        self.btnEnableDrive.clicked.connect(self.enableDriveClicked)
        self.btnDisableDrive.clicked.connect(self.disableDriveClicked)
        col1v.addWidget(self.btnEnableDrive, 0, Qt.AlignTop)
        col1v.addWidget(self.btnDisableDrive, 0, Qt.AlignTop)
        col1v.addStretch(1)
        self.col2, form2  = makeCols([("Position", self.sb_pos), ("Speed (RPM)", self.sb_spd)])     # Col2: Pos/speed controls
        self.col2.setObjectName("col2")                                 # for styling outline in apply_theme_styles()
        col3, _ = makeCols([("Main gain (Kp)", self.sb_kp),             # Col3: Gains
                            ("Speed gain (Kd)", self.sb_kd),
                            ("Integration (Ki)", self.sb_ki)])
        col4, _ = makeCols([("Drive ID", self.sb_id),                   # Col4: Drive ID + Gear number
                            ("Gear number", self.sb_gear)])
        self.btnSetPosition = QPushButton("Move to Position")
        self.btnSetPosition.setEnabled(False)
        self.btnSetPosition.setMinimumWidth(120)
        self.btnSetPosition.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.btnSetPosition.clicked.connect(self.setPositionClicked)
        btn_wrapper = QWidget()
        btn_layout = QHBoxLayout(btn_wrapper)
        btn_layout.setContentsMargins(0, 0, 0, 0)
        btn_layout.setSpacing(0)
        btn_layout.addStretch(1)
        btn_layout.addWidget(self.btnSetPosition, 0, Qt.AlignHCenter)
        btn_layout.addStretch(1)
        form2.insertRow(1, btn_wrapper)
        motionRow = QHBoxLayout()                                       # Start/Stop Motion buttons
        self.btnStartMotion = QPushButton("Start Motion")
        self.btnStopMotion  = QPushButton("Stop Motion")
        self.btnStartMotion.setEnabled(False)
        self.btnStopMotion.setEnabled(False)
        self.btnStartMotion.setMinimumWidth(100)
        self.btnStopMotion.setMinimumWidth(100)
        self.btnStartMotion.clicked.connect(self.startMotionClicked)
        self.btnStopMotion.clicked.connect(self.stopMotionClicked)
        motionRow.addWidget(self.btnStartMotion)
        motionRow.addWidget(self.btnStopMotion)
        self.col2.layout().addLayout(motionRow)
        imgPath = Path(__file__).parent / "Assets" / "logo.png"         # Footer (logo + Reset + Update)
        self.logoLabel = QLabel()
        pixmap = QPixmap(str(imgPath))
        if not pixmap.isNull():
            self.logoLabel.setPixmap(pixmap.scaledToWidth(100, Qt.SmoothTransformation))
        self.logoLabel.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        self.logoLabel.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.updateBtn = QPushButton("Update")                          # Update button (bottom-right)
        self.updateBtn.setMinimumWidth(120)
        self.updateBtn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.updateBtn.clicked.connect(self.updateClicked)
        self.updateBtn.setStyleSheet("""
            QPushButton {
                background-color: #0078d7;
                color: white;
                font-weight: bold;
                padding: 6px 10px;
                border-radius: 6px;
            }
            QPushButton:hover { background-color: #1491ff; }
            QPushButton:disabled { background-color:#555; color:#999; }
        """)
        self.resetBtn = QPushButton("Reset")                            # Reset button
        self.resetBtn.setMinimumWidth(120)
        self.resetBtn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.resetBtn.clicked.connect(self.resetClicked)
        overlayCont = QWidget()
        overlayLayout = QGridLayout(overlayCont)
        overlayLayout.setContentsMargins(0, 0, 0, 0)
        overlayLayout.setSpacing(0)
        ctrlPanelLayout.addWidget(col1)                                 # layer the widgets in control panel
        ctrlPanelLayout.addWidget(self.col2)
        ctrlPanelLayout.addWidget(col3)
        ctrlPanelLayout.addWidget(col4)
        ctrlPanelLayout.setStretch(0, 1)
        ctrlPanelLayout.setStretch(1, 1)
        ctrlPanelLayout.setStretch(2, 1)
        ctrlPanelLayout.setStretch(3, 1)
        overlayLayout.addWidget(ctrlPanel, 0, 0)
        bottomBar = QHBoxLayout()
        bottomBar.setContentsMargins(8, 0, 8, 8)                        # left, top, right, bottom
        bottomBar.setSpacing(12)
        bottomBar.addWidget(self.logoLabel, 0, Qt.AlignLeft | Qt.AlignBottom)
        bottomBar.addStretch(1)
        rightStack = QVBoxLayout()
        rightStack.setContentsMargins(0, 0, 0, 0)
        rightStack.setSpacing(6)                                        # gap between Reset and Update
        rightStack.addStretch(1)
        rightStack.addWidget(self.resetBtn, 0, Qt.AlignRight | Qt.AlignBottom)
        rightStack.addWidget(self.updateBtn, 0, Qt.AlignRight | Qt.AlignBottom)
        bottomBar.addLayout(rightStack)
        overlayLayout.addLayout(bottomBar, 0, 0, Qt.AlignBottom)
        self.boxControl.layout().addWidget(overlayCont, 1)
        topLayout = QHBoxLayout()                                       # Add widgets for overall layout
        topLayout.addWidget(self.boxBay)
        topLayout.addWidget(self.boxParams)
        topLayout.setStretch(0, 1)
        topLayout.setStretch(1, 2)
        mainLayout = QVBoxLayout()
        mainLayout.addLayout(topLayout)
        mainLayout.addWidget(self.boxControl)
        mainLayout.setStretch(0, 1)
        mainLayout.setStretch(1, 1)
        self.setLayout(mainLayout)

    def _stopWorker(self):
        """
        Stops the currently running live worker (if any) and clears pointers.
        """
        try:
            if self.workerThread is not None:
                if self.worker is not None:
                    self.worker.stopLoop()
                self.workerThread.quit()
                self.workerThread.wait(1000)
        except Exception:
            pass
        self.workerThread = None
        self.worker = None
        self.activeAxis = None

    def _startWorkerForAxis(self, axis: int):
        """
        Creates and starts a live worker/QThread ONLY for the given axis.
        Stops an existing worker if it targets a different axis.
        """
        if axis < 0 or axis >= len(availableDrives):
            return
        if self.activeAxis is not None and self.activeAxis != axis:         # Stop active worker for another axis
            self._stopWorker()
        if self.activeAxis == axis and self.workerThread is not None:       # Keeps active worker if current axis
            if self.workerThread.isRunning():
                return
        thread = QThread(self)                      # (Re)create and start
        worker = DMMWorker(axis)
        worker.moveToThread(thread)
        thread.started.connect(worker.startLoop)
        worker.finishedWorker.connect(self._onWorkerFinished)
        worker.finishedWorker.connect(thread.quit)
        worker.activeWorker.connect(self._onWorkerActive)
        thread.finished.connect(worker.deleteLater)
        self.workerThread = thread
        self.worker = worker
        self.lastLive.setdefault(axis, {"pos": None, "spd": None, "trq": None})
        self.activeAxis = axis
        thread.start()
        print(f"Started worker for axis {axis} on {portByAxis[axis] if axis < len(portByAxis) else '?'}", flush=True)

    @Slot(int)
    def _onWorkerFinished(self, axis: int):
        """
        DEBUGGING: Notifies that the background loop ended.
        """
        print(f"Worker finished for axis {axis}", flush=True)

    @Slot(int, int, int, int)
    def _onWorkerActive(self, axis: int, pos: int, spd: int, trq: int):
        """
        Receives and caches live values, and refreshes on-screen if selected.
        """
        self.lastLive.setdefault(axis, {"pos": None, "spd": None, "trq": None})
        self.lastLive[axis]["pos"] = pos
        self.lastLive[axis]["spd"] = spd
        self.lastLive[axis]["trq"] = trq
        if self.connectedList.currentRow() == axis:
            self.drive_pos.setText(str(pos))
            self.drive_spd.setText(str(spd))
            self.drive_trq.setText(str(trq))

    def _stopAllWorkers(self):
        """
        Stops the live worker (if any) and clears the live cache.
        """
        self._stopWorker()
        self.lastLive.clear()

    def _stopParamReader(self):
        """
        Stops the param reader thread (if any).
        """
        if self.paramThread:
            try:
                self.paramThread.quit()
                self.paramThread.wait(1000)
            except Exception:
                pass
        self.paramThread = None
        self.paramWorker = None
        self.paramAxis = -1

    def _startParamReader(self, axis: int):
        """
        Starts a parameter read for the specified axis.
        """
        self._stopParamReader()
        self.paramThread = QThread(self)
        self.paramWorker = ParamReaderWorker(axis)
        self.paramAxis = axis
        self.paramWorker.moveToThread(self.paramThread)
        self.paramThread.started.connect(self.paramWorker.run)
        self.paramWorker.paramsReady.connect(self._onParamsReady)
        self.paramWorker.failed.connect(self._onParamsFailed)
        self.paramThread.finished.connect(self.paramWorker.deleteLater)
        self.paramThread.start()

    @Slot(int, dict)
    def _onParamsReady(self, axis: int, payload: dict):
        """
        Mirrors read parameters into tiles/spinboxes and enables controls.
        """
        if self.connectedList.currentRow() != axis:
            self._updateDriveBayLabel(axis)
            return
        self.paramHeader.setText(f"Showing parameters for drive {axis + 1}")        # Update header/tiles

        status_raw = payload.get("driverStatus", None)                              # Display driver status in parameter box
        if status_raw is not None:
            try:
                status_text = DmmDriver.getStatus(int(status_raw))
            except Exception:
                status_text = str(status_raw)
            self.paramStatus.setText(f"Status: {status_text}")
        else:
            self.paramStatus.setText("Status: —")

        self.drive_id.setText(str(payload.get("driverIDNumber", "")))
        self.drive_kp.setText(str(payload.get("driverMainGain", "")))
        self.drive_kd.setText(str(payload.get("driverSpeedGain", "")))
        self.drive_ki.setText(str(payload.get("driverIntGain", "")))        
        gear_raw = payload.get("driverGearNumber", None)                            # Gear scaling for legacy gear
        if isinstance(gear_raw, int):
            gear_display, scale = self._detectGearScaling(gear_raw)
            self._gear_scale = scale
            self.drive_gear.setText(str(gear_display))
            self._safeSet(self.sb_gear, gear_display)
        else:
            self._gear_scale = 1
            self.drive_gear.setText("" if gear_raw is None else str(gear_raw))

        # Mirror spinboxes
        self._safeSet(self.sb_id, payload.get("driverIDNumber", None))
        self._safeSet(self.sb_kp, payload.get("driverMainGain", None))
        self._safeSet(self.sb_kd, payload.get("driverSpeedGain", None))
        self._safeSet(self.sb_ki, payload.get("driverIntGain", None))

        # Enable controls
        self.refreshBtn.setEnabled(True)
        self.updateBtn.setEnabled(True)
        self.btnStartMotion.setEnabled(True)
        self.btnStopMotion.setEnabled(True)
        self.btnSetPosition.setEnabled(True)

        # Keep list row text current
        self._updateDriveBayLabel(axis)

    @Slot(int, str)
    def _onParamsFailed(self, axis: int, err: str):
        """
        Shows a warning if a static read failed for the current axis.
        """
        if self.connectedList.currentRow() == axis:
            QMessageBox.warning(self, "Read Error", err)

    def _clearParams(self):
        """
        Clears parameter tiles and disables action buttons.
        """
        for w in (self.drive_id, self.drive_kp, self.drive_kd, self.drive_ki,
                  self.drive_pos, self.drive_spd, self.drive_trq, self.drive_gear):
            w.setText("")
        self.paramStatus.setText("Status: —")
        self.refreshBtn.setEnabled(False)
        self.updateBtn.setEnabled(False)
        self.btnStartMotion.setEnabled(False)
        self.btnStopMotion.setEnabled(False)
        self.btnSetPosition.setEnabled(False)
        self._gear_scale = 1

    def _detectGearScaling(self, raw_value):
        """
        If gear is a legacy ×5 encoded value (5*16384 and divisible by 5),
        return unscaled display value and scale = 5; else pass-through with scale=1.
        """
        scale = 1
        display_val = raw_value
        try:
            if isinstance(raw_value, int):
                if raw_value > 16384 and (raw_value % 5 == 0) and (raw_value // 5) <= 16384:
                    scale = 5
                    display_val = raw_value // 5
        except Exception:
            pass
        return display_val, scale

    def _safeSet(self, sb, value):
        """
        Helper: Sets a spinbox within bounds only if the value is int-like and not None.
        """
        try:
            if value is None:
                return
            v = int(value)
            sb.setValue(max(sb.minimum(), min(sb.maximum(), v)))
        except Exception:
            pass

    def _tickRefreshCurrentAxis(self):
        """
        UI timer: keeps on-screen live tiles fresh with the cached live values.
        """
        axis = self.connectedList.currentRow()
        if axis is None or axis < 0:
            return
        if axis in self.lastLive:
            lv = self.lastLive[axis]
            if lv.get("pos") is not None:
                self.drive_pos.setText(str(lv["pos"]))
            if lv.get("spd") is not None:
                self.drive_spd.setText(str(lv["spd"]))
            if lv.get("trq") is not None:
                self.drive_trq.setText(str(lv["trq"]))

    def paramRefresh(self):
        """
        Refresh button: re-read the current axis parameters and update UI.
        """
        row = self.connectedList.currentRow()
        if row is None or row < 0 or not self.connectedList.isEnabled():
            QMessageBox.information(self, "No Drive Selected", "Please select a drive first.")
            return
        self._startParamReader(row)  # non-blocking fresh read

    def _driveBayLabel(self, row: int) -> str:
        """
        What is displayed in the drive bay list.
        Format: 'Axis N - ID X (/dev/ttyUSBx or COMx)'
        """
        try:
            d = availableDrives[row]
            did = getattr(d, "driverIDNumber", "?")
            port = portByAxis[row] if row < len(portByAxis) else "N/A"
            return f"Axis {row + 1} - ID {did} ({port})"
        except Exception:
            return "Unknown device"

    def _updateDriveBayLabel(self, row: int):
        if 0 <= row < self.connectedList.count():
            self.connectedList.item(row).setText(self._driveBayLabel(row))

    def populateDriveBay(self):
        """
        Fills the drive list based on current connections and primes axis 0.
        """
        self.connectedList.clear()
        if not availableDrives:
            self.connectedList.addItem("No drives available")
            self.connectedList.setEnabled(False)
            self.paramHeader.setText("No drive selected.")
            self.paramStatus.setText("Status: —")
            self.disconnectBtn.setEnabled(False)
            self.connectBtn.setEnabled(True)
            self._clearParams()
            return
        self.connectedList.setEnabled(True)
        for idx in range(len(availableDrives)):
            self.connectedList.addItem(self._driveBayLabel(idx))
        self.paramHeader.setText("Select a drive to view parameters.")
        self.paramStatus.setText("Status: —")
        self.disconnectBtn.setEnabled(True)
        self.connectBtn.setEnabled(False)
        self.connectedList.setCurrentRow(0)             # Defaults axis 0 and start that worker.
        self._startWorkerForAxis(0)
        self._startParamReader(0)

    def selectAxis(self, item):
        """
        When option in list is clicked -> load that axis.
        """
        row = self.connectedList.row(item)
        self.loadAxis(row)

    def loadAxis(self, row: int):
        """
        Selects the axis: starts up the live worker + param reader for that axis,
        then seeds the live tile values if a cache exists.
        """
        if row is None or row < 0 or row >= len(availableDrives):
            self.paramHeader.setText("No drive selected.")
            self.paramStatus.setText("Status: —")
            self._clearParams()
            self._stopWorker()
            return
        self._startWorkerForAxis(row)           # Switch worker to this axis
        self._startParamReader(row)
        lv = self.lastLive.get(row, {})
        self.drive_pos.setText("—" if lv.get("pos") is None else str(lv["pos"]))
        self.drive_spd.setText("—" if lv.get("spd") is None else str(lv["spd"]))
        self.drive_trq.setText("—" if lv.get("trq") is None else str(lv["trq"]))

    def selectedDrive(self):
        """
        Returns (drive, row) for the currently selected list row, or (None, -1).
        """
        row = self.connectedList.currentRow()
        if row is None or row < 0 or row >= len(availableDrives):
            return None, -1
        return availableDrives[row], row

    def startMotionClicked(self):
        """
        If Speed != 0 -> run const speed; else move absolute to Position.
        """
        drive, _ = self.selectedDrive()
        if drive is None:
            QMessageBox.information(self, "No Drive Selected", "Please select a drive first.")
            return
        self.btnStartMotion.setEnabled(False)
        errors = []
        try:
            with serial_lock:
                did = broadcastID
                spd = int(self.sb_spd.value())
                try:
                    if spd != 0:
                        drive.turnConstSpeed(did, spd)
                    else:
                        drive.moveAbs32(did, int(self.sb_pos.value()))
                except Exception as e:
                    errors.append(f"motion command: {e}")
            if errors:
                QMessageBox.warning(self, "Start Motion (Partial)", "Some actions failed:\n• " + "\n• ".join(errors))
        finally:
            self.btnStartMotion.setEnabled(True)

    def stopMotionClicked(self):
        """
        Stops motion by commanding const speed 0.
        """
        drive, _ = self.selectedDrive()
        if drive is None:
            QMessageBox.information(self, "No Drive Selected", "Please select a drive first.")
            return
        self.btnStopMotion.setEnabled(False)
        errors = []
        try:
            with serial_lock:
                did = broadcastID
                try:
                    drive.turnConstSpeed(did, 0)
                except Exception as e:
                    errors.append(f"driveDisable: {e}")
            if errors:
                QMessageBox.warning(self, "Stop Motion (Partial)", "Some actions failed:\n• " + "\n• ".join(errors))
        finally:
            self.btnStopMotion.setEnabled(True)

    def setPositionClicked(self):
        """
        Absolute move to desired Position value.
        """
        drive, _ = self.selectedDrive()
        if drive is None:
            QMessageBox.information(self, "No Drive Selected", "Please select a drive first.")
            return
        self.btnSetPosition.setEnabled(False)
        try:
            with serial_lock:
                did = broadcastID
                drive.moveAbs32(did, int(self.sb_pos.value()))
        finally:
            self.btnSetPosition.setEnabled(True)

    def updateClicked(self):
        """
        Writes ID/gains/gear; then re-reads fresh values to update tiles/spinboxes.
        """
        drive, row = self.selectedDrive()
        if drive is None:
            QMessageBox.information(self, "No Drive Selected", "Please select a drive first.")
            return
        new_id   = self.sb_id.value()
        new_kp   = self.sb_kp.value()
        new_kd   = self.sb_kd.value()
        new_ki   = self.sb_ki.value()
        new_gear = self.sb_gear.value()
        self.updateBtn.setEnabled(False)
        errors = []
        try:
            with serial_lock:
                did_for_write = broadcastID                                     # keep stable during write pass
                try:
                    drive.setDriveIDNum(did_for_write, int(new_id))
                    setattr(drive, "driverIDNumber", int(new_id))
                except Exception as e:
                    errors.append(f"setDriveIDNum: {e}")
                try: drive.setMainGain(did_for_write, int(new_kp))
                except Exception as e: errors.append(f"setMainGain: {e}")
                try: drive.setSpeedGain(did_for_write, int(new_kd))
                except Exception as e: errors.append(f"setSpeedGain: {e}")
                try: drive.setIntGain(did_for_write, int(new_ki))
                except Exception as e: errors.append(f"setIntGain: {e}")
                try: drive.setGearNumber(did_for_write, int(new_gear))
                except Exception as e: errors.append(f"setGearNumber: {e}")
            self._updateDriveBayLabel(row)                                      # Update the list row
            QTimer.singleShot(100, lambda r=row: self._startParamReader(r))     # Re-read fresh parameters using broadcastID
            if errors:
                QMessageBox.warning(self, "Partial Update", "Some parameters failed to write:\n• " + "\n• ".join(errors))
            else:
                QMessageBox.information(self, "Updated", "Parameters written and refreshed.")
        finally:
            self.updateBtn.setEnabled(True)

    def resetClicked(self):
        """
        Sends driveReset(broadcastID), then schedules a param refresh.
        """
        drive, row = self.selectedDrive()
        if drive is None:
            QMessageBox.information(self, "No Drive Selected", "Please select a drive first.")
            return
        self.resetBtn.setEnabled(False)
        try:
            with serial_lock:
                drive.driveReset(broadcastID)
        except Exception as e:
            QMessageBox.warning(self, "Reset Failed", str(e))
        finally:
            self.resetBtn.setEnabled(True)
        QTimer.singleShot(150, lambda r=row: self._startParamReader(r))

    def enableDriveClicked(self):
        """
        Enables the selected drive via broadcastID.
        """
        drive, _ = self.selectedDrive()
        if drive is None:
            QMessageBox.information(self, "No Drive Selected", "Please select a drive first.")
            return
        self.btnEnableDrive.setEnabled(False)
        try:
            with serial_lock:
                drive.driveEnable(broadcastID)
        except Exception as e:
            QMessageBox.warning(self, "Enable Drive Failed", str(e))
        finally:
            self.btnEnableDrive.setEnabled(True)

    def disableDriveClicked(self):
        """
        Disables the selected drive via broadcastID.
        """
        drive, _ = self.selectedDrive()
        if drive is None:
            QMessageBox.information(self, "No Drive Selected", "Please select a drive first.")
            return
        self.btnDisableDrive.setEnabled(False)
        try:
            with serial_lock:
                drive.driveDisable(broadcastID)
        except Exception as e:
            QMessageBox.warning(self, "Disable Drive Failed", str(e))
        finally:
            self.btnDisableDrive.setEnabled(True)

    def connectDrives(self):
        """
        Scans ports, connects USB serial DMM drives, populates the list,
        and starts live/param workers for the selected axis.
        """
        global availableDrives, broadcastID, portByAxis
        if availableDrives:
            self.disconnectDrives()
        all_ports = list(serial.tools.list_ports.comports())                    # Filter to USB serial ports only (and COM on Windows)
        candidate_ports = [p for p in all_ports if _isUSBserialPort(p)]
        for p in candidate_ports:
            try:
                drive = DmmDriver(p.device)
                with serial_lock:
                    drive.readDriveID(broadcastID)                              # single-device-per-port: broadcast read is OK
                availableDrives.append(drive)
                print(f"Connected axis {len(availableDrives)-1} -> {p.device} (ID {getattr(drive,'driverIDNumber','?')})", flush=True)
            except Exception:
                try:
                    if 'drive' in locals() and getattr(drive, "serialConnection", None):
                        drive.serialConnection.close()
                except Exception:
                    pass
                continue
        portByAxis[:] = []
        for d in availableDrives:
            prt = getattr(getattr(d, "serialConnection", None), "port", "N/A")
            portByAxis.append(prt)
        self.lastLive.clear()
        self.populateDriveBay()
        if not availableDrives:
            QMessageBox.information(self, "No Drives Found", "No USB DMM drive detected.")

    def disconnectDrives(self):
        """
        Stops workers, closes all serial ports, and resets the UI.
        """
        global availableDrives, portByAxis
        self._stopParamReader()
        self._stopAllWorkers()
        for drive in availableDrives:
            try:
                if getattr(drive, "serialConnection", None) and drive.serialConnection.is_open:
                    drive.serialConnection.close()
            except Exception:
                pass
        availableDrives.clear()
        portByAxis.clear()
        self.connectedList.clear()
        self.connectedList.addItem("No drives available")
        self.connectedList.setEnabled(False)
        self.paramHeader.setText("No drive selected.")
        self.paramStatus.setText("Status: —")
        self._clearParams()
        self.disconnectBtn.setEnabled(False)
        self.connectBtn.setEnabled(True)

    def closeEvent(self, event):
        """
        Ensures threads are stopped and ports closed on program close.
        """
        try:
            self.disconnectDrives()
        except Exception:
            pass
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = TuningTab()
    win.show()
    sys.exit(app.exec())