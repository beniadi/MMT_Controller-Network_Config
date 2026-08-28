# motor.py
import sys, socket, re, time, math, threading
from typing import Optional, Dict, Tuple, List

from PyQt5.QtCore import QTimer, Qt, pyqtSlot, QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QMessageBox, QDialog, QDialogButtonBox,
    QFormLayout, QLineEdit, QSpinBox, QDoubleSpinBox, QGroupBox, QStatusBar,
    QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView
)

DEFAULT_TCP_PORT = 5001
DEFAULT_CTRL1_IP = "192.168.0.128"
DEFAULT_CTRL2_IP = "192.168.0.129"

STEP_PER_MM = 51200
DEFAULT_SPEED_MM_S = 15.0

# --- Tilt axis conversion (deg <-> motor steps) ---
TILT_ROT_CENTER_MM = 50.0   # R = 50 mm
TILT_MAX_DEG = 5.4          # UI clamp / safety

# (name, ctrl, port, unit)
AXES = [
    ("X",   1, 1, "mm"),
    ("Y",   1, 2, "mm"),
    ("Z",   1, 3, "mm"),
    ("CAM", 1, 4, "mm"),
    ("Tx",  2, 1, "deg"),
    ("Ty",  2, 2, "deg"),
    ("Tz",  2, 3, "deg"),
    ("Lens_Loader", 2, 4, "mm"),
]


# Axis configuration (motor model + limits + future geometry placeholders)
# t0        : origin
# Tmn/Tmx   : moving range (documentation)
# tmn/tmx   : allowed moving range (enforced clamp)
# polarity  : +1 or -1 (sign convention)
# alfa,beta,gamma : pose (future MAPS sensor use)
# tx0_lab, ty0_lab, tz0_lab : axis position in lab frame (future use)
AXIS_CFG: Dict[Tuple[int, int], Dict[str, float]] = {}

for _name, ctrl, port, unit in AXES:
    key = (int(ctrl), int(port))

    if unit == "deg":
        tmn, tmx = -TILT_MAX_DEG, TILT_MAX_DEG
        Tmn, Tmx = tmn, tmx
        t0 = 0.0
    else:
        tmn, tmx = -1000.0, 1000.0
        Tmn, Tmx = tmn, tmx
        t0 = 0.0

    AXIS_CFG[key] = {
        "t0": float(t0),

        "Tmn": float(Tmn),
        "Tmx": float(Tmx),

        "tmn": float(tmn),
        "tmx": float(tmx),

        "polarity": 1.0,   # set -1.0 if an axis is inverted

        "alfa": 0.0,
        "beta": 0.0,
        "gamma": 0.0,

        "tx0_lab": 0.0,
        "ty0_lab": 0.0,
        "tz0_lab": 0.0,
    }


class MotorTCPClient:
    def __init__(self):
        self.sock: Optional[socket.socket] = None
        self.ip, self.port = "", DEFAULT_TCP_PORT
        self._dt_send = 0.002
        self._dt_recv = 0.003
        self._lock = threading.Lock()

    def is_connected(self) -> bool:
        return isinstance(self.sock, socket.socket)

    def connect(self, ip: str, port: int, timeout_s: float = 0.8):
        self.disconnect()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(float(timeout_s))
        s.connect((ip, int(port)))
        self.sock = s
        self.ip, self.port = ip, int(port)

    def disconnect(self):
        if self.is_connected():
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None

    def send(self, msg: str):
        with self._lock:
            if not self.is_connected():
                raise RuntimeError("Not connected.")
            self.sock.send(msg.encode("unicode_escape"))
            time.sleep(self._dt_send)

    def send_recv(self, msg: str, n: int = 1024) -> str:
        with self._lock:
            if not self.is_connected():
                raise RuntimeError("Not connected.")
            self.sock.send(msg.encode("unicode_escape"))
            time.sleep(self._dt_recv)
            try:
                data = self.sock.recv(int(n))
                return data.decode("unicode_escape", errors="ignore") if data else ""
            except socket.timeout:
                return ""
            except Exception:
                return ""


class LensLoaderHomeThread(QThread):
    """
    Worker thread for Lens_Loader (ctrl=2, port=4) homing.

    Sends the exact same command sequence as commu_motor.go_home():
        4ST0  →  power on
        4MA1  →  set absolute scale
        4HM1  →  go home (origin return)

    Then polls '@R' (commu_motor.check_motors) until port 4 reports 'R' (ready).
    Status reply is a plain comma-separated string: e.g. 'R,R,R,B'
    Port 4 status is at index 3 (0-based) after splitting by comma.
    """
    finished = pyqtSignal()       # homing completed successfully
    failed   = pyqtSignal(str)    # error message string
    progress = pyqtSignal(str)    # status text for status bar

    POLL_MS    = 500              # polling interval (ms)
    TIMEOUT_S  = 30.0             # maximum wait time (seconds)

    def __init__(self, client: "MotorTCPClient", parent=None):
        super().__init__(parent)
        self._client  = client
        self._running = True

    def stop(self):
        """Stop homing and send motor stop command."""
        self._running = False
        try:
            self._client.send("4S")
        except Exception:
            pass

    def _parse_port4_status(self, reply: str) -> str:
        """
        Parse '@R' reply to get port 4 status.
        Reply format: 'B,R,R,R' or 'R,R,R,R' etc. (comma-separated, 4 chars).
        Returns 'R', 'B', 'S', or '?' if unparseable.
        """
        try:
            if not reply:
                return "?"
            parts = reply.strip().split(",")
            if len(parts) >= 4:
                return parts[3].strip()   # index 3 = port 4
            return "?"
        except Exception:
            return "?"

    def run(self):
        import time
        try:
            # Step 1 — power on port 4
            self.progress.emit("Homing: powering on Lens_Loader (4ST0)...")
            self._client.send_recv("4ST0")

            if not self._running:
                return

            # Step 2 — set absolute scale
            self.progress.emit("Homing: setting absolute scale (4MA1)...")
            self._client.send_recv("4MA1")

            if not self._running:
                return

            # Step 3 — send home command
            self.progress.emit("Homing: sending home command (4HM1)...")
            self._client.send("4HM1")

            # Step 4 — poll '@R' until port 4 is ready
            t0 = time.time()
            while self._running:
                self.msleep(self.POLL_MS)

                elapsed = time.time() - t0
                if elapsed > self.TIMEOUT_S:
                    self.failed.emit(
                        f"Homing timeout after {self.TIMEOUT_S:.0f}s — "
                        "Lens_Loader did not reach home position.")
                    return

                try:
                    reply = self._client.send_recv("@R")
                except Exception as e:
                    self.failed.emit(f"Status poll error: {e}")
                    return

                status = self._parse_port4_status(reply)
                self.progress.emit(
                    f"Homing: Lens_Loader port 4 status = '{status}'  "
                    f"({elapsed:.1f}s elapsed)")

                if status == "R":
                    break

            if not self._running:
                self.progress.emit("Homing: stopped by user.")
                return

            self.progress.emit("Homing: Lens_Loader reached home position.")
            self.finished.emit()

        except Exception as e:
            self.failed.emit(f"Homing error: {e}")


class CAMHomeThread(QThread):
    """
    Worker thread for CAM motor (ctrl=1, port=4) homing.

    Sends the same command sequence as LensLoaderHomeThread:
        4ST0  →  power on
        4MA1  →  set absolute scale
        4HM1  →  go home (origin return)

    Then polls '@R' until port 4 reports 'R' (ready).
    Status reply is a comma-separated string: e.g. 'R,R,R,B'
    Port 4 status is at index 3 (0-based) after splitting by comma.
    """
    finished = pyqtSignal()       # homing completed successfully
    failed   = pyqtSignal(str)    # error message string
    progress = pyqtSignal(str)    # status text for status bar

    POLL_MS   = 500               # polling interval (ms)
    TIMEOUT_S = 30.0              # maximum wait time (seconds)

    def __init__(self, client: "MotorTCPClient", parent=None):
        super().__init__(parent)
        self._client  = client
        self._running = True

    def stop(self):
        """Stop homing and send motor stop command."""
        self._running = False
        try:
            self._client.send("4S")
        except Exception:
            pass

    def _parse_port4_status(self, reply: str) -> str:
        """
        Parse '@R' reply to get port 4 status.
        Reply format: 'B,R,R,R' or 'R,R,R,R' etc. (comma-separated, 4 chars).
        Returns 'R', 'B', 'S', or '?' if unparseable.
        """
        try:
            if not reply:
                return "?"
            parts = reply.strip().split(",")
            if len(parts) >= 4:
                return parts[3].strip()   # index 3 = port 4
            return "?"
        except Exception:
            return "?"

    def run(self):
        import time
        try:
            # Step 1 — power on port 4
            self.progress.emit("Homing: powering on CAM motor (4ST0)...")
            self._client.send_recv("4ST0")

            if not self._running:
                return

            # Step 2 — set absolute scale
            self.progress.emit("Homing: setting absolute scale (4MA1)...")
            self._client.send_recv("4MA1")

            if not self._running:
                return

            # Step 3 — send home command
            self.progress.emit("Homing: sending home command (4HM1)...")
            self._client.send("4HM1")

            # Step 4 — poll '@R' until port 4 is ready
            t0 = time.time()
            while self._running:
                self.msleep(self.POLL_MS)

                elapsed = time.time() - t0
                if elapsed > self.TIMEOUT_S:
                    self.failed.emit(
                        f"Homing timeout after {self.TIMEOUT_S:.0f}s — "
                        "CAM motor did not reach home position.")
                    return

                try:
                    reply = self._client.send_recv("@R")
                except Exception as e:
                    self.failed.emit(f"Status poll error: {e}")
                    return

                status = self._parse_port4_status(reply)
                self.progress.emit(
                    f"Homing: CAM port 4 status = '{status}'  "
                    f"({elapsed:.1f}s elapsed)")

                if status == "R":
                    break

            if not self._running:
                self.progress.emit("Homing: stopped by user.")
                return

            self.progress.emit("Homing: CAM motor reached home position.")
            self.finished.emit()

        except Exception as e:
            self.failed.emit(f"Homing error: {e}")


class ConnectionDialog(QDialog):
    def __init__(self, parent=None, ip1=DEFAULT_CTRL1_IP, ip2=DEFAULT_CTRL2_IP, tcp_port=DEFAULT_TCP_PORT):
        super().__init__(parent)
        self.setWindowTitle("Connect (Two Controllers)")

        self.ip1_edit = QLineEdit(str(ip1))
        self.ip2_edit = QLineEdit(str(ip2))
        self.tcp_port = QSpinBox()
        self.tcp_port.setRange(1, 65535)
        self.tcp_port.setValue(int(tcp_port))

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight)
        form.setFormAlignment(Qt.AlignTop)
        form.setHorizontalSpacing(10)
        form.setVerticalSpacing(10)
        form.addRow("Controller 1 IP:", self.ip1_edit)
        form.addRow("Controller 2 IP:", self.ip2_edit)
        form.addRow("TCP port:", self.tcp_port)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 14, 14, 14)
        lay.setSpacing(12)
        lay.addLayout(form)
        lay.addWidget(btns)

    def get_values(self) -> Tuple[str, str, int]:
        return self.ip1_edit.text().strip(), self.ip2_edit.text().strip(), int(self.tcp_port.value())


def mm_to_steps(mm: float) -> int:
    return int(round(float(mm) * STEP_PER_MM))


def spd_to_steps(mm_s: float) -> int:
    return int(round(max(0.001, float(mm_s)) * STEP_PER_MM))


def try_parse_positions_4ch(reply: str) -> Dict[int, int]:
    if not reply:
        return {}
    s = reply.strip().replace("\r", "").replace("\n", "")
    idx = s.find("POS")
    if idx < 0:
        return {}
    tail = s[idx + 3:]
    parts = [p.strip() for p in tail.split(",") if p.strip() != ""]
    if len(parts) < 4:
        nums = re.findall(r"-?\d+", tail)
        if len(nums) < 4:
            return {}
        parts = nums[:4]
    try:
        steps = [int(parts[i]) for i in range(4)]
    except Exception:
        return {}
    return {port: steps[i] for i, port in enumerate([1, 2, 3, 4])}


def tilt_deg_to_step(deg: float, home_ref_step: int) -> int:
    rad = float(deg) * (math.pi / 180.0)
    step = (TILT_ROT_CENTER_MM * STEP_PER_MM) * math.sin(rad)
    return int(round(step + int(home_ref_step)))


def tilt_step_to_deg(step: int, home_ref_step: int) -> float:
    adj = float(int(step) - int(home_ref_step))
    denom = (TILT_ROT_CENTER_MM * STEP_PER_MM)
    if denom <= 0:
        return 0.0
    sinv = adj / denom
    if sinv > 1.0:
        sinv = 1.0
    if sinv < -1.0:
        sinv = -1.0
    rad = math.asin(sinv)
    return float(rad * (180.0 / math.pi))


class MotorControlMultiController(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Controller 1.0")
        self.setFixedSize(1040, 620)
        self.setStatusBar(QStatusBar(self))

        self.clients: Dict[int, MotorTCPClient] = {1: MotorTCPClient(), 2: MotorTCPClient()}
        self.ctrl_ips: Dict[int, str] = {1: DEFAULT_CTRL1_IP, 2: DEFAULT_CTRL2_IP}
        self.ctrl_port: int = DEFAULT_TCP_PORT

        self.axis_rows: Dict[Tuple[int, int], int] = {}
        self.axis_widgets: Dict[Tuple[int, int], Dict[str, object]] = {}
        self.axis_meta: Dict[Tuple[int, int], Dict[str, object]] = {}

        # per-tilt-axis reference step at "0.0 deg"
        self.tilt_home_ref_step: Dict[Tuple[int, int], int] = {}
        self._lens_home_thread: Optional["LensLoaderHomeThread"] = None
        self._cam_home_thread: Optional["CAMHomeThread"] = None
        self._developer_mode: bool = False  # controls CAM Home button visibility

        self._build_ui()
        self._apply_style()

        # expose speed control as "speed" (requested naming)
        self.speed = self.spin_speed  # QDoubleSpinBox widget

        # per-axis motor state (requested names)
        self.axis_state: Dict[Tuple[int, int], Dict[str, object]] = {}
        for name, ctrl, port, unit in AXES:
            key = (int(ctrl), int(port))
            cfg = AXIS_CFG.get(key, {})
            self.axis_state[key] = {
                "name": name,
                "unit": unit,
                "t0": float(cfg.get("t0", 0.0)),

                "Tmn": float(cfg.get("Tmn", 0.0)),
                "Tmx": float(cfg.get("Tmx", 0.0)),
                "tmn": float(cfg.get("tmn", 0.0)),
                "tmx": float(cfg.get("tmx", 0.0)),

                "polarity": float(cfg.get("polarity", 1.0)),

                "alfa": float(cfg.get("alfa", 0.0)),
                "beta": float(cfg.get("beta", 0.0)),
                "gamma": float(cfg.get("gamma", 0.0)),

                "tx0_lab": float(cfg.get("tx0_lab", 0.0)),
                "ty0_lab": float(cfg.get("ty0_lab", 0.0)),
                "tz0_lab": float(cfg.get("tz0_lab", 0.0)),

                "tcur": None,  # filled by polling
                "tpre": None,  # previous poll value
                "ttrg": None,  # last commanded target
                "speed": float(DEFAULT_SPEED_MM_S),
                "dir": 0,      # -1 / 0 / +1
            }

        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(250)
        self.poll_timer.timeout.connect(self._poll_positions)

        self._set_connected(False)

    def _set_table_alignment_all_center(self):
        for r in range(self.table.rowCount()):
            for c in range(self.table.columnCount()):
                it = self.table.item(r, c)
                if it is not None:
                    it.setTextAlignment(Qt.AlignCenter)
        for r in range(self.table.rowCount()):
            act = self.table.cellWidget(r, 5)
            if act is not None and act.layout() is not None:
                lay = act.layout()
                if isinstance(lay, QHBoxLayout):
                    lay.setAlignment(Qt.AlignCenter)

    def _build_ui(self):
        sb = self.statusBar()
        sb.setSizeGripEnabled(False)
        self.sb_state = QLabel("Disconnected")
        self.sb_state.setProperty("muted", True)
        sb.addWidget(self.sb_state, 1)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        self.panel_settings = QGroupBox("Settings")
        s_lay = QHBoxLayout(self.panel_settings)
        s_lay.setContentsMargins(14, 12, 14, 12)
        s_lay.setSpacing(12)

        left = QVBoxLayout()
        left.setSpacing(6)
        self.lbl_conn = QLabel("Not connected")
        self.lbl_conn.setProperty("title", True)
        self.lbl_conn2 = QLabel("C1: —   C2: —")
        self.lbl_conn2.setProperty("muted", True)
        left.addWidget(self.lbl_conn)
        left.addWidget(self.lbl_conn2)

        mid = QVBoxLayout()
        mid.setSpacing(6)
        mid_top = QHBoxLayout()
        mid_top.setSpacing(10)
        lbl_speed = QLabel("Speed (mm/s)")
        lbl_speed.setProperty("muted", True)
        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setRange(0.001, 50.0)
        self.spin_speed.setDecimals(3)
        self.spin_speed.setValue(DEFAULT_SPEED_MM_S)
        self.spin_speed.setFixedWidth(160)
        self.spin_speed.setAlignment(Qt.AlignCenter)
        mid_top.addWidget(lbl_speed)
        mid_top.addStretch(1)
        mid_top.addWidget(self.spin_speed)
        mid.addLayout(mid_top)

        self.btn_stop_all = QPushButton("STOP ALL")
        self.btn_stop_all.setObjectName("danger")
        self.btn_stop_all.clicked.connect(self.stop_all)
        self.btn_stop_all.setFixedHeight(36)
        mid.addWidget(self.btn_stop_all, 0, Qt.AlignRight)

        right = QVBoxLayout()
        right.setSpacing(8)
        self.btn_connect = QPushButton("Connect")
        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.setObjectName("secondary")
        self.btn_connect.setFixedSize(140, 36)
        self.btn_disconnect.setFixedSize(140, 36)
        self.btn_connect.clicked.connect(self.action_connect)
        self.btn_disconnect.clicked.connect(self.action_disconnect)
        right.addWidget(self.btn_connect)
        right.addWidget(self.btn_disconnect)
        right.addStretch(1)

        s_lay.addLayout(left, 2)
        s_lay.addLayout(mid, 2)
        s_lay.addLayout(right, 0)

        self.panel_manual = QGroupBox("Manual Control")
        m_lay = QVBoxLayout(self.panel_manual)
        m_lay.setContentsMargins(14, 12, 14, 14)
        m_lay.setSpacing(10)

        self.table = QTableWidget(len(AXES), 6, self)
        self.table.setHorizontalHeaderLabels(["Axis", "Port", "Target", "Commanded", "Reported", "Actions"])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setFocusPolicy(Qt.NoFocus)
        self.table.setShowGrid(False)
        self.table.setAlternatingRowColors(True)

        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.Fixed)
        header.setSectionResizeMode(3, QHeaderView.Fixed)
        header.setSectionResizeMode(4, QHeaderView.Fixed)
        header.setSectionResizeMode(5, QHeaderView.Stretch)

        self.table.setColumnWidth(2, 180)
        self.table.setColumnWidth(3, 150)
        self.table.setColumnWidth(4, 150)
        self.table.verticalHeader().setDefaultSectionSize(42)

        GO_W, GO_H = 75, 32
        STOP_W, STOP_H = 75, 32

        for r, (name, ctrl, port, unit) in enumerate(AXES):
            key = (int(ctrl), int(port))
            self.axis_meta[key] = {"name": name, "ctrl": int(ctrl), "port": int(port), "unit": unit}

            it_axis = QTableWidgetItem(name)
            it_axis.setData(Qt.UserRole, key)
            it_axis.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(r, 0, it_axis)

            it_port = QTableWidgetItem(f"C{int(ctrl)}:{int(port)}")
            it_port.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(r, 1, it_port)

            spin = QDoubleSpinBox()
            spin.setDecimals(3)
            spin.setFixedHeight(28)
            spin.setAlignment(Qt.AlignCenter)

            if unit == "deg":
                spin.setRange(-TILT_MAX_DEG, TILT_MAX_DEG)
                spin.setSingleStep(0.01)
                spin.setSuffix(" °")
            else:
                spin.setRange(-1000.0, 1000.0)
                spin.setSingleStep(0.1)
                spin.setSuffix(" mm")

            spin.setValue(0.0)

            spin_wrap = QWidget()
            spin_wrap_lay = QVBoxLayout(spin_wrap)
            spin_wrap_lay.setContentsMargins(0, 0, 0, 0)
            spin_wrap_lay.setSpacing(0)
            spin_wrap_lay.setAlignment(Qt.AlignCenter)
            spin_wrap_lay.addWidget(spin)
            self.table.setCellWidget(r, 2, spin_wrap)

            it_cmd = QTableWidgetItem(f"0.000 {'°' if unit == 'deg' else 'mm'}")
            it_cmd.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(r, 3, it_cmd)

            it_rep = QTableWidgetItem("—")
            it_rep.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(r, 4, it_rep)

            btn_go = QPushButton("Go")
            btn_stop = QPushButton("Stop")
            btn_stop.setObjectName("secondary")
            btn_go.setFixedSize(GO_W, GO_H)
            btn_stop.setFixedSize(STOP_W, STOP_H)

            btn_go.clicked.connect(lambda _, c=int(ctrl), p=int(port), sp=spin: self.move_to_t(c, p, sp.value()))
            btn_stop.clicked.connect(lambda _, c=int(ctrl), p=int(port): self.stop_axis(c, p))

            act = QWidget()
            act_lay = QHBoxLayout(act)
            act_lay.setContentsMargins(0, 0, 0, 0)
            act_lay.setSpacing(8)
            act_lay.setAlignment(Qt.AlignCenter)
            act_lay.addWidget(btn_go)
            act_lay.addWidget(btn_stop)

            # Add Home button for Lens_Loader (ctrl=2, port=4) - always visible
            btn_home = None
            if int(ctrl) == 2 and int(port) == 4:
                btn_home = QPushButton("Home")
                btn_home.setObjectName("secondary")
                btn_home.setFixedSize(GO_W, GO_H)
                btn_home.setToolTip("Send homing (origin return) command to Lens_Loader")
                btn_home.clicked.connect(self.home_lens_loader)
                act_lay.addWidget(btn_home)

            # Add Home button for CAM motor (ctrl=1, port=4) - HIDDEN by default (Developer Mode only)
            btn_cam_home = None
            if int(ctrl) == 1 and int(port) == 4:
                btn_cam_home = QPushButton("Home")
                btn_cam_home.setObjectName("secondary")
                btn_cam_home.setFixedSize(GO_W, GO_H)
                btn_cam_home.setToolTip(
                    "Send homing (origin return) command to CAM motor\n"
                    "[Developer Mode only]")
                btn_cam_home.clicked.connect(self.home_cam_motor)
                btn_cam_home.setVisible(False)  # Hidden by default
                act_lay.addWidget(btn_cam_home)

            self.table.setCellWidget(r, 5, act)

            self.axis_rows[key] = r
            self.axis_widgets[key] = dict(spin=spin, btn_go=btn_go, btn_stop=btn_stop,
                                          btn_home=btn_home, btn_cam_home=btn_cam_home)

        self.table.setMinimumHeight(320)
        self.table.setMaximumHeight(380)
        m_lay.addWidget(self.table)

        root.addWidget(self.panel_settings, 0)
        root.addWidget(self.panel_manual, 1)

        self._set_table_alignment_all_center()

    def _apply_style(self):
        self.setStyleSheet("""
        QMainWindow { background-color: #f3f5f7; }
        QGroupBox {
            font-size: 13px;
            font-weight: 650;
            color: #223046;
            background-color: #ffffff;
            border: 1px solid #dde3ea;
            border-radius: 14px;
            margin-top: 10px;
            padding-top: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 14px;
            padding: 2px 10px;
            background-color: #dbeafe;
            color: #1e3a8a;
            border: 1px solid #bfdbfe;
            border-radius: 10px;
        }
        QLabel[muted="true"] { color: #64748b; }
        QLabel[title="true"] { color: #0f172a; font-size: 14px; font-weight: 800; }
        QPushButton {
            font-size: 12px;
            font-weight: 750;
            background-color: #3b82f6;
            color: white;
            border: none;
            border-radius: 10px;
            padding: 8px 12px;
        }
        QPushButton:hover { background-color: #2563eb; }
        QPushButton:pressed { background-color: #1d4ed8; }
        QPushButton:disabled { background-color: #cbd5e1; color: #64748b; }
        QPushButton#secondary {
            background-color: #e9eef5;
            color: #223046;
            border: 1px solid #d7dee8;
        }
        QPushButton#secondary:hover { background-color: #dde6f2; }
        QPushButton#secondary:pressed { background-color: #cfd9e7; }
        QPushButton#danger { background-color: #fb7185; }
        QPushButton#danger:hover { background-color: #f43f5e; }
        QPushButton#danger:pressed { background-color: #e11d48; }
        QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
            background: #ffffff;
            border: 1px solid #d7dee8;
            border-radius: 10px;
            padding: 7px 10px;
            font-size: 12px;
            color: #0f172a;
        }
        QDoubleSpinBox { qproperty-alignment: 'AlignCenter'; }
        QTableWidget {
            background-color: #ffffff;
            border: 1px solid #dde3ea;
            border-radius: 12px;
            gridline-color: transparent;
            alternate-background-color: #f7fafc;
            selection-background-color: transparent;
        }
        QHeaderView::section {
            background-color: #f0f4f8;
            color: #475569;
            font-weight: 800;
            border: none;
            border-bottom: 1px solid #dde3ea;
            padding: 8px 10px;
        }
        QStatusBar {
            background: #ffffff;
            color: #334155;
            border-top: 1px solid #dde3ea;
        }
        """)

    def _any_connected(self) -> bool:
        return self.clients[1].is_connected() or self.clients[2].is_connected()

    def _both_connected(self) -> bool:
        return self.clients[1].is_connected() and self.clients[2].is_connected()

    def _set_connected(self, ok: bool):
        if ok:
            self.sb_state.setText("Connected")
            self.lbl_conn.setText("Connected")
            self.lbl_conn2.setText(
                f"C1: {self.clients[1].ip}:{self.clients[1].port}   C2: {self.clients[2].ip}:{self.clients[2].port}"
            )
        else:
            self.sb_state.setText("Disconnected")
            self.lbl_conn.setText("Not connected")
            self.lbl_conn2.setText("C1: —   C2: —")

        for (_c, _p), w in self.axis_widgets.items():
            w["btn_go"].setEnabled(ok)
            w["btn_stop"].setEnabled(ok)
            w["spin"].setEnabled(ok)
            if w.get("btn_home") is not None:
                w["btn_home"].setEnabled(ok)
            # CAM Home button: only enable if connected AND developer mode is on
            if w.get("btn_cam_home") is not None:
                w["btn_cam_home"].setEnabled(ok and self._developer_mode)

        self.btn_stop_all.setEnabled(ok)
        self.btn_disconnect.setEnabled(ok)

    def _client_for_ctrl(self, ctrl: int) -> MotorTCPClient:
        return self.clients[int(ctrl)]

    def power_on_used_ports(self):
        if not self._both_connected():
            return
        used: Dict[int, List[int]] = {1: [], 2: []}
        for _, c, p, _u in AXES:
            used[int(c)].append(int(p))
        for c in (1, 2):
            cl = self.clients[c]
            for p in sorted(set(used[c])):
                cl.send_recv(f"{p}ST0")

    def power_off_used_ports(self):
        used: Dict[int, List[int]] = {1: [], 2: []}
        for _, c, p, _u in AXES:
            used[int(c)].append(int(p))
        for c in (1, 2):
            cl = self.clients[c]
            if not cl.is_connected():
                continue
            for p in sorted(set(used[c])):
                cl.send_recv(f"{p}ST1")

    def _init_tilt_home_reference_from_current(self):
        """
        Capture the current motor steps as the reference for 0.0° for each tilt axis.
        """
        if not self._both_connected():
            return
        try:
            rep2 = self.clients[2].send_recv("@P").strip()
            steps2 = try_parse_positions_4ch(rep2)
        except Exception:
            steps2 = {}

        for (ctrl, port), meta in self.axis_meta.items():
            if meta.get("unit") != "deg":
                continue
            if int(ctrl) != 2:
                continue
            self.tilt_home_ref_step[(int(ctrl), int(port))] = int(steps2.get(int(port), 0))

    def action_connect(self):
        dlg = ConnectionDialog(self, ip1=self.ctrl_ips[1], ip2=self.ctrl_ips[2], tcp_port=self.ctrl_port)
        if dlg.exec_() != QDialog.Accepted:
            return
        ip1, ip2, port = dlg.get_values()

        try:
            self.action_disconnect(silent=True)
        except Exception:
            pass

        try:
            self.clients[1].connect(ip1, port, timeout_s=0.8)
            self.clients[2].connect(ip2, port, timeout_s=0.8)
            self.ctrl_ips[1], self.ctrl_ips[2], self.ctrl_port = ip1, ip2, int(port)

            self.power_on_used_ports()
            self._init_tilt_home_reference_from_current()
            self._set_connected(True)
            self.poll_timer.start()

            QMessageBox.information(
                self, "Connected",
                f"Connected:\nC1 {ip1}:{port}\nC2 {ip2}:{port}\nPower ON used ports (ST0) sent."
            )
        except Exception as e:
            try:
                self.clients[1].disconnect()
            except Exception:
                pass
            try:
                self.clients[2].disconnect()
            except Exception:
                pass
            self._set_connected(False)
            QMessageBox.critical(self, "Connect failed", str(e))

    def action_disconnect(self, silent: bool = False):
        try:
            self.poll_timer.stop()
        except Exception:
            pass
        try:
            self.stop_all()
        except Exception:
            pass
        try:
            self.power_off_used_ports()
        except Exception:
            pass
        try:
            self.clients[1].disconnect()
        except Exception:
            pass
        try:
            self.clients[2].disconnect()
        except Exception:
            pass

        self._set_connected(False)
        for r in range(self.table.rowCount()):
            self.table.item(r, 4).setText("—")

        if not silent:
            QMessageBox.information(self, "Disconnected", "Power OFF used ports (ST1) sent.\nDisconnected.")

    @pyqtSlot(float)
    def set_speed(self, mm_s: float):
        self.spin_speed.setValue(float(mm_s))

    def _clamp_allowed(self, ctrl: int, port: int, t: float) -> float:
        key = (int(ctrl), int(port))
        st = self.axis_state.get(key, {})
        tmn = float(st.get("tmn", -1e18))
        tmx = float(st.get("tmx", 1e18))
        if t < tmn:
            return tmn
        if t > tmx:
            return tmx
        return t

    @pyqtSlot(int, float)
    def move(self, port: int, target_mm: float, velocity_mm_s: Optional[float] = None):
        if velocity_mm_s is not None:
            self.set_speed(float(velocity_mm_s))
        key = (1, int(port))
        w = self.axis_widgets.get(key)
        if w and "spin" in w:
            w["spin"].setValue(float(target_mm))
        self.move_to_t(1, int(port), float(target_mm))

    @pyqtSlot(int)
    def stop(self, port: int):
        self.stop_axis(1, int(port))

    @pyqtSlot(int, int, float)
    def move2(self, ctrl: int, port: int, target_value: float):
        key = (int(ctrl), int(port))
        w = self.axis_widgets.get(key)
        if w and "spin" in w:
            w["spin"].setValue(float(target_value))
        self.move_to_t(int(ctrl), int(port), float(target_value))

    @pyqtSlot(int, int)
    def stop2(self, ctrl: int, port: int):
        self.stop_axis(int(ctrl), int(port))

    # requested naming: move to target position as move_to_t
    def move_to_t(self, ctrl: int, port: int, ttrg: float):
        cl = self._client_for_ctrl(int(ctrl))
        if not cl.is_connected():
            return

        key = (int(ctrl), int(port))
        meta = self.axis_meta.get(key)
        unit = meta.get("unit", "mm") if meta else "mm"

        st = self.axis_state.get(key)
        if st is None:
            return

        # clamp by allowed moving range
        raw = float(ttrg)
        ttrg_clamped = float(self._clamp_allowed(int(ctrl), int(port), raw))

        # update state
        st["ttrg"] = ttrg_clamped
        st["speed"] = float(self.spin_speed.value())

        tcur = st.get("tcur", None)
        if isinstance(tcur, (int, float)):
            dt = float(ttrg_clamped) - float(tcur)
            st["dir"] = 1 if dt > 0 else (-1 if dt < 0 else 0)
        else:
            st["dir"] = 0

        polarity = float(st.get("polarity", 1.0))

        try:
            spd = spd_to_steps(self.spin_speed.value())

            if unit == "deg":
                # keep extra safety too
                if abs(ttrg_clamped) > TILT_MAX_DEG:
                    QMessageBox.warning(
                        self, "Move rejected",
                        f"Target {ttrg_clamped:.3f}° exceeds ±{TILT_MAX_DEG}°"
                    )
                    return
                ref = int(self.tilt_home_ref_step.get(key, 0))
                tgt_steps = tilt_deg_to_step(ttrg_clamped * polarity, ref)
            else:
                tgt_steps = mm_to_steps(ttrg_clamped * polarity)

            cl.send_recv(f"{port}ST0")
            cl.send_recv(f"{port}MA1")
            cl.send_recv(f"{port}V{spd}")
            cl.send_recv(f"{port}D{tgt_steps}")
            cl.send(f"{port}G")

            row = self.axis_rows.get(key)
            if row is not None:
                suf = "°" if unit == "deg" else "mm"
                self.table.item(row, 3).setText(f"{ttrg_clamped:0.3f} {suf}")

        except Exception as e:
            QMessageBox.warning(self, "Move failed", str(e))

    # backward-compatible alias (so older UI hookups still work if any remain)
    def move_abs(self, ctrl: int, port: int, value: float):
        self.move_to_t(ctrl, port, value)

    def stop_axis(self, ctrl: int, port: int):
        cl = self._client_for_ctrl(int(ctrl))
        if not cl.is_connected():
            return
        try:
            cl.send(f"{port}S")
        except Exception as e:
            QMessageBox.warning(self, "Stop failed", str(e))

    def home_lens_loader(self):
        """
        Home the Lens_Loader motor (ctrl=2, port=4).
        Command sequence (from commu_motor.go_home):
            4ST0 → power on
            4MA1 → set absolute scale
            4HM1 → go home
        Then polls '@R' until port 4 reports 'R' (ready).
        """
        cl = self.clients[2]
        if not cl.is_connected():
            QMessageBox.warning(self, "Home Lens Loader",
                "Controller 2 is not connected.")
            return

        if self._lens_home_thread is not None and self._lens_home_thread.isRunning():
            QMessageBox.information(self, "Home Lens Loader",
                "Homing is already in progress.")
            return

        answer = QMessageBox.question(
            self, "Home Lens Loader",
            "Send homing command to Lens_Loader (ctrl=2, port=4)?\n\n"
            "Commands:  4ST0  →  4MA1  →  4HM1\n\n"
            "The motor will move to its hardware origin (0.000 mm).\n"
            "Make sure the path is clear before proceeding.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if answer != QMessageBox.Yes:
            return

        key = (2, 4)

        # Disable Home button and show "Homing..." while running
        w = self.axis_widgets.get(key)
        if w and w.get("btn_home") is not None:
            w["btn_home"].setEnabled(False)
            w["btn_home"].setText("Homing...")

        # Update table display
        row = self.axis_rows.get(key)
        if row is not None:
            self.table.item(row, 3).setText("0.000 mm")
            self.table.item(row, 4).setText("homing...")

        # Start the homing thread
        self._lens_home_thread = LensLoaderHomeThread(cl)
        self._lens_home_thread.progress.connect(
            lambda msg: self.statusBar().showMessage(msg, 4000))
        self._lens_home_thread.finished.connect(self._on_lens_home_finished)
        self._lens_home_thread.failed.connect(self._on_lens_home_failed)
        self._lens_home_thread.start()

    def _on_lens_home_finished(self):
        """Called when LensLoaderHomeThread completes successfully."""
        key = (2, 4)

        # Reset axis state to 0
        st = self.axis_state.get(key)
        if st is not None:
            st["tcur"] = 0.0
            st["tpre"] = None
            st["ttrg"] = 0.0

        # Reset spin box to 0
        w = self.axis_widgets.get(key)
        if w:
            if w.get("spin") is not None:
                w["spin"].setValue(0.0)
            if w.get("btn_home") is not None:
                w["btn_home"].setEnabled(True)
                w["btn_home"].setText("Home")

        # Update table
        row = self.axis_rows.get(key)
        if row is not None:
            self.table.item(row, 3).setText("0.000 mm")
            self.table.item(row, 4).setText("0.000 mm")

        self.statusBar().showMessage(
            "Lens_Loader: homing complete — position reset to 0.000 mm", 5000)

    def _on_lens_home_failed(self, msg: str):
        """Called when LensLoaderHomeThread encounters an error or timeout."""
        key = (2, 4)

        # Re-enable Home button
        w = self.axis_widgets.get(key)
        if w and w.get("btn_home") is not None:
            w["btn_home"].setEnabled(True)
            w["btn_home"].setText("Home")

        self.statusBar().showMessage(f"Lens_Loader homing failed: {msg}", 6000)
        QMessageBox.warning(self, "Home Lens Loader Failed", msg)

    def home_cam_motor(self):
        """
        Home the CAM motor (ctrl=1, port=4).
        Only available in Developer Mode.

        WARNING: This is a dangerous operation that can damage equipment
        if the CAM motor path is not clear. Use with extreme caution.

        Command sequence (same as Lens_Loader):
            4ST0 → power on
            4MA1 → set absolute scale
            4HM1 → go home
        Then polls '@R' until port 4 reports 'R' (ready).
        """
        if not self._developer_mode:
            QMessageBox.warning(self, "CAM Home",
                "CAM Home is only available in Developer Mode.\n\n"
                "Enable Developer Mode from the main application:\n"
                "File \u2192 Developer mode")
            return

        cl = self.clients[1]  # CAM is on Controller 1
        if not cl.is_connected():
            QMessageBox.warning(self, "Home CAM Motor",
                "Controller 1 is not connected.")
            return

        if self._cam_home_thread is not None and self._cam_home_thread.isRunning():
            QMessageBox.information(self, "Home CAM Motor",
                "CAM homing is already in progress.")
            return

        answer = QMessageBox.warning(
            self, "Home CAM Motor \u2014 DANGER",
            "\u26a0\ufe0f WARNING: CAM Motor Homing \u26a0\ufe0f\n\n"
            "This operation will move the CAM motor to its hardware origin.\n\n"
            "DANGER: This can cause collision with the camera or other components\n"
            "if the path is not clear!\n\n"
            "Commands:  4ST0  \u2192  4MA1  \u2192  4HM1\n\n"
            "Are you ABSOLUTELY SURE you want to proceed?\n\n"
            "Click 'Yes' only if you have verified the path is clear.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if answer != QMessageBox.Yes:
            return

        key = (1, 4)  # CAM motor: Controller 1, Port 4

        # Disable Home button and show "Homing..." while running
        w = self.axis_widgets.get(key)
        if w and w.get("btn_cam_home") is not None:
            w["btn_cam_home"].setEnabled(False)
            w["btn_cam_home"].setText("Homing...")

        # Update table display
        row = self.axis_rows.get(key)
        if row is not None:
            self.table.item(row, 3).setText("0.000 mm")
            self.table.item(row, 4).setText("homing...")

        # Start the homing thread
        self._cam_home_thread = CAMHomeThread(cl)
        self._cam_home_thread.progress.connect(
            lambda msg: self.statusBar().showMessage(msg, 4000))
        self._cam_home_thread.finished.connect(self._on_cam_home_finished)
        self._cam_home_thread.failed.connect(self._on_cam_home_failed)
        self._cam_home_thread.start()

    def _on_cam_home_finished(self):
        """Called when CAMHomeThread completes successfully."""
        key = (1, 4)

        # Reset axis state to 0
        st = self.axis_state.get(key)
        if st is not None:
            st["tcur"] = 0.0
            st["tpre"] = None
            st["ttrg"] = 0.0

        # Reset spin box to 0
        w = self.axis_widgets.get(key)
        if w:
            if w.get("spin") is not None:
                w["spin"].setValue(0.0)
            if w.get("btn_cam_home") is not None:
                w["btn_cam_home"].setEnabled(True)
                w["btn_cam_home"].setText("Home")

        # Update table
        row = self.axis_rows.get(key)
        if row is not None:
            self.table.item(row, 3).setText("0.000 mm")
            self.table.item(row, 4).setText("0.000 mm")

        self.statusBar().showMessage(
            "CAM motor: homing complete \u2014 position reset to 0.000 mm", 5000)

    def _on_cam_home_failed(self, msg: str):
        """Called when CAMHomeThread encounters an error or timeout."""
        key = (1, 4)

        # Re-enable Home button
        w = self.axis_widgets.get(key)
        if w and w.get("btn_cam_home") is not None:
            w["btn_cam_home"].setEnabled(True)
            w["btn_cam_home"].setText("Home")

        self.statusBar().showMessage(f"CAM motor homing failed: {msg}", 6000)
        QMessageBox.warning(self, "Home CAM Motor Failed", msg)

    def set_developer_mode(self, enabled: bool):
        """
        Enable or disable Developer Mode.
        When enabled, shows the CAM Home button (ctrl=1, port=4).
        When disabled, hides the CAM Home button.

        Called from the main RCTS application when Developer Mode is toggled.
        """
        self._developer_mode = bool(enabled)

        # Show/hide CAM Home button
        key = (1, 4)  # CAM motor
        w = self.axis_widgets.get(key)
        if w and w.get("btn_cam_home") is not None:
            btn = w["btn_cam_home"]
            btn.setVisible(bool(enabled))
            # Also update enabled state to respect connection status
            connected = self._both_connected()
            btn.setEnabled(bool(enabled) and connected)

        # Update status bar
        mode_str = "ON" if enabled else "OFF"
        self.statusBar().showMessage(f"Developer Mode: {mode_str}", 3000)

    def stop_all(self):
        try:
            for _, c, p, _u in AXES:
                cl = self.clients[int(c)]
                if not cl.is_connected():
                    continue
                try:
                    cl.send(f"{int(p)}S")
                except Exception:
                    pass
        finally:
            self.statusBar().showMessage("Stop all sent.", 1200)

    def _poll_positions(self):
        if not self._both_connected():
            return

        steps1: Dict[int, int] = {}
        steps2: Dict[int, int] = {}

        try:
            rep1 = self.clients[1].send_recv("@P").strip()
            steps1 = try_parse_positions_4ch(rep1)
        except Exception:
            steps1 = {}

        try:
            rep2 = self.clients[2].send_recv("@P").strip()
            steps2 = try_parse_positions_4ch(rep2)
        except Exception:
            steps2 = {}

        # Controller 1
        if steps1:
            for (ctrl, port), row in self.axis_rows.items():
                if int(ctrl) != 1 or int(port) not in steps1:
                    continue

                key = (int(ctrl), int(port))
                meta = self.axis_meta[key]
                unit = meta.get("unit", "mm")

                if unit == "deg":
                    ref = int(self.tilt_home_ref_step.get(key, 0))
                    val = tilt_step_to_deg(int(steps1[int(port)]), ref)

                    st = self.axis_state.get(key)
                    if st is not None:
                        st["tpre"] = st.get("tcur", None)
                        st["tcur"] = float(val)

                    self.table.item(row, 4).setText(f"{val:0.3f} °")
                else:
                    mm = float(int(steps1[int(port)])) / STEP_PER_MM

                    st = self.axis_state.get(key)
                    if st is not None:
                        st["tpre"] = st.get("tcur", None)
                        st["tcur"] = float(mm)

                    self.table.item(row, 4).setText(f"{mm:0.3f} mm")

        # Controller 2
        if steps2:
            for (ctrl, port), row in self.axis_rows.items():
                if int(ctrl) != 2 or int(port) not in steps2:
                    continue

                key = (int(ctrl), int(port))
                meta = self.axis_meta[key]
                unit = meta.get("unit", "mm")

                if unit == "deg":
                    ref = int(self.tilt_home_ref_step.get(key, 0))
                    val = tilt_step_to_deg(int(steps2[int(port)]), ref)

                    st = self.axis_state.get(key)
                    if st is not None:
                        st["tpre"] = st.get("tcur", None)
                        st["tcur"] = float(val)

                    self.table.item(row, 4).setText(f"{val:0.3f} °")
                else:
                    mm = float(int(steps2[int(port)])) / STEP_PER_MM

                    st = self.axis_state.get(key)
                    if st is not None:
                        st["tpre"] = st.get("tcur", None)
                        st["tcur"] = float(mm)

                    self.table.item(row, 4).setText(f"{mm:0.3f} mm")

    def closeEvent(self, event):
        try:
            if self._any_connected():
                self.action_disconnect(silent=True)
        except Exception:
            pass
        try:
            if self._lens_home_thread is not None and self._lens_home_thread.isRunning():
                self._lens_home_thread.stop()
                self._lens_home_thread.wait(2000)
        except Exception:
            pass
        try:
            if self._cam_home_thread is not None and self._cam_home_thread.isRunning():
                self._cam_home_thread.stop()
                self._cam_home_thread.wait(2000)
        except Exception:
            pass
        event.accept()


def create_motor_window() -> MotorControlMultiController:
    return MotorControlMultiController()


class MotorAPI:
    def __init__(self):
        self.clients: Dict[int, MotorTCPClient] = {1: MotorTCPClient(), 2: MotorTCPClient()}
        self._gui: Optional[MotorControlMultiController] = None

    def is_connected(self) -> bool:
        return self.clients[1].is_connected() and self.clients[2].is_connected()

    def show_gui(self):
        app = QApplication.instance()
        if app is None:
            raise RuntimeError("QApplication is not running. Create QApplication() first, then call motor.show_gui().")
        if self._gui is None:
            self._gui = MotorControlMultiController()
            self._gui.clients = self.clients
            self._gui._set_connected(self.clients[1].is_connected() and self.clients[2].is_connected())
        self._gui.show()
        if self.clients[1].is_connected() and self.clients[2].is_connected():
            try:
                self._gui.poll_timer.start()
            except Exception:
                pass

    def hide_gui(self):
        if self._gui is not None:
            self._gui.hide()

    def get_pos(self, port: int, ctrl: int = 1) -> Optional[float]:
        cl = self.clients.get(int(ctrl))
        if cl is None or not cl.is_connected():
            return None
        rep = cl.send_recv("@P")
        steps = try_parse_positions_4ch(rep)
        v = steps.get(int(port))
        if v is None:
            return None

        unit = "mm"
        for _n, c, p, u in AXES:
            if int(c) == int(ctrl) and int(p) == int(port):
                unit = u
                break

        if unit == "deg":
            ref = 0
            if self._gui is not None:
                ref = int(self._gui.tilt_home_ref_step.get((int(ctrl), int(port)), 0))
            return tilt_step_to_deg(int(v), int(ref))

        return float(int(v)) / STEP_PER_MM

    def get_pos2(self, ctrl: int, port: int) -> Optional[float]:
        return self.get_pos(int(port), int(ctrl))

    def connect(self, ip1: str, ip2: str, port: int, timeout_s: float = 0.8):
        self.clients[1].connect(ip1, port, timeout_s=timeout_s)
        self.clients[2].connect(ip2, port, timeout_s=timeout_s)

        used: Dict[int, List[int]] = {1: [], 2: []}
        for _name, c, p, _u in AXES:
            used[int(c)].append(int(p))
        for c in (1, 2):
            cl = self.clients[c]
            for p in sorted(set(used[c])):
                cl.send_recv(f"{p}ST0")

        if self._gui is not None:
            self._gui._init_tilt_home_reference_from_current()
            self._gui._set_connected(True)
            try:
                self._gui.poll_timer.start()
            except Exception:
                pass

    def disconnect(self):
        if self._gui is not None:
            try:
                self._gui.poll_timer.stop()
            except Exception:
                pass

        for _name, c, p, _u in AXES:
            cl = self.clients[int(c)]
            if not cl.is_connected():
                continue
            try:
                cl.send(f"{int(p)}S")
            except Exception:
                pass

        for _name, c, p, _u in AXES:
            cl = self.clients[int(c)]
            if not cl.is_connected():
                continue
            try:
                cl.send_recv(f"{int(p)}ST1")
            except Exception:
                pass

        try:
            self.clients[1].disconnect()
        except Exception:
            pass
        try:
            self.clients[2].disconnect()
        except Exception:
            pass

        if self._gui is not None:
            self._gui._set_connected(False)

    # requested naming at API level too
    def move_to_t(self, ctrl: int, port: int, ttrg: float, velocity_mm_s: Optional[float] = None):
        self.move2(int(ctrl), int(port), float(ttrg), velocity_mm_s=velocity_mm_s)

    def move(self, port: int, target_mm: float, velocity_mm_s: Optional[float] = None):
        # backward compatibility: ctrl=1 mm axes only
        if not (self.clients[1].is_connected() and self.clients[2].is_connected()):
            return
        if velocity_mm_s is not None and self._gui is not None:
            self._gui.set_speed(float(velocity_mm_s))

        cl = self.clients[1]
        spd = spd_to_steps(DEFAULT_SPEED_MM_S if self._gui is None else self._gui.spin_speed.value())
        tgt = mm_to_steps(target_mm)

        try:
            cl.send_recv(f"{int(port)}ST0")
            cl.send_recv(f"{int(port)}MA1")
            cl.send_recv(f"{int(port)}V{spd}")
            cl.send_recv(f"{int(port)}D{tgt}")
            cl.send(f"{int(port)}G")
        except Exception:
            pass

        if self._gui is not None:
            key = (1, int(port))
            w = self._gui.axis_widgets.get(key)
            if w and "spin" in w:
                w["spin"].setValue(float(target_mm))
            row = self._gui.axis_rows.get(key)
            if row is not None:
                self._gui.table.item(row, 3).setText(f"{float(target_mm):0.3f} mm")

    def move2(self, ctrl: int, port: int, target_value: float, velocity_mm_s: Optional[float] = None):
        if not (self.clients[1].is_connected() and self.clients[2].is_connected()):
            return
        if velocity_mm_s is not None and self._gui is not None:
            self._gui.set_speed(float(velocity_mm_s))

        cl = self.clients[int(ctrl)]
        spd = spd_to_steps(DEFAULT_SPEED_MM_S if self._gui is None else self._gui.spin_speed.value())

        unit = "mm"
        for _n, c, p, u in AXES:
            if int(c) == int(ctrl) and int(p) == int(port):
                unit = u
                break

        # Note: API does not enforce tmn/tmx here (GUI does). If you want, we can enforce here too.
        if unit == "deg":
            if abs(float(target_value)) > TILT_MAX_DEG:
                return
            ref = 0
            if self._gui is not None:
                ref = int(self._gui.tilt_home_ref_step.get((int(ctrl), int(port)), 0))
            tgt = tilt_deg_to_step(float(target_value), int(ref))
        else:
            tgt = mm_to_steps(float(target_value))

        try:
            cl.send_recv(f"{int(port)}ST0")
            cl.send_recv(f"{int(port)}MA1")
            cl.send_recv(f"{int(port)}V{spd}")
            cl.send_recv(f"{int(port)}D{tgt}")
            cl.send(f"{int(port)}G")
        except Exception:
            pass

        if self._gui is not None:
            key = (int(ctrl), int(port))
            w = self._gui.axis_widgets.get(key)
            if w and "spin" in w:
                w["spin"].setValue(float(target_value))
            row = self._gui.axis_rows.get(key)
            if row is not None:
                suf = "°" if unit == "deg" else "mm"
                self._gui.table.item(row, 3).setText(f"{float(target_value):0.3f} {suf}")

    def stop(self, port: int):
        if not self.clients[1].is_connected():
            return
        try:
            self.clients[1].send(f"{int(port)}S")
        except Exception:
            pass

    def stop2(self, ctrl: int, port: int):
        if not self.clients[int(ctrl)].is_connected():
            return
        try:
            self.clients[int(ctrl)].send(f"{int(port)}S")
        except Exception:
            pass

    def set_developer_mode(self, enabled: bool):
        """
        Set Developer Mode on the motor GUI.
        Shows/hides the CAM Home button (ctrl=1, port=4).
        """
        if self._gui is not None:
            self._gui.set_developer_mode(bool(enabled))


def run_standalone():
    app = QApplication.instance()
    owns_app = False
    if app is None:
        owns_app = True
        app = QApplication(sys.argv)
    w = MotorControlMultiController()
    w.show()
    if owns_app:
        sys.exit(app.exec_())
    return w


if __name__ == "__main__":
    run_standalone()
