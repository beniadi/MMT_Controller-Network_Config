"""
MMT Controller Network Configuration Tool
Reads and sets the Ethernet configuration (IP / Subnet mask / Gateway / Port) of an
MMT MMDC-series 4-axis stepper controller over RS-232.

"""

import sys
import time
import ipaddress
from typing import List, Optional, Tuple

import serial
import serial.tools.list_ports

from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QGroupBox, QStatusBar, QComboBox, QLineEdit,
    QPlainTextEdit, QCheckBox, QDoubleSpinBox, QMessageBox
)

# ---------------------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------------------

BAUDRATE = 115200

# Manual section 7.2: "Frame end interval 2 ms". Held between a completed reply and the
# next request. Kept slightly above the specified minimum.
INTER_COMMAND_DELAY_S = 0.05

# Reply framing. Manual section 8.1 puts a Carriage return at the end of a *request*;
# section 8.2 defines the *reply* as mask + echo + data (Case1/Case2) or mask + ok
# (Case3) with no terminating character at all. A reply is therefore delimited by the
# line going idle — the "Frame end interval" of section 7.2 — not by a byte value.
#
# The spec's interval is 2 ms, but a USB-serial bridge batches bytes on its own latency
# timer (commonly 16 ms), so a 2 ms gap would split one reply into fragments. The gap
# below is well clear of that while still ending a reply promptly.
FRAME_IDLE_GAP_S = 0.05

# Poll granularity for the read loop. Must be shorter than FRAME_IDLE_GAP_S so the gap
# is measured, not rounded up to the port timeout.
READ_POLL_S = 0.01

# Some firmware revisions do append CR/LF. If one arrives it ends the reply immediately;
# its absence is not an error. Raw bytes are always logged either way, so a framing
# mismatch stays visible rather than being silently absorbed.
REPLY_TERMINATORS = (b"\r", b"\n")

# All replies are masked with '*' (manual section 8.2).
REPLY_MASK = "*"

# Accepted acknowledgements for a setter. 'okAtten' is returned when motor power is off;
# it is an acceptance, not an error (manual section 8.2, Case3).
ACK_PREFIXES = ("*ok", "*okAtten")

# Factory defaults, manual section 7.3. Used only to populate the input fields on startup
# as a convenience — never written without an explicit user action.
DEFAULT_IP = "192.168.0.123"
DEFAULT_SUBNET_MASK = "255.255.255.0"
DEFAULT_GATEWAY = "192.168.0.1"
DEFAULT_PORT = "5001"

# Field definitions: (key, label, query command, setter command, default value)
# 'SM' and 'GW' exist only in the newer firmware command set. On older firmware they
# return no reply; that is reported as a per-field failure, not hidden.
NETWORK_FIELDS: List[Tuple[str, str, str, str, str]] = [
    ("ip",   "IP address",  "ip",   "ip",   DEFAULT_IP),
    ("sm",   "Subnet mask", "sm",   "sm",   DEFAULT_SUBNET_MASK),
    ("gw",   "Gateway",     "gw",   "gw",   DEFAULT_GATEWAY),
    ("port", "Port",        "port", "port", DEFAULT_PORT),
]


# ---------------------------------------------------------------------------------------
# Protocol helpers
# ---------------------------------------------------------------------------------------

def dotted_to_comma(dotted: str) -> str:
    """'192.168.0.123' -> '192,168,0,123'. Raises ValueError on an invalid address.

    The controller expects comma-separated octets (manual section 8.1). Validation is done
    with ipaddress so a typo is rejected here rather than written to the hardware.
    """
    address = ipaddress.IPv4Address(dotted.strip())
    return str(address).replace(".", ",")


def validate_port_number(text: str) -> int:
    """Validate a TCP port string and return it as an int. Raises ValueError if invalid."""
    value = int(text.strip())
    if not (1 <= value <= 65535):
        raise ValueError(f"port must be 1..65535, got {value}")
    return value


def parse_value_reply(reply: str, command: str) -> str:
    """Extract the payload from a value reply.

    Observed reply shapes:
        '*IP=192.168.0.123'      ethernet getters
        '*PORT5001'              port getter
        '*#1IR4'                 axis-scoped getters (axis prefix present)

    The payload is whatever follows the command echo. Raises ValueError if the reply does
    not carry the expected echo, so a mismatched or corrupted reply is never parsed into a
    plausible-looking value.
    """
    if not reply.startswith(REPLY_MASK):
        raise ValueError(f"reply is not masked with '{REPLY_MASK}': {reply!r}")

    body = reply[len(REPLY_MASK):]

    # Strip an axis prefix such as '#1' if the firmware includes one.
    if body.startswith("#") and len(body) >= 2 and body[1].isdigit():
        body = body[2:]

    upper_body = body.upper()
    upper_command = command.upper()
    if not upper_body.startswith(upper_command):
        raise ValueError(
            f"reply does not echo command {command!r}: {reply!r}"
        )

    payload = body[len(command):]
    if payload.startswith("="):
        payload = payload[1:]
    return payload.strip()


def is_acknowledgement(reply: str) -> bool:
    """True if the reply is an accepted-setter acknowledgement."""
    return reply.startswith(ACK_PREFIXES)


# ---------------------------------------------------------------------------------------
# Serial worker
# ---------------------------------------------------------------------------------------

class SerialCommand:
    """One request/reply exchange.

    expect_ack   True  -> the reply must be '*ok' / '*okAtten', otherwise the task fails
                 False -> the reply is a value, parsed against `command`
    abort_on_failure  True  -> stop the whole task if this command fails
                      False -> record the failure and continue (used for reads, so one
                               unsupported command does not hide the other three)
    """

    def __init__(self, key: str, label: str, command: str,
                 expect_ack: bool, abort_on_failure: bool):
        self.key = key
        self.label = label
        self.command = command
        self.expect_ack = expect_ack
        self.abort_on_failure = abort_on_failure


class SerialTaskWorker(QThread):
    """Runs a sequence of SerialCommands off the GUI thread.

    Emits every transmitted and received byte sequence so the log is a literal transcript
    of the link, not a summary. No retries: a command that does not answer is reported as
    it happened.
    """

    log_message = pyqtSignal(str)
    task_finished = pyqtSignal(dict)   # {key: value_string} for commands that succeeded
    task_failed = pyqtSignal(str)

    def __init__(self, port_name: str, commands: List[SerialCommand], read_timeout_s: float):
        super().__init__()
        self.port_name = port_name
        self.commands = commands
        self.read_timeout_s = read_timeout_s

    def run(self):
        results = {}
        try:
            with serial.Serial(
                port=self.port_name,
                baudrate=BAUDRATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=READ_POLL_S,
            ) as ser:
                self.log_message.emit(
                    f"--- opened {self.port_name} @ {BAUDRATE} 8N1 ---"
                )
                # Let the USB-serial bridge settle before the first request.
                time.sleep(0.2)
                ser.reset_input_buffer()
                ser.reset_output_buffer()

                for cmd in self.commands:
                    raw_reply = self._exchange(ser, cmd.command)

                    if raw_reply is None:
                        message = (
                            f"{cmd.label}: no reply within {self.read_timeout_s:.1f} s "
                            f"for command {cmd.command!r}"
                        )
                        self.log_message.emit(f"!! {message}")
                        if cmd.abort_on_failure:
                            self.task_failed.emit(message)
                            return
                        continue

                    reply = raw_reply.decode("ascii", errors="replace").strip()

                    if cmd.expect_ack:
                        if not is_acknowledgement(reply):
                            message = (
                                f"{cmd.label}: controller did not acknowledge. "
                                f"Sent {cmd.command!r}, received {reply!r}"
                            )
                            self.log_message.emit(f"!! {message}")
                            self.task_failed.emit(message)
                            return
                        results[cmd.key] = reply
                    else:
                        try:
                            results[cmd.key] = parse_value_reply(reply, cmd.command)
                        except ValueError as exc:
                            message = f"{cmd.label}: {exc}"
                            self.log_message.emit(f"!! {message}")
                            if cmd.abort_on_failure:
                                self.task_failed.emit(message)
                                return
                            continue

                    time.sleep(INTER_COMMAND_DELAY_S)

                self.log_message.emit("--- port closed ---")

        except serial.SerialException as exc:
            self.task_failed.emit(f"Serial port error: {exc}")
            return

        self.task_finished.emit(results)

    def _exchange(self, ser: serial.Serial, command: str) -> Optional[bytes]:
        """Send one command and read one reply frame.

        Per manual section 8.2 a reply carries no terminating character, so the frame is
        closed by the line going idle for FRAME_IDLE_GAP_S. A CR/LF, if the firmware sends
        one, closes it earlier. Returns the raw reply bytes, or None if nothing at all
        arrived before the deadline.
        """
        ser.reset_input_buffer()
        tx = (command + "\r").encode("ascii")
        ser.write(tx)
        ser.flush()
        self.log_message.emit(f"TX  {tx!r}")

        buffer = bytearray()
        deadline = time.monotonic() + self.read_timeout_s
        last_byte_at: Optional[float] = None

        while time.monotonic() < deadline:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                buffer += chunk
                last_byte_at = time.monotonic()
                if chunk[-1:] in REPLY_TERMINATORS and len(buffer) > 1:
                    self.log_message.emit(f"RX  {bytes(buffer)!r}")
                    return bytes(buffer)
                continue

            # No byte this poll. Once the line has been idle for a full frame gap the
            # reply is complete as the manual defines it.
            if last_byte_at is not None and \
                    time.monotonic() - last_byte_at >= FRAME_IDLE_GAP_S:
                self.log_message.emit(f"RX  {bytes(buffer)!r}")
                return bytes(buffer)

        if buffer:
            # Bytes kept arriving for the whole timeout without an idle gap: that is a
            # real framing problem, not the normal unterminated reply.
            self.log_message.emit(
                f"RX  {bytes(buffer)!r}  (still streaming at deadline, frame never closed)"
            )
            return bytes(buffer)

        self.log_message.emit("RX  <nothing>")
        return None


# ---------------------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------------------

class NetworkConfigWindow(QMainWindow):
    """Read and set the MMT controller's Ethernet configuration over RS-232."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MMT Controller — Network Configuration (RS-232)")
        self.setMinimumSize(880, 640)

        self.worker: Optional[SerialTaskWorker] = None
        self.current_fields = {}      # key -> QLineEdit (read-only, values from controller)
        self.new_fields = {}          # key -> QLineEdit (editable, values to write)
        self.write_enable = {}        # key -> QCheckBox

        self._build_ui()
        self._apply_styles()
        self._refresh_ports()

    # -- UI construction ----------------------------------------------------------------

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setSpacing(10)

        top_row = QHBoxLayout()
        top_row.setSpacing(10)
        top_row.addWidget(self._build_connection_group(), 1)
        top_row.addWidget(self._build_current_group(), 1)
        outer.addLayout(top_row)

        outer.addWidget(self._build_new_group())
        outer.addWidget(self._build_log_group(), 1)

        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("Select the COM port connected to the controller's RS-232/485 connector")

    def _build_connection_group(self) -> QGroupBox:
        group = QGroupBox("Connection")
        layout = QGridLayout(group)
        layout.setSpacing(8)

        layout.addWidget(QLabel("COM port"), 0, 0)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(240)
        layout.addWidget(self.port_combo, 0, 1)

        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self._refresh_ports)
        layout.addWidget(self.refresh_button, 0, 2)

        layout.addWidget(QLabel("Baud rate"), 1, 0)
        baud_label = QLabel(f"{BAUDRATE} — 8 data bits, no parity, 1 stop bit")
        baud_label.setStyleSheet("color: #6b7280;")
        layout.addWidget(baud_label, 1, 1, 1, 2)

        layout.addWidget(QLabel("Reply timeout"), 2, 0)
        self.timeout_spin = QDoubleSpinBox()
        self.timeout_spin.setRange(0.2, 10.0)
        self.timeout_spin.setSingleStep(0.1)
        self.timeout_spin.setValue(1.0)
        self.timeout_spin.setSuffix(" s")
        layout.addWidget(self.timeout_spin, 2, 1)

        hint = QLabel(
            "Use the RS-232/485 DB9 on the controller front panel.\n"
            "Pin 2 = TX, pin 3 = RX, pin 5 = GND.\n"
            "Leave pins 1, 4, 6, 7, 8, 9 unconnected for RS-232.\n"
            "MODE switch must be set to RUN, not PRG."
        )
        hint.setStyleSheet("color: #6b7280; font-size: 11px;")
        layout.addWidget(hint, 3, 0, 1, 3)

        layout.setRowStretch(4, 1)
        return group

    def _build_current_group(self) -> QGroupBox:
        group = QGroupBox("Current configuration (read from controller)")
        layout = QGridLayout(group)
        layout.setSpacing(8)

        for row, (key, label, _query, _setter, _default) in enumerate(NETWORK_FIELDS):
            layout.addWidget(QLabel(label), row, 0)
            field = QLineEdit()
            field.setReadOnly(True)
            field.setPlaceholderText("not read yet")
            self.current_fields[key] = field
            layout.addWidget(field, row, 1)

        self.read_button = QPushButton("Read from Controller")
        self.read_button.clicked.connect(self._on_read_configuration)
        layout.addWidget(self.read_button, len(NETWORK_FIELDS), 0, 1, 2)

        self.copy_button = QPushButton("Copy to New Configuration")
        self.copy_button.setObjectName("secondaryButton")
        self.copy_button.clicked.connect(self._on_copy_current_to_new)
        layout.addWidget(self.copy_button, len(NETWORK_FIELDS) + 1, 0, 1, 2)

        layout.setRowStretch(len(NETWORK_FIELDS) + 2, 1)
        return group

    def _build_new_group(self) -> QGroupBox:
        group = QGroupBox("New configuration (written to controller)")
        layout = QGridLayout(group)
        layout.setSpacing(8)

        layout.addWidget(QLabel("Write"), 0, 0)
        layout.addWidget(QLabel("Setting"), 0, 1)
        layout.addWidget(QLabel("Value"), 0, 2)

        for row, (key, label, _query, _setter, default) in enumerate(NETWORK_FIELDS, start=1):
            checkbox = QCheckBox()
            checkbox.setChecked(key == "ip")
            self.write_enable[key] = checkbox
            layout.addWidget(checkbox, row, 0, Qt.AlignCenter)

            layout.addWidget(QLabel(label), row, 1)

            field = QLineEdit(default)
            self.new_fields[key] = field
            layout.addWidget(field, row, 2)

        button_row = QHBoxLayout()
        self.apply_button = QPushButton("Apply Configuration")
        self.apply_button.setObjectName("dangerButton")
        self.apply_button.clicked.connect(self._on_apply_configuration)
        button_row.addWidget(self.apply_button)

        self.reset_button = QPushButton("Reset Controller")
        self.reset_button.setObjectName("secondaryButton")
        self.reset_button.clicked.connect(self._on_reset_controller)
        button_row.addWidget(self.reset_button)

        self.defaults_button = QPushButton("Load Factory Defaults")
        self.defaults_button.setObjectName("secondaryButton")
        self.defaults_button.clicked.connect(self._on_load_defaults)
        button_row.addWidget(self.defaults_button)

        button_row.addStretch(1)
        layout.addLayout(button_row, len(NETWORK_FIELDS) + 1, 0, 1, 3)

        note = QLabel(
            "Settings are stored in non-volatile memory. Reset the controller and read the "
            "configuration back to confirm the values actually persisted."
        )
        note.setStyleSheet("color: #6b7280; font-size: 11px;")
        note.setWordWrap(True)
        layout.addWidget(note, len(NETWORK_FIELDS) + 2, 0, 1, 3)

        layout.setColumnStretch(2, 1)
        return group

    def _build_log_group(self) -> QGroupBox:
        group = QGroupBox("Communication log")
        layout = QVBoxLayout(group)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(QFont("Consolas", 9))
        layout.addWidget(self.log_view)

        clear_button = QPushButton("Clear Log")
        clear_button.setObjectName("secondaryButton")
        clear_button.clicked.connect(self.log_view.clear)
        clear_row = QHBoxLayout()
        clear_row.addStretch(1)
        clear_row.addWidget(clear_button)
        layout.addLayout(clear_row)

        return group

    def _apply_styles(self):
        """Apply stylesheet"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f5f5f5;
            }
            QGroupBox {
                color: #2c3e50;
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 3px;
            }
            QPushButton {
                background-color: #3498db;
                color: white;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
            QPushButton:pressed {
                background-color: #1f618d;
            }
            QPushButton:disabled {
                background-color: #b0bec5;
            }
            QPushButton#secondaryButton {
                background-color: #7f8c8d;
            }
            QPushButton#secondaryButton:hover {
                background-color: #6c7a7a;
            }
            QPushButton#dangerButton {
                background-color: #e67e22;
            }
            QPushButton#dangerButton:hover {
                background-color: #ca6f1e;
            }
            QLineEdit, QComboBox, QDoubleSpinBox {
                background-color: white;
                border: 1px solid #bdc3c7;
                border-radius: 4px;
                padding: 5px;
                color: #2c3e50;
            }
            QLineEdit:read-only {
                background-color: #ecf0f1;
                color: #34495e;
            }
            QPlainTextEdit {
                background-color: #ffffff;
                border: 1px solid #bdc3c7;
                border-radius: 4px;
                color: #2c3e50;
            }
            QLabel {
                color: #2c3e50;
                font-weight: normal;
            }
        """)

    # -- Port handling ------------------------------------------------------------------

    def _refresh_ports(self):
        """Repopulate the COM port list from the OS."""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        if not ports:
            self.statusBar.showMessage("No serial ports found")
            return
        for info in ports:
            self.port_combo.addItem(f"{info.device} — {info.description}", info.device)
        self.statusBar.showMessage(f"Found {len(ports)} serial port(s)")

    def _selected_port(self) -> Optional[str]:
        if self.port_combo.count() == 0:
            return None
        return self.port_combo.currentData()

    # -- Task dispatch ------------------------------------------------------------------

    def _set_controls_enabled(self, enabled: bool):
        for button in (self.read_button, self.apply_button, self.reset_button,
                       self.refresh_button, self.copy_button, self.defaults_button):
            button.setEnabled(enabled)

    def _start_task(self, commands: List[SerialCommand], on_success, description: str):
        """Launch a serial task. Refuses to start if one is already running."""
        if self.worker is not None and self.worker.isRunning():
            QMessageBox.warning(self, "Busy", "A serial operation is already running.")
            return

        port_name = self._selected_port()
        if port_name is None:
            QMessageBox.warning(self, "No port", "Select a COM port first.")
            return

        self._append_log(f"=== {description} on {port_name} ===")
        self._set_controls_enabled(False)
        self.statusBar.showMessage(description)

        self.worker = SerialTaskWorker(port_name, commands, self.timeout_spin.value())
        self.worker.log_message.connect(self._append_log)
        self.worker.task_finished.connect(on_success)
        self.worker.task_finished.connect(lambda _: self._set_controls_enabled(True))
        self.worker.task_failed.connect(self._on_task_failed)
        self.worker.start()

    def _append_log(self, text: str):
        self.log_view.appendPlainText(text)

    def _on_task_failed(self, message: str):
        self._set_controls_enabled(True)
        self.statusBar.showMessage("Operation failed")
        QMessageBox.critical(self, "Operation failed", message)

    # -- Read ---------------------------------------------------------------------------

    def _on_read_configuration(self):
        """Query every network field. One unsupported command does not abort the rest."""
        for field in self.current_fields.values():
            field.clear()
            field.setPlaceholderText("reading…")

        commands = [
            SerialCommand(key, label, query, expect_ack=False, abort_on_failure=False)
            for key, label, query, _setter, _default in NETWORK_FIELDS
        ]
        self._start_task(commands, self._on_read_finished, "Reading network configuration")

    def _on_read_finished(self, results: dict):
        missing = []
        for key, label, _query, _setter, _default in NETWORK_FIELDS:
            field = self.current_fields[key]
            if key in results:
                field.setText(results[key])
            else:
                field.clear()
                field.setPlaceholderText("NO REPLY")
                missing.append(label)

        if missing:
            self.statusBar.showMessage(f"Read complete — no reply for: {', '.join(missing)}")
            QMessageBox.information(
                self, "Partial read",
                "These settings returned no valid reply:\n\n"
                + "\n".join(f"  - {name}" for name in missing)
                + "\n\nSM and GW exist only in the newer firmware command set. On older "
                  "firmware they are unsupported and the mask and gateway are fixed.\n\n"
                  "See the communication log for the exact bytes received."
            )
        else:
            self.statusBar.showMessage("Read complete — all settings retrieved")

    # -- Write --------------------------------------------------------------------------

    def _build_write_commands(self) -> List[SerialCommand]:
        """Validate the enabled fields and build the setter sequence.

        Raises ValueError with a field-specific message if any enabled value is invalid,
        so nothing is written when part of the input is bad.
        """
        commands = []
        for key, label, _query, setter, _default in NETWORK_FIELDS:
            if not self.write_enable[key].isChecked():
                continue

            text = self.new_fields[key].text().strip()
            if not text:
                raise ValueError(f"{label} is enabled for writing but empty.")

            if key == "port":
                try:
                    port_value = validate_port_number(text)
                except ValueError as exc:
                    raise ValueError(f"{label}: {exc}")
                payload = str(port_value)
            else:
                try:
                    payload = dotted_to_comma(text)
                except ValueError as exc:
                    raise ValueError(f"{label}: {exc}")

            commands.append(
                SerialCommand(key, label, f"{setter}{payload}",
                              expect_ack=True, abort_on_failure=True)
            )

        if not commands:
            raise ValueError("No settings are enabled for writing.")
        return commands

    def _on_apply_configuration(self):
        try:
            commands = self._build_write_commands()
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid input", str(exc))
            return

        summary = "\n".join(f"  {cmd.label}: {self.new_fields[cmd.key].text().strip()}"
                            for cmd in commands)
        confirm = QMessageBox.question(
            self, "Confirm write",
            "The following settings will be written to the controller:\n\n"
            f"{summary}\n\n"
            "Any open Ethernet connection to this controller will be lost. To reach it "
            "afterwards, the PC must be on the same subnet.\n\nProceed?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if confirm != QMessageBox.Yes:
            return

        self._start_task(commands, self._on_write_finished, "Writing network configuration")

    def _on_write_finished(self, results: dict):
        self.statusBar.showMessage("Write acknowledged — reset and read back to confirm")
        QMessageBox.information(
            self, "Write acknowledged",
            f"The controller acknowledged {len(results)} setting(s).\n\n"
            "The acknowledgement means the command was accepted, not that the value "
            "survived a power cycle. Press 'Reset Controller', then 'Read from Controller' "
            "to confirm."
        )

    # -- Reset --------------------------------------------------------------------------

    def _on_reset_controller(self):
        confirm = QMessageBox.question(
            self, "Confirm reset",
            "Send a software reset to the controller?\n\n"
            "Motion will stop and the controller will reinitialise.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if confirm != QMessageBox.Yes:
            return

        # The reset reply is 'RESET' rather than a masked value, so it is treated as a
        # non-aborting exchange: the log shows the raw bytes and the parse failure is
        # expected here.
        commands = [
            SerialCommand("reset", "Software reset", "reset",
                          expect_ack=False, abort_on_failure=False)
        ]
        self._start_task(commands, self._on_reset_finished, "Resetting controller")

    def _on_reset_finished(self, _results: dict):
        self.statusBar.showMessage("Reset sent — see log for the controller's reply")

    # -- Convenience --------------------------------------------------------------------

    def _on_copy_current_to_new(self):
        copied = 0
        for key, _label, _query, _setter, _default in NETWORK_FIELDS:
            value = self.current_fields[key].text().strip()
            if value:
                self.new_fields[key].setText(value)
                copied += 1
        self.statusBar.showMessage(f"Copied {copied} value(s) into the new configuration")

    def _on_load_defaults(self):
        for key, _label, _query, _setter, default in NETWORK_FIELDS:
            self.new_fields[key].setText(default)
        self.statusBar.showMessage("Factory default values loaded into the input fields")

    # -- Shutdown -----------------------------------------------------------------------

    def closeEvent(self, event):
        if self.worker is not None and self.worker.isRunning():
            self.worker.wait(3000)
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = NetworkConfigWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
