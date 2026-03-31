from __future__ import annotations

import queue

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from data import DataHistory, DeadReckoner, IMUSample
from diagnostics_widget import DiagnosticsWidget
from imu_serial import SerialIMUReader
from opengl_widget import Orientation3DWidget
from plot_widget import PlotWidget


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("IMU Visualizer")
        self.resize(1300, 900)

        self.reader = SerialIMUReader()
        self.history = DataHistory(maxlen=900)
        self.reckoner = DeadReckoner()

        self.orientation_widget = Orientation3DWidget()
        self.plot_widget = PlotWidget()
        self.diagnostics_widget = DiagnosticsWidget(self._reset_tracking)

        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "230400", "460800"])
        self.connect_btn = QPushButton("Connect")
        self.refresh_btn = QPushButton("Refresh Ports")
        self.status_label = QLabel("Disconnected")

        self.connect_btn.clicked.connect(self._toggle_connection)
        self.refresh_btn.clicked.connect(self._refresh_ports)

        control_layout = QHBoxLayout()
        control_layout.addWidget(QLabel("Port:"))
        control_layout.addWidget(self.port_combo)
        control_layout.addWidget(QLabel("Baud:"))
        control_layout.addWidget(self.baud_combo)
        control_layout.addWidget(self.refresh_btn)
        control_layout.addWidget(self.connect_btn)
        control_layout.addWidget(self.status_label)
        control_layout.addStretch(1)

        tabs = QTabWidget()
        tabs.addTab(self.orientation_widget, "3D Orientation")
        tabs.addTab(self.plot_widget, "Live Plots")
        tabs.addTab(self.diagnostics_widget, "Diagnostics")

        root = QWidget()
        root_layout = QVBoxLayout()
        root_layout.addLayout(control_layout)
        root_layout.addWidget(tabs)
        root.setLayout(root_layout)
        self.setCentralWidget(root)

        self._refresh_ports()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._drain_samples)
        self.timer.start(30)

    def closeEvent(self, event) -> None:
        self.reader.disconnect()
        super().closeEvent(event)

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        ports = self.reader.list_ports()
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)

    def _toggle_connection(self) -> None:
        if self.reader.is_connected:
            self.reader.disconnect()
            self.connect_btn.setText("Connect")
            self.status_label.setText("Disconnected")
            return

        port = self.port_combo.currentText()
        if not port:
            self.status_label.setText("No COM port")
            return

        baud = int(self.baud_combo.currentText())
        try:
            self.reader.connect(port=port, baudrate=baud)
            self.connect_btn.setText("Disconnect")
            self.status_label.setText(f"Connected: {port}")
        except Exception as exc:
            self.status_label.setText(f"Connect failed: {exc}")

    def _reset_tracking(self) -> None:
        self.reckoner.reset()

    def _drain_samples(self) -> None:
        updated = False
        while True:
            try:
                sample = self.reader.samples.get_nowait()
            except queue.Empty:
                break

            sample.roll_pitch_yaw_deg = sample.quaternion.to_euler_deg()
            sample = self.reckoner.update(sample)
            self.history.append(sample)
            self._update_widgets(sample)
            updated = True

        if updated:
            self.plot_widget.update_history(self.history)

    def _update_widgets(self, sample: IMUSample) -> None:
        self.orientation_widget.update_quaternion(sample.quaternion)
        self.diagnostics_widget.update_sample(sample)
