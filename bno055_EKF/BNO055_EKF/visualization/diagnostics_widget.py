from __future__ import annotations

from PyQt6.QtWidgets import QFormLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from data import IMUSample


class DiagnosticsWidget(QWidget):
    def __init__(self, on_reset_tracking) -> None:
        super().__init__()
        self._on_reset_tracking = on_reset_tracking

        self.q_label = QLabel("-")
        self.rpy_label = QLabel("-")
        self.accel_label = QLabel("-")
        self.gyro_label = QLabel("-")
        self.mag_label = QLabel("-")
        self.vel_label = QLabel("-")
        self.pos_label = QLabel("-")
        self.time_label = QLabel("-")

        form = QFormLayout()
        form.addRow("Quaternion:", self.q_label)
        form.addRow("Roll/Pitch/Yaw [deg]:", self.rpy_label)
        form.addRow("Accel:", self.accel_label)
        form.addRow("Gyro:", self.gyro_label)
        form.addRow("Mag:", self.mag_label)
        form.addRow("Velocity [m/s]:", self.vel_label)
        form.addRow("Position [m]:", self.pos_label)
        form.addRow("Computation time [us]:", self.time_label)

        reset_btn = QPushButton("Reset Velocity/Position")
        reset_btn.clicked.connect(self._on_reset_tracking)

        layout = QVBoxLayout()
        layout.addLayout(form)
        layout.addWidget(reset_btn)
        layout.addStretch(1)
        self.setLayout(layout)

    def update_sample(self, s: IMUSample) -> None:
        self.q_label.setText(f"[{s.quaternion.w:.3f}, {s.quaternion.x:.3f}, {s.quaternion.y:.3f}, {s.quaternion.z:.3f}]")
        self.rpy_label.setText(f"[{s.roll_pitch_yaw_deg[0]:.1f}, {s.roll_pitch_yaw_deg[1]:.1f}, {s.roll_pitch_yaw_deg[2]:.1f}]")
        self.accel_label.setText(f"[{s.accel[0]:.3f}, {s.accel[1]:.3f}, {s.accel[2]:.3f}]")
        self.gyro_label.setText(f"[{s.gyro[0]:.3f}, {s.gyro[1]:.3f}, {s.gyro[2]:.3f}]")
        self.mag_label.setText(f"[{s.mag[0]:.3f}, {s.mag[1]:.3f}, {s.mag[2]:.3f}]")
        self.vel_label.setText(f"[{s.velocity_mps[0]:.3f}, {s.velocity_mps[1]:.3f}, {s.velocity_mps[2]:.3f}]")
        self.pos_label.setText(f"[{s.position_m[0]:.3f}, {s.position_m[1]:.3f}, {s.position_m[2]:.3f}]")
        self.time_label.setText(str(s.comp_us))
