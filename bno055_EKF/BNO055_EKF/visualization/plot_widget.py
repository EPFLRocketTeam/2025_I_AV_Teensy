from __future__ import annotations

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt6.QtWidgets import QVBoxLayout, QWidget

from data import DataHistory


class PlotWidget(QWidget):
    def __init__(self) -> None:
        super().__init__()

        self.fig = Figure(figsize=(10, 8), dpi=100)
        self.canvas = FigureCanvas(self.fig)
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        self.ax_acc = self.fig.add_subplot(411)
        self.ax_gyro = self.fig.add_subplot(412)
        self.ax_mag = self.fig.add_subplot(413)
        self.ax_ang = self.fig.add_subplot(414)

        self.fig.tight_layout(pad=2.0)

    def update_history(self, hist: DataHistory) -> None:
        d = hist.as_lists()
        t = d["time_s"]
        if not t:
            return

        self.ax_acc.clear()
        self.ax_gyro.clear()
        self.ax_mag.clear()
        self.ax_ang.clear()

        self.ax_acc.plot(t, d["accel_x"], "r", label="Ax")
        self.ax_acc.plot(t, d["accel_y"], "g", label="Ay")
        self.ax_acc.plot(t, d["accel_z"], "b", label="Az")
        self.ax_acc.set_ylabel("Accel")
        self.ax_acc.legend(loc="upper right")

        self.ax_gyro.plot(t, d["gyro_x"], "r", label="Gx")
        self.ax_gyro.plot(t, d["gyro_y"], "g", label="Gy")
        self.ax_gyro.plot(t, d["gyro_z"], "b", label="Gz")
        self.ax_gyro.set_ylabel("Gyro")
        self.ax_gyro.legend(loc="upper right")

        self.ax_mag.plot(t, d["mag_x"], "r", label="Mx")
        self.ax_mag.plot(t, d["mag_y"], "g", label="My")
        self.ax_mag.plot(t, d["mag_z"], "b", label="Mz")
        self.ax_mag.set_ylabel("Mag")
        self.ax_mag.legend(loc="upper right")

        self.ax_ang.plot(t, d["roll"], "r", label="Roll")
        self.ax_ang.plot(t, d["pitch"], "g", label="Pitch")
        self.ax_ang.plot(t, d["yaw"], "b", label="Yaw")
        self.ax_ang.set_ylabel("Deg")
        self.ax_ang.set_xlabel("Time [s]")
        self.ax_ang.legend(loc="upper right")

        self.canvas.draw_idle()
