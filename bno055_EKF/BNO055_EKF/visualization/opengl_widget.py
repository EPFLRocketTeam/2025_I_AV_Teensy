from __future__ import annotations

import numpy as np
from PyQt6.QtWidgets import QVBoxLayout, QWidget
import pyqtgraph.opengl as gl

from data import Quaternion


class Orientation3DWidget(QWidget):
    def __init__(self) -> None:
        super().__init__()

        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=8.0, elevation=18.0, azimuth=45.0)

        self.axis = gl.GLAxisItem()
        self.axis.setSize(x=2.0, y=2.0, z=2.0)
        self.view.addItem(self.axis)

        grid = gl.GLGridItem()
        grid.setSize(x=10.0, y=10.0)
        grid.setSpacing(x=1.0, y=1.0)
        self.view.addItem(grid)

        self.cube_mesh = self._create_cube_mesh()
        self.view.addItem(self.cube_mesh)

        layout = QVBoxLayout()
        layout.addWidget(self.view)
        self.setLayout(layout)

    def _create_cube_mesh(self) -> gl.GLMeshItem:
        vertices = np.array(
            [
                [-0.5, -0.5, -0.5],
                [0.5, -0.5, -0.5],
                [0.5, 0.5, -0.5],
                [-0.5, 0.5, -0.5],
                [-0.5, -0.5, 0.5],
                [0.5, -0.5, 0.5],
                [0.5, 0.5, 0.5],
                [-0.5, 0.5, 0.5],
            ],
            dtype=np.float32,
        )
        faces = np.array(
            [
                [0, 1, 2], [0, 2, 3],
                [4, 5, 6], [4, 6, 7],
                [0, 1, 5], [0, 5, 4],
                [1, 2, 6], [1, 6, 5],
                [2, 3, 7], [2, 7, 6],
                [3, 0, 4], [3, 4, 7],
            ],
            dtype=np.int32,
        )
        return gl.GLMeshItem(
            vertexes=vertices,
            faces=faces,
            smooth=False,
            drawEdges=True,
            edgeColor=(0.08, 0.08, 0.08, 1.0),
            color=(0.1, 0.7, 0.9, 0.65),
            shader="shaded",
        )

    def update_quaternion(self, q: Quaternion) -> None:
        roll, pitch, yaw = q.to_euler_deg()
        self.cube_mesh.resetTransform()
        self.cube_mesh.rotate(yaw, 0, 0, 1)
        self.cube_mesh.rotate(pitch, 0, 1, 0)
        self.cube_mesh.rotate(roll, 1, 0, 0)
