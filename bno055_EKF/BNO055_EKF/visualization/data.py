from __future__ import annotations

from dataclasses import dataclass, field
from collections import deque
import math
from typing import Deque, Dict, List, Tuple


@dataclass
class Quaternion:
    w: float
    x: float
    y: float
    z: float

    def normalize(self) -> "Quaternion":
        n = math.sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
        if n <= 1e-9:
            return Quaternion(1.0, 0.0, 0.0, 0.0)
        return Quaternion(self.w / n, self.x / n, self.y / n, self.z / n)

    def to_euler_deg(self) -> Tuple[float, float, float]:
        q = self.normalize()

        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    def to_rotation_matrix_3x3(self) -> List[List[float]]:
        q = self.normalize()
        w, x, y, z = q.w, q.x, q.y, q.z
        return [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]


@dataclass
class IMUSample:
    quaternion: Quaternion
    gyro: Tuple[float, float, float]
    accel: Tuple[float, float, float]
    mag: Tuple[float, float, float]
    time_us: int
    comp_us: int = 0
    roll_pitch_yaw_deg: Tuple[float, float, float] = field(default=(0.0, 0.0, 0.0))
    velocity_mps: Tuple[float, float, float] = field(default=(0.0, 0.0, 0.0))
    position_m: Tuple[float, float, float] = field(default=(0.0, 0.0, 0.0))


class DeadReckoner:
    def __init__(self) -> None:
        self._last_time_us: int | None = None
        self._vel = [0.0, 0.0, 0.0]
        self._pos = [0.0, 0.0, 0.0]

    def reset(self) -> None:
        self._last_time_us = None
        self._vel = [0.0, 0.0, 0.0]
        self._pos = [0.0, 0.0, 0.0]

    def update(self, sample: IMUSample) -> IMUSample:
        if self._last_time_us is None:
            self._last_time_us = sample.time_us
            sample.velocity_mps = tuple(self._vel)
            sample.position_m = tuple(self._pos)
            return sample

        dt = max((sample.time_us - self._last_time_us) / 1_000_000.0, 0.0)
        self._last_time_us = sample.time_us

        ax, ay, az = sample.accel
        for i, a in enumerate((ax, ay, az)):
            self._vel[i] += a * dt
            self._pos[i] += self._vel[i] * dt

        sample.velocity_mps = tuple(self._vel)
        sample.position_m = tuple(self._pos)
        return sample


class DataHistory:
    def __init__(self, maxlen: int = 1000) -> None:
        self.time_s: Deque[float] = deque(maxlen=maxlen)
        self.accel_x: Deque[float] = deque(maxlen=maxlen)
        self.accel_y: Deque[float] = deque(maxlen=maxlen)
        self.accel_z: Deque[float] = deque(maxlen=maxlen)
        self.gyro_x: Deque[float] = deque(maxlen=maxlen)
        self.gyro_y: Deque[float] = deque(maxlen=maxlen)
        self.gyro_z: Deque[float] = deque(maxlen=maxlen)
        self.mag_x: Deque[float] = deque(maxlen=maxlen)
        self.mag_y: Deque[float] = deque(maxlen=maxlen)
        self.mag_z: Deque[float] = deque(maxlen=maxlen)
        self.roll: Deque[float] = deque(maxlen=maxlen)
        self.pitch: Deque[float] = deque(maxlen=maxlen)
        self.yaw: Deque[float] = deque(maxlen=maxlen)

    def append(self, sample: IMUSample) -> None:
        t = sample.time_us / 1_000_000.0
        self.time_s.append(t)

        ax, ay, az = sample.accel
        gx, gy, gz = sample.gyro
        mx, my, mz = sample.mag
        roll, pitch, yaw = sample.roll_pitch_yaw_deg

        self.accel_x.append(ax)
        self.accel_y.append(ay)
        self.accel_z.append(az)
        self.gyro_x.append(gx)
        self.gyro_y.append(gy)
        self.gyro_z.append(gz)
        self.mag_x.append(mx)
        self.mag_y.append(my)
        self.mag_z.append(mz)
        self.roll.append(roll)
        self.pitch.append(pitch)
        self.yaw.append(yaw)

    def as_lists(self) -> Dict[str, List[float]]:
        return {
            "time_s": list(self.time_s),
            "accel_x": list(self.accel_x),
            "accel_y": list(self.accel_y),
            "accel_z": list(self.accel_z),
            "gyro_x": list(self.gyro_x),
            "gyro_y": list(self.gyro_y),
            "gyro_z": list(self.gyro_z),
            "mag_x": list(self.mag_x),
            "mag_y": list(self.mag_y),
            "mag_z": list(self.mag_z),
            "roll": list(self.roll),
            "pitch": list(self.pitch),
            "yaw": list(self.yaw),
        }
