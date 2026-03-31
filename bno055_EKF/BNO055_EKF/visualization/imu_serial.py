from __future__ import annotations

import json
import queue
import threading
import time
from typing import Optional

import serial
import serial.tools.list_ports

from data import IMUSample, Quaternion


class SerialIMUReader:
    def __init__(self) -> None:
        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self.samples: "queue.Queue[IMUSample]" = queue.Queue(maxsize=5000)

    @staticmethod
    def list_ports() -> list[str]:
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port: str, baudrate: int = 115200) -> None:
        self.disconnect()
        self._serial = serial.Serial(port=port, baudrate=baudrate, timeout=0.2)
        time.sleep(0.2)
        self._serial.reset_input_buffer()
        self._serial.write(b"j")
        self._serial.write(b"\n")

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def disconnect(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None

        if self._serial and self._serial.is_open:
            try:
                self._serial.write(b"s")
                self._serial.write(b"\n")
            except Exception:
                pass
            self._serial.close()
        self._serial = None

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            ser = self._serial
            if ser is None:
                break
            try:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line or not line.startswith("{"):
                    continue
                payload = json.loads(line)
                sample = self._to_sample(payload)
                if sample is None:
                    continue
                try:
                    self.samples.put_nowait(sample)
                except queue.Full:
                    _ = self.samples.get_nowait()
                    self.samples.put_nowait(sample)
            except Exception:
                continue

    def _to_sample(self, payload: dict) -> Optional[IMUSample]:
        try:
            q = payload["q"]
            g = payload["gyro"]
            a = payload["accel"]
            m = payload["mag"]
            t = int(payload["time_us"])
            c = int(payload.get("comp_us", 0))
            return IMUSample(
                quaternion=Quaternion(float(q[0]), float(q[1]), float(q[2]), float(q[3])),
                gyro=(float(g[0]), float(g[1]), float(g[2])),
                accel=(float(a[0]), float(a[1]), float(a[2])),
                mag=(float(m[0]), float(m[1]), float(m[2])),
                time_us=t,
                comp_us=c,
            )
        except Exception:
            return None
