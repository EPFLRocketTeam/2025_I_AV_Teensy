# IMU Visualizer - Launch Guide

This desktop app connects to the Teensy over serial and visualizes IMU data in real time.

## What You Get

- 3D orientation view (quaternion-driven)
- Live plots for accelerometer, gyroscope, magnetometer, and Euler angles
- Diagnostics panel with quaternion, RPY, velocity, and position

## Prerequisites

- Teensy firmware is built and uploaded from this project
- Teensy connected by USB
- Python virtual environment exists at `.venv`

## 1. Start in Project Root

Open a terminal in:

`C:\ERT\2025_I_AV_Teensy\bno055_EKF\BNO055_EKF`

## 2. Install Dependencies (first time only)

```powershell
.\.venv\Scripts\python.exe -m pip install -r visualization\requirements.txt
```

## 3. Launch the GUI

```powershell
.\.venv\Scripts\python.exe visualization\app.py
```

## 4. Connect to Teensy in the App

- Port: `COM29` (or your current Teensy COM port)
- Baud: `115200`
- Click `Connect`

When connected, the app sends `j` to enable JSON streaming.

## 5. Stop

- Close the GUI window, or
- Press `Ctrl+C` in the terminal where the app is running

## Troubleshooting

### App opens but no data updates

1. Verify Teensy COM port:

```powershell
& "C:\Users\aymer\.platformio\penv\Scripts\platformio.exe" device list
```

2. Ensure no other app is holding the port (PlatformIO monitor, serial monitor, another GUI instance).

3. Reconnect in the GUI after selecting the correct COM port.

### COM port access denied

- Close any terminal running serial monitor.
- Close duplicate visualizer instances.
- Unplug/replug Teensy, then reconnect.

### Quick serial sanity check

```powershell
.\.venv\Scripts\python.exe -c "import serial,time; s=serial.Serial('COM29',115200,timeout=1); time.sleep(0.3); s.write(b'j\n'); print(s.readline().decode('utf-8','ignore').strip()); s.write(b's\n'); s.close()"
```

A valid data line starts with `{` and contains `q`, `gyro`, `accel`, `mag`, and `time_us`.
