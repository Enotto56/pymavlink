# Telemetry monitor (Windows quickstart)

This guide explains how to prepare a Windows PC and run `examples/telemetry_monitor.py`, a console script that connects to a MAVLink autopilot over a COM port and prints basic telemetry (mode, position, altitude, airspeed, battery).

## 1. Install prerequisites
1. Install **Python 3.9+** from [python.org](https://www.python.org/downloads/windows/) and check that `python`/`pip` are in `PATH` via `python --version`.
2. Ensure your autopilot's USB-to-serial driver is installed (Windows usually installs it automatically; if not, install the vendor's driver).

## 2. Get the code with all needed dialects
Use the PyPI package (recommended, includes generated MAVLink dialect modules):
```powershell
python -m pip install --upgrade pip
python -m pip install pymavlink
```
If you want to use the repository version instead, install it in editable mode (this also builds the dialect modules during installation):
```powershell
cd path\to\pymavlink
python -m pip install -e .
```

## 3. Find your COM port
1. Connect the flight controller via USB.
2. Open **Device Manager → Ports (COM & LPT)** and note the port name (e.g., `COM3`).
3. Use the baud rate configured on the flight controller (common values: `57600`, `115200`).

## 4. Run the telemetry monitor
From the repository root (or anywhere if `pymavlink` is installed):
```powershell
python examples\telemetry_monitor.py COM3 --baud 57600
```
Options you may need:
- `--source-system` / `--source-component` — IDs for messages you send (defaults: 255/190).
- `--heartbeat-timeout` — seconds to wait for the first heartbeat before giving up (default: 30).

When a heartbeat is received, the script prints lines like:
```
[12:34:56] LOITER (ARMED) | lat: 47.397742, lon: 8.545594, rel alt: 10.2 m | alt: 10.3 m, airspeed: 5.4 m/s | voltage: 15.60 V, current: 3.21 A, remaining: 78%
```
Press **Ctrl+C** to stop.

## 5. Common checks if nothing happens
- Verify the COM port and baud rate match your autopilot settings.
- Confirm the autopilot is powered and sending MAVLink heartbeats.
- If you see import errors about missing dialects, ensure you installed via `pip install pymavlink` or `pip install -e .` instead of just cloning the repo without installing.
- If installation from the repository fails with errors like `No XML message definitions found` or `FileNotFoundError ... ardupilotmega.xml`, download the `message_definitions` folder from the [`mavlink/mavlink`](https://github.com/mavlink/mavlink) project. Then set the environment variable to point at it before reinstalling:
  ```powershell
  set MDEF=C:\Users\<you>\path\to\mavlink\message_definitions
  python -m pip install -e .
  ```
  This lets the installer generate the dialect files locally and resolves the missing-definition error.
