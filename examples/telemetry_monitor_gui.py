"""Simple GUI telemetry monitor using pymavlink.

This script provides a minimal Tkinter interface to select a serial COM port
and baud rate, connect/disconnect, and display basic telemetry including
mode/armed state, position, altitude, airspeed, battery, and roll/pitch angles.
"""

import datetime
import importlib.util
import queue
import threading
import tkinter as tk
from tkinter import ttk
from typing import Optional

from pymavlink import mavutil

if importlib.util.find_spec("serial.tools.list_ports") is not None:
    import serial.tools.list_ports as list_ports
else:
    list_ports = None


class TelemetryPane:
    def __init__(self, parent: ttk.Frame, title: str) -> None:
        self.frame = ttk.LabelFrame(parent, text=title)

        self.master: Optional[mavutil.mavfile] = None
        self.recv_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.update_queue: "queue.Queue[dict]" = queue.Queue()

        self._build_layout()
        self._populate_ports()

    def _build_layout(self) -> None:
        padding = {"padx": 6, "pady": 3}

        control = ttk.Frame(self.frame)
        control.grid(row=0, column=0, sticky="ew")
        control.columnconfigure(1, weight=1)
        control.columnconfigure(3, weight=1)
        control.columnconfigure(5, weight=1)

        ttk.Label(control, text="Port:").grid(row=0, column=0, **padding)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(control, textvariable=self.port_var, width=12)
        self.port_combo.grid(row=0, column=1, sticky="ew", **padding)

        ttk.Label(control, text="Baud:").grid(row=0, column=2, **padding)
        self.baud_var = tk.StringVar(value="57600")
        self.baud_combo = ttk.Combobox(
            control,
            textvariable=self.baud_var,
            values=["57600", "115200", "38400", "9600"],
            width=10,
        )
        self.baud_combo.grid(row=0, column=3, sticky="ew", **padding)

        ttk.Label(control, text="Rate (Hz):").grid(row=0, column=4, **padding)
        self.rate_var = tk.StringVar(value="5")
        self.rate_combo = ttk.Combobox(
            control,
            textvariable=self.rate_var,
            values=["1", "2", "5", "10", "20"],
            width=8,
        )
        self.rate_combo.grid(row=0, column=5, sticky="ew", **padding)

        self.connect_button = ttk.Button(
            control, text="Connect", command=self._toggle_connection
        )
        self.connect_button.grid(row=0, column=6, **padding)

        self.rate_button = ttk.Button(
            control, text="Apply rate", command=self._apply_rate
        )
        self.rate_button.grid(row=1, column=4, columnspan=3, sticky="ew", **padding)

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(control, textvariable=self.status_var).grid(
            row=1, column=0, columnspan=4, sticky="w", **padding
        )

        telemetry_frame = ttk.Frame(self.frame)
        telemetry_frame.grid(row=1, column=0, sticky="nsew", padx=6, pady=4)
        telemetry_frame.columnconfigure(1, weight=1)

        self.telemetry_vars = {
            "mode": tk.StringVar(value="--"),
            "position": tk.StringVar(value="lat: ---, lon: ---, rel alt: ---"),
            "alt": tk.StringVar(value="alt: ---"),
            "airspeed": tk.StringVar(value="airspeed: ---"),
            "battery": tk.StringVar(
                value="voltage: ---, current: ---, remaining: ---%"
            ),
            "attitude": tk.StringVar(value="roll: ---, pitch: ---"),
        }

        labels = [
            ("Mode", "mode"),
            ("Position", "position"),
            ("Altitude", "alt"),
            ("Airspeed", "airspeed"),
            ("Battery", "battery"),
            ("Attitude", "attitude"),
        ]

        for idx, (label, key) in enumerate(labels):
            ttk.Label(telemetry_frame, text=f"{label}:").grid(
                row=idx, column=0, sticky="w", padx=4, pady=2
            )
            ttk.Label(telemetry_frame, textvariable=self.telemetry_vars[key]).grid(
                row=idx, column=1, sticky="w", padx=4, pady=2
            )

    def _populate_ports(self) -> None:
        ports = []
        if list_ports is not None:
            ports = [port.device for port in list_ports.comports()]
        if not ports:
            ports = ["COM3", "COM4", "/dev/ttyUSB0", "/dev/ttyACM0"]
        self.port_combo["values"] = ports
        if ports:
            self.port_var.set(ports[0])

    def _toggle_connection(self) -> None:
        if self.master is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self) -> None:
        port = self.port_var.get().strip()
        baud_str = self.baud_var.get().strip()
        rate_str = self.rate_var.get().strip()
        if not port:
            self.status_var.set("Select a port before connecting.")
            return
        try:
            baud = int(baud_str)
        except ValueError:
            self.status_var.set("Invalid baud rate.")
            return
        try:
            rate_hz = int(rate_str)
        except ValueError:
            self.status_var.set("Invalid rate (Hz).")
            return

        self.status_var.set(f"Connecting to {port} at {baud}...")
        self.connect_button.config(text="Disconnect", state="disabled")
        self.stop_event.clear()

        def _worker() -> None:
            try:
                self.master = mavutil.mavlink_connection(port, baud=baud)
                self.master.wait_heartbeat(timeout=30)
                self._request_stream_rate(rate_hz)
                self.update_queue.put(
                    {
                        "status": f"Connected to system {self.master.target_system} component {self.master.target_component}",
                    }
                )
                self._telemetry_loop()
            except Exception as exc:  # noqa: BLE001
                self.update_queue.put({"status": f"Connection failed: {exc}"})
                self.update_queue.put({"disconnect": True})

        self.recv_thread = threading.Thread(target=_worker, daemon=True)
        self.recv_thread.start()

    def _disconnect(self) -> None:
        self.stop_event.set()
        if self.master is not None:
            try:
                self.master.close()
            except Exception:  # noqa: BLE001
                pass
        self.master = None
        self.connect_button.config(text="Connect", state="normal")
        self.status_var.set("Disconnected")

    def _apply_rate(self) -> None:
        rate_str = self.rate_var.get().strip()
        try:
            rate_hz = int(rate_str)
        except ValueError:
            self.status_var.set("Invalid rate (Hz).")
            return

        if self.master is None:
            self.status_var.set("Connect first, then apply a rate.")
            return

        self._request_stream_rate(rate_hz)

    def _request_stream_rate(self, rate_hz: int) -> None:
        if self.master is None:
            return
        try:
            for stream_id in (
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
            ):
                self.master.mav.request_data_stream_send(
                    self.master.target_system,
                    self.master.target_component,
                    stream_id,
                    rate_hz,
                    1,
                )
            self.update_queue.put({"status": f"Requested telemetry at {rate_hz} Hz"})
        except Exception as exc:  # noqa: BLE001
            self.update_queue.put({"status": f"Failed to set rate: {exc}"})

    def _telemetry_loop(self) -> None:
        last_heartbeat: Optional[object] = None
        last_gps: Optional[object] = None
        last_sys_status: Optional[object] = None
        last_vfr: Optional[object] = None
        last_attitude: Optional[object] = None

        while not self.stop_event.is_set() and self.master is not None:
            msg = self.master.recv_match(
                type=[
                    "HEARTBEAT",
                    "GLOBAL_POSITION_INT",
                    "SYS_STATUS",
                    "VFR_HUD",
                    "ATTITUDE",
                ],
                blocking=True,
                timeout=1,
            )
            if msg is None:
                continue

            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                last_heartbeat = msg
            elif mtype == "GLOBAL_POSITION_INT":
                last_gps = msg
            elif mtype == "SYS_STATUS":
                last_sys_status = msg
            elif mtype == "VFR_HUD":
                last_vfr = msg
            elif mtype == "ATTITUDE":
                last_attitude = msg

            timestamp = datetime.datetime.now().strftime("%H:%M:%S")
            mode = self._describe_mode(last_heartbeat) if last_heartbeat else "--"
            position = self._format_position(last_gps)
            battery = self._format_battery(last_sys_status)
            alt = f"alt: {last_vfr.alt:.1f} m" if last_vfr else "alt: ---"
            airspeed = (
                f"airspeed: {last_vfr.airspeed:.1f} m/s" if last_vfr else "airspeed: ---"
            )
            attitude = self._format_attitude(last_attitude)

            self.update_queue.put(
                {
                    "timestamp": timestamp,
                    "mode": mode,
                    "position": position,
                    "battery": battery,
                    "alt": alt,
                    "airspeed": airspeed,
                    "attitude": attitude,
                }
            )

    def process_queue(self) -> None:
        while True:
            try:
                update = self.update_queue.get_nowait()
            except queue.Empty:
                break

            if update.get("disconnect"):
                self._disconnect()
                continue

            status = update.get("status")
            if status:
                self.status_var.set(status)
                if status.startswith("Connected"):
                    self.connect_button.config(text="Disconnect", state="normal")
                elif status.startswith("Connection failed") or status.startswith(
                    "Disconnected"
                ):
                    self.connect_button.config(text="Connect", state="normal")

            for key in ("mode", "position", "alt", "airspeed", "battery", "attitude"):
                if key in update:
                    text = update[key]
                    if "timestamp" in update:
                        text = f"[{update['timestamp']}] {text}" if key == "mode" else text
                    self.telemetry_vars[key].set(text)

    def _describe_mode(self, msg) -> str:
        base_mode = msg.base_mode
        custom_mode = msg.custom_mode
        mode_mapping = self.master.mode_mapping() if self.master else None
        if mode_mapping:
            mode = mode_mapping.get(custom_mode, str(custom_mode))
        else:
            mode = str(custom_mode)
        armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return f"{mode} ({'ARMED' if armed else 'DISARMED'})"

    def _format_position(self, msg) -> str:
        if msg is None:
            return "lat: ---, lon: ---, rel alt: ---"
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1000.0
        return f"lat: {lat:.6f}, lon: {lon:.6f}, rel alt: {alt:.1f} m"

    def _format_battery(self, msg) -> str:
        if msg is None:
            return "voltage: ---, current: ---, remaining: ---%"
        voltage = msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else None
        current = msg.current_battery / 100.0 if msg.current_battery != 65535 else None
        remaining = msg.battery_remaining if msg.battery_remaining != -1 else None
        parts = []
        parts.append(f"voltage: {voltage:.2f} V" if voltage is not None else "voltage: ---")
        parts.append(f"current: {current:.2f} A" if current is not None else "current: ---")
        parts.append(
            f"remaining: {remaining}%" if remaining is not None else "remaining: ---%"
        )
        return ", ".join(parts)

    def _format_attitude(self, msg) -> str:
        if msg is None:
            return "roll: ---, pitch: ---"
        roll_deg = msg.roll * 180.0 / 3.141592653589793
        pitch_deg = msg.pitch * 180.0 / 3.141592653589793
        return f"roll: {roll_deg:.1f}°, pitch: {pitch_deg:.1f}°"


class TelemetryMonitorUI:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Telemetry Monitor")

        container = ttk.Frame(root)
        container.grid(row=0, column=0, sticky="nsew")
        container.columnconfigure(0, weight=1)
        container.columnconfigure(1, weight=1)

        self.panes = [
            TelemetryPane(container, "Vehicle 1"),
            TelemetryPane(container, "Vehicle 2"),
        ]
        self.panes[0].frame.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
        self.panes[1].frame.grid(row=0, column=1, sticky="nsew", padx=8, pady=8)

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        self._process_queues()

    def _process_queues(self) -> None:
        for pane in self.panes:
            pane.process_queue()
        self.root.after(200, self._process_queues)


def main() -> None:
    root = tk.Tk()
    TelemetryMonitorUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
