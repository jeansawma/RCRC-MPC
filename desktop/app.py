"""Run from project root: python -m desktop.app [--demo]."""
from __future__ import annotations
import argparse
import json
from dataclasses import replace
import pathlib
import queue
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from tkinter import font as tkfont

from .protocol import History, Parameter, Sample, ProtocolError
from .transport import SerialTransport, DemoTransport

MODES = {0: "PI / PID", 1: "Finite control set MPC", 2: "Continuous control set MPC · QP"}


class Console:
    def __init__(self, root, demo=False, library=None):
        self.root = root
        self.library = library
        self.transport = None
        self.events = queue.Queue(maxsize=5000)
        self.parameters = {}
        self.history = History()
        self.latest = None
        self.connected = False
        self.handshake = False
        self.awaiting_disconnect = False
        self.last_ping = 0
        self.last_get = 0
        self.last_sample = 0
        self.last_plot = 0
        self.session = "Disconnected"
        self.font_family = tkfont.nametofont("TkDefaultFont").actual("family")
        self.root.title("RCRC Control Laboratory")
        self.root.geometry("1320x840")
        self.root.minsize(1060, 700)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        style = ttk.Style()
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("TFrame", background="#edf2f7")
        style.configure("TLabel", background="#edf2f7", foreground="#17283f")
        style.configure("Title.TLabel", font=(self.font_family, 20, "bold"))
        style.configure("Value.TLabel", font=(self.font_family, 23, "bold"))
        style.configure("Accent.TButton", foreground="#075d73", font=(self.font_family, 10, "bold"))
        style.configure("Stop.TButton", foreground="#a52c39", font=(self.font_family, 11, "bold"))
        self.build()
        self.refresh_ports()
        self.root.after(25, self.pump)
        if demo:
            self.root.after(100, self.connect_demo)

    def build(self):
        outer = ttk.Frame(self.root, padding=18)
        outer.pack(fill="both", expand=True)
        title = ttk.Frame(outer)
        title.pack(fill="x")
        ttk.Label(title, text="RCRC Control Laboratory", style="Title.TLabel").pack(side="left")
        self.source = tk.StringVar(value="STM32H723 · USB CDC")
        ttk.Label(title, textvariable=self.source).pack(side="right")
        connect = ttk.Frame(outer, padding=(0, 16, 0, 12))
        connect.pack(fill="x")
        ttk.Label(connect, text="USB device").pack(side="left", padx=(0, 8))
        self.port_var = tk.StringVar()
        self.ports = ttk.Combobox(connect, textvariable=self.port_var, width=37)
        self.ports.pack(side="left")
        ttk.Button(connect, text="Refresh", command=self.refresh_ports).pack(side="left", padx=4)
        self.connect_button = ttk.Button(connect, text="Connect", command=self.connect_serial)
        self.connect_button.pack(side="left", padx=4)
        self.demo_button = ttk.Button(connect, text="Demo simulation", command=self.connect_demo)
        self.demo_button.pack(side="left", padx=4)
        self.disconnect_button = ttk.Button(connect, text="Disconnect", command=self.disconnect, state="disabled")
        self.disconnect_button.pack(side="left", padx=4)
        ttk.Button(connect, text="Export CSV", command=self.export).pack(side="right")
        ttk.Button(connect, text="Save parameters", command=self.save_profile).pack(side="right", padx=5)
        ttk.Button(connect, text="Clear plot", command=self.clear_history).pack(side="right", padx=5)
        measurements = ttk.Frame(outer)
        measurements.pack(fill="x", pady=(0, 10))
        self.readouts = {}
        for index, (key, label) in enumerate((("v2", "RCRC OUTPUT / V"), ("reference", "REFERENCE / V"),
                                            ("v1", "FIRST CAPACITOR / V"), ("vbus", "DC BUS / V"),
                                            ("duty", "DUTY / %"))):
            panel = ttk.Frame(measurements, padding=8)
            panel.grid(row=0, column=index, sticky="nsew", padx=3)
            measurements.columnconfigure(index, weight=1)
            ttk.Label(panel, text=label).pack(anchor="w")
            variable = tk.StringVar(value="—")
            self.readouts[key] = variable
            ttk.Label(panel, textvariable=variable, style="Value.TLabel").pack(anchor="w")
        content = ttk.Panedwindow(outer, orient="horizontal")
        content.pack(fill="both", expand=True)
        chart = ttk.Frame(content)
        editor = ttk.Frame(content, padding=(14, 0, 0, 0), width=385)
        content.add(chart, weight=3)
        content.add(editor, weight=1)
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
        figure = Figure(figsize=(8, 5), dpi=100, facecolor="#ffffff")
        self.ax_voltage, self.ax_duty = figure.subplots(2, 1, sharex=True, gridspec_kw={"height_ratios": [3, 1]})
        figure.subplots_adjust(left=0.09, right=0.98, top=0.94, bottom=0.11, hspace=0.18)
        self.lines = {}
        for key, color, label, style in (("v2", "#087f8c", "Output v₂", "-"),
                                         ("v1", "#6c67af", "Capacitor v₁", "-"),
                                         ("reference", "#f08b31", "Reference", "--"),
                                         ("vbus", "#a8b1bb", "DC bus", ":")):
            self.lines[key], = self.ax_voltage.plot([], [], color=color, label=label, linestyle=style, linewidth=1.6)
        self.lines["duty"], = self.ax_duty.plot([], [], color="#284e72", linewidth=1.3, label="Duty")
        self.ax_voltage.set_ylabel("Voltage / V")
        self.ax_voltage.legend(loc="upper left", ncol=2, fontsize=8)
        self.ax_duty.set_ylabel("Duty / %")
        self.ax_duty.set_xlabel("Time / s")
        self.ax_duty.set_ylim(-2, 102)
        for axis in (self.ax_voltage, self.ax_duty):
            axis.grid(True, alpha=0.16)
            axis.spines[["top", "right"]].set_visible(False)
        self.canvas = FigureCanvasTkAgg(figure, master=chart)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        NavigationToolbar2Tk(self.canvas, chart, pack_toolbar=True)
        ttk.Label(editor, text="CONTROLLER & PARAMETERS", font=(self.font_family, 11, "bold")).pack(anchor="w", pady=(0, 8))
        self.state_text = tk.StringVar(value="Disconnected · power stage unavailable")
        ttk.Label(editor, textvariable=self.state_text, wraplength=350).pack(anchor="w", pady=(0, 10))
        controls = ttk.Frame(editor)
        controls.pack(fill="x", pady=(0, 12))
        self.arm_button = ttk.Button(controls, text="ARM", command=lambda: self.send("ARM"), style="Accent.TButton", state="disabled")
        self.arm_button.pack(side="left", fill="x", expand=True)
        self.stop_button = ttk.Button(controls, text="STOP", command=lambda: self.send("STOP"), style="Stop.TButton", state="disabled")
        self.stop_button.pack(side="left", fill="x", expand=True, padx=5)
        self.clear_button = ttk.Button(controls, text="Clear fault", command=lambda: self.send("CLEAR"), state="disabled")
        self.clear_button.pack(side="left", fill="x", expand=True)
        table_frame = ttk.Frame(editor)
        table_frame.pack(fill="both", expand=True)
        self.param_table = ttk.Treeview(table_frame, columns=("value", "unit"), show="tree headings", height=13, selectmode="browse")
        self.param_table.heading("#0", text="Parameter")
        self.param_table.heading("value", text="Value")
        self.param_table.heading("unit", text="Unit")
        self.param_table.column("#0", width=140, minwidth=90)
        self.param_table.column("value", width=100, minwidth=70)
        self.param_table.column("unit", width=45, minwidth=35)
        table_scroll = ttk.Scrollbar(table_frame, orient="vertical", command=self.param_table.yview)
        self.param_table.configure(yscrollcommand=table_scroll.set)
        table_scroll.pack(side="right", fill="y")
        self.param_table.pack(side="left", fill="both", expand=True)
        self.param_table.bind("<<TreeviewSelect>>", self.select_parameter)
        self.selected_text = tk.StringVar(value="Select a parameter above")
        ttk.Label(editor, textvariable=self.selected_text, wraplength=350).pack(anchor="w", pady=(10, 4))
        edit_row = ttk.Frame(editor)
        edit_row.pack(fill="x")
        self.value_text = tk.StringVar()
        self.value_entry = ttk.Entry(edit_row, textvariable=self.value_text)
        self.value_entry.pack(side="left", fill="x", expand=True)
        self.value_entry.bind("<Return>", lambda event: self.apply_parameter())
        self.apply_button = ttk.Button(edit_row, text="Apply", command=self.apply_parameter, state="disabled")
        self.apply_button.pack(side="left", padx=(5, 0))
        ttk.Label(editor, text="mode: 0 = PI/PID · 1 = FCS-MPC · 2 = CS-MPC\n"
                  "kd = 0 selects PI. Changes require STOP, except reference.\n"
                  "CSV exports the retained window (up to 60,000 samples).",
                  wraplength=350, foreground="#526578").pack(anchor="w", pady=(10, 5))
        self.solver_text = tk.StringVar(value="QP residual — · iterations —")
        ttk.Label(editor, textvariable=self.solver_text).pack(anchor="w")
        self.status = tk.StringVar(value="Connect native USB or choose Demo simulation. Outputs begin disabled.")
        ttk.Label(outer, textvariable=self.status, wraplength=1200, padding=(0, 12, 0, 0)).pack(anchor="w")

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            self.port_entries = list(list_ports.comports())
            self.ports["values"] = [entry.device for entry in self.port_entries]
            if not self.port_var.get() and self.port_entries:
                native = next((entry for entry in self.port_entries if entry.vid == 0x0483), self.port_entries[0])
                self.port_var.set(native.device)
        except ImportError:
            self.status.set("Install desktop dependencies with pip install -r desktop/requirements.txt")

    def begin(self, transport):
        if self.transport:
            return
        self.transport = transport
        self.connect_button.configure(state="disabled")
        self.demo_button.configure(state="disabled")
        self.status.set("Connecting…")
        transport.start()

    def connect_serial(self):
        port = self.port_var.get().strip()
        if not port:
            self.status.set("Select a USB serial device first.")
            return
        self.begin(SerialTransport(self.events, port))

    def connect_demo(self):
        self.begin(DemoTransport(self.events, self.library))

    def disconnect(self):
        if self.transport:
            self.transport.close()
            self.awaiting_disconnect = True
            self.status.set("Stopping output and disconnecting…")

    def send(self, command):
        if not self.transport or not self.connected:
            return
        if command == "ARM" and not self.handshake:
            self.status.set("Wait for the USB HELLO handshake before arming.")
            return
        try:
            self.transport.send(command)
        except (ValueError, queue.Full) as error:
            self.status.set(f"Command could not be sent: {error}")

    def select_parameter(self, event=None):
        selection = self.param_table.selection()
        if not selection:
            return
        parameter = self.parameters.get(selection[0])
        if parameter is None:
            return
        self.value_text.set(f"{parameter.value:.9g}")
        self.selected_text.set(f"{parameter.name} [{parameter.unit}] · {parameter.minimum:g} … {parameter.maximum:g}")

    def apply_parameter(self):
        selection = self.param_table.selection()
        if not selection:
            self.status.set("Select a parameter first.")
            return
        parameter = self.parameters[selection[0]]
        if self.latest and self.latest.armed and parameter.name != "reference":
            self.status.set("STOP before changing system or controller parameters.")
            return
        try:
            self.send(parameter.command(self.value_text.get()))
        except ProtocolError as error:
            self.status.set(str(error))

    def clear_history(self):
        self.history.clear()
        self.draw_plot()

    def export(self):
        if not self.history.samples:
            self.status.set("No samples to export.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV data", "*.csv")],
                                          initialfile="rcrc_measurements.csv")
        if path:
            try:
                with open(path, "w", newline="", encoding="utf-8") as output:
                    self.history.export(output)
                self.status.set(f"Exported {len(self.history.samples)} retained samples to {path}")
            except OSError as error:
                messagebox.showerror("CSV export failed", str(error))

    def save_profile(self):
        if not self.parameters:
            self.status.set("Connect and read parameters before saving a profile.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("Parameter profile", "*.json")],
                                          initialfile="rcrc_parameters.json")
        if path:
            try:
                profile = {"protocol": 1, "source": self.session,
                           "parameters": {name: {"value": item.value, "unit": item.unit}
                                          for name, item in self.parameters.items()}}
                with open(path, "w", encoding="utf-8") as output:
                    json.dump(profile, output, indent=2, allow_nan=False)
                    output.write("\n")
                self.status.set(f"Saved parameter record to {path}. Board settings remain volatile.")
            except OSError as error:
                messagebox.showerror("Parameter save failed", str(error))

    def handle(self, event):
        kind = event["type"]
        if kind == "connected":
            self.connected = True
            self.handshake = False
            self.awaiting_disconnect = False
            self.session = event["source"]
            self.source.set(self.session)
            self.parameters.clear()
            self.param_table.delete(*self.param_table.get_children())
            self.history.clear()
            self.latest = None
            self.last_sample = time.monotonic()
            self.last_ping = 0
            self.last_get = time.monotonic()
            self.arm_button.configure(state="disabled")
            for button in (self.stop_button, self.clear_button,
                           self.apply_button, self.disconnect_button):
                button.configure(state="normal")
            self.status.set("USB opened. Waiting for the firmware HELLO handshake…")
        elif kind == "disconnected":
            self.connected = False
            self.handshake = False
            self.transport = None
            self.awaiting_disconnect = False
            self.state_text.set("Disconnected · firmware timeout disables output")
            self.source.set("Disconnected")
            for button in (self.arm_button, self.stop_button, self.clear_button,
                           self.apply_button, self.disconnect_button):
                button.configure(state="disabled")
            self.connect_button.configure(state="normal")
            self.demo_button.configure(state="normal")
        elif kind == "param":
            parameter = Parameter.from_message(event)
            self.parameters[parameter.name] = parameter
            values = (f"{parameter.value:.6g}", parameter.unit)
            if self.param_table.exists(parameter.name):
                self.param_table.item(parameter.name, values=values)
            else:
                self.param_table.insert("", "end", iid=parameter.name, text=parameter.name, values=values)
        elif kind == "ack":
            if "name" in event and event["name"] in self.parameters:
                name = event["name"]
                parameter = replace(self.parameters[name], value=float(event["value"]))
                self.parameters[name] = parameter
                self.param_table.item(name, values=(f"{parameter.value:.6g}", parameter.unit))
                if self.param_table.selection() == (name,):
                    self.value_text.set(f"{parameter.value:.9g}")
                self.status.set(f"Applied {name} = {parameter.value:g} {parameter.unit}")
            elif event.get("command") != "PING":
                self.status.set(f"Board acknowledged {event.get('command', 'command')}")
        elif kind == "sample":
            sample = Sample.from_message(event)
            self.latest = sample
            self.last_sample = time.monotonic()
            self.history.append(sample)
            for key, variable in self.readouts.items():
                value = getattr(sample, key) * (100 if key == "duty" else 1)
                variable.set(f"{value:.2f}")
            state = "ARMED" if sample.armed else "STOPPED"
            fault = f" · FAULT 0x{sample.fault:08x}" if sample.fault else ""
            self.state_text.set(f"{state}{fault}\n{MODES[sample.mode]}")
            self.arm_button.configure(state="disabled" if sample.armed or sample.fault or not self.handshake else "normal")
            self.solver_text.set(f"QP residual {sample.qp_residual:.3g} · iterations {sample.qp_iterations}"
                                 f"\nPlatform fault 0x{int(event.get('platform_fault', 0)):02x}")
        elif kind in ("error", "parse_error", "transport_error"):
            self.status.set(f"{kind.replace('_', ' ').capitalize()}: {event.get('message', 'unknown error')}")
        elif kind == "hello":
            if event.get("protocol") != 1:
                self.status.set("Unsupported protocol version; disconnecting.")
                self.disconnect()
            else:
                self.handshake = True
                self.status.set("USB handshake complete. Check configuration and measured bus voltage before ARM.")

    def draw_plot(self):
        rows = list(self.history.samples)
        times = list(self.history.display_times)
        if times:
            end = times[-1]
            start = max(times[0], end - 15)
            # Bound rendering to the newest visible window, at most ~2k points.
            offset = next((index for index, value in enumerate(times) if value >= start), 0)
            stride = max(1, (len(rows) - offset) // 2000)
            rows, times = rows[offset::stride], times[offset::stride]
            for key, line in self.lines.items():
                line.set_data(times, [getattr(row, key) * (100 if key == "duty" else 1) for row in rows])
            self.ax_voltage.relim()
            self.ax_voltage.autoscale_view(scalex=False)
            self.ax_voltage.set_xlim(start, max(end, start + 1))
        else:
            for line in self.lines.values():
                line.set_data([], [])
            self.ax_voltage.set_xlim(0, 15)
            self.ax_voltage.set_ylim(0, 30)
        self.canvas.draw_idle()

    def pump(self):
        for _ in range(500):
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            try:
                self.handle(event)
            except (ProtocolError, ValueError, TypeError, KeyError) as error:
                self.status.set(f"Rejected board message: {error}")
                if event.get("type") == "sample":
                    self.arm_button.configure(state="disabled")
                    self.state_text.set(f"Invalid sensor telemetry · fault {event.get('fault', 0)}")
                    self.send("STOP")
        now = time.monotonic()
        if self.connected and not self.awaiting_disconnect:
            if now - self.last_ping >= 0.5:
                self.send("PING")
                self.last_ping = now
            if self.handshake and now - self.last_get >= 2:
                self.send("GET")
                self.last_get = now
            if now - self.last_sample > 1.5:
                self.send("STOP")
                self.state_text.set("Telemetry stale · STOP requested")
                self.arm_button.configure(state="disabled")
            if now - self.last_plot >= 0.12:
                self.draw_plot()
                self.last_plot = now
        self.root.after(25, self.pump)

    def close(self):
        if self.transport:
            self.transport.close()
            # Worker is daemon; board's heartbeat timeout is the final protection.
        self.root.after(250, self.root.destroy)


def main():
    parser = argparse.ArgumentParser(description="RCRC native USB control console")
    parser.add_argument("--demo", action="store_true", help="use C controllers and modeled plant without hardware")
    parser.add_argument("--library", type=pathlib.Path, help="path to host simulation shared library")
    arguments = parser.parse_args()
    root = tk.Tk()
    Console(root, demo=arguments.demo, library=arguments.library)
    root.mainloop()


if __name__ == "__main__":
    main()
