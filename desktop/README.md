On Windows, extract or clone the **whole project**, install Python 3.10 or newer
with Tcl/Tk support, open the **`desktop`** folder, and double-click **`run_desktop.bat`**.
The launcher finds `py` or `python`, creates a local `.venv`, installs the pinned
desktop dependencies, and opens the app. Internet access is needed for the first
installation. No administrator rights or manual environment activation is needed.
Keep the batch file inside `desktop/` in the project; it cannot run by itself.
It switches to the project folder above `desktop/` and reuses the root `.venv`.
If setup or startup fails, the console stays open so you can read the error. The launcher can
also be called from a command prompt and forwards arguments to `desktop.app`.

Alternatively, run from the repository root with Python 3.10 or newer:

```sh
python -m pip install -r desktop/requirements.txt
python -m desktop.app
```

Tk must be installed with Python (included in standard Windows/macOS installers;
on Debian/Ubuntu install `python3-tk`). Connect the **native MCU USB_OTG** connector,
not the USB/UART CH340 connector. Select its serial device and Connect. Native CDC
uses USB packets; the nominal 115200 baud setting is ignored by the firmware.
Windows uses the built-in CDC serial driver; Linux users need permission to open
the `/dev/ttyACM*` device. Refresh rescans ports.

The console retries HELLO every 250 ms while the native USB interface settles.
It waits for the firmware's version-1 reply before requesting parameters or
allowing ARM. A missing reply after three seconds closes the connection with a
clear error; retry Connect after checking the firmware and connector.

Connect loads the board's parameter definitions and ranges. Select a parameter,
enter its value in the displayed units and Apply. `mode` is 0 for PI/PID (set
`kd=0` for PI), 1 for finite control set MPC, or 2 for continuous control set MPC.
Stop before editing any parameter except `reference`. Firmware performs the
authoritative validation, including constraints between parameters. Parameters
are held in RAM and restored to defaults after reset. Save parameters writes the
current values and units to a JSON record on the laptop; it does not write MCU
flash or automatically restore settings. The table refreshes every two seconds
to reflect OLED button edits.

Check measured bus voltage, sensor scaling and configured limits before ARM.
STOP disables output. The GUI sends a heartbeat every 0.5 s and asks the board
to stop if telemetry becomes stale. Closing or disconnecting also requests STOP;
the firmware's independent heartbeat timeout covers a cable loss or crashed GUI.
Fault bits are shown in hexadecimal; see `docs/protocol.md` for the firmware's
fault definitions. CLEAR never arms the output. Connecting sends HELLO to open a
new firmware session; after a USB interruption, remove the fault cause, clear the
latched fault and explicitly ARM again. Reconnection never restores arming.

The plot displays the latest 15 s. Data retention is bounded at 60,000 received
samples (approximately 10 minutes at 100 Hz); CSV exports exactly that retained
window, including raw MCU timestamps and controller diagnostics. Clear plot
clears that recording window. MCU microsecond timer rollover is unwrapped for
plotting. Device reset clears the plot to keep separate sessions distinguishable.
Telemetry is decimated by firmware and does not capture every switching edge.

For a demonstration without hardware:

```sh
make host
python -m desktop.app --demo
```

This uses `build/libsimulation.so`, built from the **same C controller source** as
the firmware. The simulated 24 V bus, exact RCRC model, load dividers and configured
bridge drops form an ideal modeled plant. It verifies software interaction, not
USB wiring, switching loss, ADC calibration or physical hardware behavior. Choose
ARM to start, STOP to change mode, and ARM again to compare the controllers.
The simulation library is a native shared library: on Windows build an equivalent
DLL using the two C sources and pass `--library path/to/simulation.dll`.
