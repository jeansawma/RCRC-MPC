# USB commands and OLED menu

The firmware is a native USB full-speed CDC ACM device connected to the MCU board's
**USB_OTG** connector (PA11/PA12), not the separate CH340 USB/UART connector. CDC
baud rate is ignored by the USB firmware; the desktop requests 115200 for operating
system compatibility. Each command is ASCII terminated by LF. CRLF is also accepted.
Each response is one JSON object terminated by LF. Protocol version is 1. Commands
are case-sensitive. Send one command at a time and wait for its reply where applicable.

| Command | Result |
| --- | --- |
| `HELLO` | `type:hello`, `protocol:1`, device and firmware identifiers |
| `GET` | A stream of `type:param` records, followed by `type:params_end` |
| `SET name value` | `type:ack`, `command:SET`, `name`, accepted `value` |
| `ARM` | Explicitly enable closed-loop control after all interlocks pass |
| `STOP` | Immediately disable the L298 stage and reset controller history |
| `CLEAR` | Clear application faults while stopped, only after their cause has gone |
| `PING` | Heartbeat; `type:ack`, `command:PING` |

Errors use `{"type":"error","message":"..."}`. Every parameter record carries
`name`, `value`, `min`, `max`, `unit`, and `integer`. Use metadata rather than hardcoded
desktop ranges. `mode` is 0 for PI/PID (`kd=0` gives PI), 1 for finite-control-set MPC,
and 2 for continuous-control-set MPC with a QP. R/C values are in ohms/farads, the
sample interval is `sampling_us`, derivative filter time is in seconds, and `slew`
is maximum duty change **per control sample**. MPC voltage-error weights are applied
to volts. All numeric values must be finite and all input must be consumed by parsing.

Parameters other than `reference` require `STOP`. Changes are validated against a
temporary configuration before acceptance; invalid changes leave the active
configuration intact. The model/QP workspace is rebuilt before restarting. A
live reference change must be attainable at the currently measured bus voltage.
Other parameter changes can temporarily make an old reference unattainable; `ARM`
will reject that combination until the reference is corrected. FCS requires
`duty_min=0`, `duty_max=1`, `slew=1` because its outputs are actual binary switch
states. PI/CSMPC can use fractional duty and slew constraints.

`gain1`, `gain2`, `gainbus` multiply the measured physical voltages after the
board's nominal 3.3 V / 16-bit ADC and 51k/4.7k divider conversion. `offset1`,
`offset2`, `offsetbus` add volts after the gain. Calibrate with a known voltmeter
while the stage is stopped. A calibration change invalidates the old reading;
one new ADC sample is required before arming. Parameters are volatile RAM settings
and reset to firmware defaults after power cycle/reset; use the desktop export to
save a parameter profile on the laptop. Flash writes are not performed in the control loop.

Telemetry is decimated to at most 100 Hz independently of control sampling. Each
`type:sample` record contains `t_us` (32-bit microsecond timestamp, wraps every
71.6 minutes), `v1`, `v2` (controlled output), `vbus`, `reference`, `duty`, `mode`,
`armed`, `fault`, `qp_residual`, `qp_iterations`, `sample_count`, and `platform_fault`
(raw platform diagnostic bits; see `platform.md`). Invalid sensor
values are JSON `null`, never NaN. Plotting/USB congestion may drop telemetry;
control sampling continues independently. The GUI must send a valid command,
normally `PING`, at least every 2 seconds while armed. It sends one every 0.5 seconds.
Closing/disconnecting a host disables the stage even if STOP cannot be delivered.
The transport also tracks USB configuration/DTR sessions and missing SOF frames:
closing the serial port, reset, or cable loss invalidates the current session.
Reconnect never preserves arming, even within the 2-second heartbeat interval.
Send `HELLO` in the new session, remove/clear any fault, then explicitly `ARM`.
Queued commands from an old USB session are discarded by the transport. The firmware
does not auto-arm on USB connection or fault clear.

| Fault bit | Meaning |
| --- | --- |
| 1 | Invalid/nonfinite or implausibly negative voltage sample |
| 2 | Either capacitor voltage or DC bus exceeds its configured limit |
| 4 | Bus below 1 V during an armed run |
| 8 | Controller/QP failed or generated an invalid duty |
| 16 | No valid host command for more than 2 seconds while armed |
| 32 | Platform ADC, timing or initialization fault; inspect platform status/reset as needed |
| 64 | ADC sample older than max(5 ms, three sampling intervals) |
| 128 | Invalid controller configuration or failed sampling initialization |

Fault bits latch and immediately disable the stage. Remove the cause, wait for fresh
valid samples, send `CLEAR`, then explicitly `ARM` again. Irrecoverable platform
initialization faults require a reset. ADC/timing fault recovery is allowed only
while the power stage is disabled and the ADC is healthy. These software checks
supplement the power supply's current limit and bench measurements.

The SSD1306 menu shows controller, armed/fault state, both capacitor voltages,
reference, bus, duty, sample interval, and a selected parameter. UP/DOWN select any
of the USB parameters, LEFT/RIGHT decrease/increase it (5% steps for continuous
quantities, 0.1 V reference, 500 us sample interval). The same validation applies as
for USB edits; only reference can change while armed. **Holding LEFT+RIGHT stops
control immediately.** Arming and fault clear require the laptop, so a button press
cannot energize the stage accidentally. Buttons are logical active-high after the
74HC14. OLED presence is optional; its I2C transactions cannot block sampling.
