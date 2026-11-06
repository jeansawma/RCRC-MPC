# Verification and commissioning record

Date: 6 November 2026. The physical boards were not attached to this environment. Results below are software/build evidence, not a claim of measured hardware performance.

## Completed checks

| Check | Outcome |
| --- | --- |
| Cortex-M7 firmware build | Arm GNU GCC 14.2.1, hard float, `-O2`; ELF, BIN and Intel HEX generated |
| HEX checks | Every record checksum, EOF, flash address bounds, DTCM stack vector, Thumb reset vector and BIN correspondence checked by `tools/check_hex.py` |
| C controller suite | **19,944 assertions passed**, including exact-model properties, loaded equilibrium, PI antiwindup, nonfinite/config rejection, binary FCS, QP solutions/constraints and all three closed loops |
| Independent QP verification | **60 randomized cases** compared with SciPy SLSQP, horizons 2–16, varying sample periods/components/states/bus/slew limits; maximum first-action difference **1.10183×10⁻⁶ duty** |
| Independent model verification | Exact discrete matrices compared against SciPy matrix exponential, including measurement-divider loading |
| Application integration | Mocked board tests passed for safe startup, USB commands/metadata, transactional edits, calibration, voltage/bus/sensor/solver/platform/stale/heartbeat faults, clearing, OLED edits/STOP and timestamp rollover |
| USB session regression | Quick reconnect cannot preserve arming; `PING` cannot re-arm; new session requires HELLO and explicit ARM |
| Memory/undefined behavior | C controller and application tests passed AddressSanitizer and UndefinedBehaviorSanitizer; leak detection disabled because of execution-environment ptrace restrictions |
| Python tests | **15 passed**, none skipped: desktop parser/fragmentation/bounds, numeric rejection, retention/CSV/rollover, USB handshake/retry/timeout and ARM gating, same-C simulation and independent numerical oracle |
| Real desktop-window test | Tk application opened on Xorg dummy display; actual controls exercised metadata, mode2, ARM, 5→7 V reference, STOP, sample-period edit, CSV export, parameter JSON save and disconnect; screenshot `gui-demo.png` |
| Schematic review | MCU part, 16 MHz HSE, RCRC values, PWM/enable/ADC/USB/I²C/button pins and divider ratios cross-checked against supplied PDFs and raster schematics |
| IDE project metadata | `.project`/`.cproject` supplied for external Make import; XML syntax checked. STM32CubeIDE itself is not installed here |

Host numerical testing used Python 3.12, numpy 2.3.5, SciPy 1.17.0, matplotlib 3.10.8, pyserial 3.5 and pytest 8.3.5. Third-party firmware releases, commits and file hashes are in `vendor/README.md` and `vendor/SHA256SUMS`. The compiler download was checked against the pinned publisher SHA-256.

For the nominal modeled plant, 24 V bus, 5 ms sampling and 5 V reference, the last second of each five-second software simulation gave:

| Controller | Mean output | Peak-to-peak output ripple |
| --- | ---: | ---: |
| PI, default gains | 5.000000 V | <0.000001 V |
| FCS-MPC | 4.996142 V | 0.309273 V |
| CS-MPC QP | 5.000000 V | 0.000017 V |

These figures include floating-point numerical effects and the **ideal modeled plant**. They are not ADC noise/ripple specifications. FCS intentionally applies binary switch states once per sample and therefore shows appreciable ripple and a small mean error with these default weights. PI/CSMPC apply PWM duty; their desktop plant uses its average over the sampling interval, so these simulations do not model 20 kHz carrier ripple. Default gains and MPC settings are starting points, not hardware-optimized tuning. Step response plots are in `simulation.png`.

## Remaining board checks

Program with BOOT0 and STM32CubeProgrammer as described in the root README. Keep the converter supply disconnected while programming. First verify runtime native-USB CDC enumeration and stable telemetry with outputs disabled. Confirm connector orientation, common analog ground, and the actual ToC3/ToC4/bus ADC mappings and scaling using a meter. Confirm the OLED and button polarities.

At a low, current-limited bus voltage, verify enableB is low at startup and STOP, and that the three unused converter input pins remain low. Confirm ARM enables only the intended third-output control. Measure PWM frequency, sampling period, ADC acquisition timing and the longest QP execution time on the MCU. The deadline interlock must remain clear at the chosen period/horizon; host benchmark speed does not establish this. Confirm STOP, cable removal, port close and missed heartbeat disable the stage.

Measure L298 high/low output voltages and configure the bridge-drop model, adjust the ADC gains/offsets, enter actual fitted R/C and any extra loads, then tune with small references before increasing voltage. Verify response and limits with real instrumentation for each controller separately. No physical flashing, ROM DFU entry, USB enumeration, sensor calibration, power-stage testing or MCU cycle measurement was possible here.

The QP constrains duty and duty increments; it does **not** impose predictive capacitor-voltage constraints or guarantee closed-loop stability for arbitrary user parameters. Sampled overvoltage interlocks react after a measurement. Extremely ill-conditioned or infeasible configurations can make the bounded solver fail; firmware then disables the bridge and reports a fault. FCS uses a four-switch-step horizon independently of the CS-MPC horizon setting. The control model has no integral disturbance estimator for MPC. Parameters are volatile and must be re-entered after reset.

## Reproduce

From the project root, with native GCC and Arm GNU GCC configured:

```sh
make host
python -m pytest -q -s
make -j4
python tools/check_hex.py build/rcrc_h723.hex
sha256sum -c vendor/SHA256SUMS
python tools/plot_simulation.py
python -m desktop.app --demo
```

For sanitizers:

```sh
gcc -std=c11 -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer -Ifirmware/include firmware/src/control.c tests/test_control.c -lm -o build/test_control_san
ASAN_OPTIONS=detect_leaks=0 build/test_control_san
gcc -std=c11 -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer -Ifirmware/include firmware/src/control.c firmware/src/app.c tests/test_app.c -lm -o build/test_app_san
ASAN_OPTIONS=detect_leaks=0 build/test_app_san
```
