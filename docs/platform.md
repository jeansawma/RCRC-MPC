# STM32H723 board port

This port targets the supplied STM32H723ZGT6 schematic and the indicated connector aliases. Check the physical connector orientation and continuity before applying the converter supply.

| Function | MCU pin / peripheral |
|---|---|
| Third bridge input (`Duty3`) | PE11 / TIM1_CH2 |
| Third/fourth bridge shared enableB | PA13, normally SWDIO |
| Unused fourth/second/first bridge inputs | PE9 / PE13 / PE14, held low |
| enableA | PA8, held low |
| RCRC first state ToC3 | PA2 / ADC1_INP14 |
| RCRC output ToC4 | PA3 / ADC1_INP15 |
| Bus voltage | PA1 / ADC1_INP17 |
| Native USB D− / D+ | PA11 / PA12, USB1 OTG_HS internal full-speed PHY |
| OLED SCL / SDA | PB6 / PB7, I2C1 |
| OLED power control | PE2 low turns the P-channel supply switch on |
| UP / DOWN / LEFT / RIGHT | PB8 / PB9 / PE0 / PE1, active high |

PA13 stays in its reset SWD configuration until an accepted ARM enables the bridge. Converter enableB's external 10 kΩ pulldown is required. The first ARM repurposes PA13 as an output; stop drives it low. Debugging thereafter may require ST-LINK **Connect under reset**. The fourth bridge shares enableB, but its input is held low. Do not assume the first two bridges or fourth output are controlled by this firmware.

## Clock and memory choices

- HSE 16 MHz; PLL1 16/4 × 100 / 2 = 200 MHz CPU.
- AHB 100 MHz, APB1/2/3/4 50 MHz; APB timer kernels 100 MHz.
- ADC PLL2P = 16/4 × 50 / 8 = 25 MHz; 16-bit single-ended ADC, 64.5-cycle acquisition, offset and linearity calibration.
- USB PLL3Q = 16/4 × 96 / 8 = 48 MHz. The port uses the HS controller in **full-speed** mode, with the embedded PHY; no external ULPI PHY is assumed. USB VBUS sensing is disabled because there is no dedicated VBUS sense net on the supplied MCU pinout. The schematic supplies VDD33USB externally; the voltage detector is enabled.
- PLL inputs are 4 MHz with VCI range 2. Supply is LDO, VOS1, flash latency 2; this is intentionally below the MCU's maximum speed.
- 1 MiB Flash at 0x08000000; `.data`, `.bss`, ADC DMA buffers and the 4 KiB newlib heap reside in 320 KiB AXI SRAM at 0x24000000. Main/interrupt stack is in 128 KiB DTCM at 0x20000000; the linker reserves 32 KiB minimum stack space. D-cache remains disabled for DMA coherence; I-cache is enabled.
- A dedicated TIM2 counter provides 1 MHz modulo-2³² microseconds. Unsigned differences handle its approximately 71-minute rollover.

## Sampling and timing

TIM6 emits a hardware update trigger at the configured sampling interval (default 5000 µs). ADC1 scans channels 14, 15 and 17 in that order; DMA1_Stream0 transfers the three halfwords into AXI SRAM. TIM6's highest-priority interrupt detects any unfinished conversion, undelivered sample or unfinished control update from the previous period and immediately disables enableB. DMA IRQ has priority 1; USB priority 6 and I²C priority 5 cannot delay the sample boundary interrupt.

`board_sample_ready()` returns physical voltages computed using 3.3 V reference and the schematic's `(51k + 4.7k)/4.7k` voltage ratio. These defaults require measurement/calibration on the assembled board. All three conversions belong to the same sequence but have a small scan skew. Every consumed sample must be followed by `board_set_output()`, including while stopped. An output update reaching the last 50 µs of its period is rejected and latched as a deadline fault. A platform fault always overrides a request to enable the output.

TIM1 is fixed at 20 kHz. PI/PID and continuous-set MPC commands use PWM duty. Finite-set commands are required to be exactly 0 or 1; these are constant low/high PWM compare states, rather than averaged duty commands. TIM1's compare preload applies every change on its next 20 kHz update, introducing up to 50 µs actuator delay. Sampling and PWM are not phase locked. Measure ADC, computation, actuator timing and total deadline margin with the actual firmware on the board; simulation does not establish MCU execution time.

OLED writes use interrupt-driven transfers and skip old display frames. Runtime USB sends copy bounded messages into an 8-slot queue and drop them when full; command acknowledgements can therefore require a retry. No application formatting or control optimization runs in USB interrupts. The optional OLED can be absent without preventing USB/control operation.

## Platform faults and recovery

| Bit value | Meaning |
|---|---|
| 1 | Required peripheral/clock initialization failed; reboot required |
| 2 | Missed sample/control deadline |
| 4 | ADC fault or channel near full scale |
| 8 | Invalid/nonfinite duty or nonbinary finite-set command |
| 16 | Previous boot was an independent-watchdog reset |

CLEAR can clear recoverable platform faults only while the bridge is disabled and ADC data has been received within three sampling periods. A continuing ADC problem will relatch. The independent watchdog is fed from the application polling path and resets after approximately 0.5 seconds without it (LSI tolerance applies); PA13 then returns to input and the external pulldown disables the converter. Application-level voltage, heartbeat and QP faults are separate from these bits.

Native USB enumeration, ADC calibration, display operation, transistor polarity and actual output voltage remain bench checks. None can be verified by a host compilation alone.

USB connectivity additionally requires a configured device, asserted CDC DTR and a USB SOF within 10 ms. DTR closure, reset, suspend, deinitialization, unplug or loss of SOF invalidates the current session epoch and clears received commands. Application authorization is bound to the current epoch; each fresh USB connection requires HELLO and an explicit ARM. The platform captures the USB epoch with each consumed ADC sample and refuses to energize if connectivity or the epoch changed before actuator commit. This is a timing/reconnect interlock, not an authentication protocol.
