# Hardware mapping from the supplied November 6, 2026 exports

These findings come from the supplied schematic PDFs and their PNG exports. The PCB ZIP files contain raster layer images, not source CAD, Gerbers with net information, or a netlist. Routing continuity, assembly, jumper placement, voltage ratings, and actual component values cannot be verified from them. Verify the following mapping on the assembled hardware before applying bridge power.

## MCU board

U1 is **STM32H723ZGT6**, LQFP144. The MCU1 schematic shows a 16 MHz HSE crystal (X1), and a 32.768 kHz LSE crystal (X2). ADC VREF+/VDDA use the filtered analog supply net labelled `VSSA` in this drawing; MCU VSSA pin 31 is instead on AGND. Despite the confusing net name, the filtered net is drawn connected to 3.3 V through L1. Measure the ADC reference for calibration.

| Function | Schematic net | MCU pin | Evidence |
|---|---|---|---|
| RCRC drive, L298 OUT3 | PWM1_2 / Duty3 | PE11, TIM1_CH2, AF1 | MCU1 pin64; converter Connector alias; Power1 Input3 |
| Resistor drive, L298 OUT4 | PWM1_1 / Duty4 | PE9, TIM1_CH1, AF1 | MCU1 pin60; converter Connector alias; Power1 Input4 |
| LC bridge1 drive | PWM1_4 / Duty1 | PE14, TIM1_CH4, AF1 | MCU1 pin67; converter Connector alias |
| LC bridge2 drive | PWM1_3 / Duty2 | PE13, TIM1_CH3, AF1 | MCU1 pin66; converter Connector alias |
| Enable LC pair | out4 / out4x / enableA | PA8 | MCU1 pin99; MCU CN5 pin11; converter U13 pin11 |
| Enable RCRC and resistor pair | out5 / out5x / enableB | PA13 | MCU1 pin104; MCU CN5 pin12; converter U13 pin12 |
| First capacitor voltage | ADC1_INP14 / C3_MCU | PA2, ADC1 channel14 | MCU1 pin36; converter Connector and Analog |
| Second capacitor/output voltage | ADC1_INP15 / C4_MCU | PA3, ADC1 channel15 | MCU1 pin37; converter Connector and Analog |
| Converter bus voltage | ADC1_INP17 / Vs_MCU | PA1, ADC1 channel17 | MCU1 pin35; converter Connector and Analog |
| Serial TX to laptop | UART1_TX | PA9, USART1_TX, AF7 | MCU1 pin100 and MCU2 CH340 RXD |
| Serial RX from laptop | UART1_RX | PA10, USART1_RX, AF7 | MCU1 pin101 and MCU2 CH340 TXD |
| Native USB D- | USB_DM2 | PA11 | MCU1 pin102 and MCU2 USB_OTG connector |
| Native USB D+ | USB_DP2 | PA12 | MCU1 pin103 and MCU2 USB_OTG connector |

**PA13 conflicts with SWDIO.** The supplied boards route enableB through the SWDIO MCU pin. When firmware claims PA13 as a GPIO, an active SWD debugger can lose connection. Keep the bridge power supply disconnected while flashing/debugging: SWDIO traffic can also toggle enableB through this hardware connection. The normal user-selected programming path is BOOT0/ROM USB DFU, which avoids the need for SWD. If an SWD debugger is used for recovery, STM32CubeProgrammer connect-under-reset may be needed. The board wiring should be confirmed before assigning this pin. Do not drive enableB from an assumed unused GPIO. A board revision or jumper to another GPIO would avoid this conflict, but is not part of the supplied design.

`enableB` enables OUT3 **and OUT4** together inside the L298N. Keep Duty4 low when using OUT3 so the resistor circuit is not inadvertently energized. Keep enableA low to leave both LC branches disabled. The converter has 10 kOhm pulldowns on both enables. The MCU and converter schematic net spellings differ (`out4/out5` versus `out4x/out5x`), but matching connector positions establish the intended cable connections. Confirm cable orientation and pin numbering on the physical boards.

## RCRC plant

Power2 shows the following ladder directly on the converter board:

```
OUT3 -- R1 470 ohm -- ToC3 -- R2 470 ohm -- ToC4 (controlled output)
                       |                      |
                    C18 100 uF             C19 100 uF
                       |                      |
                      GND                    GND
```

No external RCRC load is shown. OUT4 separately drives U4 and U5, both 33 ohm to ground (16.5 ohm combined). The two LC branches are independent of the selected RCRC control target. An additional external load on ToC4 must be included in the plant model if fitted.

With states x1=ToC3 and x2=ToC4, resistances R1,R2, capacitances C1,C2, input u=actual OUT3 voltage, and sensing resistor loading Rs=55.7 kOhm on each state, the continuous model is:

```
dx1/dt = (u-x1)/(R1*C1) - (x1-x2)/(R2*C1) - x1/(Rs*C1)
dx2/dt = (x1-x2)/(R2*C2) - x2/(Rs*C2) - x2/(Rload*C2)
```

Use the last term only when an additional load is fitted. L298N Darlington output drops mean actual OUT3 high/low levels differ from ideal Vs and 0 V. Identification/calibration can improve prediction and reduce model mismatch. Software and simulation cannot establish switching limits, temperature behavior, ADC noise, closed-loop timing, or stability of the assembled boards.

## ADC scaling

Converter Analog shows 51 kOhm series/top and 4.7 kOhm bottom dividers for Vs, ToC1, ToC2, ToC3, and ToC4. Each divider is followed by a GS8552 unity-gain buffer. The divider's lower resistor has 2.2 nF in parallel; the buffer output has 12 ohm and 10 pF. Nominal conversion is:

```
physical_volts = adc_pin_volts * (51000+4700)/4700
               = adc_pin_volts * 11.85106383
adc_pin_volts  = ADC_code * measured_VREF / full_scale_code
```

At 3.3 V reference, nominal full-scale is 39.1085 V. This is a sensing limit, not permission to apply that voltage to the complete board. Use the assembled bridge, capacitor, supply, connector, and thermal ratings and provide headroom for transients. Fit gain/offset calibration per channel from measurements. Buffers run from +5 V, so inspect input/output fault protection before energizing; ADC pins must remain within MCU limits.

## OLED and button daughterboard

| Function | MCU pin | Meaning |
|---|---|---|
| I2C1 clock | PB6, AF4 | J2 pin3 to OLED SCL |
| I2C1 data | PB7, AF4 | J2 pin4 to OLED SDA |
| OLED power gate | PE2 | MCU1 pin1; low turns AO3401A P-channel supply switch on |
| UP | PB8 | Active high after 74HC14 inverter |
| DOWN | PB9 | Active high after 74HC14 inverter |
| LEFT | PE0 | Active high after 74HC14 inverter |
| RIGHT | PE1 | Active high after 74HC14 inverter |

MCU J2 to OLED daughterboard J1 is a 10-pin connector: pin1 NC; pin2 switched 3.3 V; pin3 SCL; pin4 SDA; pin5 UP; pin6 DOWN; pin7 LEFT; pin8 RIGHT; pins9/10 GND. Each raw button input is pulled high by 10 kOhm, goes low when pressed, and passes an RC filter before the 74HC14 inverter. Firmware sees a high level for a pressed button. Daughterboard I2C pullups are 1.8 kOhm to 3.3 V.

The schematic labels the module only `OLED`. The user subsequently confirmed **SSD1306, 128x64, I2C address 0x3C**. Use that configuration. A scan can establish an address, not independently identify the display controller.

## Laptop connection and programming

The user requested the **native USB_OTG connector** for USB CDC telemetry and commands. PA11/PA12 connect to the STM32H723 USB OTG HS controller using its internal full-speed PHY (the device has no separate USB OTG FS controller). Use GPIO AF10, the USB1/OTG_HS peripheral clock, OTG_HS interrupt, and a 48 MHz USB kernel clock. No dedicated VBUS sense signal to the MCU is shown, so the device configuration must disable hardware VBUS sensing and use the supported internal session-valid configuration. The existing second USB/UART connector instead connects to a CH340C bridge on USART1 PA9/PA10, and is not the connector for the native USB CDC firmware. The user confirmed programming through **BOOT0 and the MCU internal ROM bootloader**, using STM32CubeProgrammer over USB DFU; SWD is not required for the normal workflow. The CH340 USB/UART port is not the native USB DFU port.

## Information still needed for energized validation

- The user specified **30 V maximum bus voltage**. Confirm the actual operating bus voltage, allowed output/reference range, supply/current limits, capacitor ratings, and any extra ToC4 load before energized validation.
- Confirm PA13/enableB cable continuity. The user-selected programming method is BOOT0/internal ROM USB DFU; no SWD cable is needed.
- Measure reference voltage, channel gain/offset, actual R/C values as needed for control accuracy, and actual OUT3 high/low levels.

The schematics establish a useful implementation target, but physical timing, PWM waveforms, ADC readings, polarity, and protection must be checked on real hardware before asserting hardware validation.

## Flashing with BOOT0 and STM32CubeProgrammer

1. Disconnect the converter bridge supply while programming. Power the MCU board appropriately and connect the laptop to the **native USB_OTG** connector, not USB/UART.
2. Hold the board's BOOT0 button high while asserting and releasing NRST. Once reset samples BOOT0, the button can be released. This enters the factory system-memory bootloader when the device option bytes select the normal BOOT0-controlled boot path.
3. In STM32CubeProgrammer, select **USB**, refresh, and connect to the detected DFU device. On Windows, the STM32 DFU driver may need to be installed from CubeProgrammer's drivers directory.
4. Select the supplied Intel HEX firmware and use **Download** with verification enabled. The HEX records contain their addresses beginning at **0x08000000**; do not substitute the ROM/system-memory address.
5. Release BOOT0 low and reset again. The application should enumerate as a USB CDC serial device on the same native connector. Select that serial port in the Python GUI.
6. Keep the converter power disconnected until pin mapping, ADC calibration, and output-disabled state have been checked.

If the ROM DFU device does not appear, check cable data capability, connector selection, MCU supply, BOOT0 level during reset, and option bytes. Consult ST **AN2606** for STM32H723 bootloader interfaces and boot-entry requirements. A user-modified boot-address/boot-selection option-byte configuration can prevent the assumed BOOT0 behavior; do not rewrite unrelated option bytes blindly. USB enumeration, successful flashing, and operation of the physical boards cannot be verified in this cloud workspace.

## Implemented platform configuration reviewed

The STM32 platform uses the 16 MHz HSE with PLL1 M=4,N=100,P=2: CPU200 MHz, AXI/AHB100 MHz, APB50 MHz and timer kernels100 MHz. PLL2 P supplies25 MHz to ADC1; PLL3 Q supplies an HSE-derived48 MHz to native USB. TIM1_CH2 on PE11 runs at20 kHz. TIM6 triggers the three-channel ADC1 DMA scan; TIM2 independently supplies a free-running1 MHz timestamp. ADC DMA storage is in AXI SRAM with data cache disabled. The linker assigns1 MiB flash at0x08000000,320 KiB AXI SRAM at0x24000000 and128 KiB DTCM for the stack. The supplied H723 CMSIS startup and OTG_HS interrupt are used.

The board keeps PA13 in its reset/debug function until the application explicitly enables the selected output, then claims it as the enableB GPIO. Outputs boot disabled. EnableA and the unused L298 input pins are driven low. Fault and stop paths lower enableB directly. This is a static source review, not a measurement of reset pin levels or propagation timing.

TIM1 compare preload applies new duties, including binary FCS choices, at the next20 kHz carrier update. This introduces up to50 microseconds between the register write and its output application; include the measured input delay when refining the prediction model. Worst-case execution time, ADC trigger/sample timing and noise, USB enumeration, fault shutdown waveforms, and closed-loop results must be measured on the actual boards.
