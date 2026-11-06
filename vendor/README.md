# Pinned upstream dependencies

Only the used source subsets are vendored. Original file headers and package licenses are retained.

| Package | Upstream | Tag | Peeled commit |
|---|---|---|---|
| STM32H7 HAL | https://github.com/STMicroelectronics/stm32h7xx-hal-driver | v1.11.5 | 57ed86b4c358e2333a71fd895b17f1ccdb68739e |
| STM32H7 CMSIS device | https://github.com/STMicroelectronics/cmsis-device-h7 | v1.10.6 | e8d40ae6e2fa06afe5b46d24756f141b363342a7 |
| STM32 USB Device | https://github.com/STMicroelectronics/stm32-mw-usb-device | v2.11.5 | 947131e000d1ecebc260f22010fa75486c50b423 |
| Arm CMSIS Core | https://github.com/ARM-software/CMSIS_5 | 5.9.0 | 2b7495b8535bdcb306dac29b9ded4cfb679d7e5c |

Sources were fetched using authenticated TLS from the official repositories at those release commits. `SHA256SUMS` records the vendored file contents for reproducible local integrity checks. HAL configuration and board glue are in `firmware/platform`, not patched into upstream sources.
