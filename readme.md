# OxyStatQuad v3.0

Firmware for the OxyStatQuad, a four-channel wireless potentiostat for continuous oxygenation generation. Runs on a Silicon Labs BGM121 (Blue Gecko) SoC and communicates with a central device over BLE.

## Features

- Four independent potentiostat channels with configurable voltage setpoints
- Configurable measurement rate, duration, and sampling frequency
- Oxygenation control with adjustable period and duty cycle
- BLE data transfer with buffered measurements and acknowledged packet delivery
- BLE-based remote configuration via GATT
- Low-power EM4S shelf mode with magnetic switch wake

## Hardware

- **MCU:** Silicon Labs BGM121A256V2 (Blue Gecko BLE SoC)
- **DACs:** 2x MAX5535 (dual-channel, SPI) for setting channel voltages
- **ADC:** Internal 12-bit ADC with DMA for continuous sampling
- **Peripherals:** LETIMER (measurement timing), LDMA (ADC transfers), CRYOTIMER (sleep), GPIO MUX (channel selection)

## Repository Structure

```
oxystat.c/h            Core potentiostat control (ADC, DMA, DAC, MUX, timing)
oxyconfig.c/h          Configuration management (21-byte config protocol)
oxystat_service.c/h    BLE GATT service for data and configuration
app.c/h                Application state machine and BLE event handling
max5535.c/h            MAX5535 DAC SPI driver
blink.c/h              LED status indicators
flexiBLE_common.c/h    BLE utilities
channels.h             Channel and measurement type definitions
autogen/               Simplicity Studio generated components (required for build)
gecko_sdk_4.4.4/       Silicon Labs Gecko SDK (bundled for reproducibility)
bootloader/            Pre-built bootloader binary
config/                BLE GATT and hardware configuration
documentation/         PlantUML state diagram source
```

## Building

1. Install [Simplicity Studio v5](https://www.silabs.com/developers/simplicity-studio)
2. Import the project using `oxystat_quad.slcp`
3. The bundled Gecko SDK (v4.4.4) ensures build reproducibility

## Flashing

1. Flash the included bootloader (`bootloader\bootloader-storage-internal-ble-combined.s37`)
2. Flash the built firmware .hex, .s37, or .out file (.bin may overwrite bootloader)

