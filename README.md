# MakersPlus 2024 wristband implementation

This repo contains the code that performs motion analysis on a wrist-mounted device for the interactive experience of people synchronization.

## Hardware overview

- The MCU board is the ESP32S3 SuperMini, which includes a dual-core Xtensa CPU, a WS2818 LED, a battery regulator/controller (apparently charger too) and a PCB antena for WiFi. See its [pinout diagrams](ESP32S3_SuperMini_schematic.webp)).
- The IMU is the [ICM-20948 breakout board from Adafruit](https://learn.adafruit.com/adafruit-tdk-invensense-icm-20948-9-dof-imu). It is connected to the MCU over the I2C bus (though check the [use-spi-round2](https://github.com/lulingar/esp32s3-nostd-imu-test/compare/use-spi-round2) branch for the latest attempt I made to use the SPI bus, with some [help from the IMU driver author](https://github.com/lulingar/esp32s3-nostd-imu-test/commit/4adad41bde4731fdea87b7b77294a8609240c3b7#commitcomment-145229705))

## Firmware overview

This is a no-std Rust crate, which uses [embassy](https://github.com/embassy-rs/embassy) for providing async, high-level constructs while there being no underlying OS, and [esp-hal](https://github.com/esp-rs/esp-hal) and associated libraries for providing the infrastructure.

Highlights of this implementation:

- Leveraging the two cores of the CPU, the IMU sampling and motion analysis are executed on the second core, leaving WiFi, network stack and MQTT management on the first core. The two are connected via a message channel provided by embassy-sync.
- The network loop should be resilient enough to gracefully handle network disconnects and broker disconnects, retrying the connection as long as it is not successful.
- The connection parameters are to be provided by a `cfg.toml` file. See the [cfg.toml.example](cfg.toml.example) for reference.

Given that upstream LLVM does not include Xtensa CPU support, Espressif maintains a fork of it, which is necessary to have for building this project. They have the [espup](https://github.com/esp-rs/espup) CLI tool, which is a sort of "`cargo` for doing Xtensa in Rust".