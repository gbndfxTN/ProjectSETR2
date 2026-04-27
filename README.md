# ProjectSETR2

ProjectSETR2 is the **remote sensor node** for the SETR system. It runs on an **ESP32** with **ESP-IDF** and collects sensor measurements before sending them to the main board over **UART / RS232**.

## Project role

This board:

- initializes the UART link to the main board;
- reads a **CO2** sensor (serial + PWM signal);
- reads a **presence** sensor based on an ultrasonic module;
- aggregates measurements in a FreeRTOS queue;
- periodically sends text frames over UART.

Sent frame format:

```text
CO2_UART:<value>;CO2_PWM:<value>;PRES:<0|1>
```

## Project structure

- `src/main.c`: entry point and task creation.
- `src/co2_sensor.c`: CO2 sensor acquisition.
- `src/presence_sensor.c`: presence detection acquisition.
- `src/consumer_task.c`: queue consumer and UART output.
- `src/uart_link.c`: UART link configuration.
- `include/config.h`: hardware and timing configuration.

## Hardware configuration

Default values are defined in `include/config.h`:

- Link UART: `UART2`
- TX: `GPIO 27`
- RX: `GPIO 26`
- Speed: `9600 baud`

CO2 sensor:

- dedicated UART: `UART1`
- TX: `GPIO 16`
- RX: `GPIO 17`
- sync line: `GPIO 15`
- PWM: `GPIO 23`

Presence / ultrasonic sensor:

- TRIG: `GPIO 33`
- ECHO: `GPIO 32`

## Build and flash

### With ESP-IDF

```bash
idf.py build
idf.py flash
idf.py monitor
```

### With PlatformIO

```bash
pio run
pio run -t upload
pio device monitor
```

## Runtime notes

- The CO2 sensor may need a warm-up period before readings are reliable.
- Automatic `ABC` calibration and zero calibration are controlled in `include/config.h`.
- The presence threshold is configured through `PRESENCE_THRESHOLD_CM`.

## Dependency on the main board

ProjectSETR2 is designed to work with the main board of the system, which receives the UART frames, displays them, and can publish them to Firebase.
