# Power-Aware Logger with RTOS

Mini-project for **ESP32-S3** using **ESP-IDF v5.5.2**.
The system wakes up every second, reads sensor data, updates the OLED display, and periodically stores compact binary log records into the internal flash memory.

## Project goal

This project demonstrates:

- multitasking in FreeRTOS
- synchronization with Mutex
- queue-based communication between tasks
- power saving with **Light Sleep**
- buffered logging into internal flash
- periodic sensor sampling and display updates

## Hardware

- **ESP32-S3 N16R8**
- **BME280** — temperature, humidity, pressure sensor
- **DS1306** — real-time clock for timestamp
- **SSD1306** — OLED display
- UART terminal for commands

## Main idea

The microcontroller wakes up every **1 second**.

On each wake cycle it:

1. reads current time from **DS1306**
2. reads temperature, humidity, and pressure from **BME280**
3. updates the **SSD1306** display
4. decides whether it is time to save a log record
5. enters **Light Sleep** again for 1 second

Logging to flash is done **less often** than display updates.
For example, the display is refreshed every second, but logging can happen every **1 minute** or every **5 minutes**, depending on the compile-time setting.

To reduce flash wear, records are first collected in a **RAM buffer**.
When the buffer becomes full, it is written to flash as one sector-sized block.

## RTOS architecture

The project uses **5 FreeRTOS tasks**:

### 1. Sensor Task
Responsible for:
- reading BME280
- reading DS1306
- building one log record
- sending it to `sensor_queue`

### 2. UART Task
Responsible for:
- receiving text commands from UART
- parsing user commands
- sending commands to `command_queue`

### 3. Display Task
Responsible for:
- receiving display snapshots from `display_queue`
- updating SSD1306

### 4. Manager Task
Responsible for:
- coordinating the whole application
- receiving sensor records
- receiving UART commands
- deciding when to append a record to the RAM buffer
- forcing flash flush when the buffer is full
- entering Light Sleep for 1 second

### 5. Flash Writer Task
Responsible for:
- writing one full RAM buffer to flash
- erasing one sector before write
- advancing write offset in the log partition
- stopping logging when the partition becomes full

## Synchronization

### Mutex
The project uses an **I2C Mutex** because multiple devices share the bus:

- BME280
- SSD1306

Only one task may use the I2C bus at a time.

### Queues
The project uses the following queues:

- `sensor_queue` — Sensor Task → Manager Task
- `command_queue` — UART Task → Manager Task
- `display_queue` — Manager Task → Display Task

## Logging format

To save memory, logs are stored in **binary format**, not as text.

Each log record contains:

- year
- month
- day
- hour
- minute
- second
- temperature in **°C × 100**
- humidity in **% × 100**
- pressure in **mmHg**

Example:

- `23.45 °C` → `2345`
- `56.78 %` → `5678`

This is more compact than storing strings and is easier to write to flash efficiently.

## RAM buffer strategy

The project uses:

- **one RAM buffer**
- buffer size = **one flash sector**
- records are appended until the buffer is full
- when full, the whole buffer is written to flash

This reduces the number of flash write/erase operations.

## Flash logging policy

Logs are written into a dedicated internal flash partition called:

`logdata`

Important design decisions:

- **no wear leveling**
- **no metadata stored in flash**
- records are written sector by sector
- when the partition becomes full:
  - logging stops
  - warning is printed through UART
  - warning is shown on the OLED display

## Sleep behavior

The system uses **Light Sleep** with timer wakeup.

Behavior:

- wake up every **1 second**
- perform one measurement/display cycle
- go back to Light Sleep

This demonstrates basic energy-saving behavior while still keeping the system responsive.

## UART commands

Supported commands:

- `help` — show command list
- `status` — show current system status
- `flush` — force write current RAM buffer to flash
- `erase` — erase log partition and clear log buffer
- `log on` — enable logging
- `log off` — disable logging
