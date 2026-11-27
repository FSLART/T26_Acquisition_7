# T26_Acquisition_7

Firmware for **Acquisition board 7 (AQT7)** of the T26 car, built on **Zephyr RTOS** (v4.5.0). It runs on an
**STM32F412RET6** and:

- measures **rear brake pressure**, **left/right suspension travel** and **coolant temperature** (NTC) with the ADC,
- receives **front brake pressure** from another node over CAN,
- drives the **brake light** from the front and rear pressures,
- publishes the measurements on two CAN buses (Autonomous and DATA),
- supervises every thread with the independent watchdog,
- has a shell and log reachable through the ST-LINK alone (RTT over SWD) or on UART, and a debug snapshot
  (`acq7`) for live views in the debugger.

The firmware used to be an STM32CubeIDE/HAL project. That code was removed when the Zephyr port replaced it;
it is still in the git history. See
[Changes from the STM32CubeIDE firmware](#changes-from-the-stm32cubeide-firmware).

## TL;DR

To set up a development PC, see [Development setup](#development-setup).

**Always activate the virtual environment before using west.** Run these from the repository folder:

| Command | What it does |
|---|---|
| `..\.venv\Scripts\Activate.ps1` | Activates the Python virtual environment (it lives in the workspace folder, one level up) |
| `west build -p` | Builds the firmware into `build\` |
| `west flash -d build` | Flashes the build onto the board through the ST-LINK |
| `west rtt -d build --runner openocd` | Opens the board's log and shell through the ST-LINK (OpenOCD comes with the Zephyr SDK) |


---

## Hardware

### Clocks

| Clock | Frequency |
|---|---|
| SYSCLK / HCLK | 96 MHz (HSI 16 MHz → PLL /8 ×96 /2) |
| APB1 (CAN) | 48 MHz |
| APB2 (ADC, USART1) | 96 MHz |
| ADC clock | 24 MHz (APB2 / 4) |
| LSI (IWDG) | ~32 kHz |

No external crystal is used. All of it is set in `boards/t26/t26_aqt7/t26_aqt7.dts`.

### Pinout

| Pin | Function | Notes |
|---|---|---|
| PA7 | `BRK_PRESS` – ADC1_IN7 | Rear brake pressure sensor |
| PB0 | `SUSP_LEFT` – ADC1_IN8 | Left suspension potentiometer |
| PB1 | `SUSP_RIGHT` – ADC1_IN9 | Right suspension potentiometer |
| PC4 | `NTC1` – ADC1_IN14 | Coolant NTC |
| PC10 | `BRAKE_LIGHT` – output | **Active high**: drives the light's MOSFET gate (pull-down to GND), HIGH while braking |
| PC11 | `HEARTBEAT` – output | Toggles every 100 ms |
| PA11 / PA12 | CAN1 RX / TX | Autonomous bus, RX pull-up |
| PB12 / PB13 | CAN2 RX / TX | DATA bus, RX pull-up |
| PA9 / PA10 | USART1 TX / RX | Log and shell, 115200 8N1 |
| PA13 / PA14 | SWDIO / SWCLK | ST-LINK: flash, debug, RTT log |

### Peripherals

| Peripheral | Configuration (devicetree) |
|---|---|
| ADC1 | 12 bit, 480-cycle sampling, channels 7/8/9/14 listed in `zephyr,user` → `io-channels`, read as one sequence every 10 ms |
| IWDG | 2000 ms timeout (set by `watchdog.c`), paused while the debugger halts the CPU |
| CAN1 | 1 Mbit/s, sample point 75 %, alias `can-autonomous` |
| CAN2 | 1 Mbit/s, sample point 87.5 %, alias `can-data` (shares CAN1's filter banks, so CAN1 must stay enabled) |
| USART1 | 115200 8N1, Zephyr console (log + shell) |

---

## Sensors

The conversions are in `src/conversions.c`. They are plain C with no Zephyr calls.

### Brake pressure (rear) – `MeasureBreakPressure()`

- Sensor: **Bosch 0 261 545 053**, gauge pressure, 0.5–4.5 V for 0–140 bar, 5 V supply.
- PA7 sees the sensor output ~1:1 (`CONVERSION_FACTOR = 1.0`).
- Conversion:
  `V = adc × 3.3 / 4095`
  `p = (V − OFFSET_VOLTAGE) / 0.02857 V/bar`, clamped to 0…140 bar.
- `OFFSET_VOLTAGE = 0.614 V` is the PA7 voltage measured with the pedal released (nominal would be 0.5 V).
  It has drifted before; to re-zero, read `acq7.adc.volts[0]` at rest (see [Debugging](#debugging)) and copy
  the value here.

> **Limitation:** the ADC saturates at 3.3 V ≈ **94 bar** (`(3.3 − 0.614) / 0.02857`), and above that PA7 exceeds its 3.6 V absolute
> maximum. A divider must be added and its ratio set in `CONVERSION_FACTOR` before relying on higher pressures.

### Brake pressure (front) – received on CAN1

- Frame **`0x710`** (`autonomous_t26` AQT1, `FRT_BRK_PRESS`, 0.1 bar/bit), decoded in `src/can_rx.c`.
- Trusted only while frames keep arriving: `get_front_brake_pressure()` returns 0 if no `0x710` was seen for
  `FRONT_PRESSURE_TIMEOUT_MS` (500 ms).
- The DBC defines AQT1 as **3 bytes**; shorter frames are rejected by the DBC unpack function.

### Brake light – `src/brake_light.c`

Every 50 ms, PC10 is driven **HIGH** (light on) if the rear pressure ≥ `PRESSURE_LEVEL_REAR` **or** the front
pressure ≥ `PRESSURE_LEVEL_FRONT` (both 7 bar), and LOW otherwise. If the front pressure frame is lost, the
light follows the rear pressure only. The light is off at boot.

### Suspension – `MeasureSuspensionPosition()`

- Sensor: linear potentiometer, **75 mm** electrical stroke, 5 V supply, scaled to 3.3 V by a divider on the board.
- Absolute position: `pos_mm = 75 × V_sensor / 5`, with `V_sensor = V_pin / (3.3 / 5)`.
- **Ride-height zeroing:** the first full moving average after boot (~100 ms) is taken as ride height and logged
  (`Suspension zeroed: L ... mm, R ... mm`). Until then 0 mm is sent. Power up with the car at rest on its wheels.
- Reported value: `zero − pos_mm`
  - **positive = extension** (wheel drops)
  - **negative = compression** (wheel rises)
  - (on this car the sensor voltage drops when the suspension extends, hence the subtraction order)

### Coolant temperature (NTC) – `MeasureNTCTemperature()`

- Sensor: **Vishay NTCAIMM66H**, 10 kΩ @ 25 °C, B25/85 = 3984 K.
- Board circuit: `5 V → NTC → node A`; node A has 100 kΩ to GND and a 24.3 kΩ / 4.9 kΩ divider down to the ADC pin.
- Conversion:
  1. `V_A = V_pin × (24.3k + 4.9k) / 4.9k`
  2. `R_low = 100k ∥ (24.3k + 4.9k)`
  3. `R_ntc = R_low × (5 V − V_A) / V_A`
  4. Beta equation: `1/T = 1/298.15 + ln(R_ntc / 10k) / 3984`, result in °C
- Fault detection: ADC = 0 (open sensor) or `V_A ≥ 5 V` (shorted sensor) returns `NTC_FAULT_C` (−99 °C).
- Only one NTC is fitted. Adding more requires a new ADC channel in the devicetree (`zephyr,user`), a new
  entry in `enum sensor_channel` (`sensors.h`), and new signals in the DBC and `can_tx.c`.

---

## CAN interface

Both buses run at 1 Mbit/s with standard (11-bit) IDs, retransmit until acknowledged and recover from bus-off
automatically.

| Bus | Dir | ID | DBC message | Period | Content |
|---|---|---|---|---|---|
| CAN1 Autonomous | TX | 0x770 | `autonomous_t26` AQT7 | 50 ms | `REAR_BRK_PRESS`, 0.1 bar/bit |
| CAN1 Autonomous | RX | 0x710 | `autonomous_t26` AQT1 | — | Front brake pressure (`FRT_BRK_PRESS`), for the brake light |
| CAN2 DATA | TX | 0x770 | `data_t26` AQT7 | 125 ms | Suspension and coolant temperature, see below |

The two `0x770` frames are different messages from different DBCs: use `autonomous_t26_*` for CAN1 and
`data_t26_*` for CAN2.

CAN2 `AQT7` (`data_t26_aqt7_pack()`):

| Signal | Bytes | Type | Scale | Unit | Notes |
|---|---|---|---|---|---|
| `SUSP_L` | 0–1 | int16 LE | 0.1 | mm | + extension / − compression |
| `SUSP_R` | 2–3 | int16 LE | 0.1 | mm | + extension / − compression |
| `NTC_1` | 4 | uint8 | 1 | °C | Rounded, clamped to 0…254; **255 = sensor open/shorted** |
| — | 5–7 | — | — | — | Unused (0) |

**Sending.** `can_send_frame()` never blocks: if every TX mailbox is busy, or the bus is off, the frame is dropped
and counted in `can_stats[bus].tx_dropped`; the next period sends a fresh value.

**Bus health (`can_bus_service()`).** `main` calls it every 100 ms. For each bus it:

- reads the state and the TEC/REC error counters,
- totals each protocol error type since boot (stuff, form, ACK, bit recessive, bit dominant, CRC, RX overrun;
  counted by the driver with `CONFIG_CAN_STATS`) and keeps the most recent type as `last_error`,
- counts bus-off entries and controller restarts,
- logs a change of state or of the last error type, at most once per second per bus, with an explanation of the
  usual causes, e.g.
  `DATA (CAN2) BUS-OFF - TEC > 255: ...` followed by `DATA (CAN2) last error: BIT DOMINANT - sent 0 read 1: our TX not reaching the bus ...`,
- restarts a controller found stopped, at most every 100 ms (bus-off is left to the driver).

`can_bus_get_status(bus)` returns all of it; the debug snapshot shows it as `acq7.can.autonomous` / `acq7.can.data`.

**Receiving a new frame:** add `{CAN_BUS_DATA, DATA_T26_xxx_FRAME_ID}` (or `CAN_BUS_AUTONOMOUS`) to
`can_rx_ids[]` in `can_rx.c`, then a `case` in `can_handle_data_rx()` / `can_handle_autonomous_rx()` that
unpacks it with the DBC function and stores the value behind a getter. Each entry uses one hardware filter
(14 available).

**Sending a new frame:** add a `can_send_xxx()` encoder in `can_tx.c` that packs with the DBC function and calls
`can_send_frame()`, plus its own `next_xxx` deadline in `can_tx_thread()`.

**When the DBC changes:** the workflow regenerates `Core/DBC/`. If a message used here changes (AQT1, AQT7),
update its encoder in `can_tx.c` or decoder in `can_rx.c`.

> **Don't edit, rename or move anything in `Core/DBC/`.** It is generated by **cantools** from the T26 DBC files
> (T26_DBC repository) and the workflow overwrites it. The firmware uses `data_t26.*` and `autonomous_t26.*`;
> `powertrain_t26.*` is unused. Change the DBC in T26_DBC instead.

---

## Firmware

One thread per job. Each module owns its data and its lock; other modules only read it through a getter, so no
thread ever reads a half-updated value.

| Thread | Priority | Period | Job |
|---|---|---|---|
| `main` | 0 (Zephyr default) | 100 ms | Heartbeat (PC11) + `watchdog_service()` + `can_bus_service()` |
| `sensors` | 2 | 10 ms | Read the 4 ADC channels, average, convert, publish |
| `brake_light` | 3 | 50 ms | Light on while rear **or** front pressure ≥ 7 bar |
| `can_rx` | 4 | event, ≤ 100 ms | Decode received frames (waits at most 100 ms so it can check in with the watchdog) |
| `can_tx` | 5 | 50 ms / 125 ms | CAN1 0x770 every 50 ms, CAN2 AQT7 every 125 ms |
| `debug` | 6 | 10 ms | Refresh `acq7`; log the sensor values every `CONFIG_AQT7_DEBUG_PRINT_MS` |

A lower number runs first. All threads use absolute deadlines (`K_TIMEOUT_ABS_MS`), so their periods don't
drift with the time spent in the loop. Threads are defined with `SYS_FOREVER_MS` as start delay: they exist from
boot but only run once `main` calls their module's `*_start()`.

### Start-up and failure handling

1. `watchdog_init()` starts the IWDG first, so a hang anywhere in the setup also resets the board.
2. The heartbeat GPIO, `sensors_init()`, `brake_light_init()` and `can_bus_init()` set up the hardware.
3. If any of them fails, `watchdog_fail()` logs the reason and stops feeding: the IWDG resets the board after
   2 s and the whole setup is tried again. A permanently broken part (e.g. an unpowered CAN transceiver at
   start-up) therefore makes the board reboot every ~2 s.
4. The threads are started; each registers with the watchdog.

### Watchdog

- IWDG on the LSI, **2000 ms** timeout.
- Every thread calls `watchdog_checkin()` once per loop. `main` feeds the IWDG every 100 ms, but **only if
  every registered thread checked in** since the last feed. A single hung thread therefore resets the board.
- After 1 s without a feed, the log names the thread that stopped (`Thread can_tx stopped checking in`),
  before the reset at 2 s.
- The IWDG is paused while a debugger halts the CPU, so breakpoints don't reset the board.

To supervise a new thread: add it to `enum watchdog_client` and `client_name[]` in `watchdog.c`, call
`watchdog_register()` in its `*_start()` and `watchdog_checkin()` in its loop. Its loop must never block for
more than ~1 s.

### Sensor data

`sensors.c` samples all four channels every **10 ms** and keeps a **10-sample moving average** per channel
(100 ms window). Each sample it publishes a `struct vehicle_sensor_data`:

| Field | Unit | Notes |
|---|---|---|
| `rear_brake_bar` | bar | `MeasureBreakPressure()` |
| `susp_left_mm`, `susp_right_mm` | mm | Relative to ride height: + extension, − compression. 0 until zeroed |
| `ntc1_c` | °C | `NTC_FAULT_C` (−99) for an open or shorted sensor |
| `raw[4]` | ADC counts | Latest sample per channel, for debugging |
| `filtered[4]` | ADC counts | Moving average per channel, for debugging |

The ADC channels and their order come from the devicetree (`zephyr,user` → `io-channels`). A `BUILD_ASSERT` in
`sensors.c` stops the build if that list and `enum sensor_channel` ever have different lengths.

### Configuration (`prj.conf`)

| Option | Why |
|---|---|
| `CONFIG_GPIO`, `CONFIG_ADC`, `CONFIG_CAN`, `CONFIG_WATCHDOG` | Drivers for the devicetree nodes |
| `CONFIG_STATS`, `CONFIG_CAN_STATS` | Per-type CAN error counters used by `can_bus_service()` |
| `CONFIG_REBOOT` | `sys_reboot()`, used by `watchdog_fail()` if the IWDG isn't running |
| `CONFIG_LOG`, `CONFIG_LOG_DEFAULT_LEVEL=3` | Log at level INF, shown on both shells |
| `CONFIG_USE_SEGGER_RTT`, `CONFIG_SEGGER_RTT_BUFFER_SIZE_UP=4096`, `CONFIG_SEGGER_RTT_BUFFER_SIZE_DOWN=128` | RTT buffers: 4 KB out (log), 128 bytes in (a whole typed or pasted command) |
| `CONFIG_LOG_BACKEND_RTT=n` | The log reaches RTT through the RTT shell; a second RTT log output on the same channel would interleave with it |
| `CONFIG_CBPRINTF_FP_SUPPORT` | `%f` in log messages |
| `CONFIG_FPU` | Hardware FPU for the sensor maths |
| `CONFIG_AQT7_DEBUG_PRINT_MS=1000` | Sensor values log period, 0 = off (defined in `Kconfig`, default 100) |
| `CONFIG_SHELL`, `CONFIG_SHELL_BACKEND_RTT`, `CONFIG_SHELL_BACKEND_SERIAL`, `CONFIG_*_SHELL` | Shell over RTT and on USART1, with `gpio`, `can`, `kernel`, `device` and `sim` commands |
| `CONFIG_SHELL_STACK_SIZE=3072` | The log's float formatting runs in the shell thread and needs the extra stack |

### Firmware version

Every build knows its **version** (semver) and the **git commit** it was built from. A future CAN updater
will use this to tell which firmware a board runs. Both come from Zephyr's version system:

- **`VERSION`** (repository root) holds `VERSION_MAJOR`, `VERSION_MINOR`, `PATCHLEVEL` and `VERSION_TWEAK`.
  Zephyr turns it into `zephyr/app_version.h` (`APP_VERSION_STRING`, `APP_VERSION_MAJOR`, ...).
- **`APP_BUILD_VERSION`** in the same header is `git describe --abbrev=12 --always` of this repository, i.e.
  the commit hash (`83c5bc60089d`). `CMakeLists.txt` makes the header regenerate whenever HEAD, the branch or
  the git index changes, so an incremental build also reports the right commit.
- There is **no dirty flag**: a build with uncommitted changes reports the HEAD commit. Firmware that goes on
  the car should come from a CI release, which is always built from a clean commit.

Where it shows:

| Where | What |
|---|---|
| Boot log | `T26 AQT7 fw 0.1.0 (83c5bc60089d) starting on t26_aqt7` |
| Shell | `app version` (`0.1.0`), `app version-extended` (`0.1.0+0`), `app build-version` (the commit), from Zephyr's `CONFIG_APP_VERSION_SHELL` |
| Code | `fw_version_string()`, and `fw_version_get_hash()` for the hash as 6 bytes (e.g. for a CAN frame) |
| Flash | The `fw_info` block, below |

**`fw_info` block.** `src/fw_version.c` puts a 48-byte block right after the vector table (`0x08000150` in the
current build; `fw_info` in `build/zephyr/zephyr.map`), so a bootloader or updater can read the version from
an image without running it. It starts with the magic bytes `AQ7I`; search for them (4-byte aligned) in the
first 1 KiB of the image. Little-endian:

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | `magic` = `0x49375141` (bytes `AQ7I`) |
| 4 | 1 | `info_version` = 1, the layout version: bump `FW_INFO_VERSION` whenever the layout changes |
| 5 | 3 | `major`, `minor`, `patch` |
| 8 | 4 | `tweak` |
| 12 | 32 | `build`: `APP_BUILD_VERSION` as text, NUL-terminated |
| 44 | 4 | reserved (0) |

There is **no CAN frame for the version yet**. It will be added once the DBC defines the message. The frame
code goes in `can_tx.c`, like the other frames, and gets the bytes from `fw_version_get_hash()`.

**Versions are set by CI** (`.github/workflows/firmware.yml`):

- On every push and pull request: the unit tests (twister on `native_sim`) and a firmware build, both the
  development build and the car build (`CONFIG_SHELL=n`).
- On a push to `main` that passes: `scripts/ci_bump_version.sh` picks the next version, CI commits `VERSION`
  (`chore: release vX.Y.Z [skip ci]`), tags it `vX.Y.Z`, builds that commit and publishes a **GitHub Release**
  with `t26_aqt7-vX.Y.Z.{hex,bin,elf}`, the car build `t26_aqt7-vX.Y.Z-car.*` and `version.json`
  (version + commit).
- The next version is the last tag's `PATCHLEVEL` + 1. To release a new **MINOR** or **MAJOR**, edit
  `VERSION` in the pull request (e.g. `0.2.0`): if it is above the last tag, CI keeps it. Don't edit
  `PATCHLEVEL` by hand otherwise.
- The tags are **lightweight on purpose**. `git describe` ignores lightweight tags, so `APP_BUILD_VERSION` stays
  the plain commit hash even on a released commit. Don't create annotated tags (`git tag -a`) in this
  repository: on a commit with one, the build would report only the tag name, and `fw_version_get_hash()`
  would fail.
- If `main` has branch protection, it must allow GitHub Actions to push, or the release job fails.

---

## Debugging

### Console: log and shell over the ST-LINK

The firmware has a **shell** (a command line) on two transports, each showing the full log:

| Transport | Prompt | Needs |
|---|---|---|
| **RTT** over SWD | `rtt:~$` | **Only the ST-LINK cable**, nothing else |
| **USART1**, 115200 8N1 | `uart:~$` | PA9/PA10 wired to the ST-LINK's VCP or to a USB-UART adapter |

**With only the ST-LINK plugged in.** From the repository folder, with the virtual environment active:

```bash
west rtt -d build --runner openocd
```

It resets the board, starts OpenOCD (from the Zephyr SDK, configured by `boards/t26/t26_aqt7/support/openocd.cfg`)
and opens an interactive terminal. `Ctrl+C` stops it. Only one tool can use the ST-LINK at a time: close
STM32CubeProgrammer or a running `west debugserver` first.

How it works: RTT (SEGGER Real-Time Transfer) is a pair of RAM buffers in the firmware, 4 KB out and 128 bytes
in. OpenOCD reads and writes them through the SWD cable while the firmware runs, and serves them on TCP port
5555; `west rtt` connects a terminal to that port.

- With nothing reading RTT (e.g. on the car), the shell notices and drops its output: the firmware never waits
  for the debugger.
- To use the RTT shell during a debug session, run `west debugserver -d build --runner openocd --rtt-server`
  and connect a raw TCP terminal (e.g. PuTTY, connection type *Raw*) to `localhost:5555`.

**Over USART1.** Wire PA9 to STDC14 pin 14 (`T_VCP_TX`) and PA10 to pin 13 (`T_VCP_RX`) of the STLINK-V3MINIE
(or use a 3.3 V USB-UART adapter), then open the port (`python -m serial.tools.list_ports -v` lists them):

```bash
python -m serial.tools.miniterm COM5 115200
```

### Log

Besides start-up messages, CAN health changes and watchdog warnings, the debug thread logs the sensor values
every `CONFIG_AQT7_DEBUG_PRINT_MS`. Set `CONFIG_AQT7_DEBUG_PRINT_MS=0` in `prj.conf` to turn it off, or e.g. `1000` for once per second. A healthy
start shows `IWDG running, 2000 ms timeout` and, ~100 ms later, `Suspension zeroed: ...`.

### Shell commands and sensor simulation

Type `help` for the full list, and press `Tab` to complete commands and arguments. Useful ones:

| Command | Does |
|---|---|
| `kernel uptime` | Time since boot |
| `kernel thread list` | Every thread with its state, priority and stack usage |
| `device list` | Drivers and whether they started (ADC, CAN, watchdog, ...) |
| `can show can@40006400` | CAN1 state and error counters (`can@40006800` = CAN2) |
| `can send can@40006400 123 01 02` | Send a frame on the **real** bus (here ID 0x123, 2 bytes). The board doesn't receive its own frames, so this tests the bus and the other nodes, not this firmware's RX |
| `gpio get gpio@40020800 10` | Read a pin (here PC10, the brake light) |
| `log halt` / `log go` | Pause / resume the log on this console while typing |

`sim` replaces an ADC input with a typed value, which then goes through the same filter, conversions, brake
light and CAN frames as a real sample:

| Command | Does |
|---|---|
| `sim set <channel> <0-4095>` | Simulate raw ADC counts (`brake`, `susp_l`, `susp_r`, `ntc`) |
| `sim mv <channel> <0-3300>` | Simulate the pin voltage in mV |
| `sim off [channel]` | Back to the real input (all channels if none given) |
| `sim show` | Override state and the resulting sensor values |

Example: after `sim mv brake 2000`, `sim show` prints

```
channel  source  raw   filtered  pin mV  value
brake    SIM     2482      2482    2000  48.52 bar
```

and the brake light turns on; `sim off` goes back to the real sensor.

Turn `CONFIG_AQT7_SIM` off for the car, so a stray console byte can't fake a sensor (see
[Building for the car](#building-for-the-car)).

### Testing the CAN buses on the bench

The repository has a CANgaroo workspace and a short note for each bus (in Portuguese):

| File | Contents |
|---|---|
| `CAN1_Workspace.cangaroo`, `CANgaroo_CAN1.md` | Autonomous bus: interface settings, and a `0x710` (AQT1) test message with 50 bar of front pressure every 50 ms |
| `CAN2_Workspace.cangaroo`, `CANgaroo_CAN2.md` | DATA bus: interface settings only (the board only transmits on CAN2) |

> [!IMPORTANT]
> **CAN not working on the bench? Turn on the 120 Ω terminator on the CAN adapter first** (usually a switch or
> jumper on the adapter). Without it no frame is acknowledged: the board logs `ACK - nobody acknowledged our
> frame` and goes `PASSIVE`. Details in step 1 below.

Checklist when the adapter (e.g. a USB2CAN) shows nothing from the board, or the board logs CAN errors:

1. **Termination.** A CAN bus needs 120 Ω between CANH and CANL at each end. With the power off, measure
   CANH–CANL: 60 Ω = both ends, 120 Ω = one end (enough on a short bench cable), open = none. With no
   termination the board logs `ACK - nobody acknowledged our frame` and goes `PASSIVE` with TEC stuck at 128.
2. **Adapter settings** (in CANgaroo: Measurement Setup → the interface). Stop the measurement before changing
   them, and start it again so they take effect:
   - bitrate **1000000** in the *CAN / arbitration phase* field (the *data phase* fields are for CAN FD and
     don't matter here),
   - sample point **75 %** for CAN1, **87.5 %** for CAN2, like the board,
   - **Bus Monitoring Mode (Listen Only) off**, or the adapter never acknowledges the board's frames.

   A wrong bitrate makes the adapter answer the board's frames with error frames: the board logs
   `BIT RECESSIVE - sent 1 read 0`.
3. **Wiring:** CANH to CANH, CANL to CANL, and a common GND between the adapter and the board.
4. **Check both directions:**
   - board → adapter: CAN1 `0x770` should arrive every 50 ms, CAN2 `0x770` every 125 ms;
   - adapter → board: send `0x710` and the board's sensor log shows `Front brake 50.00 bar (coded 500, alive)`.
5. **Error counters:** `can show can@40006400` (CAN1) or `can show can@40006800` (CAN2), run a few times.
   On a healthy bus TEC/REC go down to 0 and the error counts stop growing.

The `last error` in a CAN health log line is the most recent error type **seen since boot**, not necessarily a
current one: it is logged again whenever the state changes, e.g. `OK - error active` with an old `BIT RECESSIVE`
from the moment a cable was plugged in.

### `acq7` snapshot

`debug.c` refreshes the global `struct acq7_debug acq7` every 10 ms:

| Field | Contents |
|---|---|
| `time_ms` | Uptime |
| `adc.raw[4]`, `adc.filtered[4]`, `adc.volts[4]` | Per channel: latest sample, moving average, filtered value at the pin (V). Order: brake, susp L, susp R, NTC1 |
| `sensors.*` | Rear and front brake (bar), suspensions (mm), NTC1 (°C) |
| `io.brake_light`, `io.heartbeat` | Pin states. `brake_light` is logical: **1 = light on** |
| `front.coded`, `front.alive`, `front.age_ms`, `front.rx_count` | 0x710 raw value, arriving within 500 ms, time since the last one, frames received |
| `can.tx.autonomous_aqt7`, `can.tx.data_aqt7` | Last payload queued on each bus |
| `can.autonomous`, `can.data` | `struct can_bus_status` of each bus: state, TEC/REC, error counts per type, last error, bus-off count, restarts, TX/RX counters |

To watch it while the firmware runs, add `acq7` to a live view that reads RAM over the ST-LINK without halting
the CPU: **CLion Live Watches**, **STM32CubeIDE Live Expressions** (debug configuration pointing at
`build\zephyr\zephyr.elf`), or **STM32CubeMonitor** (load the ELF, plot variables). Thread-local variables can't
be watched this way; everything worth watching is in `acq7`.

### Debugger

`west debug -d build` flashes the board, starts the ST-LINK GDB server (from STM32CubeCLT) and opens GDB stopped
at reset. `west attach -d build` connects without flashing or resetting. `west debugserver -d build` only starts
the GDB server (port 61234), e.g. for a CLion **Remote Debug** configuration with the Zephyr SDK's
`arm-zephyr-eabi-gdb.exe` and `build\zephyr\zephyr.elf` as symbol file.

For stepping through code, build with fewer optimisations (not for the car):

```bash
west build -d build -p -- -DCONFIG_DEBUG_OPTIMIZATIONS=y
```

---

## Changes from the STM32CubeIDE firmware

All of the HAL firmware's functionality is ported. Deliberate differences:

| | STM32CubeIDE (removed) | Zephyr |
|---|---|---|
| Scheduling | Super-loop with 10/50/100/125 ms tasks | One thread per job |
| Moving average | 1 sample every 50 ms → ~500 ms window | 1 sample every 10 ms → 100 ms window (faster brake light) |
| Suspension zero | ~500 ms after boot | ~100 ms after boot |
| Brake light polarity | PC10 LOW while braking, so the light was on when **not** braking | PC10 HIGH while braking (the MOSFET gate has a pull-down, so HIGH = on) |
| 0x710 decoding | Bytes 0–1, any length | DBC unpack, which needs **at least 3 bytes** (AQT1 in the DBC). If AQT1 sends 2 bytes, the front pressure stays 0 |
| Stale CAN frames | An older copy of the same ID still waiting in a mailbox is aborted | The new frame is dropped if the mailboxes are full |
| Watchdog | Fed by the main loop every 10 ms | Fed only while every thread checks in |
| CAN error types | Polled from the last error code register every 10 ms | Counted by the driver in the CAN error interrupt. On a bus with continuous errors (e.g. no other node) this is one interrupt per error |
| CAN health check | Every 10 ms (needed: each poll read and cleared the last error code register, so slower polling lost errors) | Every 100 ms: the errors are already counted in the interrupt, so nothing is lost between checks |
| `acq7.io.brake_light` | Raw pin level (0 = on) | Logical (1 = on) |
| Log | `printf` in Portuguese, UART only | Zephyr log in English, UART + RTT |

Unit tests cover `conversions.c` and `fw_version.c` only (see [Unit tests](#unit-tests)); the other modules have none yet.

---

## Development setup

The firmware is built with **west**, Zephyr's command-line tool. West works in a **workspace**: a folder that
holds this repository next to the Zephyr source and the modules it needs. This repository is the workspace's
*manifest repository*. Its `west.yml` pins the Zephyr version (**v4.5.0**) and the modules to download
(`cmsis_6`, `hal_stm32`, `segger` for RTT logging), so every team member gets the same versions.

Only this repository is tracked in git. The rest of the workspace is downloaded by `west update` and must
never be committed:

```
<workspace>\                     <- e.g. D:\t26ws, NOT a git repository
├── .venv\                       <- Python virtual environment (per PC)
├── .west\                       <- created by west init (per PC)
├── T26_Acquisition_7\           <- this repository (build\ inside it is git-ignored)
├── zephyr\                      <- downloaded by west update
└── modules\                     <- downloaded by west update (hal_stm32, cmsis_6, segger)
```

The steps below are for **Windows PowerShell**. **Linux (Ubuntu/Debian)** has its own section:
[Linux setup](#linux-setup).

### 1. Install the host tools (once per PC)

Follow the "Install dependencies" step of the
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)
for your OS. On Windows:

```bash
winget install Kitware.CMake Ninja-build.Ninja oss-winget.gperf Python.Python.3.12 Git.Git oss-winget.dtc wget 7zip.7zip
```

Close and reopen the terminal afterwards, so it picks up the new `PATH`.

You also need:

- the **Zephyr SDK 1.0.1** (the version required by Zephyr v4.5.0), installed once per PC. If you don't have
  it yet, install it in step 4 with `west sdk install`;
- **STM32CubeCLT** (STM32CubeProgrammer + ST-LINK GDB server + ST-LINK driver), for flashing and debugging.

### 2. Create the workspace and clone the repository

Use a **short path**: Zephyr builds on Windows can fail with long paths.

```bash
mkdir D:\t26ws
```

```bash
cd D:\t26ws
```

```bash
git clone git@github.com:FSLART/T26_Acquisition_7.git
```

### 3. Create the Python virtual environment and install west

```bash
py -3.12 -m venv .venv
```

```bash
.\.venv\Scripts\Activate.ps1
```

```bash
pip install west
```

If PowerShell refuses to run `Activate.ps1` ("running scripts is disabled on this system"), allow local
scripts for your user once, then activate again:

```bash
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

### 4. Initialise the workspace and download Zephyr

Run these from the workspace folder (`D:\t26ws`), with the virtual environment active (the prompt starts
with `(.venv)`). **Keep this order**: `west zephyr-export` needs the Python packages from step 4.3.

1. Initialise the workspace, using this repository's `west.yml`:

   ```bash
   west init -l T26_Acquisition_7
   ```

2. Download Zephyr and the modules (a few hundred MB):

   ```bash
   west update
   ```

3. Install Zephyr's Python requirements:

   ```bash
   pip install -r zephyr\scripts\requirements.txt
   ```

4. Register Zephyr with CMake:

   ```bash
   west zephyr-export
   ```

5. Set `t26_aqt7` as the workspace's default board (used by `west build` without `-b`, and by CLion):

   ```bash
   west config build.board t26_aqt7
   ```

6. Only if the Zephyr SDK 1.0.1 isn't installed yet:

   ```bash
   west sdk install
   ```

### 5. Build and flash

From the repository folder, with the virtual environment active:

```bash
cd D:\t26ws\T26_Acquisition_7
```

```bash
west build -p
```

```bash
west flash
```

The build goes to `build\` (git-ignored). `-p` (pristine) deletes the previous build first; leave it out for
faster rebuilds.

`west flash` uses STM32CubeProgrammer over SWD with an ST-LINK. Its command-line tool must be on your `PATH`
(adjust the version), then open a new terminal:

```
C:\ST\STM32CubeCLT_1.23.0\STM32CubeProgrammer\bin
```

Then open the log and shell with `west rtt -d build --runner openocd` (see
[Console](#console-log-and-shell-over-the-st-link)).

### Building for the car

The default build is for development: the shell (RTT and USART1) is on, `sim` can fake any sensor, and
`can send` can put frames on the bus. For the car, build without the shell, which also removes `sim`:

```bash
west build -p -- -DCONFIG_SHELL=n
```

The log stays on USART1. West prints warnings that the `CONFIG_*_SHELL` options from `prj.conf` were assigned
but got the value `n`; they are expected. To keep the shell and only remove `sim`, use `-DCONFIG_AQT7_SIM=n`
instead. The option is stored in the build folder: a later `west build -p` without it goes back to the
development build.

### Unit tests

`tests/conversions/` holds 13 ztest unit tests for `src/conversions.c` (brake pressure, suspension, NTC), with
expected values worked out by hand from the sensor datasheets and the board circuit. They run on the PC as a
`native_sim` program, which only builds on **Linux**: on Windows, use WSL (Ubuntu) with the Zephyr SDK and
Zephyr's Python requirements installed there. CI runs them on every push (see
[Firmware version](#firmware-version)).

`tests/version/` holds 6 tests for `src/fw_version.c`: commit hash parsing and the `fw_info` contents. It has
its own `VERSION` (1.2.3+4), not the firmware's, and the build needs git. From the repository folder:

```bash
west twister -T tests -p native_sim/native/64
```

The brake pressure tests assume the current calibration (`OFFSET_VOLTAGE = 0.614`, `CONVERSION_FACTOR = 1.0`).
After a re-calibration or a divider on PA7, update the expected values in `tests/conversions/src/main.c`.

### Linux setup

Same workspace layout as on Windows, with `/` in paths and `source .venv/bin/activate`. Tested on Ubuntu; the
package names are the Debian/Ubuntu ones.

1. Install the host tools (once per PC), then the ST-LINK udev rules (OpenOCD's package ships them) and give
   your user access to the USB probe:

   ```bash
   sudo apt update
   sudo apt install --no-install-recommends git cmake ninja-build gperf ccache dfu-util device-tree-compiler wget \
       python3-dev python3-pip python3-venv python3-setuptools python3-wheel xz-utils file make gcc \
       libsdl2-dev libmagic1 openocd
   sudo usermod -aG plugdev,dialout $USER
   ```

   Log out and back in so the groups apply. Check `cmake --version` is 3.20 or newer.

2. Create the workspace, clone the repository, create the virtual environment:

   ```bash
   mkdir -p ~/t26ws && cd ~/t26ws
   git clone git@github.com:FSLART/T26_Acquisition_7.git
   python3 -m venv .venv
   source .venv/bin/activate
   pip install west
   ```

3. Initialise the workspace and download Zephyr (same as Windows step 4):

   ```bash
   west init -l T26_Acquisition_7
   west update
   pip install -r zephyr/scripts/requirements.txt
   west zephyr-export
   west config build.board t26_aqt7
   west sdk install
   ```

4. Install **STM32CubeCLT** for Linux (`.sh` installer from st.com, free login required). It installs to
   `/opt/st/stm32cubeclt_<version>/`. Put the programmer on your `PATH` (adjust the version) in `~/.bashrc` or
   `~/.zshrc`:

   ```bash
   export PATH="$PATH:/opt/st/stm32cubeclt_1.23.0/STM32CubeProgrammer/bin"
   ```

5. Build and flash, from the repository folder with the virtual environment active:

   ```bash
   cd ~/t26ws/T26_Acquisition_7
   west build -p
   west flash
   ```

   The blinky check from [The t26_aqt7 board](#the-t26_aqt7-board), run from `~/t26ws`:

   ```bash
   west build -d build-blinky -b t26_aqt7 zephyr/samples/basic/blinky -p -- -DBOARD_ROOT=$HOME/t26ws/T26_Acquisition_7
   west flash -d build-blinky
   ```

6. **CLion:** open `~/t26ws/T26_Acquisition_7/CMakeLists.txt` as a project (File → Open → Open as Project).
   If CLion shows the generic *Open Project Wizard* (Debug profile, Ninja, `cmake-build-debug`) instead of
   the West settings, it did not find the workspace: cancel it and check that `~/t26ws/.west/` exists, i.e.
   the repository is *inside* the workspace folder. Then set **Settings → Build, Execution, Deployment →
   West**:

   | Setting | Value |
   |---|---|
   | Toolchain | Use default |
   | West executable | `~/t26ws/.venv/bin/west` (use the full path; CLion does not expand `~`) |
   | Board name | **Let CMake decide** |
   | Build directory | Outside the repository, e.g. `~/t26ws/build-clion` |

   If the CLion you use is the Snap, it may not see the venv or `/opt/st`: use the full paths above, or
   install the `.deb`/Toolbox build.

Every new terminal: `source ~/t26ws/.venv/bin/activate`. To update after a pull: `git pull && west update`.

Linux-specific problems:

| Error | Cause | Fix |
|---|---|---|
| `No debug probe detected` / `LIBUSB_ERROR_ACCESS` | No permission on the ST-LINK USB device | Install `openocd` (udev rules), add your user to `plugdev`, log out/in, replug the probe |
| `west flash` can't find `STM32_Programmer_CLI` | CubeProgrammer not on `PATH` | Step 4 above; open a new terminal |
| `/dev/ttyACM0: Permission denied` | User not in `dialout` | `sudo usermod -aG dialout $USER`, log out/in |
| `west: command not found` | Virtual environment not active | `source ~/t26ws/.venv/bin/activate` |

### The t26_aqt7 board

The board definition is in `boards/t26/t26_aqt7/`. The pins and clocks are copied from `T26_Acquisition_7.ioc`,
which is only a reference: Zephyr doesn't use CubeMX or its generated code.

| File | Contents |
|---|---|
| `board.yml` | Board name and SoC (`stm32f412rx`) |
| `Kconfig.t26_aqt7` | Selects the SoC |
| `t26_aqt7.dts` | Devicetree: HSI → PLL 96 MHz, APB1 48 MHz, APB2 96 MHz; USART1 console; heartbeat (PC11, aliases `led0`/`heartbeat`) and brake light (PC10, active high, alias `brake-light`); CAN1 (PA11/PA12, alias `can-autonomous`, 75 % sample point) and CAN2 (PB12/PB13, alias `can-data`, 87.5 %), both 1 Mbit/s with RX pull-ups; ADC1 channels 7/8/9/14 (PA7, PB0, PB1, PC4), 12 bit, 480-cycle sampling, listed in `zephyr,user`; IWDG (alias `watchdog0`) |
| `t26_aqt7_defconfig` | Default Kconfig options for the board (MPU, console) |
| `board.cmake` | Runners: STM32CubeProgrammer (`west flash`, software reset), ST-LINK GDB server (`west debug`), OpenOCD (`west rtt --runner openocd`) and J-Link |
| `support/openocd.cfg` | OpenOCD setup for the ST-LINK on SWD, used to read RTT |
| `t26_aqt7.yaml` | Board metadata for Zephyr's test runner (twister) |

`CMakeLists.txt` adds this folder to `BOARD_ROOT`, so the app finds the board on its own. Because `led0` points
at the heartbeat, Zephyr's blinky sample runs on the board unmodified, which checks the clocks, the board files
and flashing without the application (from the workspace folder; keep the `-DBOARD_ROOT=...` argument in single
quotes in PowerShell, or it is split at the drive letter's `:`):

```bash
west build -d build-blinky -b t26_aqt7 zephyr\samples\basic\blinky -p -- '-DBOARD_ROOT=D:/t26ws/T26_Acquisition_7'
```

```bash
west flash -d build-blinky
```

### Using CLion

1. **File → Open** → `D:\t26ws\T26_Acquisition_7\CMakeLists.txt` → **Open as Project**. The whole repository
   shows in the project tree, and CLion's git integration sees it.
2. CLion recognises the folder as a **Zephyr West project**, so it's configured in the West settings, not the
   CMake settings (**Settings → Build, Execution, Deployment → West**, or the "Open West settings" link):

| Setting | Value |
|---|---|
| Toolchain | Use default (only CMake and Ninja are used from it; the compiler comes from the Zephyr SDK) |
| West executable | `D:\t26ws\.venv\Scripts\west.exe` |
| Board name | **Let CMake decide** |
| Build directory | Outside the repository, e.g. `D:\t26ws\build-clion` |

**Board name:** `t26_aqt7` isn't in CLion's board list, because the list only contains Zephyr's own boards.
Leave it on "Let CMake decide": west then uses the default from step 4.5 (`west config build.board t26_aqt7`).
Without that setting west refuses to reuse an existing build folder ("Cached board not defined").
`CMakeLists.txt` also defaults to `t26_aqt7`, as a fallback.

Using the virtual environment's `west.exe` also makes the build use its Python, which has Zephyr's
requirements installed. You don't need to set `ZEPHYR_BASE`: `west zephyr-export` already registered Zephyr
with CMake.

### Every new terminal

`west` is only available inside the virtual environment. Activate it each time you open a terminal:

```bash
D:\t26ws\.venv\Scripts\Activate.ps1
```

### After every `git pull`

Run `west update` from the workspace folder. It downloads a new Zephyr version or module if `west.yml` changed,
and does nothing otherwise. If the Zephyr version changed, also redo step 4.3 and check the SDK version the new
Zephyr needs in `zephyr\SDK_VERSION`.

```bash
west update
```

### Troubleshooting

| Error | Cause | Fix |
|---|---|---|
| `west` is not recognized | Virtual environment not active | Activate `.venv` (see [Every new terminal](#every-new-terminal)) |
| `ModuleNotFoundError: No module named 'jsonschema'` (or another module) on `west zephyr-export` / `west build` | Zephyr's Python requirements are missing | Run step 4.3, then try again |
| `CMake is not installed or cannot be found; cannot build.` | The terminal was opened before CMake was added to `PATH` | Open a new terminal, then activate `.venv` again |
| `running scripts is disabled on this system` | PowerShell execution policy | See step 3 |
| Build fails on `CONFIG_USE_SEGGER_RTT` / a missing module | `west.yml` gained a module since your last `west update` | Run `west update` |
| `BOARD_ROOT element without a 'boards' subdirectory` / board `t26_aqt7` not found, and the command shows `D: /...` | PowerShell split `-DBOARD_ROOT=D:/...` at the `:` | Put the argument in single quotes |
| `Cached board not defined, please provide it` | The build folder exists but has no board (e.g. an earlier configure failed) | Run `west config build.board t26_aqt7`, delete the build folder, build again |
| CAN log: `ACK - nobody acknowledged our frame`, `PASSIVE`, or `BIT RECESSIVE`; the adapter shows nothing | No termination, adapter at another bitrate or in listen-only mode | See [Testing the CAN buses on the bench](#testing-the-can-buses-on-the-bench) |
| `west flash` can't find `STM32_Programmer_CLI` | STM32CubeProgrammer not on `PATH` | See [Build and flash](#5-build-and-flash) |
| `west flash`: `failed to erase memory` | Flash read-out protection or PCROP set in the option bytes (check with `STM32_Programmer_CLI -c port=swd -ob displ`) | Reset to RDP level 0 with PCROP off (`-ob RDP=0xAA SPRMOD=0 nWRP0=1 ... nWRP11=1`); this mass-erases the chip. Power-cycle the board afterwards |
| `No debug probe detected` | ST-LINK unplugged or held by another tool | Plug it in; close STM32CubeProgrammer, CubeIDE or a running OpenOCD/GDB server |

STM32CubeCLT also installs CMake and Ninja. If it comes first in your `PATH`, those copies are used. That
works, but `cmake --version` shows the CubeCLT version.

---

## Calibration checklist

| Constant | File | When to change |
|---|---|---|
| `OFFSET_VOLTAGE` | `src/conversions.c` – `MeasureBreakPressure` | Rear pressure not 0 at rest: copy `acq7.adc.volts[0]` with the pedal released |
| `CONVERSION_FACTOR` | `src/conversions.c` – `MeasureBreakPressure` | After adding a divider on PA7 |
| `SENSOR_VOLTAGE` | `src/conversions.c` | Measure the real 5 V rail (NTC and suspension) |
| `PRESSURE_LEVEL_FRONT` / `_REAR` | `src/brake_light.c` | Brake-light threshold (bar) |
| `FRONT_PRESSURE_TIMEOUT_MS` | `src/can_rx.c` | Keep ≥ 3× the `0x710` period |
| `SAMPLE_PERIOD_MS`, `AVG_SAMPLES` | `src/sensors.c` | Filter window (period × samples) |
| Ride height | automatic at boot | Power up with the car at rest on its wheels |
