# TSL2591 BLE Lux Meter

Firmware for a TSL2591 ambient-light sensor connected to a Seeed XIAO
nRF52840 Sense. It is the wireless sensor used by
[`ddcci-screen-tuning`](https://github.com/nicklausFR/ddcci-screen-tuning) to
adapt monitor brightness and contrast to ambient light.

## Features

- Automatic TSL2591 gain ranging.
- Stabilization during gain changes, ADC saturation and spectral overload.
- Self-calibrated lux estimation when a direct TSL2591 lux result is
  unreliable.
- Compact binary measurements over Bluetooth Low Energy.
- Runtime configuration over Nordic UART Service (NUS).
- Fast initial connection for reliable Windows service discovery, followed by
  low-power connection parameters.
- XIAO user LEDs disabled and onboard QSPI flash placed in deep power-down.
- Automatic recovery from abandoned or stale Windows BLE sessions.
- `auto` publication on significant lux changes, or fixed `interval` mode.

The consolidated firmware version is
`tsl2591-ble-nus-2026-07-29-5`.

## BLE protocol

The peripheral advertises as `LuxSensor` with the static random address
`CE:5A:29:91:42:7E`. It exposes Nordic UART Service:

- service: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
- RX/write: `6e400002-b5a3-f393-e0a9-e50e24dcca9e`
- TX/notify: `6e400003-b5a3-f393-e0a9-e50e24dcca9e`

Advertising is connectable and stops after a connection. Therefore,
`LuxSensor` normally disappears from Bluetooth scan results while
`ddcci-screen-tuning` is connected. Advertising restarts after a real
disconnection.

### Measurements

Measurements use one fixed 15-byte little-endian packet:

```text
<2sBBfHHHB
```

| Field | Type | Description |
| --- | --- | --- |
| magic | 2 bytes | ASCII `LT` |
| version | `uint8` | Protocol version, currently `1` |
| quality | `uint8` | Quality flags |
| lux | `float32` | Lux value; may be `NaN` when no valid value exists |
| visible | `uint16` | Visible channel count |
| infrared | `uint16` | Infrared channel count |
| full | `uint16` | Full-spectrum channel count |
| gain | `uint8` | Gain index: low, medium, high or maximum |

Quality flags are:

- bit 0: saturated or ADC over range;
- bit 1: spectral overload;
- bit 2: value held during stabilization;
- bit 3: lux value estimated;
- bit 4: gain changed during this measurement.

### Commands

Commands and responses are newline-delimited JSON written to NUS RX and
notified on NUS TX.

Request the cached measurement:

```json
{"cmd":"get"}
```

Read runtime configuration:

```json
{"cmd":"config.get"}
```

Update runtime configuration:

```json
{"cmd":"config.set","refreshMs":250,"publishLuxChangePercent":2.0,"publishMaxIntervalSeconds":60,"publishMode":"auto","integrationMs":200,"discardAfterGainChange":true}
```

Reset runtime configuration:

```json
{"cmd":"config.reset"}
```

Keep the connection alive:

```json
{"cmd":"ping"}
```

Runtime configuration is not persisted across a reset.

## Connection and power behavior

Windows needs a responsive link while it discovers GATT services. The firmware
therefore starts with a requested 15–30 ms connection interval and no
peripheral latency. Fifteen seconds after the latest host command, it requests
the steady-state parameters:

- 200 ms connection interval;
- no peripheral latency;
- 20 s supervision timeout;
- 0 dBm transmit power.

The central operating system makes the final parameter choice.

The previous 500 ms / latency 4 / -8 dBm profile caused real link losses on
Windows shortly after the connection parameter update. The current profile
intentionally spends a little more radio time to avoid repeated GATT discovery
and reconnection.

The peripheral supports an ATT MTU up to 247 bytes and three queued
notifications. This matters because a complete JSON configuration response is
larger than the default 20-byte ATT payload. Measurements still fit in one
15-byte notification.

## Windows / WinRT BLE issue

Windows WinRT can retain an incomplete or abandoned GATT connection after an
application is killed, restarted during service discovery, or disconnected
while writes are still pending. Typical symptoms are:

- `LuxSensor` is no longer visible in scans;
- the scan reports “not found” even though the board is powered;
- connection succeeds at link level but GATT service discovery times out;
- the application repeatedly connects and loses `Sensor`;
- a second client cannot connect because Windows still owns the first link.

There are two cases to distinguish:

1. **Normal:** `LuxSensor` disappears from scan lists while the application is
   connected, because connectable advertising has stopped.
2. **Stale session:** no measurements arrive and Windows cannot finish GATT
   discovery or reconnect.

The firmware and companion application use several mitigations:

- a valid, stable random-static BLE address;
- fast advertising and fast initial connection parameters;
- service-filtered WinRT discovery;
- explicit confirmation of the sensor configuration and first measurement
  before the source is reported as available;
- NUS `write without response` for normal commands and confirmed writes for
  the short heartbeat;
- a host heartbeat every 30 seconds;
- a 120-second firmware command watchdog which disconnects an abandoned
  central and resumes advertising;
- an application-level force-reconnect action.

If Windows still holds a stale session:

1. close every `ddcci-screen-tuning` instance;
2. leave the XIAO powered and wait at least 120 seconds;
3. relaunch the application once;
4. if GATT still times out, turn the Windows Bluetooth radio off and on, wait
   for the 20-second supervision timeout, then relaunch.

Power-cycling or reflashing the XIAO should not normally be necessary.

## ddcci-screen-tuning integration test

The companion application searches for `AMBIENT_BLE_NAME: LuxSensor`, decodes
the binary packet, applies the saved runtime configuration, keeps the link
alive and reconnects after interruptions.

From this repository, run:

```powershell
py -3.11 .\test_ddcci_ble_integration.py
```

Exercise every runtime field and restore the original configuration:

```powershell
py -3.11 .\test_ddcci_ble_integration.py --exercise-runtime-config
```

`ble_adv_receiver.py` is retained only as a diagnostic for the earlier
connectionless-advertising experiment. The current firmware does not emit
that manufacturer payload.

## Idle connection power test

`ble_idle_power_test/ble_idle_power_test.ino` is a separate minimal sketch for
measuring an established but idle BLE link without the TSL2591 or USB Serial.
It exposes a static Battery Service for Windows GATT compatibility, disables
the connection LED, puts QSPI flash into deep power-down and requests the same
low-power connection parameters.

After flashing the test sketch:

```powershell
py -m pip install bleak
py .\ble_idle_power_client.py
```

For a meaningful current measurement, disconnect USB after flashing, power
the XIAO through its battery input, and compare advertising current with idle
connected current.

## Build and upload

The project is configured for Arduino CLI / Arduino Maker Workshop in
`sketch.yaml`.

Compile:

```powershell
arduino-cli compile --profile profile-1782939730789 --output-dir build .
```

Upload, replacing `COM12` if necessary:

```powershell
arduino-cli upload --profile profile-1782939730789 --port COM12 --input-dir build .
```

`update-workshop-port.ps1` can update the port stored in `sketch.yaml`.
Required libraries include ArduinoJson, Adafruit TSL2591, Adafruit Unified
Sensor and Adafruit BusIO; Bluefruit and the nRF52 support libraries come from
the Seeed nRF52 board package.
