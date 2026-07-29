# TSL2591 Lux Meter

Arduino firmware for a TSL2591-based lux meter. It was initially built for
[`ddcci-screen-tuning`](https://github.com/nicklausFR/ddcci-screen-tuning), where
a host script needs stable ambient light readings to tune display brightness.

## Features

- Automatic TSL2591 gain ranging.
- Runtime lux stabilization with saturation and spectral-overload handling.
- Self-calibrated lux estimation from trusted raw readings when spectral
  overload makes direct lux readings unreliable.
- Compact binary measurements and JSON runtime configuration over connected
  BLE.
- Nordic UART Service transport, directly supported by the companion Python
  application.
- Low-power BLE parameters and XIAO onboard QSPI deep power-down.
- Two publish modes:
  - `auto`: publish when lux changes enough, with a maximum interval fallback.
  - `interval`: publish at a fixed interval.

## BLE protocol

The peripheral advertises as `LuxSensor`. It exposes Nordic UART Service:

- RX/write: `6e400002-b5a3-f393-e0a9-e50e24dcca9e`
- TX/notify: `6e400003-b5a3-f393-e0a9-e50e24dcca9e`

Measurements use a fixed 15-byte little-endian packet described by
`<2sBBfHHHB`: magic `LT`, protocol version, quality flags, lux as a 32-bit
float, visible, infrared and full-spectrum counts as 16-bit integers, then the
gain index. Commands and configuration responses remain newline-delimited
JSON.

Request the latest cached reading without triggering a new sensor measurement:

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

Runtime configuration is not persisted after reset.

The BLE link is connectable only while advertising. Advertising stops
automatically after connection and restarts after disconnection. The requested
idle parameters are a 500 ms connection interval, peripheral latency 4, and
-8 dBm transmit power; Windows makes the final parameter choice.

The host sends a heartbeat every 30 seconds. If no command arrives for 90
seconds, the firmware disconnects the stale central and resumes advertising.
This recovers automatically when a host process is killed without closing its
WinRT BLE session.

## ddcci-screen-tuning integration

The companion application defaults to `AMBIENT_SENSOR_TRANSPORT: ble` and
searches for `AMBIENT_BLE_NAME: LuxSensor`. It keeps the connection alive,
reconnects after interruptions, decodes binary measurements, reassembles JSON
configuration responses, and uses the same sensor configuration UI as before.

Run the end-to-end probe from this directory:

```powershell
python.exe .\test_ddcci_ble_integration.py
```

`ble_adv_receiver.py` remains only as a legacy diagnostic for the earlier
connectionless advertising experiment; the current firmware does not emit that
manufacturer payload.

## BLE idle connection power test

`ble_idle_power_test/ble_idle_power_test.ino` is a separate minimal sketch for
measuring the current of an established but idle BLE connection. It does not
initialize the TSL2591 or USB Serial. It exposes only Bluefruit's standard
Battery Service with a static value for Windows GATT compatibility, disables
the automatic connection LED, places the XIAO's onboard QSPI flash in deep
power-down, requests a 500-millisecond connection interval with a peripheral
latency of 4, and suspends the Arduino loop task.

After flashing that sketch, install Bleak and start the PC client:

```powershell
py -m pip install bleak
py ble_idle_power_client.py
```

The nRF advertises as `LuxIdle` while disconnected. Connectable advertising
stops automatically after the PC connects and restarts after a disconnection.
The initial BLE connection and parameter negotiation necessarily exchange link
and GATT control packets; no application data is exchanged afterward. The
central operating system has the final say on the requested connection
parameters and may substitute different values.

For a meaningful battery-current measurement, disconnect USB after flashing
and power the XIAO through its battery input. Measure once while advertising
and once after the client reports `Connecté`.

## Build

This project is configured for Arduino Maker Workshop / Arduino CLI through
`sketch.yaml`. Required libraries are listed there, including `ArduinoJson`,
`Adafruit TSL2591 Library`, and `Adafruit Unified Sensor`.
