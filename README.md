# TSL2591 Lux Meter

Arduino firmware for a TSL2591-based lux meter. It was initially built for
`ddcci-screen-tuning`, where a host script needs stable ambient light readings
to tune display brightness.

## Features

- Automatic TSL2591 gain ranging.
- Runtime lux stabilization with saturation and spectral-overload handling.
- Self-calibrated lux estimation from trusted raw readings when spectral
  overload makes direct lux readings unreliable.
- JSON line output over USB Serial.
- Runtime configuration over USB Serial, also using JSON lines.
- Two publish modes:
  - `auto`: publish when lux changes enough, with a maximum interval fallback.
  - `interval`: publish at a fixed interval.

## Serial Protocol

Measurements are emitted as one JSON object per line:

```json
{"lux":123.456,"visible":789,"ir":10,"full":799,"gain":"med","saturated":false,"adcOverRange":false,"spectral":false,"held":false,"estimated":false,"raw":31.960,"irRatio":0.013,"luxPerRaw":1.600,"fw":"tsl2591-autorange-2026-07-02-12"}
```

Request the current reading:

```json
{"cmd":"get"}
```

Read runtime configuration:

```json
{"cmd":"config.get"}
```

Update runtime configuration:

```json
{"cmd":"config.set","refreshMs":250,"publishLuxChangePercent":2.0,"publishMaxIntervalSeconds":60,"publishMode":"auto"}
```

Runtime configuration is not persisted after reset.

## Roadmap

- BLE transport for wireless configuration and readings.
- Low-power operation using sensor interrupts instead of periodic polling.

## Build

This project is configured for Arduino Maker Workshop / Arduino CLI through
`sketch.yaml`. Required libraries are listed there, including `ArduinoJson`,
`Adafruit TSL2591 Library`, and `Adafruit Unified Sensor`.
