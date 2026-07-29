"""End-to-end probe using ddcci-screen-tuning's BLE ambient reader."""

import argparse
import sys
import threading
import time
from pathlib import Path


APP_DIR = Path(__file__).resolve().parents[2] / "python" / "ddcci-screen-tuning"
sys.path.insert(0, str(APP_DIR))

from control_sources.ambient_sensor import BleNusAmbientReader  # noqa: E402


class ProbeController:
    def __init__(self):
        self.received = threading.Event()
        self.last_measurement = None

    def on_measurement(self, **measurement):
        self.last_measurement = measurement
        self.received.set()
        print("Measurement:", measurement)

    def close(self):
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--set-refresh-ms",
        type=int,
        help="Apply and verify a sensor refresh interval after connecting.",
    )
    parser.add_argument(
        "--exercise-runtime-config",
        action="store_true",
        help="Apply and verify every runtime sensor field, then restore it.",
    )
    args = parser.parse_args()

    controller = ProbeController()
    reader = BleNusAmbientReader(controller)
    if not reader.start():
        raise SystemExit(f"Unable to start BLE reader: {reader.last_error}")

    try:
        deadline = time.monotonic() + 120.0
        while time.monotonic() < deadline and not controller.received.wait(0.25):
            pass
        if not controller.received.is_set():
            raise SystemExit(f"No BLE measurement received: {reader.last_error}")
        reader.request_config()
        config_deadline = time.monotonic() + 45.0
        while reader._last_config is None and time.monotonic() < config_deadline:
            time.sleep(0.25)
        print("Sensor config:", reader._last_config)

        def apply_and_verify(changes):
            previous_config_at = reader._last_config_at or 0.0
            controller.received.clear()
            if not reader.apply_config(changes):
                raise SystemExit("Unable to queue BLE sensor configuration.")
            apply_deadline = time.monotonic() + 45.0
            while (
                (reader._last_config_at or 0.0) <= previous_config_at
                and time.monotonic() < apply_deadline
            ):
                time.sleep(0.25)
            if reader._last_config_at is None or reader._last_config_at <= previous_config_at:
                raise SystemExit(
                    f"No config confirmation received: {reader.last_error}"
                )
            for name, expected in changes.items():
                actual = reader._last_config.get(name)
                if actual != expected:
                    raise SystemExit(
                        f"Config mismatch for {name}: expected {expected!r}, "
                        f"received {actual!r}"
                    )
            print("Applied config:", changes, "->", reader._last_config)
            reader.request_measurement(force=True)
            if not controller.received.wait(15.0):
                raise SystemExit(
                    f"No measurement after config update: {reader.last_error}"
                )
            print("Post-config measurement:", controller.last_measurement)

        if args.set_refresh_ms is not None:
            apply_and_verify({"refreshMs": args.set_refresh_ms})

        if args.exercise_runtime_config:
            original = dict(reader._last_config)
            tests = (
                {"refreshMs": 500},
                {"publishLuxChangePercent": 2.5},
                {"publishMaxIntervalSeconds": 45},
                {"publishMode": "interval"},
            )
            for changes in tests:
                apply_and_verify(changes)
            restore = {
                name: original[name]
                for name in (
                    "refreshMs",
                    "publishLuxChangePercent",
                    "publishMaxIntervalSeconds",
                    "publishMode",
                )
            }
            apply_and_verify(restore)
    finally:
        reader.stop()


if __name__ == "__main__":
    main()
