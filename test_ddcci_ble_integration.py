"""End-to-end probe using ddcci-screen-tuning's BLE ambient reader."""

import argparse
import asyncio
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
        return True

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

        ready_deadline = time.monotonic() + 30.0
        while not reader.available and time.monotonic() < ready_deadline:
            time.sleep(0.05)
        if not reader.available:
            raise SystemExit(
                f"BLE reader did not finish connecting: {reader.last_error}"
            )

        def write_json(payload):
            future = asyncio.run_coroutine_threadsafe(
                reader._write_json_async(payload),
                reader.event_loop,
            )
            try:
                if not future.result(timeout=15.0):
                    raise SystemExit(f"BLE write was not accepted: {payload}")
            except Exception as exc:
                raise SystemExit(f"BLE write failed for {payload}: {exc}") from exc

        previous_config_at = reader._last_config_at or 0.0
        write_json({"cmd": "config.get"})
        config_deadline = time.monotonic() + 45.0
        while (
            (
                (reader._last_config_at or 0.0) <= previous_config_at
                or reader._last_config_cmd != "config.get"
            )
            and time.monotonic() < config_deadline
        ):
            time.sleep(0.25)
        if (
            (reader._last_config_at or 0.0) <= previous_config_at
            or reader._last_config_cmd != "config.get"
        ):
            raise SystemExit(
                f"No post-connect sensor config received: {reader.last_error}"
            )
        print("Sensor config:", reader._last_config)

        def apply_and_verify(changes):
            previous_config_at = reader._last_config_at or 0.0
            controller.received.clear()
            payload = {"cmd": "config.set"}
            payload.update(changes)
            write_json(payload)
            apply_deadline = time.monotonic() + 45.0
            while time.monotonic() < apply_deadline:
                config_is_new = (
                    (reader._last_config_at or 0.0) > previous_config_at
                    and reader._last_config_cmd == "config.set"
                )
                config_matches = config_is_new and all(
                    reader._last_config.get(name) == expected
                    for name, expected in changes.items()
                )
                if config_matches:
                    break
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
