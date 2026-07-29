"""End-to-end probe using ddcci-screen-tuning's BLE ambient reader."""

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
    controller = ProbeController()
    reader = BleNusAmbientReader(controller)
    if not reader.start():
        raise SystemExit(f"Unable to start BLE reader: {reader.last_error}")

    try:
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline and not controller.received.wait(0.25):
            pass
        if not controller.received.is_set():
            raise SystemExit(f"No BLE measurement received: {reader.last_error}")
        reader.request_config()
        config_deadline = time.monotonic() + 45.0
        while reader._last_config is None and time.monotonic() < config_deadline:
            time.sleep(0.25)
        print("Sensor config:", reader._last_config)
    finally:
        reader.stop()


if __name__ == "__main__":
    main()
