"""Receive intermittent ambient-light measurements over BLE advertising.

Install the only dependency with::

    py -m pip install bleak

Then run::

    py ble_adv_receiver.py
"""

from __future__ import annotations

import argparse
import asyncio
import struct
from datetime import datetime

from bleak import BleakScanner


# 0xFFFF is reserved for development/testing. A production device should use
# an assigned Bluetooth SIG company identifier or a service-data UUID instead.
COMPANY_ID = 0xFFFF
MAGIC = b"LT"
PROTOCOL_VERSION = 1
PAYLOAD = struct.Struct("<2sBHIHBB")
GAIN_NAMES = ("low", "med", "high", "max")


def decode_measurement(data: bytes) -> dict[str, object]:
    """Decode the bytes following the two-byte manufacturer company ID."""
    if len(data) != PAYLOAD.size:
        raise ValueError(f"unexpected payload size: {len(data)} (expected {PAYLOAD.size})")

    magic, version, sequence, lux_milli, visible, quality, gain_id = PAYLOAD.unpack(data)
    if magic != MAGIC:
        raise ValueError("not an ambient-light payload")
    if version != PROTOCOL_VERSION:
        raise ValueError(f"unsupported protocol version: {version}")

    return {
        "sequence": sequence,
        "lux": None if lux_milli == 0xFFFFFFFF else lux_milli / 1000.0,
        "visible": visible,
        "quality": quality,
        "saturated": bool(quality & 0x01),
        "gain": GAIN_NAMES[gain_id] if gain_id < len(GAIN_NAMES) else gain_id,
    }


async def scan(name: str | None, duration: float | None = None) -> None:
    last_sequence_by_address: dict[str, int] = {}

    def on_advertisement(device, advertisement_data) -> None:
        if name and advertisement_data.local_name != name:
            return
        data = advertisement_data.manufacturer_data.get(COMPANY_ID)
        if data is None:
            return
        try:
            reading = decode_measurement(data)
        except ValueError as exc:
            print(f"[WARN] {device.address}: {exc}")
            return

        sequence = int(reading["sequence"])
        if last_sequence_by_address.get(device.address) == sequence:
            return
        last_sequence_by_address[device.address] = sequence
        timestamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
        print(
            f"{timestamp} {device.address} RSSI={advertisement_data.rssi:>4} "
            f"seq={sequence:>5} lux={reading['lux']} visible={reading['visible']} "
            f"quality=0x{int(reading['quality']):02x} gain={reading['gain']}",
            flush=True,
        )

    scanner = BleakScanner(on_advertisement)
    print(f"Listening for BLE advertisements (company ID 0x{COMPANY_ID:04X})...", flush=True)
    async with scanner:
        if duration is None:
            await asyncio.Event().wait()
        else:
            await asyncio.sleep(duration)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", help="also require this advertised local name")
    parser.add_argument("--duration", type=float, help="stop after this many seconds")
    args = parser.parse_args()
    try:
        asyncio.run(scan(args.name, args.duration))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
