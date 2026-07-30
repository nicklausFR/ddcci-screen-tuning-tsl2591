"""Validate that Windows keeps a BLE link while the peripheral sleeps in WFI."""

import argparse
import asyncio
import time

from bleak import BleakClient, BleakScanner


async def run(name: str, duration: float, scan_timeout: float) -> None:
    device = await BleakScanner.find_device_by_name(name, timeout=scan_timeout)
    if device is None:
        raise RuntimeError(f"{name!r} introuvable")

    disconnected = asyncio.Event()
    started = time.monotonic()

    def on_disconnect(_: BleakClient) -> None:
        disconnected.set()

    async with BleakClient(
        device,
        disconnected_callback=on_disconnect,
        timeout=30.0,
        winrt={"address_type": "random", "use_cached_services": False},
    ) as client:
        if not client.is_connected:
            raise RuntimeError("connexion BLE non établie")

        print(f"CONNECTED address={device.address} duration={duration:g}s", flush=True)
        try:
            await asyncio.wait_for(disconnected.wait(), timeout=duration)
        except TimeoutError:
            pass

        elapsed = time.monotonic() - started
        if disconnected.is_set() or not client.is_connected:
            raise RuntimeError(f"liaison perdue après {elapsed:.1f}s")
        print(f"PASS connected_for={elapsed:.1f}s", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", default="LuxWfi")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--scan-timeout", type=float, default=20.0)
    args = parser.parse_args()
    asyncio.run(run(args.name, args.duration, args.scan_timeout))


if __name__ == "__main__":
    main()
