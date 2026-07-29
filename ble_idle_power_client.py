"""Connect to the LuxIdle BLE peripheral and keep the link idle."""

import argparse
import asyncio
from datetime import datetime

from bleak import BleakClient, BleakScanner


async def connect_once(
    name: str, scan_timeout: float, disconnected: asyncio.Event
):
    loop = asyncio.get_running_loop()
    device_found = loop.create_future()

    def on_advertisement(device, advertisement_data) -> None:
        advertised_name = advertisement_data.local_name or device.name
        if advertised_name == name and not device_found.done():
            device_found.set_result(device)

    def on_disconnect(_: BleakClient) -> None:
        disconnected.set()

    scanner = BleakScanner(on_advertisement)
    client = None
    await scanner.start()

    try:
        try:
            device = await asyncio.wait_for(device_found, timeout=scan_timeout)
        except TimeoutError as error:
            raise RuntimeError(
                f"{name!r} introuvable. Vérifie que le nRF est alimenté "
                "et qu'aucun autre central n'y est connecté."
            ) from error

        client = BleakClient(
            device,
            disconnected_callback=on_disconnect,
            timeout=30.0,
            winrt={"address_type": "random"},
        )
        await client.connect()
        return client, device
    except BaseException:
        if client is not None and client.is_connected:
            await client.disconnect()
        raise
    finally:
        await scanner.stop()


async def run(name: str, scan_timeout: float, retries: int) -> None:
    disconnected = asyncio.Event()
    last_error = None

    for attempt in range(1, retries + 1):
        print(f"Recherche de {name!r} (tentative {attempt}/{retries})...")
        disconnected.clear()
        try:
            client, device = await connect_once(
                name, scan_timeout, disconnected
            )
            break
        except (TimeoutError, OSError, RuntimeError) as error:
            last_error = error
            if attempt < retries:
                print(f"Connexion non aboutie: {error}. Nouvelle tentative...")
                await asyncio.sleep(3)
    else:
        raise RuntimeError(
            f"Connexion à {name!r} impossible après {retries} tentatives: "
            f"{last_error}"
        ) from last_error

    try:
        if not client.is_connected:
            raise RuntimeError("La connexion BLE n'a pas pu être établie.")

        connected_at = datetime.now()
        print(
            f"Connecté à {device.name or device.address} depuis "
            f"{connected_at:%Y-%m-%d %H:%M:%S}."
        )
        print("Liaison maintenue sans échange applicatif. Ctrl+C pour arrêter.")
        await disconnected.wait()
    finally:
        if client.is_connected:
            await client.disconnect()

    elapsed = datetime.now() - connected_at
    print(f"Déconnecté après {elapsed}.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Maintient une connexion BLE inactive avec le nRF."
    )
    parser.add_argument("--name", default="LuxIdle", help="nom BLE à rechercher")
    parser.add_argument(
        "--scan-timeout",
        type=float,
        default=20.0,
        help="durée maximale de recherche en secondes",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=3,
        help="nombre de tentatives de connexion",
    )
    args = parser.parse_args()

    try:
        asyncio.run(run(args.name, args.scan_timeout, max(1, args.retries)))
    except KeyboardInterrupt:
        print("\nArrêt demandé.")
    except RuntimeError as error:
        print(f"Erreur: {error}")
        raise SystemExit(1) from None


if __name__ == "__main__":
    main()
