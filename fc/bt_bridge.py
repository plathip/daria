"""Bluetooth <-> TCP bridge for the SpeedyBee F405 V5's built-in BLE.

Turns the board's Bluetooth into a local TCP MAVLink port, so any ground
station can use the wireless link:

    python fc/bt_bridge.py                 # serves tcp://127.0.0.1:5770
    # then, in another terminal / program:
    python console/fire_console.py --link tcp:127.0.0.1:5770
    # or Mission Planner: CONNECT -> TCP -> 127.0.0.1 port 5770

The BLE side is the transparent UART service (abf0): abf1 = laptop->FC,
abf2 = FC->laptop. The bridge sends one GCS heartbeat on connect because the
module only starts forwarding after the first inbound write.

Notes:
  * BLE bandwidth is far below USB, fine for telemetry, console and
    parameter work; slow for full-log downloads.
  * One radio rule still applies: this is a GCS link (sysid 255 passes
    through from whatever client connects).
"""
import argparse
import asyncio
import io
import sys
import time

from bleak import BleakClient, BleakScanner
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

NAME_HINT = 'F405'
try:   # the portal's settings decide which Bluetooth name to look for
    import json as _json
    import os as _os
    with open(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'console', 'settings.json'),
              encoding='utf-8') as _f:
        NAME_HINT = _json.load(_f).get('ble_name') or NAME_HINT
except Exception:
    pass
STATUS_PATH = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'console', 'bridge_status.json')
_status = {}


def name_hint():
    """The Bluetooth name fragment from the portal settings, re-read before every scan."""
    try:
        with open(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'console', 'settings.json'),
                  encoding='utf-8') as f:
            return _json.load(f).get('ble_name') or NAME_HINT
    except Exception:
        return NAME_HINT


def addr_hint():
    """Optional Bluetooth address chosen in Setup (for modules that advertise no name)."""
    try:
        with open(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'console', 'settings.json'),
                  encoding='utf-8') as f:
            return (_json.load(f).get('ble_addr') or '').strip().lower()
    except Exception:
        return ''


# What a SpeedyBee Bluetooth module advertises (seen on the F405 V5): service 0x00FF and manufacturer
# id 28717. Used to recognise a drone whose name nobody has typed in yet.
SPEEDYBEE_SERVICE = '000000ff-0000-1000-8000-00805f9b34fb'
SPEEDYBEE_MFR = 28717


def looks_like_drone(adv):
    try:
        return (SPEEDYBEE_SERVICE in (adv.service_uuids or [])
                and SPEEDYBEE_MFR in (adv.manufacturer_data or {}))
    except Exception:
        return False


def write_status(state, **kw):
    """What the bridge is doing, for the Setup page (console/bridge_status.json)."""
    _status.update(state=state, ts=time.time(), hint=name_hint(), **kw)
    try:
        tmp = STATUS_PATH + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            _json.dump(_status, f)
        _os.replace(tmp, STATUS_PATH)
    except Exception:
        pass


CH_WRITE = '0000abf1-0000-1000-8000-00805f9b34fb'
CH_NOTIFY = '0000abf2-0000-1000-8000-00805f9b34fb'


def gcs_heartbeat_bytes():
    buf = io.BytesIO()
    mav = mavlink2.MAVLink(buf, srcSystem=255, srcComponent=190)
    msg = mav.heartbeat_encode(mavlink2.MAV_TYPE_GCS,
                               mavlink2.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    return msg.pack(mav)


class Bridge:
    def __init__(self):
        self.clients = set()          # TCP writers
        self.ble = None               # BleakClient when connected
        self.up_bytes = 0             # FC -> laptop
        self.down_bytes = 0           # laptop -> FC
        self.to_ble = asyncio.Queue(maxsize=512)
        self.fc_bytes = 0             # bytes heard from the board since this BLE connection

    @staticmethod
    def betaflight_on_usb():
        """A Betaflight board plugged into this laptop (STM32 virtual COM port). While it is there the
        laptop is converting it: MAVLink pushed through the SpeedyBee's Bluetooth into Betaflight's MSP
        port sends the board into DFU mode within a minute (seen 2026-09-13), so the bridge stands by."""
        try:
            from serial.tools import list_ports
            return next((p.device for p in list_ports.comports()
                         if p.vid == 0x0483 and p.pid == 0x5740), None)
        except Exception:
            return None

    # ---- BLE side ----
    async def ble_task(self):
        while True:
            try:
                bf = self.betaflight_on_usb()
                if bf:
                    print(f'[ble] a Betaflight board is on USB ({bf}): standing by until it runs ArduPilot')
                    write_status('standby', standby_usb=bf)
                    await asyncio.sleep(10)
                    continue
                hint, addr = name_hint(), addr_hint()
                print(f'[ble] scanning for a board named *{hint}*' + (f' or at {addr}' if addr else '')
                      + ', else any SpeedyBee-type module ...')
                write_status('scanning', addr_hint=addr)
                by = 'name'
                dev = await BleakScanner.find_device_by_filter(
                    lambda d, ad: (bool(hint) and bool(d.name) and hint in d.name)
                    or (bool(addr) and bool(d.address) and d.address.lower() == addr), timeout=15)
                if dev is None:
                    # nothing by name/address: any board that advertises like a SpeedyBee module will do
                    dev = await BleakScanner.find_device_by_filter(lambda d, ad: looks_like_drone(ad), timeout=10)
                    by = 'fingerprint'
                if dev is None:
                    print('[ble] not found (drone powered? name right?). Retrying in 5 s')
                    write_status('not_found')
                    await asyncio.sleep(5)
                    continue
                if by == 'fingerprint':
                    print(f'[ble] no board named *{hint}*, but {dev.name!r} [{dev.address}] advertises like a '
                          'SpeedyBee module. Using it (Setup, Bluetooth, "this is my drone" locks it in)')
                print(f'[ble] connecting to {dev.name} [{dev.address}] ...')
                write_status('connecting', device=dev.name, address=dev.address, by=by)
                disconnected = asyncio.Event()
                async with BleakClient(
                        dev, disconnected_callback=lambda c: disconnected.set()) as client:
                    self.ble = client
                    chunk = max(20, (client.mtu_size or 23) - 3)
                    # drop anything queued while BLE was down, stale MAVLink
                    # must never be flushed at the aircraft on reconnect
                    while not self.to_ble.empty():
                        try:
                            self.to_ble.get_nowait()
                        except Exception:
                            break
                    self.fc_bytes = 0
                    t_up = time.time()
                    await client.start_notify(CH_NOTIFY, self.on_ble_data)
                    # one wake-up heartbeat: the module only starts forwarding after an inbound write
                    await client.write_gatt_char(CH_WRITE, gcs_heartbeat_bytes(),
                                                 response=False)
                    print(f'[ble] LINK UP (chunk {chunk} B). '
                          f'Ground stations: tcp:127.0.0.1:<port>')
                    write_status('connected', device=dev.name, address=dev.address, by=by)
                    silent = False
                    while not disconnected.is_set():
                        try:
                            data = await asyncio.wait_for(self.to_ble.get(), timeout=1.0)
                        except asyncio.TimeoutError:
                            data = None
                        if self.fc_bytes == 0:
                            # nothing heard from the board: it is not talking MAVLink (still
                            # Betaflight?) - forward nothing, and leave it alone after 8 s
                            if time.time() - t_up > 8:
                                print('[ble] the board sends no telemetry (still Betaflight?) - '
                                      'leaving it alone for 2 minutes')
                                write_status('silent', device=dev.name)
                                silent = True
                                break
                            continue
                        if data is None:
                            continue
                        for i in range(0, len(data), chunk):
                            await client.write_gatt_char(CH_WRITE,
                                                         data[i:i + chunk],
                                                         response=False)
                            self.down_bytes += len(data[i:i + chunk])
            except Exception as e:
                print(f'[ble] link lost: {e}')
                write_status('lost', error=str(e)[:120])
                silent = False
            finally:
                self.ble = None
            if silent:
                await asyncio.sleep(120)
                continue
            print('[ble] reconnecting in 3 s ...')
            await asyncio.sleep(3)

    def on_ble_data(self, _, data):
        self.up_bytes += len(data)
        self.fc_bytes += len(data)
        for w in list(self.clients):
            try:
                w.write(bytes(data))
            except Exception:
                self.clients.discard(w)

    # ---- TCP side ----
    async def handle_client(self, reader, writer):
        peer = writer.get_extra_info('peername')
        print(f'[tcp] ground station connected: {peer}')
        self.clients.add(writer)
        try:
            while True:
                data = await reader.read(1024)
                if not data:
                    break
                try:
                    self.to_ble.put_nowait(data)
                except asyncio.QueueFull:
                    pass          # BLE can't keep up, drop rather than stall
        except Exception:
            pass
        finally:
            self.clients.discard(writer)
            try:
                writer.close()
            except Exception:
                pass
            print(f'[tcp] ground station left: {peer}')

    async def stats_task(self):
        while True:
            await asyncio.sleep(10)
            if _status.get('state'):
                write_status(_status['state'])          # keeps the timestamp fresh: "bridge alive"
            state = 'UP' if self.ble else 'DOWN'
            print(f'[stat] ble={state} clients={len(self.clients)} '
                  f'fc->gcs {self.up_bytes} B, gcs->fc {self.down_bytes} B')


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tcp-port', type=int, default=5770)
    args = ap.parse_args()

    br = Bridge()
    server = await asyncio.start_server(br.handle_client, '127.0.0.1', args.tcp_port)
    print(f'[tcp] serving MAVLink on tcp:127.0.0.1:{args.tcp_port}')
    async with server:
        await asyncio.gather(br.ble_task(), br.stats_task(), server.serve_forever())


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print('\nbridge stopped')
        sys.exit(0)
