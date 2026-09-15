"""Identify a Betaflight (or iNav/other MSP) flight controller over USB.

Speaks just enough MSP v1 to ask: which firmware, which version, which board
target. Used by the portal's Detect button so a Betaflight quad is recognised
BEFORE anything is flashed, and mapped to its ArduPilot board id.

    python fc/bf_msp.py            # auto: first STM32 virtual COM port
    python fc/bf_msp.py COM5
"""
import struct
import sys
import time

import serial
from serial.tools import list_ports

MSP_API_VERSION, MSP_FC_VARIANT, MSP_FC_VERSION, MSP_BOARD_INFO = 1, 2, 3, 4
VARIANTS = {'BTFL': 'Betaflight', 'INAV': 'iNav', 'CLFL': 'Cleanflight', 'EMUF': 'EmuFlight',
            'ARDU': 'ArduPilot (MSP)'}


def _request(ser, cmd, timeout=1.0):
    ser.reset_input_buffer()
    ser.write(b'$M<' + bytes([0, cmd, cmd]))      # size 0, checksum = size ^ cmd
    t0 = time.time()
    buf = b''
    while time.time() - t0 < timeout:
        buf += ser.read(ser.in_waiting or 1)
        i = buf.find(b'$M>')
        if i >= 0 and len(buf) >= i + 5:
            size, rcmd = buf[i + 3], buf[i + 4]
            if len(buf) >= i + 5 + size + 1:
                payload = buf[i + 5:i + 5 + size]
                if rcmd == cmd:
                    return payload
                buf = buf[i + 5 + size + 1:]
    return None


def _pstr(payload, pos):
    n = payload[pos]
    return payload[pos + 1:pos + 1 + n].decode('ascii', 'replace'), pos + 1 + n


def probe(port, timeout=2.5):
    """Returns dict(variant, version, target, board, api) or None if not MSP."""
    try:
        with serial.Serial(port, 115200, timeout=0.2) as ser:
            time.sleep(0.3)
            api = _request(ser, MSP_API_VERSION, timeout)
            if api is None:
                return None
            info = {'api': '%d.%d' % (api[1], api[2]) if len(api) >= 3 else '?'}
            var = _request(ser, MSP_FC_VARIANT, timeout) or b''
            code = var[:4].decode('ascii', 'replace')
            info['variant'] = VARIANTS.get(code, code)
            ver = _request(ser, MSP_FC_VERSION, timeout) or b''
            info['version'] = '%d.%d.%d' % tuple(ver[:3]) if len(ver) >= 3 else '?'
            bi = _request(ser, MSP_BOARD_INFO, timeout) or b''
            info['board'] = bi[:4].decode('ascii', 'replace') if len(bi) >= 4 else '?'
            info['target'] = None
            if len(bi) > 9:                     # identifier(4) hwrev(2) fctype(1) caps(1) then names
                try:
                    target, p = _pstr(bi, 8)
                    board_name, p = _pstr(bi, p)
                    info['target'] = board_name or target
                    info['target_name'] = target
                except Exception:
                    pass
            return info
    except Exception:
        return None


def find_port():
    for p in list_ports.comports():
        if p.vid == 0x0483:
            return p.device
    return None


if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        raise SystemExit('no STM32 virtual COM port found')
    r = probe(port)
    print(r or 'not an MSP flight controller on %s' % port)
