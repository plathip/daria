"""Full Betaflight configuration backup over USB (read-only, changes nothing).

Captures `version`, `diff all`, and `dump all` from the Betaflight CLI into
a timestamped folder under fc/backups/. Restore instructions:
fc/RESTORE-BETAFLIGHT.md

Usage: close Betaflight Configurator first (only one program can own the
COM port), plug the FC in via USB (props OFF), then:  python backup_betaflight.py
"""
import datetime
import os
import sys
import time

import serial
from serial.tools import list_ports


def find_fc_port(wait_s=15):
    """The Betaflight board is the STM32 virtual COM port (VID 0483, PID 5740). It can take a few
    seconds to appear after a reboot, so poll; never fall back to a port that is not a USB device
    (a laptop's own COM ports - Intel AMT, Bluetooth serial - are not the drone)."""
    t0 = time.time()
    while True:
        candidates = []
        for p in list_ports.comports():
            blob = f'{p.description} {p.manufacturer or ""} {p.vid:04x}:{p.pid:04x}' \
                if p.vid else p.description
            candidates.append((p.device, blob, p.vid, p.pid))
        stm = [c for c in candidates if (c[2], c[3]) == (0x0483, 0x5740)
               or (c[1] and ('betaflight' in c[1].lower() or 'stm32 virtual' in c[1].lower()))]
        if stm:
            if len(stm) > 1:
                print('Several STM32 boards found, using the first:')
                for d, b, _, _ in stm:
                    print(f'  {d}: {b}')
            return stm[0][0], stm[0][1]
        if time.time() - t0 > wait_s:
            break
        time.sleep(1)
    print('Serial ports found (none is a Betaflight board):')
    for d, b, _, _ in candidates:
        print(f'  {d}: {b}')
    return None, None


def cli_capture(ser, command, quiet_s=1.5, max_s=30):
    ser.reset_input_buffer()
    ser.write((command + '\n').encode())
    buf = b''
    t0 = time.time()
    last_data = time.time()
    while time.time() - t0 < max_s:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
            last_data = time.time()
        elif time.time() - last_data > quiet_s:
            break
    return buf.decode(errors='replace')


def main():
    port, desc = find_fc_port()
    if port is None:
        sys.exit('FAIL: no serial port found — is the FC plugged in via USB?')
    print(f'Using {port} ({desc})')

    ser = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(1.5)
    ser.reset_input_buffer()
    ser.write(b'#')                        # enter CLI (bare byte — a trailing
    time.sleep(1.2)                        # newline breaks entry on BF 2025.12)
    ser.write(b'\n')                       # if we were ALREADY in CLI, the '#'
    time.sleep(0.5)                        # opened a comment — this closes it
    banner = ser.read(8192).decode(errors='replace')

    version = cli_capture(ser, 'version')
    if 'Betaflight' not in (banner + version):
        ser.close()
        sys.exit(f'FAIL: this does not look like Betaflight CLI.\nGot: {version[:300]}')

    print('Betaflight CLI confirmed. Capturing (takes ~20 s)...')
    status = cli_capture(ser, 'status')
    diff_all = cli_capture(ser, 'diff all', quiet_s=2.0, max_s=60)
    dump_all = cli_capture(ser, 'dump all', quiet_s=2.0, max_s=90)
    ser.write(b'exit\n')                   # reboot FC back to normal operation
    ser.close()

    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'backups', f'betaflight-{stamp}')
    os.makedirs(outdir, exist_ok=True)
    for name, content in (('version.txt', banner + version + status),
                          ('diff_all.txt', diff_all),
                          ('dump_all.txt', dump_all)):
        with open(os.path.join(outdir, name), 'w', encoding='utf-8') as f:
            f.write(content)

    print(f'\nBackup written to {outdir}')
    print(f'  version.txt  {len(version):6d} chars')
    print(f'  diff_all.txt {len(diff_all):6d} chars')
    print(f'  dump_all.txt {len(dump_all):6d} chars')
    if len(diff_all) < 500 or len(dump_all) < 5000:
        print('WARNING: capture looks too short — do not proceed to flashing!')
    else:
        print('Backup looks complete.')
    print('\n--- serial/port lines from diff (wiring map) ---')
    for line in diff_all.splitlines():
        ls = line.strip()
        if ls.startswith(('serial ', 'feature ', 'map ')) or 'gps' in ls.lower():
            print(' ', ls)


if __name__ == '__main__':
    main()
