"""Betaflight -> ArduPilot settings translator (the 'wrapper' between the two).

Reads a Betaflight `diff all` (as saved by fc/backup_betaflight.py) and produces
the ArduPilot parameters that express the same hardware facts:

  serial <UART> <functions>   -> SERIALn_PROTOCOL / SERIALn_BAUD   (which UART has
                                  the receiver, GPS, VTX control, ESC telemetry ...)
  motor_pwm_protocol          -> MOT_PWM_TYPE
  yaw_motors_reversed         -> FRAME_TYPE 12 (props-in, BetaFlightX) or 18 (props-out)
  map AETR1234                -> RCMAP_ROLL/PITCH/THROTTLE/YAW
  serialrx_provider SBUS      -> SERIALn_OPTIONS (inverted input)
  align_board_roll/pitch/yaw  -> AHRS_ORIENTATION (how the FC sits on the frame)
  vbat_scale / ibata_scale    -> BATT_VOLT_MULT / BATT_AMP_PERVLT (+ BATT_AMP_OFFSET)
  motor_poles                 -> SERVO_BLH_POLES
  vtx_freq / vtx_band+channel -> VTX_FREQ / VTX_BAND + VTX_CHANNEL
  board_name                  -> the ArduPilot board id to flash
  align_mag                   -> a NOTE only: compass drivers differ, the portal's
                                 heading check + COMPASS_AUTO_ROT settle it

The UART translation needs the board's SERIAL_ORDER line from ArduPilot's hwdef
(fetched from GitHub once and cached in fc/firmware/hwdef/).

    python fc/bf_migrate.py fc/backups/betaflight-XXXX/diff_all.txt [ap_board]
"""
import json
import os
import re
import sys
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
HWDEF_CACHE = os.path.join(HERE, 'firmware', 'hwdef')
HWDEF_URL = ('https://raw.githubusercontent.com/ArduPilot/ardupilot/master/'
             'libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef.dat')

# Betaflight serial function bits -> ArduPilot SERIALn_PROTOCOL
BF_FUNC = {
    1: ('MSP', 32),
    2: ('GPS', 5),
    64: ('RC receiver (serial RX)', 23),
    512: ('MAVLink telemetry', 2),
    1024: ('ESC telemetry', 16),
    2048: ('VTX SmartAudio', 37),
    4096: ('iBUS telemetry', 49),
    8192: ('VTX Tramp', 44),
}
BF_PROTO = {  # motor_pwm_protocol -> MOT_PWM_TYPE
    'DSHOT150': 4, 'DSHOT300': 5, 'DSHOT600': 6, 'DSHOT1200': 7,
    'ONESHOT125': 2, 'ONESHOT42': 1, 'MULTISHOT': None, 'BRUSHED': 3,
    'PWM': 0, 'STANDARD': 0, 'PROSHOT1000': None, 'DISABLED': None,
}
# Betaflight board_name -> ArduPilot board id (confident pairs only; anything else
# is looked up by fuzzy match against the official list, then verified)
BF_TO_AP = {
    'SPEEDYBEEF405V3': 'speedybeef4v3', 'SPEEDYBEEF405V4': 'speedybeef4v4',
    'SPEEDYBEEF405V5': 'speedybeef4v5', 'SPEEDYBEEF405MINI': 'SpeedyBeeF405Mini',
    'SPEEDYBEEF405WING': 'SpeedyBeeF405WING',
    'MATEKF405': 'MatekF405', 'MATEKF405TE': 'MatekF405-TE', 'MATEKF405SE': 'MatekF405-Wing',
    'MATEKF765': 'MatekF765-Wing', 'MATEKH743': 'MatekH743',
    'KAKUTEF4': 'KakuteF4', 'KAKUTEF4V2': 'KakuteF4', 'KAKUTEF7': 'KakuteF7',
    'KAKUTEF7MINI': 'KakuteF7Mini', 'KAKUTEH7': 'KakuteH7', 'KAKUTEH7MINI': 'KakuteH7Mini',
    'KAKUTEH7V2': 'KakuteH7v2',
    'MAMBAF405_2022A': 'MambaF405-2022', 'MAMBAF405US_I2C': 'MambaF405v2',
    'MAMBAH743_2022B': 'MambaH743v4',
    'FLYWOOF745': 'FlywooF745', 'FLYWOOF745NANO': 'FlywooF745Nano',
    'BEASTF7': 'BeastF7', 'BEASTH7': 'BeastH7',
    'OMNIBUSF4': 'omnibusf4', 'OMNIBUSF4SD': 'omnibusf4pro',
}


def norm(s):
    return re.sub(r'[^a-z0-9]', '', s.lower())


# ArduPilot Rotation enum for the mountings a quad FC can realistically have:
# (roll, pitch, yaw) in degrees -> AHRS_ORIENTATION value
AP_ROTATIONS = {
    (0, 0, 0): 0, (0, 0, 45): 1, (0, 0, 90): 2, (0, 0, 135): 3, (0, 0, 180): 4,
    (0, 0, 225): 5, (0, 0, 270): 6, (0, 0, 315): 7,
    (180, 0, 0): 8, (180, 0, 45): 9, (180, 0, 90): 10, (180, 0, 135): 11,
    (0, 180, 0): 12, (180, 0, 225): 13, (180, 0, 270): 14, (180, 0, 315): 15,
    (90, 0, 0): 16, (90, 0, 45): 17, (90, 0, 90): 18, (90, 0, 135): 19,
    (270, 0, 0): 20, (270, 0, 45): 21, (270, 0, 90): 22, (270, 0, 135): 23,
    (0, 90, 0): 24, (0, 270, 0): 25, (0, 180, 90): 26, (0, 180, 270): 27,
}
AP_ROTATION_NAMES = {0: 'none', 1: 'yaw 45', 2: 'yaw 90', 3: 'yaw 135', 4: 'yaw 180', 5: 'yaw 225',
                     6: 'yaw 270', 7: 'yaw 315', 8: 'roll 180', 9: 'roll 180 yaw 45', 10: 'roll 180 yaw 90',
                     11: 'roll 180 yaw 135', 12: 'pitch 180', 13: 'roll 180 yaw 225', 14: 'roll 180 yaw 270',
                     15: 'roll 180 yaw 315', 16: 'roll 90', 17: 'roll 90 yaw 45', 18: 'roll 90 yaw 90',
                     19: 'roll 90 yaw 135', 20: 'roll 270', 21: 'roll 270 yaw 45', 22: 'roll 270 yaw 90',
                     23: 'roll 270 yaw 135', 24: 'pitch 90', 25: 'pitch 270', 26: 'pitch 180 yaw 90',
                     27: 'pitch 180 yaw 270'}


def bf_angle(v):
    # Betaflight stores align_board_* in decidegrees (900 = 90 deg); old builds used degrees.
    try:
        a = float(v)
    except (TypeError, ValueError):
        return 0
    if abs(a) > 360:
        a /= 10.0
    a = round(a / 45.0) * 45
    return int(a % 360)


def board_orientation(setd):
    # (AHRS_ORIENTATION or None, description) from align_board_roll/pitch/yaw
    r, p, y = (bf_angle(setd.get('align_board_roll', 0)), bf_angle(setd.get('align_board_pitch', 0)),
               bf_angle(setd.get('align_board_yaw', 0)))
    if (r, p, y) == (0, 0, 0):
        return 0, 'not rotated'
    key = (r, p, y)
    if key not in AP_ROTATIONS and p == 180 and r == 180:
        key = (0, 0, (y + 180) % 360)          # roll180+pitch180 == yaw180
    if key in AP_ROTATIONS:
        return AP_ROTATIONS[key], 'roll %d pitch %d yaw %d' % (r, p, y)
    return None, 'roll %d pitch %d yaw %d (no ArduPilot rotation matches - set AHRS_ORIENTATION by hand)' % (r, p, y)


def ap_board_for(bf_name, ap_boards):
    """(ap_board_id or None, how) using the table, then a fuzzy match on the official list."""
    if not bf_name:
        return None, 'no board_name in the Betaflight dump'
    cand = BF_TO_AP.get(bf_name.upper())
    if cand and (not ap_boards or cand in ap_boards):
        return cand, 'known pairing'
    if ap_boards:
        n = norm(bf_name)
        for b in ap_boards:
            if norm(b) == n:
                return b, 'name match'
        # e.g. SPEEDYBEEF405V5 vs speedybeef4v5: drop the '05'
        for b in ap_boards:
            if norm(b) == n.replace('405', '4') or norm(b).replace('405', '4') == n:
                return b, 'name match'
    return None, 'no ArduPilot board with this name'


def hwdef_serial_order(board):
    """SERIAL_ORDER tokens for an ArduPilot board (cached). None if unknown/offline."""
    os.makedirs(HWDEF_CACHE, exist_ok=True)
    fp = os.path.join(HWDEF_CACHE, board + '.dat')
    text = None
    if os.path.exists(fp):
        text = open(fp, encoding='utf-8', errors='replace').read()
    else:
        try:
            req = urllib.request.Request(HWDEF_URL % board, headers={'User-Agent': 'FireFightingDronePortal/1.0'})
            text = urllib.request.urlopen(req, timeout=15).read().decode('utf-8', 'replace')
            open(fp, 'w', encoding='utf-8').write(text)
        except Exception:
            return None
    for depth in range(3):                       # SERIAL_ORDER may live in an included file
        m = re.search(r'^SERIAL_ORDER\s+(.+)$', text, re.M)
        if m:
            return m.group(1).split()
        inc = re.search(r'^include\s+\.\./([A-Za-z0-9_\-]+)/hwdef\.dat', text, re.M)
        if not inc:
            return None
        sub = hwdef_serial_order(inc.group(1))
        return sub
    return None


def parse_diff(text):
    d = {'serial': {}, 'set': {}, 'features': [], 'board_name': None,
         'manufacturer_id': None, 'map': None, 'resources': {}, 'version': None}
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith('# Betaflight'):
            d['version'] = line[2:]
        elif line.startswith('board_name '):
            d['board_name'] = line.split()[1]
        elif line.startswith('manufacturer_id '):
            d['manufacturer_id'] = line.split()[1]
        elif line.startswith('feature '):
            d['features'].append(line.split()[1])
        elif line.startswith('serial '):
            parts = line.split()
            if len(parts) >= 3:
                ident = parts[1].upper()
                m = re.match(r'U?S?ART(\d+)', ident)
                if m:
                    uart = int(m.group(1))
                elif ident.isdigit() and int(ident) < 20:
                    uart = int(ident) + 1
                else:
                    continue
                d['serial'][uart] = {'mask': int(parts[2]),
                                     'gps_baud': int(parts[4]) if len(parts) > 4 else None}
        elif line.startswith('set '):
            m = re.match(r'set\s+(\w+)\s*=\s*(.+)', line)
            if m:
                d['set'][m.group(1)] = m.group(2).strip()
        elif line.startswith('map '):
            d['map'] = line.split()[1].upper()
        elif line.startswith('resource MOTOR '):
            parts = line.split()
            d['resources'][int(parts[2])] = parts[3]
    return d


def translate(diff_text, ap_board=None, ap_boards=None):
    """Returns {'board': {...}, 'params': [{'name','value','why'}], 'notes': [...]}"""
    d = parse_diff(diff_text)
    out, notes = [], []
    guess, how = ap_board_for(d['board_name'], ap_boards)
    board = ap_board or guess
    info = {'betaflight_board': d['board_name'], 'manufacturer': d['manufacturer_id'],
            'ardupilot_board': guess, 'how': how, 'betaflight_version': d['version']}

    # frame
    out.append({'name': 'FRAME_CLASS', 'value': 1, 'why': 'quadcopter'})
    props_out = d['set'].get('yaw_motors_reversed', 'OFF').upper() == 'ON'
    out.append({'name': 'FRAME_TYPE', 'value': 18 if props_out else 12,
                'why': ('Betaflight motor order, props-OUT (yaw_motors_reversed = ON)' if props_out
                        else 'Betaflight motor order, props-IN (BetaFlightX): no motor remapping needed')})

    # ESC protocol
    proto = d['set'].get('motor_pwm_protocol', 'DSHOT600').upper()
    ap_proto = BF_PROTO.get(proto, 6)
    if ap_proto is None:
        notes.append('Betaflight used %s, which ArduPilot does not offer - DShot300 is set instead; '
                     'your ESCs must support DShot (all BLHeli_32 / BLHeli_S / AM32 / Bluejay do).' % proto)
        ap_proto = 5
    out.append({'name': 'MOT_PWM_TYPE', 'value': ap_proto, 'why': 'ESC protocol was %s in Betaflight' % proto})
    if d['set'].get('dshot_bidir', 'OFF').upper() == 'ON':
        notes.append('Bidirectional DShot was on. ArduPilot needs the "-bdshot" firmware variant of your '
                     'board for that (optional; only matters for the RPM harmonic notch filter).')

    # receiver / GPS / VTX / ESC telemetry UARTs
    order = hwdef_serial_order(board) if board else None
    if order is None:
        notes.append('Could not read the SERIAL_ORDER of ArduPilot board "%s" (offline, or unknown board): '
                     'the UART assignments below could not be translated - set SERIALx_PROTOCOL by hand.' % board)
    else:
        def ap_index(uart_n):
            for i, tok in enumerate(order):
                if re.fullmatch(r'U?S?ART%d' % uart_n, tok.upper()):
                    return i
            return None
        rx_provider = d['set'].get('serialrx_provider', 'CRSF').upper()
        for uart_n, cfg in sorted(d['serial'].items()):
            idx = ap_index(uart_n)
            funcs = [(bit, BF_FUNC[bit]) for bit in BF_FUNC if cfg['mask'] & bit]
            if not funcs:
                continue
            if idx is None:
                notes.append('Betaflight UART%d carried %s but ArduPilot board %s has no serial port for it.'
                             % (uart_n, ', '.join(f[1][0] for f in funcs), board))
                continue
            for bit, (label, proto_ap) in funcs:
                out.append({'name': 'SERIAL%d_PROTOCOL' % idx, 'value': proto_ap,
                            'why': '%s was on Betaflight UART%d = ArduPilot SERIAL%d' % (label, uart_n, idx)})
                if bit == 2 and cfg['gps_baud']:
                    out.append({'name': 'SERIAL%d_BAUD' % idx, 'value': cfg['gps_baud'] // 1000,
                                'why': 'GPS baud from Betaflight (%d)' % cfg['gps_baud']})
                if bit == 64 and rx_provider == 'SBUS':
                    out.append({'name': 'SERIAL%d_OPTIONS' % idx, 'value': 3,
                                'why': 'SBUS receiver: inverted signal (Betaflight used its inverter)'})
        if not any(cfg['mask'] & 64 for cfg in d['serial'].values()):
            notes.append('No serial receiver found in the Betaflight dump (SPI receiver?). ArduPilot needs a '
                         'UART receiver (ELRS/CRSF, SBUS): wire it to a spare UART and set that SERIALx_PROTOCOL=23.')

    # stick channel order
    if d['map'] and d['map'][:4] != 'AETR':
        pos = {c: i + 1 for i, c in enumerate(d['map'][:4])}
        for ch, letter in (('ROLL', 'A'), ('PITCH', 'E'), ('THROTTLE', 'T'), ('YAW', 'R')):
            out.append({'name': 'RCMAP_' + ch, 'value': pos[letter],
                        'why': 'Betaflight channel map was %s' % d['map']})

    # how the flight controller sits on the frame (Betaflight board alignment)
    orient, desc = board_orientation(d['set'])
    if orient is None:
        notes.append('Betaflight board alignment was %s. Set AHRS_ORIENTATION by hand, then run the '
                     'orientation check in Setup.' % desc)
    elif orient != 0:
        out.append({'name': 'AHRS_ORIENTATION', 'value': orient,
                    'why': 'FC mounted rotated in Betaflight (%s) = ArduPilot rotation "%s". Confirm with '
                           'the orientation check in Setup.' % (desc, AP_ROTATION_NAMES.get(orient, orient))})

    # battery monitor scaling: same physics, different units
    if 'vbat_scale' in d['set']:
        try:
            vs = float(d['set']['vbat_scale'])
            out.append({'name': 'BATT_VOLT_MULT', 'value': round(vs / 10.0, 3),
                        'why': 'Betaflight vbat_scale %g = divider %.1f:1' % (vs, vs / 10.0)})
        except ValueError:
            pass
    if 'ibata_scale' in d['set']:
        try:
            isc = float(d['set']['ibata_scale'])
            if isc > 0:
                apv = round(10000.0 / isc, 2)     # ibata_scale is 0.1 mV per A
                out.append({'name': 'BATT_AMP_PERVLT', 'value': apv,
                            'why': 'Betaflight ibata_scale %g = %.1f mV/A = %.1f A per volt' % (isc, isc / 10.0, apv)})
                off = float(d['set'].get('ibata_offset', 0) or 0)
                if off:
                    out.append({'name': 'BATT_AMP_OFFSET', 'value': round(off / 1000.0 / apv, 4),
                                'why': 'Betaflight ibata_offset %g mA' % off})
        except ValueError:
            pass
    if 'motor_poles' in d['set']:
        try:
            out.append({'name': 'SERVO_BLH_POLES', 'value': int(d['set']['motor_poles']),
                        'why': 'motor pole count from Betaflight (ESC RPM telemetry)'})
        except ValueError:
            pass

    # video transmitter, when Betaflight controlled it over SmartAudio / Tramp
    if any(cfg['mask'] & (2048 | 8192) for cfg in d['serial'].values()):
        try:
            freq = int(d['set'].get('vtx_freq', 0) or 0)
            if freq:
                out.append({'name': 'VTX_FREQ', 'value': freq, 'why': 'VTX frequency from Betaflight (MHz)'})
            else:
                band, chan = int(d['set'].get('vtx_band', 0) or 0), int(d['set'].get('vtx_channel', 0) or 0)
                if band and chan:
                    out.append({'name': 'VTX_BAND', 'value': band - 1,
                                'why': 'Betaflight band %d (ArduPilot counts from 0: A=0 B=1 E=2 F=3 R=4)' % band})
                    out.append({'name': 'VTX_CHANNEL', 'value': chan, 'why': 'VTX channel from Betaflight'})
            notes.append('VTX power is a table index in Betaflight and milliwatts in ArduPilot (VTX_POWER): '
                         'set it by hand, e.g. 25 or 200.')
        except ValueError:
            pass

    # things that do NOT translate 1:1
    mag = d['set'].get('align_mag') or d['set'].get('mag_align_yaw')
    if mag and str(mag).upper() not in ('DEFAULT', '0', 'CW0'):
        notes.append('Betaflight had the compass rotated (align_mag %s). ArduPilot compass drivers already '
                     'apply their own default rotation, so this is NOT copied: calibrate the compass with '
                     'COMPASS_AUTO_ROT = 2 (the portal does this) and run the heading check in Setup.' % mag)
    if 'OSD' in d['features'] or any(k.startswith('osd_') for k in d['set']):
        notes.append('Betaflight had an OSD. ArduPilot has its own (OSD_TYPE = 1 on boards with a MAX7456 chip); '
                     'the layout is not copied - optional, for FPV flying only.')
    if d['set'].get('failsafe_procedure'):
        notes.append('Betaflight failsafe was %s. ArduPilot does RTL on radio loss (FS_THR_ENABLE = 1) - '
                     'run the radio failsafe test in Setup; the receiver itself must be set to "no pulses".'
                     % d['set']['failsafe_procedure'])
    if any(v != 'NONE' for v in d['resources'].values()):
        notes.append('Betaflight had motor outputs re-assigned (resource MOTOR). ArduPilot uses the pads in '
                     'their native M1-M4 order: run the motor test and use "Apply mapping" if a corner is wrong.')
    if 'vbat_scale' in d['set'] or 'ibata_scale' in d['set']:
        notes.append('Battery scaling was converted from Betaflight units. Still compare the voltage the drone '
                     'reports with a multimeter in the Battery step - a wrong divider means wrong failsafes.')
    notes.append('PIDs, rates and OSD layouts are not translated: ArduPilot tunes itself (AUTOTUNE) and the '
                 'proven parameter set already flies a 5-inch quad well.')
    return {'board': info, 'params': out, 'notes': notes, 'serial_order': order}


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    text = open(sys.argv[1], encoding='utf-8', errors='replace').read()
    res = translate(text, sys.argv[2] if len(sys.argv) > 2 else None)
    print(json.dumps(res['board'], indent=2))
    for p in res['params']:
        print('%-20s %-8s # %s' % (p['name'], p['value'], p['why']))
    for n in res['notes']:
        print('NOTE:', n)
