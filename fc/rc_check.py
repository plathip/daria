"""RC link check — read-only proof the laptop <-> FC <-> radio chain is alive.

Connects to the flight controller over USB (auto-detected by vendor id
0x1209), waits for a heartbeat, then prints the RC channel count and live
channel values. Move the sticks and watch the numbers change.

No arming, no parameter writes, no RC overrides — safe with props on or off.

Usage:
    python fc/rc_check.py               # stream until Ctrl+C
    python fc/rc_check.py --seconds 6   # stop after 6 s (for scripted checks)

If it prints chancount=0 or no RC_CHANNELS at all, remember the ELRS WiFi
trap: the receiver must see the transmitter within ~60 s of power-up, so
switch the radio ON first, then power-cycle the aircraft.
"""
import argparse
import sys
import time

from pymavlink import mavutil
from serial.tools import list_ports


def find_fc():
    for p in list_ports.comports():
        if p.vid == 0x1209:          # ArduPilot USB vendor id
            return p.device
    raise SystemExit('No ArduPilot board found on USB. Is it plugged in and powered?')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=0,
                    help='stop after this many seconds (0 = run until Ctrl+C)')
    args = ap.parse_args()

    port = find_fc()
    print('connecting on %s ...' % port)
    m = mavutil.mavlink_connection(port, baud=115200, source_system=255)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        raise SystemExit('No heartbeat in 10 s. Cold power-cycle the board '
                         '(unplug, replug) and try again.')
    print('connected: system %d component %d' % (m.target_system, m.target_component))

    # ask for RC_CHANNELS at 4 Hz (not saved; harmless if already streaming)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
        250000, 0, 0, 0, 0, 0)

    deadline = (time.time() + args.seconds) if args.seconds else None
    got = False
    last_print = 0.0
    while deadline is None or time.time() < deadline:
        msg = m.recv_match(type='RC_CHANNELS', blocking=True, timeout=2)
        if msg is None:
            if not got:
                print('no RC_CHANNELS yet ...')
            continue
        # only trust the vehicle itself, never another GCS on the link
        if msg.get_srcSystem() != m.target_system or msg.get_srcComponent() != 1:
            continue
        got = True
        now = time.time()
        if now - last_print < 0.5:
            continue
        last_print = now
        n = min(msg.chancount, 18)
        vals = ['%d:%d' % (i, getattr(msg, 'chan%d_raw' % i)) for i in range(1, n + 1)]
        print('chancount=%-2d  %s' % (msg.chancount, '  '.join(vals) if vals else '(no RC input)'))

    if not got:
        print('\nHeartbeat OK but no RC_CHANNELS received — link to the FC works,')
        print('but check the radio (on before aircraft power? ELRS in WiFi mode?).')
        sys.exit(1)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\ninterrupted')
        sys.exit(1)
