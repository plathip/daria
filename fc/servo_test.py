"""Bench-test the payload-release servo on a spare output (S5/S6).

Sends DO_SET_SERVO center / +45 / center / -45 / center with pauses,
and reads SERVO_OUTPUT_RAW back so you can see the FC really drove the
pin. Refuses to run if the aircraft is armed. Motors are never touched.

Usage:
    python fc/servo_test.py            # channel 6 = the pad marked S5 (!) on this board
    python fc/servo_test.py --ch 5     # other servo pad
"""
import argparse
import sys
import time

from pymavlink import mavutil
from serial.tools import list_ports

CENTER, CW45, CCW45 = 1500, 1900, 1100   # MG90S: ~800 us over ~90 deg


def find_fc():
    for p in list_ports.comports():
        if p.vid == 0x1209:
            return p.device
    raise SystemExit('No ArduPilot board on USB.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ch', type=int, default=6)   # our servo: pad labeled S5, answers as output 6
    args = ap.parse_args()

    m = mavutil.mavlink_connection(find_fc(), baud=115200, source_system=255)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        raise SystemExit('No heartbeat.')
    if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise SystemExit('ABORT: aircraft is ARMED. This is a bench-only tool.')
    print('connected, disarmed. testing output %d' % args.ch)

    # stream servo outputs at 5 Hz so we can read back the pin value
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            36, 200000, 0, 0, 0, 0, 0)   # SERVO_OUTPUT_RAW

    def readback():
        t0 = time.time()
        while time.time() - t0 < 2:
            msg = m.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
            if msg and msg.get_srcSystem() == m.target_system and msg.get_srcComponent() == 1:
                return getattr(msg, 'servo%d_raw' % args.ch, None)
        return None

    def set_pwm(pwm, label):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                args.ch, pwm, 0, 0, 0, 0, 0)
        t0 = time.time()
        rb = None
        while time.time() - t0 < 3:              # the pin takes ~1 s to show it
            rb = readback()
            if rb == pwm:
                print('  %-18s -> commanded %d us, pin reads %d us  OK' % (label, pwm, rb))
                return True
        print('  %-18s -> commanded %d us, pin reads %s  (!!)' % (label, pwm, rb))
        return False

    ok = True
    ok &= set_pwm(CENTER, 'center')
    time.sleep(3)
    ok &= set_pwm(CW45, '+45 (clockwise)')
    time.sleep(3)
    ok &= set_pwm(CENTER, 'center')
    time.sleep(3)
    ok &= set_pwm(CCW45, '-45 (anticlockwise)')
    time.sleep(3)
    ok &= set_pwm(CENTER, 'center')
    print('electrical result: %s' % ('all commands reached the pin' if ok else 'SOME COMMANDS DID NOT TAKE'))


if __name__ == '__main__':
    main()
