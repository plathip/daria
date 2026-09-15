"""Manual bench control of the payload release servo (output 6).

    python fc/release.py open     # 2500 us, keeps holding (pulses on)
    python fc/release.py close    # 1500 us, then relaxes silent
"""
import sys
import time

from pymavlink import mavutil
from serial.tools import list_ports

OPEN_PWM, CLOSE_PWM, CH = 2500, 1500, 6


def main():
    want = sys.argv[1] if len(sys.argv) > 1 else 'open'
    pwm = OPEN_PWM if want == 'open' else CLOSE_PWM
    port = next(p.device for p in list_ports.comports() if p.vid == 0x1209)
    m = mavutil.mavlink_connection(port, baud=115200, source_system=255)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        raise SystemExit('no heartbeat')
    if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise SystemExit('ABORT: armed')
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            36, 200000, 0, 0, 0, 0, 0)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                            CH, pwm, 0, 0, 0, 0, 0)
    t0 = time.time()
    rb = None
    while time.time() - t0 < 3:
        msg = m.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
        if msg and msg.get_srcComponent() == 1:
            rb = getattr(msg, 'servo%d_raw' % CH)
            if rb == pwm:
                break
    print('%s: commanded %d -> pin %s' % (want.upper(), pwm, rb))
    if want == 'close':
        time.sleep(1.5)
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                CH, 0, 0, 0, 0, 0, 0)
        print('relaxed (silent)')
    else:
        print('HOLDING with pulses on - run "python fc/release.py close" when done')


if __name__ == '__main__':
    main()
