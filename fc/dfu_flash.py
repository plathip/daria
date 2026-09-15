"""Flash ArduPilot onto an STM32 flight controller over USB DFU - no Betaflight Configurator needed.

    python fc/dfu_flash.py <arducopter_with_bl.hex | .bin> [--port COMx] [--info] [--verify-only]

What it does, in order:
  1. If a Betaflight board is on a COM port (or --port given), asks it over MSP to reboot into the
     STM32 ROM bootloader (DFU) - no BOOT button needed.
  2. Waits for the "STM32 BOOTLOADER" USB device (VID 0483 PID DF11).
  3. Erases only the flash pages the image covers, writes it at 0x08000000, READS EVERY BYTE BACK
     and compares, then leaves DFU so the board boots ArduPilot.

Needs the WinUSB driver on the DFU device (ImpulseRC Driver Fixer / Zadig install it; if you ever
flashed with Betaflight Configurator on this PC it is already there) and `pip install pyusb libusb-package`.
The ROM bootloader cannot be damaged: whatever happens, BOOT-button + this tool (or Configurator) recovers.
"""
import argparse
import os
import struct
import sys
import time

FLASH_BASE = 0x08000000
DFU_DETACH, DFU_DNLOAD, DFU_UPLOAD, DFU_GETSTATUS, DFU_CLRSTATUS, DFU_GETSTATE, DFU_ABORT = range(7)
ST_IDLE, ST_DNBUSY, ST_DNLOAD_IDLE, ST_MANIFEST_SYNC, ST_MANIFEST, ST_MANIFEST_WAIT, ST_UPLOAD_IDLE, ST_ERROR = 2, 4, 5, 6, 7, 8, 9, 10
STATE_NAME = {0: 'appIDLE', 1: 'appDETACH', 2: 'dfuIDLE', 3: 'dfuDNLOAD-SYNC', 4: 'dfuDNBUSY', 5: 'dfuDNLOAD-IDLE',
              6: 'dfuMANIFEST-SYNC', 7: 'dfuMANIFEST', 8: 'dfuMANIFEST-WAIT-RESET', 9: 'dfuUPLOAD-IDLE', 10: 'dfuERROR'}


def log(msg):
    print(msg, flush=True)


# ---------------------------------------------------------------- image files
def parse_hex(path):
    """Intel HEX -> (start_address, bytes). Gaps inside the image are filled with 0xFF."""
    mem = {}
    ext = 0
    for line in open(path, encoding='ascii', errors='replace'):
        line = line.strip()
        if not line.startswith(':'):
            continue
        rec = bytes.fromhex(line[1:])
        n, addr, typ, data = rec[0], (rec[1] << 8) | rec[2], rec[3], rec[4:4 + rec[0]]
        if (sum(rec) & 0xFF) != 0:
            raise ValueError('hex checksum error in line: ' + line[:20])
        if typ == 0:
            base = ext + addr
            for i, b in enumerate(data):
                mem[base + i] = b
        elif typ == 4:
            ext = ((data[0] << 8) | data[1]) << 16
        elif typ == 2:
            ext = ((data[0] << 8) | data[1]) << 4
        elif typ == 1:
            break
    if not mem:
        raise ValueError('no data records in ' + path)
    lo, hi = min(mem), max(mem)
    img = bytearray(b'\xff' * (hi - lo + 1))
    for a, b in mem.items():
        img[a - lo] = b
    return lo, bytes(img)


def load_image(path):
    if path.lower().endswith('.hex'):
        return parse_hex(path)
    data = open(path, 'rb').read()
    return FLASH_BASE, data


# ---------------------------------------------------------------- Betaflight -> DFU
def betaflight_port(port=None):
    from serial.tools import list_ports
    for p in list_ports.comports():
        if port and p.device != port:
            continue
        if p.vid == 0x0483 and p.pid == 0x5740:      # STM32 virtual COM port = Betaflight / iNav
            return p.device
    return port


def msp_reboot_to_dfu(port):
    """MSP_REBOOT (68) with rebootMode 1 = ROM bootloader (DFU)."""
    import serial
    payload = bytes([1])
    frame = bytearray(b'$M<') + bytes([len(payload), 68]) + payload
    crc = 0
    for b in frame[3:]:
        crc ^= b
    frame.append(crc)
    with serial.Serial(port, 115200, timeout=0.5) as ser:
        time.sleep(0.3)
        ser.reset_input_buffer()
        ser.write(frame)
        ser.flush()
        time.sleep(0.5)


def find_dfu(backend):
    import usb.core
    return usb.core.find(idVendor=0x0483, idProduct=0xDF11, backend=backend)


# ---------------------------------------------------------------- DFU (DfuSe, STM32 ROM bootloader)
class Dfu:
    def __init__(self, dev, timeout=5000):
        import usb.util
        self.usb = usb.util
        self.dev = dev
        self.timeout = timeout
        self.iface = 0
        try:
            dev.set_configuration()
        except Exception:
            pass
        self.usb.claim_interface(dev, self.iface)
        dev.set_interface_altsetting(interface=self.iface, alternate_setting=0)
        self.layout = self._layout()
        self.xfer = self._transfer_size()

    def _layout(self):
        """[(start, size), ...] for every page, from the '@Internal Flash /0x08000000/04*016Kg,01*064Kg,07*128Kg' string."""
        cfg = self.dev.get_active_configuration()
        intf = self.usb.find_descriptor(cfg, bInterfaceNumber=0, bAlternateSetting=0)
        desc = self.usb.get_string(self.dev, intf.iInterface)
        parts = desc.split('/')
        base = int(parts[1], 16)
        pages = []
        addr = base
        for seg in parts[2].split(','):
            seg = seg.strip()
            cnt, rest = seg.split('*')
            size = int(rest[:-2]) * {'K': 1024, 'M': 1024 * 1024, ' ': 1, 'B': 1}[rest[-2]]
            for _ in range(int(cnt)):
                pages.append((addr, size))
                addr += size
        self.flash_desc = desc
        self.flash_base, self.flash_end = base, addr
        return pages

    def _transfer_size(self):
        cfg = self.dev.get_active_configuration()
        intf = self.usb.find_descriptor(cfg, bInterfaceNumber=0, bAlternateSetting=0)
        extra = bytes(intf.extra_descriptors)
        if len(extra) >= 9 and extra[1] == 0x21:      # DFU functional descriptor
            return extra[5] | (extra[6] << 8)
        return 2048

    # --- low level
    def status(self):
        st = self.dev.ctrl_transfer(0xA1, DFU_GETSTATUS, 0, self.iface, 6, self.timeout)
        poll = st[1] | (st[2] << 8) | (st[3] << 16)
        if poll:
            time.sleep(poll / 1000.0)
        return st[0], st[4]                            # (bStatus, bState)

    def clear(self):
        self.dev.ctrl_transfer(0x21, DFU_CLRSTATUS, 0, self.iface, None, self.timeout)

    def abort(self):
        self.dev.ctrl_transfer(0x21, DFU_ABORT, 0, self.iface, None, self.timeout)

    def idle(self):
        for _ in range(3):
            _, state = self.status()
            if state == ST_IDLE:
                return
            if state == ST_ERROR:
                self.clear()
            else:
                self.abort()
        _, state = self.status()
        if state != ST_IDLE:
            raise RuntimeError('bootloader stuck in %s' % STATE_NAME.get(state, state))

    def _cmd(self, data):
        self.dev.ctrl_transfer(0x21, DFU_DNLOAD, 0, self.iface, data, self.timeout)
        _, state = self.status()                       # dfuDNBUSY, with the poll time the command needs
        if state != ST_DNBUSY:
            raise RuntimeError('command %02x refused: %s' % (data[0], STATE_NAME.get(state, state)))
        st, state = self.status()
        if state != ST_DNLOAD_IDLE:
            raise RuntimeError('command %02x failed: status %d state %s' % (data[0], st, STATE_NAME.get(state, state)))

    def set_address(self, addr):
        self._cmd(struct.pack('<BI', 0x21, addr))

    def erase_page(self, addr):
        self._cmd(struct.pack('<BI', 0x41, addr))

    # --- high level
    def pages_for(self, start, length):
        end = start + length
        return [(a, s) for a, s in self.layout if a < end and a + s > start]

    def write(self, start, data, progress=None):
        self.idle()
        self.set_address(start)
        self.abort(); self.idle()
        self.set_address(start)
        n = 0
        for off in range(0, len(data), self.xfer):
            chunk = data[off:off + self.xfer]
            self.dev.ctrl_transfer(0x21, DFU_DNLOAD, 2 + n, self.iface, chunk, self.timeout)
            _, state = self.status()
            if state != ST_DNBUSY:
                raise RuntimeError('write refused at 0x%08x: %s' % (start + off, STATE_NAME.get(state, state)))
            st, state = self.status()
            if state != ST_DNLOAD_IDLE:
                raise RuntimeError('write failed at 0x%08x: status %d %s' % (start + off, st, STATE_NAME.get(state, state)))
            n += 1
            if progress and n % 16 == 0:
                progress(off + len(chunk), len(data))
        self.abort(); self.idle()

    def read(self, start, length, progress=None):
        self.idle()
        self.set_address(start)
        self.abort(); self.idle()
        out = bytearray()
        n = 0
        while len(out) < length:
            want = min(self.xfer, length - len(out))
            got = self.dev.ctrl_transfer(0xA1, DFU_UPLOAD, 2 + n, self.iface, want, self.timeout)
            if not len(got):
                break
            out += bytes(got)
            n += 1
            if progress and n % 16 == 0:
                progress(len(out), length)
        self.abort(); self.idle()
        return bytes(out)

    def leave(self, addr=FLASH_BASE):
        """Manifest: set the start address, send an empty download, the bootloader jumps to the firmware."""
        self.idle()
        self.set_address(addr)
        self.abort(); self.idle()
        self.set_address(addr)
        self.dev.ctrl_transfer(0x21, DFU_DNLOAD, 0, self.iface, None, self.timeout)
        try:
            self.status()
        except Exception:
            pass                                       # the device is gone: that is the success path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('image', nargs='?', help='arducopter_with_bl.hex or .bin')
    ap.add_argument('--port', help='COM port of a Betaflight board to reboot into DFU first')
    ap.add_argument('--info', action='store_true', help='only show what is connected')
    ap.add_argument('--verify-only', action='store_true', help='compare flash with the image, write nothing')
    ap.add_argument('--no-reboot', action='store_true', help='do not ask Betaflight to enter DFU')
    args = ap.parse_args()

    try:
        import usb.core                                  # noqa
        import libusb_package
        backend = libusb_package.get_libusb1_backend()
    except Exception as e:
        log('USB library missing (%s): run  pip install pyusb libusb-package' % e)
        return 2

    dev = find_dfu(backend)
    if dev is None and not args.no_reboot:
        port = betaflight_port(args.port)
        if port:
            log('Betaflight board on %s: asking it to reboot into the bootloader (DFU) ...' % port)
            try:
                msp_reboot_to_dfu(port)
            except Exception as e:
                log('  could not talk MSP on %s: %s' % (port, e))
            for _ in range(40):
                time.sleep(0.5)
                dev = find_dfu(backend)
                if dev is not None:
                    break
    if dev is None:
        log('NO board in DFU mode. Hold the BOOT button while plugging in USB, then run again.\n'
            'If Windows shows "STM32 BOOTLOADER" with a warning icon, run ImpulseRC Driver Fixer once.')
        return 3

    try:
        dfu = Dfu(dev)
    except Exception as e:
        log('DFU device found but cannot be opened (%s).\nUsually the driver: run ImpulseRC Driver Fixer (or Zadig -> WinUSB) '
            'once with the board in DFU mode, then retry.' % e)
        return 4
    log('DFU: %s  transfer %d bytes  serial %s' % (dfu.flash_desc, dfu.xfer, getattr(dev, 'serial_number', '?')))
    if args.info or not args.image:
        return 0

    start, data = load_image(args.image)
    log('image: %s  %d bytes at 0x%08x (%s)' % (os.path.basename(args.image), len(data), start,
                                                'ArduPilot with bootloader' if b'ArduPilot' in data or b'APJ' in data else 'raw'))
    if start < dfu.flash_base or start + len(data) > dfu.flash_end:
        log('image does not fit this chip (flash 0x%08x-0x%08x)' % (dfu.flash_base, dfu.flash_end))
        return 5

    def prog(done, total):
        log('  %3d%%  %d / %d' % (100 * done // total, done, total))

    if not args.verify_only:
        pages = dfu.pages_for(start, len(data))
        log('erasing %d pages (%d KB) ...' % (len(pages), sum(s for _, s in pages) // 1024))
        dfu.idle()
        for a, s in pages:
            dfu.erase_page(a)
        log('writing ...')
        dfu.write(start, data, prog)
    log('reading back to verify ...')
    back = dfu.read(start, len(data), prog)
    if back != data:
        bad = next(i for i in range(min(len(back), len(data))) if back[i] != data[i]) if back[:len(data)] != data else len(back)
        log('VERIFY FAILED at offset %d (0x%08x): the board must NOT be flown. Run again; if it fails again, '
            'flash with Betaflight Configurator.' % (bad, start + bad))
        return 6
    log('verified: every byte matches.')
    if not args.verify_only:
        log('leaving DFU - the board boots ArduPilot now (10-20 s). Then click Detect.')
        dfu.leave(start)
    return 0


if __name__ == '__main__':
    sys.exit(main())
