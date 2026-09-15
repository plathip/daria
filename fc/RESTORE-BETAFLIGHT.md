# How to get back to Betaflight (exactly as it was)

Your complete Betaflight configuration is saved under `fc/backups/betaflight-<date>/`
(`diff_all.txt` is the important one; `dump_all.txt` is the belt-and-suspenders
full copy). Flashing ArduPilot does NOT destroy anything permanently — the
chip is simply rewritten, and can be rewritten back any time in ~5 minutes:

1. **Enter DFU mode**: unplug USB, hold the BOOT button on the FC, plug USB
   in while holding, release. (Or from ArduPilot: it shows up in Betaflight
   Configurator's flasher as a DFU device after this.)
2. **Flash Betaflight**: Betaflight Configurator → Firmware Flasher →
   target `SPEEDYBEEF405V5` → select the same Betaflight version you had
   (see the first lines of `version.txt` in the backup) → enable
   "Full chip erase" → Flash Firmware.
3. **Restore your config**: connect, open the CLI tab, paste the entire
   contents of `diff_all.txt`, press Enter, then type `save`.
4. Done — every rate, PID, OSD layout, VTX table and switch is back.

Note: the OX32 ESCs are never touched by any of this — motor/ESC firmware
lives on the ESC board, not the FC.
