# BMCU370
BMCU Xing-C modified version latest (BMCU-C 370 Hall Version V0.1-0020) source code. Original project: [Xing-C/BMCU370x](https://github.com/Xing-C/BMCU370x). Includes some minor personal optimizations.


# Links
- English wiki: https://wiki.yuekai.fr/
- Chinese wiki: https://bmcu.wanzii.cn/


# Changelog
Copied from group files

### 2025-07-17 - 0020
- Fixed lighting logic errors that caused some states to not light up
- Fixed channels unexpectedly coming online
- Fixed anti-disconnect feature (previously not working)
- Rewrote lighting system, fixed flickering issues, reduced refresh rate
- When a channel has an error, attempts to update with red light every 3 seconds to prevent channels inserted after BMCU enters working state from not lighting up


### 2025-07-06 - 0019 Modified Version
Also compatible with dual micro-switch Hall version.

**Changes from original 0019 compared to original 0013:**
- P1X1 can now support 16 colors depending on the firmware flashed
- Fixed issue where filament information could not be saved after P1X1 printer firmware update (currently latest 00.01.06.62) or with latest slicer software (currently 2.1.1.52)
- Modified online logic detection to prevent incorrect channel online status in certain states
- Modified motor control logic, using different calls for high and low voltage positions

**Changes from previous 0013 modified version (lighting effects anti-overheating version):**
- Mainboard lighting: red breathing when not connected to printer, white breathing during normal operation
- Further reduced brightness of buffer lights and mainboard lights
- Unload section: removed control for A1