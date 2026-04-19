# RAK4630 Software Flow (J-Link)

## Board Setup
- Board package: `rak_rui:nrf52:WisCoreRAK4631Board`
- Sketch output directory: `./build`
- Flash image to use: `RAK4630LEDTEST.ino_full.hex`

## Why `_full.hex`
The `rak_rui` platform expects SoftDevice content. Flashing only `*.ino.hex` can lead to startup faults.
Always flash `*.ino_full.hex` for this project flow.

## Daily Workflow
1. Build:
   - VS Code task: `Arduino: Build RAK4631`
2. Upload:
   - VS Code task: `Arduino: Upload J-Link`

Current upload task runs:
- `nrfjprog --jdll /Applications/SEGGER/JLink_V794e/libjlinkarm.dylib -f nrf52 --recover`
- `nrfjprog --jdll /Applications/SEGGER/JLink_V794e/libjlinkarm.dylib -f nrf52 --program ${workspaceFolder}/build/RAK4630LEDTEST.ino_full.hex --chiperase --verify`
- `nrfjprog --jdll /Applications/SEGGER/JLink_V794e/libjlinkarm.dylib -f nrf52 --reset`

## Manual Terminal Commands
```bash
cd /Users/louis/Downloads/RAK4630LEDTEST
arduino-cli compile --fqbn rak_rui:nrf52:WisCoreRAK4631Board --output-dir ./build .
nrfjprog --jdll /Applications/SEGGER/JLink_V794e/libjlinkarm.dylib -f nrf52 --recover
nrfjprog --jdll /Applications/SEGGER/JLink_V794e/libjlinkarm.dylib -f nrf52 --program ./build/RAK4630LEDTEST.ino_full.hex --chiperase --verify
nrfjprog --jdll /Applications/SEGGER/JLink_V794e/libjlinkarm.dylib -f nrf52 --reset
```

## If Flash Fails with J-Link Error -102
1. Close tools that hold J-Link (RTT Client/Viewer, GDB server).
2. Kill stale workers if needed:
```bash
killall -9 JLinkRTTClientExe 2>/dev/null || true
killall -9 jlinkarm_nrf_worker_osx 2>/dev/null || true
```
3. Retry recover + flash sequence.

## Hardware Debug Notes Learned
- If GPIO drives correctly with load removed, software is likely fine.
- Diode check (Schottky): one-way conduction around ~0.2-0.3 V is expected.
- For low-side NMOS motor stage, verify MOSFET orientation and gate path if gate voltage collapses.

## Saved Test Sketch
Archived test sketch path:
- `saved_tests/button_motor_feedback_test/button_motor_feedback_test.ino`
