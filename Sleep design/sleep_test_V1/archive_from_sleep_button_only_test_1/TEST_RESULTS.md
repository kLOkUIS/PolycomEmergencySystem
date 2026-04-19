# sleep_button_only Test 1 Results

## Test Date
17 April 2026

## Methodology
Diagnostic matrix to identify source of high current draw (~407 µA vs expected ~38-40 µA baseline).

## Phase 1: Variant Testing (WITH debounce loops)

| Variant | Skip LoRa | LPM | Pre-sleep | Direct Sleep | Avg Current (µA) | Notes |
|---------|---:|---:|---:|---:|---:|---|
| A | 1 | 1 | 0 | 0 | 407.11 | Pure nRF52, no LoRa |
| B | 0 | 1 | 1 | 0 | 407.37 | Only api.lora.precv(0) |
| C | 0 | 1 | 0 | 1 | 407.33 | Only Radio.Sleep() |
| D | 0 | 1 | 1 | 1 | 407.38 | Both LoRa calls |

## Phase 2: Root Cause Fix (REMOVED debounce loops)

### Issue Identified
The sketches contained debounce loops that woke the MCU every 25ms:
```cpp
while (buttonPressedRaw()) {
  api.system.sleep.all(25);  // Prevents deep sleep!
}
```

### Solution
Removed all debounce loops. Rely on RUI3's interrupt edge detection with no busy-polling.

### Result
**✓ Baseline (strict, no debounce): 25.25 µA** ← Ready for TX integration

This is **10x better than variant A-D** and exceeds forum expectations (38–40 µA).

## Key Learnings

1. **Debounce loops defeat deep sleep** — periodic `sleep.all(25)` wakes the MCU even while "sleeping"
2. **RUI3 interrupt edge detection is reliable** — no need for software debounce during sleep
3. **LoRa calls alone don't drain power** — only when blocking inside loops
4. **Hardware floor: 25.25 µA** on your custom board with LPM enabled

## Validated Baseline Code

Location: 
- [Sleep design/sleep_button_only/sleep_button_only.ino](Sleep%20design/sleep_button_only/sleep_button_only.ino)
- [firmwares/sleep_button_only/sleep_button_only.ino](firmwares/sleep_button_only/sleep_button_only.ino)

Ready for TX integration.
