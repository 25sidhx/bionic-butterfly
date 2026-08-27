# Bionic Butterfly — RC Flapping Wing (S-DiY v1.1, corrected)

A 2-servo RC bionic butterfly. This repo is the **corrected firmware + build kit**
for the S-DiY "RC Bionic Butterfly" project (TruongVanSu91, 02/2026).

## What's here

| Path | Contents |
|------|----------|
| `firmware/firmware.ino` | **v1.1 corrected** flight code (verified-compiling, 8612 bytes) |
| `lib/PPMReader/` | Vendored Nikkilae PPMReader, rewritten to link cleanly on arduino:avr (no duplicate interrupt-vector table) |
| `print/` | 3 STL parts (servo mount, FC mount, wing arm) + slicer profile |
| `schematics/`, `docs/` | Original hand-drawn circuit + wing-design images |
| `BUILD_MANUAL.txt` | Full build/flash/wiring guide |
| `v11_diff.txt` | diff of original v1.0 → v1.1 |

## Fixes in v1.1 (vs the original S-DiY release)

- **[A] Real failsafe** — the original link-loss test was dead code (`latestValidChannelValue` never returns 0 after first link). v1.1 uses an edge-watchdog: >100 ms without a PPM pulse = link lost → wings ramp to neutral.
- **[B] Control authority at full flap** — flap amplitude now shrinks when steer+elevator+trim demand is high, so `constrain(5,175)` never silently eats your steering at max throttle.
- **[C] Smooth flap speed** — float math instead of integer `map()` truncation.
- **[D] `wasFailsafe` → `resetSinePhase`** — renamed; the old name did double duty (real failsafe + mode-transition reset).
- **[E] `ppm.begin()` guard** — commented, with a note to enable only if your PPMReader version needs explicit init.

Pin map: PPM on D2, left servo D4, right servo D5, LED D6 — matches the circuit
drawing. (If your board wires servos to D9/D10, change the two `.attach()` calls.)

## Build

```
arduino-cli compile --fqbn arduino:avr:nano \
  --libraries lib \
  --output-dir firmware/build-v11 firmware/
```

Flash the produced `firmware/build-v11/firmware.ino.hex` to an Arduino Nano
(ATmega328P, old bootloader for most clones).

## Note

This is the **code + build kit only** — no physical components are included.
The flying machine is yours to print and assemble.
