# Skyzone → AtomRC Head Tracker Bridge

`Gimbal-tracking-Gemini.ino` turns the Skyzone HT OUT jack into smooth pan/tilt motion on an AtomRC two–servo gimbal driven by an ESP32.  
The project’s goal is to mirror the pilot’s head movement (≈180° usable yaw) while letting the camera sweep its full 270° mechanical travel. To hide the extra 90°, the sketch keeps the gimbal synchronized through most of the range and progressively accelerates the camera near the edges so it can cover the remaining arc without the pilot noticing a hard stop. A light smoothing filter and user-tunable deadband keep the rig steady in turbulent moments.

## Project layout

```
headtracker/
├── Gimbal-tracking-Gemini.ino          # Main firmware for ESP32 + ESP32Servo
└── helpers-doc/
    ├── PPM_Remapper_Arduino.ino        # Example showing raw PPM channel copy
    └── What is IRAM_ATTR.txt           # Notes about placing ISRs in IRAM
```

Only `Gimbal-tracking-Gemini.ino` needs to be uploaded to the ESP32. The helper document is a reminder of how the earlier PPM remapper worked and why the interrupt is tagged with `IRAM_ATTR`.

## Hardware overview
- **Input** – Skyzone FPV goggles (tested with 04X PRO) HT OUT jack, emitting head tracking on PPM channels 5 (yaw) and 6 (pitch).
- **Controller** – ESP32 running the sketch, powered from the goggles or a dedicated BEC.
- **Output** – AtomRC gimbal servos on GPIO18 (pan) and GPIO19 (tilt).
- **Power/signal split** – The HT OUT signal is shared: one branch goes to the flight controller as usual, the second branch feeds the ESP32 which regenerates the servo PWM.

## How the firmware works
1. **PPM capture** – GPIO2 uses an interrupt (`ppmInterrupt`) to decode 8 PPM slots. The ISR rejects out-of-range pulses and marks a frame complete when it sees a sync gap (`PPM_FRAME_GAP`).
2. **Channel extraction** – `processPPMFrame` reads channels 5 and 6, applies safety limits, then maps them onto the servo limits. By tweaking `PAN_MIN/MAX` and `TILT_MIN/MAX` you can match any gimbal.
3. **Head ↔ camera ratio** – Because the pilot only rotates about 180° comfortably, most of the range keeps a 1:1 mapping. Near the extremes (where the servo range exceeds the pilot’s), the output gets stretched so the camera sweeps to 270° without noticeable jumps.
4. **Smoothing and deadband** – `updateServos` uses a first-order “slew limiter” governed by `smoothFactor` and `deadband`. The defaults remove micro-jitters but you can tune them at runtime.
5. **Serial tooling** – Through the USB serial monitor you can center the gimbal, dump live channel values, run a travel test, or change tuning parameters, which is handy when the tracker is strapped to a pilot.

## Key settings

| Setting | Location | Purpose |
| --- | --- | --- |
| `PPM_INPUT_PIN` | top of sketch | GPIO wired to HT OUT (must be interrupt capable) |
| `PAN/TILT_PIN` | top of sketch | Servo outputs; move if your AtomRC controller uses other pins |
| `PAN_*` & `TILT_*` | top of sketch | Define the mechanical range and centers for each axis |
| `deadband` | runtime variable | Neutral zone around center to suppress micro-shakes |
| `smoothFactor` | runtime variable | Higher values = slower but steadier motion |

### Serial commands
- `center` – return both servos to their defined centers.
- `debug` – print the raw PPM values and current servo targets.
- `limits` – report the configured min/max angles.
- `deadband <value>` – change the deadband (0–50).
- `smooth <value>` – change the smoothing factor (1–50).
- `test` – run a scripted sweep to check binding or stalled motors.

## Usage checklist
1. Wire the HT OUT signal and ground from the Skyzone goggles to ESP32 GPIO2 and GND. Share the signal with the flight controller if necessary using a simple Y harness.
2. Connect AtomRC pan servo to GPIO18 and tilt servo to GPIO19 (or adjust the pin constants).
3. Flash `Gimbal-tracking-Gemini.ino` with the Arduino IDE or PlatformIO (board: ESP32 Dev Module).
4. Open the serial monitor at 115200 baud; when the tracker is stationary run `center` to align the gimbal mechanically.
5. Wear the goggles, move your head slowly through its normal range, and verify the gimbal stays synchronized. If you feel oscillations, raise `smoothFactor`; if latency feels high, lower it instead.
6. Once tuned, disconnect USB power, power the ESP32 from your flight battery/BEC, and strap it to the goggles or the headset strap.

## Operating modes

### 1. Wired PWM (single ESP32)
```
Skyzone HT OUT  ──> GPIO2 (PPM) ──> ESP32 ──> GPIO18/19 ──> AtomRC pan/tilt servos
                              GND ────────────────────────┘
```
The goggles’ head-tracker signal is read and immediately turned into PWM on GPIO18/19. This is the original “two wires only” setup for benches or when the ESP32 sits close to the gimbal.

### 2. ELRS bridge (UART payload)
```
[Goggles ESP32] --PPM--> smoothing --> UART TX17 ====> DIY ELRS TX (bind phrase X)
                                       ^
                                       |  (secret phrase set in ExpressLRS Configurator)
                                       v
[Drone ESP32] <== ELRS RX (same phrase) --UART--> packet decode --> GPIO18/19 servos
```
We reuse the existing ExpressLRS DIY hardware: the goggles ESP32 sends the smoothed angles over Serial1 to an ELRS TX module that shares your bind phrase. On the drone, an ELRS RX outputs the same serial stream to another ESP32, which drives the servos. A secret phrase means re-linking is automatic if power glitches.

### 3. ESP-NOW pair (two ESP32s)
```
Goggles ESP32   Wi-Fi STA + ESP-NOW  ~~~wireless~~~  Drone ESP32
PPM -> filter -> packet             <----MAC pair---->  packet -> servos
```
Both ESP32 boards talk directly over ESP-NOW using their Wi-Fi MAC addresses (fill them in at the top of the TX/RX sketches). No router, no FC involvement—just a private wireless bridge with the same smoothing and failsafe behavior.

The convertion to UART signal over ELRS, with addition of 2 ELRS module (one used as tranmitter and the other one as receptor) is yet to come.
