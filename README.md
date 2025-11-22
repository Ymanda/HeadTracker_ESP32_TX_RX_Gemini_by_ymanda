# 1. Skyzone → AtomRC Head Tracker Bridge

Two ESP32 sketches:
- `HeadTracker_ESP32_TX_Gemini_by_ymanda.ino` (goggles side)
- `HeadTracker_ESP32_RX_Gemini_by_ymanda.ino` (drone/gimbal side)

Works three ways: wired PWM, CRSF/ELRS, or ESP-NOW bridge. RX prefers CRSF when present and falls back to ESP-NOW automatically.

## 2. Pin maps (default)
**Goggles ESP32 (TX sketch)**
- PPM in: `GPIO25` (interrupt pin; change `PPM_INPUT_PIN` if needed)
- Servos (bench/local PWM): `GPIO18` pan, `GPIO19` tilt
- ELRS UART TX → module RX: `GPIO17` @ 420000 baud
- Mode switch inputs (pulled-up): `GPIO32` (SW A), `GPIO33` (SW B)

**Drone ESP32 (RX sketch)**
- Servos: `GPIO18` pan, `GPIO19` tilt
- CRSF UART RX ← module TX: `GPIO16` @ 420000 baud
- PPM (optional local mode): `GPIO2` (`PPM_INPUT_PIN`; can be changed)

**Pin caution:** Avoid sensitive/boot pins if they conflict on your board (e.g., `GPIO0/2/15`, `EN`, flash/PSRAM pins). If you move anything, update the constants at the top of each sketch.

## 3. 3-position switch (TX side)
- A=LOW, B=HIGH → **Wired PWM** (local bench)
- A=HIGH, B=LOW → **CRSF/ELRS** (normal use)
- A=HIGH, B=HIGH (middle) → **ESP-NOW** (manual fallback)
RX side: no switch needed—auto-prefers CRSF when frames are present, else ESP-NOW.

## 4. ASCII sketches
**Wired PWM (bench, single ESP32)**
```
Skyzone HT OUT (PPM) ---> GPIO25 (PPM in)
                       \-> smoothing -> GPIO18/19 -> servos
GND shared
```

**CRSF / ELRS link**
```
[Goggles ESP32 TX] --PPM--> filter -> CRSF @420k -> ELRS TX module
                                         ~~~ RF ~~~
[Drone ESP32 RX] <- CRSF from ELRS RX -> decode -> servos (GPIO18/19)
```

**ESP-NOW pair**
```
Goggles ESP32 (ESP-NOW TX)  ~~~ Wi-Fi STA/ESP-NOW ~~~  Drone ESP32 (ESP-NOW RX)
PPM -> filter -> packet                                packet -> servos
```

## 5. Finding Skyzone HT OUT wires with a multimeter
1) Plug the stereo jack into the Skyzones. Open the plug if possible.  
2) Set multimeter to continuity/beep.  
3) **Ground:** put black probe on the sleeve (outer ring). Touch red probe to the wires; the one that beeps to sleeve is ground.  
4) **Signal:** keep black on ground wire. Switch to DC volts; power on goggles/head tracker so PPM is present. Probe the remaining wires with red; the one showing a small pulsing voltage is the PPM signal (pan/tilt). The third lead is the “dead”/unused wire—ignore it.  
5) Wire PPM → ESP32 PPM input; ground → ESP32 GND (shared with servos and ELRS module).

## 6. USB “red-wire cut” trick
When the ESP32 is powered from a flight battery/BEC and you also plug USB for Serial Monitor:
- Cut the **5V (red)** conductor inside the USB cable.
- Leave GND and data (D+/D−) intact.
This prevents USB backfeeding the BEC and avoids needing TVS/fast diodes for plug-in transients.

## 7. ELRS module roles (important)
- Goggles side needs a **TX-capable ELRS module**.
- Drone side needs an **RX module**.
Using two RX modules will never bind (the photo with two receivers is illustrative only). Ensure both share the same bind phrase and region (LBT/FCC).

## 8. Operating modes & failsafe
- **TX side:** switch sets Wired PWM / CRSF-ELRS / ESP-NOW. In ELRS mode, flip to middle if you want to force ESP-NOW as a manual fallback.
- **RX side:** auto-selects source. CRSF has priority; if no valid CRSF frames within timeout, it uses ESP-NOW. No manual action required.

## 9. Useful serial commands
- `center` – return servos to centers
- `debug` – dump channel/targets
- `test` – sweep servos
- `deadband X` (TX) – adjust deadband
- `mode pwm|auto` (RX) – local PPM vs RF auto

## 10. Flashing quick steps
- Board: ESP32 Dev Module (Arduino IDE)
- Baud: 115200 for Serial Monitor; 420000 on UART1 for CRSF
- Set pins above as needed; upload TX sketch to goggles ESP32, RX sketch to drone ESP32
