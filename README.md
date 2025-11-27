# Skyzone → AtomRC Head Tracker Bridge

Two ESP32 sketches for the **goggles side** (choose one):
- `HeadTracker_ESP32_TX_Gemini_by_ymanda.ino` — baseline TX with 3-position switch and simple smoothing.
- `HeadTracker_ESP32_RX_Gemini_by_ymanda.ino` — enhanced TX with joystick/button control, auto-center, mode cycling, and adaptive filtering.

Both can drive a local gimbal over PWM, push CRSF to an ELRS TX module, or send ESP-NOW packets. The hardware switch decides the output; there is no automatic fallback.

## 1) Pin maps (default)
**Common to both sketches**
- PPM in from Skyzone head tracker: `GPIO25` (interrupt pin)
- Servos (local bench PWM): `GPIO18` pan, `GPIO19` tilt
- ELRS UART TX → module RX: `GPIO17` @ 420000 baud (UART1 TX only)
- Mode switch inputs (pulled-up): `GPIO32` (SW A), `GPIO33` (SW B)

**Extra on enhanced sketch**
- Recenter button / joystick-UP input: `GPIO26` to GND

**Pin caution:** Avoid sensitive/boot pins if they conflict on your board (e.g., `GPIO0/2/15`, `EN`, flash/PSRAM pins). If you remap, update the constants at the top of each sketch.

## 2) 3-position switch (TX side)
- A=LOW, B=HIGH → **Wired PWM** (bench/local servos)
- A=HIGH, B=LOW → **CRSF/ELRS** (normal)
- A=HIGH, B=HIGH (middle) → **ESP-NOW** (manual)

## 3) Enhanced TX behaviors (`HeadTracker_ESP32_RX_...ino`)
- **Startup:** holds center for 0.5 s, waits ~0.8 s of stable PPM, then auto-centers so the current head pose becomes the new 90°/90° reference.
- **Button on GPIO26:** hold to force center; releasing re-centers offsets to the current pose (same effect as the `center` command). Double-click cycles modes: Normal → Racing fixed (pan center, tilt +20° up ≈ 70° servo) → Hybrid tilt-only (pan locked at activation, tilt tracks) → Normal.
- **Failsafe:** if PPM is missing for >1 s, targets return to center.
- **Filtering:** adaptive Gain+Skip filter (pan range 2–10°, tilt range 2–5°). Pan may skip frames at small errors for smoothness; tilt always updates each loop. ESP-NOW sends the filtered `currentPan/currentTilt`.

## 4) Baseline TX behaviors (`HeadTracker_ESP32_TX_...ino`)
- Same pins and switch; no button logic or mode cycling.
- Smoothing: simple low-pass toward targets (`PAN_GAIN`/`TILT_GAIN` = 0.20). ESP-NOW and CRSF use the smoothed targets; flag bit0 in packets marks PPM validity.

## 5) Operating paths
- **Wired PWM:** local servos on `GPIO18/19` (bench/test).
- **CRSF / ELRS:** UART1 TX on `GPIO17` @ 420000 baud into an ELRS TX module (RX pin unused).
- **ESP-NOW:** send `ControlPacket` with checksum; set `ESP_NOW_PEER_MAC` in the sketch to the RX/drone ESP32 STA MAC. Bit0 of `flags` reports PPM validity.

## 6) Useful serial commands (both sketches)
- `center` – re-center offsets to the current pose (and reset to NORMAL on the enhanced sketch)
- `debug` – dump PPM channels and current targets/mode
- `test` – sweep local servos
- `deadband X` – adjust deadband (degrees)

## 7) Flashing quick steps
- Board: ESP32 Dev Module (Arduino IDE)
- Serial Monitor: 115200 baud; CRSF UART1: 420000 baud TX on `GPIO17`
- Set pins/macros as needed; upload the chosen TX sketch to the goggles-side ESP32.

## 8) ASCII sketches
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

## 9) Finding Skyzone HT OUT wires with a multimeter
1) Plug the stereo jack into the Skyzones. Open the plug if possible.  
2) Set multimeter to continuity/beep.  
3) **Ground:** put black probe on the sleeve (outer ring). Touch red probe to the wires; the one that beeps to sleeve is ground.  
4) **Signal:** keep black on ground wire. Switch to DC volts; power on goggles/head tracker so PPM is present. Probe the remaining wires with red; the one showing a small pulsing voltage is the PPM signal (pan/tilt). The third lead is the “dead”/unused wire—ignore it.  
5) Wire PPM → ESP32 PPM input; ground → ESP32 GND (shared with servos and ELRS module).

## 10) USB “red-wire cut” trick
When the ESP32 is powered from a flight battery/BEC and you also plug USB for Serial Monitor:
- Cut the **5V (red)** conductor inside the USB cable.
- Leave GND and data (D+/D−) intact.
This prevents USB backfeeding the BEC and avoids needing TVS/fast diodes for plug-in transients.

## 11) ELRS module roles (important)
- Goggles side needs a **TX-capable ELRS module**.
- Drone side needs an **RX module**.
Using two RX modules will never bind (the photo with two receivers is illustrative only). Ensure both share the same bind phrase and region (LBT/FCC).

## 12) Getting ESP-NOW MAC addresses (for pairing)
You need each ESP32’s Wi‑Fi STA MAC to fill `ESP_NOW_PEER_MAC` in the TX sketch.

**Method A — Arduino IDE (easy)**
1) Open Arduino IDE → File → Examples → WiFi → `WiFiScan` (or any blank sketch).  
2) Replace contents with:
```
#include <WiFi.h>
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.printf("STA MAC: %s\n", WiFi.macAddress().c_str());
}
void loop() {}
```
3) Select board `ESP32 Dev Module`, correct COM/tty port.  
4) Upload, then open Serial Monitor at 115200. The printed `STA MAC` is what you copy (format: `AA:BB:CC:DD:EE:FF`).
5) Repeat on the second ESP32 and note both MACs. Put the RX/drone MAC into `ESP_NOW_PEER_MAC` in the TX sketch. If you want reverse pairing, also add the TX MAC in the RX peer list (currently TX just sends, so only TX needs the RX MAC).

**Method B — CLI with esptool**
1) Install `esptool.py` (comes with Arduino IDE install or `pip install esptool`).  
2) Connect the ESP32 over USB and find the port (e.g., `/dev/ttyUSB0`, `/dev/ttyACM0`, `COM3`).  
3) Run: `esptool.py --port /dev/ttyUSB0 read_mac`  
4) Note the `MAC:` line. Repeat for the other board.

Tip: If your ESP32 boot log shows the MAC at reset, you can also just open Serial Monitor at power-up and read the `wifi: mode : sta (...) mac:aa:bb:cc:dd:ee:ff` line.
