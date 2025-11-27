
ESP32 HeadTracker – Skyzone → AtomRC (ELRS / ESP-NOW)

Overview
Transforms Skyzone head-tracking PPM into PAN/TILT commands sent through ELRS (CRSF) or ESP-NOW, then drives an AtomRC gimbal using a second ESP32.

Diagram:

Skyzone HT (PPM)
        │
        ▼
 ┌───────────────┐        CRSF/ELRS        ┌───────────────┐
 │   ESP32 TX    │ ───────────────────────▶ │   ESP32 RX    │
 │ (Goggles side)│                          │ (Gimbal side) │
 └───────────────┘        ESP-NOW fallback  └───────────────┘
        │                                           │
        └── PWM bench (optional)                    └── Servos PAN/TILT


Pin Mapping

TX (Goggles)
PPM input (Skyzone)  → GPIO25
Mode switch A        → GPIO32
Mode switch B        → GPIO33
Button / Joystick UP → GPIO26
Pan servo (bench)    → GPIO18
Tilt servo (bench)   → GPIO19
ELRS CRSF TX → module → GPIO17

RX (Drone)
CRSF UART RX         → GPIO16
ESP-NOW (WiFi STA)   → internal
Pan servo            → GPIO18
Tilt servo           → GPIO19


3-Position Output Switch (TX)
A=LOW,  B=HIGH → Wired PWM (bench)
A=HIGH, B=LOW  → CRSF / ELRS  (normal flight)
A=HIGH, B=HIGH → ESP-NOW


Button Logic (TX – GPIO26)
Hold button:
    camera forced to center
Release button:
    recenter (new offsets)
Double click:
    NORMAL → RACING → TILT-ONLY → NORMAL → ...


Mode details:
NORMAL      : Pan+Tilt follow head tracker
RACING      : Pan=90°, Tilt=+20° up
TILT-ONLY   : Pan locked at activation, Tilt follows PPM


Angle Conventions

PAN:
    0°   = left
    90°  = center
    180° = right

TILT:
    smaller value = up (sky)
    90°           = horizon
    larger value  = down (ground)


Data Flow

CRSF / ELRS Path (priority)
TX:
    PPM → filtering → degrees → CRSF RC frame → ELRS module
RX:
    ELRS RX → CRSF RC frame → degrees → smoothing → servos

ESP-NOW Fallback
TX:
    filtered degrees → ControlPacket → ESP-NOW send
RX:
    ESP-NOW packet → checksum → targetPan/targetTilt → smoothing → servos

Priority:
CRSF alive → use CRSF
else if ESP-NOW alive → use ESP-NOW
else → auto-center


Filtering (TX)

error = |target – current|

If error small:
    gain = low
    update rate = slow
If error large:
    gain = high
    update rate = fast

PAN range: 2°→10° (gain 0.04 → 1.0)
TILT range: 2°→5° (gain 0.01 → 1.0)


ControlPacket Format (ESP-NOW)

struct {
    uint16 header   = 0xA55A
    uint16 pan      = filtered pan deg
    uint16 tilt     = filtered tilt deg
    uint16 flags    = bit0 = PPM valid
    uint16 checksum = XOR(header,pan,tilt,flags,0x55AA)
}


Useful Serial Commands
center      = recenter
debug       = dump status
test        = servo sweep test (local)
deadband X  = set deadband (TX only)


Wiring ASCII

Skyzone → ESP32 TX
Skyzone HT PPM ─────────> GPIO25
Skyzone GND ────────────> ESP32 GND

ESP32 TX → ELRS
GPIO17 (TX) → ELRS module RX
GND shared

ESP32 RX → Servos
GPIO18 → Pan servo
GPIO19 → Tilt servo
5V/GND → from BEC


ESP-NOW MAC Setup

#include <WiFi.h>
WiFi.mode(WIFI_STA);
Serial.println(WiFi.macAddress());

Copy RX MAC into TX:
const uint8_t ESP_NOW_PEER_MAC[6] = {xx,xx,xx,xx,xx,xx};


Files Summary
HeadTracker_ESP32_TX_Gemini_by_ymanda.ino  → goggles-side logic
HeadTracker_ESP32_RX_Gemini_by_ymanda.ino  → gimbal-side logic
README.md                                   → this document

Auteur: Yannick Mandaba (Ymanda)
LAst edit: 27 NOV 2025