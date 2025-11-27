// HeadTracker_ESP32_TX_Gemini_by_ymanda.ino
//
// Skyzone (PPM) → ESP32 → ELRS / ESP-NOW → AtomRC gimbal
// ------------------------------------------------------
// Rôle de ce sketch (côté lunettes / "goggles"):
//   - Lire le PPM headtracker des lunettes Skyzone
//   - Convertir PAN / TILT en angles logiques centrés sur 90°
//   - Appliquer un filtre "Gain + Skip" (réponse douce / rapide selon l'erreur)
//   - Gérer les modes via le petit joystick / bouton:
//        • Press (court ou long) : aller au centre tant que c'est appuyé,
//                                 puis RECENTER (recalage) au relâchement
//        • Double-clic : faire défiler les modes
//              MODE_NORMAL        (tracking complet pan+tilt)
//           -> MODE_RACING_FIXED  (pan fixe 0°, tilt fixe +20° up)
//           -> MODE_HYBRID_TILT_ONLY (pan figé, tilt seulement)
//           -> MODE_NORMAL -> ...
//   - Envoyer les angles filtrés vers:
//        • un module ELRS par CRSF, ou
//        • un récepteur ESP-NOW (AtomRC RX)
//
// Conventions d'angles (pour l'humain, autour du centre):
//   - PAN (gauche ↔ droite) :
//        panDeg ≈ 90°  → caméra en face
//        panDeg < 90°  → caméra vers la gauche (≈ -Δ°)
//        panDeg > 90°  → caméra vers la droite (≈ +Δ°)
//
//   - TILT (haut ↔ bas) sur ce gimbal précis :
//        tiltDeg ≈ 90°  → horizon
//        tiltDeg plus petit  → caméra vers le ciel (tilt "up", +Δ° logique)
//        tiltDeg plus grand  → caméra vers le sol (tilt "down", -Δ° logique)
//
// IMPORTANT : ce fichier côté "TX" ne pilote les servos locaux (panServo/tiltServo)
//             QUE si l'ESP est en MODE_WIRED_PWM (bench). En vol normal
//             on est en CRSF/ELRS ou ESP-NOW, et c'est le RX qui pilote
//             les servos du gimbal.

#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <HardwareSerial.h>
#include <math.h>

// ============================================================================
// SECTION 0 — CONFIG UTILISATEUR
// ============================================================================

// MAC de l'ESP32 côté drone (RX) pour ESP-NOW
const uint8_t ESP_NOW_PEER_MAC[6] = {0xF4, 0x65, 0x0B, 0x49, 0x12, 0x94};

// Sélecteur 3 positions (interrupteur physique 3-états) :
//   POS1 (LOCAL)  -> MODE_WIRED_PWM : ESP32 pilote directement un gimbal local
//   POS2 (MILIEU) -> MODE_ESPNOW    : sortie radio ESP-NOW
//   POS3 (ELRS)   -> MODE_CRSF_ELRS : sortie CRSF vers module ELRS
const int MODE_SW_PIN_A = 32;
const int MODE_SW_PIN_B = 33;

// Petit "joystick" AtomRC : on n'utilise ici que la direction "UP"
// câblée comme un bouton vers GND (avec pull-up côté ESP)
const int RECENTER_BUTTON_PIN = 26;

// Temps max entre 2 clics pour les compter comme un double-clic
const unsigned long CLICK_TIMEOUT_MS = 350;

// Position tilt fixe pour le mode "Racing" (caméra légèrement vers le haut).
// Logique : +20° vers le haut par rapport à l'horizon.
// Sur ce gimbal : horizon ≈ 90°, monter = angle plus petit → 90 - 20 = 70.
const float LOGICAL_TILT_RACING_UP_DEG = +20.0f;
float RacingPauseTiltDeg = 90.0f - LOGICAL_TILT_RACING_UP_DEG;  // ≈ 70° servo

// ============================================================================
// SECTION 1 — PPM & SERVOS (TX local)
// ============================================================================

Servo panServo;
Servo tiltServo;

// Entrées / sorties physiques
const int PPM_INPUT_PIN = 25;    // PPM headtracker venant des Skyzone
const int PAN_PIN       = 18;    // servos locaux (option bench)
const int TILT_PIN      = 19;

// Espace PAN logique (correspond au servo côté RX/AtomRC)
// On reste sur 0..180 pour le servo, centre = 90.
const int PAN_MIN     = 0;       // ≈ -90° logique (plein gauche)
const int PAN_CENTER  = 90;
const int PAN_MAX     = 180;     // ≈ +90° logique (plein droite)

// Espace TILT logique (adapter si besoin à ton gimbal)
// Ici on décide : 30° = haut, 150° = bas, centre horizon = 90°.
const int TILT_MIN    = 30;      // haut (caméra vers le ciel)
const int TILT_CENTER = 90;      // horizon
const int TILT_MAX    = 150;     // bas  (caméra vers le sol)

// PPM brut : on ne lit que les ~6 premiers canaux
volatile unsigned long ppmChannels[8] = {
  1500,1500,1500,1500,1500,1500,1500,1500
};
volatile int   ppmChannelIndex   = 0;
volatile bool  ppmFrameComplete  = false;
volatile unsigned long lastValidFrame = 0;

// Aides debug PPM (ISR)
volatile unsigned long lastPulseDur = 0;
volatile unsigned long isrHits      = 0;
volatile int           lastPpmIndex = 0;

// PPM timing (µs)
const unsigned long PPM_PULSE_MIN  = 900;
const unsigned long PPM_PULSE_MAX  = 2100;
const unsigned long PPM_FRAME_GAP  = 4000;
const unsigned long PPM_TIMEOUT    = 1000;

// Position logique actuelle / cible (en degrés servo)
float currentPan  = PAN_CENTER;
float currentTilt = TILT_CENTER;
float targetPan   = PAN_CENTER;
float targetTilt  = TILT_CENTER;

// Deadband angulaire (ici, inutilisé, mais gardé pour expérimentation)
int deadband = 0;

// ============================================================================
// SECTION 2 — PHASES DE DÉMARRAGE & AUTO-CENTER
// ============================================================================

enum TrackerPhase {
  PHASE_BOOT_HOLD = 0,       // au boot: on force le centre, on ignore le PPM
  PHASE_WAIT_STABLE_PPM = 1, // on attend un PPM stable pour auto-center
  PHASE_TRACKING      = 2    // tracking normal
};

TrackerPhase trackerPhase = PHASE_BOOT_HOLD;

// timestamps
unsigned long bootTime = 0;

// Durée pendant laquelle on reste au centre après boot
const unsigned long STARTUP_HOLD_MS      = 500;   // 0.5s

// Durée de PPM stable avant de lancer l'auto-center initial
const unsigned long PPM_STABLE_CENTER_MS = 800;   // ~0.8s

// Premier moment où on a vu un PPM "valide"
unsigned long firstStablePpmTime = 0;

// Auto-center déjà fait ?
bool autoCenterDone = false;

// ============================================================================
// SECTION 3 — OFFSETS DYNAMIQUES (recenterAtCurrentPose)
// ============================================================================
//
// Les lunettes donnent un PAN/TILT absolu dans leur référentiel interne.
// On veut pouvoir dire : "ce que je regarde MAINTENANT = 0° / horizon".
// On le fait en stockant un offset appliqué à tous les angles ensuite.

float lastRawPanDeg  = PAN_CENTER;   // dernier PAN brut (avant offset)
float lastRawTiltDeg = TILT_CENTER;  // dernier TILT brut (avant offset)
bool  lastRawValid   = false;

// Offsets dynamiques en degrés :
//   angle_affiché = angle_brut + offset
float panOffsetDeg  = 0.0f;
float tiltOffsetDeg = 0.0f;

// Frame lock global (désactivé dans la logique, mais gardé pour plus tard)
bool frameLocked   = false;
int  lockedPanDeg  = PAN_CENTER;
int  lockedTiltDeg = TILT_CENTER;

// ============================================================================
// SECTION 4 — MODES (NORMAL / RACING / HYBRID) & BOUTON
// ============================================================================

enum TrackerMode {
  MODE_NORMAL = 0,        // Pan+Tilt suivent les lunettes
  MODE_RACING_FIXED,      // Pan fixe 0°, Tilt fixe (légèrement up)
  MODE_HYBRID_TILT_ONLY   // Pan figé, Tilt suit les lunettes
};

TrackerMode trackerMode = MODE_NORMAL;

// Gestion des clics sur le bouton (UP du mini-joystick)
unsigned long lastClickTime = 0;
int           clickCount    = 0;

// Pendant que le bouton est maintenu appuyé, on force la caméra au centre.
// Au relâchement : on recalcule l'offset pour que la pose actuelle des lunettes
// devienne le nouveau "0° / horizon".
bool centerHoldActive = false;

// ============================================================================
// SECTION 5 — FILTRE GAIN + SKIP (PAN/TILT)
// ============================================================================
//
// L'idée : plus la caméra est loin de la cible, plus elle doit bouger vite.
// Quand elle s'approche, elle doit ralentir pour ne pas osciller.
// On utilise un petit modèle linéaire par plage d'angle.
//
// Hypothèse : la boucle "effective" (PPM+loop) tourne autour de 50 Hz
const float BASE_PULSES_PER_SECOND = 50.0f;

// Une plage de configuration Gain/Skip
struct GainSkipRange {
  float angleMin;     // en degrés : seuil à partir duquel on commence à accélérer
  float angleMax;     // en degrés : au-delà de ce seuil on est "plein gaz"
  float gainLow;      // gain appliqué quand erreur = angleMin
  float gainHigh;     // gain appliqué quand erreur = angleMax
  float skipLowPps;   // mises à jour par seconde quand erreur = angleMin
  float skipHighPps;  // mises à jour par seconde quand erreur = angleMax
};

// PAN : entre 2° et 10° d’erreur
//   - en dessous de 2° : tout doux
//   - au dessus de 10° : rapide, suit quasiment instantanément
const GainSkipRange PAN_RANGE_1 = {
  2.0f,   // angleMin
  10.0f,  // angleMax
  0.04f,  // gainLow : 4% de l'erreur par mise à jour
  1.0f,   // gainHigh : 100% de l'erreur => on colle à la cible
  2.0f,   // skipLowPps : 2 mises à jour/s quand erreur petite
  50.0f   // skipHighPps: 50 mises à jour/s quand erreur grande
};

// TILT : entre 2° et 5° d’erreur (plus doux car très visible)
const GainSkipRange TILT_RANGE_1 = {
  2.0f,   // angleMin
  5.0f,   // angleMax
  0.01f,  // gainLow : 1% de l'erreur par mise à jour
  1.0f,   // gainHigh : 100% quand très loin
  30.0f,  // skipLowPps : 30 mises à jour/s quand erreur petite
  50.0f   // skipHighPps: 50 mises à jour/s quand erreur grande
};

// Convertir "mises à jour par seconde" → "nombre de boucles à sauter"
uint16_t pulsesPerSecondToSkipFrames(float pulsesPerSecond) {
  if (pulsesPerSecond <= 0.0f) pulsesPerSecond = 1.0f;
  if (pulsesPerSecond > BASE_PULSES_PER_SECOND)
    pulsesPerSecond = BASE_PULSES_PER_SECOND;

  float framesPerUpdate = BASE_PULSES_PER_SECOND / pulsesPerSecond;
  if (framesPerUpdate < 1.0f) framesPerUpdate = 1.0f;

  return (uint16_t)(framesPerUpdate + 0.5f); // arrondi
}

// Calculer gain + skip pour une erreur d'angle donnée
void computeGainAndSkip(const GainSkipRange& cfg,
                        float angleAbs,
                        float& outGain,
                        uint16_t& outSkipFrames)
{
  if (angleAbs <= cfg.angleMin) {
    outGain       = cfg.gainLow;
    outSkipFrames = pulsesPerSecondToSkipFrames(cfg.skipLowPps);
    return;
  }
  if (angleAbs >= cfg.angleMax) {
    outGain       = cfg.gainHigh;
    outSkipFrames = pulsesPerSecondToSkipFrames(cfg.skipHighPps);
    return;
  }

  // interpolation linéaire entre les deux
  float t = (angleAbs - cfg.angleMin) / (cfg.angleMax - cfg.angleMin);  // 0..1

  float gain    = cfg.gainLow    + t * (cfg.gainHigh    - cfg.gainLow);
  float skipPps = cfg.skipLowPps + t * (cfg.skipHighPps - cfg.skipLowPps);

  outGain       = gain;
  outSkipFrames = pulsesPerSecondToSkipFrames(skipPps);
}

// État interne du filtre pour PAN
uint32_t panPulseCounter  = 0;
uint16_t panSkipFrames    = 1;
float    panDynamicGain   = 1.0f;

// État interne du filtre pour TILT
uint32_t tiltPulseCounter = 0;
uint16_t tiltSkipFrames   = 1;
float    tiltDynamicGain  = 1.0f;

// ============================================================================
// SECTION 6 — MODES DE SORTIE
// ============================================================================

enum OutputMode {
  MODE_WIRED_PWM = 0,   // bench: ESP32 pilote un gimbal local
  MODE_CRSF_ELRS = 1,   // sortie CRSF vers module ELRS
  MODE_ESPNOW    = 2    // sortie ESP-NOW vers RX
};

OutputMode outputMode = MODE_CRSF_ELRS;

// UART CRSF vers module ELRS
HardwareSerial elrsSerial(1);
const int ELRS_UART_TX_PIN = 17;
const int ELRS_UART_RX_PIN = -1;
const unsigned long ELRS_UART_BAUD = 420000;
bool elrsReady = false;

// Constantes CRSF
const uint8_t  CRSF_DEVICE_ADDRESS        = 0xC8;
const uint8_t  CRSF_FRAMETYPE_RC_CHANNELS = 0x16;
const uint8_t  CRSF_RC_PAYLOAD_LEN        = 22;
const uint8_t  CRSF_RC_CHANNEL_COUNT      = 16;
const uint16_t CRSF_CHANNEL_MID           = 992;

// ESP-NOW (TX)
bool    espNowReady = false;
uint8_t espNowPeer[6] = {0};
const unsigned long WIRELESS_SEND_INTERVAL_MS = 10;
unsigned long lastWirelessSend = 0;

// Packet radio commun ESP-NOW (et debug)
struct __attribute__((packed)) ControlPacket {
  uint16_t header;   // 0xA55A
  uint16_t pan;      // degrés filtrés (currentPan)
  uint16_t tilt;     // degrés filtrés (currentTilt)
  uint16_t flags;    // bit0 = PPM valide
  uint16_t checksum;
};

float wirelessPan  = PAN_CENTER;
float wirelessTilt = TILT_CENTER;

// ============================================================================
// PROTOTYPES
// ============================================================================
void processSerialCommand(String command);
void testServoMovement();
void IRAM_ATTR ppmInterrupt();
void setOutputMode(OutputMode newMode, bool announce);
void handleOutputs(bool ppmValid);
void transmitWirelessTargets(bool ppmValid);
void initEspNow();
void initElrsUart();
const char* modeName(OutputMode mode);
uint16_t packetChecksum(const ControlPacket& pkt);
void processPPMFrame();
void updateFilterGainSkip();
void updateServos();
void sendCrsfRcFrame(int panDeg, int tiltDeg, bool valid);
uint8_t crsfCalcCRC(const uint8_t* data, uint8_t len);
void encodeCrsfChannels(const uint16_t* channels, uint8_t* payload);
uint16_t microsecondsToCrsf(int microseconds);
int degreesToMicroseconds(int degrees, int degMin, int degMax);
OutputMode readHardwareMode();
void recenterAtCurrentPose();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  bootTime = millis();

  memcpy(espNowPeer, ESP_NOW_PEER_MAC, sizeof(espNowPeer));

  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  currentPan  = constrain(PAN_CENTER,  PAN_MIN,  PAN_MAX);
  currentTilt = constrain(TILT_CENTER, TILT_MIN, TILT_MAX);
  targetPan   = currentPan;
  targetTilt  = currentTilt;
  panServo.write((int)currentPan);
  tiltServo.write((int)currentTilt);

  Serial.println("TX: Head Tracker (goggles)");

  // PPM input
  pinMode(PPM_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), ppmInterrupt, CHANGE);

  // mode switch pins
  pinMode(MODE_SW_PIN_A, INPUT_PULLUP);
  pinMode(MODE_SW_PIN_B, INPUT_PULLUP);

  // bouton / joystick UP
  pinMode(RECENTER_BUTTON_PIN, INPUT_PULLUP);

  // lecture du mode de sortie au boot
  outputMode = readHardwareMode();
  setOutputMode(outputMode, true);

  Serial.printf("ESP-NOW peer MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                espNowPeer[0], espNowPeer[1], espNowPeer[2],
                espNowPeer[3], espNowPeer[4], espNowPeer[5]);

  Serial.println("Commands: center, debug, test, deadband X");
  delay(200);
}

// ============================================================================
// AUTO-CENTER : recale les offsets pour que la pose actuelle des lunettes
// devienne le nouveau centre PAN/TILT.
// ============================================================================
void recenterAtCurrentPose() {
  if (!lastRawValid) {
    Serial.println("[AUTO-CENTER] ignored (no valid raw yet)");
    return;
  }

  // Offsets : on veut que rawPan/rawTilt actuels deviennent PAN_CENTER/TILT_CENTER
  panOffsetDeg  = (float)PAN_CENTER  - lastRawPanDeg;
  tiltOffsetDeg = (float)TILT_CENTER - lastRawTiltDeg;

  targetPan   = PAN_CENTER;
  targetTilt  = TILT_CENTER;
  currentPan  = PAN_CENTER;
  currentTilt = TILT_CENTER;

  frameLocked   = false;
  lockedPanDeg  = PAN_CENTER;
  lockedTiltDeg = TILT_CENTER;

  Serial.printf("[AUTO-CENTER] rawPan=%.1f rawTilt=%.1f -> offsetPan=%.1f offsetTilt=%.1f\n",
                lastRawPanDeg, lastRawTiltDeg, panOffsetDeg, tiltOffsetDeg);
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  // 1) hardware switch (permet de changer de mode de sortie en vol)
  static OutputMode lastHwMode = outputMode;
  OutputMode hwMode = readHardwareMode();
  if (hwMode != lastHwMode) {
    lastHwMode = hwMode;
    setOutputMode(hwMode, true);
  }

  // 2) Frame PPM prête ?
  if (ppmFrameComplete) {
    processPPMFrame();
    ppmFrameComplete = false;
    lastValidFrame = millis();
  }

  // 3) état PPM brut (présent / absent)
  unsigned long now = millis();
  bool ppmValidRaw = (now - lastValidFrame) <= PPM_TIMEOUT;
  bool ppmValid    = ppmValidRaw;

  // 4) Phases de démarrage / auto-center initial
  switch (trackerPhase) {

    case PHASE_BOOT_HOLD:
      // Au tout début : centre fixe, on ne considère pas le PPM
      targetPan  = PAN_CENTER;
      targetTilt = TILT_CENTER;
      ppmValid   = false;

      if (now - bootTime > STARTUP_HOLD_MS) {
        trackerPhase = PHASE_WAIT_STABLE_PPM;
        firstStablePpmTime = 0;
        Serial.println("[TRACKER] -> PHASE_WAIT_STABLE_PPM");
      }
      break;

    case PHASE_WAIT_STABLE_PPM:
      targetPan  = PAN_CENTER;
      targetTilt = TILT_CENTER;
      ppmValid   = false;

      if (ppmValidRaw) {
        if (firstStablePpmTime == 0) {
          firstStablePpmTime = now;
        }
        if (!autoCenterDone &&
            (now - firstStablePpmTime >= PPM_STABLE_CENTER_MS) &&
            lastRawValid) {

          // Auto-center initial : centre lunettes -> 90/90
          recenterAtCurrentPose();
          autoCenterDone = true;
          trackerPhase   = PHASE_TRACKING;
          Serial.println("[TRACKER] auto-center done -> PHASE_TRACKING");
        }
      } else {
        firstStablePpmTime = 0;
      }
      break;

    case PHASE_TRACKING:
      ppmValid = ppmValidRaw;
      break;
  }

  // Failsafe : si on perd le PPM pendant le tracking → revenir au centre
  if (trackerPhase == PHASE_TRACKING && !ppmValidRaw) {
    targetPan  = PAN_CENTER;
    targetTilt = TILT_CENTER;
  }

  // ➜ Forcer le centre tant que le bouton est maintenu
  if (centerHoldActive) {
    targetPan  = PAN_CENTER;
    targetTilt = TILT_CENTER;
  }

  // 5) Gestion du bouton (UP du mini-joystick AtomRC)
  {
    static int btnPrev = HIGH;
    int btn = digitalRead(RECENTER_BUTTON_PIN);
    unsigned long nowMs = millis();

    // FRONT DESCENDANT = bouton appuyé
    if (btn == LOW && btnPrev == HIGH) {
      Serial.println("[BTN] pressed");
      centerHoldActive = true;

      // Tant qu'appuyé, on reste au centre (voir plus haut)
      targetPan  = PAN_CENTER;
      targetTilt = TILT_CENTER;

      // Gestion de séquence de clics
      if (nowMs - lastClickTime > CLICK_TIMEOUT_MS) {
        clickCount = 0;
      }
    }

    // FRONT MONTANT = bouton relâché
    if (btn == HIGH && btnPrev == LOW) {
      Serial.println("[BTN] released");

      // 1) on recalcule les offsets (nouveau "zéro" lunettes)
      if (centerHoldActive) {
        recenterAtCurrentPose();
        centerHoldActive = false;
      }

      // 2) on compte ce clic (pour le double-clic)
      clickCount++;
      lastClickTime = nowMs;
    }

    btnPrev = btn;

    // 3) Interprétation de la séquence (simple / double clic)
    if (clickCount > 0 && (nowMs - lastClickTime > CLICK_TIMEOUT_MS)) {

      if (clickCount == 1) {
        // Rien de plus à faire : center + recenter déjà effectués au relâchement
        Serial.println("[CLICK] single -> center + recenter");
      }
      else if (clickCount == 2) {
        // Double-clic : cycle des modes
        if (trackerMode == MODE_NORMAL) {
          trackerMode = MODE_RACING_FIXED;
        } else if (trackerMode == MODE_RACING_FIXED) {
          trackerMode = MODE_HYBRID_TILT_ONLY;
          lockedPanDeg = (int)currentPan;   // on fige le pan ici
        } else {
          trackerMode = MODE_NORMAL;
        }

        Serial.printf("[CLICK] double -> mode now = %d (0=NORMAL,1=RACING,2=TILT_ONLY)\n",
                      (int)trackerMode);
      } else {
        Serial.printf("[CLICK] %d clicks -> ignored (only 1 or 2 used)\n", clickCount);
      }

      clickCount = 0;
    }
  }

  // 6) Application des modes spéciaux sur les cibles (sauf phases boot/wait)
  if (trackerPhase == PHASE_TRACKING) {
    if (trackerMode == MODE_RACING_FIXED) {
      // Mode "racing": pan fixe devant, tilt figé légèrement vers le haut
      targetPan  = PAN_CENTER;
      targetTilt = RacingPauseTiltDeg;
    } else if (trackerMode == MODE_HYBRID_TILT_ONLY) {
      // Mode hybride: PAN figé là où il était au moment de l'activation,
      // TILT suit normalement les lunettes (via processPPMFrame + offsets).
      targetPan = lockedPanDeg;
    }
  }

  // 7) Filtre Gain + Skip (PAN / TILT)
  updateFilterGainSkip();

  // 8) Sorties (ESP-NOW / CRSF / PWM local)
  handleOutputs(ppmValid);

  // 9) CLI série
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    processSerialCommand(command);
  }

  // 10) Debug PPM (fréquence ISR)
  static unsigned long lastDbg = 0;
  if (now - lastDbg > 1000) {
    lastDbg = now;

    noInterrupts();
    unsigned long hits = isrHits;
    unsigned long dur  = lastPulseDur;
    int           idx  = lastPpmIndex;
    isrHits = 0;
    interrupts();

    Serial.printf("[PPM DBG] isrHits=%lu lastDur=%lu us lastIdx=%d\n",
                  hits, dur, idx);
  }

  delay(20);
}

// ============================================================================
// ISR PPM : reconstruction des canaux depuis la trame PPM
// ============================================================================
void IRAM_ATTR ppmInterrupt() {
  static unsigned long lastPulseTime = 0;
  unsigned long currentTime = micros();

  bool pinState = digitalRead(PPM_INPUT_PIN);

  if (pinState == HIGH) {
    lastPulseTime = currentTime;   // début du pulse
  } else {
    unsigned long pulseDuration = currentTime - lastPulseTime;

    lastPulseDur = pulseDuration;
    isrHits++;
    lastPpmIndex = ppmChannelIndex;

    if (pulseDuration > PPM_FRAME_GAP) {
      // nouveau frame
      ppmChannelIndex = 0;
    } else if (pulseDuration >= PPM_PULSE_MIN &&
               pulseDuration <= PPM_PULSE_MAX) {
      if (ppmChannelIndex < 8) {
        ppmChannels[ppmChannelIndex] = pulseDuration;
        ppmChannelIndex++;
        if (ppmChannelIndex >= 6) {
          ppmFrameComplete = true;
        }
      }
    }
  }
}

// ============================================================================
// Utilisation de la frame PPM (PAN/TILT) + offsets
// (FRAME LOCK GLOBAL DESACTIVE ici pour simplifier la mise au point).
// ============================================================================
void processPPMFrame() {
  // Mapping Skyzone : CH6 = PAN, CH5 = TILT (adapter si besoin)
  unsigned long panPulse  = ppmChannels[5];  // CH6 -> PAN
  unsigned long tiltPulse = ppmChannels[4];  // CH5 -> TILT

  if (panPulse < PPM_PULSE_MIN  || panPulse > PPM_PULSE_MAX)  panPulse  = 1500;
  if (tiltPulse < PPM_PULSE_MIN || tiltPulse > PPM_PULSE_MAX) tiltPulse = 1500;

  // 1) PPM -> degrés bruts
  int rawPanDeg  = map(panPulse,  1000, 2000, PAN_MIN,  PAN_MAX);
  int rawTiltDeg = map(tiltPulse, 1000, 2000, TILT_MIN, TILT_MAX);

  lastRawPanDeg  = rawPanDeg;
  lastRawTiltDeg = rawTiltDeg;
  lastRawValid   = true;

  // 2) Application des offsets (auto-center dynamique)
  int adjPanDeg  = rawPanDeg  + (int)roundf(panOffsetDeg);
  int adjTiltDeg = rawTiltDeg + (int)roundf(tiltOffsetDeg);

  adjPanDeg  = constrain(adjPanDeg,  PAN_MIN,  PAN_MAX);
  adjTiltDeg = constrain(adjTiltDeg, TILT_MIN, TILT_MAX);

  // 3) Pas de frame-lock : la caméra suit directement les angles ajustés
  targetPan  = adjPanDeg;
  targetTilt = adjTiltDeg;

  static int frameCount = 0;
  if (frameCount++ % 50 == 0) {
    Serial.printf(
      "PPM: pan=%luus raw=%d adj=%d tgt=%d | tilt=%luus raw=%d adj=%d tgt=%d\n",
      panPulse, rawPanDeg, adjPanDeg, (int)targetPan,
      tiltPulse, rawTiltDeg, adjTiltDeg, (int)targetTilt
    );
  }
}

// ============================================================================
// Filtre Gain + Skip (PAN / TILT)
// ============================================================================
void updateFilterGainSkip() {

  // ================== PAN ==================
  {
    float panErrorAbs = fabs(targetPan - currentPan);

    // Calcul du gain & skip en fonction de l'erreur
    computeGainAndSkip(PAN_RANGE_1,
                       panErrorAbs,
                       panDynamicGain,
                       panSkipFrames);

    panPulseCounter++;

    // On n'applique la correction que toutes les "panSkipFrames" boucles
    if (panPulseCounter >= panSkipFrames) {
      panPulseCounter = 0;

      float panError = targetPan - currentPan;
      currentPan += panError * panDynamicGain;
      currentPan = constrain(currentPan, PAN_MIN, PAN_MAX);
    }

    static uint32_t panDebugCounter = 0;
    if (++panDebugCounter >= 50) {
      panDebugCounter = 0;
      Serial.printf("[PAN] err=%.1f gain=%.3f skipFrames=%u cur=%d tgt=%d\n",
                    panErrorAbs, panDynamicGain, panSkipFrames,
                    (int)currentPan, (int)targetPan);
    }
  }

  // ================== TILT ==================
  {
    float tiltErrorAbs = fabs(targetTilt - currentTilt);

    computeGainAndSkip(TILT_RANGE_1,
                       tiltErrorAbs,
                       tiltDynamicGain,
                       tiltSkipFrames);

    // Pour le tilt, on a choisi de ne PAS sauter de frames (plus fluide)
    tiltSkipFrames = 1;
    tiltPulseCounter++;

    if (tiltPulseCounter >= tiltSkipFrames) {
      tiltPulseCounter = 0;

      float tiltError = targetTilt - currentTilt;
      currentTilt += tiltError * tiltDynamicGain;
      currentTilt = constrain(currentTilt, TILT_MIN, TILT_MAX);
    }

    static uint32_t tiltDebugCounter = 0;
    if (++tiltDebugCounter >= 50) {
      tiltDebugCounter = 0;
      Serial.printf("[TILT] err=%.1f gain=%.3f cur=%d tgt=%d\n",
                    tiltErrorAbs, tiltDynamicGain,
                    (int)currentTilt, (int)targetTilt);
    }
  }
}

// ============================================================================
// PWM local (bench uniquement)
// ============================================================================
void updateServos() {
  panServo.write((int)currentPan);
  tiltServo.write((int)currentTilt);
}

// ============================================================================
// CLI série & tests
// ============================================================================
void processSerialCommand(String command) {
  command.toLowerCase();
  if (command == "center") {
    recenterAtCurrentPose();
    trackerMode = MODE_NORMAL;
    Serial.println("Center + MODE_NORMAL");
  } else if (command == "debug") {
    for (int i=0;i<8;i++) {
      Serial.printf("CH%d %lu\n", i+1, ppmChannels[i]);
    }
    Serial.printf("Mode sortie: %s | TrackerMode: %d | Target: %d %d\n",
                  modeName(outputMode), (int)trackerMode,
                  (int)targetPan, (int)targetTilt);
  } else if (command == "test") {
    testServoMovement();
  } else if (command.startsWith("deadband")) {
    int v = command.substring(command.indexOf(' ')+1).toInt();
    deadband = v;
    Serial.printf("Deadband=%d\n", deadband);
  } else {
    Serial.println("Unknown");
  }
}

void testServoMovement() {
  // Petit test local des servos (utile en MODE_WIRED_PWM)
  for (int pos = PAN_CENTER; pos <= PAN_MAX; pos += 10) {
    panServo.write(pos); delay(200);
  }
  for (int pos = PAN_MAX; pos >= PAN_MIN; pos -= 10) {
    panServo.write(pos); delay(200);
  }
  for (int pos = PAN_MIN; pos <= PAN_CENTER; pos += 10) {
    panServo.write(pos); delay(200);
  }
  for (int pos = TILT_CENTER; pos <= TILT_MAX; pos += 6) {
    tiltServo.write(pos); delay(200);
  }
  for (int pos = TILT_MAX; pos >= TILT_MIN; pos -= 6) {
    tiltServo.write(pos); delay(200);
  }
  for (int pos = TILT_MIN; pos <= TILT_CENTER; pos += 6) {
    tiltServo.write(pos); delay(200);
  }
}

// ============================================================================
// Sorties radio (ESP-NOW / CRSF / PWM local)
// ============================================================================
void handleOutputs(bool ppmValid) {
  if (outputMode == MODE_WIRED_PWM) {
    updateServos();
    return;
  }
  if (millis() - lastWirelessSend >= WIRELESS_SEND_INTERVAL_MS) {
    transmitWirelessTargets(ppmValid);
    lastWirelessSend = millis();
  }
}

void transmitWirelessTargets(bool ppmValid) {
  // On envoie la valeur déjà filtrée (currentPan/currentTilt)
  wirelessPan  = currentPan;
  wirelessTilt = currentTilt;

  ControlPacket pkt;
  pkt.header = 0xA55A;
  pkt.pan    = constrain(static_cast<int>(wirelessPan  + 0.5f), PAN_MIN,  PAN_MAX);
  pkt.tilt   = constrain(static_cast<int>(wirelessTilt + 0.5f), TILT_MIN, TILT_MAX);
  pkt.flags  = ppmValid ? 0x0001 : 0x0000;
  pkt.checksum = packetChecksum(pkt);

  if (outputMode == MODE_ESPNOW) {
    if (!espNowReady) initEspNow();
    if (espNowReady) {
      esp_err_t res = esp_now_send(espNowPeer, reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
      if (res != ESP_OK) {
        Serial.printf("[ESP-NOW] send error %d\n", res);
      }
    }
  } else if (outputMode == MODE_CRSF_ELRS) {
    sendCrsfRcFrame(pkt.pan, pkt.tilt, ppmValid);
  }
}

void setOutputMode(OutputMode newMode, bool announce) {
  outputMode = newMode;
  switch (outputMode) {
    case MODE_CRSF_ELRS:
      initElrsUart();
      break;
    case MODE_ESPNOW:
      initEspNow();
      break;
    case MODE_WIRED_PWM:
    default:
      break;
  }
  if (announce) {
    Serial.printf("Output mode set to %s\n", modeName(outputMode));
  }
}

void initElrsUart() {
  if (elrsReady) return;
  if (ELRS_UART_TX_PIN < 0) {
    Serial.println("ELRS UART TX pin not defined!");
    return;
  }
  elrsSerial.begin(ELRS_UART_BAUD, SERIAL_8N1, ELRS_UART_RX_PIN, ELRS_UART_TX_PIN);
  elrsReady = true;
  Serial.println("ELRS CRSF UART (TX) ready on GPIO 17.");
}

void initEspNow() {
  if (espNowReady) return;
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, espNowPeer, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer!");
  }
  espNowReady = true;
  Serial.println("ESP-NOW link (TX) ready.");
}

const char* modeName(OutputMode mode) {
  switch (mode) {
    case MODE_WIRED_PWM: return "Wired PWM";
    case MODE_CRSF_ELRS: return "CRSF / ELRS";
    case MODE_ESPNOW:    return "ESP-NOW";
    default:             return "Unknown";
  }
}

uint16_t packetChecksum(const ControlPacket& pkt) {
  return pkt.header ^ pkt.pan ^ pkt.tilt ^ pkt.flags ^ 0x55AA;
}

// ============================================================================
// CRSF helpers
// ============================================================================
void sendCrsfRcFrame(int panDeg, int tiltDeg, bool valid) {
  if (!elrsReady) initElrsUart();
  if (!elrsReady) return;

  uint16_t channels[CRSF_RC_CHANNEL_COUNT];
  for (int i = 0; i < CRSF_RC_CHANNEL_COUNT; ++i) {
    channels[i] = CRSF_CHANNEL_MID;
  }

  if (valid) {
    int panMicro  = degreesToMicroseconds(panDeg,  PAN_MIN,  PAN_MAX);
    int tiltMicro = degreesToMicroseconds(tiltDeg, TILT_MIN, TILT_MAX);
    channels[0] = microsecondsToCrsf(panMicro);
    channels[1] = microsecondsToCrsf(tiltMicro);
  }

  uint8_t payload[CRSF_RC_PAYLOAD_LEN] = {0};
  encodeCrsfChannels(channels, payload);

  const uint8_t frameLen = 1 + CRSF_RC_PAYLOAD_LEN + 1;
  uint8_t frame[2 + frameLen];
  frame[0] = CRSF_DEVICE_ADDRESS;
  frame[1] = frameLen;
  frame[2] = CRSF_FRAMETYPE_RC_CHANNELS;
  memcpy(&frame[3], payload, CRSF_RC_PAYLOAD_LEN);
  frame[3 + CRSF_RC_PAYLOAD_LEN] = crsfCalcCRC(&frame[2], 1 + CRSF_RC_PAYLOAD_LEN);

  elrsSerial.write(frame, sizeof(frame));
}

uint8_t crsfCalcCRC(const uint8_t* data, uint8_t len) {
  const uint8_t POLY = 0xD5;
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x80) crc = (crc << 1) ^ POLY;
      else            crc <<= 1;
    }
  }
  return crc;
}

void encodeCrsfChannels(const uint16_t* channels, uint8_t* payload) {
  uint32_t bitBuffer    = 0;
  uint8_t  bitsInBuffer = 0;
  uint8_t  byteIndex    = 0;
  for (int ch = 0; ch < CRSF_RC_CHANNEL_COUNT; ++ch) {
    bitBuffer    |= ((uint32_t)channels[ch] & 0x7FF) << bitsInBuffer;
    bitsInBuffer += 11;
    while (bitsInBuffer >= 8) {
      payload[byteIndex++] = bitBuffer & 0xFF;
      bitBuffer   >>= 8;
      bitsInBuffer -= 8;
    }
  }
  if (bitsInBuffer > 0 && byteIndex < CRSF_RC_PAYLOAD_LEN) {
    payload[byteIndex++] = bitBuffer & 0xFF;
  }
  while (byteIndex < CRSF_RC_PAYLOAD_LEN) {
    payload[byteIndex++] = 0;
  }
}

uint16_t microsecondsToCrsf(int microseconds) {
  microseconds = constrain(microseconds, 988, 2012);
  long value   = map(microseconds, 988, 2012, 172, 1811);
  return (uint16_t)constrain(value, 172L, 1811L);
}

int degreesToMicroseconds(int degrees, int degMin, int degMax) {
  degrees = constrain(degrees, degMin, degMax);
  long micro = map(degrees, degMin, degMax, 1000, 2000);
  return (int)constrain(micro, 1000L, 2000L);
}

// ============================================================================
// Lecture de l'inter 3 positions (mode sortie)
// ============================================================================
OutputMode readHardwareMode() {
  bool aLow = (digitalRead(MODE_SW_PIN_A) == LOW);
  bool bLow = (digitalRead(MODE_SW_PIN_B) == LOW);

  if (aLow && !bLow) return MODE_WIRED_PWM;  // bench local
  if (!aLow && bLow) return MODE_CRSF_ELRS;  // ELRS / CRSF
  return MODE_ESPNOW;                        // middle => ESP-NOW
}
