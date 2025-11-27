// RX_Headtracker.ino  (Gemini by ymanda + commentaires)
// ------------------------------------------------------
// Côté drone / gimbal (AtomRC).
//
// Rôle de ce sketch (RX):
//   - Recevoir les angles PAN/TILT filtrés envoyés par le TX (goggles)
//       • EN PRIORITÉ par CRSF / ELRS (UART)
//       • En secours par ESP-NOW (si CRSF est perdu)
//   - Lisser la commande (smoothing) pour des mouvements propres
//   - Appliquer un offset mécanique sur le PAN (PAN_OFFSET_DEG)
//   - Appliquer éventuellement une inversion PAN/TILT pour s’aligner
//     avec la cinématique réelle du gimbal AtomRC
//   - Piloter directement les deux servos du gimbal : panServo / tiltServo
//
// Conventions d’angles (logiques, côté RX):
//   - PAN (gauche ↔ droite) : 0..180, centre à 90
//       0   ≈ plein gauche         (~ -90° logique)
//       90  ≈ caméra en face       (0° logique)
//       180 ≈ plein droite         (~ +90° logique)
//
//   - TILT (haut ↔ bas) : 40..120, centre à 90
//       30  ≈ caméra vers le ciel  ("up")   (~ +50° logique)
//       90  ≈ horizon              (0° logique)
//       150 ≈ caméra vers le sol   ("down") (~ -30° logique)
//
//   Le TX travaille sur 30..150 pour le tilt et envoie ses valeurs en CRSF.
//   Ici, on re-map ces valeurs dans une plage mécaniquement plus safe,
//   adaptée au gimbal (40..120).
//
// NOTE : on garde volontairement le même protocole de paquet que le TX
//        pour ESP-NOW (ControlPacket struct).

#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <HardwareSerial.h>

// ============================================================================
// SECTION A — Servos & paramètres de base
// ============================================================================

Servo panServo;
Servo tiltServo;

// --- INVERSION & OFFSET PAN/TILT ---
//
// INVERT_PAN :  true  si le gimbal tourne à l’envers quand on regarde les angles
// INVERT_TILT:  true  si la caméra monte quand on lui demande de descendre, etc.
//
// PAN_OFFSET_DEG : correction mécanique simple. Exemple :
//   - Si quand on envoie 90° en PAN, la caméra regarde un peu à droite,
//     on peut mettre PAN_OFFSET_DEG = -10 pour recentrer matériellement.
//   - Ici on utilise +15° parce que ton montage regarde un peu trop à gauche.
const bool INVERT_PAN  = true;
const bool INVERT_TILT = false;

int PAN_OFFSET_DEG = 15;  // Offset mécanique sur PAN (en degrés servo)

// Sorties servos (AtomRC / ton câblage)
const int PAN_PIN       = 18;
const int TILT_PIN      = 19;

// Espace PAN — 0..180, centre 90 (identique à TX)
const int PAN_MIN    = 0;
const int PAN_CENTER = 90;
const int PAN_MAX    = 180;

// Espace TILT — adapté au gimbal (un peu plus serré que le TX)
//  40 ≈ haut (up), 120 ≈ bas (down), 90 ≈ horizon
const int TILT_CENTER = 90;
const int TILT_MIN    = 40;
const int TILT_MAX    = 120;

// État actuel et cibles (en degrés servo)
int currentPan  = PAN_CENTER;
int currentTilt = TILT_CENTER;
int targetPan   = PAN_CENTER;
int targetTilt  = TILT_CENTER;

// Smoothing côté RX :
//   - deadband : éventuellement utile plus tard
//   - smoothFactor : plus la valeur est grande, plus le servo se déplace doucement
int   deadband     = 1;
int   smoothFactor = 5;   // typiquement 4–8 donne des mouvements propres

// Lissage RF (pour ESP-NOW / CRSF)
// On lisse linkPan/linkTilt avant de les convertir en targetPan/targetTilt
const float LINK_SMOOTH_ALPHA   = 0.25f;    // 0.0 = très lisse, 1.0 = brut
const unsigned long LOG_INTERVAL_MS = 2000; // heartbeat toutes les 2 s

// ============================================================================
// SECTION B — RF sources (CRSF + ESP-NOW)
// ============================================================================

// ---------------- ELRS / CRSF UART (réception) ----------------
//
// On reçoit un flux CRSF depuis un module ELRS branché sur GPIO16.
// On ne fait QUE du RX (pas de TX) côté drone.
HardwareSerial elrsSerial(1);
const int ELRS_UART_TX_PIN = -1;   // non utilisé
const int ELRS_UART_RX_PIN = 16;   // ELRS -> ESP32
const unsigned long ELRS_UART_BAUD = 420000;
bool elrsReady = false;

// Constantes CRSF
const uint8_t  CRSF_DEVICE_ADDRESS        = 0xC8;
const uint8_t  CRSF_FRAMETYPE_RC_CHANNELS = 0x16;
const uint8_t  CRSF_RC_PAYLOAD_LEN        = 22;
const uint8_t  CRSF_RC_CHANNEL_COUNT      = 16;
const uint8_t  CRSF_MAX_FRAME_LEN         = 64;

// ---------------- ESP-NOW (réception) ----------------
//
// En secours si CRSF est perdu. Le TX envoie alors un ControlPacket
// identique à celui qu’il encode aussi vers ELRS (dans une autre forme).
bool espNowReady = false;

// Paquet radio — DOIT matcher exactement le TX
struct __attribute__((packed)) ControlPacket {
  uint16_t header;   // 0xA55A
  uint16_t pan;      // angle PAN filtré côté TX (currentPan)
  uint16_t tilt;     // angle TILT filtré côté TX (currentTilt)
  uint16_t flags;    // bit0 = PPM valide côté TX
  uint16_t checksum; // header ^ pan ^ tilt ^ flags ^ 0x55AA
};

uint16_t packetChecksum(const ControlPacket& pkt) {
  return pkt.header ^ pkt.pan ^ pkt.tilt ^ pkt.flags ^ 0x55AA;
}

// Angles RF lissés
float linkPan  = PAN_CENTER;
float linkTilt = TILT_CENTER;

// Source RF active
enum ActiveSource {
  SOURCE_NONE = 0,
  SOURCE_ESPNOW,
  SOURCE_CRSF
};

ActiveSource   activeSource      = SOURCE_NONE;
unsigned long  lastEspNowTime    = 0;
unsigned long  lastCrsfTime      = 0;
const unsigned long SOURCE_TIMEOUT_MS = 400;   // >400ms sans paquet => source perdue

// ============================================================================
// PROTOTYPES
// ============================================================================
void updateServos();
void processSerialCommand(String command);
void testServoMovement();

void initEspNowRX();
void initElrsUartRX();
const char* sourceName(ActiveSource s);

// Callback ESP-NOW (nouvelle signature core 3.x)
void onEspNowRecv(const esp_now_recv_info* info, const uint8_t* data, int len);

// CRSF helpers
void readCrsfPackets();
void applyCrsfChannels(const uint8_t* payload);
uint8_t crsfCalcCRC(const uint8_t* data, uint8_t len);
int crsfToMicroseconds(uint16_t value);
int microsecondsToDegrees(int microseconds, int degMin, int degMax);

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Servos gimbal
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);

  currentPan  = constrain(PAN_CENTER,  PAN_MIN,  PAN_MAX);
  currentTilt = constrain(TILT_CENTER, TILT_MIN, TILT_MAX);
  targetPan   = currentPan;
  targetTilt  = currentTilt;

  panServo.write(currentPan);
  tiltServo.write(currentTilt);

  // RF
  initEspNowRX();
  initElrsUartRX();
  activeSource = SOURCE_NONE;

  Serial.println("RX: HeadTracker gimbal (Gemini)");
  Serial.println("  CRSF/ELRS = PRIORITY");
  Serial.println("  ESP-NOW   = FALLBACK");
  Serial.println("  No local PPM on RX");
  Serial.println("Commands: center, debug, test");
  delay(200);
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();
  static unsigned long lastHeartbeat = 0;
  bool haveSignal = false;

  // 1) Lire les paquets CRSF (si présents)
  readCrsfPackets();

  // 2) Sélectionner source active
  //    CRSF a la priorité sur ESP-NOW.
  bool crsfAlive = (now - lastCrsfTime   <= SOURCE_TIMEOUT_MS);
  bool espAlive  = (now - lastEspNowTime <= SOURCE_TIMEOUT_MS);

  if (crsfAlive) {
    activeSource = SOURCE_CRSF;
  } else if (espAlive) {
    activeSource = SOURCE_ESPNOW;
  } else {
    activeSource = SOURCE_NONE;
  }

  // 3) Smoothing des servos vers targetPan/targetTilt
  int panErr  = targetPan  - currentPan;
  int tiltErr = targetTilt - currentTilt;

  if (abs(panErr) > 0) {
    currentPan += panErr / smoothFactor;
    if (abs(panErr) < smoothFactor) currentPan = targetPan;
  }
  if (abs(tiltErr) > 0) {
    currentTilt += tiltErr / smoothFactor;
    if (abs(tiltErr) < smoothFactor) currentTilt = targetTilt;
  }

  // 4) Si plus aucune source RF => recentre lentement
  if (activeSource == SOURCE_NONE) {
    targetPan  = PAN_CENTER;
    targetTilt = TILT_CENTER;
  } else {
    haveSignal = true;
  }

  // 5) Mise à jour des servos physiques
  updateServos();

  // 6) CLI série
  if (Serial.available()) {
    String cmd = Serial.readString();
    cmd.trim();
    cmd.toLowerCase();
    processSerialCommand(cmd);
  }

  // 7) Heartbeat debug périodique
  if (now - lastHeartbeat > LOG_INTERVAL_MS) {
    lastHeartbeat = now;
    Serial.printf("[HB] src=%s tgtPan=%d tgtTilt=%d curPan=%d curTilt=%d\n",
                  sourceName(activeSource),
                  targetPan, targetTilt,
                  currentPan, currentTilt);
    if (!haveSignal) {
      Serial.println("  No RF signal (centering).");
    }
  }

  delay(10);
}

// ============================================================================
// SECTION C — Pilotage des servos (application offset/inversion)
// ============================================================================
void updateServos() {
  // On part des angles "logiques" 0..180 suivis par RX
  int outPan  = currentPan;
  int outTilt = currentTilt;

  // 1) Offset mécanique PAN
  outPan += PAN_OFFSET_DEG;

  // 2) Inversions éventuelles (mappage logique -> réalité mécanique)
  if (INVERT_PAN) {
    // mirroir autour de la moitié de la plage
    // ex: 0 ↔ 180, 60 ↔ 120, 90 ↔ 90
    outPan = PAN_MIN + PAN_MAX - outPan;
  }
  if (INVERT_TILT) {
    // mirroir autour de (TILT_MIN + TILT_MAX)/2
    outTilt = TILT_MIN + TILT_MAX - outTilt;
  }

  // 3) Limites mécaniques de sécurité
  outPan  = constrain(outPan,  PAN_MIN,  PAN_MAX);
  outTilt = constrain(outTilt, TILT_MIN, TILT_MAX);

  // 4) Sortie servo
  panServo.write(outPan);
  tiltServo.write(outTilt);
}

// ============================================================================
// SECTION D — CLI (série) & test gimbal
// ============================================================================
void processSerialCommand(String command) {
  if (command == "center") {
    // Force la cible au centre (sans changer offset)
    targetPan  = PAN_CENTER;
    targetTilt = TILT_CENTER;
    Serial.println("[CMD] Center");
  } else if (command == "debug") {
    Serial.printf("[DBG] src=%s tgtPan=%d tgtTilt=%d curPan=%d curTilt=%d\n",
                  sourceName(activeSource),
                  targetPan, targetTilt,
                  currentPan, currentTilt);
  } else if (command == "test") {
    testServoMovement();
  } else {
    Serial.println("[CMD] Unknown");
  }
}

void testServoMovement() {
  Serial.println("[TEST] Full servo sweep");

  // PAN : centre → max → min → centre
  for (int p = PAN_CENTER; p <= PAN_MAX; p += 10) {
    panServo.write(p); delay(150);
  }
  for (int p = PAN_MAX; p >= PAN_MIN; p -= 10) {
    panServo.write(p); delay(150);
  }
  panServo.write(PAN_CENTER);

  // TILT : centre → bas → haut → centre
  for (int t = TILT_CENTER; t <= TILT_MAX; t += 6) {
    tiltServo.write(t); delay(150);
  }
  for (int t = TILT_MAX; t >= TILT_MIN; t -= 6) {
    tiltServo.write(t); delay(150);
  }
  tiltServo.write(TILT_CENTER);

  currentPan  = PAN_CENTER;
  currentTilt = TILT_CENTER;
  targetPan   = PAN_CENTER;
  targetTilt  = TILT_CENTER;

  Serial.println("[TEST] Done.");
}

// ============================================================================
// SECTION E — ESP-NOW RX
// ============================================================================
void initEspNowRX() {
  if (espNowReady) return;

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed");
    return;
  }

  // Signature callback core 3.x :
  // void (*)(const esp_now_recv_info* info, const uint8_t* data, int len)
  esp_now_register_recv_cb(onEspNowRecv);

  espNowReady = true;
  Serial.println("[ESP-NOW] RX ready");
}

void onEspNowRecv(const esp_now_recv_info* info, const uint8_t* data, int len) {
  // MAC de l’émetteur (si besoin pour debug)
  const uint8_t* mac = info->src_addr;
  (void)mac; // éviter warning "unused"

  if (len != (int)sizeof(ControlPacket)) return;

  ControlPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.header != 0xA55A) return;
  if (pkt.checksum != packetChecksum(pkt)) return;
  if ((pkt.flags & 0x0001u) == 0) return; // bit0 = PPM valide côté TX

  // Lissage RF
  linkPan  += (pkt.pan  - linkPan)  * LINK_SMOOTH_ALPHA;
  linkTilt += (pkt.tilt - linkTilt) * LINK_SMOOTH_ALPHA;

  // Conversion directe en cibles (0..180 pour PAN, 40..120 pour TILT)
  targetPan  = constrain(static_cast<int>(linkPan  + 0.5f), PAN_MIN,  PAN_MAX);
  targetTilt = constrain(static_cast<int>(linkTilt + 0.5f), TILT_MIN, TILT_MAX);

  lastEspNowTime = millis();
}

// ============================================================================
// SECTION F — CRSF RX (ELRS)
// ============================================================================
//
// On lit un flux CRSF standard venant d’ELRS et on extrait les 16 canaux RC.
// Le TX encode PAN/TILT dans les canaux 1 & 2 (CRSF index 0 & 1).

// Parseur simple d’une trame CRSF
uint8_t crsfBuffer[CRSF_MAX_FRAME_LEN];
uint8_t crsfIndex = 0;
uint8_t crsfFrameLength = 0;
bool    inFrame = false;

void initElrsUartRX() {
  if (elrsReady) return;
  if (ELRS_UART_RX_PIN < 0) {
    Serial.println("[CRSF] RX pin not defined");
    return;
  }
  elrsSerial.begin(ELRS_UART_BAUD, SERIAL_8N1, ELRS_UART_RX_PIN, ELRS_UART_TX_PIN);
  elrsReady = true;
  Serial.println("[CRSF] ELRS UART RX ready on GPIO 16");
}

void readCrsfPackets() {
  if (!elrsReady) return;

  while (elrsSerial.available()) {
    uint8_t b = elrsSerial.read();

    if (!inFrame) {
      if (b == CRSF_DEVICE_ADDRESS) {
        inFrame = true;
        crsfIndex = 0;
        crsfBuffer[crsfIndex++] = b;
      }
      continue;
    }

    crsfBuffer[crsfIndex++] = b;

    // Byte 1 = longueur
    if (crsfIndex == 2) {
      crsfFrameLength = b;
      if (crsfFrameLength > CRSF_MAX_FRAME_LEN - 2) {
        inFrame = false;
      }
    }

    // Frame complète : [addr][len][type+payload...][crc]
    if (crsfIndex >= 2 && crsfIndex == crsfFrameLength + 2) {
      uint8_t type = crsfBuffer[2];
      uint8_t* payload = &crsfBuffer[3];
      uint8_t payloadLen = crsfFrameLength - 2;
      uint8_t crc = crsfBuffer[2 + payloadLen];

      uint8_t calc = crsfCalcCRC(&crsfBuffer[2], payloadLen);
      if (crc == calc &&
          type == CRSF_FRAMETYPE_RC_CHANNELS &&
          payloadLen == CRSF_RC_PAYLOAD_LEN) {
        applyCrsfChannels(payload);
      }

      inFrame = false;
    }

    if (crsfIndex >= CRSF_MAX_FRAME_LEN) {
      inFrame = false;
    }
  }
}

// Décodage des 16 canaux (11 bits chacun) depuis le payload CRSF
void applyCrsfChannels(const uint8_t* payload) {
  uint16_t ch[CRSF_RC_CHANNEL_COUNT];
  uint32_t bitBuffer = 0;
  uint8_t bitsInBuffer = 0;
  uint8_t byteIndex = 0;

  for (int i = 0; i < CRSF_RC_CHANNEL_COUNT; ++i) {
    while (bitsInBuffer < 11) {
      bitBuffer |= ((uint32_t)payload[byteIndex++]) << bitsInBuffer;
      bitsInBuffer += 8;
    }
    ch[i] = bitBuffer & 0x7FF;
    bitBuffer >>= 11;
    bitsInBuffer -= 11;
  }

  // CH1 & CH2 : mêmes canaux que côté TX (PAN / TILT)
  int panMicro  = crsfToMicroseconds(ch[0]);
  int tiltMicro = crsfToMicroseconds(ch[1]);

  int panDeg  = microsecondsToDegrees(panMicro,  PAN_MIN,  PAN_MAX);
  int tiltDeg = microsecondsToDegrees(tiltMicro, TILT_MIN, TILT_MAX);

  targetPan  = constrain(panDeg,  PAN_MIN,  PAN_MAX);
  targetTilt = constrain(tiltDeg, TILT_MIN, TILT_MAX);

  lastCrsfTime = millis();
}

// CRC, mapping helpers
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

int crsfToMicroseconds(uint16_t value) {
  value = constrain(value, 172, 1811);
  long micro = map(value, 172, 1811, 988, 2012);
  return (int)constrain(micro, 1000L, 2000L);
}

int microsecondsToDegrees(int microseconds, int degMin, int degMax) {
  microseconds = constrain(microseconds, 1000, 2000);
  long deg = map(microseconds, 1000, 2000, degMin, degMax);
  return (int)constrain(deg, degMin, degMax);
}

// ============================================================================
// SECTION G — Divers
// ============================================================================
const char* sourceName(ActiveSource s) {
  switch (s) {
    case SOURCE_NONE:   return "NONE";
    case SOURCE_ESPNOW: return "ESP-NOW";
    case SOURCE_CRSF:   return "CRSF";
    default:            return "?";
  }
}
