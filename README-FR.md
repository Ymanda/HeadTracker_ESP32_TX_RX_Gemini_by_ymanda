ESP32 HeadTracker – Skyzone → AtomRC (ELRS / ESP-NOW)

Vue d’ensemble
Transforme le PPM head-tracker Skyzone en commandes PAN/TILT envoyées via ELRS (CRSF) ou ESP-NOW, puis pilote un gimbal AtomRC avec un second ESP32.
Diagramme :
Skyzone HT (PPM)
│
▼
┌───────────────┐        CRSF/ELRS         ┌───────────────┐
│   ESP32 TX    │     ──────────────▶     │   ESP32 RX    │
│ (côté lunettes)│                        │ (côté gimbal) │
└───────────────┘        ESP-NOW secours  └───────────────┘
│                                                  │
└── PWM bench (optionnel)                          └── Servos PAN/TILT

Affectation des broches
=======================
TX (lunettes)
============
PPM input (Skyzone)  → GPIO25
Mode switch A        → GPIO32
Mode switch B        → GPIO33
Button / Joystick UP → GPIO26
Pan servo (bench)    → GPIO18
Tilt servo (bench)   → GPIO19
ELRS CRSF TX → module → GPIO17

RX (drone)
==========
CRSF UART RX         → GPIO16
ESP-NOW (WiFi STA)   → interne
Pan servo            → GPIO18
Tilt servo           → GPIO19

Interrupteur 3 positions (TX)
=============================
A=LOW,  B=HIGH → PWM filaire (bench)
A=HIGH, B=LOW  → CRSF / ELRS (vol normal)
A=HIGH, B=HIGH → ESP-NOW

Logique du bouton (TX – GPIO26)
===============================
Maintien : caméra forcée au centre
Relâcher : recentrage (nouveaux offsets)
Double-clic : NORMAL → RACING → TILT-ONLY → NORMAL → …

Modes de mouvement caméra
=========================
NORMAL      : Pan+Tilt suivent le head tracker
RACING      : Pan=90°, Tilt=+20° vers le haut
TILT-ONLY   : Pan figé à l’activation, Tilt suit le PPM

Conventions d’angle
===================
PAN :
0°   = gauche
90°  = centre
180° = droite

TILT :
valeur plus petite = haut (ciel)
90°                = horizon
valeur plus grande = bas (sol)

Flux de données
===============

1) MODE COMM : chemin CRSF / ELRS (prioritaire)
===============================================
TX :
PPM → filtrage → degrés → trame RC CRSF → module ELRS
RX :
ELRS RX → trame RC CRSF → degrés → lissage → servos

1) MODE COMM : secours ESP-NOW (WiFi)
=====================================
TX :
degrés filtrés → ControlPacket → envoi ESP-NOW
RX :
paquet ESP-NOW → checksum → targetPan/targetTilt → lissage → servos

Priorité :
CRSF OK → utiliser CRSF
sinon si ESP-NOW OK → utiliser ESP-NOW
sinon → auto-centre

Filtrage (TX)
=============
error = |target – current|

Si erreur petite : position lunettes proche de la caméra
gain = faible
taux de mise à jour = lent

Si erreur grande : position lunettes éloignée de la caméra
gain = élevé
taux de mise à jour = rapide

QU’EST-CE QUI EST “PETIT” OU “GRAND” ?
======================================
PAN : 2°→10° (gain 0,04 → 1,0)
TILT : 2°→5° (gain 0,01 → 1,0)

Format ControlPacket (ESP-NOW)
struct {
uint16 header   = 0xA55A
uint16 pan      = pan filtré en degrés
uint16 tilt     = tilt filtré en degrés
uint16 flags    = bit0 = PPM valide
uint16 checksum = XOR(header,pan,tilt,flags,0x55AA)
}

Commandes série utiles (CLI)
center      = recentrer
debug       = état
test        = test balayage servos (local)
deadband X  = définir le deadband (TX uniquement)

Schémas de câblage (ASCII)
==========================

Lien : Skyzone → ESP32 TX :
Skyzone HT PPM ─────────> GPIO25
Skyzone GND ────────────> ESP32 GND

Lien : ESP32 TX → ELRS
GPIO17 (TX) → ELRS module RX
GND commun

Lien : ESP32 RX → Servos
GPIO18 → Pan servo
GPIO19 → Tilt servo
5V/GND → depuis BEC

Configuration MAC ESP-NOW
=========================
(POUR LE LIEN WIFI ESP-NOW ON UTILISE L’ADRESSE MAC DE LA CARTE)

1) Écrire ce script dans l’IDE Arduino et l’exécuter.
#include <WiFi.h>
WiFi.mode(WIFI_STA);
Serial.println(WiFi.macAddress());

2) Copier la MAC du RX dans le TX :
const uint8_t ESP_NOW_PEER_MAC[6] = {xx,xx,xx,xx,xx,xx};

Résumé des fichiers
HeadTracker_ESP32_TX_Gemini_by_ymanda.ino  → logique côté lunettes
HeadTracker_ESP32_RX_Gemini_by_ymanda.ino  → logique côté gimbal
README.md                                   → ce document
Auteur : Yannick Mandaba (Ymanda)
Dernière modif : 27 NOV 2025
