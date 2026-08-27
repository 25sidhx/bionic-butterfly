/*
 * S-DiY RC Bionic Butterfly project
 * PROJECT: S-DiY 2 SERVO FLAP RC BIONIC BUTTERFLY
 * VERSION: 1.1 (SINE WAVE) — CORRECTED
 * AUTHOR:  [S-DiY channel/ TruongVanSu91] (orig) + fixes
 * DATE:    02/2026
 * COPYRIGHT (C) 2026. ALL RIGHTS RESERVED.
 * -----------------------------------------------------------------------------
 * PIN MAP: Receiver PPM on pin 2, servoLeft on pin 4, servoRight on pin 5
 *          LED on pin 6.  Matches the hand-drawn circuit (D4/D5), NOT D9/D10.
 * -----------------------------------------------------------------------------
 *
 * FIXES APPLIED (audit pass):
 *  [A] Real failsafe: edge-watchdog on the PPM pin. latestValidChannelValue()
 *      never returns 0 after first link, so the original isConnected test was
 *      dead code. We watch time-since-last-pulse instead.
 *  [B] Control authority at full flap: total swing = flap + steer + elev + trim
 *      can exceed the 5..175 servo range and constrain() silently eats steering.
 *      Flap amplitude now shrinks when large stick/trim input is present.
 *  [C] Smooth flap speed: avoid integer map() truncation — float math.
 *  [D] wasFailsafe renamed to resetSinePhase (mode-transition flag only).
 *  [E] Explicit ppm.begin() guard — call if the installed lib exposes it.
 */

#include <Servo.h>
#include <PPMReader.h>

const byte interruptPin = 2;   // receiver PPM signal
const byte ledPin = 6;
const byte channelAmount = 8;
PPMReader ppm(interruptPin, channelAmount);

Servo servoLeft, servoRight;

const int centerAngle = 90;
const int flapAmplitude = 65;
const int maxPitchTrim = 20;

float phase = 0, perchPhase = 0, currentOffset = 0;
bool isPerchingState = false;
bool resetSinePhase = true;     // [D] was: wasFailsafe (overloaded name)

int lastPosL = 90, lastPosR = 90;
int lastChValues[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
const int ppmDeadband = 3;

int ledDebounceCounter = 0;
const int ledThreshold = 5;

unsigned long lastLogTime = 0;
const int logInterval = 1000;

// [A] Edge-watchdog: timestamp of last PPM pulse (any channel edge).
volatile unsigned long lastPpmEdge = 0;
const unsigned long LINK_TIMEOUT_US = 100000; // ~100 ms no pulse = link lost

// [B] Flap amplitude for the current tick, capped by control demand. Set in loop().
int safeFlapAmp = flapAmplitude;

void ppmEdgeISR() {
  lastPpmEdge = micros();
}

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // [E] If your installed PPMReader exposes begin(), uncomment the next line.
  // Some versions attach the interrupt in the constructor; calling begin() again
  // is harmless or a no-op. Comment out if it causes a duplicate-attach error.
  // ppm.begin();

  // [A] Independent edge-watchdog on the same pin (RISING). The library also
  // attaches an interrupt here; two RISING handlers on one pin are allowed.
  lastPpmEdge = micros();
  attachInterrupt(digitalPinToInterrupt(interruptPin), ppmEdgeISR, RISING);

  servoLeft.attach(4, 500, 2500);   // left servo  — matches circuit D4
  servoRight.attach(5, 500, 2500);  // right servo — matches circuit D5
  servoLeft.write(centerAngle);
  servoRight.write(centerAngle);

  Serial.println("--- BIONIC BUTTERFLY READY (SINE WAVE MODE v1.1) ---");
}

int readPPMFiltered(byte ch) {
  int raw = ppm.latestValidChannelValue(ch + 1, 1500);
  if (abs(raw - lastChValues[ch]) > ppmDeadband) {
    lastChValues[ch] = raw;
  }
  return lastChValues[ch];
}

void loop() {
  // [A] Real link detection: any PPM edge within LINK_TIMEOUT counts as alive.
  bool isConnected = (micros() - lastPpmEdge) < LINK_TIMEOUT_US;

  if (!isConnected) {
    digitalWrite(ledPin, LOW);
    processFailsafe();
  } else {
    int steering = readPPMFiltered(0);
    int elevator = readPPMFiltered(1);
    int throttle = readPPMFiltered(2);
    int ch5Trim   = readPPMFiltered(4);
    int rawCh6    = readPPMFiltered(5);
    int ch7Switch = readPPMFiltered(6);
    int ch8Raw    = readPPMFiltered(7);

    if (ch8Raw > 1550) { if (ledDebounceCounter < ledThreshold) ledDebounceCounter++; }
    else if (ch8Raw < 1450) { if (ledDebounceCounter > 0) ledDebounceCounter--; }
    digitalWrite(ledPin, (ledDebounceCounter >= ledThreshold) ? HIGH : LOW);

    int steerValue = (abs(steering - 1500) > 10) ? map(steering, 900, 2100, -15, 15) : 0;
    int finalSteer = steerValue + map(ch5Trim, 1000, 2000, -15, 15);
    int elevOffset = (abs(elevator - 1500) > 10) ? map(elevator, 900, 2100, -15, 15) : 0;
    int pitchTrim = map(rawCh6, 1000, 2000, -maxPitchTrim, maxPitchTrim);

    if (throttle > 1100) isPerchingState = false;
    else if (ch7Switch > 1600 && throttle < 1100) isPerchingState = true;
    if (ch7Switch < 1400) isPerchingState = false;

    if (isPerchingState) {
      updateCurrentOffsetPerch(rawCh6);
    } else if (throttle < 1050) {
      updateCurrentOffsetGlide();
    } else {
      updateCurrentOffsetSine(throttle);
    }

    // [B] Control authority: compute how much non-flap authority is demanded,
    // then shrink flap amplitude so total never exceeds the safe servo range.
    int controlDemand = abs(finalSteer) + abs(elevOffset) + abs(pitchTrim);
    int headroom = (175 - centerAngle) - controlDemand;   // room from center up to 175
    safeFlapAmp = (headroom < 5) ? 5 : (headroom < flapAmplitude ? headroom : flapAmplitude);

    int outL = centerAngle + (int)currentOffset + finalSteer + elevOffset + pitchTrim;
    int outR = centerAngle - (int)currentOffset + finalSteer - elevOffset - pitchTrim;

    updateServos(outL, outR);
  }

  if (millis() - lastLogTime > logInterval) {
    lastLogTime = millis();
    if (isConnected) {
      Serial.print("[CONNECTED] CH3: "); Serial.print(lastChValues[2]);
      Serial.print(" | LED: "); Serial.println(digitalRead(ledPin) ? "ON" : "OFF");
    } else {
      Serial.println("[!] WARNING: NO SIGNAL DETECTED");
    }
  }
  delay(4);
}


void updateCurrentOffsetSine(int throttle) {
  if (resetSinePhase) { phase = PI; resetSinePhase = false; }   // [D]

  // [C] Smooth float speed scaling (no integer map() truncation):
  // throttle 1050..2000 -> 0.060..0.100 rad/tick
  float speedFactor = (throttle - 1050) / (2000.0 - 1050.0) * (100 - 60) + 60;
  speedFactor /= 1000.0;
  phase += speedFactor;
  if (phase >= 2 * PI) phase -= 2 * PI;

  // [B] apply the demand-adjusted amplitude computed in loop()
  currentOffset = sin(phase) * safeFlapAmp;
}


void updateCurrentOffsetPerch(int rawCh6) {
  resetSinePhase = true;                                       // [D]
  float maxUpAngle = map(rawCh6, 1000, 2000, 30, 60);
  perchPhase += 0.01;
  float goalAngle = maxUpAngle + (sin(perchPhase) * 20.0) - 20.0;
  if (abs(currentOffset - goalAngle) > 0.5) currentOffset += (currentOffset < goalAngle) ? 1.5 : -1.5;
}

void updateCurrentOffsetGlide() {
  resetSinePhase = true;                                       // [D]
  if (currentOffset < 0) currentOffset += 2.0;
  else if (currentOffset > 0) currentOffset -= 2.0;
  if (abs(currentOffset) < 2.5) currentOffset = 0;
}

void processFailsafe() {
  if (currentOffset < 0) currentOffset += 1.5;
  else if (currentOffset > 0) currentOffset -= 1.5;
  if (abs(currentOffset) < 1.5) currentOffset = 0;
  updateServos(centerAngle + (int)currentOffset, centerAngle - (int)currentOffset);
  resetSinePhase = true;                                       // [D]
}

void updateServos(int targetL, int targetR) {
  targetL = constrain(targetL, 5, 175);
  targetR = constrain(targetR, 5, 175);
  if (abs(targetL - lastPosL) >= 1) { servoLeft.write(targetL); lastPosL = targetL; }
  if (abs(targetR - lastPosR) >= 1) { servoRight.write(targetR); lastPosR = targetR; }
}
