#include <Arduino.h>
#include <stdio.h>
#include <Wire.h>

// === CONFIGURATION PINS ===
const byte pwm_pin = D1;      // PWM vers driver moteur
const byte dir_pin = D2;      // Direction moteur

// === i / dir ===
int i = 0;
bool dir = true;

// === SETUP ===
void setup() {
  Serial.begin(9600);

  // Moteur
  pinMode(pwm_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
}

// === LOOP PRINCIPALE ===
void loop() {
  static unsigned long last_time = 0;
  unsigned long now = millis();

  if (now - last_time >= 100) {  // 10 Hz
    last_time = now;
    i += 1;
    if (i % 256 == 0) {
      i = 0;
      dir = !dir;
    }

    analogWrite(pwm_pin, i % 256);
    digitalWrite(dir_pin, (dir? HIGH : LOW));

    // --- Debug série ---
    Serial.print("\t gamma (N·m): ");
    Serial.println(i * (dir? 1 : -1));
  }
}
