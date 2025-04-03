#include <stdio.h>
#include <Arduino.h>
#include <Wire.h>

volatile int tickCount = 0;

void IRAM_ATTR onTick() {
  tickCount++;
}

void setup() {
  Serial.begin(9600);
  pinMode(D1, INPUT_PULLUP); // Canal A par exemple
  attachInterrupt(digitalPinToInterrupt(D1), onTick, RISING);
}

void loop() {
  static int lastTick = 0;

  if (Serial.available()) {
    Serial.read();  // Attente d'une touche
    Serial.println("Reset compteur. Tournez manuellement le moteur");
    tickCount = 0;
    delay(5000); // Temps pour faire un tour
    Serial.print("Ticks comptés : ");
    Serial.println(tickCount);
  }
}