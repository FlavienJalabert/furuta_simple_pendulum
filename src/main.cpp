
#include <Arduino.h>
#include <stdio.h>
#include <wire.h>

// === CONFIGURATION PINS ===
const byte pinA = D5;         // Encodeur canal A
const byte pinB = D6;         // Encodeur canal B
const byte pwm_pin = D1;      // PWM vers driver moteur
const byte dir_pin = D2;      // Direction moteur

// === PARAMÈTRES DU MOTEUR & DE COMMANDE ===
const float gamma_max = 0.6;      // Couple max (N·m)
const float epsilon_u = 8.0;      // Coefficient de conversion : V/N·m
const float V_max = 12.0;         // Tension d’alim max (V)
const int pwm_max = 1023;         // Résolution PWM ESP8266

// === PARAMÈTRES D’ENCODEUR ===
const int ticks_per_rev = 1000;   // Ticks par révolution
const float angle_per_tick = 2 * M_PI / ticks_per_rev;

// === VARIABLES D’ÉTAT ===
volatile long encoder_ticks = 0;
float theta = 0.0;
float dtheta = 0.0;

// === INTERNE POUR DERIVÉE ===
float dtheta_prev = 0.0;

// === MATRICE DE GAIN ===
float k1 = 14.5;
float k2 = 2.0;

// === INTERRUPTIONS ===
void IRAM_ATTR handle_encoder() {
  bool b = digitalRead(pinB);
  encoder_ticks += (b ? -1 : 1);  // Sens de rotation
}

// === SETUP ===
void setup() {
  Serial.begin(9600);

  // Moteur
  pinMode(pwm_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);

  // Encodeur
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA), handle_encoder, RISING);
}

// === CALCUL DE THETA ET DTHETA ===
void update_encoder() {
  static long prev_ticks = 0;
  static unsigned long prev_time = 0;

  long ticks = encoder_ticks;
  unsigned long now = millis();

  long dt_ticks = ticks - prev_ticks;
  unsigned long dt_ms = now - prev_time;

  if (dt_ms > 0) {
    theta = ticks * angle_per_tick;
    dtheta = (dt_ticks * angle_per_tick) / (dt_ms / 1000.0);
  }

  prev_ticks = ticks;
  prev_time = now;
}

// === STRATÉGIE DE COMMANDE ===
float compute_command(float theta, float dtheta) {
  const float tol_theta = 9.0 * M_PI / 10.0;

  float ddtheta = dtheta - dtheta_prev;
  dtheta_prev = dtheta;

  float gamma = 0.0;

  if (fabs(theta - M_PI) < tol_theta) {
    // Phase de stabilisation linéaire
    gamma = -k1 * (theta - copysign(M_PI, theta)) - k2 * dtheta;
  }
  else {
    // Phase bang-bang
    gamma = (dtheta >= 0) ? gamma_max : -gamma_max;
  }

  // Saturation du couple
  if (fabs(gamma) > gamma_max) {
    gamma = copysign(gamma_max, gamma);
  }

  return gamma;
}

// === COMMANDE MOTEUR ===
void set_motor(float gamma) {
  float epsilon = gamma * epsilon_u;             // Convertir couple en tension
  epsilon = constrain(epsilon, -V_max, V_max);   // Saturation tension

  int pwm_val = (int)(fabs(epsilon) / V_max * pwm_max);
  pwm_val = constrain(pwm_val, 0, pwm_max);

  digitalWrite(dir_pin, epsilon >= 0 ? HIGH : LOW);
  analogWrite(pwm_pin, pwm_val);
}

// === LOOP PRINCIPALE ===
void loop() {
  static unsigned long last_time = 0;
  unsigned long now = millis();

  if (now - last_time >= 10) {  // 100 Hz
    last_time = now;

    update_encoder();                               // calculer theta et dtheta
    float gamma = compute_command(theta, dtheta);   // Calculer la commande
    set_motor(gamma);                               // Appliquer la commande

    // --- Debug série ---
    Serial.print("theta (rad): ");
    Serial.print(theta, 3);
    Serial.print("\t dtheta (rad/s): ");
    Serial.print(dtheta, 3);
    Serial.print("\t gamma (N·m): ");
    Serial.println(gamma, 3);
  }
}
