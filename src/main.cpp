#include <Arduino.h>
#include <stdio.h>
#include <Wire.h>

// === CONFIGURATION PINS ===
const byte pinA = D5;         // Encodeur canal A
const byte pinB = D6;         // Encodeur canal B
const byte pwm_pin = D1;      // PWM vers driver moteur
const byte dir_pin = D2;      // Direction moteur

// === PARAMÈTRES DU MOTEUR & DE COMMANDE ===
const float gamma_max = 0.2;               // Couple max du moteur (N·m)
const float threshold = 9.0 * M_PI / 10.0; // Seuil de basculement en linéaire (rad)
const float tol_dtheta = 0.2;              // Tolérance de vitesse (rad/s)

// === PARAMÈTRES D’ENCODEUR ===
const int ticks_per_rev = 1000;   // Ticks par révolution
const float angle_per_tick = 2 * M_PI / ticks_per_rev;

// === MATRICE DE GAIN ===
float k1 = 14.5; // gain proportionnel
float k2 = 2.0;  // gain dérivé

// === PARAMÈTRES DE L'ESP8266 ===
const int pwm_max = 256;                   // Résolution PWM ESP8266
const float V_Max = 3.3;                   // Tension maximum de ESP8266
const float epsilon_u = V_Max / gamma_max; // Coefficient de conversion (V/N·m)

// === VARIABLES D’ÉTAT ===
volatile long encoder_ticks = 0;
float theta = 0.0;
float dtheta = 0.0;

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
  float theta_vu = fmod(theta + M_PI, 2 * M_PI) - M_PI;
  float gamma = 0;

  if (fabs(theta_vu) < threshold) {
    // Swing Up
    if (fabs(dtheta) < tol_dtheta && fabs(theta_vu) < 0.1) {
      // impulsion de départ
      gamma = gamma_max;
    }
    if (fabs(dtheta) > tol_dtheta) {
      // Commande Bang-Bang
      gamma = copysign(gamma_max, dtheta);
    }
  } else {
    // Commande linéaire
    float theta_error = fmod(theta, 2 * M_PI) - M_PI;
    gamma = - k1 * theta_error - k2 * dtheta;
  }

  return gamma;
}

// === COMMANDE MOTEUR ===
void set_motor(float gamma) {
  float epsilon = gamma * epsilon_u; // Convertir couple en tension

  int pwm_val = (int)(fabs(epsilon) / V_Max * pwm_max);
  pwm_val = constrain(pwm_val, 0, pwm_max);

  digitalWrite(dir_pin, epsilon >= 0 ? HIGH : LOW);
  analogWrite(pwm_pin, pwm_val);
}

// === LOOP PRINCIPALE ===
void loop() {
  static unsigned long last_time = 0;
  unsigned long now = millis();

  if (now - last_time >= 1) {  // 1000 Hz
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
    Serial.println(gamma, 5);
  }
}
