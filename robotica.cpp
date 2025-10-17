#include <Arduino.h>

// =================== TIEMPOS ===================
unsigned long t1 = 0, t2 = 0;
const int dt_us = 20000; // 2 ms (500 Hz) -> asegurate que sea 2000 si quieres 500Hz

// =================== MOTOR R ===================
#define ENC_R_A 4
#define ENC_R_B 5
const int senR1 = 7;
const int senR2 = 6;
volatile long pulseCountR = 0;
const int PULSOS_REV_R = 11050;
const float DEG_PER_PULSE_R = 360.0 / PULSOS_REV_R;

// PID R
float KpR = 1.0, KiR = 0.5, KdR = 0.06; // reduje Kd un poco para empezar
float e_prev_R = 0.0, integral_R = 0.0;
float th_des_R = 0.0;
float d_filtered_R = 0.0;

// =================== MOTOR E ===================
#define ENC_E_A 8
#define ENC_E_B 9
const int senE1 = 10;
const int senE2 = 11;
volatile long pulseCountE = 0;
const int PULSOS_REV_E = 11050;
const float DEG_PER_PULSE_E = 360.0 / PULSOS_REV_E;

// PID E
float KpE = 1.0, KiE = 0.5, KdE = 0.06;
float e_prev_E = 0.0, integral_E = 0.0;
float th_des_E = 0.0;
float d_filtered_E = 0.0;

// =================== PLANAR (ULTRASÓNICO) ===================
#define TRIG_PIN 17
#define ECHO_PIN 18
const int senP1 = 12;
const int senP2 = 13;
const int channelP1 = 4;
const int channelP2 = 5;

float p_ref = 0.0;           // referencia (cm)
float distancia_actual = 0.0;
float dead_band = 0.2;       // cm
int PWM_P = 180;             // intensidad fija del planar

// =================== PWM CONFIG ===================
const int freqPWM = 20000;
const int pwmResolution = 8; // 8 bits -> 0..255
const int chR1 = 0, chR2 = 1;
const int chE1 = 2, chE2 = 3;

// =================== RECEPCIÓN SERIAL ===================
String inputString = "";
bool stringComplete = false;

// =================== ENCODER ISR ===================
void IRAM_ATTR handleEncoderR_A() {
  int a = digitalRead(ENC_R_A);
  int b = digitalRead(ENC_R_B);
  if (a == b) pulseCountR++;
  else pulseCountR--;
}
void IRAM_ATTR handleEncoderR_B() {
  int a = digitalRead(ENC_R_A);
  int b = digitalRead(ENC_R_B);
  if (a != b) pulseCountR++;
  else pulseCountR--;
}

void IRAM_ATTR handleEncoderE_A() {
  int a = digitalRead(ENC_E_A);
  int b = digitalRead(ENC_E_B);
  if (a == b) pulseCountE++;
  else pulseCountE--;
}
void IRAM_ATTR handleEncoderE_B() {
  int a = digitalRead(ENC_E_A);
  int b = digitalRead(ENC_E_B);
  if (a != b) pulseCountE++;
  else pulseCountE--;
}

// =================== FUNCIONES PLANAR ===================
float medirDistancia() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duracion = pulseIn(ECHO_PIN, HIGH, 30000);
  float distancia = duracion * 0.0343 / 2.0; // cm
  return distancia;
}

void controlPlanar() {
  distancia_actual = medirDistancia()/100;
  float KpP = 5.0;
  float error = distancia_actual - p_ref;
  float u = KpP * error;
  u = constrain(u, -100, 230); // ajustar a 8-bit

  if (abs(error) < dead_band) {
    u = 0;
  }
  if (u > 0) {
    ledcWrite(channelP1, (int)u);
    ledcWrite(channelP2, 0);
  } else {
    ledcWrite(channelP1, 0);
    ledcWrite(channelP2, (int)(-u));
  }
}

// Helper: wrap angle to [-180,180)
float wrap180(float ang) {
  float a = fmod(ang, 360.0f);
  if (a >= 180.0f) a -= 360.0f;
  if (a < -180.0f) a += 360.0f;
  return a;
}

// shortest angle from current to target (both in degrees, -180..180)
float angleErrorShortest(float target, float current_wrapped) {
  float diff = target - current_wrapped;
  if (diff > 180.0f) diff -= 360.0f;
  if (diff < -180.0f) diff += 360.0f;
  return diff;
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);

  // Encoders R y E
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), handleEncoderR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), handleEncoderR_B, CHANGE);

  pinMode(ENC_E_A, INPUT_PULLUP);
  pinMode(ENC_E_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_E_A), handleEncoderE_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_E_B), handleEncoderE_B, CHANGE);

  // PWM channels
  ledcSetup(chR1, freqPWM, pwmResolution);
  ledcSetup(chR2, freqPWM, pwmResolution);
  ledcSetup(chE1, freqPWM, pwmResolution);
  ledcSetup(chE2, freqPWM, pwmResolution);
  ledcSetup(channelP1, freqPWM, pwmResolution);
  ledcSetup(channelP2, freqPWM, pwmResolution);

  ledcAttachPin(senR1, chR1);
  ledcAttachPin(senR2, chR2);
  ledcAttachPin(senE1, chE1);
  ledcAttachPin(senE2, chE2);
  ledcAttachPin(senP1, channelP1);
  ledcAttachPin(senP2, channelP2);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

// =================== LOOP ===================
void loop() {
  t1 = micros();

  // === 1️⃣ Leer encoders ===
  noInterrupts();
  long countR = pulseCountR;
  long countE = pulseCountE;
  interrupts();

  float angR = countR * DEG_PER_PULSE_R; // acumulado, puede ser >360
  float angE = countE * DEG_PER_PULSE_E;

  // wrapped for display/control ([-180,180))
  float angR_wrapped = wrap180(angR);
  float angE_wrapped = wrap180(angE);

  // === 2️⃣ Leer referencias desde Serial ===
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    stringComplete = false;
    float p_ref_m, r_ref_rad, e_ref_rad;
    int n = sscanf(inputString.c_str(), "%f,%f,%f", &p_ref_m, &r_ref_rad, &e_ref_rad);
    inputString = "";

    if (n == 3) {
      // Convertir unidades
      p_ref = p_ref_m;               // m -> cm (si envías metros)
      float r_ref_deg = (r_ref_rad * 180.0 / PI) * (-1.0f);
      float e_ref_deg = e_ref_rad * 180.0 / PI;

      th_des_R = constrain(r_ref_deg, -180.0f, 180.0f);
      th_des_E = constrain(e_ref_deg, -180.0f, 180.0f);
    }
  }

  // === 3️⃣ Control PID R ===
  float Ts = dt_us / 1e6f;
  float eR = angleErrorShortest(th_des_R, angR_wrapped); // <-- USAR WRAPPED ANGLE
  integral_R += eR * Ts;
  integral_R = constrain(integral_R, -200.0f, 200.0f);
  float dR_raw = (eR - e_prev_R) / Ts;
  // filtro exponencial para derivada
  const float alpha = 0.85; // mayor = más suavizado
  d_filtered_R = alpha * d_filtered_R + (1.0f - alpha) * dR_raw;
  float uR = KpR * eR + KiR * integral_R + KdR * d_filtered_R;
  e_prev_R = eR;
  float usatR = constrain(uR, -12.0f, 12.0f);

  // mapa consistente a 8-bit
  float PWMf_R = map((int)roundf(usatR*100), -1200, 1200, -255, 255); // ejemplo de escala
  PWMf_R = constrain(PWMf_R, -255, 255);
  int PWM_R = roundf(fabs(PWMf_R));

  // === 4️⃣ Control PID E ===
  float eE = angleErrorShortest(th_des_E, angE_wrapped);
  integral_E += eE * Ts;
  integral_E = constrain(integral_E, -200.0f, 200.0f);
  float dE_raw = (eE - e_prev_E) / Ts;
  d_filtered_E = alpha * d_filtered_E + (1.0f - alpha) * dE_raw;
  float uE = KpE * eE + KiE * integral_E + KdE * d_filtered_E;
  e_prev_E = eE;
  float usatE = constrain(uE, -12.0f, 12.0f);
  float PWMf_E = map((int)roundf(usatE*100), -1200, 1200, -255, 255);
  PWMf_E = constrain(PWMf_E, -255, 255);
  int PWM_E = roundf(fabs(PWMf_E));

  // === 5️⃣ Salidas a los puentes H ===
  // Motor R
  if (PWMf_R > 0) { ledcWrite(chR1, PWM_R); ledcWrite(chR2, 0); }
  else if (PWMf_R < 0) { ledcWrite(chR1, 0); ledcWrite(chR2, PWM_R); }
  else { ledcWrite(chR1, 0); ledcWrite(chR2, 0); }

  // Motor E
  if (PWMf_E > 0) { ledcWrite(chE1, PWM_E); ledcWrite(chE2, 0); }
  else if (PWMf_E < 0) { ledcWrite(chE1, 0); ledcWrite(chE2, PWM_E); }
  else { ledcWrite(chE1, 0); ledcWrite(chE2, 0); }

  // === 6️⃣ Control planar ===
  controlPlanar();

  // === 7️⃣ Mantener periodo fijo ===
  t2 = micros();
  while ((t2 - t1) < dt_us) t2 = micros();
  unsigned long delta_t = t2 - t1;

  // === 8️⃣ Enviar datos al Serial ===
  Serial.print(p_ref);  Serial.print(",");
  Serial.print(distancia_actual); Serial.print(",");
  Serial.print(th_des_R);  Serial.print(",");
  Serial.print(angR_wrapped); Serial.print(",");
  Serial.print(eR);        Serial.print(",");
  Serial.print(PWMf_R);    Serial.print(",");
  Serial.print(th_des_E);  Serial.print(",");
  Serial.print(angE_wrapped); Serial.print(",");
  Serial.print(eE);        Serial.print(",");
  Serial.print(PWMf_E);
  Serial.print(",");
  Serial.println(delta_t);
}
