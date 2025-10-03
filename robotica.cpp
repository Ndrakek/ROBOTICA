#include <Arduino.h>

unsigned long t1 = 0, t2 = 0;
int dt_us = 2000;   // tiempo de muestreo 2ms (500 Hz)

// ================= MOTOR R =================
#define ENCODER_R_A 4
#define ENCODER_R_B 5
const int senR1 = 7;
const int senR2 = 6;
volatile long pulseCountR = 0;
const int channelR1 = 0;
const int channelR2 = 1;

// ================= MOTOR E =================
#define ENCODER_E_A 8
#define ENCODER_E_B 9
const int senE1 = 10;
const int senE2 = 11;
volatile long pulseCountE = 0;
const int channelE1 = 2;
const int channelE2 = 3;

// ================= MOTOR P (ULTRASÓNICO) =================
#define TRIG_PIN 17
#define ECHO_PIN 18
const int senP1 = 12;
const int senP2 = 13;
const int channelP1 = 4;
const int channelP2 = 5;

// ================= CONFIGURACIÓN GENERAL =================
const int PULSOS_REV = 11050;
const float DEG_PER_PULSE = 360.0 / PULSOS_REV;
const int freqPWM = 20000;
const int pwmResolution = 8;

// PID común
float Kp = 2.0f, Ki = 1.5f, Kd = 0.6f;

// ======== ESTADO PID R =========
float eR_prev = 0, intR = 0;
static float th_des_R = 0.0f;
static float last_input_R = 0.0f;
static bool last_input_R_init = false;

// ======== ESTADO PID E =========
float eE_prev = 0, intE = 0;
static float th_des_E = 0.0f;
static float last_input_E = 0.0f;
static bool last_input_E_init = false;

// ======== ESTADO PID P =========
float eP_prev = 0, intP = 0;
static float th_des_P = 0.0f;
static float last_input_P = 0.0f;
static bool last_input_P_init = false;

// ================= FILTRO MEDIA MÓVIL PARA P =================
#define N_FILTRO 5
float bufferP[N_FILTRO] = {0};
int indexP = 0;
bool bufferLlenoP = false;

float filtrarDistancia(float nuevaMuestra) {
  bufferP[indexP] = nuevaMuestra;
  indexP = (indexP + 1) % N_FILTRO;

  int n = bufferLlenoP ? N_FILTRO : indexP;
  if (indexP == 0) bufferLlenoP = true;

  float suma = 0;
  for (int i = 0; i < n; i++) suma += bufferP[i];
  return suma / n;
}

// ================= ISR ENC R =================
void IRAM_ATTR handleEncoderRA() {
  int A = digitalRead(ENCODER_R_A);
  int B = digitalRead(ENCODER_R_B);
  if (A == B) pulseCountR++;
  else pulseCountR--;
}
void IRAM_ATTR handleEncoderRB() {
  int A = digitalRead(ENCODER_R_A);
  int B = digitalRead(ENCODER_R_B);
  if (A != B) pulseCountR++;
  else pulseCountR--;
}

// ================= ISR ENC E =================
void IRAM_ATTR handleEncoderEA() {
  int A = digitalRead(ENCODER_E_A);
  int B = digitalRead(ENCODER_E_B);
  if (A == B) pulseCountE++;
  else pulseCountE--;
}
void IRAM_ATTR handleEncoderEB() {
  int A = digitalRead(ENCODER_E_A);
  int B = digitalRead(ENCODER_E_B);
  if (A != B) pulseCountE++;
  else pulseCountE--;
}

// ================= FUNCIONES =================
int pidControl(float &th_des, float &last_input, bool &init,
               float &e_prev, float &integral,
               float medida, float consigna) {

  if (!init) {
    th_des = medida + (consigna - medida);
    last_input = consigna;
    init = true;
  } else {
    th_des += (consigna - last_input);
    last_input = consigna;
  }

  float e = th_des - medida;
  float Ts = dt_us / 1e6f;

  integral += e * Ts;
  integral = constrain(integral, -100.0f, 100.0f);

  float derivative = (e - e_prev) / Ts;
  float u = Kp * e + Ki * integral + Kd * derivative;
  e_prev = e;

  float usat = constrain(u, -12.0f, 12.0f);
  float PWMf = usat * (255.0f / 12.0f);
  PWMf = constrain(PWMf, -255.0f, 255.0f);

  return (int)round(PWMf);
}

float leerDistanciaM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duracion = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30ms
  float distancia_cm = duracion * 0.034 / 2.0;
  return distancia_cm / 100.0; // en metros
}

void setup() {
  Serial.begin(115200);

  // encoder R
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), handleEncoderRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_B), handleEncoderRB, CHANGE);

  // encoder E
  pinMode(ENCODER_E_A, INPUT_PULLUP);
  pinMode(ENCODER_E_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_E_A), handleEncoderEA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_E_B), handleEncoderEB, CHANGE);

  // sensor ultrasónico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // pwm R
  ledcSetup(channelR1, freqPWM, pwmResolution);
  ledcSetup(channelR2, freqPWM, pwmResolution);
  ledcAttachPin(senR1, channelR1);
  ledcAttachPin(senR2, channelR2);

  // pwm E
  ledcSetup(channelE1, freqPWM, pwmResolution);
  ledcSetup(channelE2, freqPWM, pwmResolution);
  ledcAttachPin(senE1, channelE1);
  ledcAttachPin(senE2, channelE2);

  // pwm P
  ledcSetup(channelP1, freqPWM, pwmResolution);
  ledcSetup(channelP2, freqPWM, pwmResolution);
  ledcAttachPin(senP1, channelP1);
  ledcAttachPin(senP2, channelP2);

  Serial.println("refR,angR,errorR,PWM_R,refE,angE,errorE,PWM_E,refP,posP,errorP,PWM_P,dt_us");
}

void loop() {
  t1 = micros();

  // leer encoders
  noInterrupts();
  long countR = pulseCountR;
  long countE = pulseCountE;
  interrupts();

  float angR = countR * DEG_PER_PULSE;
  float angE = countE * DEG_PER_PULSE;

  // leer sensor P con filtro media móvil
  float medidaP = leerDistanciaM();
  float posP = filtrarDistancia(medidaP);

  // recibir consignas p,r,e
  static float consigna_p = 0.0f;     // en metros
  static float consigna_r_deg = 0.0f; // en grados
  static float consigna_e_deg = 0.0f; // en grados

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    float p, r, e;
    if (sscanf(line.c_str(), "%f,%f,%f", &p, &r, &e) == 3) {
      consigna_p = p;                            // metros
      consigna_r_deg = (r * 180.0f / PI) * -1;   // rad → deg
      consigna_e_deg = (e * 180.0f / PI);        // rad → deg
    }
  }

  // aplicar PID R
  int PWM_R = pidControl(th_des_R, last_input_R, last_input_R_init,
                         eR_prev, intR, angR, consigna_r_deg);
  if (PWM_R > 0) {
    ledcWrite(channelR1, PWM_R);
    ledcWrite(channelR2, 0);
  } else if (PWM_R < 0) {
    ledcWrite(channelR1, 0);
    ledcWrite(channelR2, -PWM_R);
  } else {
    ledcWrite(channelR1, 0);
    ledcWrite(channelR2, 0);
  }

  // aplicar PID E
  int PWM_E = pidControl(th_des_E, last_input_E, last_input_E_init,
                         eE_prev, intE, angE, consigna_e_deg);
  if (PWM_E > 0) {
    ledcWrite(channelE1, PWM_E);
    ledcWrite(channelE2, 0);
  } else if (PWM_E < 0) {
    ledcWrite(channelE1, 0);
    ledcWrite(channelE2, -PWM_E);
  } else {
    ledcWrite(channelE1, 0);
    ledcWrite(channelE2, 0);
  }

  // aplicar PID P con dirección forzada + banda muerta
  int PWM_P = pidControl(th_des_P, last_input_P, last_input_P_init,
                         eP_prev, intP, posP, consigna_p);

  float errorP = consigna_p - posP;
  float deadband = 0.03; // ±3 cm

  if (fabs(errorP) < deadband) {
    PWM_P = 0;
    ledcWrite(channelP1, 0);
    ledcWrite(channelP2, 0);
  } else {
    PWM_P = abs(PWM_P);
    if (posP > consigna_p) {
      ledcWrite(channelP1, PWM_P);
      ledcWrite(channelP2, 0);
    } else {
      ledcWrite(channelP1, 0);
      ledcWrite(channelP2, PWM_P);
    }
  }

  // esperar periodo fijo
  t2 = micros();
  while ((t2 - t1) < dt_us) t2 = micros();
  unsigned long delta_t = t2 - t1;

  // debug
  Serial.print(th_des_R); Serial.print(",");
  Serial.print(fmod(angR,360.0f)); Serial.print(",");
  Serial.print(th_des_R - angR); Serial.print(",");
  Serial.print(PWM_R); Serial.print(",");

  Serial.print(th_des_E); Serial.print(",");
  Serial.print(fmod(angE,360.0f)); Serial.print(",");
  Serial.print(th_des_E - angE); Serial.print(",");
  Serial.print(PWM_E); Serial.print(",");

  Serial.print(consigna_p); Serial.print(",");
  Serial.print(posP); Serial.print(",");
  Serial.print(errorP); Serial.print(",");
  Serial.print(PWM_P); Serial.print(",");

  Serial.println(delta_t);
}
