#include <Arduino.h>

const int IN1 = 10;
const int IN2 = 9;

const int PIN_A = 6;
const int PIN_B = 7;

const int FREQ_HZ    = 5000;
const int RESOLUCION = 8;

const float CPR          = 70.0;
const int   INTERVALO_MS = 100;

volatile long conteo = 0;

// ──────────────────────────────────────────
void IRAM_ATTR isrEncoder() {
  if (digitalRead(PIN_B) == HIGH) conteo++;
  else                            conteo--;
}

// ──────────────────────────────────────────
float calcularRPM() {
  static long          conteoAnterior = 0;
  static unsigned long tAnterior      = 0;

  unsigned long ahora = millis();
  long snapshot;

  noInterrupts();
    snapshot = conteo;
  interrupts();

  float dt    = (ahora - tAnterior) / 1000.0;
  long  delta = snapshot - conteoAnterior;
  float rpm   = (delta / CPR) * (60.0 / dt);

  conteoAnterior = snapshot;
  tAnterior      = ahora;

  return rpm;
}

// ──────────────────────────────────────────
void frenar() {
  ledcWrite(IN1, 0);
  ledcWrite(IN2, 0);
}

void adelante(int velocidad) {
  ledcWrite(IN1, constrain(velocidad, 0, 255));
  ledcWrite(IN2, 0);
}

void atras(int velocidad) {
  ledcWrite(IN1, 0);
  ledcWrite(IN2, constrain(velocidad, 0, 255));
}

// ──────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(3000);

  ledcAttach(IN1, FREQ_HZ, RESOLUCION);
  ledcAttach(IN2, FREQ_HZ, RESOLUCION);
  frenar();

  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_A), isrEncoder, RISING);

  Serial.println("Listo!");
}

// ──────────────────────────────────────────
void loop() {
  static unsigned long t0 = millis();
  static bool started = false;

  if (!started) {
    adelante(255);   // escalón de PWM
    started = true;
    t0 = millis();
  }

  float rpm = calcularRPM();
  unsigned long t = millis() - t0;

  Serial.print(t);
  Serial.print(",");
  Serial.println(rpm);

  delay(INTERVALO_MS);

  if (t > 3000) {
    frenar();
    while (1); // detener loop
  }
}