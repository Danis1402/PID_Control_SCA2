#include <Arduino.h>

// --- Pines ---
const int IN1 = 10;
const int IN2 = 9;
const int PIN_A = 6;
const int PIN_B = 7;

// --- Configuración PWM (ESP32) ---
const int FREQ_HZ    = 5000;
const int RESOLUCION = 8;

// --- Configuración Encoder ---
const float CPR = 70.0; // ¡Verifica si este valor incluye la reductora!

// --- Variables de Control PID ---
const int   INTERVALO_MS = 100;   // Ts = 100ms (0.1s)
float setpoint = 1000;        // Velocidad deseada en RPM

// Coeficientes obtenidos de MATLAB (Ya multiplicados por el factor de conversión 4.4)
const float f = 4.45;
const float b0 = 0.0628*f; 
const float b1 = -0.0446*f;

float u_prev = 0; // Salida anterior (u[n-1])
float e_prev = 0; // Error anterior (e[n-1])

volatile long conteo = 0;
unsigned long tiempoUltimoControl = 0;

// ──────────────────────────────────────────
void IRAM_ATTR isrEncoder() {
  if (digitalRead(PIN_B) == HIGH) conteo++;
  else                            conteo--;
}

// ──────────────────────────────────────────
float calcularRPM(float dt_segundos) {
  static long conteoAnterior = 0;
  long snapshot;

  noInterrupts();
    snapshot = conteo;
  interrupts();

  long delta = snapshot - conteoAnterior;
  // RPM = (deltapulsos / CPR) * (60 segundos / dt)
  float rpm = (delta / CPR) * (60.0 / dt_segundos);

  conteoAnterior = snapshot;
  return rpm;
}

// ──────────────────────────────────────────
void adelante(int velocidad) {
  ledcWrite(IN1, constrain(velocidad, 0, 255));
  ledcWrite(IN2, 0);
}

void frenar() {
  ledcWrite(IN1, 0);
  ledcWrite(IN2, 0);
}

// ──────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  
  // Configuración de motores
  ledcAttach(IN1, FREQ_HZ, RESOLUCION);
  ledcAttach(IN2, FREQ_HZ, RESOLUCION);
  frenar();

  // Configuración de encoder
  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_A), isrEncoder, RISING);

  Serial.println("Sistema de Control PI Iniciado...");
  delay(1000);
}

// ──────────────────────────────────────────
void loop() {
  unsigned long tiempoActual = millis();

  // Ejecutar el control exactamente cada 10ms
  if (tiempoActual - tiempoUltimoControl >= INTERVALO_MS) {
    float dt = (tiempoActual - tiempoUltimoControl) / 1000.0;
    tiempoUltimoControl = tiempoActual;

    // 1. Leer velocidad actual
    float rpmActual = calcularRPM(dt);

    // 2. Calcular error
    float e_now = setpoint - rpmActual;

    // 3. Ecuación en diferencias del PI: u[n] = u[n-1] + b0*e[n] + b1*e[n-1]
    float u_now = u_prev + (b0 * e_now) + (b1 * e_prev);

    // 4. Saturación (Anti-windup implícito al limitar u_now antes de guardarla)
    if (u_now > 255) u_now = 255;
    if (u_now < 0)   u_now = 0;

    // 5. Actuar sobre el motor
    adelante((int)u_now);

    // 6. Actualizar estados para el siguiente ciclo
    u_prev = u_now;
    e_prev = e_now;

    // 7. Monitorización (Usa el Serial Plotter de Arduino)
    Serial.print("Setpoint:"); Serial.print(setpoint);    Serial.print(", ");
    Serial.print("RPM:");      Serial.print(rpmActual);   Serial.print(", ");
    Serial.print("PWM:");      Serial.println(u_now);     Serial.print(", ");
    Serial.print("Error:"); Serial.print(e_now);
  }
}