#include <Arduino.h>

// --- Configuración de Pines ---
const int IN1 = 10;
const int IN2 = 9;
const int PIN_A = 6;
const int PIN_B = 7;

// --- Configuración PWM ---
const int FREQ_HZ    = 5000;
const int RESOLUCION = 8;

// --- Configuración Encoder ---
const float CPR          = 70.0; 
const int   INTERVALO_MS = 10;   // Ts = 0.01s (Mismo que usamos en MATLAB)

// --- Variables de Estado del Sistema ---
volatile long conteo = 0;
unsigned long tiempoAnterior = 0;

// --- Parámetros del Filtro Paso Bajo ---
float rpmFiltrada = 0;
const float alpha = 0.2; 

// --- Parámetros del Controlador PI ---
float setpoint = 800.0;  // Velocidad deseada en RPM
float u_prev = 0;        // u[n-1]
float e_prev = 0;        // e[n-1]

// Coeficientes de MATLAB escalados por el factor f=4.45
const float f  = 0.5; 
const float b0 = 0.0628 * f; 
const float b1 = -0.0446 * f;

// ──────────────────────────────────────────
void IRAM_ATTR isrEncoder() {
  if (digitalRead(PIN_B) == HIGH) conteo++;
  else                            conteo--;
}

// ──────────────────────────────────────────
float calcularRPM(float dt) {
  static long conteoAnterior = 0;
  long snapshot;
  noInterrupts();
    snapshot = conteo;
  interrupts();
  long delta = snapshot - conteoAnterior;
  float rpm = (delta / CPR) * (60.0 / dt);
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
  ledcAttach(IN1, FREQ_HZ, RESOLUCION);
  ledcAttach(IN2, FREQ_HZ, RESOLUCION);
  frenar();

  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_A), isrEncoder, RISING);
  
  delay(1000);
  Serial.println("Control PI Iniciado...");
}

// ──────────────────────────────────────────
void loop() {
  unsigned long ahora = millis();

  if (ahora - tiempoAnterior >= INTERVALO_MS) {
    float dt = (ahora - tiempoAnterior) / 1000.0;
    tiempoAnterior = ahora;

    // 1. Obtener medida y filtrar (RETROALIMENTACIÓN)
    float rpmMedida = calcularRPM(dt);
    rpmFiltrada = ( (1.0 - alpha) * rpmFiltrada ) + ( alpha * rpmMedida );

    // 2. Calcular Error actual: e[n]
    float e_now = setpoint - rpmFiltrada;

    // 3. ECUACIÓN EN DIFERENCIAS (CONTROLADOR)
    // u[n] = u[n-1] + b0*e[n] + b1*e[n-1]
    float u_now = u_prev + (b0 * e_now) + (b1 * e_prev);

    // 4. Saturación y Anti-Windup implícito
    if (u_now > 255) u_now = 255;
    if (u_now < 0)   u_now = 0;

    // 5. Actuar sobre el motor (SALIDA)
    adelante((int)u_now);

    // 6. Actualizar estados para el siguiente ciclo
    u_prev = u_now;
    e_prev = e_now;

    // 7. Monitorización para Serial Plotter
    Serial.print("Setpoint:");    Serial.print(setpoint);    Serial.print(",");
    Serial.print("RPM_Filtrada:"); Serial.print(rpmFiltrada); Serial.print(",");
    Serial.print("PWM:");         Serial.println(u_now);
  }
}