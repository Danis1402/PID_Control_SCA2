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
const float CPR = 70.0; 

// --- Variables de Control PID ---
const int   INTERVALO_MS = 20;   // Ts = 100ms (0.1s)
float setpoint = 800;             // Velocidad deseada en RPM

// Coeficientes obtenidos de MATLAB
const float f = 1;
const float b0 = 0.0628 * f; 
const float b1 = -0.0263 * f;

float u_prev = 0; // Salida anterior (u[n-1])
float e_prev = 0; // Error anterior (e[n-1])

// --- Variables del Filtro Paso Bajo ---
float rpmFiltrada = 0;
const float alpha = 0.4; // Factor de filtrado (0.1 = mucho filtro, 0.9 = poco filtro, 1 = no hay filtro)

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
  
  ledcAttach(IN1, FREQ_HZ, RESOLUCION);
  ledcAttach(IN2, FREQ_HZ, RESOLUCION);
  frenar();

  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_A), isrEncoder, RISING);

  Serial.println("Sistema de Control PI con Filtro Iniciado...");
  delay(1000);
}

// ──────────────────────────────────────────
void loop() {
  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoUltimoControl >= INTERVALO_MS) {
    float dt = (tiempoActual - tiempoUltimoControl) / 1000.0;
    tiempoUltimoControl = tiempoActual;

    // 1. Leer velocidad actual (bruta)
    float rpmMedida = calcularRPM(dt);

    // 2. Aplicar Filtro Paso Bajo
    // rpmFiltrada = (1 - alpha) * anterior + alpha * actual
    rpmFiltrada = ( (1.0 - alpha) * rpmFiltrada ) + ( alpha * rpmMedida );

    // 3. Calcular error usando la señal FILTRADA
    float e_now = setpoint - rpmFiltrada;

    // 4. Ecuación en diferencias del PI
    float u_now = u_prev + (b0 * e_now) + (b1 * e_prev);

    // 5. Saturación (Anti-windup)
    if (u_now > 255) u_now = 255;
    if (u_now < 0)   u_now = 0;

    // 6. Actuar sobre el motor
    adelante((int)u_now);

    // 7. Actualizar estados
    u_prev = u_now;
    e_prev = e_now;

    // 8. Monitorización
    // Imprimimos la medida real y la filtrada para comparar en el Serial Plotter
    Serial.print("Setpoint:");  Serial.print(setpoint);    Serial.print(",");
    Serial.print("RPM_Filt:");  Serial.print(rpmFiltrada); Serial.print(",");
    Serial.print("Error:");     Serial.print(e_now);       Serial.print(",");
    Serial.print("PWM:");       Serial.println(u_now);
  }
}