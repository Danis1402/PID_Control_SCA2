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
const int   INTERVALO_MS = 10;   // Tiempo de muestreo (10ms)

// --- Variables de Estado ---
volatile long conteo = 0;
unsigned long tiempoAnterior = 0;

// --- Variables del Filtro Paso Bajo ---
float rpmFiltrada = 0;
const float alpha = 0.1; // Factor de suavizado (0.1 a 0.3 recomendado)

// ──────────────────────────────────────────
void IRAM_ATTR isrEncoder() {
  // Leemos el canal B para determinar el sentido
  if (digitalRead(PIN_B) == HIGH) conteo++;
  else                            conteo--;
}

// ──────────────────────────────────────────
float calcularRPM(float dt) {
  static long conteoAnterior = 0;
  long snapshot;

  // Bloqueo de interrupciones para lectura segura del contador
  noInterrupts();
    snapshot = conteo;
  interrupts();

  long delta = snapshot - conteoAnterior;
  
  // Cálculo de RPM: (pulsos / CPR) * (60 segundos / dt)
  float rpm = (delta / CPR) * (60.0 / dt);

  conteoAnterior = snapshot;
  return rpm;
}

// ──────────────────────────────────────────
void adelante(int velocidad) {
  ledcWrite(IN1, constrain(velocidad, 0, 255));
  ledcWrite(IN2, 0);
}

// ──────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  
  // Configuración de salidas PWM
  ledcAttach(IN1, FREQ_HZ, RESOLUCION);
  ledcAttach(IN2, FREQ_HZ, RESOLUCION);

  // Configuración de entradas Encoder
  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_A), isrEncoder, RISING);
}

// ──────────────────────────────────────────
void loop() {
  unsigned long ahora = millis();

  // Ejecución precisa cada 10ms
  if (ahora - tiempoAnterior >= INTERVALO_MS) {
    float dt = (ahora - tiempoAnterior) / 1000.0;
    tiempoAnterior = ahora;

    // Aplicamos una velocidad constante de prueba
    adelante(180);

    // 1. Obtenemos la medida bruta (con saltos de cuantización)
    float rpmMedida = calcularRPM(dt);

    // 2. Aplicamos el Filtro Paso Bajo
    // Ecuación: y[n] = (1 - alpha) * y[n-1] + alpha * x[n]
    rpmFiltrada = ( (1.0 - alpha) * rpmFiltrada ) + ( alpha * rpmMedida );

    // 3. Monitorización para el Serial Plotter
    // Imprimimos ambas para comparar el efecto del filtro
    Serial.print("RPM_Bruta:");    Serial.print(rpmMedida);   Serial.print(",");
    Serial.print("RPM_Filtrada:"); Serial.println(rpmFiltrada);
  }
}