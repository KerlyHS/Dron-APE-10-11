const int POT_X = A0;
const int POT_Y = A1;
const int TRIG_PIN = A2;
const int ECHO_PIN = A3;
const int BUZZER = 10;
const int LED_ROJO = 12;
const int LED_VERDE = 13;
const int M1_IN1 = 2;
const int M1_IN2 = 3;
const int M2_IN1 = 4;
const int M2_IN2 = 5;
const int M3_IN1 = 6;
const int M3_IN2 = 7;
const int M4_IN1 = 8;
const int M4_IN2 = 9;
// Tiempos que simulan vTaskDelay en FreeRTOS
const unsigned long PERIODO_SENSORES = 100;
const unsigned long PERIODO_PROCESO = 200;
const unsigned long PERIODO_ACTUACION = 300;
unsigned long tiempoSensores = 0;
unsigned long tiempoProceso = 0;
unsigned long tiempoActuacion = 0;

// Cola simulada: sensores -> procesamiento
int colaEjeX = 0;
int colaEjeY = 0;
long colaDistancia = 0;
bool colaSensoresDisponible = false;
// Cola simulada: procesamiento -> actuación
bool peligroDistancia = false;
bool inclinacionX = false;
bool inclinacionY = false;
bool sistemaEstable = true;
bool colaEventosDisponible = false;
int estado = 0;
const int CENTRO = 512;
const int UMBRAL_INCLINACION = 180;
const int DISTANCIA_PELIGRO = 20;

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(BUZZER, OUTPUT);
  pinMode(LED_ROJO, OUTPUT);
  pinMode(LED_VERDE, OUTPUT);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);
  detenerMotores();
  digitalWrite(LED_VERDE, HIGH);
  digitalWrite(LED_ROJO, LOW);
  digitalWrite(BUZZER, LOW);

  Serial.println("Sistema de dron iniciado");
}

// Planificador principal: ejecuta tareas según prioridad y periodo
void loop() {
  unsigned long ahora = millis();

  if (ahora - tiempoSensores >= PERIODO_SENSORES) {
    tiempoSensores = ahora;
    tareaSensores();
  }

  if (ahora - tiempoProceso >= PERIODO_PROCESO) {
    tiempoProceso = ahora;
    tareaProcesamiento();
  }

  if (ahora - tiempoActuacion >= PERIODO_ACTUACION) {
    tiempoActuacion = ahora;
    tareaActuacion();
  }
}

// Tarea 1: adquisición de datos, prioridad alta
void tareaSensores() {
  colaEjeX = analogRead(POT_X);
  colaEjeY = analogRead(POT_Y);
  colaDistancia = leerDistancia();

  colaSensoresDisponible = true;
}

// Tarea 2: procesamiento de datos, prioridad media
void tareaProcesamiento() {
  if (!colaSensoresDisponible) return;

  colaSensoresDisponible = false;

  peligroDistancia = false;
  inclinacionX = false;
  inclinacionY = false;
  sistemaEstable = false;
  estado = 0;

  if (colaDistancia > 0 && colaDistancia < DISTANCIA_PELIGRO) {
    peligroDistancia = true;
    estado = 1;
  }

  if (abs(colaEjeX - CENTRO) > UMBRAL_INCLINACION) {
    inclinacionX = true;
    estado = 2;
  }

  if (abs(colaEjeY - CENTRO) > UMBRAL_INCLINACION) {
    inclinacionY = true;
    estado = 3;
  }

  if (!peligroDistancia && !inclinacionX && !inclinacionY) {
    sistemaEstable = true;
    estado = 0;
  }

  colaEventosDisponible = true;
}

// Tarea 3: actuación y comunicación serial, prioridad baja
void tareaActuacion() {
  if (!colaEventosDisponible) return;

  colaEventosDisponible = false;

  if (sistemaEstable) {
    normal();
  } else {
    alerta();
  }

  telemetria();
}

// Lectura del sensor ultrasónico
long leerDistancia() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duracion = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duracion == 0) return 999;

  return duracion * 0.034 / 2;
}

// Estado normal: motores activos, LED verde encendido
void normal() {
  digitalWrite(LED_VERDE, HIGH);
  digitalWrite(LED_ROJO, LOW);
  digitalWrite(BUZZER, LOW);

  motorAdelante(M1_IN1, M1_IN2);
  motorAdelante(M2_IN1, M2_IN2);
  motorAdelante(M3_IN1, M3_IN2);
  motorAdelante(M4_IN1, M4_IN2);
}

// Estado de alerta: activa buzzer, LED rojo y corrige motores
void alerta() {
  digitalWrite(LED_VERDE, LOW);
  digitalWrite(LED_ROJO, HIGH);
  digitalWrite(BUZZER, HIGH);

  if (peligroDistancia) {
    detenerMotores();
  } else if (inclinacionX) {
    motorAdelante(M1_IN1, M1_IN2);
    motorDetenido(M2_IN1, M2_IN2);
    motorAdelante(M3_IN1, M3_IN2);
    motorDetenido(M4_IN1, M4_IN2);
  } else if (inclinacionY) {
    motorDetenido(M1_IN1, M1_IN2);
    motorAdelante(M2_IN1, M2_IN2);
    motorDetenido(M3_IN1, M3_IN2);
    motorAdelante(M4_IN1, M4_IN2);
  }
}

// Telemetría del sistema por Serial Monitor
void telemetria() {
  Serial.print("X:");
  Serial.print(colaEjeX);

  Serial.print(" Y:");
  Serial.print(colaEjeY);

  Serial.print(" Distancia:");
  Serial.print(colaDistancia);
  Serial.print("cm ");

  Serial.print(" Estado:");

  if (estado == 0) Serial.println("Estable");
  else if (estado == 1) Serial.println("Obstaculo");
  else if (estado == 2) Serial.println("Inclinacion X");
  else if (estado == 3) Serial.println("Inclinacion Y");
}

// Control básico de motores mediante L293D
void motorAdelante(int in1, int in2) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void motorDetenido(int in1, int in2) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void detenerMotores() {
  motorDetenido(M1_IN1, M1_IN2);
  motorDetenido(M2_IN1, M2_IN2);
  motorDetenido(M3_IN1, M3_IN2);
  motorDetenido(M4_IN1, M4_IN2);
}
