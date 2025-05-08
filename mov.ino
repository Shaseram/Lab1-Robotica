// --- AJUSTES DE VELOCIDAD Y DURACIÓN ---
// Velocidad (0 = detenido, 255 = máximo)
int velocidadAvance = 120; // Prueba con un valor entre 100 y 200 para empezar
int velocidadGiro = 100;   // Los giros suelen necesitar menos velocidad para ser precisos

// Duraciones en milisegundos (ms)
const int DURACION_AVANCE = 1500;   // Antes 3000ms
const int DURACION_ATRAS = 1200;    // Antes 2000ms
const int DURACION_GIRO = 800;      // Antes 1500ms (ajusta esto para giros de ~90 grados)
const int DURACION_PAUSA = 500;     // Antes 1000ms
const int DURACION_PAUSA_CICLO = 1500; // Antes 3000ms

// --- Definición de Pines (como los tenías) ---
// Motor A (Izquierdo)
const int motorA_IN1 = 4;  // Pin de dirección 1 para Motor A
const int motorA_IN2 = 5;  // Pin de dirección 2 para Motor A
const int motorA_ENA = 6; // Pin Enable/PWM para velocidad de Motor A (PWM)

// Motor B (Derecho)
const int motorB_IN3 = 7;  // Pin de dirección 1 para Motor B
const int motorB_IN4 = 8;  // Pin de dirección 2 para Motor B
const int motorB_ENB = 9; // Pin Enable/PWM para velocidad de Motor B (PWM)

void setup() {
  // Configurar todos los pines de control como SALIDA
  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorA_ENA, OUTPUT);

  pinMode(motorB_IN3, OUTPUT);
  pinMode(motorB_IN4, OUTPUT);
  pinMode(motorB_ENB, OUTPUT);

  Serial.begin(9600); // Para mensajes en el Monitor Serie
  Serial.println("Setup completo. Listo para prueba de movimientos con velocidad controlada.");
  Serial.print("Velocidad Avance/Retroceso: "); Serial.println(velocidadAvance);
  Serial.print("Velocidad Giro: "); Serial.println(velocidadGiro);

  // Empezar con los motores detenidos
  detener();
}

void loop() {
  Serial.println("MOVIMIENTO: ADELANTE");
  adelante();
  delay(DURACION_AVANCE);

  detener();
  Serial.println("DETENIDO");
  delay(DURACION_PAUSA);

  Serial.println("MOVIMIENTO: ATRAS");
  atras();
  delay(DURACION_ATRAS);

  detener();
  Serial.println("DETENIDO");
  delay(DURACION_PAUSA);

  Serial.println("MOVIMIENTO: GIRO DERECHA");
  girarDerecha();
  delay(DURACION_GIRO);

  detener();
  Serial.println("DETENIDO");
  delay(DURACION_PAUSA);

  Serial.println("MOVIMIENTO: GIRO IZQUIERDA");
  girarIzquierda();
  delay(DURACION_GIRO);

  detener();
  Serial.println("DETENIDO");
  delay(DURACION_PAUSA_CICLO);
}

// --- FUNCIONES DE MOVIMIENTO DEL ROBOT ---

void detener() {
  // Apagar ambos motores poniendo su velocidad a 0
  analogWrite(motorA_ENA, 0);
  analogWrite(motorB_ENB, 0);

  // Opcional pero buena práctica: poner los pines de dirección en bajo también
  digitalWrite(motorA_IN1, LOW);
  digitalWrite(motorA_IN2, LOW);
  digitalWrite(motorB_IN3, LOW);
  digitalWrite(motorB_IN4, LOW);
}

void adelante() {
  // Motor A (Izquierdo) - Lógica estándar para su "adelante"
  digitalWrite(motorA_IN1, HIGH);
  digitalWrite(motorA_IN2, LOW);
  analogWrite(motorA_ENA, velocidadAvance); // Usar velocidadAvance

  // Motor B (Derecho) - Lógica INVERTIDA para su "adelante"
  digitalWrite(motorB_IN3, LOW);
  digitalWrite(motorB_IN4, HIGH);
  analogWrite(motorB_ENB, velocidadAvance); // Usar velocidadAvance
}

void atras() {
  // Motor A (Izquierdo) - Lógica estándar para su "atrás"
  digitalWrite(motorA_IN1, LOW);
  digitalWrite(motorA_IN2, HIGH);
  analogWrite(motorA_ENA, velocidadAvance); // Usar velocidadAvance

  // Motor B (Derecho) - Lógica INVERTIDA para su "atrás"
  digitalWrite(motorB_IN3, HIGH);
  digitalWrite(motorB_IN4, LOW);
  analogWrite(motorB_ENB, velocidadAvance); // Usar velocidadAvance
}

void girarDerecha() {
  // Para girar a la DERECHA: Motor Izquierdo ADELANTE, Motor Derecho ATRÁS

  // Motor A (Izquierdo) - ADELANTE
  digitalWrite(motorA_IN1, HIGH);
  digitalWrite(motorA_IN2, LOW);
  analogWrite(motorA_ENA, velocidadGiro); // Usar velocidadGiro

  // Motor B (Derecho) - ATRÁS
  digitalWrite(motorB_IN3, HIGH);
  digitalWrite(motorB_IN4, LOW);
  analogWrite(motorB_ENB, velocidadGiro); // Usar velocidadGiro
}

void girarIzquierda() {
  // Para girar a la IZQUIERDA: Motor Izquierdo ATRÁS, Motor Derecho ADELANTE

  // Motor A (Izquierdo) - ATRÁS
  digitalWrite(motorA_IN1, LOW);
  digitalWrite(motorA_IN2, HIGH);
  analogWrite(motorA_ENA, velocidadGiro); // Usar velocidadGiro

  // Motor B (Derecho) - ADELANTE
  digitalWrite(motorB_IN3, LOW);
  digitalWrite(motorB_IN4, HIGH);
  analogWrite(motorB_ENB, velocidadGiro); // Usar velocidadGiro
}