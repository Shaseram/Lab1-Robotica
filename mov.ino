int velocidadAvance = 120;
int velocidadGiro = 100;

const int DURACION_AVANCE = 1500;
const int DURACION_ATRAS = 1200;
const int DURACION_GIRO = 800;
const int DURACION_PAUSA = 500;
const int DURACION_PAUSA_CICLO = 1500;

const int motorA_IN1 = 4;
const int motorA_IN2 = 5;
const int motorA_ENA = 6;

const int motorB_IN3 = 7;
const int motorB_IN4 = 8;
const int motorB_ENB = 9;

void setup() {
  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorA_ENA, OUTPUT);

  pinMode(motorB_IN3, OUTPUT);
  pinMode(motorB_IN4, OUTPUT);
  pinMode(motorB_ENB, OUTPUT);

  Serial.begin(9600);
  Serial.println("Setup completo. Listo para prueba de movimientos con velocidad controlada.");
  Serial.print("Velocidad Avance/Retroceso: "); Serial.println(velocidadAvance);
  Serial.print("Velocidad Giro: "); Serial.println(velocidadGiro);

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

void detener() {
  analogWrite(motorA_ENA, 0);
  analogWrite(motorB_ENB, 0);

  digitalWrite(motorA_IN1, LOW);
  digitalWrite(motorA_IN2, LOW);
  digitalWrite(motorB_IN3, LOW);
  digitalWrite(motorB_IN4, LOW);
}

void adelante() {
  digitalWrite(motorA_IN1, HIGH);
  digitalWrite(motorA_IN2, LOW);
  analogWrite(motorA_ENA, velocidadAvance);

  digitalWrite(motorB_IN3, LOW);
  digitalWrite(motorB_IN4, HIGH);
  analogWrite(motorB_ENB, velocidadAvance);
}

void atras() {
  digitalWrite(motorA_IN1, LOW);
  digitalWrite(motorA_IN2, HIGH);
  analogWrite(motorA_ENA, velocidadAvance);

  digitalWrite(motorB_IN3, HIGH);
  digitalWrite(motorB_IN4, LOW);
  analogWrite(motorB_ENB, velocidadAvance);
}

void girarDerecha() {
  digitalWrite(motorA_IN1, HIGH);
  digitalWrite(motorA_IN2, LOW);
  analogWrite(motorA_ENA, velocidadGiro);

  digitalWrite(motorB_IN3, HIGH);
  digitalWrite(motorB_IN4, LOW);
  analogWrite(motorB_ENB, velocidadGiro);
}

void girarIzquierda() {
  digitalWrite(motorA_IN1, LOW);
  digitalWrite(motorA_IN2, HIGH);
  analogWrite(motorA_ENA, velocidadGiro);

  digitalWrite(motorB_IN3, LOW);
  digitalWrite(motorB_IN4, HIGH);
  analogWrite(motorB_ENB, velocidadGiro);
}