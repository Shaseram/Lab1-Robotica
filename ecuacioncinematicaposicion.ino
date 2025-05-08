const int enA = 6;
const int in1 = 4;
const int in2 = 5;
const int enB = 9;
const int in3 = 7;
const int in4 = 8;

const float WHEEL_BASE = 0.15;
const float MAX_MOTOR_SPEED_MPS = 0.5;

float robotX = 0.0;
float robotY = 0.0;
float robotTheta = 0.0;

int currentPwmLeft = 0;
int currentPwmRight = 0;
int dirLeft = 0;
int dirRight = 0;

unsigned long lastTimeKinematics = 0;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.begin(9600);
  Serial.println("Estimacion de Posicion con Cinematica Diferencial");
  Serial.println("Robot inicia en (0,0) Theta=0");

  lastTimeKinematics = millis();
}

void moverAdelante(int velocidadIzquierda, int velocidadDerecha) {
  velocidadIzquierda = constrain(velocidadIzquierda, 0, 255);
  velocidadDerecha = constrain(velocidadDerecha, 0, 255);

  Serial.print("Moviendo Adelante - PWM Izq: "); Serial.print(velocidadIzquierda);
  Serial.print(" PWM Der: "); Serial.println(velocidadDerecha);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, velocidadIzquierda);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, velocidadDerecha);

  currentPwmLeft = velocidadIzquierda;
  currentPwmRight = velocidadDerecha;
  dirLeft = 1;
  dirRight = 1;
}

void moverAtras(int velocidadIzquierda, int velocidadDerecha) {
  velocidadIzquierda = constrain(velocidadIzquierda, 0, 255);
  velocidadDerecha = constrain(velocidadDerecha, 0, 255);

  Serial.print("Moviendo Atras - PWM Izq: "); Serial.print(velocidadIzquierda);
  Serial.print(" PWM Der: "); Serial.println(velocidadDerecha);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, velocidadIzquierda);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, velocidadDerecha);

  currentPwmLeft = velocidadIzquierda;
  currentPwmRight = velocidadDerecha;
  dirLeft = -1;
  dirRight = -1;
}

void girarIzquierda(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);
  Serial.print("Girando Izquierda - Velocidad: "); Serial.println(velocidad);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, velocidad);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, velocidad);

  currentPwmLeft = velocidad;
  currentPwmRight = velocidad;
  dirLeft = -1;
  dirRight = 1;
}

void girarDerecha(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);
  Serial.print("Girando Derecha - Velocidad: "); Serial.println(velocidad);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, velocidad);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, velocidad);

  currentPwmLeft = velocidad;
  currentPwmRight = velocidad;
  dirLeft = 1;
  dirRight = -1;
}

void detenerMotores() {
  Serial.println("Deteniendo Motores");
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  currentPwmLeft = 0;
  currentPwmRight = 0;
  dirLeft = 0;
  dirRight = 0;
}

void actualizarCinematica() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTimeKinematics) / 1000.0;
  lastTimeKinematics = currentTime;

  if (deltaTime == 0) return;

  float vL_mps = dirLeft  * (currentPwmLeft / 255.0) * MAX_MOTOR_SPEED_MPS;
  float vR_mps = dirRight * (currentPwmRight / 255.0) * MAX_MOTOR_SPEED_MPS;

  float v_robot_mps = (vR_mps + vL_mps) / 2.0;
  float omega_robot_radps = (vR_mps - vL_mps) / WHEEL_BASE;

  float deltaTheta = omega_robot_radps * deltaTime;
  float avgTheta = robotTheta + deltaTheta / 2.0;

  float deltaX = v_robot_mps * cos(avgTheta) * deltaTime;
  float deltaY = v_robot_mps * sin(avgTheta) * deltaTime;

  robotX += deltaX;
  robotY += deltaY;
  robotTheta += deltaTheta;

  while (robotTheta > PI) robotTheta -= 2 * PI;
  while (robotTheta < -PI) robotTheta += 2 * PI;
}

void loop() {
  actualizarCinematica();

  moverAdelante(180, 180);
  delay(2000);
  actualizarCinematica();
  imprimirPose();

  detenerMotores();
  delay(1000);
  actualizarCinematica();
  imprimirPose();

  girarDerecha(150);
  delay(1000);
  actualizarCinematica();
  imprimirPose();

  detenerMotores();
  delay(1000);
  actualizarCinematica();
  imprimirPose();

  moverAdelante(180,180);
  delay(1500);
  actualizarCinematica();
  imprimirPose();

  detenerMotores();
  delay(1000);
  actualizarCinematica();
  imprimirPose();


  Serial.println("--- Fin de Secuencia --- Esperando 10s ---");
  delay(10000);
}

void imprimirPose() {
  Serial.print("Pose Estimada: X=");
  Serial.print(robotX, 4);
  Serial.print(" m, Y=");
  Serial.print(robotY, 4);
  Serial.print(" m, Theta=");
  Serial.print(robotTheta * 180.0 / PI, 2);
  Serial.println(" deg");
}