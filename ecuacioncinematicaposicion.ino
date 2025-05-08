// Configuración de pines
const int enA = 6;
const int in1 = 4;
const int in2 = 5;
const int enB = 9;
const int in3 = 7;
const int in4 = 8;

// --- Constantes para Cinemática ---
// ¡¡AJUSTA ESTOS VALORES PARA TU ROBOT!!
const float WHEEL_BASE = 0.15;      // Metros (ej: 15 cm)
const float MAX_MOTOR_SPEED_MPS = 0.5; // Metros por segundo (velocidad lineal máxima estimada de una rueda a PWM 255)
                                       // Este es un valor CRÍTICO y difícil de estimar sin pruebas.
                                       // Puedes empezar con un valor y ajustarlo.

// --- Variables de Estado del Robot (Pose) ---
float robotX = 0.0;     // Posición X del robot en metros
float robotY = 0.0;     // Posición Y del robot en metros
float robotTheta = 0.0; // Orientación del robot en radianes (0 = hacia el eje X positivo)

// --- Variables de Control de Motores ---
int currentPwmLeft = 0;
int currentPwmRight = 0;
// Para la dirección: 1 para adelante, -1 para atrás, 0 para detenido (en términos de cálculo de velocidad)
int dirLeft = 0;
int dirRight = 0;

// --- Temporización para Cinemática ---
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

// --- Funciones de Control de Motores (Modificadas para guardar PWM y dirección) ---
void moverAdelante(int velocidadIzquierda, int velocidadDerecha) {
  // Asegurarse que la velocidad no exceda 255
  velocidadIzquierda = constrain(velocidadIzquierda, 0, 255);
  velocidadDerecha = constrain(velocidadDerecha, 0, 255);

  Serial.print("Moviendo Adelante - PWM Izq: "); Serial.print(velocidadIzquierda);
  Serial.print(" PWM Der: "); Serial.println(velocidadDerecha);

  digitalWrite(in1, HIGH); // Asume que esta es tu config para adelante
  digitalWrite(in2, LOW);
  analogWrite(enA, velocidadIzquierda);

  digitalWrite(in3, HIGH); // Asume que esta es tu config para adelante
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

void girarIzquierda(int velocidad) { // Gira sobre su eje
  velocidad = constrain(velocidad, 0, 255);
  Serial.print("Girando Izquierda - Velocidad: "); Serial.println(velocidad);

  digitalWrite(in1, LOW);  // Izquierda atrás
  digitalWrite(in2, HIGH);
  analogWrite(enA, velocidad);

  digitalWrite(in3, HIGH); // Derecha adelante
  digitalWrite(in4, LOW);
  analogWrite(enB, velocidad);

  currentPwmLeft = velocidad;
  currentPwmRight = velocidad;
  dirLeft = -1; // Rueda izquierda hacia atrás
  dirRight = 1; // Rueda derecha hacia adelante
}

void girarDerecha(int velocidad) { // Gira sobre su eje
  velocidad = constrain(velocidad, 0, 255);
  Serial.print("Girando Derecha - Velocidad: "); Serial.println(velocidad);

  digitalWrite(in1, HIGH); // Izquierda adelante
  digitalWrite(in2, LOW);
  analogWrite(enA, velocidad);

  digitalWrite(in3, LOW);  // Derecha atrás
  digitalWrite(in4, HIGH);
  analogWrite(enB, velocidad);

  currentPwmLeft = velocidad;
  currentPwmRight = velocidad;
  dirLeft = 1;  // Rueda izquierda hacia adelante
  dirRight = -1; // Rueda derecha hacia atrás
}

void detenerMotores() {
  Serial.println("Deteniendo Motores");
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW); // Opcional, para frenado más activo si L298N lo soporta bien
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  currentPwmLeft = 0;
  currentPwmRight = 0;
  dirLeft = 0;
  dirRight = 0;
}

// --- Función de Actualización de Cinemática ---
void actualizarCinematica() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTimeKinematics) / 1000.0; // Delta t en segundos
  lastTimeKinematics = currentTime;

  if (deltaTime == 0) return; // Evitar división por cero si el tiempo no ha cambiado

  // 1. Convertir PWM a velocidad lineal estimada para cada rueda (m/s)
  //    Asumimos relación lineal: velocidad = (PWM / 255) * MAX_MOTOR_SPEED_MPS
  float vL_mps = dirLeft  * (currentPwmLeft / 255.0) * MAX_MOTOR_SPEED_MPS;
  float vR_mps = dirRight * (currentPwmRight / 255.0) * MAX_MOTOR_SPEED_MPS;

  // 2. Calcular velocidad lineal del robot (v) y velocidad angular (omega)
  float v_robot_mps = (vR_mps + vL_mps) / 2.0;
  float omega_robot_radps = (vR_mps - vL_mps) / WHEEL_BASE; // Radianes por segundo

  // 3. Calcular cambio en orientación y posición
  float deltaTheta = omega_robot_radps * deltaTime;
  // Usar el promedio de la orientación para mayor precisión (método de Euler mejorado)
  float avgTheta = robotTheta + deltaTheta / 2.0;

  float deltaX = v_robot_mps * cos(avgTheta) * deltaTime;
  float deltaY = v_robot_mps * sin(avgTheta) * deltaTime;

  // 4. Actualizar la pose del robot
  robotX += deltaX;
  robotY += deltaY;
  robotTheta += deltaTheta;

  // Normalizar theta para que esté entre -PI y PI (opcional, pero bueno para consistencia)
  while (robotTheta > PI) robotTheta -= 2 * PI;
  while (robotTheta < -PI) robotTheta += 2 * PI;
}

void loop() {
  // Actualizar la cinemática en cada ciclo
  actualizarCinematica();

  // Secuencia de prueba de movimiento
  moverAdelante(180, 180); // Moverse adelante
  delay(2000);             // durante 2 segundos
  actualizarCinematica();  // Actualizar una vez más justo después del movimiento para capturar el último estado
  imprimirPose();

  detenerMotores();
  delay(1000);
  actualizarCinematica();
  imprimirPose();

  girarDerecha(150);       // Girar a la derecha
  delay(1000);             // durante 1 segundo (debería ser aprox 90 grados si WHEEL_BASE y MAX_MOTOR_SPEED_MPS son razonables)
  actualizarCinematica();
  imprimirPose();

  detenerMotores();
  delay(1000);
  actualizarCinematica();
  imprimirPose();

  moverAdelante(180,180); // Moverse adelante de nuevo
  delay(1500);
  actualizarCinematica();
  imprimirPose();

  detenerMotores();
  delay(1000);
  actualizarCinematica();
  imprimirPose();


  Serial.println("--- Fin de Secuencia --- Esperando 10s ---");
  delay(10000); // Esperar antes de repetir
}

void imprimirPose() {
  Serial.print("Pose Estimada: X=");
  Serial.print(robotX, 4); // Imprimir con 4 decimales
  Serial.print(" m, Y=");
  Serial.print(robotY, 4);
  Serial.print(" m, Theta=");
  Serial.print(robotTheta * 180.0 / PI, 2); // Convertir a grados para facilitar la lectura
  Serial.println(" deg");
}