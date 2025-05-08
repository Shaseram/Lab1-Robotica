#include <MPU9250_asukiaaa.h>
#include <Wire.h>
#include <math.h>

const int motorA_IN1 = 4;
const int motorA_IN2 = 5;
const int motorA_ENA = 6;
const int motorB_IN3 = 7;
const int motorB_IN4 = 8;
const int motorB_ENB = 9;

const float WHEEL_BASE = 0.15;
const float WHEEL_DIAMETER = 0.065;
const float WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;
const float PWM_TO_SPEED_FACTOR = 0.0020;

MPU9250_asukiaaa mpu;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
const int CALIBRATION_SAMPLES = 1000;
float currentRoll = 0, currentPitch = 0, currentYaw = 0;
float currentYawRate = 0;
const float K_COMP = 0.98;

float robotX = 0.0;
float robotY = 0.0;
float robotTheta = 0.0;

unsigned long tiempoPrevioLoop = 0;
unsigned long tiempoPrevioIMU = 0;

float targetYaw = 0.0;
const float KP_YAW = 2.5;

const float KP_PITCH_POWER = 1.5;
const float KP_ROLL_STEER = 1.0;
const float MAX_PITCH_FOR_ADJUST = 20.0;
const float MAX_ROLL_FOR_STEER = 15.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(motorA_IN1, OUTPUT); pinMode(motorA_IN2, OUTPUT); pinMode(motorA_ENA, OUTPUT);
  pinMode(motorB_IN3, OUTPUT); pinMode(motorB_IN4, OUTPUT); pinMode(motorB_ENB, OUTPUT);

  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();
  delay(100);

  bool sensorOk = true;
  if (mpu.accelUpdate() != 0) { sensorOk = false; Serial.println("Error Accel Update Setup"); }
  if (mpu.gyroUpdate() != 0) { sensorOk = false; Serial.println("Error Gyro Update Setup"); }

  if (sensorOk) {
    Serial.println("Sensor MPU iniciado.");
    calibrateGyroscope();
  } else {
    Serial.println("ERROR CRITICO MPU. Deteniendo.");
    while (1);
  }

  stopRobot();
  tiempoPrevioLoop = millis();
  tiempoPrevioIMU = millis();
  Serial.println("Setup completo. Iniciando demostraciones...");
}

void calibrateGyroscope() {
  Serial.println("Calibrando Giroscopio... No mover el sensor.");
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    if (mpu.gyroUpdate() == 0) {
      sumX += mpu.gyroX(); sumY += mpu.gyroY(); sumZ += mpu.gyroZ();
    } else { i--; }
    delay(3);
  }
  gyroBiasX = (float)sumX / CALIBRATION_SAMPLES;
  gyroBiasY = (float)sumY / CALIBRATION_SAMPLES;
  gyroBiasZ = (float)sumZ / CALIBRATION_SAMPLES;
  Serial.println("Calibracion Gyro completa:");
  Serial.print("Bias X: "); Serial.print(gyroBiasX, 4);
  Serial.print(" | Y: "); Serial.print(gyroBiasY, 4);
  Serial.print(" | Z: "); Serial.println(gyroBiasZ, 4);
}

void updateIMU(float dt_imu) {
  if (mpu.accelUpdate() != 0 || mpu.gyroUpdate() != 0) {
    Serial.println("Error al leer IMU en updateIMU");
    return;
  }
  float ax = mpu.accelX(); float ay = mpu.accelY(); float az = mpu.accelZ();
  float gx = mpu.gyroX() - gyroBiasX;
  float gy = mpu.gyroY() - gyroBiasY;
  float gz = mpu.gyroZ() - gyroBiasZ;
  currentYawRate = gz;
  float accelRoll = atan2(ay, az) * (180.0 / M_PI);
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * (180.0 / M_PI);
  float gyroDeltaRoll = gx * dt_imu;
  float gyroDeltaPitch = gy * dt_imu;
  float gyroDeltaYaw = gz * dt_imu;
  currentRoll = K_COMP * (currentRoll + gyroDeltaRoll) + (1.0 - K_COMP) * accelRoll;
  currentPitch = K_COMP * (currentPitch + gyroDeltaPitch) + (1.0 - K_COMP) * accelPitch;
  currentYaw += gyroDeltaYaw;
}

void setMotor(int motor, int direccion, int pwm) {
  byte pinIn1, pinIn2, pinEna;
  bool logicaAdelanteEsHighLow;
  if (motor == 0) {
    pinIn1 = motorA_IN1; pinIn2 = motorA_IN2; pinEna = motorA_ENA;
    logicaAdelanteEsHighLow = true;
  } else {
    pinIn1 = motorB_IN3; pinIn2 = motorB_IN4; pinEna = motorB_ENB;
    logicaAdelanteEsHighLow = false;
  }
  if (direccion == 1) {
    digitalWrite(pinIn1, logicaAdelanteEsHighLow ? HIGH : LOW);
    digitalWrite(pinIn2, logicaAdelanteEsHighLow ? LOW : HIGH);
  } else if (direccion == -1) {
    digitalWrite(pinIn1, logicaAdelanteEsHighLow ? LOW : HIGH);
    digitalWrite(pinIn2, logicaAdelanteEsHighLow ? HIGH : LOW);
  } else {
    digitalWrite(pinIn1, LOW); digitalWrite(pinIn2, LOW);
  }
  analogWrite(pinEna, pwm);
}

void moveRobotPWM(int pwmLeft, int pwmRight) {
  setMotor(0, pwmLeft > 0 ? 1 : (pwmLeft < 0 ? -1 : 0), abs(constrain(pwmLeft, -255, 255)));
  setMotor(1, pwmRight > 0 ? 1 : (pwmRight < 0 ? -1 : 0), abs(constrain(pwmRight, -255, 255)));
}

void stopRobot() {
  setMotor(0, 0, 0); setMotor(1, 0, 0);
}

void updateOdometry(int pwmLeft, int pwmRight, float dt_odom) {
  float speedLeft_signed = pwmLeft * PWM_TO_SPEED_FACTOR;
  float speedRight_signed = pwmRight * PWM_TO_SPEED_FACTOR;

  float v_robot = (speedRight_signed + speedLeft_signed) / 2.0;
  float omega_robot = (speedRight_signed - speedLeft_signed) / WHEEL_BASE;

  float delta_s = v_robot * dt_odom;
  float delta_theta = omega_robot * dt_odom;

  robotX += delta_s * cos(robotTheta + delta_theta / 2.0);
  robotY += delta_s * sin(robotTheta + delta_theta / 2.0);
  robotTheta += delta_theta;
}

int demoState = 0;
unsigned long demoStartTime = 0;

void loop() {
  unsigned long currentTime = millis();
  float dt_loop = (currentTime - tiempoPrevioLoop) / 1000.0;
  tiempoPrevioLoop = currentTime;

  if (currentTime - tiempoPrevioIMU >= 10) {
    float dt_imu_actual = (currentTime - tiempoPrevioIMU) / 1000.0;
    if (dt_imu_actual > 0) {
        updateIMU(dt_imu_actual);
    }
    tiempoPrevioIMU = currentTime;
  }

  switch (demoState) {
    case 0:
      Serial.println("\n--- Demo: Control Velocidad PWM (sin IMU) ---");
      Serial.println("Adelante LENTO (PWM 120) por 2s...");
      moveRobotPWM(120, 120);
      demoStartTime = currentTime;
      demoState = 1;
      break;

    case 1:
      updateOdometry(120, 120, dt_loop);
      if (currentTime - demoStartTime >= 2000) {
        stopRobot(); Serial.println("DETENIDO.");
        Serial.print("Odom: X="); Serial.print(robotX,2); Serial.print(" Y="); Serial.print(robotY,2); Serial.print(" Th="); Serial.println(robotTheta*180.0/M_PI,1);
        delay(1000);
        Serial.println("Adelante RAPIDO (PWM 220) por 2s...");
        moveRobotPWM(220, 220);
        demoStartTime = currentTime;
        demoState = 2;
      }
      break;

    case 2:
      updateOdometry(220, 220, dt_loop);
      if (currentTime - demoStartTime >= 2000) {
        stopRobot(); Serial.println("DETENIDO.");
        Serial.print("Odom: X="); Serial.print(robotX,2); Serial.print(" Y="); Serial.print(robotY,2); Serial.print(" Th="); Serial.println(robotTheta*180.0/M_PI,1);
        delay(1000);
        demoState = 3;
        robotX = 0; robotY = 0; robotTheta = 0; currentYaw = 0; targetYaw = 0;
      }
      break;

    case 3:
      Serial.println("\n--- Demo: Linea Recta con Correccion IMU (Yaw, Pitch, Roll) ---");
      Serial.print("Target Yaw: "); Serial.println(targetYaw);
      Serial.println("Moviendo por 5 segundos...");
      demoStartTime = currentTime;
      demoState = 4;
      break;

    case 4:
      {
        float baseSpeedPWM_original = 150;
        float adjustedBaseSpeedPWM = baseSpeedPWM_original;

        if (abs(currentPitch) < MAX_PITCH_FOR_ADJUST) {
          float pitch_power_adj = KP_PITCH_POWER * currentPitch;
          adjustedBaseSpeedPWM += pitch_power_adj;
          adjustedBaseSpeedPWM = constrain(adjustedBaseSpeedPWM, 70, 250);
        } else {
          Serial.print("Pitch ("); Serial.print(currentPitch); Serial.print("deg) EXTREMO. Usando vel. base o considerar detener.");
        }

        float yawError = targetYaw - currentYaw;
        float yaw_correction = KP_YAW * yawError;

        float roll_steering_adj = 0.0;
        if (abs(currentRoll) < MAX_ROLL_FOR_STEER) {
          roll_steering_adj = KP_ROLL_STEER * currentRoll;
        } else {
          Serial.print("Roll ("); Serial.print(currentRoll); Serial.print("deg) EXTREMO. Sin corrección roll-steer.");
        }
        
        int pwmLeft  = round(adjustedBaseSpeedPWM + yaw_correction - roll_steering_adj);
        int pwmRight = round(adjustedBaseSpeedPWM - yaw_correction + roll_steering_adj);
        
        pwmLeft  = constrain(pwmLeft, -255, 255);
        pwmRight = constrain(pwmRight, -255, 255);

        moveRobotPWM(pwmLeft, pwmRight);
        updateOdometry(pwmLeft, pwmRight, dt_loop);

        if ((currentTime - demoStartTime) % 750 < 20 && dt_loop > 0) {
          Serial.print("T:"); Serial.print((currentTime-demoStartTime)/1000.0,1);
          Serial.print(" YAW:"); Serial.print(currentYaw,1);
          Serial.print(" R:"); Serial.print(currentRoll,1);
          Serial.print(" P:"); Serial.print(currentPitch,1);
          Serial.print(" V.BaseAdj:"); Serial.print(adjustedBaseSpeedPWM,0);
          Serial.print(" Yc:"); Serial.print(yaw_correction,0);
          Serial.print(" Rc:"); Serial.print(roll_steering_adj,0);
          Serial.print(" Lpwm:"); Serial.print(pwmLeft);
          Serial.print(" Rpwm:"); Serial.println(pwmRight);
          Serial.print("   Odom:X=");Serial.print(robotX,2);Serial.print(" Y=");Serial.print(robotY,2);Serial.print(" Th=");Serial.println(robotTheta*180.0/M_PI,1);
        }

        if (currentTime - demoStartTime >= 5000) {
          stopRobot();
          Serial.println("DETENIDO - Fin prueba linea recta con correccion IMU.");
          Serial.print("Odometria final: X="); Serial.print(robotX); Serial.print(" Y="); Serial.print(robotY); Serial.print(" Theta="); Serial.println(robotTheta * 180.0/M_PI);
          delay(2000);
          demoState = 99;
        }
      }
      break;

    case 99:
      Serial.println("--- Demostraciones Completadas ---");
      delay(10000);
      demoState = 0;
      robotX=0; robotY=0; robotTheta=0; currentYaw=0; targetYaw=0; currentRoll=0; currentPitch=0;
      break;
  }
}