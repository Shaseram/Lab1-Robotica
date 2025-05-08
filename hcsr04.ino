const int TRIGGER_PIN = 10;
const int ECHO_PIN = 11;

long duracion;
float distanciaCm;
float distanciaPulgadas;

void setup() {
  Serial.begin(9600);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("Sensor HC-SR04 listo para medir distancias.");
  Serial.println("Apunta el sensor hacia un objeto.");
}

void loop() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  duracion = pulseIn(ECHO_PIN, HIGH);

  distanciaCm = duracion * 0.0343 / 2.0;

  distanciaPulgadas = duracion * 0.0135 / 2.0;


  Serial.print("Distancia: ");
  if (distanciaCm <= 0 || distanciaCm > 400) {
    Serial.println("Fuera de rango");
  } else {
    Serial.print(distanciaCm);
    Serial.print(" cm");
    Serial.println();
  }

  delay(500);
}