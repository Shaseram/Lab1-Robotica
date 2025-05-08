// Definición de pines para el sensor HC-SR04
const int TRIGGER_PIN = 10; // Pin Trig del HC-SR04 conectado al pin 10 del Arduino
const int ECHO_PIN = 11;    // Pin Echo del HC-SR04 conectado al pin 11 del Arduino

// Variables para la duración del pulso y la distancia
long duracion;
float distanciaCm;
float distanciaPulgadas; // Opcional: para mostrar en pulgadas también

void setup() {
  Serial.begin(9600); // Iniciar la comunicación serial para mostrar los resultados

  // Configurar los pines del sensor
  pinMode(TRIGGER_PIN, OUTPUT); // El pin Trigger es una salida (envía el pulso)
  pinMode(ECHO_PIN, INPUT);   // El pin Echo es una entrada (recibe el pulso)

  Serial.println("Sensor HC-SR04 listo para medir distancias.");
  Serial.println("Apunta el sensor hacia un objeto.");
}

void loop() {
  // 1. Generar un pulso limpio en el pin Trigger
  //    Asegurarse de que el Trigger esté en bajo al menos 2 microsegundos.
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  // 2. Enviar el pulso ultrasónico: poner el Trigger en alto por 10 microsegundos.
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // 3. Medir la duración del pulso de eco
  //    La función pulseIn() mide el tiempo (en microsegundos) que el pin Echo
  //    permanece en estado ALTO.
  duracion = pulseIn(ECHO_PIN, HIGH);

  // 4. Calcular la distancia
  //    La velocidad del sonido es aproximadamente 0.0343 cm/microsegundo.
  //    La distancia es (tiempo * velocidad_del_sonido) / 2, porque el sonido
  //    viaja hasta el objeto Y DE VUELTA.
  //    Distancia (cm) = (duracion_microsegundos * 0.0343) / 2
  //    o, simplificado: duracion_microsegundos / 58.2 (aproximadamente)
  distanciaCm = duracion * 0.0343 / 2.0;
  // distanciaCm = duracion / 58.2; // Fórmula alternativa y común

  // Opcional: Calcular distancia en pulgadas
  // Velocidad del sonido aprox. 0.0135 pulgadas/microsegundo
  // Distancia (pulgadas) = (duracion_microsegundos * 0.0135) / 2
  // o, simplificado: duracion_microsegundos / 148 (aproximadamente)
  distanciaPulgadas = duracion * 0.0135 / 2.0;
  // distanciaPulgadas = duracion / 148.0; // Fórmula alternativa y común


  // 5. Mostrar la distancia en el Monitor Serie
  Serial.print("Distancia: ");
  if (distanciaCm <= 0 || distanciaCm > 400) { // El HC-SR04 tiene un rango práctico (aprox 2cm a 400cm)
    Serial.println("Fuera de rango");
  } else {
    Serial.print(distanciaCm);
    Serial.print(" cm");
    // Serial.print("  |  "); // Descomenta si quieres ver pulgadas también
    // Serial.print(distanciaPulgadas);
    // Serial.println(" pulgadas");
    Serial.println(); // Solo cm
  }

  delay(500); // Esperar medio segundo antes de la siguiente medición
}