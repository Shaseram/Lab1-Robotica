# Integrantes:

- Vicente Mediano Larenas
- Daniel Miranda
- Nikolai Navea
- Vicente Arratia
- Javier Sepúlveda

# Descripción del Proyecto

El Laboratorio 1 consiste en construir un robot móvil básico con Arduino y controlar sus motores DC con un driver L298N, integrando sensores como IMU MPU 6050 (para ajustar la dirección y corregir la inclinación) y HC-SR04 (capaz de medir distancias mediante ultrasonidos). La idea es lograr que los sensores funcionen conjuntamente con el robot.

# Solución Laboratorio 1

## Parte 1

### Explicación de componentes

https://github.com/user-attachments/assets/fa5c1360-0b9e-49dc-8e75-e3a2013125ac

### Motores y movimiento

https://github.com/user-attachments/assets/34038446-59d4-4df3-b032-a5db0761b406

### Control de velocidad

???????????????????????????

### Verificación funcionamiento HC-SR04

https://github.com/user-attachments/assets/8063e1f8-1b50-4685-a3ae-ef028e34a567

### Preguntas Parte 1

-  **¿Qué función cumplen los sensores, actuadores y controladores en el
 robot?**
    - Sensores: Permiten al robot percibir su entorno y estado interno (ej. distancia a obstáculos con ultrasónico, color con RGB, orientación con IMU). Recopilan información.
    - Actuadores: Permiten al robot realizar acciones físicas y moverse (ej. los motores DC que mueven las ruedas). Ejecutan las decisiones.
    - Controladores: (Ej. Arduino UNO) Son el "cerebro" del robot. Procesan la información de los sensores y toman decisiones para comandar a los actuadores.- pepe
- **¿Cómo se puede estimar la velocidad sin encoders?**
    - Se puede estimar aplicando una potencia constante a los motores durante un tiempo conocido, luego se mide la distancia recorrida y se calcula la velocidad (distancia/tiempo).
- **¿Cómo afecta la falta de encoders a la precisión del movimiento?**
    - Negativamente porque sin encoders el control es en "bucle abierto", el robot no sabe realmente cuánto se ha movido o girado. Dependiendo del tipo de superficie o pequeños desniveles causarán que el robot no recorra las distancias o no realice los giros con la exactitud deseada.
- **¿Qué es PWM y cómo ayuda a controlar la velocidad de los motores?**
    - PWM (Pulse Width Modulation o Modulación por Ancho de Pulso) es una técnica para controlar la cantidad de potencia enviada a un dispositivo. Se hace encendiendo y apagando rápidamente una señal digital. El "ancho del pulso" es el porcentaje de tiempo que la señal está encendida. Para los motores DC, variar el ancho de pulso del PWM cambia el voltaje promedio que reciben, lo que a su vez controla su velocidad de giro: a mayor ancho de pulso, mayor velocidad.
- **¿Cómo afecta el control de velocidad a la precisión de la navegación sin encoders?**
    - Aunque el control de velocidad con PWM permite que el robot se mueva a diferentes velocidades, la falta de encoders sigue limitando la precisión. El robot puede intentar ir a una velocidad específica por un tiempo determinado para cubrir una distancia, pero no sabrá si realmente lo hizo, o si las condiciones cambiaron. Por lo tanto, la navegación seguirá siendo imprecisa.

## Parte 2

### Cinemática y control de velocidad con IMU

### Inclinación y correción en tiempo real con IMU

### Preguntas Parte 2

- **¿Cómo se calcula la velocidad del robot sin encoders usando PWM?**
    - Con una calibración, por ejemplo, se fija un valor de PWM a los motores. Se hace funcionar el robot en línea recta sobre una superficie plana durante un tiempo medido y luego se mide la distancia que recorrió. La velocidad estimada para ese PWM es **velocidad = distancia / tiempo**. Se repite para diferentes valores de PWM si se desean múltiples velocidades.
- **¿Cómo factores afectan la trayectoria y velocidad del robot al cambiar los intervalos de tiempo?**
    - Trayectoria: Se ve afectada por diferencias entre motores/ruedas, irregularidades o cambios de fricción en la superficie, pequeñas inclinaciones, y distribución desigual del peso. Al variar el tiempo, estos pequeños errores se acumulan, desviando al robot.
    - Velocidad: Se ve afectada por el nivel de carga de la batería, la fricción de la superficie, las inclinaciones, y la carga que lleve el robot. Cambiar el intervalo de tiempo simplemente define cuánto tiempo operará bajo estas condiciones, afectando la distancia total recorrida.
- **¿Cuáles son las ventajas y desventajas de usar un IMU para ajustar la dirección en lugar de encoders?**
    - Ventajas: Mide directamente la orientación y el cambio angular (giro/yaw) del chasis, independientemente del deslizamiento de las ruedas. Puede detectar inclinaciones (pitch/roll) y suele ser más simple de integrar mecánicamente si el chasis no está preparado para encoders.
    - Desventajas: Sufre de "drift" (deriva) en el giroscopio, acumulando error en el ángulo con el tiempo. Los datos pueden ser ruidosos y requerir filtrado, no mide directamente la distancia recorrida y puede ser sensible a vibraciones.
- **¿Qué efecto tiene la inclinación o el giro en el movimiento del robot, y cómo se corrige con el IMU?**
    - Efecto de la Inclinación (Pitch/Roll): Puede causar pérdida de tracción en alguna rueda, cambiar la dinámica de movimiento, o hacer que el robot se desvíe de su trayectoria por efecto de la gravedad en una pendiente lateral.
    - Efecto del Giro (Yaw): Provoca que el robot se desvíe de la línea recta o del rumbo deseado.
    - **Corrección con el IMU**:
        - Para la Inclinación: Si el IMU (acelerómetro) detecta una inclinación que afecta la dirección (ej. en una pendiente lateral), se puede aplicar una lógica similar de corrección en la velocidad de los motores para contrarrestar la tendencia a desviarse. Para estabilidad, se podrían tomar acciones más drásticas como detenerse o cambiar de dirección.
        - Para el Giro (Yaw): El IMU (giroscopio) detecta la velocidad angular o el cambio de ángulo. Si el robot se desvía del rumbo deseado, el controlador (Arduino) ajusta la velocidad de los motores de forma diferencial para corregir la trayectoria y mantener el rumbo.

# Códigos utilizados
