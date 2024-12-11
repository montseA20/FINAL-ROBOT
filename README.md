*Motor Controller Node*

Este proyecto implementa un nodo de control de motores utilizando ROS 2, Python y una Raspberry Pi para gestionar el movimiento de dos motores de corriente continua (DC). El nodo interactúa con un tema llamado /apriltag_data para recibir datos de etiquetas AprilTag, ajustar el comportamiento de los motores según la distancia y el ID de las etiquetas detectadas.

*Requisitos*

Hardware

Raspberry Pi con configuración GPIO habilitada.

Dos motores DC.

Controlador de motores con soporte para PWM.

Sistema AprilTag para detectar etiquetas y publicar datos.

Software

ROS 2 (compatible con el proyecto).

Librerías Python:

rclpy: API cliente de ROS 2.

RPi.GPIO: Manejo de pines GPIO de la Raspberry Pi.

std_msgs: Para la definición de mensajes estándar (en este caso, Int32MultiArray).

time: Para manejar pausas temporales.

Configuración del Hardware

Pines GPIO Utilizados

Motor 1:

Pin de control 1: GPIO 17

Pin de control 2: GPIO 18

Pin PWM: GPIO 25

Motor 2:

Pin de control 1: GPIO 22

Pin de control 2: GPIO 23

Pin PWM: GPIO 27

Asegúrate de conectar correctamente los motores al controlador y al GPIO según esta configuración.

*Funcionamiento*

*Inicialización*

El nodo se llama motor_controller. Durante la inicialización:

Se configuran los pines GPIO para el control de los motores.

Se inicializan los objetos PWM para controlar la velocidad de los motores.

Se suscribe al tema /apriltag_data.

Suscripción al Tema /apriltag_data

El nodo espera mensajes de tipo Int32MultiArray, donde:

Elemento 0: ID de la etiqueta detectada.

Elemento 1: Distancia en cm a la etiqueta.

*Comportamiento del Nodo

Si la distancia a la etiqueta detectada es menor o igual a 25 cm, se realiza una acción dependiendo del ID de la etiqueta.

Las acciones incluyen:

Girar 90 grados en sentido horario o antihorario.

Girar 180 grados.

Detener los motores y finalizar el programa.

Si la distancia es mayor a 25 cm, los motores siguen avanzando.

*Funciones Clave*

keep_motors_moving_forward(): Configura los motores para avanzar.

rotate_90_degrees(clockwise): Gira los motores 90 grados en el sentido especificado.

rotate_180_degrees(): Gira los motores 180 grados.

stop_motors(): Detiene los motores.

set_motor_speed(motor_num, speed): Ajusta la velocidad de un motor mediante PWM.

*Ejecución*

Asegúrate de tener ROS 2 configurado en tu sistema.

Ejecuta el nodo:

python3 motor_controller.py

Observa los logs para verificar el comportamiento.

*Consideraciones*

Al finalizar la ejecución, los pines GPIO se limpian automáticamente para evitar problemas en el hardware.

Asegúrate de que el hardware esté correctamente conectado y alimentado antes de ejecutar el programa.

*Futuras Mejoras*

Agregar un mecanismo de control de errores para verificar la conexión de hardware.

Incluir parámetros configurables para la velocidad y el tiempo de rotación.

Mejorar el manejo de las excepciones en caso de errores durante la ejecución.
