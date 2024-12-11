import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import RPi.GPIO as GPIO
import time  # Librería para manejar pausas temporales

# Definir los pines para el control de dos motores DC y sus respectivos pines ENABLE
motor1_pin_1 = 17
motor1_pin_2 = 18
pwm_pin_1 = 25

motor2_pin_1 = 22
motor2_pin_2 = 23
pwm_pin_2 = 27

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.get_logger().info("Motor Controller Node Initialized")

        # Configuración de los pines GPIO
        GPIO.setmode(GPIO.BCM)

        # Configuración del Motor 1
        GPIO.setup(motor1_pin_1, GPIO.OUT)
        GPIO.setup(motor1_pin_2, GPIO.OUT)
        GPIO.setup(pwm_pin_1, GPIO.OUT)
        self.pwm1 = GPIO.PWM(pwm_pin_1, 1000)
        self.pwm1.start(0)

        # Configuración del Motor 2
        GPIO.setup(motor2_pin_1, GPIO.OUT)
        GPIO.setup(motor2_pin_2, GPIO.OUT)
        GPIO.setup(pwm_pin_2, GPIO.OUT)
        self.pwm2 = GPIO.PWM(pwm_pin_2, 1000)
        self.pwm2.start(0)

        # Suscriptor al tema '/apriltag_data' (ID y distancia)
        self.create_subscription(Int32MultiArray, '/apriltag_data', self.tag_callback, 10)

        # Variable para almacenar la distancia actual al tag
        self.current_distance = float('inf')
        self.current_tag_id = None

        # Por defecto, ambos motores avanzan
        self.var_aux=99
        self.keep_motors_moving_forward()

    def tag_callback(self, msg):
        data = msg.data
        if len(data) >= 2:
            self.current_tag_id = data[0]
            self.current_distance = data[1]  # La distancia se publica como entero

            self.get_logger().info(f"Received AprilTag ID: {self.current_tag_id}, Distance: {self.current_distance} cm")

            # Verificar si la distancia es menor o igual a 20 cm
            if self.current_distance <= 25:
                self.perform_action_based_on_tag()
                self.keep_motors_moving_forward()
            else:
                self.keep_motors_moving_forward()

    def perform_action_based_on_tag(self):
    
        if self.current_tag_id == 0:
            self.var_aux2=self.current_tag_id
            if(self.var_aux2 != self.var_aux):
            	self.rotate_90_degrees(clockwise=False)
            self.var_aux = self.var_aux2
        elif self.current_tag_id == 1:
            self.var_aux2=self.current_tag_id
            if(self.var_aux2 != self.var_aux):
            	self.rotate_90_degrees(clockwise=True)
            self.var_aux = self.var_aux2
        elif self.current_tag_id == 2:
            self.var_aux2=self.current_tag_id
            if(self.var_aux2 != self.var_aux):
            	self.rotate_180_degrees()
            self.var_aux = self.var_aux2
        elif self.current_tag_id == 3:
            self.var_aux2=self.current_tag_id
            if(self.var_aux2 != self.var_aux):
            	self.rotate_90_degrees(clockwise=False)
            self.var_aux = self.var_aux2   
        elif self.current_tag_id == 4:
            self.var_aux2=self.current_tag_id
            if(self.var_aux2 != self.var_aux):
            	self.rotate_180_degrees()
            	self.stop_motors()
            	self.get_logger().info("Finalizing program")
            	rclpy.shutdown()
            	

            
            

    def rotate_90_degrees(self, clockwise):
        direction = "clockwise" if clockwise else "counterclockwise"
        self.get_logger().info(f"Rotating 90 degrees {direction}")

        # Configuración de dirección para los motores
        if clockwise:
            GPIO.output(motor1_pin_1, GPIO.LOW)
            GPIO.output(motor1_pin_2, GPIO.HIGH)
            GPIO.output(motor2_pin_1, GPIO.HIGH)
            GPIO.output(motor2_pin_2, GPIO.LOW)
        else:
            GPIO.output(motor1_pin_1, GPIO.HIGH)
            GPIO.output(motor1_pin_2, GPIO.LOW)
            GPIO.output(motor2_pin_1, GPIO.LOW)
            GPIO.output(motor2_pin_2, GPIO.HIGH)

        self.set_motor_speed(1, 100)
        self.set_motor_speed(2, 100)
        time.sleep(2.5)
        self.keep_motors_moving_forward()

    def rotate_180_degrees(self):
        self.get_logger().info("Rotating 180 degrees")
        GPIO.output(motor1_pin_1, GPIO.HIGH)
        GPIO.output(motor1_pin_2, GPIO.LOW)
        GPIO.output(motor2_pin_1, GPIO.LOW)
        GPIO.output(motor2_pin_2, GPIO.HIGH)

        self.set_motor_speed(1, 100)
        self.set_motor_speed(2, 100)
        time.sleep(5)
        self.keep_motors_moving_forward()

    def keep_motors_moving_forward(self):
        self.get_logger().info("Both motors moving forward")
        GPIO.output(motor1_pin_1, GPIO.HIGH)
        GPIO.output(motor1_pin_2, GPIO.LOW)
        GPIO.output(motor2_pin_1, GPIO.HIGH)
        GPIO.output(motor2_pin_2, GPIO.LOW)
        self.set_motor_speed(1, 100)
        self.set_motor_speed(2, 100)

    def stop_motors(self):
        self.get_logger().info("Stopping both motors")
        GPIO.output(motor1_pin_1, GPIO.LOW)
        GPIO.output(motor1_pin_2, GPIO.LOW)
        GPIO.output(motor2_pin_1, GPIO.LOW)
        GPIO.output(motor2_pin_2, GPIO.LOW)
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

    def set_motor_speed(self, motor_num, speed):
        if motor_num == 1:
            self.pwm1.ChangeDutyCycle(speed)
        elif motor_num == 2:
            self.pwm2.ChangeDutyCycle(speed)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
