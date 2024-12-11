import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import cv2
import apriltag
import numpy as np

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        self.get_logger().info("AprilTag Detector Node Initialized")

        # Publicador de tag_id y distancia
        self.tag_publisher = self.create_publisher(Int32MultiArray, '/apriltag_data', 10)

        # Parámetros de la cámara y el tag
        self.focal_length = 700  # Longitud focal de la cámara en píxeles (ajusta según tu cámara)
        self.tag_size = 10  # Tamaño real del lado del AprilTag en cm

    def detect_tags(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = apriltag.Detector()
        results = detector.detect(gray)

        for result in results:
            # Obtener el ID del tag
            tag_id = result.tag_id

            # Calcular la distancia al tag
            corners = result.corners  # Esquinas del tag detectado
            width_in_pixels = np.linalg.norm(corners[0] - corners[1])  # Distancia entre dos esquinas adyacentes
            distance = (self.focal_length * self.tag_size) / width_in_pixels

            # Publicar el ID y la distancia
            tag_data_msg = Int32MultiArray()
            tag_data_msg.data = [tag_id, int(distance)]  # Convertir distancia a entero para simplificar
            self.tag_publisher.publish(tag_data_msg)

            self.get_logger().info(f"Detected AprilTag ID: {tag_id}, Distance: {distance:.2f} cm")

        return frame

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        node.get_logger().error("Could not open video device")
        return

    while rclpy.ok():
        ret, frame = cap.read()
        if not ret:
            node.get_logger().error("Failed to read frame")
            break

        # Detectar los tags y procesar la imagen
        node.detect_tags(frame)

    cap.release()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
