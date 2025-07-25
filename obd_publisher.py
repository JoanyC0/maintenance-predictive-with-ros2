import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import pandas as pd
import numpy as np

class OBDPublisher(Node):
    def __init__(self):
        super().__init__('obd_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'obd_data', 10)

        # Charger CSV avec 17 colonnes (0 à 16)
        self.df = pd.read_csv('/home/jacquescormery/ros2_ws/dataset/fichier0.csv')
        self.data = self.df.iloc[:, 0:17].values.astype(np.float32)  # 17 features

        self.window_size = 20
        self.index = 0

        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        # Boucle sur les données
        if self.index + self.window_size > len(self.data):
            self.get_logger().info("Fin des données simulées, boucle...")
            self.index = 0

        current_sequence = self.data[self.index:self.index + self.window_size]
        self.get_logger().info(f"Publié séquence OBD ligne {self.index} à {self.index + self.window_size} - shape: {current_sequence.shape}")

        msg = Float64MultiArray()
        msg.data = current_sequence.flatten().tolist()  # Aplatir séquence 2D en 1D
        self.publisher_.publish(msg)

        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = OBDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

