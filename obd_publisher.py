import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import pandas as pd

class OBDPublisher(Node):
    def __init__(self):
        super().__init__('obd_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'obd_data', 10)

        # Charger le CSV
        self.df = pd.read_csv('/home/jacquescormery/ros2_ws/dataset/fichier0.csv')

        # Sélection des 17 features utilisées par ton modèle (colonnes 0 à 16 par exemple)
        self.data = self.df.iloc[:, 0:17].values

        self.index = 0
        self.timer = self.create_timer(1.0, self.publish_data)  # Publie chaque seconde

    def publish_data(self):
        if self.index >= len(self.data):
            self.get_logger().info("Fin des données simulées, boucle...")
            self.index = 0

        current_data = self.data[self.index]
        self.get_logger().info(f"Publié données OBD ligne {self.index} - taille: {current_data.shape}")

        msg = Float64MultiArray()
        msg.data = current_data.tolist()
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

