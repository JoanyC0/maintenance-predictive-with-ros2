import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import pandas as pd
from datetime import datetime
import random  # Nécessaire pour random.uniform

def etat_to_hi_simule(etat):
    if etat == 0:
        return random.uniform(0.8, 1.0)
    elif etat == 1:
        return random.uniform(0.4, 0.7)
    elif etat == 2:
        return random.uniform(0.0, 0.3)
    else:
        return 0.0

class HIActualNode(Node):
    def __init__(self):
        super().__init__('hi_actual_node')

        # Chargement CSV sans header (pas de ligne d'en-tête)
        col_names = [f'feat_{i}' for i in range(17)] + ['etat', 'etat_text']
        self.df = pd.read_csv('/home/jacquescormery/ros2_ws/dataset/dataset_Après_KMeans_clean.csv',
                              header=None, names=col_names)

        self.index = 0  # Index pour parcourir le dataset

        # Abonnement au signal model_done
        self.done_subscription = self.create_subscription(
            Bool,
            'model_done',
            self.done_callback,
            10)

        self.publisher = self.create_publisher(
            Float64,
            'hi_actual_topic',
            10)

        self.get_logger().info('📡 Node HI réel lancé (HI simulé selon état).')

    def done_callback(self, msg: Bool):
        if msg.data:  # signal model_done == True

            if self.index + 20 >= len(self.df):
                self.get_logger().info('✅ Fin du dataset atteinte, arrêt publication HI simulé.')
                return

            etat_val = int(self.df.iloc[self.index + 20]['etat'])
            simulated_hi = etat_to_hi_simule(etat_val)

            now = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            self.get_logger().info(f'✅ HI simulé généré à index {self.index + 20} pour état {etat_val} : {simulated_hi:.3f} à {now}')

            msg_pub = Float64()
            msg_pub.data = simulated_hi
            self.publisher.publish(msg_pub)

            self.get_logger().info(f'📤 HI simulé publié à {now}')

            self.index += 1  # Avance la fenêtre

def main(args=None):
    rclpy.init(args=args)
    node = HIActualNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

