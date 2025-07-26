import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import pandas as pd
from datetime import datetime
import os
import random

class HIActualNode(Node):
    def __init__(self):
        super().__init__('hi_actual_node')

        dataset_path = '/home/jacquescormery/ros2_ws/dataset/dataset_Après_KMeans_clean.csv'
        if not os.path.exists(dataset_path):
            self.get_logger().error(f'❌ Fichier introuvable : {dataset_path}')
            raise FileNotFoundError(f"Fichier manquant : {dataset_path}")

        col_names = [f'feat_{i}' for i in range(17)] + ['etat', 'etat_text']
        self.df = pd.read_csv(dataset_path, header=None, names=col_names)

        self.index = 0

        # HI moyen par cluster (à ajuster selon ta connaissance des clusters)
        self.cluster_hi_map = {
            0: 0.1,  # cluster sain
            1: 0.5,  # cluster dégradé
            2: 0.9   # cluster panne
        }

        # Amplitude max du bruit à ajouter
        self.noise_std = 0.05

        self.done_subscription = self.create_subscription(
            Bool,
            'model_done',
            self.done_callback,
            10
        )
        self.publisher = self.create_publisher(Float64, 'hi_actual_topic', 10)
        self.get_logger().info('📡 Node HI réel simulé lancé (bruité selon cluster).')

    def done_callback(self, msg: Bool):
        if msg.data:
            next_index = self.index + 20
            if next_index >= len(self.df):
                self.get_logger().warn('⛔ Fin du dataset atteinte, arrêt publication HI réel.')
                return

            row = self.df.iloc[next_index]
            cluster = int(row['etat'])  # la colonne cluster est 'etat'
            base_hi = self.cluster_hi_map.get(cluster, 0.0)

            # Générer HI bruité autour de base_hi
            hi_bruite = random.gauss(base_hi, self.noise_std)

            # Clamp HI entre 0 et 1
            hi_bruite = max(0.0, min(1.0, hi_bruite))

            now = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            etat_text = row['etat_text']
            self.get_logger().info(
                f'✅ HI simulé bruité à index {next_index} : {hi_bruite:.3f} '
                f'(Cluster : {cluster} - État : {etat_text}) à {now}'
            )

            msg_pub = Float64()
            msg_pub.data = hi_bruite
            self.publisher.publish(msg_pub)

            self.get_logger().info(f'📤 HI simulé publié à {now}')
            self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = HIActualNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Interruption clavier reçue, arrêt du noeud.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

