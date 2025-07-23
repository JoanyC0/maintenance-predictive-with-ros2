import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # On reçoit une classe entière
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

class HIAnalyzerNode(Node):
    def __init__(self):
        super().__init__('hi_analyzer_node')

        self.df = pd.read_csv('/home/jacquescormery/ros2_ws/dataset/fichier0.csv')
        self.cycles = np.arange(len(self.df))
        self.max_cycle = self.cycles[-1]
        self.get_logger().info(f'Max cycle défini comme index max : {self.max_cycle}')

        # HI réel normalisé 1 → 0 en continu
        self.hi_reel = (1 - (self.cycles / self.max_cycle)).tolist()

        self.index = 0
        self.hi_preds = []

        # S’abonner au topic avec des classes entières (Int32)
        self.subscription = self.create_subscription(
            Int32,
            'prediction_class',  # reçoit classes 0,1,2
            self.hi_callback,
            10)

        self.get_logger().info('🚀 Node d’analyse HI lancé.')

        plt.ion()
        self.fig, self.axs = plt.subplots(1, 2, figsize=(12, 6))

    def hi_callback(self, msg):
        classe_predite = int(msg.data)

        # Mapping classe → HI continu
        hi_pred = self.class_to_hi(classe_predite)
        if self.index >= len(self.hi_reel):
            self.get_logger().info('📊 Données terminées, fermeture graphique...')
            plt.ioff()
            plt.show()
            rclpy.shutdown()
            return

        actual = self.hi_reel[self.index]
        self.hi_preds.append(hi_pred)

        self.get_logger().info(f'🔍 Index {self.index} - HI prédit (mapped) : {hi_pred:.3f} | HI réel : {actual:.3f}')

        self.axs[0].clear()
        self.axs[0].plot(self.hi_reel[:len(self.hi_preds)], label='HI réel', color='green')
        self.axs[0].plot(self.hi_preds, label='HI prédit', color='blue', linestyle='--')
        self.axs[0].set_title('Health Index Réel vs Prédit (live)')
        self.axs[0].set_xlabel('Index')
        self.axs[0].set_ylabel('Health Index')
        self.axs[0].legend()

        self.axs[1].clear()
        errors = np.abs(np.array(self.hi_reel[:len(self.hi_preds)]) - np.array(self.hi_preds))
        self.axs[1].plot(errors, color='red')
        self.axs[1].set_title('Erreur absolue par point')
        self.axs[1].set_xlabel('Index')
        self.axs[1].set_ylabel('Erreur')

        plt.tight_layout()
        plt.pause(0.001)

        self.index += 1

    def class_to_hi(self, classe):
        # Map classes 0,1,2 → HI entre 1 et 0 (discret)
        mapping = {
            0: 1.0,   # Bon fonctionnement
            1: 0.5,   # À surveiller
            2: 0.0    # Panne imminente
        }
        return mapping.get(classe, 0.0)  # Default à 0 si classe inconnue

def main(args=None):
    rclpy.init(args=args)
    node = HIAnalyzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

