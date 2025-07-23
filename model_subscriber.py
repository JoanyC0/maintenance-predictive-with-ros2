import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32
import numpy as np
from tensorflow.keras.models import load_model

class ModelSubscriberNode(Node):
    def __init__(self):
        super().__init__('model_subscriber_node')  # ✅ Nom générique pour le node

        try:
            self.model = load_model('/home/jacquescormery/ros2_ws/models/lstm_model.h5')
            self.get_logger().info('✅ Modèle chargé avec succès.')
        except Exception as e:
            self.get_logger().error(f'❌ Erreur chargement modèle : {e}')
            raise

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'obd_data',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(Int32, 'prediction_class', 10)

        self.class_labels = {
            0: 'Bon fonctionnement',
            1: 'À surveiller',
            2: 'Panne imminente'
        }

        self.get_logger().info('🚀 Node de prédiction lancé (mode classification).')

    def listener_callback(self, msg: Float64MultiArray):
        taille = len(msg.data)
        self.get_logger().info(f"📩 Données reçues. Taille : {taille}")

        if taille != 17:
            self.get_logger().error(f"❌ Taille inattendue : {taille} au lieu de 17. Abandon traitement.")
            return

        try:
            features = np.array(msg.data, dtype=np.float32)
            input_model = features.reshape(1, 1, 17)

            prediction = self.model.predict(input_model, verbose=0)
            predicted_class = int(np.argmax(prediction))

            msg_pub = Int32()
            msg_pub.data = predicted_class
            self.publisher.publish(msg_pub)

            label = self.class_labels.get(predicted_class, 'Classe inconnue')
            self.get_logger().info(f'🔔 Classe prédite : {predicted_class} ({label})')
        except Exception as e:
            self.get_logger().error(f'❌ Erreur traitement message : {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ModelSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

