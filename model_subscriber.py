import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray  # On change pour Float64MultiArray
import numpy as np
from tensorflow.keras.models import load_model

class ModelSubscriberNode(Node):
    def __init__(self):
        super().__init__('model_subscriber_node')

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

        self.publisher = self.create_publisher(Float64MultiArray, 'prediction_probs', 10)

        self.get_logger().info('🚀 Node de prédiction lancé (mode probabilités).')

    def listener_callback(self, msg: Float64MultiArray):
        taille = len(msg.data)
        self.get_logger().info(f"📩 Données reçues. Taille : {taille}")

        if taille != 17:
            self.get_logger().error(f"❌ Taille inattendue : {taille} au lieu de 17. Abandon traitement.")
            return

        try:
            features = np.array(msg.data, dtype=np.float32)
            input_model = features.reshape(1, 1, 17)

            prediction = self.model.predict(input_model, verbose=0)[0]  # tableau 1D des proba

            # Préparation message Float64MultiArray
            msg_pub = Float64MultiArray()
            msg_pub.data = prediction.tolist()
            self.publisher.publish(msg_pub)

            self.get_logger().info(f'🔔 Probabilités prédites : {prediction}')
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

