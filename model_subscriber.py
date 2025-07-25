import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Bool
import numpy as np
import time
from tensorflow.keras.models import load_model

class ModelSubscriberNode(Node):
    def __init__(self):
        super().__init__('model_subscriber_node')

        try:
            self.model = load_model('/home/jacquescormery/ros2_ws/models/GRU_model.h5')
            self.get_logger().info('✅ Modèle chargé avec succès.')
        except Exception as e:
            self.get_logger().error(f'❌ Erreur chargement modèle : {e}')
            raise

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'obd_data',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(
            Float64,
            'hi_predit_topic',
            10)

        self.done_publisher = self.create_publisher(
            Bool,
            'model_done',
            10)

        self.window_size = 20
        self.n_features = 17

        self.get_logger().info('🚀 Node de prédiction lancé (Health Index prédit par le modèle)')

    def listener_callback(self, msg: Float64MultiArray):
        taille = len(msg.data)
        self.get_logger().info(f"📩 Données reçues. Taille : {taille}")

        expected_size = self.window_size * self.n_features
        if taille != expected_size:
            self.get_logger().error(f"❌ Taille inattendue : {taille} au lieu de {expected_size}. Abandon traitement.")
            return

        try:
            features = np.array(msg.data, dtype=np.float32)
            input_model = features.reshape(1, self.window_size, self.n_features)

            start_time = time.time()
            prediction_probs = self.model.predict(input_model, verbose=0)[0]
            end_time = time.time()
            prediction_duration = (end_time - start_time) * 1000  # ms

            weights = [1.0, 0.5, 0.0]
            hi = float(np.dot(prediction_probs, weights))

            msg_pub = Float64()
            msg_pub.data = hi
            self.publisher.publish(msg_pub)

            self.get_logger().info(f'✅ HI prédit publié : {hi:.3f} (⏱️ {prediction_duration:.2f} ms)')

            # Publie le signal "fin de prédiction"
            self.done_publisher.publish(Bool(data=True))

        except Exception as e:
            self.get_logger().error(f'❌ Erreur traitement message : {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ModelSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

