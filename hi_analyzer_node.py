import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class HIAnalyzerNode(Node):
    def __init__(self):
        super().__init__('hi_analyzer_node')

        self.hi_reel = None
        self.hi_predit = None

        self.actual_hi = []
        self.pred_hi = []
        self.timestamps = []
        self.time = 0
        self.max_points = 100

        self.sub_reel = self.create_subscription(
            Float64,
            'hi_actual_topic',
            self.callback_reel,
            10)

        self.sub_predit = self.create_subscription(
            Float64,
            'hi_predit_topic',
            self.callback_predit,
            10)

        self.publisher = self.create_publisher(
            Float64,
            'hi_analysis',
            10)

        self.get_logger().info('🔍 Node d’analyse HI avec graphe lancé.')

        self.setup_plot()

    def callback_reel(self, msg):
        self.hi_reel = msg.data
        self.get_logger().info(f'📥 HI réel reçu : {self.hi_reel:.3f}')
        self.try_compare()

    def callback_predit(self, msg):
        self.hi_predit = msg.data
        self.get_logger().info(f'📥 HI prédit reçu : {self.hi_predit:.3f}')
        self.try_compare()

    def try_compare(self):
        if self.hi_reel is None or self.hi_predit is None:
            return

        ecart = abs(self.hi_reel - self.hi_predit)
        self.get_logger().info(
            f'📊 Comparaison HI : Réel={self.hi_reel:.3f} | Prédit={self.hi_predit:.3f} | Écart={ecart:.3f}'
        )

        msg = Float64()
        msg.data = ecart
        self.publisher.publish(msg)

        self.timestamps.append(self.time)
        self.time += 1
        self.actual_hi.append(self.hi_reel)
        self.pred_hi.append(self.hi_predit)

        if len(self.timestamps) > self.max_points:
            self.timestamps = self.timestamps[-self.max_points:]
            self.actual_hi = self.actual_hi[-self.max_points:]
            self.pred_hi = self.pred_hi[-self.max_points:]

        self.hi_reel = None
        self.hi_predit = None

    def setup_plot(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("📈 Health Index Analyse")
        self.ax.set_xlabel("Temps")
        self.ax.set_ylabel("HI")
        self.ax.set_ylim(0, 1.1)

        self.line_reel, = self.ax.plot([], [], label='HI Réel', color='blue')
        self.line_pred, = self.ax.plot([], [], label='HI Probable', color='green')

        # ✅ Zones fixes
        self.ax.axhspan(0.0, 0.3, facecolor='green', alpha=0.1, label='✅ Bon fonctionnement')
        self.ax.axhspan(0.3, 0.7, facecolor='orange', alpha=0.1, label='⚠️ À surveiller')
        self.ax.axhspan(0.7, 1.1, facecolor='red', alpha=0.1, label='🛑 Panne imminente')

        self.status_text = self.ax.text(
            0.01, -0.12, "", transform=self.ax.transAxes,
            ha='left', fontsize=13, fontweight='bold',
            bbox=dict(facecolor='white', alpha=0.9, boxstyle='round,pad=0.3')
        )

        self.ax.legend(loc='upper right')
        self.fig.subplots_adjust(bottom=0.25)

        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=500)
        plt.tight_layout()
        plt.show(block=False)

    def update_plot(self, frame):
        self.line_reel.set_data(self.timestamps, self.actual_hi)
        self.line_pred.set_data(self.timestamps, self.pred_hi)
        self.ax.set_xlim(max(0, self.time - self.max_points), self.time)

        # Dernier état de HI prédite
        if self.pred_hi:
            last_pred = self.pred_hi[-1]
            if last_pred > 0.7:
                label = "🛑 Panne imminente"
                color = "red"
            elif last_pred > 0.3:
                label = "⚠️ À surveiller"
                color = "orange"
            else:
                label = "✅ Bon fonctionnement"
                color = "green"

            self.status_text.set_text(label)
            self.status_text.set_color(color)

        return self.line_reel, self.line_pred, self.status_text

def main(args=None):
    rclpy.init(args=args)
    node = HIAnalyzerNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

