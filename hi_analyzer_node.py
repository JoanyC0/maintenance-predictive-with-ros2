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
        self.diff_hi = []
        self.timestamps = []
        self.time = 0
        self.max_points = 100

        self.pred_lower = []
        self.pred_upper = []
        self.state_colors = []
        self.state_labels = []

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
        self.diff_hi.append(ecart)

        if self.hi_predit > 0.7:
            lower = max(0, self.hi_predit - 0.3)
            upper = min(1.1, self.hi_predit)
            color = 'red'
            label = "🛑 Panne imminente"
        elif self.hi_predit > 0.3:
            lower = max(0, self.hi_predit - 0.1)
            upper = min(1.1, self.hi_predit + 0.1)
            color = 'orange'
            label = "⚠️ À surveiller" 
        else:
            lower = 0
            upper = min(1.1, self.hi_predit + 0.3)
            color = 'green'
            label = "✅ Bon fonctionnement"

        self.pred_lower.append(lower)
        self.pred_upper.append(upper)
        self.state_colors.append(color)
        self.state_labels.append(label)

        if len(self.timestamps) > self.max_points:
            self.timestamps = self.timestamps[-self.max_points:]
            self.actual_hi = self.actual_hi[-self.max_points:]
            self.pred_hi = self.pred_hi[-self.max_points:]
            self.diff_hi = self.diff_hi[-self.max_points:]
            self.pred_lower = self.pred_lower[-self.max_points:]
            self.pred_upper = self.pred_upper[-self.max_points:]
            self.state_colors = self.state_colors[-self.max_points:]
            self.state_labels = self.state_labels[-self.max_points:]

        self.hi_reel = None
        self.hi_predit = None

    def setup_plot(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("📈 Health Index Analyse")
        self.ax.set_xlabel("Temps")
        self.ax.set_ylabel("HI")
        self.ax.set_ylim(0, 1.1)

        self.line_reel, = self.ax.plot([], [], label='HI Réel', color='blue')
        self.line_pred, = self.ax.plot([], [], label='HI Prédit', color='green')

        # 👇 Texte déplacé vers la gauche
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

        while self.ax.collections:
            self.ax.collections.pop()

        if len(self.timestamps) > 1:
            for i in range(len(self.timestamps) - 1):
                self.ax.fill_between(
                    [self.timestamps[i], self.timestamps[i + 1]],
                    [self.pred_lower[i], self.pred_lower[i + 1]],
                    [self.pred_upper[i], self.pred_upper[i + 1]],
                    color=self.state_colors[i],
                    alpha=0.2
                )

            self.status_text.set_text(self.state_labels[-1])
            self.status_text.set_color(self.state_colors[-1])

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

