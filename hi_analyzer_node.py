import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patheffects as path_effects  # pour l’ombre sur le texte

class HIAnalyzerNode(Node):
    def __init__(self):
        super().__init__('hi_analyzer_node')

        self.index = 0
        self.hi_preds = []
        self.dominant_classes = []  # Stocker la classe dominante pour chaque mesure

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'prediction_probs',
            self.hi_callback,
            10)

        self.get_logger().info('🚀 Node d’analyse HI lancé.')

        plt.ion()
        plt.style.use('seaborn-darkgrid')  # fond + grille moderne
        plt.rcParams.update({
            'axes.titlesize': 18,
            'axes.labelsize': 14,
            'xtick.labelsize': 12,
            'ytick.labelsize': 12,
            'legend.fontsize': 13,
            'figure.facecolor': '#f0f0f0',
            'axes.edgecolor': '#444444',
            'axes.linewidth': 1.2,
            'grid.color': '#bbbbbb',
            'grid.linewidth': 0.8,
        })

        self.fig, self.ax = plt.subplots(figsize=(8, 6))

    def hi_callback(self, msg):
        probs = msg.data
        if len(probs) != 3:
            self.get_logger().warning('Probas reçues ne contiennent pas 3 classes')
            return

        hi_pred = probs[0] * 1.0 + probs[1] * 0.5 + probs[2] * 0.0
        self.hi_preds.append(hi_pred)

        dominant_class = np.argmax(probs)
        self.dominant_classes.append(dominant_class)

        label = self.class_label(dominant_class)

        self.get_logger().info(f'Index {self.index} - Health Index prédit: {hi_pred:.3f}')

        self.ax.clear()

        # Courbe HI plus épaisse, couleur douce
        self.ax.plot(self.hi_preds, label='Health Index', color='#1f77b4', linestyle='--', linewidth=2)

        # Points avec contour noir et alpha pour transparence
        indices = np.arange(len(self.hi_preds))
        colors = [self.class_color(c) for c in self.dominant_classes]
        self.ax.scatter(indices, self.hi_preds, color=colors, s=60, alpha=0.75,
                        edgecolors='k', linewidth=0.7, zorder=5)

        self.ax.set_title('évolution de l etat du moteur', fontweight='bold', pad=15)
        self.ax.set_xlabel('Index', labelpad=12)
        self.ax.set_ylabel('Health Index', labelpad=12)
        self.ax.set_ylim(-0.1, 1.1)
        self.ax.grid(True, linestyle='--', alpha=0.7)

        self.ax.legend(frameon=True, shadow=True, fancybox=True, facecolor='white')

        # Texte du label avec ombre blanche pour bien ressortir
        self.ax.text(0.5, 0.9, f'{label}', transform=self.ax.transAxes,
                     fontsize=16, ha='center', color='red', weight='bold',
                     path_effects=[path_effects.withStroke(linewidth=3, foreground='white')])

        plt.tight_layout()
        plt.pause(0.001)

        self.index += 1

    def class_label(self, classe):
        labels = {
            0: 'Bon fonctionnement',
            1: 'À surveiller',
            2: 'Panne imminente'
        }
        return labels.get(classe, 'Inconnu')

    def class_color(self, classe):
        colors = {
            0: 'green',    # Bon fonctionnement
            1: 'orange',   # À surveiller
            2: 'red'       # En panne
        }
        return colors.get(classe, 'black')

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

