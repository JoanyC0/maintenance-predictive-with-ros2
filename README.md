# maintenance-predictive-with-ros2

ROS 2 package for vehicle predictive maintenance using a trained LSTM model.

## Nodes

- `obd_publisher_node`: publishes OBD-like simulated data from a CSV file.
- `model_subscriber_node`: loads a trained Keras model and publishes class probabilities.
- `hi_analyzer_node`: visualizes the Health Index over time.

## Installation

```bash
pip3 install -r requirements.txt
