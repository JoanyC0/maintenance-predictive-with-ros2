# 🛠️ maintenance-predictive-with-ros2

**ROS 2 package** for predictive maintenance in vehicles using a trained **LSTM or GRU** model.  
The system simulates OBD data, predicts a Health Index using a Keras model, compares predicted and actual values, and visualizes their evolution in a dynamic graph.

---

## 📦 Nodes

- `obd_publisher_node`  
  Publishes simulated OBD data from a CSV file.

- `model_subscriber_node`  
  Loads a trained Keras model and publishes the predicted Health Index.

- `hi_actual_node`  
  Publishes the actual Health Index values from the dataset for comparison.

- `hi_analyzer_node`  
  Compares real and predicted values, publishes the difference, and visualizes the curves with a dynamic confidence zone around the predictions.

---

## ⚙️ Installation

Install the Python dependencies:

```bash
pip3 install -r requirements.txt
