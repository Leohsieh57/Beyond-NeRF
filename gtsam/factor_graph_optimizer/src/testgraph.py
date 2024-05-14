import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
# Load data from CSV
data = pd.read_csv('/home/anaisjeger/catkin_ws/src/Beyond-NeRF/gtsam/factor_graph_optimizer/pose_data.csv')

# Renaming the columns to x, y, and z for clarity
data.columns = ['x', 'y', 'z']
# Convert the Series to numpy arrays
x_values = np.array(data['x'])
y_values = np.array(data['y'])

# Plotting
plt.figure(figsize=(10, 5))
plt.plot(x_values, y_values, marker='o', linestyle='-', markersize=4, label='Trajectory')
plt.title('XY Trajectory Plot')
plt.xlabel('X position')
plt.ylabel('Y position')
plt.legend()
plt.grid(True)
plt.show()


