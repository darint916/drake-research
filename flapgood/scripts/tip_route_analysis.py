import json
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import os
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

current_dir = os.path.dirname(os.path.abspath(__file__))
data_dir = os.path.join(current_dir, '..', 'opt_data')
# Load the optimization results
csv_path = os.path.join(data_dir, 'four_bar_tip_pos.csv')
df = pd.read_csv(csv_path)
df = df[(df['time'] >= 3.0) & (df['time'] <= 6.0)]
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(df['x'], df['y'], df['z'])
ax.set_xlabel('X (world)')
ax.set_ylabel('Y (world)')
ax.set_zlabel('Z (world)')
plt.title("Four-bar Tip Trajectory")
plt.show()

fig2, ax2 = plt.subplots()
ax2.plot(df['x'], df['z'])
ax2.set_xlabel('X (world)')
ax2.set_ylabel('Z (world)')
plt.title("Four-bar Tip Trajectory (X–Z Projection)")
plt.show()
