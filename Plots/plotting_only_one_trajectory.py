"""This code is to plot only one trajectory """
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# === Load Actual Data from CSV ===
csv_file = '/home/arash/Documents/Plotting_final/Manual/first.csv'
df_actual = pd.read_csv(csv_file)

# Extract actual position data
x_a = df_actual['x'].to_numpy()
y_a = df_actual['y'].to_numpy()

# Use real z if present, otherwise simulate
if 'z' in df_actual.columns:
    z_a = df_actual['z'].to_numpy()
else:
    z_a = np.full_like(x_a, -2.0) + np.linspace(0, 0.01, len(x_a))  # Simulated Z


fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')


ax.plot(x_a, y_a,  color='blue', label='Actual Trajectory', linewidth=2)

ax.scatter(x_a[0], y_a[0],  color='green', s=40, label='Start')
ax.scatter(x_a[-1], y_a[-1],  color='red', s=40, label='End')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Trajectory: Actual vs Desired', fontsize=12, weight='bold')
ax.legend()
ax.grid(True)
ax.set_box_aspect([1,1,1])
plt.tight_layout()
plt.show()

# === 2D Plot: X-Y Top-Down View ===
plt.figure(figsize=(8, 6))

plt.plot(x_a, y_a, '-', color='blue', label='Actual Trajectory', linewidth=2)
plt.scatter(x_a[0], y_a[0], color='green', s=40, label='Start')
plt.scatter(x_a[-1], y_a[-1], color='red', s=40, label='End')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('2D Top-Down View: Actual vs Desired Trajectory')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()


