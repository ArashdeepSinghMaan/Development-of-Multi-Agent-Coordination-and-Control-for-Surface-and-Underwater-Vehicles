import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
csv_file = '/home/arash/Documents/Plotting_final/default_trajectory.csv'  # Replace with your actual path
df = pd.read_csv(csv_file)

# Extract position columns (case-sensitive!)
x = df['X']
y = df['Y']
z = df['Z']

# Plotting in 3D
fig = plt.figure(figsize=(10, 7))  # Set figure size
ax = fig.add_subplot(111, projection='3d')

# Set the trajectory line style
ax.plot(x, y, z, label='Trajectory', color='purple', linewidth=2, marker='o', markersize=5)

# Enhance the grid for better readability
ax.grid(True, linestyle='--', alpha=0.5)

# Labels with more professional fonts and sizes
ax.set_xlabel('X', fontsize=14, labelpad=10)
ax.set_ylabel('Y', fontsize=14, labelpad=10)
ax.set_zlabel('Z', fontsize=14, labelpad=10)
ax.set_title('3D Trajectory Plot', fontsize=16, pad=20)

# Set axis limits for better control and symmetry (optional)
ax.set_xlim([x.min() - 1, x.max() + 1])
ax.set_ylim([y.min() - 1, y.max() + 1])
ax.set_zlim([z.min() - 1, z.max() + 1])

# Improve tick appearance
ax.tick_params(axis='both', which='major', labelsize=12)
ax.view_init(30, 45)  # Set the angle for the 3D view

# Add a legend with a clear style
ax.legend(loc='upper left', fontsize=12, frameon=False, fancybox=True, shadow=True)

# Optional: Add a color gradient for more visual appeal
# This could be used if you want to add a gradient based on z-values or other metrics
# from matplotlib import cm
# norm = plt.Normalize(z.min(), z.max())
# cmap = cm.viridis
# ax.scatter(x, y, z, c=z, cmap=cmap, norm=norm)

plt.tight_layout()
plt.show()