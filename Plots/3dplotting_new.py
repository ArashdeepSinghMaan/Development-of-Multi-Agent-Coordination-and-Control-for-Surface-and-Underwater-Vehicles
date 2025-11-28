"""
This file is used to make plots for actual followed trajectory and desired trajectory
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# === Load Actual Data from CSV ===
csv_file = '/home/arash/Documents/Plotting_final/latest/backstepping/second.csv'
df_actual = pd.read_csv(csv_file)

# Extract actual position data
x_a = df_actual['x'].to_numpy()
y_a = df_actual['y'].to_numpy()

# Use real z if present, otherwise simulate
if 'z' in df_actual.columns:
    z_a = df_actual['z'].to_numpy()
else:
    z_a = np.full_like(x_a, -2.0) + np.linspace(0, 0.01, len(x_a))  # Simulated Z
    z_d = np.full_like(x_a, -2.0) + np.linspace(0, 0.01, len(x_a))


x_d =df_actual['x_d'].to_numpy()
y_d=df_actual['y_d'].to_numpy()
# === 3D Plot: Actual vs Desired ===
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

ax.plot(x_d, y_d,z_d,  color='orange', label='Desired Trajectory', linewidth=2)
ax.plot(x_a, y_a,z_a,  color='blue', label='Actual Trajectory', linewidth=2)

ax.scatter(x_a[0], y_a[0], z_d, color='green', s=40, label='Start')
ax.scatter(x_a[-1], y_a[-1],z_a,  color='red', s=40, label='End')

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
plt.plot(x_d, y_d, '*', color='red', label='Desired Trajectory', linewidth=2)
plt.plot(x_a, y_a, '--', color='blue', label='Actual Trajectory', linewidth=2)
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







"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.patches as patches

# --- Read CSV data ---
csv_file = '/home/arash/Documents/Plotting_final/Cascaded FeedBack Linearization/case1.csv'  # Replace with your actual path
df = pd.read_csv(csv_file)

# Extract position columns (case-sensitive!)
x = df['x'].to_numpy()
y = df['y'].to_numpy()
z = z = np.full_like(x, -2.0)+ np.linspace(0, 0.01, len(x)) 


# Time (optional: generate artificial time vector if not in CSV)
t = np.linspace(0, len(x) - 1, len(x))  # use row index as time

# Define snapshot time points (adjust as needed)
#snapshots = [0, len(t)*0.2, len(t)*0.3, len(t)*0.5, len(t)-1]
snapshots = [0]
indices = [int(np.clip(round(s), 0, len(t)-1)) for s in snapshots]

# --- Cube function ---
def draw_cube(ax, center, size=1.0, color='orange',label=None):
    cx, cy, cz = center
    s = size / 2
    vertices = np.array([
        [cx - s, cy - s, cz - s],
        [cx + s, cy - s, cz - s],
        [cx + s, cy + s, cz - s],
        [cx - s, cy + s, cz - s],
        [cx - s, cy - s, cz + s],
        [cx + s, cy - s, cz + s],
        [cx + s, cy + s, cz + s],
        [cx -  s, cy + s, cz + s]
    ])

    faces = [
        [vertices[j] for j in [0, 1, 2, 3]],
        [vertices[j] for j in [4, 5, 6, 7]],
        [vertices[j] for j in [0, 1, 5, 4]],
        [vertices[j] for j in [2, 3, 7, 6]],
        [vertices[j] for j in [1, 2, 6, 5]],
        [vertices[j] for j in [0, 3, 7, 4]]
    ]

    cube = Poly3DCollection(faces, facecolors=color, edgecolors='black', linewidths=0.6, alpha=0.7)
   
    if label:
        cube.set_label(label)
    ax.add_collection3d(cube)
    return cube

# --- Path tracer ---
def draw_path_cylinder(ax, x, y, z, color='royalblue'):
    for i in range(len(x) - 1):
        ax.plot([x[i], x[i+1]], [y[i], y[i+1]], [z[i], z[i+1]], color=color, linewidth=4)


# --- Plot each snapshot in a separate figure ---
for i, idx in enumerate(indices):
    fig = plt.figure(figsize=(10, 8),dpi=300)
    ax = fig.add_subplot(111, projection='3d')
    #Creates a new figure of size 10x8 inches with high resolution (dpi=300).
    #Adds a 3D subplot (111 = 1 row, 1 column, 1st subplot).

   # draw_path_cylinder(ax, x[:idx+1], y[:idx+1], z[:idx+1])
   # draw_cube(ax, center=(15, 0.1, -2), size=2, color='orange',label='obstacle')
   # draw_cube(ax, center=(17, 1, -2), size=1, color='grey',label='obstacle')
  #  draw_cube(ax, center=(14, 2.5, -2), size=1, color='silver',label='obstacle')

    # Start and End markers
    ax.plot(x, y, z, color='blue', linewidth=1.5, alpha=0.8)
    ax.scatter(x[0], y[0], z[0], color='green', s=20, marker='o', label='Start')
    ax.scatter(x[-1], y[-1], z[-1], color='red', s=20, marker='X', label='End')
    ax.tick_params(axis='both', labelsize=5)
    ax.grid(True)
    fig.patch.set_facecolor('white')

    ax.set_xlim(np.min(x)-10, np.max(x)+10)
    ax.set_ylim(np.min(y)-10, np.max(y)+10)
    ax.set_zlim(np.min(z)-5, np.max(z)+5)
    ax.set_xlabel("X",fontsize=5)
    ax.set_ylabel("Y",fontsize=5)
    ax.set_zlabel("Z",fontsize=5)
    ax.set_title(f"3D Trajectory with Single Obstacle",fontsize=8,weight='bold')

    ax.legend(loc='upper right', fontsize=6, frameon=False)
    ax.set_box_aspect([1, 1, 1])  # x:y:z = 1:1:1

   # range_x = np.ptp(x)
   # range_y = np.ptp(y)
   # range_z = np.ptp(z)

    # Avoid division by zero
   # range_z = range_z if range_z != 0 else 1e-3

   # ax.set_box_aspect([np.ptp(x), 8, 4])
   # plt.tight_layout()
        # Save Top View (Z from top)
    #ax.view_init(elev=90, azim=-90)
   # plt.savefig(f'top_view_snapshot_{i+1}.png', dpi=600, bbox_inches='tight')

    # Save Side View (X-Z plane)
    #ax.view_init(elev=0, azim=0)
    #plt.savefig(f'side_view_snapshot_{i+1}.png', dpi=600, bbox_inches='tight')

    # Save Front View (Y-Z plane)
  #  ax.view_init(elev=0, azim=90)
   # plt.savefig(f'front_view_snapshot_{i+1}.png', dpi=600, bbox_inches='tight')

   



# --- 2D Trajectory Plot ---


# Add the square to the axes
# --- 2D Trajectory Plot ---
xx = 15        # x-coordinate of bottom-left corner
yy = 0.1       # y-coordinate of bottom-left corner
sside = 2       # side length of the square

# Create a new 2D figure and axis
fig2, ax2 = plt.subplots(figsize=(8, 6), dpi=300)

# Add the trajectory and points
ax2.plot(x, y, linewidth=2.5, color='royalblue')
ax2.scatter(x[0], y[0], color='green', s=80, label='Start')
ax2.scatter(x[-1], y[-1], color='red', s=100, label='End')

# Create and add the square
#square = patches.Rectangle((xx, yy), sside, sside, linewidth=1.5, edgecolor='blue', facecolor='cyan',label='obstacle')
#ax2.add_patch(square)
#square = patches.Rectangle((18, 1), 1, 1, linewidth=1.5, edgecolor='blue', facecolor='cyan')
#ax2.add_patch(square)
#square = patches.Rectangle((15, 2.5), 1, 1, linewidth=1.5, edgecolor='blue', facecolor='cyan')
#ax2.add_patch(square)
# Finalize the plot
ax2.set_title('2D Top-Down Trajectory with Single Obstacle', fontsize=12, weight='bold')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.axis('equal')
ax2.grid(True)
ax2.legend()
plt.tight_layout()
plt.show()
"""