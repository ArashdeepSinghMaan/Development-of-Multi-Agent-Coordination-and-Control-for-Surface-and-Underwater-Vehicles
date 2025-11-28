import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# --- Read CSV data ---
csv_file = '/home/arash/Documents/Plotting_final/simple.csv'  # Replace with your actual path
df = pd.read_csv(csv_file)

# Extract position columns (case-sensitive!)
x = df['x'].to_numpy()
y = df['y'].to_numpy()
z = z = np.full_like(x, -2.0)+ np.linspace(0, 0.01, len(x)) 


# Time (optional: generate artificial time vector if not in CSV)
t = np.linspace(0, len(x) - 1, len(x))  # use row index as time

# Define snapshot time points (adjust as needed)
snapshots = [0, len(t)*0.2, len(t)*0.3, len(t)*0.5, len(t)-1]
indices = [int(np.clip(round(s), 0, len(t)-1)) for s in snapshots]

# --- Cuboid function ---
def draw_cuboid(ax, center, side, color='orange'):
    cx, cy, cz = center
    s = side/2
    x_range = [cx - s, cx + s]
    y_range = [cy - s, cy + s]
    z_range = [cz - s, cz + s]
    vertices = [
        [(x_range[0], y_range[0], z_range[0]), (x_range[1], y_range[0], z_range[0]),
         (x_range[1], y_range[1], z_range[0]), (x_range[0], y_range[1], z_range[0])],
        # Top face
        [(x_range[0], y_range[0], z_range[1]), (x_range[1], y_range[0], z_range[1]),
         (x_range[1], y_range[1], z_range[1]), (x_range[0], y_range[1], z_range[1])],
        # Front face
        [(x_range[0], y_range[0], z_range[0]), (x_range[1], y_range[0], z_range[0]),
         (x_range[1], y_range[0], z_range[1]), (x_range[0], y_range[0], z_range[1])],
        # Back face
        [(x_range[0], y_range[1], z_range[0]), (x_range[1], y_range[1], z_range[0]),
         (x_range[1], y_range[1], z_range[1]), (x_range[0], y_range[1], z_range[1])],
        # Left face
        [(x_range[0], y_range[0], z_range[0]), (x_range[0], y_range[1], z_range[0]),
         (x_range[0], y_range[1], z_range[1]), (x_range[0], y_range[0], z_range[1])],
        # Right face
        [(x_range[1], y_range[0], z_range[0]), (x_range[1], y_range[1], z_range[0]),
         (x_range[1], y_range[1], z_range[1]), (x_range[1], y_range[0], z_range[1])]
    ]
    ax.add_collection3d(Poly3DCollection(vertices, facecolors=color, edgecolors='k', alpha=0.6))


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

    draw_path_cylinder(ax, x[:idx+1], y[:idx+1], z[:idx+1])
    #draw_cuboid(ax, center=(7.5, -0.6, -2), side=1, color='orange')

    # Start and End markers
    ax.plot(x, y, z, color='royalblue', linewidth=3.5, alpha=0.8)
    ax.scatter(x[0], y[0], z[0], color='green', s=80, marker='o', label='Start')
    ax.scatter(x[-1], y[-1], z[-1], color='red', s=100, marker='X', label='End')
    ax.grid(True)
    fig.patch.set_facecolor('white')

    ax.set_xlim(np.min(x)-5, np.max(x)+5)
    ax.set_ylim(np.min(y)-5, np.max(y)+5)
    ax.set_zlim(np.min(z)-5, np.max(z)+5)
    ax.set_xlabel("X",fontsize=10)
    ax.set_ylabel("Y",fontsize=10)
    ax.set_zlabel("Z",fontsize=10)
    ax.set_title(f"Snapshot {i+1}: Index {idx}",fontsize=10,weight='bold')
    ax.legend(loc='upper right', fontsize=10, frameon=False)
   # range_x = np.ptp(x)
   # range_y = np.ptp(y)
   # range_z = np.ptp(z)

    # Avoid division by zero
   # range_z = range_z if range_z != 0 else 1e-3

  #  ax.set_box_aspect([np.ptp(x), np.ptp(y), np.ptp(z)])
    plt.tight_layout()
        # Save Top View (Z from top)
    ax.view_init(elev=90, azim=-90)
    plt.savefig(f'top_view_snapshot_{i+1}.png', dpi=600, bbox_inches='tight')

    # Save Side View (X-Z plane)
    ax.view_init(elev=0, azim=0)
    plt.savefig(f'side_view_snapshot_{i+1}.png', dpi=600, bbox_inches='tight')

    # Save Front View (Y-Z plane)
    ax.view_init(elev=0, azim=90)
    plt.savefig(f'front_view_snapshot_{i+1}.png', dpi=600, bbox_inches='tight')

   



# --- 2D Trajectory Plot ---
plt.figure(figsize=(8, 6), dpi=300)
plt.plot(x, y, linewidth=2.5, color='royalblue')
plt.scatter(x[0], y[0], color='green', s=80, label='Start')
plt.scatter(x[-1], y[-1], color='red', s=100, label='End')
#draw_cuboid(ax, center=(10, 1.3, -2), side=1, color='orange')
plt.title('2D Top-Down Trajectory', fontsize=12, weight='bold')
plt.xlabel('X')
plt.ylabel('Y')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.tight_layout()
#plt.savefig('2d_trajectory.png', dpi=600, bbox_inches='tight')

plt.show()
