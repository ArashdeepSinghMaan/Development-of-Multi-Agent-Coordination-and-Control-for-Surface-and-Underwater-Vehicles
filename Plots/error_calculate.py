"""
This code is for calculating errors for controller to evaluate the performance of the controller. 
"""
import numpy as np
import pandas as pd

# Load actual trajectory
df = pd.read_csv('/home/arash/Documents/Plotting_final/latest/backstepping/first.csv')

# Parameters of your trajectory
#A, B, C, omega = 50.0, 15.0, 2.0, 0.01
t = df['time'].to_numpy()

# Recompute desired trajectory from time
#x_d = A * np.sin(0.8*omega * t)
#y_d = B * np.sin(omega * t) 
#z_d = -2.0 + C * np.sin(omega * t / 2)

# Actual
x = df['x'].to_numpy()
y = df['y'].to_numpy()

x_d =df['x_d'].to_numpy()
y_d=df['y_d'].to_numpy()
#z = df['z'].to_numpy()

# Errors
e_x = x_d - x
e_y = y_d - y
#e_z = z_d - z
e_norm = np.sqrt(e_x**2 + e_y**2  )

dt = np.mean(np.diff(t))

# Metrics (same as before)
mse = np.mean(e_norm**2)
rmse = np.sqrt(mse)
max_error = np.max(e_norm)
final_error = e_norm[-1]
ise = np.sum(e_norm**2) * dt
iae = np.sum(np.abs(e_norm)) * dt
itse = np.sum(t * e_norm**2) * dt
itae = np.sum(t * np.abs(e_norm)) * dt
tolerance = 0.5
within_tolerance_idx = np.where(e_norm < tolerance)[0]
time_to_converge = t[within_tolerance_idx[0]] if len(within_tolerance_idx) > 0 else None

print(f"RMSE: {rmse:.4f}")
print(f"Max Error: {max_error:.4f}")
print(f"Final Error: {final_error:.4f}")
print(f"ISE: {ise:.4f}, IAE: {iae:.4f}, ITSE: {itse:.4f}, ITAE: {itae:.4f}")
print(f"Time to Converge (error < {tolerance}m): {time_to_converge:.2f}s")
