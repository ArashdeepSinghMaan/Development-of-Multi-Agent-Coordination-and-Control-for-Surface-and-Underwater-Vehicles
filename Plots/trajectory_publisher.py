"""
This ROS NOde is responsible for publishing the user required feasible trajectory
"""
#!/usr/bin/env python3

import os
import csv
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import tf2_ros
import tf2_geometry_msgs

from math import atan2, asin

from math import atan2, asin

def quaternion_to_euler(x, y, z, w):
    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = atan2(t0, t1)

    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = asin(t2)

    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)

    return roll, pitch, yaw

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher_node')

        # Publishers for x, y, and yaw
        self.pub_x = self.create_publisher(Float64, '/user/trajectory/x', 10)
        self.pub_y = self.create_publisher(Float64, '/user/trajectory/y', 10)
        self.pub_yaw = self.create_publisher(Float64, '/user/trajectory/yaw', 10)

        self.trajectory = []
        self.index = 0

        # Load CSV
        filename = os.path.expanduser('/home/arash/Documents/Plotting_final/Manual/30_5.csv')
        self.load_csv(filename)

        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_next)

        self.get_logger().info('Trajectory Publisher Node has started.')

    def load_csv(self, filename):
        try:
            with open(filename, 'r') as csv_file:
                reader = csv.DictReader(csv_file)
                for row in reader:
                    x = float(row['x'])
                    y = float(row['y'])
                    qx = float(row['qx'])
                    qy = float(row['qy'])
                    qz = float(row['qz'])
                    qw = float(row['qw'])

                    # Convert quaternion to yaw (Euler angle in radians)
                    _, _, yaw = quaternion_to_euler(qx, qy, qz, qw)
                    self.trajectory.append((x, y, yaw))
            self.get_logger().info(f"Loaded {len(self.trajectory)} points from {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to load CSV: {e}")

    def publish_next(self):
        if self.index < len(self.trajectory):
            x, y, yaw = self.trajectory[self.index]

            self.pub_x.publish(Float64(data=x))
            self.pub_y.publish(Float64(data=y))
            self.pub_yaw.publish(Float64(data=yaw))

            self.get_logger().info(f"[{self.index}] x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad")
            self.index += 1
        else:
            self.get_logger().info("Finished publishing all trajectory points.")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
