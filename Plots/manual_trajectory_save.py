#!/usr/bin/env python3

import os
import csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class TrajectorySaver(Node):
    def __init__(self):
        super().__init__('trajectory_saver_node')

        self.trajectory = []
        self.start_time = self.get_clock().now()

        self.create_subscription(
            Odometry,
            '/my_lrauv_modified/submarine/odometry',
            self.odom_callback,
            10
        )

        self.get_logger().info('Trajectory Saver Node has started.')

    def odom_callback(self, msg: Odometry):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        orientation = msg.pose.pose.orientation

        row = [
            elapsed,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ]
        self.trajectory.append(row)

        self.get_logger().info(
            f"[t={elapsed:.2f}] Logged x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}"
        )

    def save_to_csv(self, filename=None):
        if filename is None:
            filename = os.path.expanduser('/home/arash/Documents/Plotting_final/Manual/first.csv')

        with open(filename, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                "time", 
                "x", "y",
                "qx", "qy", "qz", "qw"
            ])
            writer.writerows(self.trajectory)

        self.get_logger().info(f"Trajectory saved to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, saving trajectory...')
    finally:
        node.save_to_csv()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
