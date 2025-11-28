
"""
This ROS Code for saving Trajectory for both actual followed and desired. 
The trajectory will be saved in a csv file. 
"""
#!/usr/bin/env python3

import os
import csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from message_type.msg import Traj


class TrajectorySaver(Node):
    def __init__(self):
        super().__init__('trajectory_saver_node')

        self.actual = None
        self.desired = None
        self.trajectory = []
        self.start_time = self.get_clock().now()

        self.create_subscription(
            Odometry,
            '/my_lrauv_modified/submarine/odometry',
            self.odom_callback,
            10
        )
        self.create_subscription(
            Traj,
            '/controller/trajectory/desired',
            self.traj_callback,
            10
        )

        self.get_logger().info('Trajectory Saver Node has started.')

    def odom_callback(self, msg: Odometry):
        self.get_logger().info('Got Odometry callback')
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        self.actual = {
            'time': elapsed,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'orientation': msg.pose.pose.orientation,
        }

        if self.desired is None:
            return

        row = [
            self.desired['time'],
            self.desired['x_d'],
            self.desired['y_d'],
            self.actual['x'],
            self.actual['y'],
            self.actual['orientation'].x,
            self.actual['orientation'].y,
            self.actual['orientation'].z,
            self.actual['orientation'].w,
        ]
        self.trajectory.append(row)
        self.get_logger().info(
            f"[t={elapsed:.2f}] Logged x={self.actual['x']:.2f}, "
            f"x_d={self.desired['x_d']:.2f}"
        )

    def traj_callback(self, msg: Traj):
        self.get_logger().info('Got Traj callback')
        self.desired = {
            'time': msg.time,
            'x_d': msg.x_d,
            'y_d': msg.y_d,
        }

    def save_to_csv(self, filename=None):
        if filename is None:
            filename = os.path.expanduser('/home/arash/Documents/Plotting_final/latest/backstepping/firs.csv')

        with open(filename, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                "time", "x_d", "y_d",
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
