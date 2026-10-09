import csv
import math
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class Capture(Node):
    def __init__(self):
        super().__init__(
            'task3_capture', parameter_overrides=[Parameter('use_sim_time', value=True)]
        )
        self.f = open(sys.argv[1], 'w', buffering=1)
        self.writer = csv.writer(self.f)
        self.writer.writerow(['kind', 'time', 'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'wall_time'])
        self.create_subscription(Odometry, '/odometry/filtered', self.odom, 100)
        self.create_subscription(Twist, '/cmd_vel', self.cmd, 100)

    def odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        v = m.twist.twist
        self.writer.writerow(
            [
                'odom',
                m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                m.pose.pose.position.x,
                m.pose.pose.position.y,
                yaw,
                v.linear.x,
                v.linear.y,
                v.angular.z,
                time.time(),
            ]
        )

    def cmd(self, m):
        self.writer.writerow(
            [
                'cmd',
                self.get_clock().now().nanoseconds * 1e-9,
                0,
                0,
                0,
                m.linear.x,
                m.linear.y,
                m.angular.z,
                time.time(),
            ]
        )


rclpy.init()
n = Capture()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
finally:
    n.f.close()
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
