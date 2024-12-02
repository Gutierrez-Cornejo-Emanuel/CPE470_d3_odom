import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from alpha_delta.msg import AlphaDelta
import math


class OdometryDriver(Node):

    def __init__(self):
        super().__init__('odom_driver')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.alpha_delta_sub = self.create_subscription(AlphaDelta, '/aruco_pose', self.alpha_delta_callback)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile = qos_profile_sensor_data)
        self.distance_moved = 0.0
        self.previous_point_x = -1000
        self.previous_point_y = -1000
        self.aligned = False
        timer_period = 0.3
        self.create_timer(timer_period, self.move30cm)
    def odom_callback(self, msg:Odometry):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self.previous_point_x == -1000:
            self.previous_point_x = x
            self.previous_point_y = y
        self.distance_moved += math.dist((x, y), (self.previous_point_x, self.previous_point_y))
        self.previous_point_x, self.previous_point_y = x,y
        self.get_logger().info('Total distance moved: "%s"' % str(self.distance_moved))
    def move30cm(self):
        if self.distance_moved < 0.3 and self.aligned:
            msg = Twist()
            msg.linear.x = 0.2
            self.publisher_.publish(msg)
    def alpha_delta_callback(self, msg):
        print(f'alpha: {msg.alpha}, delta: {msg.delta}')
        if (abs(msg.alpha) > 3 or msg.alpha == -1000) and not self.aligned:
            twist = Twist()
            twist.angular.z = 0.25 * (-1 if msg.alpha < 0 else 1)
            self.publisher_.publish(twist)
        elif abs(msg.alpha) < 3:
            self.aligned = True

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = OdometryDriver()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
