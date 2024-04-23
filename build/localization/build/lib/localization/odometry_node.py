import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Quaternion,TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
import tf2_ros
from math import sin, cos, pi


class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')

        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f"Using Simulation Time: {use_sim_time}")

        self.subscriber = self.create_subscription(Twist,'/cmd_vel',self.listener_callback,10)
        self.odom_publisher = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0

        self.vx = 0.0
        self.vtheta = 0.0

        self.last_time = self.get_clock().now()
        #self.timer = self.create_timer(0.1, self.publish_odometry)  # 20 Hz

    def listener_callback(self, msg):
        self.vx = msg.linear.x
        self.vtheta = msg.angular.z
        self.publish_odometry()


    def publish_odometry(self):
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        delta_x = (self.vx * cos(self.theta)) * delta_time
        delta_y = (self.vx * sin(self.theta)) * delta_time
        delta_theta = self.vtheta * delta_time

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        quaternion = self.euler_to_quaternion(0,0,self.theta)

        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        self.odom_publisher.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = current_time.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = self.z
        tf.transform.rotation.x = quaternion[0]
        tf.transform.rotation.y = quaternion[1]
        tf.transform.rotation.z = quaternion[2]
        tf.transform.rotation.w = quaternion[3]
        self.broadcaster.sendTransform(tf)  

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry() 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

