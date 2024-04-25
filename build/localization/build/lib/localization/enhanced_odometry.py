import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Quaternion,TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as TimeMsg
import tf2_ros 
import numpy as np
from math import sin, cos, pi, atan2
import open3d as o3d
from filterpy.kalman import KalmanFilter


class FusedOdom(Node):
	def __init__(self):
		super().__init__('fused_odometry')

		#self.declare_parameter('use_sim_time', True)
		use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
		self.get_logger().info(f"Using Simulation Time: {use_sim_time}")

		self.vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
		self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
		self.odom_publisher = self.create_publisher(Odometry, '/fused_odom', 10)
		self.broadcaster = tf2_ros.TransformBroadcaster(self)

		self.previous_cloud = None

		self.kf = KalmanFilter(dim_x=3, dim_z =3)
		self.init_kf()

		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.theta = 0.0

		self.vx = 0.0
		self.vtheta =0.0

		self.last_time = self.get_clock().now()

	def init_kf(self):
		self.kf.x = np.array([0., 0., 0.])  # Initial state [x, y, heading]
		self.kf.F = np.eye(3)               # State transition matrix
		self.kf.H = np.eye(3)    # Measurement function
		self.kf.R = np.eye(3) * 0.5         # Measurement noise
		self.kf.P *= 10
		self.kf.Q = np.eye(3)*0.1
		
	def vel_callback(self, msg):
		self.vx = msg.linear.x
		self.vtheta = msg.angular.z
		self.publish_odometry()

	def lidar_callback(self, scan_msg):
		angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
		valid = np.where((np.array(scan_msg.ranges) > scan_msg.range_min) &
		                 (np.array(scan_msg.ranges) < scan_msg.range_max))[0]
		points = np.vstack((np.array(scan_msg.ranges)[valid] * np.cos(angles[valid]),
		                    np.array(scan_msg.ranges)[valid] * np.sin(angles[valid]),
		                    np.zeros_like(valid))).T
		cloud = o3d.geometry.PointCloud()
		cloud.points = o3d.utility.Vector3dVector(points)

		if self.previous_cloud is not None:
			self.process_scan_matching(self.previous_cloud, cloud)
		self.previous_cloud = cloud

	def publish_odometry(self):
		current_time = self.get_clock().now()
		dt = (current_time - self.last_time).nanoseconds/ 1e9
		self.last_time = current_time

		delta_x = self.vx*cos(self.vtheta)*dt
		delta_y = self.vx*sin(self.vtheta)*dt
		delta_theta = self.vtheta*dt

		self.theta += delta_theta
		quaternion = self.euler_to_quaternion(0,0, self.theta)

		self.kf.predict()
		self.kf.update([delta_x,delta_y,delta_theta])

		self.x += self.kf.x[0]
		self.y += self.kf.y[1]
		# print("translation x", self.kf.x[0])
		# print("translation y", self.kf.x[1])
		# print("quaternion", quaternion)

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


	def process_scan_matching(self, source, target):
		icp_result = o3d.pipelines.registration.registration_icp(
			        source, target, max_correspondence_distance=0.1, 
			        init=np.eye(4),  # Identity matrix as initial transformation
			        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint())
		self.update_lidar_odometry(icp_result.transformation)		

	def update_lidar_odometry(self, transformation):
		dx = transformation[0, 3] 
		dy = transformation[1, 3] 
		dtheta = atan2(transformation[1, 0], transformation[0, 0])

		self.kf.predict()
		self.kf.update(np.array([[dx], [dy], [dtheta]]))

	def euler_to_quaternion(self, roll, pitch, yaw):
		qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
		qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
		qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
		qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
		return [qx, qy, qz, qw]


def main(args = None):
	rclpy.init(args = args)
	node = FusedOdom()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == "main":
	main()