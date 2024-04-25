import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf2_ros 
import numpy as np
from math import sin, cos, pi, atan2
import open3d as o3d
from scipy.spatial.transform import Rotation as R

class FusedOdom(Node):
	def __init__(self):
		super().__init__('fused_odometry')
		# use_sim_time = self.get_parameter('use_sim_time', True).get_parameter_value().bool_value
		# self.get_logger().info(f"Using Simulation Time: {use_sim_time}")

		self.vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
		self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
		self.odom_publisher = self.create_publisher(Odometry, '/fused_odom', 10)
		self.broadcaster = tf2_ros.TransformBroadcaster(self)

		self.previous_cloud = None
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.theta = 0.0

		self.vx = 0.0

		self.transformation = np.eye(4)  # Initialize as identity matrix
		self.last_time = self.get_clock().now()

	def vel_callback(self, msg):
		self.vx = msg.linear.x

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

	def process_scan_matching(self, source, target):
		icp_result = o3d.pipelines.registration.registration_icp(
		    target, source, max_correspondence_distance=0.05, 
		    init=np.eye(4),  # Use the last transformation as the initial guess
		    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint())
		self.transformation = np.dot(icp_result.transformation, self.transformation)
		self.update_lidar_odometry(self.transformation,icp_result.transformation)        

	def update_lidar_odometry(self, comp_transformation,transformation):
		current_time = self.get_clock().now()
		delta_time = (current_time - self.last_time).nanoseconds / 1e9
		self.last_time = current_time


		delta_x = transformation[0, 3]
		delta_y = transformation[1, 3]
		delta_theta = atan2(transformation[1, 0], transformation[0, 0])

		global_delta_x = delta_x * sin(self.theta) - delta_y * cos(self.theta)
		global_delta_y = delta_x * sin(self.theta) + delta_y * cos(self.theta)

		self.x += global_delta_x
		self.y += global_delta_y
		self.theta += delta_theta

		rotation_matrix = comp_transformation[:3, :3]
		rotation = R.from_matrix(rotation_matrix)        
		quaternion = rotation.as_quat()

		odom = Odometry()
		odom.header = Header()
		odom.header.stamp = self.get_clock().now().to_msg()
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
		tf.header.stamp = self.get_clock().now().to_msg()
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

if __name__ == "__main__":
	main()
