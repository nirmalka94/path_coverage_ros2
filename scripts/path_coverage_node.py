#!/usr/bin/env python3


# ----------- be careful!! export ROS_DOMAIN_ID=2
# . /usr/share/gazebo/setup.sh; source /opt/ros/humble/setup.bash; source ros2_ws/install/setup.bash; ros2 run path_coverage path_coverage_node.py




import os
import numpy as np
import pdb
import json
import tempfile
from shapely.geometry import Polygon, Point # pip3 install shapely
from math import *
import time
import subprocess # sudo apt-get install ruby-full
import yaml

from scipy.spatial.transform import Rotation

from path_coverage.list_helper import * #from itertools import pairwise
from path_coverage.trapezoidal_coverage import calc_path as trapezoid_calc_path
from path_coverage.border_drive import border_calc_path

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point as rosPoint

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import ReliabilityPolicy, QoSProfile
from ament_index_python.packages import get_package_share_directory

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateThroughPoses, ComputePathToPose

from geometry_msgs.msg import PointStamped, PoseStamped
from action_msgs.msg import GoalStatus




INSCRIBED_INFLATED_OBSTACLE = 253





class MapDrive(Node): 
	def __init__(self):
		super().__init__('map_drive') 

		# self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		# rate = 50
		# self.looprate = rospy.Rate(rospy.get_param('~hz', rate))
		# linear_speed = 0.2
		# goal_distance = 0.1
		# linear_duration = goal_distance / linear_speed
		# self.cmd = Twist()
		# self.cmd.linear.x = linear_speed
		# self.count = int(linear_duration * rate)

		self.sub_node = rclpy.create_node('path_verifier_client')
		self.sub_node.get_logger().info('Created path_verifier_client node')
		self.compute_path_to_pose_client = ActionClient(self.sub_node, ComputePathToPose, 'compute_path_to_pose')
		self.goal_msg = ComputePathToPose.Goal()
		self.x = None
		self.y = None
		
		self.rospack  = get_package_share_directory('path_coverage') 

		# Define the file name for the YAML file
		self.filename = "/home/hazeezadebayo/ros2_ws/src/robovak2_test/params/full_path_cov_poses.yaml" # "pose_output.yaml"

		# Initialize 
		self.pose_output = {}
		self.last_points = {}
		self.last_path = None
		self.lClickPoints = []
		self.local_costmap = None
		self.global_costmap = None
		self.goal_handle = None
		self.result_future = None
		self.feedback = None

		self.declare_parameter("global_frame", "map")
		self.declare_parameter("robot_width", 0.6) 
		self.declare_parameter("costmap_max_non_lethal", 70)
		self.declare_parameter("boustrophedon_decomposition", True)
		self.declare_parameter("border_drive", False)
		self.declare_parameter("base_frame", "base_footprint")

		self.global_frame = self.get_parameter("global_frame").get_parameter_value().string_value 
		self.robot_width = self.get_parameter("robot_width").get_parameter_value().double_value
		self.costmap_max_non_lethal = self.get_parameter("costmap_max_non_lethal").get_parameter_value().integer_value
		self.boustrophedon_decomposition = self.get_parameter("boustrophedon_decomposition").get_parameter_value().bool_value #  False #
		self.border_drive = self.get_parameter("border_drive").get_parameter_value().bool_value
		self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value

		self.create_subscription(PointStamped, "/clicked_point", self.rvizPointReceived, 1)
		# self.global_map_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.map_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
		self.create_subscription(OccupancyGrid, f"/global_costmap/costmap", self.globalCostmapReceived, 10) 
		self.create_subscription(OccupancyGrid, f"/local_costmap/costmap", self.localCostmapReceived, 10)  

		self.pub_marker = self.create_publisher(Marker, 'path_coverage_marker', 16) 
		
		self.tfBuffer = Buffer()
		self.tf_listener = TransformListener(self.tfBuffer, self)

		self.get_logger().info('parameters::::global_frame::robot_width::costmap_max_non_lethal::boustrophedon_decomposition::border_drive::base_frame.')
		self.get_logger().info('::::::::::::::'+str(self.global_frame)+'::'+str(self.robot_width)+'::'+str(self.costmap_max_non_lethal)+'::'+str(self.boustrophedon_decomposition)+'::'+str(self.border_drive)+'::'+str(self.base_frame)+'.')
		self.get_logger().info("Path coverage node initialized successfully...")
	





	def localCostmapReceived(self, costmap):
		#self.get_logger().info('local costmap received')
		self.local_costmap = costmap
		self.local_costmap_width = costmap.info.width*costmap.info.resolution
		self.local_costmap_height = costmap.info.height*costmap.info.resolution





	def globalCostmapReceived(self, costmap):
		#self.get_logger().info('global costmap received')
		self.global_costmap = costmap


		

	
	def visualization_cleanup(self):
		for id, points in self.last_points.items():
			if points is not None:
				self.visualize_trapezoid(points, id=id, show=False)
			self.last_points = {}
		if self.last_path is not None:
			self.visualize_path(self.last_path, False)
			self.last_path = None






	def visualize_cell(self, points, show=True, close=True):
		self.visualize_trapezoid(points, show, close)





	def visualize_area(self, points, show=True, close=True):
		self.visualize_trapezoid(points, show, close, id=1, red=1.0, blue=0.0)





	def visualize_trapezoid(self, points, show=True, close=True, id=0, red=0.0, green=0.0, blue=1.0):
		#self.get_logger().info("viz_trapezoid- 01")# -------------------------------------
		if len(points) < 2: 
			return
		#self.get_logger().info("viz_trapezoid- 02") # -------------------------------------
		self.last_points[id] = points if show else None
		#self.get_logger().info("viz_trapezoid- 03: ") # -------------------------------------
		msg = Marker()
		msg.header.frame_id = self.global_frame
		msg.header.stamp = self.get_clock().now().to_msg() # rospy.Time.now()
		msg.ns = "trapezoid"
		# msg.lifetime = Duration(seconds=0).to_msg() # rospy.Duration(0)
		msg.id = id
		msg.type = Marker.LINE_STRIP
		msg.action = Marker.ADD if show else Marker.DELETE
		msg.pose.orientation.w = float(1)
		msg.pose.orientation.x = float(0)
		msg.pose.orientation.y = float(0)
		msg.pose.orientation.z = float(0)
		msg.scale.x = 0.02
		# blue
		msg.color.r = red
		msg.color.g = green
		msg.color.b = blue
		msg.color.a = 1.0
		#self.get_logger().info("viz_trapezoid- 04: ") # -------------------------------------

		if close:
			points = points + [points[0]]
		#	self.get_logger().info("viz_trapezoid- 05: ") # -------------------------------------
		for point in points:
			point_msg = rosPoint()
			point_msg.x = point[0]
			point_msg.y = point[1]
			msg.points.append(point_msg)
		#	self.get_logger().info("viz_trapezoid- 06: ") # -------------------------------------

		self.pub_marker.publish(msg)
		#self.get_logger().info("viz_trapezoid- 07: ") # -------------------------------------
		time.sleep(0.3)
		self.get_logger().info("viz_trapezoid completed...") # -------------------------------------






	def visualize_path(self, path, show=True):
		i = 0
		#self.get_logger().info("visualize_path- 001: ") # -------------------------------------
		self.last_path = path if show else None
		#self.get_logger().info("visualize_path- 002: ") # -------------------------------------
		for pos_last,pos_cur in pairwise(path):
		#	self.get_logger().info("visualize_path- 003: ") # -------------------------------------
			msg = Marker()
			msg.header.frame_id = self.global_frame
			msg.header.stamp =  self.get_clock().now().to_msg() # rospy.Time.now()
			msg.ns = "path"
			# msg.lifetime = Duration(seconds=2).to_msg() # erases the line after 2 secs.
			msg.id = i
			msg.type = Marker.ARROW
			msg.action = Marker.ADD if show else Marker.DELETE
			msg.pose.orientation.w = float(1)
			msg.pose.orientation.x = float(0)
			msg.pose.orientation.y = float(0)
			msg.pose.orientation.z = float(0)
			msg.scale.x = 0.01 # shaft diameter
			msg.scale.y = 0.03 # head diameter
			# green
			msg.color.g = 1.0
			msg.color.a = 1.0
		#	self.get_logger().info("visualize_path- 004: ") # -------------------------------------

			point_msg_start = rosPoint()
			point_msg_start.x = pos_last[0]
			point_msg_start.y = pos_last[1]
			msg.points.append(point_msg_start)
			point_msg_end = rosPoint()
			point_msg_end.x = pos_cur[0]
			point_msg_end.y = pos_cur[1]
			msg.points.append(point_msg_end)
		#	self.get_logger().info("visualize_path- 005: ") # -------------------------------------

			i+=1
		#	self.get_logger().info("visualize_path- 006: ") # -------------------------------------
			self.pub_marker.publish(msg)
			time.sleep(0.3)
		self.get_logger().info("visualize_path completed...") # -------------------------------------




	



	def rvizPointReceived(self, point):
		# print('i heard: ', point)
		self.lClickPoints.append(point)
		points = [(p.point.x, p.point.y) for p in self.lClickPoints]
		self.global_frame = point.header.frame_id
		# print('len(self.lClickPoints): ', len(self.lClickPoints))
		if len(self.lClickPoints) > 2:
			# All points must have same frame_id
			if len(set([p.header.frame_id for p in self.lClickPoints])) != 1:
				raise ValueError()
			points_x = [p.point.x for p in self.lClickPoints]
			points_y = [p.point.y for p in self.lClickPoints]
			avg_x_dist = list_avg_dist(points_x)
			avg_y_dist = list_avg_dist(points_y)
			dist_x_first_last = abs(points_x[0] - points_x[-1])
			dist_y_first_last = abs(points_y[0] - points_y[-1])
			if dist_x_first_last < avg_x_dist/10.0 and dist_y_first_last < avg_y_dist/10.0:
				# last point is close to maximum, construct polygon
				self.get_logger().info("Creating polygon %s" % (str(points)))
				#self.get_logger().info("i got here 0")
				self.visualize_area(points, close=True)
				#self.get_logger().info("i got here 00")
				if self.boustrophedon_decomposition:
					self.get_logger().info("do_boustrophedon initiated...")
					self.do_boustrophedon(Polygon(points), self.global_costmap)
				else:
					self.get_logger().info("drive_polygon initiated...")
					self.drive_polygon(Polygon(points))
				self.visualize_area(points, close=True, show=False)
				self.lClickPoints = []
				# this signifies the end afterwhich everything is cleaned.
				self.get_logger().info("writing the data to the YAML file...")
				# empty the pose_output dict
				# Write the data to the YAML file
				self.pose_output["update_time"] = time.time_ns()
				with open(self.filename, "w") as f:
					yaml.dump(self.pose_output, f)
				self.pose_output = {}	
				self.get_logger().info("this signifies the end afterwhich everything is cleaned")
				return
			# self.get_logger().info("i got here 6:")
		self.visualize_area(points, close=False)
		self.get_logger().info("finished successfully rvizPointReceived func.")



	def make_Polygons_shapely_polygons(self, Polygons):
		polygons = []
		polygon_area = []
		for polygon in Polygons:
			coords = [(x, y) for x, y in polygon]
			shapely_polygon = Polygon(coords)
			area = shapely_polygon.area
			polygons.append(shapely_polygon)
			polygon_area.append(area)
		return polygons, polygon_area


	def are_polygons_connected(self, poly1, poly2, threshold=2): # 35
		p1 = Polygon(poly1)
		p2 = Polygon(poly2)
		for c1 in p1.exterior.coords:
			for c2 in p2.exterior.coords:
				if math.sqrt((c1[0]-c2[0])**2 + (c1[1]-c2[1])**2) <= threshold:
					# print("c1,c2 :", c1,c2)
					return True
		return False



	def are_polygons_connected_with_increased_thresh(self, poly1, poly2):
		return self.are_polygons_connected(poly1, poly2, threshold=10)


	def find_connected_polygons(self, Polygons, polygon_area_threshold=150):

		if len(Polygons) <= 2:
			return Polygons

		polygons, polygon_area = self.make_Polygons_shapely_polygons(Polygons)

		connected_polygons = []
		associated_polygons = set()

		for i, poly1 in enumerate(polygons):

			index1 = polygons.index(poly1)

			for j, poly2 in enumerate(polygons[i+1:], start=i+1):
				if self.are_polygons_connected(poly1, poly2):
					index2 = polygons.index(poly2)
					connected_polygons.append((index1, index2))
					associated_polygons.add(poly1)
					associated_polygons.add(poly2)

			if poly1 not in associated_polygons:
				for poly2 in polygons:
				#for j, poly2 in enumerate(polygons[i+1:], start=i+1):
					if poly2 != poly1:
						if self.are_polygons_connected_with_increased_thresh(poly1, poly2):
							index2 = polygons.index(poly2)
							connected_polygons.append((index1, index2))
							associated_polygons.add(poly1)
							associated_polygons.add(poly2)
							break

			if poly1 not in associated_polygons:    
				connected_polygons.append((index1,))
				associated_polygons.add(poly1)

		if connected_polygons:

			order = []
			parent_children = {}

			for pair in connected_polygons:
				# print(pair)

				if isinstance(pair, tuple) and len(pair) == 2:
					parent, child = pair
					parent_children.setdefault(parent, []).append(child)

			for parent in range(len(connected_polygons)):
				if parent not in parent_children:
					continue
				order.append(parent)
				stack = parent_children[parent][::-1]
				while stack:
					node = stack.pop()
					if node not in parent_children:
						order.append(node)
						continue
					order.extend([node]+parent_children[node][::-1])			

			new_order = []
			for elem in order:
				if elem not in new_order:
					new_order.append(elem)
			
			# print("new_order: ", new_order)
			# self.get_logger().info('new_order: ' + str(new_order) + '.')

			ordered_polygons = [Polygons[i] for i in new_order]
			ordered_polygon_areas = [polygon_area[i] for i in new_order]

			filtered_polygons = []; 

			for polygon, area in zip(ordered_polygons, ordered_polygon_areas):
				if area >= polygon_area_threshold:
					filtered_polygons.append(polygon)

			parent_children = None; del parent_children
			order = None; del order
			new_order = None; del new_order
			ordered_polygons = None; del ordered_polygons
			ordered_polygon_areas = None; del ordered_polygon_areas

			#print("ordered_polygons: ", ordered_polygons)
			return filtered_polygons
		else:
			print("No polygons are connected.")















	def do_boustrophedon(self, poly, costmap):
		# Cut polygon area from costmap
		#self.get_logger().info("1: ") # -------------------------------------
		(minx, miny, maxx, maxy) = poly.bounds
		#self.get_logger().info("2: ") # -------------------------------------
		#self.get_logger().info("Converting costmap at x=%.2f..%.2f, y=%.2f..%.2f for Boustrophedon Decomposition" % (minx, maxx, miny, maxy))
		#self.get_logger().info("3: ")
		#self.get_logger().info("3: ", minx, miny, maxx, maxy) # -------------------------------------
		#self.get_logger().info("3.5: ", costmap.info.origin.position.x, costmap.info.origin.position.y, costmap.info.resolution) # -------------------------------------
		#self.get_logger().info(" ")
		# Convert to costmap coordinate
		minx = round((minx-costmap.info.origin.position.x)/costmap.info.resolution)
		maxx = round((maxx-costmap.info.origin.position.x)/costmap.info.resolution)
		miny = round((miny-costmap.info.origin.position.y)/costmap.info.resolution)
		maxy = round((maxy-costmap.info.origin.position.y)/costmap.info.resolution)
		#self.get_logger().info("4: ") # -------------------------------------
		# Check min/max limits
		if minx < 0: minx = 0
		if maxx > costmap.info.width: maxx = costmap.info.width
		if miny < 0: miny = 0
		if maxy > costmap.info.height: maxy = costmap.info.height
		#self.get_logger().info("5: ") # -------------------------------------
		# Transform costmap values to values expected by boustrophedon_decomposition script
		rows = []
		for ix in range(int(minx), int(maxx)):
			# self.get_logger().info("6: ") # -------------------------------------
			column = []
			for iy in range(int(miny), int(maxy)):
				x = ix*costmap.info.resolution+costmap.info.origin.position.x
				y = iy*costmap.info.resolution+costmap.info.origin.position.y
				data = costmap.data[int(iy*costmap.info.width+ix)]
				if data == -1 or not poly.contains(Point([x,y])):
					# Unknown or not inside polygon: Treat as obstacle
					column.append(0)
				elif data <= self.costmap_max_non_lethal:
					# Freespace (non-lethal)
					column.append(-1)
				else:
					# Obstacle
					column.append(0)
			rows.append(column)


		#print("7: ") # -------------------------------------
		# self.get_logger().info("=..................=") 

		polygons = []
		with tempfile.NamedTemporaryFile(delete=False,mode='w') as ftmp:
			ftmp.write(json.dumps(rows))
			ftmp.flush()

		#	print("8: ") # -------------------------------------
			
			boustrophedon_script = os.path.join(self.rospack, "scripts/boustrophedon_decomposition.rb") # boustrophedon_script = os.path.join(self.rospack.get_path('path_coverage'), "scripts/boustrophedon_decomposition.rb")
			
		#	print("9: ") # -------------------------------------

			try:
				result = subprocess.run(["ruby", boustrophedon_script, ftmp.name], capture_output=True, text=True)
				polygons = json.loads(result.stdout)
			except subprocess.CalledProcessError as e:
				print("**** Error: ", e)


		#self.get_logger().info("10: ") # -------------------------------------
		# print("----- Polygons: " +str(polygons)+".")

		ordered_polygons = self.find_connected_polygons(polygons)

		for poly in ordered_polygons:
			points = [
					(
					(point[0]+minx)*costmap.info.resolution+costmap.info.origin.position.x,
					(point[1]+miny)*costmap.info.resolution+costmap.info.origin.position.y
					) for point in poly]
		#	print("11: ") # -------------------------------------
			#self.get_logger().info('polygon index: ' + str(ordered_polygons.index(poly)) + '.')
			self.drive_polygon(Polygon(points))
		#self.get_logger().info("12: ") # -------------------------------------
		self.get_logger().info("Boustrophedon Decomposition completed...")







	def euler_to_quaternion(self, yaw, pitch, roll):
		qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
		qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
		qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
		qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
		return [qx, qy, qz, qw]











	def _getPathImpl(self, start, goal, planner_id='', use_start=False):

		# self.sub_node.get_logger().debug("-------------------- 'ComputePathToPose' --------------------")
		while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=1.0):
			self.sub_node.get_logger().info("'ComputePathToPose' action server not available, waiting...")

		self.goal_msg.start = start
		self.goal_msg.goal = goal
		self.goal_msg.planner_id = planner_id
		self.goal_msg.use_start = use_start

		#self.sub_node.get_logger().info('Getting path...1')
		send_goal_future = self.compute_path_to_pose_client.send_goal_async(self.goal_msg)
		#self.sub_node.get_logger().info('Getting path...2')
		rclpy.spin_until_future_complete(self.sub_node, send_goal_future)
		#self.sub_node.get_logger().info('Getting path...3')
		self.goal_handle = send_goal_future.result()
		#self.sub_node.get_logger().info('Getting path...4')

		if not self.goal_handle.accepted:
			self.get_logger().error('Get path was rejected!')
			return None
		#self.sub_node.get_logger().info('Getting path...5')

		self.result_future = self.goal_handle.get_result_async()
		#self.sub_node.get_logger().info('Getting path...6')
		rclpy.spin_until_future_complete(self.sub_node, self.result_future)
		#self.sub_node.get_logger().info('Getting path...7')
		self.status = self.result_future.result().status
		#self.sub_node.get_logger().info('Getting path...8')
		if self.status != GoalStatus.STATUS_SUCCEEDED:
			self.sub_node.get_logger().warn(f'Getting path failed with status code: {self.status}')
			return None
		self.sub_node.get_logger().info("Get path completed...") 
		return self.result_future.result().result









	def getPath(self, start, goal, planner_id='', use_start=False):
		"""Send a `ComputePathToPose` action request."""
		rtn = self._getPathImpl(start, goal, planner_id='', use_start=False)
		if not rtn:
			return None
		else:
			return rtn.path








	def get_closest_possible_goal(self, pos_last, pos_next, angle, tolerance):

		'''  
		plan.pose = geometry_msgs.msg.PoseStamped(
			header=std_msgs.msg.Header(
				stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), 
				frame_id=''), 
			pose=geometry_msgs.msg.Pose(
				position=geometry_msgs.msg.Point(x=6.100000272691251, y=-4.599999736249448, z=0.0), 
				orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)))	
		'''

		angle_quat = self.euler_to_quaternion(angle,0,0)  # tf.transformations.quaternion_from_euler(0, 0, angle)

		start = PoseStamped()
		start.header.frame_id = self.global_frame # 'map'
		start.header.stamp = self.get_clock().now().to_msg()
		start.pose.position.x = pos_last[0]
		start.pose.position.y = pos_last[1]
		start.pose.orientation.x = angle_quat[0]
		start.pose.orientation.y = angle_quat[1]
		start.pose.orientation.z = angle_quat[2]
		start.pose.orientation.w = angle_quat[3]

		goal = PoseStamped()
		goal.header.frame_id = self.global_frame # 'map'
		goal.header.stamp = self.get_clock().now().to_msg()
		goal.pose.position.x = pos_next[0]
		goal.pose.position.y = pos_next[1]
		goal.pose.orientation.x = angle_quat[0]
		goal.pose.orientation.y = angle_quat[1]
		goal.pose.orientation.z = angle_quat[2]
		goal.pose.orientation.w = angle_quat[3]

		# sanity check a valid path exists
		plan = self.getPath(start, goal) # plan = self.move_base_plan(start, goal, tolerance).plan

		if plan == None:
			return None
		
		if len(plan.poses) == 0:
			return None
		
		#pdb.set_trace()
		closest = None

		for pose in plan.poses:
			pose.header.stamp = self.get_clock().now().to_msg() # rospy.Time(0) # time for lookup does not need to be exact since we are stopped
			pose.header.frame_id = self.global_frame 

			to_frame_rel = pose
			from_frame_rel = self.local_costmap.header.frame_id

			try:
		#		self.get_logger().info(f'plan...1 {from_frame_rel}')
				local_pose_transform = self.tfBuffer.lookup_transform(self.global_frame, from_frame_rel, rclpy.time.Time()) 	
				local_pose = self.transform_pose(to_frame_rel, local_pose_transform)
		#		print("plan...2", local_pose)

			except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as ex:
				# self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
				self.get_logger().info('plan...3')
				pass 

		#	self.get_logger().info('plan...4')
			cellx = round((local_pose.pose.position.x - self.local_costmap.info.origin.position.x) / self.local_costmap.info.resolution)
			celly = round((local_pose.pose.position.y - self.local_costmap.info.origin.position.y) / self.local_costmap.info.resolution)
		#	self.get_logger().info('plan...5')
			cellidx = int(celly*self.local_costmap.info.width+cellx)
		#	self.get_logger().info('plan...6')
			if cellidx < 0 or cellidx >= len(self.local_costmap.data):
				self.get_logger().warn("get_closest_possible_goal landed outside costmap, returning original goal.")
				return pos_next
			cost = self.local_costmap.data[cellidx]
		#	self.get_logger().info('plan...7')
			if (cost >= INSCRIBED_INFLATED_OBSTACLE):
		#		self.get_logger().info('plan...8')
				break
		#	self.get_logger().info('plan...9')
			closest = pose
		#	self.get_logger().info(f'plan...10 closest: {closest}')
		self.get_logger().info("get_closest_possible_goal completed...") 
		return (closest.pose.position.x, closest.pose.position.y)



	




	



	def transform_pose(self, pose, transform):
		# self.get_logger().info('transform_pose...0')
		# Extract translation and rotation from transform
		translation = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
		# self.get_logger().info('transform_pose...1')
		rotation = Rotation.from_quat([transform.transform.rotation.x, transform.transform.rotation.y,
                                    transform.transform.rotation.z, transform.transform.rotation.w])
		# self.get_logger().info('transform_pose...2')
		# Extract position and orientation from input pose
		position = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
		orientation = Rotation.from_quat([pose.pose.orientation.x, pose.pose.orientation.y,
												pose.pose.orientation.z, pose.pose.orientation.w])
		#self.get_logger().info('transform_pose...3')
		# Apply translation and rotation	
		transformed_position = rotation.apply(position) + translation # rotation * position + translation
		transformed_orientation = rotation * orientation
		#self.get_logger().info('transform_pose...4')
		# Create and return transformed pose
		transformed_pose = PoseStamped()
		#transformed_pose.header = transform.header.child_id
		transformed_pose.pose.position.x = transformed_position[0]
		transformed_pose.pose.position.y = transformed_position[1]
		transformed_pose.pose.position.z = transformed_position[2]
		transformed_pose.pose.orientation.w = transformed_orientation.as_quat()[0]
		transformed_pose.pose.orientation.x = transformed_orientation.as_quat()[1]
		transformed_pose.pose.orientation.y = transformed_orientation.as_quat()[2]
		transformed_pose.pose.orientation.z = transformed_orientation.as_quat()[3]

		return transformed_pose








	def drive_path(self, path):
		self.get_logger().info("o_o 1: ") # -------------------------------------
		self.visualize_path(path)

		self.get_logger().info("o_o 2: ") # -------------------------------------

		try:
			initial_pos = self.tfBuffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time()) 
		except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as ex:
			pass 

		#self.get_logger().info("o_o 3: ") # -------------------------------------
		if self.x == None and self.y == None:
			path.insert(0, (initial_pos.transform.translation.x, initial_pos.transform.translation.y))
		else:
			path.insert(0, (self.x, self.y))

		# self.get_logger().info("o_o -==----: ")
		# self.get_logger().info("o_o 4: "+ str(path)) # -------------------------------------

		for pos_last,pos_next in pairwise(path):
			
			if not rclpy.ok:
				return
			

			# self.get_logger().info("o_o 5: ") # -------------------------------------
			pos_diff = np.array(pos_next)-np.array(pos_last)

		#	self.get_logger().info("o_o 6: ") # -------------------------------------
			# angle from last to current position
			angle = atan2(pos_diff[1], pos_diff[0])

			# self.get_logger().info("o_o 7: ") # -------------------------------------

			if abs(pos_diff[0]) < self.local_costmap_width/2.0 and abs(pos_diff[1]) < self.local_costmap_height/2.0:
				# goal is visible in local costmap, check path is clear
		#		self.get_logger().info("o_o 8: ") # -------------------------------------
				tolerance = min(pos_diff[0], pos_diff[1])
				closest = self.get_closest_possible_goal(pos_last, pos_next, angle, tolerance)
		#		self.get_logger().info("o_o 9: ") # -------------------------------------
				if closest is None:
					continue
				pos_next = closest

			# self.get_logger().info("o_o 10: ") # -------------------------------------
			self.write_pose(pos_last[0], pos_last[1], angle) # rotate in direction of next goal

			# for i in range(self.count):
			# 	i += 1
			# 	self.cmd_pub.publish(self.cmd)
			# 	self.looprate.sleep()
			
			self.get_logger().info('EDGE1: x: ' + str(pos_last[0]) + ', y: ' + str(pos_last[1]) + ', th: '+ str(angle) + '.')
			time.sleep(0.7)
			self.get_logger().info('EDGE2 x: ' + str(pos_next[0]) + ', y: ' + str(pos_next[1]) + ', th: '+ str(angle) + '.')
		#	self.get_logger().info("o_o 11: ") # -------------------------------------
			self.write_pose(pos_next[0], pos_next[1], angle)
			time.sleep(0.7)
		
		# self.get_logger().info("o_o 9: ")
		self.visualize_path(path, False)
		self.get_logger().info("drive path completed...") # -------------------------------------

	








	def write_pose(self, x, y, angle):
		#self.get_logger().info("Moving to (%f, %f, %.0f)" % (x, y, angle*180/pi))

		angle_quat = self.euler_to_quaternion(angle,0,0)

		# Append the values to the data dictionary with the index as the key
		index = len(self.pose_output) + 1
		self.pose_output[index] = {
                                "position":
                                {
                                    "x": x,
                                    "y": y,
                                    "z": 0.0
                                },
                                "orientation":
                                {
                                    "w": angle_quat[3],
                                    "x": angle_quat[0],
                                    "y": angle_quat[1],
                                    "z": angle_quat[2]
                                },
                            } 
		self.x = x
		self.y = y









	def drive_polygon(self, polygon):
		#self.get_logger().info("x_x 1: ") # -------------------------------------
		self.visualize_cell(polygon.exterior.coords[:])
		#self.get_logger().info("x_x 2: ") # -------------------------------------

		# Align longest side of the polygon to the horizontal axis
		angle = get_angle_of_longest_side_to_horizontal(polygon)
		#self.get_logger().info("x_x 3: ") # -------------------------------------
		if angle == None:
			self.get_logger().warn("Can not return polygon")
			return
		
		#self.get_logger().info("x_x 4: ") # -------------------------------------
		angle+=pi/2 # up/down instead of left/right
		poly_rotated = rotate_polygon(polygon, angle)
		#self.get_logger().info("x_x 5: ") # -------------------------------------

		self.get_logger().debug("Rotated polygon by %.0f: %s" % (angle*180/pi, str(poly_rotated.exterior.coords[:])))

		if self.border_drive:
		#	self.get_logger().info("x_x 6: ") # -------------------------------------
			path_rotated = border_calc_path(poly_rotated, self.robot_width)
		#	self.get_logger().info("x_x 7: ") # -------------------------------------
			path = rotate_points(path_rotated, -angle)
		#	self.get_logger().info("x_x 8: ") # -------------------------------------
			self.drive_path(path)
		#	self.get_logger().info("x_x 9: ") # -------------------------------------


		# run
		# self.get_logger().info("x_x 10: ") # -------------------------------------
		path_rotated = trapezoid_calc_path(poly_rotated, self.robot_width)
		# self.get_logger().info("x_x 11: ") # -------------------------------------
		path = rotate_points(path_rotated, -angle)
		# self.get_logger().info("x_x 12: ") # -------------------------------------
		self.drive_path(path)
		# self.get_logger().info("x_x 13: ") # -------------------------------------

		# cleanup
		self.visualize_cell(polygon.exterior.coords[:], False)
		#self.get_logger().info("x_x 14: ") # -------------------------------------
		self.get_logger().debug("Polygon done")
		





	def private_shutdown(self):
		self.visualization_cleanup()
		"""Cancel pending task request and kill sub_node."""
		print('Canceling current sub_node task i.e. if any.')
		if self.result_future:
			future = self.goal_handle.cancel_goal_async()
			rclpy.spin_until_future_complete(self.sub_node, future)
		self.sub_node.destroy_node()
		return







def main(args=None):
	rclpy.init(args=args)
	p = MapDrive()
	try:
		rclpy.spin(p)
	except:
		p.private_shutdown()
		p.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
    main()









































# sudo sh free_up_space.sh
# sudo apt-get clean
# sudo apt-get autoclean
# sudo apt-get autoremove
# sudo journalctl --vacuum-time=0.1d
# rm -rf ~/.cache/thumbnails/*







