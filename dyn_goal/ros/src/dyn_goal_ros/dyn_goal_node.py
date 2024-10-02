#!/usr/bin/env python
import rospy
import math
import numpy as np
import tf
import tf2_ros
import geometry_msgs.msg as geometry_msgs
import yaml
import threading
from time import time, sleep

from rospy import loginfo as log
from geometry_msgs.msg import PoseStamped as pose
from geometry_msgs.msg import PoseStamped
from dyn_goal.msg import dyn_goal_msg
from move_base_msgs.msg import MoveBaseActionResult
from trajectory_msgs.msg import JointTrajectory
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Empty

# for rviz visualization
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

from actions_tiago_ros.tiago_api import TiagoAPI
tiago_api_factory = TiagoAPI

#memory variables (trans is the last known poi position; rot last known poi orientation; control: if it is the first time)
class Memory:
	trans = geometry_msgs.Point()
	yaw = 0.0  # rad (2D robot movement)
	control = False
	time = 0.0

class Control:
	activated = False
	goal = "tracked_person"
	origin = "base_link"
	dist = 0.0
	stop_on_arrival = False

class DynGoal(object):
	def __init__(self):
		try:
			# need to download a .yaml config with all the needed parameters !
			rospy.loginfo("Loading node config")
			config_path = rospy.myargv()[1]
			config = yaml.safe_load(open(config_path))
		except IndexError:
			rospy.logerr("Could not open the configuration file. Please make sure this file exists and is properly called in the launch file!")
			return

		# get node params from config
		debug 						= config["debug"]
		self.mocup					= config["mocup"]
		self.origin_frame 			= config["origin_frame"]
		node_name					= config["node_name"]
		rate						= config["rate"]
		self.update_head_control	= config["update_head"]
		self.activate_ppl_flw		= config["activate_ppl_detection"]
		self.update_on_rotation		= config["update_on_rotation"]
		self.number_of_points		= config["number_of_points"]
		self.granularity 			= config["granularity"]
		self.allow_backwards		= config["allow_backwards"]
		#Thresholds
		self.movementThreshold 		= config["movement_threshold"]
		self.rotationThreshold 		= config["rotational_threshold"]
		self.availabilityThreshold 	= config["availability_threshold"]
		self.convert_offset 		= config["convert_offset"]
		self.full_circle			= config["full_circle"]
		self.time_without_receiving_tf = rospy.Duration(config["time_without_receiving_tf"])
		self.time_threshold			= 2.0

		# initializes the node (if debug, initializes in debug mode)
		if debug == True:
			rospy.init_node(node_name, anonymous=False, log_level=rospy.DEBUG)
			rospy.loginfo("%s node created [DEBUG MODE]" % node_name)
		else:
			rospy.init_node(node_name, anonymous=False)
			rospy.loginfo("%s node created" % node_name)

		self.listener = tf.TransformListener()
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		#Topic to publish the pose you want to reach
		self.pose_publisher = rospy.Publisher('/move_base_simple/goal', pose, queue_size=1)

		#Topic to publish the pose you want to reach
		#self.head_publisher = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=2)

		#Topic to draw in rviz
		self.marker_publisher = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=5)

		#Topic of status
		self.status_pub = rospy.Publisher('dyn_goal_status', String, queue_size=5)

		#To track
		self.__following_event_pub = rospy.Publisher('/people_follower/follower/event_in', String, queue_size = 5)
		self.__following_event_pub = rospy.Subscriber('/people_follower/follower/event_in', String, self.__callback_event_following)

		#Subscribe to the topic that controls the dynamic goal operation
		self.sub_control = rospy.Subscriber("move_base_simple/dyn_goal", dyn_goal_msg , self.controlCallback)

		#Subscribe to the topic of the static goal
		self.sub_static_goal = rospy.Subscriber("/move_base_simple/goal",PoseStamped, self.staticCallback)

		# Subscribe to the goal result topic
		self.__result_subscriber = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.__result_cb)

		#Subscribe to Costmap
		self.sub_costmap_2d = rospy.Subscriber("move_base/global_costmap/costmap",OccupancyGrid, self.costmapCallback)

		#service to clean costmaps
		self.clear_srv = rospy.ServiceProxy('move_base/clear_costmaps', Empty)

		# Configurations
		self.head_rest_time = 0.1

		#initializations
		self.__event_out_following = None
		self.memory = Memory()
		self.control = Control()
		self.map_ = None
		self.map_info = None
		self.updatingGoal = False
		self.send_goal_to_move_base_thread = None
		self.update_head_thread = None
		self.result_navigation = None
		self.requested_follow = True
		self.rate = rospy.Rate(rate)
		self.tiago_api = tiago_api_factory()

	def destroySubscribers(self):
		# rospy.set_param('/move_base/DWAPlannerROS/min_vel_x',-0.6)
		# rospy.set_param('/move_base/DWAPlannerROS/max_vel_x',0.6)
		self.sub_control.unregister()
		self.sub_costmap_2d.unregister()
		self.__result_subscriber.unregister()
		log("Hope you enjoyed our service! Come back any time!")

	def asyncPublish(self, pub, data):
		for i in range(1,3):
			#If there is a new goal stop the code from sending again and leaves the thread
			if self.stop_thread_goal_pub:
				rospy.logdebug("Stop sending goal: " + str(data.pose.position.x) + "," + str(data.pose.position.y))
				break

			data.header.seq = i
			rospy.logdebug("Pose: " + str(data.pose.position.x) + "," + str(data.pose.position.y) + "  trial: " + str(data.header.seq))
			self.updatingGoal = True
			pub.publish(data)
			rospy.sleep(1)


	def run(self):

		while not rospy.is_shutdown():
			if self.control.activated:
				if self.requested_follow:
					# reset the navigation result
					if self.control.stop_on_arrival:
						self.result_navigation = None
					self.requested_follow = False
				#set velocity to only front and a little slower
				#if not self.memory.control:
					# rospy.set_param('/move_base/DWAPlannerROS/min_vel_x',0.0)
					# rospy.set_param('/move_base/DWAPlannerROS/max_vel_x',0.5)
				#obtain goal tf location in relation to origin tf (default is goal=tracked_person and origin=base_link)
				time_now = rospy.Time.now()
				try:
					transform = self.tf_buffer.lookup_transform(self.control.origin, self.control.goal, rospy.Time(0), rospy.Duration(1.0))
					# (trans,rot) = self.listener.lookupTransform(self.control.origin, self.control.goal, rospy.Time(0))
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					continue

				# check the time of transform to know if it was published recently or not
				if time_now-transform.header.stamp > rospy.Duration(2.0):
					self.tiago_api.navigation.stop_movement()
					if time_now-transform.header.stamp > self.time_without_receiving_tf:
						self.control.activated = False
						self.result_navigation = None
						if self.update_head_control and self.update_head_thread is not None:
							self.stop_thread_head = True
							self.update_head_thread.join()
						self.status_pub.publish(String(data="e_lost_person"))
						continue
					self.tiago_api.hri.say_and_wait(text="I seemed to have lost you. Can you please step back in front of me.")
					rospy.sleep(5)
					continue

				trans = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
				rot = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]


				target_position = self.vectorToPoint(trans)
				quaternion = (rot[0], rot[1], rot[2], rot[3])
				target_yaw = tf.transformations.euler_from_quaternion(quaternion)[2]

				#get pose of the robot
				try:
					(self.trans_robot , self.rot_robot) = self.listener.lookupTransform("/map", "/base_link", rospy.Time(0))
				except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					rospy.logerr("Couldn't get robot's pose")
					return

				trans_robot = self.vectorToPoint(self.trans_robot)
				if ((trans[0]-self.trans_robot[0])**2 + (trans[0]-self.trans_robot[0])**2 > 8**2) and not self.control.stop_on_arrival:
					self.tiago_api.navigation.stop_movement()
					self.tiago_api.hri.say_and_wait(text="I seemed to have lost you. Can you please step back in front of me.")
					rospy.sleep(5)


				#to follow the first time, memory_control is set to 0, and then 1 so it only enters the refresh goal pose
				#cycle if the pose of the tf is different
				elif self.hasTargetMoved(target_position, target_yaw) or not self.memory.control:
					#clear the costmaps
					#if not self.mocup:
						#self.clear_srv.call()

					print("Target moved!")
					new_pose = pose()
					new_pose.header.stamp = rospy.Time.now()
					#new_pose.header.seq = 2
					new_pose.header.frame_id = self.origin_frame		#TODO: check if this is right

					# If the robot is closer to the distance than it should, stay put
					self.robot_position = trans_robot
					if self.allow_backwards or self.distanceToRobot(target_position) > self.control.dist:
						#Change target_position to be:
						final_position = self.chooseGoal(target_position, trans_robot)
					else:
						final_position = trans_robot

					if target_position != final_position:
						final_orientation = self.getAngle(final_position,target_position)
					else:
						final_orientation = rot
					new_pose.pose.position.x = final_position.x
					new_pose.pose.position.y = final_position.y
					new_pose.pose.position.z = final_position.z
					new_pose.pose.orientation.x = final_orientation[0]
					new_pose.pose.orientation.y = final_orientation[1]
					new_pose.pose.orientation.z = final_orientation[2]
					new_pose.pose.orientation.w = final_orientation[3]

					#Kill previous thread if it exists
					if self.send_goal_to_move_base_thread is not None:
						self.stop_thread_goal_pub = True
						self.send_goal_to_move_base_thread.join()

					#Initialize the thread to send the goal to move_base
					self.stop_thread_goal_pub = False
					self.send_goal_to_move_base_thread = threading.Thread(target=self.asyncPublish, args=(self.pose_publisher, new_pose))
					self.send_goal_to_move_base_thread.start()

					#add threshold in differences (maybe in a function and call in the if maybe)
					#saves in memory the last pose sent so that it sends only different poses, and not copies
					self.memory.trans = target_position
					self.memory.yaw = target_yaw
					self.memory.control = True
					self.memory.time = rospy.Time.now().secs
				else:
					self.memory.control = True

				# If the robot should stop when reaches a goal, sets activated to false
				if self.control.stop_on_arrival and self.result_navigation == 3:
					rospy.logdebug("Stopping dynamic approach")
					self.status_pub.publish(String(data="e_success"))
					self.control.activated = False
					self.result_navigation = None
					if self.update_head_control and self.update_head_thread is not None:
						self.stop_thread_head = True
						self.update_head_thread.join()

				# if self.update_head_control: #THIS WAS HERE BEFORE BUT I MADE IT ASYNC SO IT ALWAYS FOLLOWS PERSON, NO MATTER HOE LONG IT TAKES FOR IT TO COMPUTE THE POSE
				# 	self.updateHeadOrientation()

			elif self.memory.control:
				#reset the control variable and the speed params
				self.memory.control = False
				# rospy.set_param('/move_base/DWAPlannerROS/min_vel_x',-0.6)
				# rospy.set_param('/move_base/DWAPlannerROS/max_vel_x',0.6)

			#When it is off make it sleep until a new message is sent to the control topic, otherwise cycle
			self.rate.sleep()

	#Function to detect if the target movement is relevant to resend as goal
	def hasTargetMoved(self,trans, yaw):
		if rospy.Time.now().secs - self.memory.time > self.time_threshold:
			dist = self.dist2D(self.memory.trans.x , self.memory.trans.y , trans.x , trans.y)
			dist_yaw = abs(yaw - self.memory.yaw)
			if dist > self.movementThreshold or (dist_yaw > self.rotationThreshold and self.update_on_rotation):
				return True
			else:
				return False
		else:
			return False


	#Function that detects if the goal is aleady closer than the threshold
	def isGoalClose(self,trans):
		dist = self.dist2D(self.memory.trans.x , self.memory.trans.y , trans.x , trans.y)
		if dist < self.control.dist:
			rospy.logwarn("Robot is closer than it should to the target goal!")
			return True
		else:
			return False

	#Function used to sort the points on the circle around the target goal by distance to the robot
	def distanceToRobot(self,point):
		return self.dist2D(point.x, point.y, self.robot_position.x, self.robot_position.y)

	#Function that determines if a certain point on the map is available to be set as goal
	def isCellAvailable(self, point, occupancy):
		#get the costmap by subscribing to nav_msgs/OccupancyGrid.msg
		if self.mocup:
			return True
		else:
			try:
				[x,y] = self.worldToMap(point.x,point.y)
				cost = self.map_[int(round(x))][int(round(y))]

				if cost <= occupancy:
					return True
				else:
					return False
			except Exception as e:
				rospy.logwarn('Cannot check point: [' + str(point.x) + ';' + str(point.y) + ']')
				return False

	#Several points in a straight line will be computed and checked to determine if there is a free straight path to the target
	def isPathToTargetAvailable(self, origin, target):
		if self.mocup:
			return True
		else:
			result = True
			#equation of the line y=mx+b
			dx = target.x - origin.x
			dy = target.y - origin.y
			if dx != 0:
				m = dy/dx
			else:
				return False #fix this, maybe by making an equation x=my+b
			b = target.y - m*target.x

			#Calculate the interval of x between  2 consecutive points in the straight line
			number_of_points = int(round(20 * self.control.dist))
			dx=float(dx)
			interval = dx/number_of_points

			line = []
			for i in range(1,int(round(number_of_points/2))):
				point = Point()
				point.x = origin.x + interval*i
				point.y = m * point.x + b
				point.z = 0
				line.append(point)
				if not self.isCellAvailable(point=point,occupancy=self.availabilityThreshold):
					result = False
			# self.show_spheres_in_rviz(line)

			return result

	def chooseGoal(self,target_position, robot_position):
		#radius of the circle
		r = self.control.dist

		self.robot_position = robot_position

		robot_position_for_angle = np.array([robot_position.x,robot_position.y])
		target_position_for_angle = np.array([target_position.x,target_position.y])

		#make a circle of positions, check which ones are closer to the robot, and if it is reachable
		circle = []

		if r > 0:
			found_proper_goal = False
			extra_distance = 0
			while not found_proper_goal:
				r += extra_distance * self.granularity
				#number of points to check in the circumference
				number_points = int(r * self.number_of_points)

				for i in range(0,number_points):
					point = Point()
					point.x = round(math.cos(2*math.pi/number_points*i)*r,2) + target_position.x
					point.y = round(math.sin(2*math.pi/number_points*i)*r,2) + target_position.y
					point.z = 0

					#calculate if the point should be added
					point_position_for_angle = np.array([point.x,point.y])

					#calculate if the point is behind the person
					ba = point_position_for_angle - target_position_for_angle
					bc = robot_position_for_angle - target_position_for_angle

					cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
					angle = np.degrees(np.arccos(cosine_angle))

					#only append if the angle is within the desired percentage of the circle (2 degrees of tolerances)
					if abs(angle) < (180 * self.full_circle) + 2:
						circle.append(point)

				#order the circle variable by proximity to the ROBOT
				circle.sort(key=self.distanceToRobot)
				self.show_spheres_in_rviz(circle)

				#iterate the ordered set check if it is a free cell with a clear path to the goal
				while not rospy.is_shutdown() and len(circle) != 0:
					if not self.isCellAvailable(circle[0],occupancy=0):
						del circle[0]
					else:
						if not self.isPathToTargetAvailable(circle[0],target_position):
							del circle[0]
						else:
							break
				#Checks if any desired point was left in the circle that could be used as goal
				if len(circle) > 0 or extra_distance > 50:
					rospy.logdebug("Extra distance added to the goal was: " + str(extra_distance * self.granularity) + " meters")
					found_proper_goal = True
				else:
					#If no point at the desired distance can be chosen, the distance is slowly increased until a point can be chosen
					extra_distance += 1

		else:
			#In the case the distance to the target is 0, our goal is the target itself
			circle.append(target_position)

		#return goal, that is the closest point to the robot that was not removed from the list by the previous conditions
		if len(circle) > 0:
			return circle[0]
		else:
			#backup case when the circle has no points left and to prevent the crashing of the code, the robot goes towards the person
			return target_position

	def dist2D(self,x1,y1,x2,y2):
		return math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))

	def vectorToPoint(self,vector):
		point = geometry_msgs.Point()
		point.x = vector[0]
		point.y = vector[1]
		point.z = vector[2]
		return point

	def getAngle(self, origin, target):
		if target.x != origin.x:
			angle_raw = math.atan((target.y- origin.y) / abs(target.x-origin.x))
			#conditions to counter the decrease of the angle
			if target.x-origin.x < 0 and angle_raw < 0:
				angle_refined = - (math.pi + angle_raw)
			elif target.x-origin.x < 0 and angle_raw > 0:
				angle_refined = math.pi - angle_raw
			elif target.y == origin.y and target.x > origin.x:
				angle_refined = 0
			elif target.y == origin.y and target.x < origin.x:
				angle_refined = math.pi
			else:
				angle_refined = angle_raw

			angle = angle_refined
		elif target.y > origin.y:
			angle = math.pi/2
		elif target.y < origin.y:
			angle = -math.pi/2
		else:
			angle = 0

		#condition to solve the case where the calculated angle and the robot rotation are in different multiples of 2*pi
		if angle > math.pi:
			angle = angle - 2 * math.pi
		elif angle < -math.pi:
			angle = angle + 2 * math.pi

		#get the rotation of the body
		return tf.transformations.quaternion_from_euler(0,0,angle)

	#Function that depending where the target is changes the orientation of the head to follow that target
	def updateHeadOrientation(self):

		while not self.stop_thread_head:

			#FOR NOW IT IS FIXED BUT SEE HOW IT CAN BE DYNAMIC
			angle_pitch = 0.0
			#get pose of the head
			try:
				(trans_head,rot_head) = self.listener.lookupTransform("base_link", "head_2_link", rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				rospy.logerr("Couldn't get head pose")
				return

			#get rotation of the robot body (check if it is yaw or pitch)
			quaternion_to_euler_head = tf.transformations.euler_from_quaternion(rot_head)
			yaw_head = quaternion_to_euler_head[2]
			#pitch_head = -quaternion_to_euler_head[1]


			try:
				(trans_poi,rot_poi) = self.listener.lookupTransform("base_link", "tracked_person", rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				rospy.logerr("Couldn't get tracked person pose in relation to base_link")
				return

			if trans_poi[0] != 0: #because division by 0 is impossible
				if trans_poi[0] < 0 and trans_poi[1] > 0:
					angle_yaw = 0.85
				elif trans_poi[0] < 0 and trans_poi[1] <= 0:
					angle_yaw = -0.85
				else:
					angle_yaw = math.atan(trans_poi[1] / abs(trans_poi[0]))

			elif trans_poi[1] < 0: # if the target is in line with the robot, and y > 0, then the person is to the right
				angle_yaw = -0.85
			elif trans_poi[1] > 0: # if the target is in line with the robot, and y < 0, then the person is to the left
				angle_yaw = 0.85
			else:
				angle_yaw = 0

			if abs(angle_yaw - yaw_head) > 0.05:  # tune this threshold
				#rospy.loginfo("Rotating head to: " + str(angle_yaw))
				hri_actions.rotate_head_value(angle_yaw, angle_pitch, wait=False, timeout=5)

			sleep(self.head_rest_time)


	#Function to tranform a map coordinate into a world one
	def mapToWorld(self, mx, my):
		wx = self.map_info.info.origin.position.x + (mx + self.convert_offset) * self.map_info.info.resolution
		wy = self.map_info.info.origin.position.y + (my + self.convert_offset) * self.map_info.info.resolution
		return [wx, wy]

	#Function to tranform a world coordinate into a map one
	def worldToMap(self, wx, wy):
		origin_x = self.map_info.info.origin.position.x
		origin_y = self.map_info.info.origin.position.y
		resolution = self.map_info.info.resolution

		if (wx < origin_x or wy < origin_y):
			return None

		mx = (wx - origin_x) / resolution - self.convert_offset
		my = (wy - origin_y) / resolution - self.convert_offset

		if (mx < self.map_info.info.width and my < self.map_info.info.height):
			return [mx , my]

		return None

	#Callback called when receiving a control instruction
	def controlCallback(self, data):
		self.control.activated = data.activated
		self.control.goal = data.dyn_goal_tf
		self.control.origin = data.origin_tf
		if data.stop_on_arrival is not None:
			self.control.stop_on_arrival = data.stop_on_arrival
		rospy.logerr(self.control.stop_on_arrival)

		if data.dist >= 0:
			self.control.dist = data.dist
		else:
			self.control.dist = 1.2
		if data.activated:
			self.status_pub.publish(String(data="e_alive"))
			if self.activate_ppl_flw:
				rospy.logdebug("Activating socrob people tracking")
				self.track_person(track=True, timeout=6.0)
			if self.update_head_control:
				self.stop_thread_head = False
				self.update_head_thread = threading.Thread(target=self.updateHeadOrientation, args=())
				self.update_head_thread.start()
			rospy.loginfo("Dyn_Goal activated with distance to target of " + str(self.control.dist) + " m")
			self.requested_follow = True
		else:
			rospy.loginfo("Dyn goal was disabled by a control topic message.")
			self.status_pub.publish(String(data="e_stopped"))
			self.tiago_api.navigation.stop_movement()
			if self.activate_ppl_flw:
				rospy.logdebug("Disabling socrob people tracking")
				self.track_person(track=False, timeout=6.0)
			if self.update_head_control and self.update_head_thread is not None:
				self.stop_thread_head = True
				self.update_head_thread.join()

	#Callback called when a static goal is received, it will deactivate the dynamic goal to ensure the mutual exclusivity
	def staticCallback(self, data):
		if self.updatingGoal:
			self.updatingGoal = False	#this means that the message received was sent by dyn_goal and should not deactivate the dyn_goal
		else:
			self.control.activated = False
			rospy.loginfo("Static \"/move_base_simple/goal\" was detected. This static goal automatically disables any currently active Dyn_Goal")

	#Callback for the costmap
	def costmapCallback(self, data):
		self.map_info = data		#TODO: maybe smthg more

		#self.map_ = np.arrange(data.info.width*data.info.height).reshape(data.info.width,data.info.height)
		self.map_ = np.zeros((data.info.width,data.info.height))

		for i in range(0,data.info.height):
			for j in range(0,data.info.width):
				self.map_[j][i] = data.data[i*data.info.width + j]
		self.map_info.data = None

	def __callback_event_following(self, msg):
		self.__event_out_following = msg.data

	def show_spheres_in_rviz(self, points):
		marker_array = []
		for i in range(0,len(points)):
			marker = Marker(
					type=Marker.SPHERE,
					id=i,
					lifetime=rospy.Duration(60),
					pose=Pose(Point(points[i].x, points[i].y, points[i].z), Quaternion(0, 0, 0, 1)),
					scale=Vector3(0.1, 0.1, 0.1),
					header=Header(frame_id='/'+self.origin_frame),
					color=ColorRGBA(0.0, 0.0, 1.0, 0.8))
			marker_array.append(marker)
		self.marker_publisher.publish(marker_array)

	def track_person(self, track, timeout=6.0):
		# check that our publishers have connections
		if self.__following_event_pub.get_num_connections() == 0:
			rospy.logwarn('[mbot class] There is no subscriber for follower event in, I will try anyway but beware!')
			#return False

		self.__event_out_following = None
		pub_str = String('e_track_poi') if track else String('e_stop')
		try:
			self.__following_event_pub.publish(pub_str)
		except Exception as e:
			rospy.logfatal('[mbot class] could not publish event in, very bad!!')
			return False

		if track:

			timestart = time()
			while not rospy.is_shutdown() and time()-timestart < timeout and self.__event_out_following is None:
				rospy.sleep(0.1)

			# timeout or failure? stop all
			if self.__event_out_following is None:
				# self.__following_event_pub.publish(String('e_stop'))
				rospy.logerr("Timeout in requesting to track poi")
				return False

			return self.__event_out_following == 'e_success' or self.__event_out_following == 'e_tracking_acquired'

		else:
			return True

	def __result_cb(self, data):
		#rospy.logwarn("Move status: %s", str(data.status.status))
		self.result_navigation = data.status.status
		rospy.logdebug("Move status: %s", str(self.result_navigation))

def main():
	# create object of the class DynGoal (constructor will get executed!)
	my_object = DynGoal()
	# call run method of class DynGoal
	my_object.run()
	# destroy subscriptions
	my_object.destroySubscribers()

if __name__ == '__main__':
	main()
