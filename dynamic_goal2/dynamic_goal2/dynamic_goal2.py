import math
import numpy as np

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from tf_transformations import quaternion_from_euler, euler_from_quaternion

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from dynamic_goal2_action.action import DynamicGoal
from nav2_msgs.action import NavigateToPose

class Memory:
  last_point = None
  last_yaw = None
  first_time = True

class DynamicGoal2(Node):
  def __init__(self):
    super().__init__("dynamic_goal2")
    self._tf_buffer = Buffer()
    self._tf_listener = TransformListener(self._tf_buffer, self)

    # Parameters
    self.declare_parameter("origin_frame", "map")
    self.declare_parameter("robot_frame", "base_footprint")
    self.declare_parameter("rviz_visualization", True)
    self.declare_parameter("costmap_width", 191)
    self.declare_parameter("costmap_height", 177)
    self.declare_parameter("granularity", 0.1)
    self.declare_parameter("movement_threshold", 0.50)
    self.declare_parameter("number_points", 50)
    self.declare_parameter("occupancy_path", 90.0)
    self.declare_parameter("radius", 0.50)
    self.declare_parameter("rate", 0.50)
    self.declare_parameter("rotational_threshold", 0.90)

    self._origin_frame = self.get_parameter("origin_frame").get_parameter_value().string_value
    self._robot_frame = self.get_parameter("robot_frame").get_parameter_value().string_value
    self._rviz_visualization = self.get_parameter("rviz_visualization").get_parameter_value().bool_value
    self._costmap_width = self.get_parameter("costmap_width").get_parameter_value().integer_value
    self._costmap_height = self.get_parameter("costmap_height").get_parameter_value().integer_value
    self._granularity = self.get_parameter("granularity").get_parameter_value().double_value
    self._movement_threshold = self.get_parameter("movement_threshold").get_parameter_value().double_value
    self._number_points = self.get_parameter("number_points").get_parameter_value().integer_value
    self._occupancy_path = self.get_parameter("occupancy_path").get_parameter_value().double_value
    self._radius = self.get_parameter("radius").get_parameter_value().double_value
    self._rate = self.get_parameter("rate").get_parameter_value().double_value
    self._rotational_threshold = self.get_parameter("rotational_threshold").get_parameter_value().double_value

    self._goal = ""
    self._goal_handle = None
    self._navigation_goal_future = None
    self._get_navigation_result_future = None
    self._navigation_goal_finnished = True

    self._memory = Memory()
    self._rate = self.create_rate(self._rate)

    self._map = np.zeros((self._costmap_width, self._costmap_height))
    self._map_info = None
    self._full_circle = 0.8 # TODO

    # Actions
    self._dynamic_goal_server = ActionServer(self, DynamicGoal, "/dynamic_goal2", self._dynamic_goal_execute_callback)
    self._navigation_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
    # self._navigation_client.wait_for_server() # TODO

    # Publishers
    self._marker_publisher_pub = self.create_publisher(MarkerArray, "/visualization_marker_array", 10)

    # Subscribers
    self._costmap_2D_sub = self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._costmap_callback, 10)

  def choose_navigation_goal(self, robot_position, target_position):
    circle = []
    radius = self._radius
    extra_distance = 0.0
    goal_found = False
    while not goal_found:
      radius += extra_distance * self._granularity
      number_points = int(radius * self._number_points)

      for i in range(0, number_points):
        point = Point()
        point.x = round(math.cos(2 * math.pi / number_points * i) * radius, 2) + target_position.x
        point.y = round(math.sin(2 * math.pi / number_points * i) * radius, 2) + target_position.y
        point.z = 0.0
        circle.append(point)

      circle.sort(key=lambda point: self.distance_to_robot(point, robot_position))
      if self._rviz_visualization:
        self.show_spheres_rviz(circle)

      while len(circle) != 0:
        if not self.is_cell_available(circle[0], 0.0):
          del circle[0]

        else:
          if not self.is_path_to_target_available(circle[0], target_position):
            del circle[0]

          else:
            break

      if len(circle) > 0:
        goal_found = True

      else:
        extra_distance += 0.1

    q = self.get_quaternion(circle[0], target_position)
    return circle[0], q

  def get_quaternion(self, goal_position, target_position):
    dy = target_position.y - goal_position.y
    dx = target_position.x - goal_position.x
    yaw = math.atan2(dy, dx)
    q = quaternion_from_euler(0, 0, yaw)
    return q

  def is_cell_available(self, point, occupancy):
    [x, y] = self.map_to_index(point.x, point.y)
    cost = self._map[int(round(x)), int(round(y))]
    if cost <= occupancy:
      return True
    else:
      return False
    
  def is_path_to_target_available(self, origin, target):
    dx = target.x - origin.x
    dy = target.y - origin.y
    if dx != 0:
      m = dy / dx
    else:
      return False # FIXME
    
    b = target.y - m * target.x

    number_points = int(round(20 * self._radius))
    dx = float(dx)
    interval = dx / number_points

    line = []
    for i in range(1, int(round(number_points / 2))):
      point = Point()
      point.x = origin.x + interval * i
      point.y = m * point.x + b
      point.z = 0.0
      
      line.append(point)
      if not self.is_cell_available(point=point, occupancy=self._occupancy_path):
        return False
    
    if self._rviz_visualization:
      self.show_spheres_rviz(line)

    return True

  def map_to_index(self, ix, iy):
    origin_x = self._map_info.origin.position.x
    origin_y = self._map_info.origin.position.y
    resolution = self._map_info.resolution

    mx = (ix - origin_x) / resolution
    my = (iy - origin_y) / resolution
    return [mx , my]

  def show_spheres_rviz(self, points):
    marker_array = MarkerArray()
    for i, point in enumerate(points):
      marker = Marker()
      marker.header.frame_id = self._origin_frame
      marker.header.stamp = self.get_clock().now().to_msg()
      marker.id = i
      marker.type = Marker.SPHERE
      marker.action = Marker.ADD
      marker.pose = Pose(
        position=Point(x=point.x, y=point.y, z=point.z),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
      )

      marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
      marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
      marker.lifetime = Duration(sec=600)
      marker_array.markers.append(marker)

    self._marker_publisher_pub.publish(marker_array)

  def distance_2D(self, x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

  def distance_to_robot(self, point, robot_position):
    return self.distance_2D(point.x, point.y, robot_position.x, robot_position.y)
  
  def has_target_move(self, point, yaw):
    distance = self.distance_2D(point.x, point.y, self._memory.last_point.x, self._memory.last_point.y)
    yaw_distance = abs(yaw - self._memory.last_yaw)
    if distance > self._movement_threshold or yaw_distance > self._rotational_threshold:
      return True
    else:
      return False
    
  # Callbacks

  def _costmap_callback(self, msg):
    self._map_info = msg.info
    for i in range(0, self._costmap_height):
      for j in range(0, self._costmap_width):
        self._map[j][i] = msg.data[i * self._map.shape[0] + j]

  # TODO
  def _cancel_navigation_goal_callback(self, future):
    self._goal = ""
    cancel_response = future.result()
    if cancel_response.return_code == 0:
      self.get_logger().warn("Navigation goal was cancelled.")
    else:
      # TODO
      pass

    self.get_logger().info("Dynamic Goal 2 has stopped.")

  def _dynamic_goal_execute_callback(self, goal_handle):
    goal_name = goal_handle.request.goal
    self._navigation_goal_finnished = False
    while not self._navigation_goal_finnished:
      update_goal = True
      try:
        target_transform = self._tf_buffer.lookup_transform(self._origin_frame, goal_name, Time())

      except TransformException as e:
        self.get_logger().warn(f"Could not transform from {self._origin_frame} to {goal_name}: {e}")
        # TODO

        self._rate.sleep()
        continue

      target_point = Point()
      target_point.x = target_transform.transform.translation.x
      target_point.y = target_transform.transform.translation.y
      target_point.z = target_transform.transform.translation.z

      target_rotation = [target_transform.transform.rotation.x, target_transform.transform.rotation.y, target_transform.transform.rotation.z, target_transform.transform.rotation.w]
      target_yaw = euler_from_quaternion(target_rotation)[2]

      if not self._memory.first_time:
        update_goal = self.has_target_move(target_point, target_yaw)

      elif self._memory.first_time:
        self._memory.last_point = target_point
        self._memory.last_yaw = target_yaw
        self._memory.first_time = False
        update_goal = True

      if not update_goal: #FIXME
        self._rate.sleep()
        continue
      
      try:
        robot_transform = self._tf_buffer.lookup_transform(self._origin_frame, self._robot_frame, Time())
      
      except TransformException as e:
        self.get_logger().warn(f"Could not transform from {self._origin_frame} to {self._robot_frame}: {e}")
        # TODO

        self._rate.sleep()
        continue

      robot_point = Point()
      robot_point.x = robot_transform.transform.translation.x
      robot_point.y = robot_transform.transform.translation.y
      robot_point.z = 0.0
      
      goal, q = self.choose_navigation_goal(robot_point, target_point)

      pose = PoseStamped()
      pose.header.frame_id = self._origin_frame
      pose.header.stamp = self.get_clock().now().to_msg()
      pose.pose.position.x = goal.x
      pose.pose.position.y = goal.y
      pose.pose.orientation.x = q[0]
      pose.pose.orientation.y = q[1]
      pose.pose.orientation.z = q[2]
      pose.pose.orientation.w = q[3]

      async_goal = NavigateToPose.Goal()
      async_goal.pose = pose
      self._navigation_goal_future = self._navigation_client.send_goal_async(async_goal)
      self._navigation_goal_future.add_done_callback(self._navigation_goal_response_callback)
      self._rate.sleep()
    
    goal_handle.succeed()
    result = DynamicGoal.Result()
    result.success = True
    result.message = ""
    return result

  def _get_navigation_result_callback(self, future):
    result = future.result() 
    if result.status == 4:
      self.get_logger().info("Navigation goal was achieved successfully.")
      self._navigation_goal_finnished = True

    # TODO
    elif result.status == 6:
      # When you send a new navigation goal and the previous one has not finnished, the previous is terminated.
      pass
      
  def _navigation_goal_response_callback(self, future):
    self._goal_handle = future.result()
    if not self._goal_handle.accepted:
      self.get_logger().warn("Navigation goal rejected.")
      self._goal = ""
      # TODO
      return

    self.get_logger().info("Navigation goal accepted.")
    self._get_navigation_result_future = self._goal_handle.get_result_async()
    self._get_navigation_result_future.add_done_callback(self._get_navigation_result_callback)

def main(args=None):
  rclpy.init(args=args)
  node = DynamicGoal2()
  executor = MultiThreadedExecutor()
  executor.add_node(node)
  executor.spin()
  executor.shutdown()

  node.destroy_node()
  rclpy.shutdown()