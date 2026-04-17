import math
import numpy as np

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from dynamic_goal2_action.action import DynamicGoal


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
    self.declare_parameter("rviz_line_visualization", False)
    self.declare_parameter("rviz_circle_visualization", True)
    self.declare_parameter("granularity", 0.1)
    self.declare_parameter("movement_threshold", 0.50)
    self.declare_parameter("number_points", 50)
    self.declare_parameter("occupancy_path", 90.0)
    self.declare_parameter("radius", 0.50)
    self.declare_parameter("rate", 0.50)
    self.declare_parameter("rotational_threshold", 0.90)

    self._origin_frame = self.get_parameter("origin_frame").get_parameter_value().string_value
    self._robot_frame = self.get_parameter("robot_frame").get_parameter_value().string_value
    self._rviz_line_visualization = self.get_parameter("rviz_line_visualization").get_parameter_value().bool_value
    self._rviz_circle_visualization = self.get_parameter("rviz_circle_visualization").get_parameter_value().bool_value
    self._granularity = self.get_parameter("granularity").get_parameter_value().double_value
    self._movement_threshold = self.get_parameter("movement_threshold").get_parameter_value().double_value
    self._number_points = self.get_parameter("number_points").get_parameter_value().integer_value
    self._occupancy_path = self.get_parameter("occupancy_path").get_parameter_value().double_value
    self._radius = self.get_parameter("radius").get_parameter_value().double_value
    self._rate = self.get_parameter("rate").get_parameter_value().double_value
    self._rotational_threshold = self.get_parameter("rotational_threshold").get_parameter_value().double_value

    self._goal = ""
    self._goal_handle = None
    self._current_navigation_goal = None
    self._navigation_goal_future = None
    self._get_navigation_result_future = None
    self._navigation_goal_finnished = True
    self._pending_cancel = False
    self._dynamic_goal_active = False
    self._warned_no_map = False

    self._memory = Memory()
    self._rate = self.create_rate(self._rate)

    self._map = None
    self._map_info = None
    self._costmap_width = None
    self._costmap_height = None
    self._full_circle = 0.8  # TODO

    # Actions
    self._dynamic_goal_server = ActionServer(
      self,
      DynamicGoal,
      "/dynamic_goal2",
      execute_callback=self._dynamic_goal_execute_callback,
      goal_callback=self._dynamic_goal_goal_callback,
      cancel_callback=self._dynamic_goal_cancel_callback,
    )
    self._navigation_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
    if not self._navigation_client.wait_for_server(timeout_sec=2.0):
      self.get_logger().warn("/navigate_to_pose action server is not available yet.")

    # Publishers
    self._marker_publisher_pub = self.create_publisher(MarkerArray, "/visualization_marker_array", 10)

    # Subscribers
    self._costmap_2D_sub = self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._costmap_callback, 10)

    self.add_on_set_parameters_callback(self._on_param_change)

  def choose_navigation_goal(self, robot_position, target_position):
    radius = self._radius
    extra_distance = 0.0
    goal_found = False

    robot_position_for_angle = np.array([robot_position.x, robot_position.y])
    target_position_for_angle = np.array([target_position.x, target_position.y])

    while not goal_found:
      circle = []
      radius += extra_distance * self._granularity
      number_points = int(radius * self._number_points)
      if number_points <= 0:
        number_points = 1

      for i in range(0, number_points):
        point = Point()
        point.x = round(math.cos(2 * math.pi / number_points * i) * radius, 2) + target_position.x
        point.y = round(math.sin(2 * math.pi / number_points * i) * radius, 2) + target_position.y
        point.z = 0.0

        # calculate if the point should be added
        point_position_for_angle = np.array([point.x, point.y])

        # calculate if the point is behind the person
        ba = point_position_for_angle - target_position_for_angle
        bc = robot_position_for_angle - target_position_for_angle

        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        cosine_angle = np.clip(cosine_angle, -1.0, 1.0)
        angle = np.degrees(np.arccos(cosine_angle))

        # only append if the angle is within the desired percentage of the circle (2 degrees of tolerances)
        if abs(angle) < (180 * self._full_circle) + 2:
          circle.append(point)

      circle.sort(key=lambda point: self.distance_to_robot(point, robot_position))
      if self._rviz_circle_visualization:
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
        if (
          self._costmap_width is not None
          and self._costmap_height is not None
          and extra_distance > max(self._costmap_width, self._costmap_height)
        ):
          break

    if len(circle) == 0:
      # Fallback to the target position if no valid ring point is found.
      q = self.get_quaternion(target_position, target_position)
      return target_position, q

    if self._rviz_circle_visualization:
      self.show_spheres_rviz(circle, highlight_index=0)

    q = self.get_quaternion(circle[0], target_position)

    return circle[0], q

  def _on_param_change(self, params):
    for param in params:
      if param.name == "origin_frame":
        self._origin_frame = param.value
      elif param.name == "robot_frame":
        self._robot_frame = param.value
      elif param.name == "rviz_line_visualization":
        self._rviz_line_visualization = param.value
      elif param.name == "rviz_circle_visualization":
        self._rviz_circle_visualization = param.value
      elif param.name == "granularity":
        self._granularity = param.value
      elif param.name == "movement_threshold":
        self._movement_threshold = param.value
      elif param.name == "number_points":
        self._number_points = param.value
      elif param.name == "occupancy_path":
        self._occupancy_path = param.value
      elif param.name == "radius":
        self._radius = param.value
      elif param.name == "rate":
        if param.value > 0.0:
          self._rate = self.create_rate(param.value)
      elif param.name == "rotational_threshold":
        self._rotational_threshold = param.value

    return SetParametersResult(successful=True)

  def get_quaternion(self, goal_position, target_position):
    dy = target_position.y - goal_position.y
    dx = target_position.x - goal_position.x
    yaw = math.atan2(dy, dx)
    q = quaternion_from_euler(0, 0, yaw)
    return q

  def is_cell_available(self, point, occupancy):
    if self._map_info is None or self._map is None:
      return False

    [x, y] = self.map_to_index(point.x, point.y)
    ix = int(round(x))
    iy = int(round(y))

    if ix < 0 or iy < 0 or ix >= self._map.shape[0] or iy >= self._map.shape[1]:
      return False

    cost = self._map[ix, iy]
    if cost <= occupancy:
      return True
    else:
      return False

  def is_path_to_target_available(self, origin, target):
    dx = target.x - origin.x
    dy = target.y - origin.y

    number_points = int(round(20 * self._radius))
    if number_points <= 1:
      number_points = 2

    line = []
    for i in range(1, int(round(number_points / 2))):
      t = i / float(number_points)
      point = Point()
      point.x = origin.x + dx * t
      point.y = origin.y + dy * t
      point.z = 0.0

      line.append(point)
      if not self.is_cell_available(point=point, occupancy=self._occupancy_path):
        return False

    if self._rviz_line_visualization:
      self.show_spheres_rviz(line)

    return True

  def map_to_index(self, ix, iy):
    origin_x = self._map_info.origin.position.x
    origin_y = self._map_info.origin.position.y
    resolution = self._map_info.resolution

    mx = (ix - origin_x) / resolution
    my = (iy - origin_y) / resolution
    self.get_logger().debug(
      f"map_to_index: world=({ix:.2f}, {iy:.2f}), origin=({origin_x:.2f}, {origin_y:.2f}), "
      f"resolution={resolution:.2f}, result=({mx:.2f}, {my:.2f})"
    )
    return [mx, my]

  def show_spheres_rviz(self, points, highlight_index=None):
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
      if highlight_index is not None and i == highlight_index:
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
      else:
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
    yaw_delta = yaw - self._memory.last_yaw
    yaw_distance = abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))
    if distance > self._movement_threshold or yaw_distance > self._rotational_threshold:
      return True
    else:
      return False

  def _has_active_navigation_goal(self):
    return (
      self._goal_handle is not None
      and self._get_navigation_result_future is not None
      and not self._get_navigation_result_future.done()
    )

  def _is_current_goal_still_available(self):
    if self._current_navigation_goal is None or self._map_info is None:
      return True

    if not self.is_cell_available(self._current_navigation_goal, 0.0):
      return False

    return True

  # Callbacks

  def _costmap_callback(self, msg):
    self._map_info = msg.info
    self._costmap_width = msg.info.width
    self._costmap_height = msg.info.height

    width = msg.info.width
    height = msg.info.height

    if width <= 0 or height <= 0:
      return

    if self._map is None or self._map.shape != (width, height):
      self._map = np.zeros((width, height))

    data = np.array(msg.data)
    if data.size != width * height:
      self.get_logger().warn("Received malformed costmap data size.")
      return

    self._map = data.reshape((height, width)).T
    self._warned_no_map = False

  def _cancel_navigation_goal_callback(self, future):
    cancel_response = future.result()
    if len(cancel_response.goals_canceling) > 0:
      self.get_logger().info("Requested cancellation of active navigation goal.")
    else:
      self.get_logger().warn("Navigation goal cancellation was not accepted.")
    self._pending_cancel = False

  def _cancel_active_navigation_goal(self):
    if self._goal_handle is None:
      return

    try:
      cancel_future = self._goal_handle.cancel_goal_async()
      cancel_future.add_done_callback(self._cancel_navigation_goal_callback)
    except Exception as e:
      self.get_logger().warn(f"Failed to cancel navigation goal: {e}")

  def _dynamic_goal_goal_callback(self, goal_request):
    _ = goal_request
    if self._dynamic_goal_active:
      self.get_logger().warn("DynamicGoal2 already has an active goal. Rejecting new goal.")
      return GoalResponse.REJECT
    return GoalResponse.ACCEPT

  def _dynamic_goal_cancel_callback(self, goal_handle):
    _ = goal_handle
    self.get_logger().info("Received cancel request for DynamicGoal2 action.")
    self._cancel_active_navigation_goal()
    return CancelResponse.ACCEPT

  def _dynamic_goal_execute_callback(self, goal_handle):
    self.get_logger().info("Starting new DynamicGoal execution")

    # RESET INTERNAL STATE
    self._navigation_goal_finnished = False
    self._goal_handle = None
    self._navigation_goal_future = None
    self._get_navigation_result_future = None
    self._pending_cancel = False

    # Reset memory so next goal works correctly
    self._memory.first_time = True
    self._memory.last_point = None
    self._memory.last_yaw = None

    goal_name = goal_handle.request.goal
    self._navigation_goal_finnished = False
    self._dynamic_goal_active = True
    self._memory.first_time = True

    try:
      while not self._navigation_goal_finnished:
        if goal_handle.is_cancel_requested:
          self._cancel_active_navigation_goal()
          goal_handle.canceled()
          result = DynamicGoal.Result()
          result.success = False
          result.message = "Dynamic goal canceled by client request."
          return result

        update_goal = False
        try:
          target_transform = self._tf_buffer.lookup_transform(self._origin_frame, goal_name, Time())
        except TransformException as e:
          self.get_logger().warn(f"Could not transform from {self._origin_frame} to {goal_name}: {e}")
          self._rate.sleep()
          continue

        target_point = Point()
        target_point.x = target_transform.transform.translation.x
        target_point.y = target_transform.transform.translation.y
        target_point.z = target_transform.transform.translation.z

        target_rotation = [
          target_transform.transform.rotation.x,
          target_transform.transform.rotation.y,
          target_transform.transform.rotation.z,
          target_transform.transform.rotation.w,
        ]
        target_yaw = euler_from_quaternion(target_rotation)[2]

        if not self._memory.first_time:
          update_goal = self.has_target_move(target_point, target_yaw)
          if not update_goal and self._has_active_navigation_goal():
            if not self._is_current_goal_still_available():
              self.get_logger().warn("Current navigation goal is blocked. Recomputing goal.")
              update_goal = True
        elif self._memory.first_time:
          update_goal = True

        if update_goal:
          self._memory.last_point = target_point
          self._memory.last_yaw = target_yaw
          self._memory.first_time = False

          if self._map_info is None:
            if not self._warned_no_map:
              self.get_logger().warn("No costmap received yet, waiting before sending navigation goals.")
              self._warned_no_map = True
            self._rate.sleep()
            continue

          try:
            robot_transform = self._tf_buffer.lookup_transform(self._origin_frame, self._robot_frame, Time())
          except TransformException as e:
            self.get_logger().warn(f"Could not transform from {self._origin_frame} to {self._robot_frame}: {e}")
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

          # Cancel the currently active Nav2 goal before sending the updated one.
          if self._has_active_navigation_goal():
            self._cancel_active_navigation_goal()

          async_goal = NavigateToPose.Goal()
          async_goal.pose = pose
          self._current_navigation_goal = Point(x=goal.x, y=goal.y, z=goal.z)
          self._navigation_goal_future = self._navigation_client.send_goal_async(async_goal)
          self._navigation_goal_future.add_done_callback(self._navigation_goal_response_callback)

        self._rate.sleep()

      goal_handle.succeed()
      result = DynamicGoal.Result()
      result.success = True
      result.message = "Dynamic goal finished successfully."
      return result
    finally:
      self._dynamic_goal_active = False

  def _get_navigation_result_callback(self, future):
    result = future.result()
    if result.status == GoalStatus.STATUS_SUCCEEDED:
      self.get_logger().info("Navigation goal was achieved successfully.")
      self._navigation_goal_finnished = True
      self._current_navigation_goal = None
    elif result.status == GoalStatus.STATUS_CANCELED:
      self.get_logger().info("Navigation goal was canceled.")
      self._current_navigation_goal = None
    elif result.status == GoalStatus.STATUS_ABORTED:
      self.get_logger().warn("Navigation goal was aborted.")
      self._current_navigation_goal = None

  def _navigation_goal_response_callback(self, future):
    self._goal_handle = future.result()
    if not self._goal_handle.accepted:
      self.get_logger().warn("Navigation goal rejected.")
      self._goal = ""
      self._current_navigation_goal = None
      return

    if self._pending_cancel:
      cancel_future = self._goal_handle.cancel_goal_async()
      cancel_future.add_done_callback(self._cancel_navigation_goal_callback)

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
