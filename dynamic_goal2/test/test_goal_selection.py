"""Goal choice around obstacles: straight-line vs path distance (no ROS graph needed)."""

import logging
import math
import time

import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import MapMetaData

from dynamic_goal2.dynamic_goal2 import DynamicGoal2

RESOLUTION = 0.05


def make_node(costmap, goal_selection):
  """A DynamicGoal2 with only what choose_navigation_goal needs, on a costmap indexed [x, y]."""
  node = DynamicGoal2.__new__(DynamicGoal2)
  logger = logging.getLogger('dynamic_goal2_test')
  node.get_logger = lambda: logger
  node._map = costmap
  node._map_info = MapMetaData(resolution=RESOLUTION, width=costmap.shape[0], height=costmap.shape[1])
  node._raw_map = None
  node._raw_map_info = None
  node._costmap_width = costmap.shape[0]
  node._costmap_height = costmap.shape[1]
  node._radius = 1.0
  node._granularity = 0.1
  node._number_points = 50
  node._full_circle = 1.0
  node._occupancy_threshold = 70.0
  node._occupancy_path = 90.0
  node._path_to_target_fraction = 0.0
  node._align_only_when_within_radius = True
  node._allow_unknown_cells = True
  node._rviz_circle_visualization = False
  node._rviz_line_visualization = False
  node._goal_selection = goal_selection
  return node


def wall_costmap():
  """5 x 5 m of free floor, a wall at x = 2.0 m from y = 0 up to 3.5 m, inflated like Nav2.

  The only way from the left of the wall to its right is above y = 3.5 m.
  """
  size = int(5.0 / RESOLUTION)
  costmap = np.zeros((size, size))
  x = (np.arange(size) + 0.5) * RESOLUTION
  y = (np.arange(size) + 0.5) * RESOLUTION
  xx, yy = np.meshgrid(x, y, indexing='ij')
  # Distance to the wall segment
  dy = np.clip(yy, 0.0, 3.5) - yy
  distance = np.hypot(xx - 2.0, dy)
  costmap[distance < 0.45] = 99
  costmap[distance < 0.05] = 100
  ramp = (distance >= 0.45) & (distance < 0.7)
  costmap[ramp] = 98 * np.exp(-4.0 * (distance[ramp] - 0.45))
  return costmap


def test_path_goes_around_the_wall():
  costmap = wall_costmap()
  robot = Point(x=1.0, y=0.6)
  target = Point(x=3.2, y=1.6)

  straight, _ = make_node(costmap, 'straight_line').choose_navigation_goal(robot, target)
  path, _ = make_node(costmap, 'path').choose_navigation_goal(robot, target)

  # Straight line: the free point nearest the robot through the wall, below the target
  assert straight.y < target.y
  # Path: the robot has to go round the top of the wall, so the goal is above the target
  assert path.y > target.y
  for goal in (straight, path):
    assert math.hypot(goal.x - target.x, goal.y - target.y) >= 1.0 - RESOLUTION
    assert goal.x > 2.0


def test_same_side_as_straight_line_in_the_open():
  costmap = np.zeros((100, 100))
  robot = Point(x=1.0, y=2.5)
  target = Point(x=3.5, y=2.5)

  straight, _ = make_node(costmap, 'straight_line').choose_navigation_goal(robot, target)
  path, _ = make_node(costmap, 'path').choose_navigation_goal(robot, target)

  # Without obstacles both choose the point between the robot and the target
  assert abs(straight.x - 2.5) < 0.05 and abs(straight.y - 2.5) < 0.05
  assert abs(path.x - 2.5) < 0.05 and abs(path.y - 2.5) < 0.05


def test_unreachable_points_are_dropped():
  costmap = wall_costmap()
  # Close the way round: the wall now reaches the top edge
  costmap[int(1.6 / RESOLUTION):int(2.4 / RESOLUTION), :] = 99
  node = make_node(costmap, 'path')
  field = node._path_distances(Point(x=1.0, y=0.6), Point(x=3.2, y=1.6))
  assert math.isinf(node._path_distance_at(field, Point(x=3.2, y=0.6)))
  assert math.isfinite(node._path_distance_at(field, Point(x=0.5, y=4.0)))


def test_robot_inside_inflation_still_finds_a_path():
  costmap = wall_costmap()
  # The robot stands 0.3 m from the wall, inside its inscribed band (cost 99)
  robot = Point(x=1.7, y=1.0)
  node = make_node(costmap, 'path')
  field = node._path_distances(robot, Point(x=3.2, y=1.6))
  assert math.isfinite(node._path_distance_at(field, Point(x=1.0, y=1.0)))

  goal, _ = node.choose_navigation_goal(robot, Point(x=0.5, y=2.5))
  assert goal is not None


def test_nothing_reachable_falls_back_to_straight_line():
  costmap = wall_costmap()
  # The robot is boxed in by a ring of lethal cells beyond its escape radius
  robot = Point(x=1.0, y=1.0)
  x = (np.arange(costmap.shape[0]) + 0.5) * RESOLUTION
  y = (np.arange(costmap.shape[1]) + 0.5) * RESOLUTION
  xx, yy = np.meshgrid(x, y, indexing='ij')
  ring = np.abs(np.hypot(xx - robot.x, yy - robot.y) - 0.7) < 0.1
  costmap[ring] = 100

  start = time.time()
  goal, _ = make_node(costmap, 'path').choose_navigation_goal(robot, Point(x=3.2, y=1.6))
  assert time.time() - start < 2.0
  assert goal is not None


def test_goal_does_not_overtake_the_target():
  # A 1 m wide corridor along y (x 2.0 to 3.0); a costly band (80, above the 70 limit)
  # crosses it where the circle meets the corridor on the robot's side
  size = int(5.0 / RESOLUTION)
  costmap = np.full((size, size), 100.0)
  costmap[int(2.0 / RESOLUTION):int(3.0 / RESOLUTION), :] = 0.0
  costmap[int(2.0 / RESOLUTION):int(3.0 / RESOLUTION), int(1.4 / RESOLUTION):int(1.75 / RESOLUTION)] = 80.0
  robot = Point(x=2.5, y=0.3)
  target = Point(x=2.5, y=2.5)

  goal, _ = make_node(costmap, 'path').choose_navigation_goal(robot, target)

  # Not the free point past the target (y 3.5), but a point before the costly band
  assert goal.y < 1.4
  assert abs(goal.x - 2.5) < 0.5


def test_no_overtaking_when_the_target_is_an_obstacle():
  # Same corridor, but the target is a person the laser sees: lethal legs and the
  # inscribed band around them, so no cell within 0.5 m of the target is reachable
  size = int(5.0 / RESOLUTION)
  costmap = np.full((size, size), 100.0)
  costmap[int(2.0 / RESOLUTION):int(3.0 / RESOLUTION), :] = 0.0
  costmap[int(2.0 / RESOLUTION):int(3.0 / RESOLUTION), int(1.4 / RESOLUTION):int(1.75 / RESOLUTION)] = 80.0
  robot = Point(x=2.5, y=0.3)
  target = Point(x=2.5, y=2.5)
  x = (np.arange(size) + 0.5) * RESOLUTION
  y = (np.arange(size) + 0.5) * RESOLUTION
  xx, yy = np.meshgrid(x, y, indexing='ij')
  to_person = np.hypot(xx - target.x, yy - target.y)
  costmap[to_person < 0.55] = 99
  costmap[to_person < 0.15] = 100

  goal, _ = make_node(costmap, 'path').choose_navigation_goal(robot, target)

  assert goal.y < target.y
