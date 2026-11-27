# DynamicGoal

`dynamic_goal2` drives a Nav2 robot to a moving target: given a TF frame (a person, an object), it keeps sending the robot to a free point at a set distance from the frame, facing it, and updates the goal as the frame moves. It works with any Nav2 robot; Nav2 does the planning and control.

| Package | Content |
| --- | --- |
| `dynamic_goal2` | The node (`ros2 run dynamic_goal2 dynamic_goal2`) |
| `dynamic_goal2_action` | The `DynamicGoal` action: `string goal` (the TF frame to approach) -> `bool success`, `string message` |

## Interfaces

| Kind | Name | Type | Use |
| --- | --- | --- | --- |
| Action server | `/dynamic_goal2` | `dynamic_goal2_action/action/DynamicGoal` | Approach or follow the frame named in `goal`; cancel to stop. A new goal replaces the active one. |
| Action client | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | The goals sent to Nav2 |
| Subscription | `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Where the robot can stand (costs 0-100) |
| Subscription | `/map` | `nav_msgs/msg/OccupancyGrid` | Unknown cells, rejected unless `allow_unknown_cells` |
| Publisher | `/visualization_marker_array` | `visualization_msgs/msg/MarkerArray` | The candidate points (blue) and the chosen one (red) |

The target's position is read from TF (`origin_frame` -> `goal`), so whatever publishes the frame (a person tracker, a static transform) drives the robot.

## How the goal is chosen

At every update (`rate`), if the target moved more than `movement_threshold` (or turned more than `rotational_threshold`), or the current goal became blocked:

1. Candidate points are placed on a circle of `radius` around the target (`number_points` per metre of radius), keeping the part of the circle facing the robot (`full_circle`: 1.0 the whole circle, 0.5 the half towards the robot).
2. The candidates are sorted, nearest to the robot first (`goal_selection`, below).
3. The first candidate that is free (costmap cost up to `occupancy_threshold`, known) and has a clear line towards the target (cost up to `occupancy_path` along `path_to_target_fraction` of the line) becomes the goal, facing the target.
4. If none qualifies, the circle grows (by `granularity`) and the search repeats.

When the robot is already within `radius` of the target and `align_only_when_within_radius` is set, it only turns to face it.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `origin_frame` | `map` | Frame of the goals and of the costmap |
| `robot_frame` | `base_footprint` | The robot's frame |
| `radius` | 0.50 | Distance (m) to keep from the target |
| `rate` | 0.50 | Updates per second |
| `movement_threshold` | 0.25 | Target displacement (m) that triggers a new goal |
| `rotational_threshold` | 0.90 | Target rotation (rad) that triggers a new goal |
| `number_points` | 50 | Candidate points per metre of radius |
| `full_circle` | 1.0 | Fraction of the circle, centred on the robot's side, that is searched |
| `granularity` | 0.1 | How fast the circle grows when no candidate is free |
| `occupancy_threshold` | 0.0 | Highest costmap cost (0-100) of a goal. 0 accepts only cells outside every obstacle's inflation, which can leave no candidate in narrow places; with Nav2's costmaps any value below 99 (the robot's inscribed radius) is a valid goal |
| `occupancy_path` | 90.0 | Highest cost along the line from the goal to the target |
| `path_to_target_fraction` | 0.5 | Part of that line that is checked (0 to 1) |
| `allow_unknown_cells` | false | Accept goals in unknown map cells |
| `align_only_when_within_radius` | true | Within `radius`, only turn to face the target |
| `follow_until_cancel` | true | Keep following after reaching the target; false finishes when the goal is reached |
| `goal_selection` | `straight_line` | How candidates are sorted: `straight_line` by distance to the robot; `path` by the length of the robot's path to them through the costmap (below) |
| `cancel_before_new_goal` | true | Cancel the active Nav2 goal before sending the next; false lets Nav2 replace it (below) |
| `rviz_circle_visualization` | true | Publish the candidate points |
| `rviz_line_visualization` | false | Publish the checked line to the target |

All of them can be changed while the node runs (`ros2 param set`) except `occupancy_threshold` and `full_circle`, which are only read at start: setting them later is accepted but has no effect.

## Following around furniture: `goal_selection` and `cancel_before_new_goal`

With the defaults, the nearest candidate is the nearest in a straight line. When furniture stands between the robot and the target (a person walking round a sofa), the straight-line nearest points are behind the furniture; the free ones left switch from one side of the target to the other as it moves, and the robot drives back and forth. Each update also cancels the active Nav2 goal before sending the next, which stops the robot every `1 / rate` seconds.

- `goal_selection: path` measures the robot's path to every candidate over the costmap (Dijkstra on the cells with cost below 99, in a window around the robot and the target; a few milliseconds) and takes the nearest reachable one. Candidates the robot can't reach are dropped, and so are those whose path goes past the target (longer than the path to the target plus half the radius; the path to the target ends at the best reachable cell within `radius` of it, since a person seen by the laser is an obstacle in the costmap): when the side facing the robot is too costly, the circle grows on that side instead of sending the robot round the target. In open space it chooses the same point as `straight_line`. Within 0.5 m of the robot the path may also cross inscribed cells (99), so a robot standing in an obstacle's inflation, or next to a person the laser marked, can still get out. If no candidate within 3 m of the circle is reachable, that update falls back to `straight_line`.
- `cancel_before_new_goal: false` sends each new goal straight away; Nav2 replaces the active goal without stopping the robot, so the goal can be updated more often.

Both are off by default, so robots that work well with the original behaviour don't change. They were made for an omnidirectional base following a person through a furnished room, together with `occupancy_threshold` around 70.

## Tests

```bash
cd ~/ros2_ws && python3 -m pytest -p no:anyio src/DynamicGoal/dynamic_goal2/test/test_goal_selection.py
```

`test_goal_selection.py` checks the goal choice on synthetic costmaps without a ROS graph: a wall between the robot and the target (straight line picks the side through the wall, path the side the robot can reach), open space (both agree), unreachable cells, a robot standing inside an obstacle's inflation, a robot with nothing reachable (falls back to straight line), and a corridor where the only free point is past the target (not chosen), also with the target as an inflated obstacle, like a person.
