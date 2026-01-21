import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from dynamic_goal2_action.action import DynamicGoal


class TestClient(Node):
  def __init__(self):
    super().__init__("test_client")
    self._action_client = ActionClient(self, DynamicGoal, "/dynamic_goal2")
    self._action_client.wait_for_server()

  def send_goal(self, goal_name: str):
    goal_msg = DynamicGoal.Goal()
    goal_msg.goal = goal_name

    self.get_logger().info(f"Sending goal: {goal_name}.")
    goal_future = self._action_client.send_goal_async(goal_msg)
    goal_future.add_done_callback(self.goal_response_callback)

  def goal_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
        self.get_logger().warn("Goal rejected by server.")
        return

    self.get_logger().info("Goal accepted. Waiting for result...")
    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(self.get_result_callback)

  def get_result_callback(self, future):
    result = future.result().result
    self.get_logger().info(f"Goal finished: success={result.success}.'")

def main(args=None):
  rclpy.init(args=args)
  node = TestClient()
  node.send_goal("pringles2")

  rclpy.spin(node)

  node.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
  main()
