import os
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from threading import Thread

from ament_index_python.packages import get_package_share_directory
from tf2_ros.transform_broadcaster import TransformBroadcaster

from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import TransformStamped

class SpawnObject(Node):
  def __init__(self):
    super().__init__("spawn_object")
    self.set_parameters([Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    self.broadcaster = TransformBroadcaster(self)
    self._spawn_entity_client = self.create_client(SpawnEntity, "/spawn_entity")
    while not self._spawn_entity_client.wait_for_service(timeout_sec=2.0):
      self.get_logger().warn("Waiting for '/spawn_entity' service...")

    self.timer = self.create_timer(0.1, self.broadcast)

  def spawn_object(self):
    object_path = os.path.join(
      get_package_share_directory("pal_gazebo_worlds"),
      "models", "cocacola", "cocacola.sdf"
    )

    with open(object_path, "r") as f:
      sdf_xml = f.read()

    request = SpawnEntity.Request()
    request.name = "cocacola" # Name of the model
    request.xml = sdf_xml
    request.reference_frame = "map"
    request.initial_pose.position.x = 2.75
    request.initial_pose.position.y = 0.540
    request.initial_pose.position.z = 0.680
    request.initial_pose.orientation.x = 0.0
    request.initial_pose.orientation.y = 0.0
    request.initial_pose.orientation.z = 0.0
    request.initial_pose.orientation.w = 1.0

    response: SpawnEntity.Response = self._spawn_entity_client.call(request=request)
    self.get_logger().info(f"Result: {response.status_message}.")

  def broadcast(self):
    static_transform = TransformStamped()
    static_transform.header.stamp = self.get_clock().now().to_msg()
    static_transform.header.frame_id = "map"
    static_transform.child_frame_id = "cola"

    static_transform.transform.translation.x = 2.75
    static_transform.transform.translation.y = 0.540
    static_transform.transform.translation.z = 0.7975
    static_transform.transform.rotation.x = 0.0
    static_transform.transform.rotation.y = 0.0
    static_transform.transform.rotation.z = 0.0
    static_transform.transform.rotation.w = 1.0

    self.broadcaster.sendTransform(static_transform)
    # self.get_logger().info("Published tf: map -> target_object.")


def main(args=None):
  rclpy.init(args=args)
  node = SpawnObject()
  spin_thread = Thread(target=rclpy.spin, args=(node,))
  spin_thread.start()

  node.spawn_object()
  try:
      spin_thread.join()
  except KeyboardInterrupt:
      pass

  node.destroy_node()
  rclpy.shutdown()