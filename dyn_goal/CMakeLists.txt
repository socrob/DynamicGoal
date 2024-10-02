cmake_minimum_required(VERSION 2.8.3)
project(dyn_goal)

find_package(catkin REQUIRED COMPONENTS
   rospy
   std_msgs
   tf2_msgs
   message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  dyn_goal_msg.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs)

catkin_package(
 CATKIN_DEPENDS
 message_runtime
)


