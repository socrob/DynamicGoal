#!/usr/bin/env python
import rospy
import yaml
import tf

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

class TopicToTf(object):
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
        node_name					= config["node_name"]
        rate						= config["rate"]
        self.topic_name         	= config["topic_name"]
        control_topic_name          = config["control_topic_name"]
        self.origin_frame           = config["origin_frame"]
        self.target_frame           = config["target_frame"]
        
        # initializes the node (if debug, initializes in debug mode)
        if debug == True:
            rospy.init_node(node_name, anonymous=False, log_level=rospy.DEBUG)
            rospy.loginfo("%s node created [DEBUG MODE]" % node_name)
        else:
            rospy.init_node(node_name, anonymous=False)
            rospy.loginfo("%s node created" % node_name)
       
  
        #Subscribe to the topic that controls the dynamic goal operation
        self.sub_point = rospy.Subscriber(self.topic_name, PointStamped , self.poiPositionCallback)
        #Subscribe to the topic that controls if this node should public the tf
        self.sub_control = rospy.Subscriber(control_topic_name, String, self.controlCallback)

        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(rate)

        # variables
        self.point = None
        self.active = False

    def poiPositionCallback(self, data):
        poi_position = data.point
        point_in_map_frame = None

        if data.header.frame_id != self.origin_frame:
            try:
                point_in_map_frame = self.listener.transformPoint(self.origin_frame,data)
                poi_position = point_in_map_frame.point
                # self.br.sendTransform((poi_position.x, poi_position.y, poi_position.z),
                #             (0.0, 0.0, 0.0, 1.0),
                #             rospy.Time.now(),
                #             self.target_frame,
                #             "map")
                            #self.origin_frame)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        pass
        # else:
        #     self.br.sendTransform((poi_position.x, poi_position.y, poi_position.z),
        #                  (0.0, 0.0, 0.0, 1.0),
        #                  rospy.Time.now(),
        #                  self.target_frame,
        #                  data.header.frame_id)
        self.point = [poi_position.x, poi_position.y, poi_position.z]

    def controlCallback(self, msg):
        if msg.data == "e_start":
            self.active = True
        elif msg.data == "e_stop":
            self.active = False

    def run(self):
        while not rospy.is_shutdown():
            if self.point is not None and self.active:
                self.br.sendTransform(self.point,
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            self.target_frame,
                            self.origin_frame)
            self.rate.sleep()

def main():
	# create object of the class DynGoal (constructor will get executed!)
	my_object = TopicToTf()
	# call run method of class DynGoal
	my_object.run()

if __name__ == '__main__':
	main()