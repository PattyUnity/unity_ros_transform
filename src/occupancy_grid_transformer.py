#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, OccupancyGrid

class OccupancyGridTransformer:
    def __init__(self):
        rospy.init_node('occupancy_grid_transformer', anonymous=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.parent_frame = rospy.get_param('~parent_frame', 'parent_frame')
        self.child_frame = rospy.get_param('~child_frame', 'child_frame')
        self.map_origin_offset = rospy.get_param('~map_origin_offset', [0.0, 0.0, 0.0])
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.trigger_topic = rospy.get_param('~trigger_topic', '/trigger')
        self.input_map_topic = rospy.get_param('~input_map_topic', '/input_map')
        self.output_map_topic = rospy.get_param('~output_map_topic', '/output_map')

        self.transformed_map = None
        self.tf_ready = False
        self.use_odom_offset = False

        self.trigger_sub = rospy.Subscriber(self.trigger_topic, Bool, self.trigger_callback)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        self.map_sub = rospy.Subscriber(self.input_map_topic, OccupancyGrid, self.map_callback)
        self.map_pub = rospy.Publisher(self.output_map_topic, OccupancyGrid, queue_size=10)

    def trigger_callback(self, trigger_msg):
        if trigger_msg.data:
            self.tf_ready = True
        else:
            self.tf_ready = False

    def odom_callback(self, odom_msg):
        if self.use_odom_offset and self.tf_ready:
            # Extract z-height from odometry message and add it to the map origin offset
            z_height = odom_msg.pose.pose.position.z
            self.map_origin_offset[2] += z_height

    def map_callback(self, map_msg):
        if self.tf_ready:
            try:
                transform = self.tf_buffer.lookup_transform(self.parent_frame, self.child_frame, rospy.Time(0))
                translation = transform.transform.translation

                # Create a copy of the input map
                self.transformed_map = map_msg

                # Update the map origin based on the TF transform and optional offset
                self.transformed_map.info.origin.position.x += translation.x + self.map_origin_offset[0]
                self.transformed_map.info.origin.position.y += translation.y + self.map_origin_offset[1]
                self.transformed_map.info.origin.position.z += translation.z + self.map_origin_offset[2]

                # Publish the transformed map
                self.map_pub.publish(self.transformed_map)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("TF lookup failed")

if __name__ == '__main__':
    try:
        occupancy_grid_transformer = OccupancyGridTransformer()
        occupancy_grid_transformer.run()
    except rospy.ROSInterruptException:
        pass
