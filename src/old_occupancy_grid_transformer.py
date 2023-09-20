#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

class OccupancyGridTransformer:
    def __init__(self):
        rospy.init_node('occupancygrid_transformer')
        self.parent_frame = rospy.get_param("~parent_frame", "map")
        self.child_frame = rospy.get_param("~child_frame", "robot/map")
        self.origin_position_offset = rospy.get_param('~origin_position_offset', [0.0, 0.0, 0.0])
        self.origin_orientation_offset = rospy.get_param('~origin_orientation_offset', [0.0, 0.0, 0.0])

        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)

        self.map_pub = rospy.Publisher("/global/map", OccupancyGrid, queue_size=50)
        self.original_map = None
        self.updated_map = None
        rospy.Subscriber("/robot/map", OccupancyGrid, self.map_callback)

    def map_callback(self, msg):
        self.original_map = msg

    def update_map_origin(self, floor_transform):
        if self.updated_map is not None:
            self.updated_map.info.origin.position.x += floor_transform.transform.translation.x
            self.updated_map.info.origin.position.y += floor_transform.transform.translation.y
            self.updated_map.info.origin.position.z += floor_transform.transform.translation.z
            self.updated_map.info.origin.orientation.x += floor_transform.transform.rotation.x
            self.updated_map.info.origin.orientation.y += floor_transform.transform.rotation.y
            self.updated_map.info.origin.orientation.z += floor_transform.transform.rotation.z
            self.updated_map.info.origin.orientation.w += floor_transform.transform.rotation.w

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                floor_transform = self.tfBuffer.lookup_transform(
                    self.parent_frame, self.child_frame, rospy.Time())
            except tf2.LookupException as e:
                rospy.logwarn("Robot Transform lookup for Floor offset failed: {}".format(e))
                rate.sleep()
                continue

            floor_transform.transform.translation.x += self.origin_position_offset[0]
            floor_transform.transform.translation.y += self.origin_position_offset[1]
            floor_transform.transform.translation.z += self.origin_position_offset[2]
            quaternion = quaternion_from_euler(*self.origin_orientation_offset)
            floor_transform.transform.rotation.x += quaternion[0]
            floor_transform.transform.rotation.y += quaternion[1]
            floor_transform.transform.rotation.z += quaternion[2]
            floor_transform.transform.rotation.w += quaternion[3]

            if self.original_map is not None:
                self.updated_map = self.original_map
                self.update_map_origin(floor_transform)
                self.map_pub.publish(self.updated_map)

            rate.sleep()

if __name__ == "__main__":
    try:
        transformer = OccupancyGridTransformer()
        transformer.run()
    except rospy.ROSInterruptException:
        pass


# class OccupancyGridTransformer:
#     def __init__(self):
#         rospy.init_node('occupancy_grid_transformer', anonymous=True)
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
#         self.parent_frame = rospy.get_param('~parent_frame', 'parent_frame')
#         self.child_frame = rospy.get_param('~child_frame', 'child_frame')
#         self.map_origin_offset = rospy.get_param('~map_origin_offset', [0.0, 0.0, 0.0])
#         self.odom_topic = rospy.get_param('~odom_topic', '/odom')
#         self.trigger_topic = rospy.get_param('~trigger_topic', '/trigger')
#         self.input_map_topic = rospy.get_param('~input_map_topic', '/input_map')
#         self.output_map_topic = rospy.get_param('~output_map_topic', '/output_map')

#         self.transformed_map = None
#         self.tf_ready = False
#         self.use_odom_offset = False

#         self.trigger_sub = rospy.Subscriber(self.trigger_topic, Bool, self.trigger_callback)
#         self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
#         self.map_sub = rospy.Subscriber(self.input_map_topic, OccupancyGrid, self.map_callback)
#         self.map_pub = rospy.Publisher(self.output_map_topic, OccupancyGrid, queue_size=10)

#     def trigger_callback(self, trigger_msg):
#         if trigger_msg.data:
#             self.tf_ready = True
#         else:
#             self.tf_ready = False

#     def odom_callback(self, odom_msg):
#         if self.use_odom_offset and self.tf_ready:  
#             # Extract z-height from odometry message and add it to the map origin offset
#             z_height = odom_msg.pose.pose.position.z
#             self.map_origin_offset[2] += z_height

#     def map_callback(self, map_msg):
#         if self.tf_ready:
#             try:
#                 transform = self.tf_buffer.lookup_transform(self.parent_frame, self.child_frame, rospy.Time(0))
#                 translation = transform.transform.translation

#                 # Create a copy of the input map
#                 self.transformed_map = map_msg

#                 # Update the map origin based on the TF transform and optional offset
#                 self.transformed_map.info.origin.position.x += translation.x + self.map_origin_offset[0]
#                 self.transformed_map.info.origin.position.y += translation.y + self.map_origin_offset[1]
#                 self.transformed_map.info.origin.position.z += translation.z + self.map_origin_offset[2]

#                 # Publish the transformed map
#                 self.map_pub.publish(self.transformed_map)

#             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#                 rospy.logwarn("TF lookup failed")

# if __name__ == '__main__':
#     try:
#         occupancy_grid_transformer = OccupancyGridTransformer()
#         occupancy_grid_transformer.run()
#     except rospy.ROSInterruptException:
#         pass