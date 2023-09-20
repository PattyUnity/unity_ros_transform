#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import quaternion_from_euler

def map_callback(msg):
    global original_map
    original_map = msg

def update_map_origin(floor_transform):
    global updated_map
    updated_map.info.origin.position.x += floor_transform.transform.translation.x
    updated_map.info.origin.position.y += floor_transform.transform.translation.y
    updated_map.info.origin.position.z += floor_transform.transform.translation.z
    updated_map.info.origin.orientation.x += floor_transform.transform.rotation.x
    updated_map.info.origin.orientation.y += floor_transform.transform.rotation.y
    updated_map.info.origin.orientation.z += floor_transform.transform.rotation.z
    updated_map.info.origin.orientation.w += floor_transform.transform.rotation.w

    
if __name__ == "__main__":
    rospy.init_node('occupancygrid_transformer')
    parent_frame = rospy.get_param("~parent_frame", "map")
    child_frame = rospy.get_param("~child_frame","robot/map")
    origin_position_offset = rospy.get_param('~origin_position_offset', [0.0, 0.0, 0.0])
    origin_orientation_offset = rospy.get_param('~origin_orientation_offset', [0.0, 0.0, 0.0])

    floor_transform = Transform()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    map_pub = rospy.Publisher("/global/map", OccupancyGrid, queue_size=50)

    original_map = OccupancyGrid()
    updated_map = OccupancyGrid()
    rospy.Subscriber("/robot/map", OccupancyGrid, map_callback)

    while not rospy.is_shutdown():
        try:
            floor_transform = tfBuffer.lookup_transform(parent_frame,child_frame,rospy.Time())
        except tf2.LookupException as e:
            rospy.logwarn("Robot Transform lookup for Floor offset failed: {}".format(e))
            rate.sleep()
            continue
        
        floor_transform.transform.translation.x += origin_position_offset[0]
        floor_transform.transform.translation.y += origin_position_offset[1]
        floor_transform.transform.translation.z += origin_position_offset[2]
        quaternion = quaternion_from_euler(origin_orientation_offset[0], origin_orientation_offset[1], origin_orientation_offset[2])
        floor_transform.transform.rotation.x += quaternion[0]
        floor_transform.transform.rotation.y += quaternion[1]
        floor_transform.transform.rotation.z += quaternion[2]
        floor_transform.transform.rotation.w += quaternion[3]

        update_map_origin(floor_transform)

        map_pub.publish(updated_map)

        rate.sleep()