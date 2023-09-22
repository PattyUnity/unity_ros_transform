#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, OccupancyGrid
import tf

def map_callback(msg):
    global original_map, map_pub, tf_transform, updated_map, z_offset_floor, manual_position_offset, manual_orientation_offset,human_localization_pose,z_offset_human, is_localized
    original_map = msg

    # Copy the original map
    updated_map = original_map
    

    # Add transform from the global frame (ros origin) to the original map origin
    updated_map.info.origin.position.x += tf_transform.transform.translation.x
    updated_map.info.origin.position.y += tf_transform.transform.translation.y
    updated_map.info.origin.position.z += tf_transform.transform.translation.z
    quaternion_combined = tf.transformations.quaternion_multiply(updated_map.info.origin.orientation, tf_transform.transform.rotation)
    updated_map.info.origin.orientation.x = quaternion_combined[0]
    updated_map.info.origin.orientation.y = quaternion_combined[1]
    updated_map.info.origin.orientation.z = quaternion_combined[2]
    updated_map.info.origin.orientation.w = quaternion_combined[3]

    # Add floor transform (ususally only z height)
    updated_map.info.origin.position.z += z_offset_floor

    # Add manual transform
    updated_map.info.origin.position.x += manual_position_offset[0]
    updated_map.info.origin.position.y += manual_position_offset[1]
    updated_map.info.origin.position.z += manual_position_offset[2]
    manual_offset_quaternion = tf.transformations.quaternion_from_euler(manual_orientation_offset[0], manual_orientation_offset[1], manual_orientation_offset[2])
    quaternion_combined = tf.transformations.quaternion_multiply(updated_map.info.origin.orientation, manual_offset_quaternion)
    updated_map.info.origin.orientation.x = quaternion_combined[0]
    updated_map.info.origin.orientation.y = quaternion_combined[1]
    updated_map.info.origin.orientation.z = quaternion_combined[2]
    updated_map.info.origin.orientation.w = quaternion_combined[3]

    # Add human transform
    if is_localized:
        # Add human to robot world transform (all applied) 
        updated_map.info.origin.position.x += human_localization_pose.pose.pose.position.x
        updated_map.info.origin.position.y += human_localization_pose.pose.pose.position.y
        updated_map.info.origin.position.z += human_localization_pose.pose.pose.position.z
        quaternion_combined = tf.transformations.quaternion_multiply(updated_map.info.origin.orientation, human_localization_pose.pose.pose.orientation)
        updated_map.info.origin.orientation.x = quaternion_combined[0]
        updated_map.info.origin.orientation.y = quaternion_combined[1]
        updated_map.info.origin.orientation.z = quaternion_combined[2]
        updated_map.info.origin.orientation.w = quaternion_combined[3]
    else:
        # Add human height (only z height) 
        updated_map.info.origin.position.z += z_offset_human 

    update_map_origin(tf_transform)

    map_pub.publish(updated_map)

def human_localization_callback(msg):
    global human_localization_pose
    human_localization_pose = msg

def is_localized_callback(msg):
    global is_localized
    is_localized = msg.data
    if is_localized and (human_localization_pose.pose.pose.position.x != 0 or 
                     human_localization_pose.pose.pose.position.y != 0 or 
                     human_localization_pose.pose.pose.position.z != 0):
        rospy.Timer(rospy.Duration(1), stop_human_localization_subscriber, oneshot=True)

def stop_human_localization_subscriber(event):
    global human_localization_subscriber
    human_localization_subscriber.unregister()

if __name__ == "__main__":
    rospy.init_node('occupancygrid_transformer')
    parent_frame = rospy.get_param("~parent_frame", "map")
    child_frame = rospy.get_param("~child_frame","robot/map")
    z_offset_human = rospy.get_param("~z_offset_human", 0)
    z_offset_floor = rospy.get_param("~z_offset_floor", 0)
    manual_position_offset = rospy.get_param('~manual_position_offset', [0.0, 0.0, 0.0])
    manual_orientation_offset = rospy.get_param('~manual_orientation_offset', [0.0, 0.0, 0.0])

    tf_transform = Transform()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    map_pub = rospy.Publisher("/global/map", OccupancyGrid, queue_size=50)
    

    original_map = OccupancyGrid()      
    rospy.Subscriber("/robot/map", OccupancyGrid, map_callback)

    is_localized = False
    human_localization_pose = Odometry()
    rospy.Subscriber("/is_localized", Bool, is_localized_callback)
    human_localization_subscriber = rospy.Subscriber("/human_localization_odom", Odometry, human_localization_callback)


    while not rospy.is_shutdown():
        updated_map = OccupancyGrid()    
        try:
            tf_transform = tfBuffer.lookup_transform(parent_frame,child_frame,rospy.Time())
            updated_map.header.frame_id = tf_transform.header.frame_id
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        

        rate.sleep()