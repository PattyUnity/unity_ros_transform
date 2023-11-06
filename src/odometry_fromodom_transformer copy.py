#!/usr/bin/env python

import rospy
import std_msgs
from std_msgs.msg import Bool 
import geometry_msgs
from geometry_msgs.msg import TransformStamped, Transform
import nav_msgs
from nav_msgs.msg import Odometry
import tf
import tf2_ros
import threading


def update_odom_callback(message):
    global odom_pub, original_odom, tf_transform, updated_odom, z_offset_floor, manual_position_offset, manual_orientation_offset,human_localization_pose,z_offset_human, is_localized
    original_odom = message

    # Copy the original odom
    updated_odom = original_odom
    updated_odom.header.frame_id = tf_transform.header.frame_id

    # Add transform from the global frame (ros origin) to the original odom
    updated_odom.pose.pose.position.x += tf_transform.transform.translation.x
    updated_odom.pose.pose.position.y += tf_transform.transform.translation.y
    updated_odom.pose.pose.position.z += tf_transform.transform.translation.z
    q_1 = get_quaternion_in_tuples(updated_odom,case_transform=False)
    q_2 = get_quaternion_in_tuples(tf_transform,case_transform=True)
    quaternion_combined = tf.transformations.quaternion_multiply(q_1, q_2)
    updated_odom.pose.pose.orientation.x = quaternion_combined[0]
    updated_odom.pose.pose.orientation.y = quaternion_combined[1]
    updated_odom.pose.pose.orientation.z = quaternion_combined[2]
    updated_odom.pose.pose.orientation.w = quaternion_combined[3]

    # Add floor transform (ususally only z height)
    updated_odom.pose.pose.position.z += z_offset_floor

    # Add manual transform
    updated_odom.pose.pose.position.x += manual_position_offset[0]
    updated_odom.pose.pose.position.y += manual_position_offset[1]
    updated_odom.pose.pose.position.z += manual_position_offset[2]
    manual_offset_quaternion = tf.transformations.quaternion_from_euler(manual_orientation_offset[0], manual_orientation_offset[1], manual_orientation_offset[2])
    q_1 = get_quaternion_in_tuples(updated_odom,case_transform=False)     
    quaternion_combined = tf.transformations.quaternion_multiply(q_1, manual_offset_quaternion)
    updated_odom.pose.pose.orientation.x = quaternion_combined[0]
    updated_odom.pose.pose.orientation.y = quaternion_combined[1]
    updated_odom.pose.pose.orientation.z = quaternion_combined[2]
    updated_odom.pose.pose.orientation.w = quaternion_combined[3]

    # Add human transform
    if is_localized:
        # Add human to robot world transform (all applied) 
        updated_odom.pose.pose.position.x += human_localization_pose.pose.pose.position.x
        updated_odom.pose.pose.position.y += human_localization_pose.pose.pose.position.y
        updated_odom.pose.pose.position.z += human_localization_pose.pose.pose.position.z
        q_1 = get_quaternion_in_tuples(updated_odom,case_transform=False)
        q_2 = get_quaternion_in_tuples(human_localization_pose,case_transform=False)
        quaternion_combined = tf.transformations.quaternion_multiply(q_1, q_2)
        updated_odom.pose.pose.orientation.x = quaternion_combined[0]
        updated_odom.pose.pose.orientation.y = quaternion_combined[1]
        updated_odom.pose.pose.orientation.z = quaternion_combined[2]
        updated_odom.pose.pose.orientation.w = quaternion_combined[3]
    else:
        # Add human height (only z height) 
        updated_odom.pose.pose.position.z += z_offset_human 

    odom_pub.publish(updated_odom)



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
    # human_localization_subscriber.unregister()

def get_quaternion_in_tuples(msg,case_transform):
    if case_transform:
        q_out = (msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w)
    else:
        q_out = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    return q_out 

if __name__ == "__main__":
    rospy.init_node('tf2_listener')
    parent_frame = rospy.get_param("~parent_frame", "map")
    child_frame = rospy.get_param("~child_frame","vision_1/odom")
    z_offset_human = rospy.get_param("~z_offset_human", 0)
    z_offset_floor = rospy.get_param("~z_offset_floor", 0)
    manual_position_offset = rospy.get_param('~manual_position_offset', [0.0, 0.0, 0.0])
    manual_orientation_offset = rospy.get_param('~manual_orientation_offset', [0.0, 0.0, 0.0])
    # manual_position_offset = [float(manual_position_offset_in[0])]
    # manual_orientation_offset = tuple(manual_orientation_offset)

    print(manual_position_offset)

    tf_transform = TransformStamped()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    odom_pub = rospy.Publisher("global_odom",Odometry, queue_size=50)     
    

    original_odom = Odometry()
    rospy.Subscriber("odom", Odometry, update_odom_callback)

    is_localized = False
    human_localization_pose = Odometry()
    rospy.Subscriber("/is_localized", Bool, is_localized_callback)
    human_localization_subscriber = rospy.Subscriber("/human_localization_odom", Odometry, human_localization_callback)


    while not rospy.is_shutdown():
        updated_odom = Odometry() 
        try:
            tf_transform = tfBuffer.lookup_transform(parent_frame, child_frame, rospy.Time(0))
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # rate.sleep()
            print(e)
            continue

         

        rate.sleep


