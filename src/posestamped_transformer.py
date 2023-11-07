#!/usr/bin/env python

import rospy
import std_msgs
from std_msgs.msg import Bool 
import sensor_msgs
from sensor_msgs.msg import Image
import geometry_msgs
from geometry_msgs.msg import TransformStamped, Transform, PoseStamped
import tf
import threading
import numpy as np

def msg_callback(message):
    global global_pub, tf_transform, global_msg
    if tf_transform is None:
        return

    local_msg = message

    # Convert transform from tf into rotation matrix
    trans = tf_transform[0]
    rot =  tf_transform[1]
    map_to_pose_tf = tf.transformations.quaternion_matrix(rot)
    map_to_pose_tf[0:3, -1] = trans

    # Convert pose of the object into rotation matrix
    pose_local_tf = tf.transformations.quaternion_matrix([local_msg.pose.orientation.x,local_msg.pose.orientation.y, local_msg.pose.orientation.z, local_msg.pose.orientation.w])
    pose_local_tf[0:3,-1] = [local_msg.pose.position.x, local_msg.pose.position.y, local_msg.pose.position.z]

    # print('human received', local_msg.pose.position)

    # multiply two transformation matrices
    pose_resultant = np.dot(map_to_pose_tf, pose_local_tf)
    # Convert the resultant transformation matrix to position and orientation
    global_msg.pose.position.x = pose_resultant[0,3]
    global_msg.pose.position.y = pose_resultant[1,3]
    global_msg.pose.position.z = pose_resultant[2,3]
    q_resultant = tf.transformations.quaternion_from_matrix(pose_resultant)
    global_msg.pose.orientation.x = q_resultant[0]
    global_msg.pose.orientation.y = q_resultant[1]
    global_msg.pose.orientation.z = q_resultant[2]
    global_msg.pose.orientation.w = q_resultant[3]

    global_msg.header = local_msg.header

    global_pub.publish(global_msg)


if __name__ == "__main__":
    rospy.init_node('tf2_listener')
    parent_frame = rospy.get_param("~parent_frame", "map")
    child_frame = rospy.get_param("~child_frame","vision_1/odom")


    tf_listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    global_pub = rospy.Publisher("global_topic",PoseStamped, queue_size=5)  

    rospy.Subscriber("local_topic", PoseStamped, msg_callback) 
    global_msg = PoseStamped()  

    while not rospy.is_shutdown():
        try:
            tf_transform = tf_listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))

            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            continue

         

        rate.sleep()