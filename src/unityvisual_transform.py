#!/usr/bin/env python

import rospy
import std_msgs
import geometry_msgs
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import nav_msgs
from nav_msgs.msg import Odometry
import tf2_ros

def get_6Dof_pose(trans, rot, time=None):
        """
        :param trans translation x,y,z
        :param rot rotation x,y,z,w
        """
        if time is None:
            time = rospy.get_rostime()
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = time
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = trans[0]
        pose_msg.pose.pose.position.y = trans[1]
        pose_msg.pose.pose.position.z = trans[2]

        pose_msg.pose.pose.orientation.w = rot[3]
        pose_msg.pose.pose.orientation.x = rot[0]
        pose_msg.pose.pose.orientation.y = rot[1]
        pose_msg.pose.pose.orientation.z = rot[2]
        return pose_msg

if __name__ == "__main__":
    rospy.init_node('unityvisual_tf2_listener')
    print("unityvisual_tf2_listener initialized")

    output_topic = rospy.get_param("~output_topic", "/output_topic")
    parent_frame = rospy.get_param("~parent_frame", "map")
    child_frame = rospy.get_param("~child_frame","vision_1/odom")
    x_offset = rospy.get_param("~x_offset", 0)
    y_offset = rospy.get_param("~y_offset", 0)
    z_offset = rospy.get_param("~z_offset", 0)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)

    odom_pub = rospy.Publisher(output_topic,Odometry,queue_size =50)


    