#!/usr/bin/env python

import rospy
import std_msgs
import geometry_msgs
import nav_msgs
from nav_msgs.msg import Odometry
import tf2_ros

if __name__ == "__main__":
    rospy.init_node('tf2_listener')
    parent_frame = rospy.get_param("~parent_frame", "map")
    child_frame = rospy.get_param("~child_frame","vision_1/odom")
    z_offset = rospy.get_param("~z_offset", 0)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    odom_pub = rospy.Publisher("odom_to_map",Odometry, queue_size=50)  



    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(parent_frame, child_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg = Odometry()

        msg.header = trans.header
        msg.child_frame_id = trans.child_frame_id
        msg.pose.pose.position.x = trans.transform.translation.x
        msg.pose.pose.position.y = trans.transform.translation.y
        msg.pose.pose.position.z = trans.transform.translation.z + z_offset
        msg.pose.pose.orientation = trans.transform.rotation

        odom_pub.publish(msg)

        rate.sleep()