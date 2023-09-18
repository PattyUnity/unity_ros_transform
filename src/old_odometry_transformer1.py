#!/usr/bin/env python

import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def odom_callback(odom_msg, args):    
    global_odom = args[0]
    tf_Buffer = args[1]
    parent_frame = args[2]
    child_frame = args[3]

    listener = tf.Transformer(True,rospy.Duration(10.0))

    try:
        listener.waitForTransform(parent_frame,child_frame, rospy.Time(), rospy.Duration(2.0))
    except ():
        return
        

    try:
        # listener.waitForTransform( parent_frame, child_frame, rospy.Time.now() ,rospy.Duration(2.0))
        transform = tf_Buffer.lookup_transform(parent_frame, child_frame, rospy.Time())
        translation = transform.transform.translation
        rotation = transform.transform.rotation
    except (tf2_ros.ExtrapolationException):
        print("tf lookup failed")
        return global_odom

    global_odom.header = odom_msg.header    
    global_odom.child_frame_id = transform.child_frame_id
    global_odom.pose.pose.position = odom_msg.pose.pose.position
    global_odom.pose.pose.orientation = odom_msg.pose.pose.orientation
    global_odom.twist = odom_msg.twist

    # # Add tf transfrom to the original odom
    # global_odom.pose.pose.position.x += translation.x
    # global_odom.pose.pose.position.y += translation.y
    # global_odom.pose.pose.position.z += translation.z
    # global_odom.pose.pose.orientation.x += rotation.x
    # global_odom.pose.pose.orientation.y += rotation.y
    # global_odom.pose.pose.orientation.z += rotation.z
    # global_odom.pose.pose.orientation.w += rotation.w

    return global_odom

if __name__ == "__main__":
    rospy.init_node('odometry_transformer', anonymous=True)
    child_frame = rospy.get_param('~child_frame', 'child_frame')
    parent_frame = rospy.get_param('~parent_frame', 'parent_frame')

    tf_Buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_Buffer)

    global_odom = Odometry()

    rate = rospy.Rate(10.0)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback, (global_odom, tf_Buffer, parent_frame,child_frame)) #Original odometry wrt local frame
    odom_pub = rospy.Publisher('/global_odom', Odometry, queue_size=10) #Modified odometry wrt global frame

    while not rospy.is_shutdown():
        global_odom = odom_callback(odom_sub, (global_odom, tf_Buffer, parent_frame, child_frame))   
        odom_pub.publish(global_odom)
        rate.sleep()

        