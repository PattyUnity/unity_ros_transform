#!/usr/bin/env python

import rospy
import std_msgs
from std_msgs.msg import Bool 
import geometry_msgs
import nav_msgs
from nav_msgs.msg import Odometry
import tf2_ros
import threading

def done_climbing_stair():
    global z_offset_floor, new_floor_reached
    if (new_floor_reached):
        rate2 = rospy.Rate(5)
        num_messages_to_collect = 10
        collected_transforms = []
        
        for _ in range(num_messages_to_collect):
            try:
                trans_floor = tfBuffer.lookup_transform(parent_frame, child_frame, rospy.Time())
                collected_transforms.append(trans_floor.transform.translation.x) #must be .z but change to .y to debug
            except tf2.LookupException as e:
                rospy.logwarn("Robot Transform lookup for Floor offset failed: {}".format(e))
            rate2.sleep()

        if len(collected_transforms)==num_messages_to_collect:
            z_offset_floor = sum(collected_transforms) / len(collected_transforms)
            print("Successfully determined floor height = " + z_offset_floor)


def check_climbing_history(booldata):
    global new_floor_reached, floor_reached_timer
    consecutive_num = 10
    gold_standard_history = [True]*consecutive_num + [False]*consecutive_num
    bool_history = []

    for _ in range(consecutive_num*2):
        bool_history.append(booldata.data)

    if are_arrays_equal(bool_history, gold_standard_history):
        if not new_floor_reached:
            new_floor_reached = True
            rospy.logdebug("new_floor_reached is TRUE!!!!")
            floor_reached_timer = rospy.Time.now()

def are_arrays_equal(arr1, arr2):
    if len(arr1) != len(arr2):
        return False  # Arrays have different lengths, so they can't be equal
    
    for i in range(len(arr1)):
        if arr1[i] != arr2[i]:
            return False  # Mismatch found at index i
    
    return True  # All elements match

def check_floor_reached_thread():
    global new_floor_reached
    while not rospy.is_shutdown():
        if new_floor_reached and rospy.Time.now() - floor_reached_timer > rospy.Duration(600):  # 10 minutes = 600 seconds
            new_floor_reached = False
            rospy.logwarn("new_floor_reached status is reset")
        rospy.sleep(1)  # Sleep for 1 second to reduce CPU usage

if __name__ == "__main__":
    rospy.init_node('tf2_listener')
    parent_frame = rospy.get_param("~parent_frame", "map")
    child_frame = rospy.get_param("~child_frame","vision_1/odom")
    z_offset_human = rospy.get_param("~z_offset_human", 0)
    z_offset_floor = rospy.get_param("~z_offset_floor", 0)


    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    odom_pub = rospy.Publisher("global_odom",Odometry, queue_size=50)  
    trans = Odometry()

    rospy.Subscriber("is_climbing",Bool,check_climbing_history)
    new_floor_reached = False
    floor_reached_timer = None

    # Create and start a separate thread for checking floor reached
    floor_reached_thread = threading.Thread(target=check_floor_reached_thread)
    floor_reached_thread.daemon = True  # Allow the thread to exit when the main program exits
    floor_reached_thread.start()

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
        msg.pose.pose.position.z = trans.transform.translation.z + z_offset_human + z_offset_floor
        msg.pose.pose.orientation = trans.transform.rotation

        odom_pub.publish(msg)

        rate.sleep()