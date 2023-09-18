#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class OdometryTransformer:
    def __init__(self):
        rospy.init_node('odometry_transformer', anonymous=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.child_frame = rospy.get_param('~child_frame', 'child_frame')
        self.parent_frame = rospy.get_param('~parent_frame', 'parent_frame')
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback) #Original odometry wrt local frame
        self.odom_pub = rospy.Publisher('/global_odom', Odometry, queue_size=10) #Modified odometry wrt global frame

        # self.trigger_sub = rospy.Subscriber('trigger', Bool, self.trigger_callback) #Listening to a bool trigger when an event occurs (e.g. 2nd fl is reached)
        # self.event_is_true = False
        # self.event_odom_sub = rospy.Subscriber('event_odom', Odometry, self.event_odom_callback)
        # # self.pose_offset_method = rospy.get_param('~pose_offset_method', 'tf')
        # self.pose_topic = rospy.get_param('~pose_topic', '/pose_topic')
        # self.position_offset = rospy.get_param('~position_offset', [0.0, 0.0, 0.0])
        # self.rotation_format = rospy.get_param('~rotation_format', 'euler')  # Options: 'euler' or 'quaternion'
        # self.rotation_offset_euler = rospy.get_param('~rotation_offset_euler', [0.0, 0.0, 0.0])  # Euler angles by default
        # self.rotation_offset_quaternion = rospy.get_param('~rotation_offset_quaternion', [0.0, 0.0, 0.0, 0.0])  

        

    def odom_callback(self, odom_msg):
        try:
            transform = self.tf_buffer.lookup_transform(self.parent_frame, self.child_frame, rospy.Time(0))
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            print([translation, rotation])

            
            global_odom.header = odom_msg.header
            global_odom.child_frame_id = self.parent_frame
            global_odom.pose.pose.position = odom_msg.pose.pose.position
            global_odom.pose.pose.orientation = odom_msg.pose.pose.orientation
            #global_odom.pose.pose.position = translation
            #global_odom.pose.pose.orientation = rotation
            global_odom.twist = odom_msg.twist

            # Add tf transfrom to the original odom
            global_odom.pose.pose.position.x += translation.x
            global_odom.pose.pose.position.y += translation.y
            global_odom.pose.pose.position.z += translation.z
            global_odom.pose.pose.orientation.x += rotation.x
            global_odom.pose.pose.orientation.y += rotation.y
            global_odom.pose.pose.orientation.z += rotation.z
            global_odom.pose.pose.orientation.w += rotation.w

            # # Apply position offset if provided as a tuple
            # global_odom.pose.pose.position.x += self.position_offset[0]
            # global_odom.pose.pose.position.y += self.position_offset[1]
            # global_odom.pose.pose.position.z += self.position_offset[2]

            # # Apply rotation offset based on the specified format
            # if self.rotation_format == 'euler':
            #     self.apply_rotation_offset_euler(global_odom, *self.rotation_offset_euler)
            # elif self.rotation_format == 'quaternion':
            #     self.apply_rotation_offset_quaternion(global_odom, *self.rotation_offset_quaternion)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup failed")

    def trigger_callback(self, trigger_msg):
        if trigger_msg.data:
            self.event_is_true = True
            
        else:
            self.event_is_true = False

    def apply_rotation_offset_euler(self, msg, roll, pitch, yaw):
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        msg.pose.pose.orientation.x += quaternion[0]
        msg.pose.pose.orientation.y += quaternion[1]
        msg.pose.pose.orientation.z += quaternion[2]
        msg.pose.pose.orientation.w += quaternion[3]

    def apply_rotation_offset_quaternion(self, msg, x, y, z, w):
        msg.pose.pose.orientation.x += x
        msg.pose.pose.orientation.y += y
        msg.pose.pose.orientation.z += z
        msg.pose.pose.orientation.w += w

    def event_odom_callback(self, message):
        rospy.loginfo("Event is triggered. Adding position.z offset.")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            
            # if self.event_is_true:
                
            # else:
            #     rospy.loginfo("Waiting for trigger...")
            global_odom = Odometry()
                
            self.odom_pub.publish(global_odom)
            rate.sleep()

if __name__ == '__main__':
    try:
        odometry_transformer = OdometryTransformer()
        odometry_transformer.run()
    except rospy.ROSInterruptException:
        pass
