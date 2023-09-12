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
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.trigger_sub = rospy.Subscriber('trigger', Bool, self.trigger_callback)
        self.odom_pub = rospy.Publisher('global_odom', Odometry, queue_size=10)
        self.tf_ready = False
        self.pose_offset_method = rospy.get_param('~pose_offset_method', 'tf')
        self.pose_topic = rospy.get_param('~pose_topic', '/pose_topic')
        self.position_offset = rospy.get_param('~position_offset', [0.0, 0.0, 0.0])
        self.rotation_format = rospy.get_param('~rotation_format', 'euler')  # Options: 'euler' or 'quaternion'
        self.rotation_offset = rospy.get_param('~rotation_offset', [0.0, 0.0, 0.0])  # Euler angles by default

    def odom_callback(self, odom_msg):
        try:
            transform = self.tf_buffer.lookup_transform(self.parent_frame, self.child_frame, rospy.Time(0))
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            global_odom = Odometry()
            global_odom.header = odom_msg.header
            global_odom.child_frame_id = self.parent_frame
            global_odom.pose.pose.position = translation
            global_odom.pose.pose.orientation = rotation
            global_odom.twist = odom_msg.twist

            if self.pose_offset_method == 'tf':
                global_odom.pose.pose.position.x += translation.x
                global_odom.pose.pose.position.y += translation.y
                global_odom.pose.pose.position.z += translation.z

            # Apply position offset if provided as a tuple
            global_odom.pose.pose.position.x += self.position_offset[0]
            global_odom.pose.pose.position.y += self.position_offset[1]
            global_odom.pose.pose.position.z += self.position_offset[2]

            # Apply rotation offset based on the specified format
            if self.rotation_format == 'euler':
                self.apply_rotation_offset_euler(global_odom, *self.rotation_offset)
            elif self.rotation_format == 'quaternion':
                self.apply_rotation_offset_quaternion(global_odom, *self.rotation_offset)

            self.odom_pub.publish(global_odom)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup failed")

    def trigger_callback(self, trigger_msg):
        if trigger_msg.data:
            self.tf_ready = True
        else:
            self.tf_ready = False

    def apply_rotation_offset_euler(self, odom_msg, roll, pitch, yaw):
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

    def apply_rotation_offset_quaternion(self, odom_msg, x, y, z, w):
        odom_msg.pose.pose.orientation.x += x
        odom_msg.pose.pose.orientation.y += y
        odom_msg.pose.pose.orientation.z += z
        odom_msg.pose.pose.orientation.w += w

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.tf_ready:
                rospy.loginfo("Transforms are ready. Listening to odometry...")
                rate.sleep()
            else:
                rospy.loginfo("Waiting for trigger...")
                rate.sleep()

if __name__ == '__main__':
    try:
        odometry_transformer = OdometryTransformer()
        odometry_transformer.run()
    except rospy.ROSInterruptException:
        pass
