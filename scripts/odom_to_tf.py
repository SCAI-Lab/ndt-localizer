#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs  # To convert Pose to TransformStamped

def callback(msg):
    
    # Create a TransformStamped message to be published
    transform = TransformStamped()
    
    transform.header = msg.header
    transform.header.frame_id = "camera_init"
    transform.child_frame_id = "livox"
    
    transform.transform.translation = msg.pose.pose.position
    transform.transform.rotation = msg.pose.pose.orientation

    # Send the transform
    broadcaster.sendTransform(transform)

if __name__ == "__main__":
    rospy.init_node('odom_to_tf_publisher')

    # Create a tf2 broadcaster
    broadcaster = tf2_ros.TransformBroadcaster()

    # Subscribe to the odom topic
    rospy.Subscriber('/Odometry', Odometry, callback)

    rospy.spin()
