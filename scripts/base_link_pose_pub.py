#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped

class TransformPublisher:
    def __init__(self):
        rospy.init_node('transform_publisher', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publisher for the stamped pose
        self.pose_pub = rospy.Publisher('base_link_pose', PoseStamped, queue_size=10)

        self.rate = rospy.Rate(10.0)  # 10 Hz

    def publish_transform_as_pose(self):
        while not rospy.is_shutdown():
            try:
                # Listen for the transform
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
                
                # Constructing the stamped pose
                pose = PoseStamped()
                pose.header.stamp = transform.header.stamp
                pose.header.frame_id = transform.header.frame_id
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation.x = transform.transform.rotation.x
                pose.pose.orientation.y = transform.transform.rotation.y
                pose.pose.orientation.z = transform.transform.rotation.z
                pose.pose.orientation.w = transform.transform.rotation.w
                
                # Publishing the stamped pose
                self.pose_pub.publish(pose)
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("Failed to get transform: %s", str(e))

            self.rate.sleep()
    
    def sleep(self):
        self.rate.sleep()

if __name__ == '__main__':
        transform_publisher = TransformPublisher()
        transform_publisher.publish_transform_as_pose()
        while not rospy.is_shutdown():
            try:
                transform_publisher.publish_transform_as_pose()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("Failed to get transform: %s", str(e))
            transform_publisher.sleep()
