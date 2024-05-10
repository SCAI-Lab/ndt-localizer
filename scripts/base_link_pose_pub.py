#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from threading import Lock

class TransformPublisher:
    def __init__(self):
        rospy.init_node('base_link_pose_pub_node', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_link_frame = rospy.get_param('~base_link_frame', 'base_link')

        self.pub_ndt_tf = rospy.get_param('~pub_ndt_tf', False)


        # Publisher for the stamped pose
        self.pose_pub = rospy.Publisher('base_link_pose', PoseStamped, queue_size=10)

        if self.pub_ndt_tf:
            self._ndt_pose_sub = rospy.Subscriber('/ndt_pose', PoseStamped, self.ndt_callback)

        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.rate = rospy.Rate(10.0)  # 10 Hz

        self.pose_mutex = Lock()

        self.ndt_pose = None

    def ndt_callback(self, data):
        with self.pose_mutex:
            self.ndt_pose = data

    def publish_transform_as_pose(self):
        while not rospy.is_shutdown():
            try:
                # Listen for the transform
                transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_link_frame, rospy.Time())
                
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

            with self.pose_mutex:
                if self.ndt_pose:
                    transform = TransformStamped()
                    transform.header = self.ndt_pose.header
                    transform.header.stamp = rospy.Time.now()
                    transform.header.frame_id = "map"
                    transform.child_frame_id = "camera_init"
                    transform.transform.translation = self.ndt_pose.pose.position
                    transform.transform.rotation = self.ndt_pose.pose.orientation
                    self.broadcaster.sendTransform(transform)

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
