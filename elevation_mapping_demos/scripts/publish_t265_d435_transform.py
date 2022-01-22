#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
import tf2_geometry_msgs
import tf.transformations
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped


def odom_callback(msg):
    br = tf.TransformBroadcaster()

    # Create 4x4 numpy optical to world transformation
    optical_to_world_numpy_rot = tf.transformations.quaternion_matrix([msg.pose.pose.orientation.x,
                                                                       msg.pose.pose.orientation.y,
                                                                       msg.pose.pose.orientation.z,
                                                                       msg.pose.pose.orientation.w])
    optical_to_world_numpy_trans = tf.transformations.translation_matrix([msg.pose.pose.position.x,
                                                                          msg.pose.pose.position.y,
                                                                          msg.pose.pose.position.z - 0.3012])
    optical_to_world_numpy = np.dot(optical_to_world_numpy_trans, optical_to_world_numpy_rot)

    # Publish transform
    br.sendTransform(tf.transformations.translation_from_matrix(link_to_world),
                     tf.transformations.quaternion_from_matrix(link_to_world),
                     rospy.Time.now(),
                     "d435_link",
                     "t265_odom_frame")
                     

def main():
    # Create ROS node
    rospy.init_node('tf_publisher')

    # Subscribe to VIO odom topic
    rospy.Subscriber('/pose', PoseWithCovarianceStamped, odom_callback)

    # Spin it
    rospy.spin()


if __name__ == "__main__":
    main()