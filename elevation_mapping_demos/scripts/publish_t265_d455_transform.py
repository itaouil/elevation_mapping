#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
import tf2_geometry_msgs
import tf.transformations
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped


def odom_callback(msg):
    br = tf.TransformBroadcaster()

    # 45 degree rotation around the pitch axis
    # and 90 degree rotation around the roll axis
    q_rot_pitch_45 = tf.transformations.quaternion_from_euler(0, 0.785398, 0)
    q_rot_roll_90 = tf.transformations.quaternion_from_euler(1.5708, 0, 0)

    # D455 rotation w.r.t t265 odometry
    optical_to_world_numpy_rot = tf.transformations.quaternion_matrix([msg.pose.pose.orientation.x,
                                                                       msg.pose.pose.orientation.y,
                                                                       msg.pose.pose.orientation.z,
                                                                       msg.pose.pose.orientation.w])
    optical_to_world_numpy_rot = tf.transformations.quaternion_multiply(optical_to_world_numpy_rot, q_rot_pitch_45)
    optical_to_world_numpy_rot = tf.transformations.quaternion_multiply(optical_to_world_numpy_rot, q_rot_roll_90)

    # D455 translation w.r.t t265 odometry
    optical_to_world_numpy_trans = tf.transformations.translation_matrix([msg.pose.pose.position.x,
                                                                          msg.pose.pose.position.y,
                                                                          msg.pose.pose.position.z + 1.6])
    optical_to_world_numpy = np.dot(optical_to_world_numpy_trans, optical_to_world_numpy_rot)

    # Publish transform
    br.sendTransform(tf.transformations.translation_from_matrix(optical_to_world_numpy_trans),
                     tf.transformations.quaternion_from_matrix(optical_to_world_numpy_rot),
                     rospy.Time.now(),
                     "d455_link",
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