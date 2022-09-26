#!/usr/bin/env python3

# Imports
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


def odom_callback(msg, arg):
    publisher = arg

    # New PWCS message
    new_message = PoseWithCovarianceStamped()
    new_message.header = msg.header
    new_message.pose.pose.position = msg.pose.pose.position

    # Publish message
    publisher.publish(new_message)


if __name__ == '__main__':
    rospy.init_node('odom_to_pose')

    topic_name = rospy.get_param('~from_frame')
    publisher_name = rospy.get_param('~to_frame')

    publisher = rospy.Publisher(publisher_name, PoseWithCovarianceStamped, queue_size=1)

    if topic_name == '' or publisher_name == '':
        rospy.logerr("Could not get subscriber's topic name or publisher's topics name. Exiting...")
    else:
        rospy.Subscriber(topic_name, Odometry, odom_callback, publisher)
        rospy.spin()