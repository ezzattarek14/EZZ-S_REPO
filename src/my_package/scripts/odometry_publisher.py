#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance

def publish_odometry():
    rospy.init_node('odometry_publisher', anonymous=True)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Example pose
        odom.pose.pose.position.x = 1.0
        odom.pose.pose.position.y = 2.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        # Example twist
        odom.twist.twist.linear.x = 0.5
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.1

        odom_pub.publish(odom)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_odometry()
    except rospy.ROSInterruptException:
        pass

