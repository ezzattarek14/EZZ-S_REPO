#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from geometry_msgs.msg import Quaternion
import tf
import math

class EncoderToOdom:
    def __init__(self):
        rospy.init_node('encoder_to_odom')

        # Subscribe to encoder ticks
        rospy.Subscriber('/lwheel', Int16, self.lwheel_callback)
        rospy.Subscriber('/rwheel', Int16, self.rwheel_callback)

        # Publish odometry message
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # Robot parameters
        self.wheel_base = 0.5  # Distance between wheels in meters
        self.wheel_radius = 0.1  # Radius of the wheels in meters
        self.ticks_per_revolution = 500  # Encoder ticks per wheel revolution
        self.ticks_left = 0
        self.ticks_right = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = rospy.Time.now()

    def lwheel_callback(self, msg):
        self.ticks_left = msg.data

    def rwheel_callback(self, msg):
        self.ticks_right = msg.data

    def compute_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Calculate distance traveled by each wheel
        distance_left = (2 * math.pi * self.wheel_radius * self.ticks_left) / self.ticks_per_revolution
        distance_right = (2 * math.pi * self.wheel_radius * self.ticks_right) / self.ticks_per_revolution

        # Compute linear and angular velocity
        distance = (distance_right + distance_left) / 2.0
        delta_theta = (distance_right - distance_left) / self.wheel_base

        # Update position
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)
        self.theta += delta_theta

        # Prepare the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, self.theta))

        # Set the velocity
        odom.twist.twist.linear.x = distance / dt
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = delta_theta / dt

        # Publish the odometry message
        self.odom_pub.publish(odom)

        self.last_time = current_time

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.compute_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = EncoderToOdom()
        node.spin()
    except rospy.ROSInterruptException:
        pass

