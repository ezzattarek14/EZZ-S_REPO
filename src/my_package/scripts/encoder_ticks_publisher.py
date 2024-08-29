#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def encoder_ticks_publisher():
    rospy.init_node('encoder_ticks_publisher', anonymous=True)

    # Create publishers for lwheel and rwheel topics
    lwheel_pub = rospy.Publisher('/lwheel', Int16, queue_size=10)
    rwheel_pub = rospy.Publisher('/rwheel', Int16, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    # Example: Start with initial tick counts
    left_ticks = 0
    right_ticks = 0

    while not rospy.is_shutdown():
        # Simulate the ticks increment (or replace this with your actual logic)
        left_ticks += 1
        right_ticks += 1

        # Publish the tick data
        lwheel_pub.publish(left_ticks)
        rwheel_pub.publish(right_ticks)

        # Log the tick data using the format() method
        rospy.loginfo("Left wheel ticks: {}".format(left_ticks))
        rospy.loginfo("Right wheel ticks: {}".format(right_ticks))

        rate.sleep()

if __name__ == '__main__':
    try:
        encoder_ticks_publisher()
    except rospy.ROSInterruptException:
        pass

