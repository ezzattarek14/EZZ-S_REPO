#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

def publish_imu():
    rospy.init_node('imu_publisher', anonymous=True)
    imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = "imu_link"

        # Example orientation
        imu.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        # Example angular velocity
        imu.angular_velocity = Vector3(0.0, 0.0, 0.1)

        # Example linear acceleration
        imu.linear_acceleration = Vector3(0.0, 0.0, 0.0)

        imu_pub.publish(imu)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_imu()
    except rospy.ROSInterruptException:
        pass

