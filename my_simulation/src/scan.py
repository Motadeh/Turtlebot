#!/usr/bin/env python
# from math import pi

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def callback(msg):
    # values at 90 degrees
    print(msg.ranges[0])
    # values at 0 degrees
    print(msg.ranges[359])
    # values at -90 degrees
    print(msg.ranges[719])

    if msg.ranges[359] > 0.5:
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        twist = Twist()

        twist.linear.x = 0.3

        pub.publish(twist)
    else:
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # duration = 10

        twist = Twist()

        twist.angular.z = 0.04

        # twist.linear.x = 0

        pub.publish(twist)
        rospy.Rate(3).sleep()

        twist.angular.z = 0
        pub.publish(twist)

        twist.linear.x = 0.3

        pub.publish(twist)


def wall_detector():
    rospy.init_node('wall_detection')

    rospy.Subscriber("/kobuki/laser/scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped with ctrl+c
    rospy.spin()


if __name__ == '__main__':
    wall_detector()
