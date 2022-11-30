#! /usr/bin/env python

import rospy

from threading import Thread, Lock
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from pid import PID

class MazeSolver:

    def __init__(self):
        rospy.init_node('MazeSolverNode')
        self.rate = rospy.Rate(10)

        rospy.Subscriber("/kobuki/laser/scan", LaserScan, self.laserscan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.velPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.vel = Twist()
        self.laser = None
        self.odom = None

    def odom_callback(self, odom_msg):
        self.odom = odom_msg

    def laserscan_callback(self, laser_msg):
        self.laser = laser_msg
        # self.vel.linear.x = 0.3
        # self.velPub.publish(self.vel)

        
    def startSolver(self):

        rospy.loginfo("start Maze Solver Node")

        while not rospy.is_shutdown():
            if (self.laser and self.odom):
                # rospy.loginfo(self.odom)
                self.vel.angular.z = 0.3
                self.velPub.publish(self.vel)


        # if the robot uses the right sensor for wall follow
        # if(not self.leftHand):
        #     self.laserIndex = 0
        #     self.minLasersSide = [190, 210]
        #     self.turnSpeed *= (-1)

        # while not rospy.is_shutdown():
        #     if(self.laser and self.odom): # laser and odom data arrived from callback

            #     self.velPub.publish(self.vel)

            # self.rate.sleep()
        rospy.spin()


if __name__ == "__main__":
        maze_solver = MazeSolver()
        maze_solver.startSolver()