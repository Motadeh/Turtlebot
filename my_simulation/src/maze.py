#! /usr/bin/env python

import rospy

from threading import Thread, Lock
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan

from tf.transformations import euler_from_quaternion

from pid import PID

class MazeSolver:

    def __init__(self):
        rospy.init_node('MazeSolverNode')
        self.rate = rospy.Rate(4)

        rospy.Subscriber("/kobuki/laser/scan", LaserScan, self.laserscan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.velPub = rospy.Publisher('/cmd_vel', Twist, queue_size = 5)

        self.vel = Twist()
        self.laser = None
        self.odom = None

        # set up wall detection
        self.minLaserValues = 60            # amount of laserspoints that must be next to each other
        self.intervalEpsilon = 1.5          # upper- and lower-bound
        self.mutex2 = Lock()

        # set up loop detection
        self.knownPoints = []
        self.epsilonAroundPoints = 0.1     # circle around the saved points
        self.timeoutForDetection = 12       # delay in seconds

        # set up wall follower
        self.leftHand = True            # True, if the robot uses the left sensor for wall follow
        self.laserIndex = 359           # left laser index
        self.turnSpeed = 0.3
        self.distanceToWall = 0.6       # distance from the wall to the robot and min distance for obstacles detection
        self.angle = 1.50               # around 90 degree
        self.minLasersSide = [150,170]  # lasers used for basic obstacles detection
        self.mutex = Lock()

        self.goal = 1

        # set up PID
        kp = 6
        ki = 2
        kd = 1.5
        outMin = -0.4
        outMax = 0.4
        iMin = -0.4
        iMax = 0.4
        self.pid = PID(kp, ki, kd, outMin, outMax, iMin, iMax)

        # actual drive state (initial: WallDetection)
        self.driveState = "moveForward"

        self.move_forward = False

    def odom_callback(self, odom_msg):
        self.odom = odom_msg

    def laserscan_callback(self, laser_msg):
        self.laser = laser_msg

    def startSolver(self):
        rospy.loginfo("start Maze Solver Node")

        # if the robot uses the right sensor for wall follow
        if(not self.leftHand):
            self.laserIndex = 0
            self.minLasersSide = [190, 210]
            self.turnSpeed *= (-1)

        while not rospy.is_shutdown():
            if(self.laser and self.odom): # laser and odom data arrived from callback

                # append the origin to the known points
                if(len(self.knownPoints) == 0):
                    self.knownPoints.append([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, rospy.Time.now().to_sec()])

                # take the min laser value in front of the robot (simple obstacles detection)
                actMinLaserValue = min(min(self.laser.ranges[170:190], self.laser.ranges[self.minLasersSide[0] : self.minLasersSide[1]]))
                    
                if(self.goal == 1):
                    self.goToGoal([4,-3], actMinLaserValue)
                elif(self.goal == 2):
                    self.goToGoal([2,-7], actMinLaserValue)
                elif(self.goal > 2):
                    self.goToGoal([0,0], actMinLaserValue)
                    # rospy.loginfo(self.driveState)
                    # self.driveState = "WallDetection"
                    # if(self.driveState == "moveForward"):
                    #     self.vel.linear.x = 0.0
                    #     self.vel.angular.z = 0.0
                    #     self.wallDetection(actMinLaserValue)
                    # elif(self.driveState == "driveToWall"):
                    #     if(self.mutex.locked() == False):
                    #         # obstacle in front of the robot or wall arrived
                    #         if(actMinLaserValue <= self.distanceToWall):
                    #             # save the actual position used for loop detection
                    #             self.knownPoints.append([self.odom.pose.pose.position.x,self.odom.pose.pose.position.y, rospy.Time.now().to_sec()])

                    #             self.vel.linear.x = 0.0
                    #             self.vel.angular.z = 0.0
                    #             self.rotate_angle(self.angle, 0.3)
                    #             self.driveState = "WallFollow"
                    #         else:
                    #             self.vel.linear.x = 0.3
                    #             self.vel.angular.z = 0.0
                    # elif(self.driveState == "WallFollow"):
                    #     if(self.mutex.locked() == False):
                    #         # check known points (loop detection)
                    #         for i in range(0, len(self.knownPoints)):
                    #             if(self.odom.pose.pose.position.x - self.epsilonAroundPoints <= self.knownPoints[i][0] <= self.odom.pose.pose.position.x + self.epsilonAroundPoints and
                    #             self.odom.pose.pose.position.y - self.epsilonAroundPoints <= self.knownPoints[i][1] <= self.odom.pose.pose.position.y + self.epsilonAroundPoints and
                    #             self.knownPoints[i][2] + self.timeoutForDetection <  rospy.Time.now().to_sec()):
                    #                 rospy.loginfo("Loop detected")
                    #                 self.driveState = "moveForward"

                    #         self.wallFollower(actMinLaserValue)

            
                self.velPub.publish(self.vel)
            self.rate.sleep()

        rospy.spin()

    def goToGoal(self, goalPoint, actMinLaserValue):
        goal = Point()

        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        rot_q = self.odom.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        goal.x = goalPoint[0]
        goal.y = goalPoint[1]

        inc_x = goal.x - x                      #distance robot to goal in x
        inc_y = goal.y - y                      #distance robot to goal in y
        angle_to_goal = math.atan2 (inc_y, inc_x)    #calculate angle through distance from robot to goal in x and y

        dist = math.sqrt(pow(inc_x, 2) + pow(inc_y, 2))

        turn = math.atan2(math.sin(angle_to_goal-theta), math.cos(angle_to_goal-theta))

        rospy.loginfo(dist)

        if(actMinLaserValue < self.distanceToWall):
            self.knownPoints.append([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, rospy.Time.now().to_sec()])

            self.rotate_angle(self.angle, self.turnSpeed)

        elif(self.driveState == "moveForward" and self.goal < 3):
            pidValue = self.pid.pidExecute(self.distanceToWall, self.laser.ranges[self.laserIndex])

            if abs(angle_to_goal - theta) < 0.1:
                self.move_forward = True

            if(pidValue == 0):
                self.vel.angular.z = 0.2 * turn
                if self.move_forward == True:
                    if 0.1 * dist > 0.1 and 0.1 * dist < 0.3:
                        self.vel.linear.x = 0.5 * dist
                    elif 0.1 * dist > 0.3:
                        self.vel.linear.x = 0.7
                    else:
                        self.vel.angular.z = 0.0
                        rospy.loginfo('a ti dey')
                        self.goal += 1
            elif(pidValue != 0):
                self.vel.angular.z = pidValue
                if self.move_forward == True:
                    if 0.1 * dist > 0.1 and 0.1 * dist < 0.3:
                        self.vel.linear.x = 0.05 * dist
                    elif 0.1 * dist > 0.3:
                        self.vel.linear.x = 0.25
                    else:
                        self.vel.angular.z = 0.0
                        self.vel.linear.x = 0.0
                        self.velPub.publish(self.vel)
                        rospy.loginfo('a ti dey')
                        self.goal += 1

            for i in range(0, len(self.knownPoints)):
                if(self.odom.pose.pose.position.x - self.epsilonAroundPoints <= self.knownPoints[i][0] <= self.odom.pose.pose.position.x + self.epsilonAroundPoints and
                self.odom.pose.pose.position.y - self.epsilonAroundPoints <= self.knownPoints[i][1] <= self.odom.pose.pose.position.y + self.epsilonAroundPoints and
                self.knownPoints[i][2] + self.timeoutForDetection <  rospy.Time.now().to_sec()):
                    rospy.loginfo("Loop detected")
                    
                    self.driveState = "redirect"
                    
        elif(self.driveState == "redirect"):
            rospy.loginfo('its here')
            self.turnSpeed *= -1
            
            self.vel.linear.x = 0.3
            self.vel.angular.z = -2 * self.turnSpeed
            self.velPub.publish(self.vel)
            self.driveState = "moveForward"

    # def reset():


    def wallDetection(self, actMinLaserValue):
        self.mutex2.acquire()
        try:
            if(actMinLaserValue < self.distanceToWall):
                self.rotate_angle(self.angle, self.turnSpeed)
            else:
                # search for a wall to follow
                arrValues = []
                i = 0
                # do wall detection
                
                while(i < len(self.laser.ranges)):
                    valCounter = 0
                    for k in range(i+1, len(self.laser.ranges)):
                        # check if actual value is in interval
                        if(self.laser.ranges[i] - self.intervalEpsilon) <= self.laser.ranges[k] <= (self.laser.ranges[i] + self.intervalEpsilon):
                            valCounter += 1
                        else:
                            if(valCounter > self.minLaserValues):
                                arrValues.append([self.laser.ranges[i], i, k])
                            valCounter = 0
                            i = k
                        if(k == len(self.laser.ranges) - 1):
                            i = len(self.laser.ranges)
                
                rospy.loginfo(arrValues)

                # turn the robot to the wall (use the wall with the max distance to the robot)
                getLaserIndex = 180;
                if(arrValues):
                    rospy.loginfo(int(math.floor((max(arrValues)[1] + max(arrValues)[2]) / 2)))
                    getLaserIndex = int(math.floor((max(arrValues)[1] + max(arrValues)[2]) / 2))
                if(getLaserIndex <= 175 or getLaserIndex >= 185):
                    tmpAngle = self.laser.angle_min + (getLaserIndex * self.laser.angle_increment)
                    rospy.loginfo(self.laser.angle_min)
                    rospy.loginfo(self.laser.angle_increment)
                    if(getLaserIndex < 170): # rotation to the right
                        self.rotate_angle(abs(tmpAngle), -0.3)
                    else:   # rotation to the left
                        self.rotate_angle(abs(tmpAngle), 0.3)

                # wait until the robot has stopped the rotation
                rospy.sleep(1.5)
                self.driveState = "driveToWall"
        finally:
            self.mutex2.release()

    def wallFollower(self, actMinLaserValue):
        pidValue = self.pid.pidExecute(self.distanceToWall, self.laser.ranges[self.laserIndex])

        if(self.leftHand):
            pidValue = pidValue * (-1)

        # obstacle in front of the robot
        if(actMinLaserValue < self.distanceToWall):
            self.rotate_angle(self.angle, -0.3)
        elif(pidValue == 0):
            self.vel.linear.x = 0.3
            self.vel.angular.z = 0.0
        elif(pidValue != 0):
            if(self.laser.ranges[359] <= self.distanceToWall):

                rospy.loginfo('test')
                self.vel.linear.x = 0.3
                self.vel.angular.z = 0.0

            else:
                self.vel.linear.x = 0.3
                self.vel.angular.z = pidValue
            # rospy.loginfo(self.laser.ranges[719])
            # if((self.laser.ranges[719] or self.laser.ranges[359] or self.laser.ranges[0]) <= self.distanceToWall):
            #     self.vel.linear.x = 0.3
            #     self.vel.angular.z = 0.0        
                
            # else: 
                
            #     while(actMinLaserValue > self.distanceToWall):
            #     if(self.laserIndex == 359):
            #         rospy.loginfo(self.laserIndex)
            #         rospy.loginfo(self.laser.ranges[self.laserIndex])
            #         rospy.loginfo(actMinLaserValue)
            #         if(self.laser.ranges[719]):
            #             self.vel.linear.x = 0.15
            #             self.vel.angular.z = pidValue
            #             self.velPub.publish(self.vel)
            # # self.pid.resetValues()
            #         self.vel.linear.x = 0.0
            #         self.vel.angular.z = 0
            #         self.velPub.publish(self.vel)

    # simple rotation function by the relativ angle in rad
    def rotate_angle(self, angle, speed):
        self.mutex.acquire()
        try:
            angular_speed = speed
            relative_angle = angle
            self.vel.linear.x = 0.0
            self.vel.angular.z = angular_speed

            # setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_angle = 0

            while(current_angle < relative_angle):
                self.velPub.publish(self.vel)
                t1 = rospy.Time.now().to_sec()
                current_angle = abs(angular_speed)*(t1-t0)

            # forcing the robot to stop after rotation
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0
            self.velPub.publish(self.vel)
            # fixes issue #6 (Weird Driving at Wall Following)
            self.pid.resetValues()
        finally:
            self.mutex.release()

if __name__ == "__main__":
        maze_solver = MazeSolver()
        maze_solver.startSolver()