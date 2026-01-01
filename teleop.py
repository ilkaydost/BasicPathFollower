#!/usr/bin/env python
##################################
# Basic Path follower of a tricycle robot in ROS and Gazebo
# Author: Ilkay Dost
# Date: January 2021 Created - Implemented
# Updated: January 2026 - Published on GitHub
# Description: This code makes a tricycle robot to follow a path defined by user input coordinates.
# The path is interpolated and the robot moves through the points using a simple proportional controller.
# The robot's odometry is used to determine its current position and orientation.
# The robot adjusts its steering angle and speed to reach each point in the path.
# The code uses ROS topics to publish velocity commands and subscribe to odometry data.
# The robot stops when it reaches the final point in the path within a specified tolerance.
# The code also visualizes the path using matplotlib.
# To run this code, ensure you have a ROS environment set up with a tricycle robot model in Gazebo.
##################################

#Inspiration video: https://www.youtube.com/watch?v=Qh15Nol5htM
#Youtube Link result: https://www.youtube.com/watch?v=FR37Owa-Jxk

###############################
# Importing the libraries
###############################

import rospy
from geometry_msgs.msg import Twist, Pose2D, PoseWithCovarianceStamped, PoseArray ,PointStamped, Point
from math import pow, atan2, sqrt, atan
from nav_msgs.msg import Odometry, MapMetaData
from sensor_msgs.msg import JointState 
from nav_msgs.msg import GetMapActionGoal
import roslib
from tf.transformations import quaternion_from_euler
#from gazebo_msgs import GetModelState
from tf.transformations import euler_from_quaternion
import numpy as np
import  matplotlib.pyplot as plt




class Teleoperation:

    def __init__(self):
        # unique node (using anonymous=True).
        rospy.init_node('teleop', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/tricycle/cmd_vel', Twist, queue_size=1)
        rospy.loginfo("Velocity publisher construct")
        self.joint_subscriber = rospy.Subscriber('/tricycle/odom', Odometry, self.newOdom)        
        rospy.loginfo("Odometryr publisher construct")
        

        self.pose = Odometry().pose.pose
        self.goal_pose = Point()
        self.counter = 0
        self.path_list = []
        self.goal_x = []
        self.goal_y = []
        self.rate = rospy.Rate(5)
        self.max_x = 0
   
    def newOdom(self, msg):
        global theta
        
        self.pose = msg.pose.pose.position
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def path(self):
        
        i = 0
        x = int(input("How many time you will get coordinate: "))
        for i in range (x):
            self.goal_pose.x = float(input("Set your x goal: "))
            self.goal_x.append(self.goal_pose.x)
            self.goal_pose.y = float(input("Set your y goal: "))
            self.goal_y.append(self.goal_pose.y)

        #self.max_x = max(self.goal_pose.x)
        #xvals = np.linspace(0,self.max_x,8)
        xvals = np.linspace(0,max(self.goal_x),9)
        yinterpolation = np.interp(xvals,self.goal_x,self.goal_y)


        goal_list = list(zip(self.goal_x, self.goal_y))
        self.path_list = list(zip(xvals,yinterpolation))
        rospy.loginfo(self.path_list)

        plt.plot(self.goal_x, self.goal_y, 'o')
        plt.plot(xvals, yinterpolation, '-x')
        plt.show()
        
    def euclidean_distance(self):
        #Euclidean distance between current pose and the goal
        if self.counter == 0:
            return sqrt(pow((self.path_list[0][0] - self.pose.x), 2) + pow((self.path_list[0][1] - self.pose.y), 2))
        elif self.counter == 1:
            return sqrt(pow((self.path_list[1][0] - self.pose.x), 2) + pow((self.path_list[1][1] - self.pose.y), 2))
        elif self.counter == 2:
            return sqrt(pow((self.path_list[2][0] - self.pose.x), 2) + pow((self.path_list[2][1] - self.pose.y), 2))
        elif self.counter == 3:
            return sqrt(pow((self.path_list[3][0] - self.pose.x), 2) + pow((self.path_list[3][1] - self.pose.y), 2))
        elif self.counter == 4:
            return sqrt(pow((self.path_list[4][0] - self.pose.x), 2) + pow((self.path_list[4][1] - self.pose.y), 2))
        elif self.counter == 5:
            return sqrt(pow((self.path_list[5][0] - self.pose.x), 2) + pow((self.path_list[5][1] - self.pose.y), 2))
        elif self.counter == 6:
            return sqrt(pow((self.path_list[6][0] - self.pose.x), 2) + pow((self.path_list[6][1] - self.pose.y), 2))
        elif self.counter == 7:
            return sqrt(pow((self.path_list[7][0] - self.pose.x), 2) + pow((self.path_list[7][1] - self.pose.y), 2))
        elif self.counter == 8:
            return sqrt(pow((self.path_list[8][0] - self.pose.x), 2) + pow((self.path_list[8][1] - self.pose.y), 2))
        elif self.counter == 9:
            return sqrt(pow((self.path_list[9][0] - self.pose.x), 2) + pow((self.path_list[9][1] - self.pose.y), 2))

    def steering_angle(self):
        #rospy.loginfo("STEERING DEGREE: %.4f", 57.3*atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x))
        #return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
        if self.counter == 0:
            return atan2(self.path_list[0][1]  - self.pose.y, self.path_list[0][0] - self.pose.x)
            #return atan2(self.path_list[counter][ 1]  - self.pose.y, self.path_list[counter][counter] - self.pose.x)
        elif self.counter == 1:
            return atan2(self.path_list[1][1]  - self.pose.y, self.path_list[1][0] - self.pose.x)
        elif self.counter == 2:
            return atan2(self.path_list[2][1]  - self.pose.y, self.path_list[2][0] - self.pose.x)
        elif self.counter == 3:
            return atan2(self.path_list[3][1]  - self.pose.y, self.path_list[3][0] - self.pose.x)
        elif self.counter == 4:
            return atan2(self.path_list[4][1]  - self.pose.y, self.path_list[4][0] - self.pose.x)
        elif self.counter == 5:
            return atan2(self.path_list[5][1]  - self.pose.y, self.path_list[5][0] - self.pose.x)
        elif self.counter == 6:
            return atan2(self.path_list[6][1]  - self.pose.y, self.path_list[6][0] - self.pose.x)
        elif self.counter == 7:
            return atan2(self.path_list[7][1]  - self.pose.y, self.path_list[7][0] - self.pose.x)
        elif self.counter == 8:
            return atan2(self.path_list[8][1]  - self.pose.y, self.path_list[8][0] - self.pose.x)
        elif self.counter == 9:
            return atan2(self.path_list[9][1]  - self.pose.y, self.path_list[9][0] - self.pose.x)


    def move2goal(self):
        
        
        self.path()
        
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = input("Set your tolerance: ")

        vel_msg = Twist()
        #self.heading = self.steering_angle(goal_pose) - theta

        for i in range(9):

            while abs(self.euclidean_distance() >= distance_tolerance):
                
                rospy.loginfo("Angle: %.3f", 57.3 * self.steering_angle())
                rospy.loginfo("Theta: %.3f", theta*57.3)
                rospy.loginfo("Steering Angle - theta: %.5f", 57.3 * self.steering_angle() - theta*57.3)

                if ( self.steering_angle()*57.3  - 57.3 * theta < -1.0 and self.steering_angle()*57.3  - 57.3* theta >= -90):
                    # Linear velocity in the x-axis.
                    rospy.loginfo("Turn right")

                    vel_msg.linear.x = 0.02
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0

                    # Angular velocity in the z-axis.
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = self.steering_angle()
                    # Publishing our vel_msg
                    self.velocity_publisher.publish(vel_msg)

                    # Publish at the desired rate.
                    self.rate.sleep()
                
                elif ( (self.steering_angle()*57.3 - theta*57.3 > 1.0 ) and (self.steering_angle()*57.3 - theta*57.3 <= 90)):
                    rospy.loginfo("Turn left")

                    vel_msg.linear.x = 0.02
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0

                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = -self.steering_angle()

                    self.velocity_publisher.publish(vel_msg)

                    self.rate.sleep()
                
                
                elif ((self.steering_angle()*57.3 - theta*57.3 <= 1) and (self.steering_angle()*57.3 - theta*57.3 >= -1)):
                    rospy.loginfo("Move forward")
                    
                    vel_msg.linear.x = 0.15
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0

                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = 0
                    
                    self.velocity_publisher.publish(vel_msg)

                    self.rate.sleep()
                
                elif (self.steering_angle()*57.3  - 57.3* theta < -90 and self.steering_angle()*57.3  - 57.3* theta >= -180):
                    
                    rospy.loginfo("Turn left/counterclockwise")

                    vel_msg.linear.x = 0.02
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0

                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = -self.steering_angle()

                    self.velocity_publisher.publish(vel_msg)

                    self.rate.sleep()

                elif (self.steering_angle()*57.3  - 57.3* theta > 90 and self.steering_angle()*57.3  - 57.3* theta < 180):
                
                    rospy.loginfo("Turn right/counterclockwise")

                    vel_msg.linear.x = 0.02
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0

                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = self.steering_angle()

                    self.velocity_publisher.publish(vel_msg)

                    self.rate.sleep()

            self.counter += 1
            rospy.loginfo("Counter: %d", self.counter)


        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("######################################")
        rospy.loginfo("#############Path Completed###########")
        rospy.loginfo("######################################")

        """
        if (vel_msg.linear.x ==0):
            a = Teleoperation()
            a.move2goal()
        """

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = Teleoperation()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
