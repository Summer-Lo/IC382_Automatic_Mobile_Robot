#!/usr/bin/env python3

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
import pid_controller
import time

pi = 3.14

class robot():
    def __init__(self):
        rospy.init_node("pid_controller")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)                    # cmd/vel
        self.rate = rospy.Rate(10)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("initialpose", Empty, queue_size=10)     # odom/reset
        for i in range(10):
            self.reset_pub.publish(Empty())                                                               # publish empty message for reseting the odom
            self.rate.sleep()
        
        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
                # setpoint Config 
        # ---------------<START>---------------
        self.target = Pose2D()
        self.target1 = Pose2D()
        self.target2 = Pose2D()
        self.target3 = Pose2D()
        self.target.x = 3
        self.target.y = 3
        self.target.theta = 1.5707963268
        self.target1.x = 3.0
        self.target2.theta = 1.5707963268
        self.target3.x = 3.0
        self.msg = Empty()
        # ---------------<END>---------------
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')
        # PID controller config
    
    def shutdown(self):
        speed.linear.x = 0
        speed.angular.z = 0
        self.vel_pub.publish(speed)

    
    def distance(self,x1,y1,x2,y2):
        return((x1-x2)**2 + (y1-y2)**2)**0.5

    def run(self):
        global pi
        # add your code here to adjust your movement based on 2D pose feedback
        # Use the Controller          
        stage = 1
        speed = Twist()                  
        inc_x = self.target.x - self.pose.x
        inc_y = self.target.y - self.pose.y
        inc_theta = self.pose.theta
        #angle_to_target = atan2(inc_y, inc_x)
        angle_to_target = 0
        rospy.loginfo("Current Target is (Position X = %f, Position Y = %f)", self.target.x,self.target.y)
        pid = pid_controller.Controller(P = 2.5, D = 5.0, set_point=angle_to_target)
        while not rospy.is_shutdown():
            inc_x = self.target.x - self.pose.x
            inc_y = self.target.y - self.pose.y
            print("[System] ----------------------Entering into Stage 1!----------------------")

            if(stage == 1):										# Moving forward
                #angle_to_target = atan2(inc_y, inc_x)							# Calculate the angle different between target and current
                angle_to_target = self.pose.theta
                rospy.loginfo("[Information] Stage 1 Odom Theta is: %f (%f)",self.pose.theta, self.pose.theta*180/pi)
                ang_vel = pid.update(angle_to_target) 						# Using PID to calculate the angular speed
                print("[Information] Current angle to adjust is: ", ang_vel)
                speed.linear.x = 0.15
                speed.angular.z = -1 * float(ang_vel)			# clockwise = negative
                #rospy.loginfo("Current information: Target X = %f, Target Y = %f, Pose X = %f, Pose Y = %f", self.target.x, self.target.y, self.pose.x, self.pose.y)
                #if self.distance(self.target.x, self.target.y, self.pose.x, self.pose.y) < 0.3:			# Arrived
                if (abs(self.target1.x - self.pose.x) <= 0.1):
                    speed.linear.x = 0									# Stop
                    speed.angular.z = 0									# Stop
                    stage = 2
                    self.reset_pub.publish(self.msg)
                    time.sleep(4)
                    print("[System] ----------------------Entering into Stage 2!----------------------")

            elif(stage == 2):										# Rotating with 90 degrees
                pid.setPoint(set_point=1.5707963268)							# Set Stage 2 Goal
                pid.setPD(P = 0.1, D = 0.9)								# Modify value of P and D for stage 2
                angle_to_target = self.pose.theta
                rospy.loginfo("[Information] Stage 2 Odom Theta is: %f (%f)",self.pose.theta, self.pose.theta*180/pi)
                ang_vel = pid.update(angle_to_target)
                print("[Information] Current angle to adjust is: ",ang_vel)
                speed.linear.x = 0
                speed.angular.z = -1 * float(ang_vel)
                print("[Decision] Theta different between target and current is: ", (abs(float(self.target.theta) - float(self.pose.theta))))
                if (abs(float(self.target2.theta) - float(self.pose.theta)) <= 0.05):
                    speed.linear.x = 0
                    speed.angular.z = 0
                    stage = 3
                    self.reset_pub.publish(self.msg)
                    time.sleep(4) 
                    print("[System] ----------------------Entering into Stage 3!----------------------")

            elif(stage == 3):                                                                           # Rotating with 90 degrees
                pid.setPoint(set_point=0.0)                                                    # Set Stage 2 Goal
                pid.setPD(P = 0.6, D = 0.6)                                                            # Modify value of P and D for stage 2
                angle_to_target = self.pose.theta
                rospy.loginfo("[Information] Stage 3 Odom Theta is: %f (%f)",self.pose.theta, self.pose.theta*180/pi)
                ang_vel = pid.update(angle_to_target)
                print("[Information] Current angle to adjust is: ",ang_vel)
                speed.linear.x = 0.15
                speed.angular.z = -1 * float(ang_vel)
                if (abs(self.target3.x - self.pose.x) <= 0.1):			# Arrived
                    speed.linear.x = 0
                    speed.angular.z = 0
                    stage = 4
                    self.reset_pub.publish(self.msg)
                    time.sleep(2)

            self.vel_pub.publish(speed)
            self.rate.sleep()
            time.sleep(0.05)
        pass


    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            # display (x, y, theta) on the terminal
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':
    whatever = robot()
    whatever.run()
    #except KeyboardInterrupt:
        #whatever = shutdown()
