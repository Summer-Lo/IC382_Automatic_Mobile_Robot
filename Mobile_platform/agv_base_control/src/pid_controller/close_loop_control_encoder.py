#!/usr/bin/env python3

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, Vector3
import pid_controller

class robot():
    def __init__(self):
        rospy.init_node("pid_controller")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)                    # cmd/vel
        self.rate = rospy.Rate(10)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("odom/reset", Empty, queue_size=10)     # odom/reset
        for i in range(10):
            self.reset_pub.publish(Empty())                                                               # publish empty message for reseting the odom
            self.rate.sleep()
        
        # encoder configuration
        self.encoder_per_rotation_m1 = 116.05
        self.encoder_per_rotation_m2 = 123.1
        self.encoder_ratio_m1_m2 = float(self.encoder_per_rotation_m1/self.encoder_per_rotation_m2)

        # subscribe to odometry
        self.pose = Pose2D()
        self.tick = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
                # setpoint Config 
        # ---------------<START>---------------
        self.target = Pose2D()
        self.target.x = 5
        self.target.y = 0
        self.target.theta = 0
        # ---------------<END>---------------
        self.odom_sub = rospy.Subscriber("/stm32/encoders", Vector3, self.encoder_callback)

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
        # add your code here to adjust your movement based on 2D pose feedback
        # Use the Controller          
        self.tick.x = self.pose.x
        self.tick.y = self.pose.y
        speed = Twist()                  
        rospy.loginfo("Current Target is (Position X = %f, Position Y = %f)", self.target.x,self.target.y)
        pid = pid_controller.Controller(P = 0.6, D = 0.6, set_point=self.encoder_ratio_m1_m2)
        while not rospy.is_shutdown():
            if(self.pose.x - self.tick.x != 0 and self.pose.y - self.tick.y != 0):
                encoder_m1 = self.pose.x - self.tick.x						# Calculate the difference of encoder value
                encoder_m2 = self.pose.y - self.tick.y
                ratio = float(encoder_m1/encoder_m2)
                angle_to_target = ratio 
                rospy.loginfo("ratio of encorder calculated is: %f",angle_to_target)
                ang_vel = pid.update(angle_to_target) 						# Using PID to calculate the angular speed
                print("Current angle to adjust is: ", ang_vel)
                speed.linear.x = 0.15
                speed.angular.z =  ang_vel/2			# clockwise = negative
                #rospy.loginfo("Current information: Target X = %f, Target Y = %f, Pose X = %f, Pose Y = %f", self.target.x, self.target.y, self.pose.x, self.pose.y)
                if self.distance(self.target.x, self.target.y, self.pose.x, self.pose.y) < 0.3:			# Arrived
                    speed.linear.x = 0									# Stop
                    speed.angular.z = 0									# Stop
                self.vel_pub.publish(speed)
                self.rate.sleep()
        pass


    def encoder_callback(self, msg):
        # get pose = (x, y) [encoder1, encoder2] from odometry topic
        self.pose.x = msg.x
        self.pose.y = msg.y
        print(f"Encoder values are: %s and %s",self.pose.x, self.pose.y)
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
