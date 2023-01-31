#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>

#define PI 3.14159265

// Basic ROS configuration
const char node_name[] = "encoder_to_odometry_publisher";      // node name
const char encoder_topic[] = "/stm32/encoders";                // publisher
const char odom_topic[] = "/odom";                             // subsriber
const char speed_topic[] = "/cmd_vel";                         // subscriber
const char initial_topic[] = "/initialpose";                   // subscriber
const char source_link[] = "odom";                             // TF pair
const char target_link[] = "base_link";                        // TF pair
const int ros_update_hz = 10;                                   // 5 Hz or 10 Hz (ROS)

// Robot physical parameters
const double offset_distance_error = 3.580;                                                  // distance compensation
const double offset_theta_error = 0.18;                                                      // theta compensation
const double robot_wheel_seperation = 0.45;                                                  // in meter
const double robot_wheel_radius = 0.0625;                                                    // in meter
const double encoder_per_rotation_m1 = 2239; //2239;                                         // encoder value per rotation of motor1 Right
const double encoder_per_rotation_m2 = 2353; //2353;                                         // encoder value per rotation of motor2 Left

double distance_per_count_m1 = 0.00016265673; //0.000175328; //(double)(2*PI*robot_wheel_radius)/encoder_per_rotation_m1;    // 0.000175328 Distance for an encoder pulse in m
double distance_per_count_m2 = 0.00014667685; //0.000166886; //(double)(2*PI*robot_wheel_radius)/encoder_per_rotation_m2 ;   // 0.000166886 Distance for an encoder pulse in m
double encoder_min = 0;
double encoder_max = 65535;
double encoder_low_wrap =  (encoder_max - encoder_min) * 0.3 + encoder_min;
double encoder_max_wrap =  (encoder_max - encoder_min) * 0.7 + encoder_min;

// Variables
geometry_msgs::Twist msg;
ros::Time current_time, last_time;  // ROS Time
double linear_x = 0;                // robot x
double linear_y = 0;                // robot y
double angular_z = 0;               // robot angular z

// Instantaneous variables
double dm1 = 0;
double dm2 = 0;
double dc = 0;
double dx = 0;
double dy = 0;
double dtheta = 0;
double dt = 0;

// Odometry data
double x = 0;
double y = 0;
double theta = 0.00000000001;;

// Previous value
double _x_tick = 0;
double _y_tick = 0;
double _theta = 0;
int init = 0;

// Velocity data
double vx = 0;
double vr = 0;
double vm1 = 0;
double vm2 = 0;

bool encoderRecieved = false;

ros::Publisher odom_pub;

void resetOdometry(const std_msgs::Empty::ConstPtr &msg)
{
    init = 0;
    theta = 0.00000000001;
    dtheta = 0;
    _theta = 0;
    x=0;
    y=0;
    printf("############  Reset Pose!  ############\r\n");
    printf("pose x: %f | pose y: %f | pose theta: %f(%f)\r\n", 0.0, 0.0, 0.0, 0.0);   
}

void VelocityCallback(const geometry_msgs::Twist& msg)
{
    linear_x = msg.linear.x;
    linear_y = msg.linear.y;
    angular_z = msg.angular.z;
}

void EncoderCallback(const geometry_msgs::Vector3::ConstPtr& encoder_ticks)
{
    current_time = ros::Time::now();
	dt = (current_time - last_time).toSec();
    last_time = current_time;  
    //dt = 0.2;

	// This calculation assume the encoder will send the value within 0 - 65535 //
    // Time intervals
	if (!init)
	{
        _x_tick = encoder_ticks->x;
        _y_tick = encoder_ticks->y;
        theta = 0.00000000001;
        dtheta = 0;
        init++;
        return;
	}

    if ((encoder_ticks->x == _x_tick) && (encoder_ticks->y == _y_tick))
	{   
        dm1 = 0;
        dm2 = 0;
        vm1 = 0;
        vm2 = 0;
    }
    else
    {
        // Number of counts in dt
      	// encoder_ticks->x + (65535 - _dm1) used to handle overflow problem of encoder register
        if ((encoder_ticks->x < encoder_low_wrap) && (_x_tick > encoder_max_wrap)) 
        {
            dm1 = (encoder_ticks->x + encoder_max - _x_tick) * distance_per_count_m1 ;
        }
        else 
        {
            dm1 = (encoder_ticks->x - _x_tick) * distance_per_count_m1 ; 
        }
        
        if ((encoder_ticks->y < encoder_low_wrap) && (_y_tick > encoder_max_wrap)) 
        {
            dm2 = (encoder_ticks->y + encoder_max - _y_tick) * distance_per_count_m2 ;
        }
        else 
        {
            dm2 = (encoder_ticks->y - _y_tick) * distance_per_count_m2 ;
        }

        _x_tick = encoder_ticks->x;
        _y_tick = encoder_ticks->y;

        // Calculate center turning curve
        dc = (dm2+dm1)*0.5;
        
        // Calculate orientation
        dtheta = (dm2-dm1)/robot_wheel_seperation;

        vm1=dm1/dt;
        vm2=dm2/dt;
        //vx = dc / dt;
        //vr = dtheta / dt;

        if (dc !=0)
        {
            dx = dc*cos(dtheta);  
            dy = dc*sin(dtheta);
            
            x = x + dx;
            y = y + dy;
        }

        if( dtheta != 0) 
            theta = theta + dtheta; 
     
        printf("Twist   vx: %f | vy: %f | wz: %f\r\n", linear_x, linear_y, angular_z);
        printf("Current vm1: %f | vm2: %f | wz: 0.0\r\n", vm1, vm2);        
        printf("encoder_1: %f | encoder_2: %f\r\n", encoder_ticks->x, encoder_ticks->y);     
        printf("dm1: %f | dm2: %f | dtheta: %f | dt: %f\r\n", dm1, dm2, dtheta, dt);
        printf("dc = %f \r\n", dc);
        printf("dx: %f | dy: %f | dtheta: %f(%f)\r\n", dx, dy, dtheta, dtheta*180/PI);
        printf("pose x: %f | pose y: %f | pose theta: %f(%f)\r\n", x, y, theta, theta*180/PI);
        printf("\r\n");

        encoderRecieved= true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(encoder_topic, 10, EncoderCallback);
    //ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 50);
    odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1);
    ros::Subscriber vel_sub = nh.subscribe(speed_topic,10,VelocityCallback);
    ros::Subscriber sub2 = nh.subscribe(initial_topic, 10, resetOdometry);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Rate r(ros_update_hz);
    printf("Encoder To Odom Transformation started!\r\n");

    // Publish loop
    while (nh.ok())
    {
        if (encoderRecieved)
        {
            // Only Yaw should be considered
            //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
            geometry_msgs::Quaternion odom_quat ; 
    
            odom_quat.x = 0.0; 
            odom_quat.y = 0.0; 
            odom_quat.z = 0.0; 
            odom_quat.z = sin( theta / 2 );   
            odom_quat.w = cos( theta / 2 ); 

            // A TF should be setup between base_link and odom
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = source_link;
            odom_trans.child_frame_id = target_link;
            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            // Send the transform
            odom_broadcaster.sendTransform(odom_trans);

            // Publish odom message
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = source_link;
            //set the position
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;   
            //set the velocity
            odom.child_frame_id = target_link;
            odom.twist.twist.linear.x=vx;
            odom.twist.twist.linear.y=0;
            odom.twist.twist.angular.z =vr; 

            //publish the message
            odom_pub.publish(odom);
            //last_time = current_time; 
            encoderRecieved=false;
        } 
     
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
