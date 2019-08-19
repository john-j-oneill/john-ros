
/// ROS
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <cmath>

/// Messages
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Float64.h>


//now instead of ackermann msgs, we subscribe to the hacky cmd_vel message from teb planner

//https://en.wikipedia.org/wiki/Ackermann_steering_geometry

//Published Topics:
//    </left_steering_ctrlr/command (std_msgs/Float64)
//        Command for the left wheel steering.
//    </right_steering_ctrlr/command (std_msgs/Float64)
//        Command for the right wheel steering.
//    </left_front_axle_ctrlr/command (std_msgs/Float64)
//        Command for the left front wheel propel.
//    </right_front_axle_ctrlr/command (std_msgs/Float64)
//        Command for the right front wheel #include <math.h>propel.
//    </left_rear_axle_ctrlr/command (std_msgs/Float64)
//        Command for the left rear wheel propel.
//    </right_rear_axle_ctrlr/command (std_msgs/Float64)
//        Command for the right rear wheel propel.


//Subscribed Topics:
//    </cmd_vel (geometry_msgs/Twist)
//        Desired speed and steering angle from teb planner plugin (angular.z = steer_angle)

//cuz radians
#define PI 3.14159265358979323846
#define EPSILON 0.0000001

//ROBOT PARAMETERS
double rear_wheel_dia = 1.16586; //meters
double front_wheel_dia = 0.68326; //meters
double rear_wheel_sep = 1.3716; //meters
double front_wheel_sep = 1.48082; //meters
double wheel_base = 1.8542; //meters
double radius_curvature = 0.0; // meters

//publisher objects
ros::Publisher left_steer_pub;
ros::Publisher right_steer_pub;
ros::Publisher left_front_propel_pub;
ros::Publisher right_front_propel_pub;
ros::Publisher left_rear_propel_pub;
ros::Publisher right_rear_propel_pub;
ros::NodeHandle *nh;

//so we dont redeclare our messages
std_msgs::Float64 left_steer_msg;
std_msgs::Float64 right_steer_msg;
std_msgs::Float64 left_front_propel_msg;
std_msgs::Float64 right_front_propel_msg;
std_msgs::Float64 left_rear_propel_msg;
std_msgs::Float64 right_rear_propel_msg;

//time out on velocities
ros::Time last_cmd_time;
double timeout_cmd_vel = 1.0;

float twist2angle(float twist_rate, float propel_speed)
{
  return std::atan(wheel_base/propel_speed*twist_rate);
}

float angle2twist(float steering_angle,float propel_speed)
{
  return propel_speed/wheel_base*std::tan(steering_angle);
}

/// Callback when we get a new ackermann command
// http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg){
    //record last command time
    last_cmd_time = ros::Time::now();

    //get propel and steer commands from the cmd vel
    double propel_cmd = cmd_msg->linear.x; // in meters/second
    double steer_cmd = twist2angle(cmd_msg->angular.z,propel_cmd); //in radians

    double angular_velocity_front_cmd = propel_cmd / (front_wheel_dia/2.0);
    double angular_velocity_rear_cmd = propel_cmd / (rear_wheel_dia/2.0);

    //format our command messages
    left_steer_msg.data = steer_cmd;
    right_steer_msg.data = steer_cmd;
    left_front_propel_msg.data = angular_velocity_front_cmd;
    right_front_propel_msg.data = angular_velocity_front_cmd;
    left_rear_propel_msg.data = angular_velocity_rear_cmd;
    right_rear_propel_msg.data = angular_velocity_rear_cmd;

    //Publish all the commands
    left_steer_pub.publish(left_steer_msg);
    right_steer_pub.publish(right_steer_msg);
    left_front_propel_pub.publish(left_front_propel_msg);
    right_front_propel_pub.publish(right_front_propel_msg);
    left_rear_propel_pub.publish(left_rear_propel_msg);
    right_rear_propel_pub.publish(right_rear_propel_msg);
}

//zero velocities if we haven't received a cmd in a while
void timercallback(const ros::TimerEvent&)
{
    ros::Duration timesincecmd = ros::Time::now() - last_cmd_time;
    if(timesincecmd.toSec() > timeout_cmd_vel){
        //we'll just stop tractor
        //format our command messages
        left_steer_msg.data = 0.0;
        right_steer_msg.data = 0.0;
        left_front_propel_msg.data = 0.0;
        right_front_propel_msg.data = 0.0;
        left_rear_propel_msg.data = 0.0;
        right_rear_propel_msg.data = 0.0;

        //Publish all the commands
        left_steer_pub.publish(left_steer_msg);
        right_steer_pub.publish(right_steer_msg);
        left_front_propel_pub.publish(left_front_propel_msg);
        right_front_propel_pub.publish(right_front_propel_msg);
        left_rear_propel_pub.publish(left_rear_propel_msg);
        right_rear_propel_pub.publish(right_rear_propel_msg);

    }
}


int main (int argc, char** argv)
{
    /// Initialize ROS
    ros::init (argc, argv, "ackermann_steering_controller");
    nh = new ros::NodeHandle("~");

    //get some params set in our launch file
    nh->getParam("rear_wheel_dia", rear_wheel_dia);
    nh->getParam("front_wheel_dia", front_wheel_dia);
    nh->getParam("rear_wheel_sep", rear_wheel_sep);
    nh->getParam("front_wheel_sep", front_wheel_sep);
    nh->getParam("wheel_base", wheel_base);
    //nh->getParam("/min_turn_angle", min_turn_angle);
    //nh->getParam("/max_turn_angle", max_turn_angle);
    //nh->getParam("/min_velocity", min_velocity);
    //nh->getParam("/max_velocity", max_velocity);
    //nh->getParam("/acc_lim_x", acc_lim_x);
    //nh->getParam("/vel_lim_theta", vel_lim_theta);

    //for publishing out steering commands to our simulated tractor
    left_steer_pub = nh->advertise<std_msgs::Float64>("/left_steering_ctrlr/command", 1);
    right_steer_pub = nh->advertise<std_msgs::Float64>("/right_steering_ctrlr/command", 1);

    //for publishing out propel commands to our simulated tractor
    left_front_propel_pub = nh->advertise<std_msgs::Float64>("/left_front_axle_ctrlr/command", 1);
    right_front_propel_pub = nh->advertise<std_msgs::Float64>("/right_front_axle_ctrlr/command", 1);
    left_rear_propel_pub = nh->advertise<std_msgs::Float64>("/left_rear_axle_ctrlr/command", 1);
    right_rear_propel_pub = nh->advertise<std_msgs::Float64>("/right_rear_axle_ctrlr/command", 1);

    // subscribe to teb planner
    ros::Subscriber ackermannSub;
    ackermannSub = nh->subscribe("/cmd_vel",1, cmdCallback);

    //timout checker
    last_cmd_time = ros::Time::now();
    ros::Timer veltimout = nh->createTimer(ros::Duration(0.1), timercallback);

    // Spin
    ros::spin ();
}



