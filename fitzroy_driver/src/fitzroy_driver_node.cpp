#include "ros/ros.h"
#include "ros/console.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "iostream"
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/JointState.h"
#include "diagnostic_msgs/DiagnosticStatus.h"

#include <sstream>

///////////////   ALL SENSORS STUFFED INTO AN ARRAY OF FLOATS   ////////////////////////////////
/////////////// This is partly because the arduino ros library  ////////////////////////////////
/////////////// is limited in the message types it supports,    ////////////////////////////////
/////////////// and partly to save bandwidth & processing.      ////////////////////////////////
/////////////// As much as possible, they are in standard units ////////////////////////////////
enum array_indices_teensy_out_v1{
    PROTOCOL_VERSION, /// This should be incremented for future versions so we can know if we are in agreement with the Arduino
    PROPEL,     /// In radians/sec.  Redundant to a motor, but it makes it a little easier if they are explicit?
    STEER,		/// In radians       Redundant to a motor, but it makes it a little easier if they are explicit?
    DTIME,		/// For integration  Redundant
    VOLT_BAT,	/// Battery state
    VOLT_5V,	/// Maybe a battery state msg?
    TOF_1,		/// Range
    TOF_2,		/// Range
    TOF_3,		/// Range
    US_1,       /// Range
    US_2,       /// Range
    AUX_1,		/// Could be IR range sensor? \todo use param to tell what it actually is.
    AUX_2,		/// Could be IR range sensor? \todo use param to tell what it actually is.
    AUX_3,		/// Could be IR range sensor? \todo use param to tell what it actually is.
/// The MPU 6050 goes into an Imu message
    IMU_ACC_X,
    IMU_ACC_Y,
    IMU_ACC_Z,
    IMU_ROT_X,
    IMU_ROT_Y,
    IMU_ROT_Z,
/// The GPS goes into a NavSatFix message
    GPS_LAT,    /// Degrees
    GPS_LON,    /// Degrees
    GPS_ALT,    /// Meters
    GPS_HDOP,   /// Unitless, 1.0 means best, bigger means worse
    GPS_COG,    /// Degrees
    GPS_SOG,    /// m/s
/// These could be redundant to propel/steer but are all here for completeness
/// They will all be packed into one JointState message
    MOT1_POS,   /// Radians or meters
    MOT1_VEL,   /// Rad/s or m/s
    MOT1_EFT,   /// Should be Nm or N, but will just be PWM %
    MOT2_POS,
    MOT2_VEL,
    MOT2_EFT,
    MOT3_POS,
    MOT3_VEL,
    MOT3_EFT,
    MOT4_POS,
    MOT4_VEL,
    MOT4_EFT,
/// Servos do not have feedback, but we can rate limit them and assume that we are at the target
/// \todo Figure out which message to include this in. Maybe the joint states?
    SERVO_1,
    SERVO_2,
/// Dunno why we would need this, but it's on the MPU6050
    TEMPERATURE,
    TEENSY_OUT_ARRSIZE
};

enum array_indices_teensy_in{
    TARGET_PROPEL,      /// In radians/sec
    TARGET_STEER,       /// In radians
/// These could be redundant to propel/steer but are all here for completeness
    TARGET_MOT1,
    TARGET_MOT2,
    TARGET_MOT3,
    TARGET_MOT4,
    TARGET_SERVO_1,
    TARGET_SERVO_2,
    TEENSY_IN_ARRSIZE
};

/// Robot dimensions
float wheel_base = 0.137f;
float rear_wheel_dia = 0.064f;
float front_wheel_dia = rear_wheel_dia;
std::string robot_name = "fitzroy";
std::string motor1_name = robot_name + "_motor1";
std::string motor2_name = robot_name + "_motor2";
std::string motor3_name = robot_name + "_motor3";
std::string motor4_name = robot_name + "_motor4";
std::string servo1_name = robot_name + "_servo1";
std::string servo2_name = robot_name + "_servo2";

float ir_range_of_detection = 0.05; // meters
float ir_field_of_view = 0.1; // radians

/// HC-SR04
float us_field_of_view = 30.0*M_PI/180;
float us_min_range = 0.02;
float us_max_range = 4.00;

/// VL53L0X
float tof_field_of_view = 25.0*M_PI/180;
float tof_min_range = 0.03;
float tof_max_range = 2.00;

float max_batt_voltage = 12.7; // VDC
float min_batt_voltage = -11.5; // VDC
float no_batt_voltage  = -6.0;  // VDC

float cmdvel_timeout = 2.0;
float gps_x_cov = 1.0;
float gps_y_cov = 1.0;

/// timing stuff
ros::Time current_time, last_time;

/// broadcaster stuff
ros::Publisher pub_to_serial;
ros::Publisher pub_odom;
ros::Publisher pub_battery;
ros::Publisher pub_5v;
ros::Publisher pub_arduino_status;
ros::Publisher pub_gps;
ros::Publisher pub_imu;
ros::Publisher pub_joints;
ros::Publisher pub_temp;

/// Range messages
ros::Publisher pub_tof_1;
ros::Publisher pub_tof_2;
ros::Publisher pub_tof_3;
ros::Publisher pub_us_1;
ros::Publisher pub_us_2;
ros::Publisher pub_ir_1;
ros::Publisher pub_ir_2;
ros::Publisher pub_ir_3;

float current_x = 0.f;
float current_y = 0.f;
float current_th = 0.f;


void propogate(float dt,float propel_speed, float twist_rate)
{
    /// Propagate the robot using basic odom
    current_x += propel_speed*cos(current_th) * dt;
    current_y += propel_speed*sin(current_th) * dt;
    current_th += twist_rate * dt;
}

double twist2angle(double twist_rate, double propel_speed)
{
  return std::atan2(wheel_base*twist_rate,propel_speed);
}

double angle2twist(double steering_angle,double propel_speed)
{
  return propel_speed/wheel_base*std::tan(steering_angle);
}

//callbacks for getting velocity data
void sub_actual(const std_msgs::Float32MultiArray& msg)
{
    current_time = ros::Time::now();
    if(msg.data.size()>DTIME){
        static tf::TransformBroadcaster odom_broadcaster;

		//compute the odoms
        double propel_speed = msg.data[PROPEL] * rear_wheel_dia / 2.0;
        double steering_angle = msg.data[STEER];
        double twist_rate = angle2twist(steering_angle, propel_speed);
		propogate((current_time - last_time).toSec(),propel_speed,twist_rate);

		//Now setup all the odom nodes
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(current_th);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint";

		odom_trans.transform.translation.x = current_x;
		odom_trans.transform.translation.y = current_y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = current_x;
		odom.pose.pose.position.y = current_y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = propel_speed;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.angular.z = twist_rate;

		//publish the odom message
		pub_odom.publish(odom);

		//timer update
        last_time = current_time;
    }

    if(msg.data.size()>VOLT_BAT){
        sensor_msgs::BatteryState batt_msg;
        batt_msg.design_capacity=2.2;
        batt_msg.header.frame_id=robot_name + "_battery";
        batt_msg.header.stamp=current_time;
        batt_msg.power_supply_technology=sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
        batt_msg.power_supply_health=sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        batt_msg.power_supply_status=sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        batt_msg.voltage=msg.data[VOLT_BAT];

        /// \warning This is assuming that we can measure the state of charge from the voltage!
        /// This is a silly thing to do if we are drawing power from the battery!
        /// Which we are always doing!
        /// So this is all bunk!
        batt_msg.percentage=(msg.data[VOLT_BAT]-min_batt_voltage)/(max_batt_voltage-min_batt_voltage);
        if(batt_msg.percentage>1.0){
            batt_msg.percentage=1.0;
        }
        if(batt_msg.percentage<=0.0){
            batt_msg.percentage=0.0;
            batt_msg.power_supply_health=sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
        }

        if(msg.data[VOLT_BAT] < no_batt_voltage){
            ROS_WARN_THROTTLE(5.0,"Fitzroy Switched Off.");
            batt_msg.power_supply_status=sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
            batt_msg.power_supply_health=sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        }else if(msg.data[VOLT_BAT] < min_batt_voltage){
            batt_msg.power_supply_health=sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
            ROS_WARN_THROTTLE(5.0,"Fitzroy Battery Dead.  Recharge.");
        }

        pub_battery.publish(batt_msg);
    }
    if(msg.data.size()>VOLT_5V){
        /// This is a bit silly, but at least its a tiny message.
        std_msgs::Float32 volt_msg;
        volt_msg.data = msg.data[VOLT_5V];
        pub_5v.publish(volt_msg);
    }
    if(msg.data.size()>TOF_1){
        sensor_msgs::Range range_msg;
        range_msg.radiation_type = 2; /// Only options are IR and US, so for TOF I just picked something that isn't those two.
        range_msg.min_range = tof_min_range;
        range_msg.max_range = tof_max_range;
        range_msg.header.frame_id = robot_name + "_tof1";
        range_msg.header.stamp = current_time;
        range_msg.field_of_view = tof_field_of_view;
        range_msg.range = msg.data[TOF_1];
        pub_tof_1.publish(range_msg);
    }
    if(msg.data.size()>TOF_2){
        sensor_msgs::Range range_msg;
        range_msg.radiation_type = 2; /// Only options are IR and US, so for TOF I just picked something that isn't those two.
        range_msg.min_range = tof_min_range;
        range_msg.max_range = tof_max_range;
        range_msg.header.frame_id = robot_name + "_tof2";
        range_msg.header.stamp = current_time;
        range_msg.field_of_view = tof_field_of_view;
        range_msg.range = msg.data[TOF_2];
        pub_tof_2.publish(range_msg);
    }
    if(msg.data.size()>TOF_3){
        sensor_msgs::Range range_msg;
        range_msg.radiation_type = 2; /// Only options are IR and US, so for TOF I just picked something that isn't those two.
        range_msg.min_range = tof_min_range;
        range_msg.max_range = tof_max_range;
        range_msg.header.frame_id = robot_name + "_tof3";
        range_msg.header.stamp = current_time;
        range_msg.field_of_view = tof_field_of_view;
        range_msg.range = msg.data[TOF_3];
        pub_tof_3.publish(range_msg);
    }
    if(msg.data.size()>US_1){
        sensor_msgs::Range range_msg;
        range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
        range_msg.min_range = us_min_range;
        range_msg.max_range = us_max_range;
        range_msg.header.frame_id = robot_name + "_us1";
        range_msg.header.stamp = current_time;
        range_msg.field_of_view = us_field_of_view;
        range_msg.range = msg.data[US_1];
        pub_us_1.publish(range_msg);
    }
    if(msg.data.size()>US_2){
        sensor_msgs::Range range_msg;
        range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
        range_msg.min_range = us_min_range;
        range_msg.max_range = us_max_range;
        range_msg.header.frame_id = robot_name + "_us2";
        range_msg.header.stamp = current_time;
        range_msg.field_of_view = us_field_of_view;
        range_msg.range = msg.data[US_2];
        pub_us_2.publish(range_msg);
    }
    if(msg.data.size()>AUX_1){
        sensor_msgs::Range range_msg;
        range_msg.radiation_type = sensor_msgs::Range::INFRARED;
        range_msg.min_range = range_msg.max_range = ir_range_of_detection;
        range_msg.header.frame_id = robot_name + "_ir1";
        range_msg.header.stamp = current_time;
        range_msg.field_of_view = ir_field_of_view;
        /// Apparently for on/off range sensors we use +/- infinity? Seems odd to me, but okay.
        if(msg.data[AUX_1]<0.5f)
        {
            range_msg.range = -std::numeric_limits<float>::infinity();
        }else{
            range_msg.range =  std::numeric_limits<float>::infinity();
        }
        pub_ir_1.publish(range_msg);
    }
    if(msg.data.size()>AUX_2){
        sensor_msgs::Range range_msg;
        range_msg.radiation_type = sensor_msgs::Range::INFRARED;
        range_msg.min_range = range_msg.max_range = ir_range_of_detection;
        range_msg.header.frame_id = robot_name + "_ir2";
        range_msg.header.stamp = current_time;
        range_msg.field_of_view = ir_field_of_view;
        /// Apparently for on/off range sensors we use +/- infinity? Seems odd to me, but okay.
        if(msg.data[AUX_2]<0.5f)
        {
            range_msg.range = -std::numeric_limits<float>::infinity();
        }else{
            range_msg.range =  std::numeric_limits<float>::infinity();
        }
        pub_ir_2.publish(range_msg);
    }
    if(msg.data.size()>AUX_3){
        sensor_msgs::Range range_msg;
        range_msg.radiation_type = sensor_msgs::Range::INFRARED;
        range_msg.min_range = range_msg.max_range = ir_range_of_detection;
        range_msg.header.frame_id = robot_name + "_ir3";
        range_msg.header.stamp = current_time;
        range_msg.field_of_view = ir_field_of_view;
        /// Apparently for on/off range sensors we use +/- infinity? Seems odd to me, but okay.
        if(msg.data[AUX_3]<0.5f)
        {
            range_msg.range = -std::numeric_limits<float>::infinity();
        }else{
            range_msg.range =  std::numeric_limits<float>::infinity();
        }
        pub_ir_3.publish(range_msg);
    }
    if(msg.data.size()>IMU_ROT_Z)
    {
        /// \todo fill out the rest of the message, including covariance and whatnot
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = current_time;
        imu_msg.header.frame_id = robot_name + "_imu";
        imu_msg.linear_acceleration.x = msg.data[IMU_ACC_X];
        imu_msg.linear_acceleration.y = msg.data[IMU_ACC_Y];
        imu_msg.linear_acceleration.z = msg.data[IMU_ACC_Z];
        imu_msg.angular_velocity.x = msg.data[IMU_ROT_X];
        imu_msg.angular_velocity.y = msg.data[IMU_ROT_Y];
        imu_msg.angular_velocity.z = msg.data[IMU_ROT_Z];
        pub_imu.publish(imu_msg);
    }
    if(msg.data.size()>GPS_LON)
    {
        /// \todo include altitude and HDOP, then calculate covariance from HDOP
        sensor_msgs::NavSatFix gps_msg;
        gps_msg.header.stamp = current_time;
        gps_msg.header.frame_id = robot_name + "_gps";
        if(isnan(msg.data[GPS_LAT]) || isnan(msg.data[GPS_LON]))
        {
            gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        }else{
            gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        }
        /// The ublox neo-6m is gps only
        gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        double usual_error = 2.0; // meters. Wild guess.
        double cov = std::pow(usual_error * msg.data[GPS_HDOP],2.0);
        gps_msg.position_covariance[0] = cov;
        gps_msg.position_covariance[4] = cov;
        gps_msg.position_covariance[8] = cov;
        gps_msg.altitude = msg.data[GPS_ALT];
        gps_msg.latitude = msg.data[GPS_LAT];
        gps_msg.longitude = msg.data[GPS_LON];
        pub_gps.publish(gps_msg);
    }
    if(msg.data.size()>MOT4_EFT)
    {
        sensor_msgs::JointState joint_msg;
        joint_msg.header.stamp = current_time;
        joint_msg.header.frame_id = robot_name + "_joints";
        joint_msg.name.push_back(motor1_name);
        joint_msg.name.push_back(motor2_name);
        joint_msg.name.push_back(motor3_name);
        joint_msg.name.push_back(motor4_name);
        joint_msg.position.push_back(msg.data[MOT1_POS]);
        joint_msg.position.push_back(msg.data[MOT2_POS]);
        joint_msg.position.push_back(msg.data[MOT3_POS]);
        joint_msg.position.push_back(msg.data[MOT4_POS]);
        joint_msg.velocity.push_back(msg.data[MOT1_VEL]);
        joint_msg.velocity.push_back(msg.data[MOT2_VEL]);
        joint_msg.velocity.push_back(msg.data[MOT3_VEL]);
        joint_msg.velocity.push_back(msg.data[MOT4_VEL]);
        joint_msg.effort.push_back(msg.data[MOT1_EFT]);
        joint_msg.effort.push_back(msg.data[MOT2_EFT]);
        joint_msg.effort.push_back(msg.data[MOT3_EFT]);
        joint_msg.effort.push_back(msg.data[MOT4_EFT]);

        if(msg.data.size()>SERVO_2)
        {
            /// This is a bit silly, since we don't have position feedback on the servos anyways, but hey whynot.
            joint_msg.name.push_back(servo1_name);
            joint_msg.name.push_back(servo2_name);
            joint_msg.position.push_back(msg.data[SERVO_1]);
            joint_msg.position.push_back(msg.data[SERVO_2]);
            /// No velocity or effort info available. Maybe shoud be NANs?
            joint_msg.velocity.push_back(0.0);
            joint_msg.velocity.push_back(0.0);
            joint_msg.effort.push_back(0.0);
            joint_msg.effort.push_back(0.0);
        }

        pub_joints.publish(joint_msg);
    }
    if(msg.data.size()>TEMPERATURE)
    {
        sensor_msgs::Temperature temp_msg;
        temp_msg.header.stamp = current_time;
        temp_msg.header.frame_id = robot_name + "_imu";
        temp_msg.variance = 0.0; /// 0 is interpreted as variance unknown
        temp_msg.temperature = msg.data[TEMPERATURE];
        pub_temp.publish(temp_msg);
    }
    else{
        //ROS_WARN_ONCE("Ronny Arduino Message Undersized!  WTF, Mate?");
    }
}

void watchdogCallback(const ros::TimerEvent&){
    ros::Duration dt=ros::Time::now() - last_time;
    diagnostic_msgs::DiagnosticStatus msg;
    msg.level=diagnostic_msgs::DiagnosticStatus::OK;
    msg.name=robot_name + "_arduino";
    if(dt.toSec()>cmdvel_timeout){
        /// If we haven't seen a valid message in two seconds, something has gone horribly wrong.
        msg.level=diagnostic_msgs::DiagnosticStatus::ERROR;
    }
    pub_arduino_status.publish(msg);
}

void sub_cmd_vel(const geometry_msgs::Twist &msg)
{
    std_msgs::Float32MultiArray serial_msg;
    serial_msg.data.resize(TEENSY_IN_ARRSIZE);
    serial_msg.data[TARGET_PROPEL] = msg.linear.x / (rear_wheel_dia / 2.0);
    serial_msg.data[TARGET_STEER] = twist2angle(msg.angular.z,msg.linear.x);
    /// \todo add other motors and servos here.
    pub_to_serial.publish(serial_msg);
}

int main(int argc, char **argv)
{
	//intialize ROS
	ros::init(argc, argv, "fitzroy_driver");

	//Setup all the nodes
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	ros::Subscriber sub_velocity_actual = nh.subscribe("velocity_actual", 1000, sub_actual);
	ros::Subscriber sub_velocity_target = nh.subscribe("cmd_vel", 1000, sub_cmd_vel);

	/// broadcaster stuff
    pub_to_serial		= nh.advertise<std_msgs::Float32MultiArray		>("Velocity_Out", 1);

	/// Sensors
    pub_odom			= nh.advertise<nav_msgs::Odometry				>("odom", 1);
    pub_battery			= nh.advertise<sensor_msgs::BatteryState		>("battery", 1);
    pub_5v				= nh.advertise<std_msgs::Float32				>("logic_voltage", 1);
    pub_arduino_status  = nh.advertise<diagnostic_msgs::DiagnosticStatus >("arduino_status", 1);
    pub_gps				= nh.advertise<sensor_msgs::NavSatFix			>("gps", 1);
    pub_imu				= nh.advertise<sensor_msgs::Imu                 >("imu", 1);
    pub_joints			= nh.advertise<sensor_msgs::JointState			>("joint_states", 1);
    pub_temp			= nh.advertise<sensor_msgs::Temperature         >("temperature", 1);

	/// Range messages
	pub_tof_1			= nh.advertise<sensor_msgs::Range				>("tof_1", 1);
	pub_tof_2			= nh.advertise<sensor_msgs::Range				>("tof_2", 1);
	pub_tof_3			= nh.advertise<sensor_msgs::Range				>("tof_3", 1);
	pub_us_1    		= nh.advertise<sensor_msgs::Range				>("ultrasonic_1", 1);
	pub_us_2    		= nh.advertise<sensor_msgs::Range				>("ultrasonic_2", 1);
    pub_ir_1			= nh.advertise<sensor_msgs::Range				>("pub_ir_1", 1);
    pub_ir_2			= nh.advertise<sensor_msgs::Range				>("pub_ir_2", 1);
    pub_ir_3			= nh.advertise<sensor_msgs::Range				>("pub_ir_3", 1);

	ros::Timer timer = nh.createTimer(ros::Duration(1.0), watchdogCallback);

	//timing stuff
	current_time = ros::Time::now();
	last_time = current_time;

    pnh.getParam("wheel_base",wheel_base);
    pnh.getParam("rear_wheel_dia",rear_wheel_dia);
    pnh.getParam("front_wheel_dia",front_wheel_dia);
    pnh.getParam("robot_name",robot_name);
    pnh.getParam("motor1_name",motor1_name);
    pnh.getParam("motor2_name",motor2_name);
    pnh.getParam("motor3_name",motor3_name);
    pnh.getParam("motor4_name",motor4_name);
    pnh.getParam("servo1_name",servo1_name);
    pnh.getParam("servo2_name",servo2_name);


    pnh.getParam("ir_range_of_detection",ir_range_of_detection);
    pnh.getParam("ir_field_of_view",ir_field_of_view);

    ros::spin();


	return 0;
}
