#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <tf/transform_datatypes.h>
#include <limits>
#include <visualization_msgs/Marker.h>

/**
 *
 * This node will move the robot based on bug algorithms
 *
 */

/// The node is publishing a geometry_msgs/Twist
ros::Publisher pub_cmd_vel, pub_state;

/// Define all #defines here!
#define RAD2DEG (180/M_PI)
#define MAX_LIN_VEL 1.5
#define MAX_ANG_VEL 0.675
#define SAFE_WALL_DISTANCE 1.0
#define WALL_ANG_TO_RIGHT (-M_PI/2)
#define MIN_LIN_VEL 0.25
#define SAFE_WALL_APPROACH_DISTANCE (1.25*SAFE_WALL_DISTANCE)
#define ALPHA (1.0/7.0)
#define KP (-70.0)
#define WALL_ANGLE_ERROR_THRESHOLD 0.1
#define WALL_DIST_ERROR_THRESHOLD 0.001
#define TARGET_SQUARE_THRESH 0.5


/// Params
bool  verbose = false;
float angleKp = -0.675;
float sLineThreshold = 0.1; /// This is the threshold to determine if you are close enough to the S-Line yet
float startingSquareThresh = 0.3; /// This is to tell if you have circumnavigated (re-entered the starting box)
float lookaheadDistance = 2.0; /// How far ahead to steer towards

/// Define global variables
geometry_msgs::Twist cmd_vel;
bool wallInFront = false;

/// Set up an enum for robot states
enum robot_states_t {FINDING_SLINE, MOVING_ON_SLINE, FINDING_WALL, WALL_FOLLOWING, REACHED_GOAL, ERROR_STATE};
robot_states_t robot_state = REACHED_GOAL;

struct point{
    float x;
    float y;
};

struct line{
    point start;
    point end;
    float length;
};


struct pose{
    point pos;
    float yaw;
};

line path;
pose robot_pose;
point wall_follow_start;
float max_distance_from_wall_follow_start;
bool got_robot_pose;
bool do_i_know_target=false;
bool last_tracking_error_sign;

struct tracking{
    float tracking_error;
    float angle_error;
    float distance;
    float corrective_radius;
};

float distance_to_pt(point p1, point p2)
{
    return std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+
                     (p1.y-p2.y)*(p1.y-p2.y));
}

float distance_to_line(line l, point p)
{

    /* https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points */
    /* Note that this distance is signed, which tells us which side of the line we are on */
    /* This is our tracking error. */
    float distance =((((l.end.y-l.start.y)*p.x)
                     -((l.end.x-l.start.x)*p.y))
                     +((l.end.x*l.start.y) - (l.end.y*l.start.x)))/l.length;
    return distance;
}

/*!
 * \brief wrap angle to unit circle
 * \param angle in radians, any range
 * \return angle in radians, -pi to +pi
 */
float wrap_angle(float angle)
{
    angle+= M_PI;
    angle = ( angle - ( (2*M_PI) * ((float) std::floor( angle / (2*M_PI) ) ) ) ) - M_PI;
    return angle;
}

float nearest_angle_diff(float to,float from)
{
    /// Get angles to same range
    to = wrap_angle(to);
    from = wrap_angle(from);
    float angle = to - from;
    /// Account for shorter angle during wraparound
    if      (angle >   M_PI) { angle -= 2 * M_PI; }
    else if (angle <= -M_PI) { angle += 2 * M_PI; }
    return angle;
}

float bearing_error_to_line(line l, float yaw)
{
    float line_bearing = std::atan2(l.end.y-l.start.y,l.end.x-l.start.x);
    float angle = nearest_angle_diff(line_bearing,yaw);
    return angle;
}


/*!
 * \brief get yawrate for twist message
 * \param radius radius of curvature to follow
 * \param propel forward velocity
 * \return yawrate in radians per second
 */
float radius_to_yawrate(float radius,float propel)
{
    /// Avoid dividing by zero
    if(std::fabs(radius)<1e-6f){return 0.f;}
    return (propel/radius);
}

/*!
 * \brief corrective_radius
 * \param tracking_error        Normal distance to the path
 * \param angle_error           Difference between bearing of robot and bearing of path (-pi to +pi)
 * \param lookahead_distance    Point to target on the path ahead of the current position
 * \param min_turn_radius       Could be constraint of ackermann vehicle
 * \param max_angle_error       Any error beyond this means turn as sharp as possible to get facing the right way
 * \return radius               Radius of curvature to hit path exactly lookahead distance ahead
 */
float corrective_radius(float tracking_error, float angle_error, float lookahead_distance, float min_turn_radius = FLT_MIN, float max_angle_error = M_PI_2)
{
    float radius = (tracking_error*tracking_error + lookahead_distance*lookahead_distance) / (2.f*(tracking_error*std::cos(angle_error) + lookahead_distance*std::sin(angle_error)));
    if((radius < min_turn_radius && radius >=0.f) || angle_error > max_angle_error)
    {
        //std::cout << "Capping radius to " << min_turn_radius << " because radius of " << radius << " too small or angle of " << angle_error << " too big" << std::endl;
        return  min_turn_radius;
    }
    if((radius >-min_turn_radius && radius < 0.f) || angle_error <-max_angle_error)
    {
        //std::cout << "Capping radius to " << -min_turn_radius << " because radius of " << radius << " too small or angle of " << angle_error << " too big" << std::endl;
        return -min_turn_radius;
    }
    return radius;
}

tracking follow_line(line l, pose robot_pose, float lookahead_distance, float min_turn_radius = FLT_MIN, float max_angle_error = M_PI_2)
{
    tracking t;
    t.tracking_error = distance_to_line(l,robot_pose.pos);
    t.angle_error = bearing_error_to_line(l,robot_pose.yaw);
    t.corrective_radius = corrective_radius(t.tracking_error,t.angle_error,lookahead_distance,min_turn_radius,max_angle_error);
    t.distance = distance_to_pt(l.end,robot_pose.pos);
    return t;
}

/// Callback function for target
void targetScanCallback(const nav_msgs::Odometry::ConstPtr& targetScan){

    float eps=0.001;/// 1mm

    /// Only update if we got a new target
    if(got_robot_pose && (std::fabs(path.end.x - float(targetScan->pose.pose.position.x))>eps ||
                          std::fabs(path.end.x - float(targetScan->pose.pose.position.x))>eps))
    {
        path.end.x = targetScan->pose.pose.position.x;
        path.end.y = targetScan->pose.pose.position.y;
        path.start = robot_pose.pos;
        path.length = distance_to_pt(path.start,path.end);
        ROS_INFO("Got new target of x=%6.3f,y=%6.3f",path.end.x,path.end.y);
        do_i_know_target = true;
    }
}


/// Callback function for base scan
void baseScanCallback(const sensor_msgs::LaserScan::ConstPtr& baseScan){

    /// Don't bother if we don't know where we are or where we are going
    if(!do_i_know_target){
        return;
    }

    /// This is redunant, but make sure we stop even if one message drops out. \todo Add timer to avoid needing this.
    if(robot_state == REACHED_GOAL || robot_state == ERROR_STATE){
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        pub_cmd_vel.publish(cmd_vel);
        return;
    }


    /// Define some local variables
    float theta;
    float dist;
    float min_theta = 0.0;
    float min_dist = 0.0;
    float wallAngleError;
    float wallDistError;


    /// Set min_dist to its max value to initiate it
    min_dist = std::numeric_limits<float>::max();

    /// Parse through laser scan to find minimum distance to an object and bearing to that object
    for(int ii = 0; ii < baseScan->ranges.size(); ii++){
        theta = baseScan->angle_min + ii*(baseScan->angle_increment);
        dist = baseScan->ranges[ii];
        if (dist < min_dist){
            min_dist = dist;
            min_theta = theta;
        }
    }
    //ROS_INFO("min_theta=%f,min_dist=%f",min_theta,min_dist);

    /// Check to see if a wall is in front (Scan only thetas from -M_PI/4 to M_PI/4)
    for(int ii = floor((-M_PI/4.0-baseScan->angle_min)/baseScan->angle_increment); ii < floor((M_PI/4.0-baseScan->angle_min)/baseScan->angle_increment); ii++){
        if(baseScan->ranges[ii] < SAFE_WALL_APPROACH_DISTANCE){
            wallInFront = true;
            break;
        }
        else{
            wallInFront = false;
        }
    }

    /// Check to see if the robot is within the wall approach distance to detect the wall
    if (min_dist <= SAFE_WALL_APPROACH_DISTANCE && robot_state != FINDING_SLINE && (min_theta > -(M_PI/2) && min_theta < (M_PI/2)) && wallInFront){
        robot_state = FINDING_WALL;
        wall_follow_start = robot_pose.pos;
        max_distance_from_wall_follow_start = 0.f;
    }

    /// ==== Wall Following State ====
    if (robot_state == FINDING_WALL || robot_state == WALL_FOLLOWING){
        /// First thing to do is stop
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        pub_cmd_vel.publish(cmd_vel);

        /// Calculate errors for "parallelness" to the wall and distance from wall
        wallAngleError = (min_theta - WALL_ANG_TO_RIGHT);
        wallDistError = -(min_dist - SAFE_WALL_DISTANCE);

        if (robot_state == FINDING_WALL && std::fabs(wallAngleError) < sLineThreshold)
        {
            robot_state = WALL_FOLLOWING;
        }
        if (robot_state == FINDING_WALL)
        {

            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = wallAngleError*angleKp;
            if( cmd_vel.angular.z > MAX_ANG_VEL){
                cmd_vel.angular.z = MAX_ANG_VEL;
            }
            if( cmd_vel.angular.z <-MAX_ANG_VEL){
                cmd_vel.angular.z =-MAX_ANG_VEL;
            }
        }else{
            float radius = corrective_radius(wallDistError,wallAngleError,lookaheadDistance);
            ROS_INFO_COND(verbose,"Following Wall. Angle error = %6.3frad, Tracking error = %6.3fm, Radius = %6.3fm",wallAngleError,wallDistError,radius);
            if(wallInFront){
                cmd_vel.linear.x = MIN_LIN_VEL;
            }else{
                cmd_vel.linear.x = 0.5*MAX_LIN_VEL;
            }
            cmd_vel.angular.z = -radius_to_yawrate(radius,cmd_vel.linear.x);
        }
        /// Publish the velocity
        pub_cmd_vel.publish(cmd_vel);
    }
}


/// Callback function for base scan
void gpsScanCallback(const nav_msgs::Odometry::ConstPtr& gpsScan){

    /// Copy robot pose into global so scan callback can use it
    robot_pose.pos.x = gpsScan->pose.pose.position.x;
    robot_pose.pos.y = gpsScan->pose.pose.position.y;
    tf::Pose currentRobotPoseTF;
    tf::poseMsgToTF(gpsScan->pose.pose,currentRobotPoseTF);
    robot_pose.yaw = tf::getYaw(currentRobotPoseTF.getRotation());
    got_robot_pose = true;

    std_msgs::Int8 state_msg;
    state_msg.data = (int8_t) robot_state;
    pub_state.publish(state_msg);

    /// If we don't have a target, or are at target, just stop
    if(!do_i_know_target || robot_state == REACHED_GOAL || robot_state == ERROR_STATE){
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        pub_cmd_vel.publish(cmd_vel);
        return;
    }

    tracking t = follow_line(path,robot_pose,lookaheadDistance);

    /// Calculate which direction the robot needs to rotate to get to the S-Line (Dependent on being above or below the target)
    if (robot_state != WALL_FOLLOWING && robot_state != FINDING_WALL && robot_state != MOVING_ON_SLINE){
        if (std::fabs(t.angle_error) > sLineThreshold){
            robot_state = FINDING_SLINE;
        }
        else{
            robot_state = MOVING_ON_SLINE;
        }
    }
    if(t.distance < TARGET_SQUARE_THRESH){
        /// WE MADE IT TO THE GOAL!
        ROS_INFO_COND(robot_state != REACHED_GOAL,"MADE IT TO THE GOAL!!");
        robot_state = REACHED_GOAL;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        pub_cmd_vel.publish(cmd_vel);
        return;
    }

    /// Find the S-Line by rotating in place
    if (robot_state == FINDING_SLINE){
        ROS_INFO_COND(verbose,"FINDING S-LINE. Angle error = %6.3frad",t.angle_error);
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = t.angle_error*angleKp;
        if( cmd_vel.angular.z > MAX_ANG_VEL){
            cmd_vel.angular.z = MAX_ANG_VEL;
        }
        if( cmd_vel.angular.z <-MAX_ANG_VEL){
            cmd_vel.angular.z =-MAX_ANG_VEL;
        }
    }
    /// Move along the S-Line
    else if (robot_state == MOVING_ON_SLINE){

        ROS_INFO_COND(verbose,"ON THE S-LINE. Angle error = %6.3frad, Tracking error = %6.3fm, Radius = %6.3fm",t.angle_error,t.tracking_error,t.corrective_radius);
        if(wallInFront){
            cmd_vel.linear.x = MIN_LIN_VEL;
        }else{
            cmd_vel.linear.x = 0.5*MAX_LIN_VEL;
        }
        cmd_vel.angular.z = -radius_to_yawrate(t.corrective_radius,cmd_vel.linear.x);
    }
    /// Follow a wall state
    else if (robot_state == WALL_FOLLOWING || robot_state == FINDING_WALL){
        bool tracking_error_sign = t.tracking_error>0.f;
        ROS_INFO_COND(verbose,"WALL FOLLOWING tracking=%6.3f, sign=%d, last=%d",t.tracking_error,(int)tracking_error_sign,(int)last_tracking_error_sign);

        float distance_from_wall_follow_start = distance_to_pt(robot_pose.pos,wall_follow_start);
        if(distance_from_wall_follow_start>max_distance_from_wall_follow_start){
            max_distance_from_wall_follow_start = distance_from_wall_follow_start;
        }

        /// Check if you are in the start point square
        if(distance_from_wall_follow_start < startingSquareThresh){
            if(max_distance_from_wall_follow_start > 2.f*startingSquareThresh){
                ROS_ERROR("Circumnavigated entire obstacle. WTF, mate?");
                robot_state=ERROR_STATE;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                pub_cmd_vel.publish(cmd_vel);

            }else{
                ROS_INFO_COND(verbose,"I'M STILL IN THE STARTING SQUARE! dist=%6.3fm",distance_from_wall_follow_start);
                /// DO NOTHING
            }
        }
        else{
            if(tracking_error_sign!=last_tracking_error_sign){
                robot_state = FINDING_SLINE;
            }
        }
        last_tracking_error_sign = tracking_error_sign;
    }

    /// I am not in a valid state so STOP!
    else{
        ROS_WARN("Something has gone horribly wrong! I am stopping");
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }

    /// Publish a velocity for any robot state EXCEPT state 2 (The LIDAR baseScanCallback handles state 2)
    if (robot_state != WALL_FOLLOWING && robot_state != FINDING_WALL){
        pub_cmd_vel.publish(cmd_vel);
    }
}


int main(int argc, char **argv)
{
    /// Name your node
    ros::init(argc, argv, "bug_nav");

    /// Setup a ROS node handle (nh_)
    ros::NodeHandle nh_;

    /// Publisher object that decides what kind of topic to publish and how fast.
    pub_cmd_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_state   = nh_.advertise<std_msgs::Int8>("/bug_state", 1);

    /// Setup up the subscribers
    ros::Subscriber sub_gps_target = nh_.subscribe("/target_pose_ground_truth",1,targetScanCallback); /// Subscribe to target pos
    ros::Subscriber sub_base_scan = nh_.subscribe("/base_scan",1,baseScanCallback); /// Subscribes to base scan
    ros::Subscriber sub_gps_pose = nh_.subscribe("/base_pose_ground_truth",1,gpsScanCallback); /// Subscribes to gps of current pos

    ros::spin();

    return 0;
}


