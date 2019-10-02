#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <limits>
#include <visualization_msgs/Marker.h>

/**
 *
 * This node will move the robot based on bug algorithms
 *
 */

/// The node is publishing a geometry_msgs/Twist
ros::Publisher pub_cmd_vel;

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

/// Define global variables
float min_theta = 0.0;
float min_dist = 0.0;
bool firstTimeWallFollowing = true;
float minDistWallFollowing;
float x_minDistWallFollowing;
float y_minDistWallFollowing;
bool outOfCircle = false;
double foundShortestDistance;
float x_circumnavigation;
float y_circumnavigation;
bool outOfStartingSquare = false;
float xDeltaToStart;
float yDeltaToStart;
float xDeltaToTarget;
float yDeltaToTarget;
bool circumnavigated = false;
bool bugAlgorithm1 = true;
bool startingAlgorithm = true;
float bearing_SLine;
float distance_SLine;
float x_target;
float y_target;
float x_target_prev = 0.0;
float y_target_prev = 0.0;
float distToTarget;
float bearToTarget;
geometry_msgs::Twist cmd_vel;

bool do_i_know_target=false;
bool bearingSwitched;
bool negativeBearing;
bool safeToSwitchFromWall = true;
float rotated_min_theta;
bool targetSelected = true;
bool justSelectedTarget = false;
bool wallInFront = false;

/// Set up an enum for robot states
enum robot_states_t {FINDING_SLINE, MOVING_ON_SLINE, WALL_FOLLOWING, REACHED_GOAL};
robot_states_t robot_state = FINDING_SLINE;

void initializeBools(){
    targetSelected = true;
    firstTimeWallFollowing = true;
    outOfCircle = false;
    startingAlgorithm = true;
    circumnavigated = false;
    outOfStartingSquare = false;
    justSelectedTarget = false;
    robot_state = FINDING_SLINE;
}

struct point{
    float x;
    float y;
};

struct line{
    point start;
    point end;
    float length;
};

line path;

struct pose{
    point pos;
    float yaw;
};
pose robot_pose;
bool got_robot_pose;

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
    float radius = (tracking_error*tracking_error + lookahead_distance*lookahead_distance) / (tracking_error*std::cos(angle_error) + lookahead_distance*std::sin(angle_error));
//    if((radius < min_turn_radius && radius >=0.0) || angle_error > max_angle_error)
//    {
//        ///
//        return  min_turn_radius;
//    }
//    if((radius >-min_turn_radius && radius < 0.0) || angle_error <-max_angle_error)
//    {
//        return -min_turn_radius;
//    }
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

    if(got_robot_pose && (std::fabs(path.end.x - float(targetScan->pose.pose.position.x))>eps ||
                          std::fabs(path.end.x - float(targetScan->pose.pose.position.x))>eps))
    {
        path.end.x = targetScan->pose.pose.position.x;
        path.end.y = targetScan->pose.pose.position.y;
        path.start = robot_pose.pos;
        path.length = distance_to_pt(path.start,path.end);
        do_i_know_target = true;
    }
}


/// Callback function for base scan
void baseScanCallback(const sensor_msgs::LaserScan::ConstPtr& baseScan){
    if(!do_i_know_target || !targetSelected){
        return;
    }

    if(robot_state == REACHED_GOAL){
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        pub_cmd_vel.publish(cmd_vel);
        return;
    }


    /// Define some local variables
    float theta;
    float dist;
    float wallAngleError;
    float wallDistError;
    float angularSpeed;


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
        if(robot_state == MOVING_ON_SLINE){
            distance_SLine = distToTarget;
        }
        robot_state = WALL_FOLLOWING;
    }

    /// ==== Wall Following State ====
    if (robot_state == WALL_FOLLOWING){
        /// First thing to do is stop
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        pub_cmd_vel.publish(cmd_vel);

        /// Calculate errors for "parallelness" to the wall and distance from wall
        wallAngleError = (min_theta - WALL_ANG_TO_RIGHT);
        wallDistError = (min_dist - SAFE_WALL_DISTANCE);

        /// Calculate the angular velocity based on weighted sum of both criteria (Parallelness and distance from wall)
        angularSpeed = wallAngleError*ALPHA*KP - wallDistError*(1-ALPHA)*KP;
        cmd_vel.angular.z = angularSpeed;

        /// Make sure the angular velocity does not exceed the limits
        if(angularSpeed > MAX_ANG_VEL){
            cmd_vel.angular.z = MAX_ANG_VEL;
        }
        if(angularSpeed < -MAX_ANG_VEL){
            cmd_vel.angular.z = -MAX_ANG_VEL;
        }

        /// If the bearing is "far off" and the distance is "far off" then rotate proportional to angle error and move forward SLOWLY
        if (fabs(wallAngleError) > WALL_ANGLE_ERROR_THRESHOLD && fabs(wallDistError) > WALL_DIST_ERROR_THRESHOLD){
            if(wallInFront){
                cmd_vel.linear.x = 0.1*MIN_LIN_VEL;
            }else{
                cmd_vel.linear.x = MIN_LIN_VEL;
            }
            /// Calculate the angular velocity
            angularSpeed = angularSpeed = wallAngleError*(ALPHA)*KP;
            cmd_vel.angular.z = angularSpeed;
            if(angularSpeed > MAX_ANG_VEL){
                cmd_vel.angular.z = MAX_ANG_VEL;
            }
            if(angularSpeed < -MAX_ANG_VEL){
                cmd_vel.angular.z = -MAX_ANG_VEL;
            }

        }
        /// If the bearing is "far off" but the distance is "close" then only move proportional to angle error
        else if(fabs(wallAngleError) > WALL_ANGLE_ERROR_THRESHOLD && fabs(wallDistError) <= WALL_DIST_ERROR_THRESHOLD){
            if(wallInFront){
                cmd_vel.linear.x = 0.1*MIN_LIN_VEL;
            }else{
                cmd_vel.linear.x = MIN_LIN_VEL;
            }
        }
        /// If the bearing is "close" but the distance is "far off" then move slowly right and forward
        else if(fabs(wallAngleError) <= WALL_ANGLE_ERROR_THRESHOLD && fabs(wallDistError) > WALL_DIST_ERROR_THRESHOLD){
            if(wallInFront){
                cmd_vel.linear.x = 0.1*MIN_LIN_VEL;
            }else{
                cmd_vel.linear.x = MIN_LIN_VEL;
            }
        }
        /// If the bearing is "close" and the distance is "close" then only move proportional to angle error
        else if(fabs(wallAngleError) <= WALL_ANGLE_ERROR_THRESHOLD && fabs(wallDistError) <= WALL_DIST_ERROR_THRESHOLD){
            if(wallInFront){
                cmd_vel.linear.x = MIN_LIN_VEL;
            }else{
                cmd_vel.linear.x = MAX_LIN_VEL;
            }
        }
        /// Something has gone wrong!
        else{
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            std::cout << "SOMETHING HAS GONE TERRIBLY WRONG! I AM STOPPING" << std::endl;
        }
        /// Publish the velocity
        pub_cmd_vel.publish(cmd_vel);
    }
}


/// Callback function for base scan
void gpsScanCallback(const nav_msgs::Odometry::ConstPtr& gpsScan){
    robot_pose.pos.x = gpsScan->pose.pose.position.x;
    robot_pose.pos.y = gpsScan->pose.pose.position.y;
    tf::Pose currentRobotPoseTF;
    tf::poseMsgToTF(gpsScan->pose.pose,currentRobotPoseTF);
    robot_pose.yaw = tf::getYaw(currentRobotPoseTF.getRotation());
    got_robot_pose = true;

    if(!do_i_know_target){
        return;
    }
    tracking t = follow_line(path,robot_pose,2.0);

    if(robot_state == REACHED_GOAL){
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        pub_cmd_vel.publish(cmd_vel);
        return;
    }

    /// Define some local variables
    float currentGPSx;
    float currentGPSy;
    float currentGPStheta;
    float angleKp = -0.675;
    float angleError,absAngleError;
    float sLineThreshold = 0.1; /// This is the threshold to determine if you are close enough to the S-Line yet
    float startingSquareThresh = 0.3; /// This is to tell if you have circumnavigated (re-entered the starting box)
    float shortestSquareThresh = 0.1; /// This is to tell if you have made it back to the shortest point (after circumnavigating)
    float diffBearingRobotToTarget;
    float diffBearingTargetToRobot;

    /// Populate the x,y, and theta variables of the robot
    currentGPSx = gpsScan->pose.pose.position.x;
    currentGPSy = gpsScan->pose.pose.position.y;
    currentGPStheta = tf::getYaw(currentRobotPoseTF.getRotation());


    /// Calculate the difference between current x and y and target x and y
    float deltaX = x_target - currentGPSx;
    float deltaY = y_target - currentGPSy;

    /// Calculate distance and bearing to target
    distToTarget = sqrt(deltaX*deltaX + deltaY*deltaY);
    bearToTarget = atan2(deltaY,deltaX);

    if (!bugAlgorithm1 && startingAlgorithm){
        bearing_SLine = bearToTarget;
        distance_SLine = distToTarget;
        startingAlgorithm = false;
    }


    /// Calculate which direction the robot needs to rotate to get to the S-Line (Dependent on being above or below the target)
    angleError = t.angle_error;
    absAngleError=std::fabs(angleError);
    if (robot_state != WALL_FOLLOWING){
        if (absAngleError > sLineThreshold){
            robot_state = FINDING_SLINE;
        }
        else if(absAngleError <= sLineThreshold){
            robot_state = MOVING_ON_SLINE;
        }
    }

    /// Find the S-Line by rotating in place
    if (robot_state == FINDING_SLINE){
        std::cout << "FINDING S-LINE!! " << std::endl;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = angleError*angleKp;
        if(angleError*angleKp > MAX_ANG_VEL){
            cmd_vel.angular.z = MAX_ANG_VEL;
        }
        if(angleError*angleKp < -MAX_ANG_VEL){
            cmd_vel.angular.z = -MAX_ANG_VEL;
        }
        if(fabs(deltaX) < TARGET_SQUARE_THRESH && fabs(deltaY) < TARGET_SQUARE_THRESH){
            /// WE MADE IT TO THE GOAL!
            robot_state = REACHED_GOAL;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            pub_cmd_vel.publish(cmd_vel);
            std::cout << "MADE IT TO THE GOAL!!" << std::endl;
            return;
        }
    }
    /// Move along the S-Line
    else if (robot_state == MOVING_ON_SLINE){

        ROS_INFO("ON THE S-LINE. Angle error = %6.3frad, Tracking error = %6.3fm, Radius = %6.3fm",t.angle_error,t.tracking_error,t.corrective_radius);
        if(wallInFront){
	    cmd_vel.linear.x = MIN_LIN_VEL;
        }else{
            cmd_vel.linear.x = 0.5*MAX_LIN_VEL;
        }
        cmd_vel.angular.z = 0.0;
        if(fabs(deltaX) < TARGET_SQUARE_THRESH && fabs(deltaY) < TARGET_SQUARE_THRESH){
            /// WE MADE IT TO THE GOAL!
            robot_state = REACHED_GOAL;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            pub_cmd_vel.publish(cmd_vel);
            std::cout << "MADE IT TO THE GOAL!!" << std::endl;
            return;
        }
    }
    /// Follow a wall state
    else if (robot_state == WALL_FOLLOWING){
        std::cout << "WALL FOLLOWING!!! " << std::endl;

        /// This is for bug algorithm 1 -- The difference is to circumnavigate then leave the wall
        if(bugAlgorithm1){
            if (firstTimeWallFollowing){
                minDistWallFollowing = std::numeric_limits<float>::max();
                x_circumnavigation = currentGPSx;
                y_circumnavigation = currentGPSy;
                y_minDistWallFollowing = std::numeric_limits<float>::max();
                x_minDistWallFollowing = std::numeric_limits<float>::max();
                outOfStartingSquare = false;
                circumnavigated = false;
                firstTimeWallFollowing = false;
            }

            /// Find the minimum distance to the wall and store the x and y coordinates at this point
            if(distToTarget < minDistWallFollowing && !circumnavigated){
                minDistWallFollowing = distToTarget;
                x_minDistWallFollowing = currentGPSx;
                y_minDistWallFollowing = currentGPSy;
            }

            /// Calculate difference between current x and y and the circumnavigated x and y at start point of wall

            if(!circumnavigated)
            {
                xDeltaToStart = currentGPSx - x_circumnavigation;
                yDeltaToStart = currentGPSy - y_circumnavigation;

                /// Check if you are in the start point square
                if(fabs(xDeltaToStart) < startingSquareThresh && fabs(yDeltaToStart) < startingSquareThresh){
                    if(outOfStartingSquare){
                        circumnavigated = true;
                        std::cout << "I CIRCUMNAVIGATED!!!!!!" << std::endl;
                    }
                    else{
                        std::cout << "I'M STILL IN THE STARTING SQUARE" << std::endl;
                    }
                }
                else{
                    outOfStartingSquare = true;
                }
            }

            if(circumnavigated)
            {
                xDeltaToTarget = currentGPSx - x_minDistWallFollowing;
                yDeltaToTarget = currentGPSy - y_minDistWallFollowing;

                /// Check if you are in the shortest point from feature square
                if(fabs(xDeltaToTarget) < shortestSquareThresh && fabs(yDeltaToTarget) < shortestSquareThresh){
                    std::cout << "FOUND THE SHORTEST DISTANCE" << std::endl;
                    robot_state = FINDING_SLINE;
                    firstTimeWallFollowing = true;
                    /// FOUND IT SO STOP
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                }
            }
        }
        /// This is for bug algorithm 2 -- The difference is to leave the wall after reaching the S line again
        else if (!bugAlgorithm1){



            /// Determine if this is the first time on the obstacle's wall
            if (firstTimeWallFollowing){
                x_circumnavigation = currentGPSx;
                y_circumnavigation = currentGPSy;
                outOfStartingSquare = false;
                firstTimeWallFollowing = false;
                bearingSwitched = false;

            }

            /// Calculate difference between current x and y and the circumnavigated x and y at start point of wall

            xDeltaToStart = currentGPSx - x_circumnavigation;
            yDeltaToStart = currentGPSy - y_circumnavigation;

            /// Calculate the difference between bearing to target and bearing to sLine
            diffBearingTargetToRobot = std::fmod((bearToTarget - bearing_SLine+10*M_PI),2*M_PI);
            diffBearingRobotToTarget = std::fmod((bearing_SLine - bearToTarget+10*M_PI),2*M_PI);

            /// Check if you are in the start point square
            if(fabs(xDeltaToStart) < startingSquareThresh && fabs(yDeltaToStart) < startingSquareThresh){
                if(!outOfStartingSquare){
                    std::cout << "I'M STILL IN THE STARTING SQUARE!" << std::endl;
                    /// DO NOTHING
                }
            }
            else{
                if (!outOfStartingSquare){
                    if(diffBearingTargetToRobot < diffBearingRobotToTarget){
                        negativeBearing = true;
                    }
                    else{
                        negativeBearing = false;
                    }
                    outOfStartingSquare = true;
                }
            }

            /// Check to see if the bearing sign has switched
            ///
            ///
            if(outOfStartingSquare){
                if(negativeBearing && (diffBearingTargetToRobot > diffBearingRobotToTarget)){
                    if((distToTarget < distance_SLine) && safeToSwitchFromWall){
                        bearingSwitched = true;
                    }
                    else{
                        negativeBearing = !negativeBearing;
                    }
                }
                else if(!negativeBearing && (diffBearingTargetToRobot < diffBearingRobotToTarget)){
                    if((distToTarget < distance_SLine) && safeToSwitchFromWall){
                        bearingSwitched = true;
                    }
                    else{
                        negativeBearing = !negativeBearing;
                    }
                }
            }

            if(bearingSwitched && outOfStartingSquare && distToTarget < distance_SLine){
                distance_SLine = distToTarget;
                robot_state = FINDING_SLINE;
                firstTimeWallFollowing = true;
            }

        }
        if(fabs(deltaX) < TARGET_SQUARE_THRESH && fabs(deltaY) < TARGET_SQUARE_THRESH){
            /// WE MADE IT TO THE GOAL!
            robot_state = REACHED_GOAL;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            pub_cmd_vel.publish(cmd_vel);
            std::cout << "MADE IT TO THE GOAL!!" << std::endl;
            return;
        }
    }

    /// I am not in a valid state so STOP!
    else{
        std::cout << "Something has gone horribly wrong! I am stopping" << std::endl;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }

    /// Publish a velocity for any robot state EXCEPT state 2 (The LIDAR baseScanCallback handles state 2)
    if (robot_state != WALL_FOLLOWING){
        pub_cmd_vel.publish(cmd_vel);
    }
}


int main(int argc, char **argv)
{
    /// Name your node
    ros::init(argc, argv, "stage_mover");


    for (int ii = 1; ii < argc; ii++) {
        if (std::string(argv[ii]) == "bug1") {
            bugAlgorithm1 = true;
        }
        else if(std::string(argv[ii]) == "bug2"){
            bugAlgorithm1 = false;
        }
        else{
            bugAlgorithm1 = true;
        }
    }

    /// Setup a ROS node handle (nh_)
    ros::NodeHandle nh_;

    /// Publisher object that decides what kind of topic to publish and how fast.
    pub_cmd_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    /// Setup up the subscribers
    ros::Subscriber sub_gps_target = nh_.subscribe("/target_pose_ground_truth",1,targetScanCallback); /// Subscribe to target pos
    ros::Subscriber sub_base_scan = nh_.subscribe("/base_scan",1,baseScanCallback); /// Subscribes to base scan
    ros::Subscriber sub_gps_pose = nh_.subscribe("/base_pose_ground_truth",1,gpsScanCallback); /// Subscribes to gps of current pos

    if(bugAlgorithm1){
        std::cout << "STARTING BUG 1 ALGORITHM!" << std::endl;
    }
    else if(!bugAlgorithm1){
        std::cout << "STARTING BUG 2 ALGORTIHM!" << std::endl;
    }


    ros::spin();

    return 0;
}

