/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "corvus_ik/corvus_point.h"


ros::NodeHandle *nh;
ros::NodeHandle *pnh;

tf::TransformBroadcaster *pose_broadcaster; /// This is a tf
tf::TransformListener *listener;

ros::Publisher pub_joint_target;

class CorvusInverseKinematics
{
public:

    std::string base_link,target_link,robot_name;
    corvus_point cp;
    int num_divs;
    bool verbose;
    eTransform target_trans;
    ePose old_pose;

    void getTarget(void){
        tf::StampedTransform transform;
        try{
          listener->lookupTransform(base_link, target_link,
                                   ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_ERROR_THROTTLE(10,"%s",ex.what());
          return;
        }

        tf::Matrix3x3 tf_rot(transform.getRotation());
        Eigen::Matrix3f rot;
        rot << tf_rot.getColumn(0).getX(),tf_rot.getColumn(1).getX(),tf_rot.getColumn(2).getX(),
               tf_rot.getColumn(0).getY(),tf_rot.getColumn(1).getY(),tf_rot.getColumn(2).getY(),
               tf_rot.getColumn(0).getZ(),tf_rot.getColumn(1).getZ(),tf_rot.getColumn(2).getZ();
        target_trans.linear()=rot;
        target_trans.translation()=Eigen::Vector3f(transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ());

    }

    void ikCallback(const ros::TimerEvent&)
    {
        int NUM_JOINTS=6;int GOLD_ARM=0;int GREEN_ARM=1;

        sensor_msgs::JointState msg;
        msg.header.stamp=ros::Time::now();

        getTarget();
        old_pose.resize(NUM_JOINTS);
        for(int joint=0;joint<NUM_JOINTS;joint++){
            old_pose(joint)=0.0;
        }
        bool success=cp.ik(target_trans,old_pose,num_divs,verbose);
        if(success){
            //std::cout << x << "," << y << "," << z << "," << std::endl;
            int arm=GOLD_ARM;
            for(int joint=0;joint<NUM_JOINTS;joint++){
                msg.position.push_back(cp.joint_pose(joint));
                std::stringstream ss;
                ss << robot_name;
                ss << "_" << "joint_" << joint;
                msg.name.push_back(ss.str());
            }
            pub_joint_target.publish(msg);
        }else{
            //ROS_ERROR("Failed to solve IK!");
        }
    }

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "corvus_ik_node");

    CorvusInverseKinematics corvusInverseKinematics;

    nh = new ros::NodeHandle;
    pnh = new ros::NodeHandle("~");

    // Publish tf's
    pose_broadcaster = new tf::TransformBroadcaster;
    listener = new tf::TransformListener;

    /// Get the name of the robot as a parameter
    pnh->getParam("robot_name", corvusInverseKinematics.robot_name);
    pnh->getParam("target_link", corvusInverseKinematics.target_link);
    pnh->getParam("base_link", corvusInverseKinematics.base_link);
    pnh->getParam("num_divs", corvusInverseKinematics.num_divs);
    pnh->getParam("verbose", corvusInverseKinematics.verbose);

    pub_joint_target      = nh->advertise<sensor_msgs::JointState>   ("joint_targets",1);

    ros::Timer timer      = nh->createTimer(ros::Duration(0.01), &CorvusInverseKinematics::ikCallback, &corvusInverseKinematics);

    ros::spin();

    /// Code Cleanup
    delete nh;
    delete pnh;
    delete pose_broadcaster;
    delete listener;

    return 0;
}

