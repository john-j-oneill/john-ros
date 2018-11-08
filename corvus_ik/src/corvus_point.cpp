#include "corvus_ik/corvus_point.h"
#include <iostream>

corvus_point::corvus_point()
{
    //Default to an identity
    frame.setIdentity();
    //Assume zero pose
    joint_pose.setZero();
    joint_vel.setZero();
}


/*!
 * \brief Constructor using inverse kinematics from a target frame (6 DOF)
 * \param frame_in Target frame (Position and orientation)
 * \param old_pose Previous pose, used to calculate cost of movement to find cheapest solution
 * \param t time (seconds)
 */
corvus_point::corvus_point(eTransform frame_in, ePose old_pose, Real t){
    error=!ik(frame_in,old_pose,J0_DIV);
    //Real target_error=frame_in.error(trans(joint_pose));
    //if(target_error>0.01){
        //std::cout.precision(3);
        //std::cout.setf( std::ios::fixed, std:: ios::floatfield );
        //std::cout << "error: "  << target_error << " " << error << std::endl;
        //ik(frame_in,old_pose,227,true);
    //}
    //Real target_error=frame_in.error(trans(joint_pose));
    frame=frame_in;
    time=t;
    //Depending on how much error there was in the ik(), we can use the following to correct the actual frame:
    //fk();
    //collision_detection();
}

/*!
 * \brief Constructor using forward kinematics from a known pose
 * \param new_pose New pose in joint space
 * \param t time (seconds)
 */
corvus_point::corvus_point(ePose new_pose,Real t){
    joint_pose=new_pose;
    time=t;
    fk();
    error=collision_detection(joint_pose);
    target_error=0.0;
}

/*!
 * \brief relative move from a coordinate system on the end effector
 *
 * This function is used with the LEAP motion to find a target
 * point based on a displacement in the end effector coordinate
 * system.  Note that it currently uses a sort of semi-redundant
 * forward kinematics (see how similar the math is to fk() ) with
 * the dx dy and dz placed at the end of the kinematic chain.
 * (Math equations are from matlab)
 * \param dx change in x (in the end effector coordinate system)
 * \param dy change in y (in the end effector coordinate system)
 * \param dz change in z (in the end effector coordinate system)
 */
void corvus_point::relative_move(Real dx,Real dy,Real dz){
    Real x=frame.translation()(0);
    Real y=frame.translation()(1);
    Real z=frame.translation()(2);
    Real J0=joint_pose(0);//meters
    Real J1=joint_pose(1);
    Real J2=joint_pose(2);
    Real J3=joint_pose(3);
    //x+=  dx*sin(J1) - dz*cos(J2 + J3)*cos(J1) - dy*sin(J2 + J3)*cos(J1);
    //y+=  dz*sin(J2 + J3)                      - dy*cos(J2 + J3);
    //z+=- dx*cos(J1) - dz*cos(J2 + J3)*sin(J1) - dy*sin(J2 + J3)*sin(J1);
    x= dx*sin(J1) - (cos(J1)*(7960*sin(J2 + J3) + 6997*sin(J2)))/20000 - dz*cos(J2 + J3)*cos(J1) - dy*sin(J2 + J3)*cos(J1);
    y=                          dz*sin(J2 + J3) -(6997*cos(J2)) /20000 - dy*cos(J2 + J3) - (199*cos(J2 + J3))/500 - 157/2000;
    z=-J0         - (sin(J1)*(7960*sin(J2 + J3) + 6997*sin(J2)))/20000 - dz*cos(J2 + J3)*sin(J1) - dy*sin(J2 + J3)*sin(J1) - dx*cos(J1);
    frame.translation()=Eigen::Vector3f(-x,y,z);
    basic_ik(joint_pose,x,y,z,J0);
    collision_detection(joint_pose);
}

void corvus_point::relative_move(Eigen::Vector3f leap_hand){
    //Eigen::Vector3f wrist_to_pen(0.0,0.000,0.275);//!<The triangle acrylic John and Jason made
    Eigen::Vector3f wrist_to_pen(0.0,0.050,0.234);//!<Long slanty diagonal free floating pen
    ///Since there is no force sensor, right now we are just setting 'Y' (height above
    ///the table) to a constant value
    leap_hand(1)=-.55;
    ik(leap_hand,wrist_to_pen(1),wrist_to_pen(2));
    fk();
}

void corvus_point::run_away(Real height,Real theta,Real force){
    const Real meters_per_kg=0.002; //max will be approx .5kg, so make it something reasonable
                                      //This can be thought of as a k_p

    Eigen::Vector3f point_joint_space(0,-height,0);
    Eigen::Vector3f direction_joint_space(sin(theta),0,cos(theta));
    direction_joint_space=direction_joint_space*force*meters_per_kg;
    eTransform joint_to_cart=trans(joint_pose,6);
    //Eigen::Vector3f point_cart_space=joint_to_cart*point_joint_space;
    Eigen::Vector3f new_point_cart_space=joint_to_cart*(point_joint_space+direction_joint_space);
    //std::cout << joint_to_cart.matrix() << std::endl;
    //std::cout << new_point_cart_space.get_string() << std::endl;
    error=!ik(new_point_cart_space,Eigen::Vector3f(0,-height,0),-1);
    fk();
}

eTransform corvus_point::leap_to_base(Eigen::Vector3f leap_position){
    //Eigen::Vector3f leap_position(0.0,0.050,0.121);

    eTransform tempframe;
    Eigen::Matrix3f rot;
    rot << 1.0, 0.0, 0.0,
           0.0, 0.0,-1.0,
           0.0, 1.0, 0.0;
    tempframe.linear()=rot;
    tempframe.translation()=leap_position;
    return frame*tempframe;
}

/*!
 * \brief very basic collision detection
 * \param joints The joint space definition to be evaluated
 * \retval true collision happened!
 * \retval false no problems
 */
bool corvus_point::collision_detection(ePose joints){
    Real safety_margin=1.*M_PI/180.;
    if(joints(1)>M_PI)
        return true;
    if(joints(1)<-M_PI)
        return true;
    if(joints(2)>M_PI_2-safety_margin)
        return true;
    if(joints(2)<-M_PI/3.+safety_margin)
        return true;
    if(joints(3)>M_PI*5./6.-safety_margin)
        return true;
    if(joints(3)<-M_PI/2.+safety_margin)
        return true;
    /*if(joints(1)<M_PI_2+safety_margin && joints(1)>-M_PI_2-safety_margin){
        //Over table
        if(frame.translation()(1)>-0.1)
            return true;
    }else*/{
        //Not over table
        if(joints(2)<-M_PI/6.+safety_margin)
            return true;
        if(frame.translation()(1)>0)
            return true;
    }
    return false;
}

/*!
 * \brief Transformation matrix using forward kinematics from a pose
 *
 * This function uses precomputed transformations using Matlab symbolic
 * toolbox, which should give nearly optimized code, by avoiding any
 * unneccisary multiplication by zero or one, and allowing an optimizing
 * compiler to do its job.
 *
 * \param joints            The joint pose to use
 * \param transformations   The number of transformations to use (default is all 6, acceptable values are 1-6)
 * \return                  Transformation matrix
 */
eTransform corvus_point::trans(ePose joints,int transformations){

    /// \note this code generated by fk_c_code.m using Matlab

    ///Precompute trig, for speed
    Real c1=cos(joints(1));
    Real s1=sin(joints(1));
    Real c2=cos(joints(2) + M_PI_2);
    Real s2=sin(joints(2) + M_PI_2);
    Real c3=cos(joints(3) + M_PI_2);
    Real s3=sin(joints(3) + M_PI_2);
    Real c4=cos(joints(4) + M_PI_2);
    Real s4=sin(joints(4) + M_PI_2);
    Real c5=cos(joints(5));
    Real s5=sin(joints(5));

    ///\todo could precompute some common factors, for speed
    Real a2=D_A2_A3;
    Real a3=D_A3_A5;

    Real r11,r12,r13,r21,r22,r23,r31,r32,r33,p_x,p_y,p_z;
    if(transformations==1){
        ///Calculate rotation matrix
        r11=1;
        r12=0;
        r13=0;
        r21=0;
        r22=0;
        r23=-1;
        r31=0;
        r32=1;
        r33=0;

        ///Calculate transformation
        p_x=0;
        p_y=0;
        p_z=joints(0);
    }else if(transformations==2){
        ///Calculate rotation matrix
        r11=c1;
        r12=0;
        r13=s1;
        r21=0;
        r22=-1;
        r23=0;
        r31=s1;
        r32=0;
        r33=-c1;

        ///Calculate transformation
        p_x=0;
        p_y=0;
        p_z=joints(0);
    }else if(transformations==3){
        ///Calculate rotation matrix
        r11=c1*c2;
        r12=-c1*s2;
        r13=s1;
        r21=-s2;
        r22=-c2;
        r23=0;
        r31=c2*s1;
        r32=-s1*s2;
        r33=-c1;

        ///Calculate transformation
        p_x=a2*c1*c2;
        p_y=-a2*s2;
        p_z=joints(0) + a2*c2*s1;
    }else if(transformations==4){
        ///Calculate rotation matrix
        r11=c1*c2*c3 - c1*s2*s3;
        r12=s1;
        r13=c1*c2*s3 + c1*c3*s2;
        r21=- c2*s3 - c3*s2;
        r22=0;
        r23=c2*c3 - s2*s3;
        r31=c2*c3*s1 - s1*s2*s3;
        r32=-c1;
        r33=c2*s1*s3 + c3*s1*s2;

        ///Calculate transformation
        p_x=a2*c1*c2;
        p_y=-a2*s2;
        p_z=joints(0) + a2*c2*s1;
    }else if(transformations==5){
        ///Calculate rotation matrix
        r11=c4*(c1*c2*c3 - c1*s2*s3) + s1*s4;
        r12=c1*c2*s3 + c1*c3*s2;
        r13=s4*(c1*c2*c3 - c1*s2*s3) - c4*s1;
        r21=-c4*(c2*s3 + c3*s2);
        r22=c2*c3 - s2*s3;
        r23=-s4*(c2*s3 + c3*s2);
        r31=c4*(c2*c3*s1 - s1*s2*s3) - c1*s4;
        r32=c2*s1*s3 + c3*s1*s2;
        r33=c1*c4 + s4*(c2*c3*s1 - s1*s2*s3);

        ///Calculate transformation
        p_x=a3*(c1*c2*s3 + c1*c3*s2) + a2*c1*c2;
        p_y=a3*(c2*c3 - s2*s3) - a2*s2;
        p_z=joints(0) + a3*(c2*s1*s3 + c3*s1*s2) + a2*c2*s1;
    }else if(transformations==6){
        ///Calculate rotation matrix
        r11=s5*(c1*c2*s3 + c1*c3*s2) + c5*(c4*(c1*c2*c3 - c1*s2*s3) + s1*s4);
        r12=c5*(c1*c2*s3 + c1*c3*s2) - s5*(c4*(c1*c2*c3 - c1*s2*s3) + s1*s4);
        r13=s4*(c1*c2*c3 - c1*s2*s3) - c4*s1;
        r21=s5*(c2*c3 - s2*s3) - c5*c4*(c2*s3 + c3*s2);
        r22=c5*(c2*c3 - s2*s3) + c4*s5*(c2*s3 + c3*s2);
        r23=-s4*(c2*s3 + c3*s2);
        r31=c5*(c4*(c2*c3*s1 - s1*s2*s3) - c1*s4) + s5*(c2*s1*s3 + c3*s1*s2);
        r32=c5*(c2*s1*s3 + c3*s1*s2) - s5*(c4*(c2*c3*s1 - s1*s2*s3) - c1*s4);
        r33=c1*c4 + s4*(c2*c3*s1 - s1*s2*s3);

        ///Calculate transformation
        p_x=a3*(c1*c2*s3 + c1*c3*s2) + a2*c1*c2;
        p_y=a3*(c2*c3 - s2*s3) - a2*s2;
        p_z=joints(0) + a3*(c2*s1*s3 + c3*s1*s2) + a2*c2*s1;
    }else{
        std::cout << "Unexpected number of transformations: " << transformations << std::endl;
        std::cout << "Defaulting to standard algorithm " << std::endl;
        Real  DH[6][4]={{M_PI_2, 0.0    , joints(0), 0.0             },
                         {M_PI_2, 0.0    , 0.0      , 0.0   +joints(1)},
                         {0.0   , 0.34985, 0.0      , M_PI_2+joints(2)},
                         {M_PI_2, 0.0    , 0.0      , M_PI_2+joints(3)},
                         {M_PI_2, 0.0    , 0.398    , M_PI_2+joints(4)},
                         {0.0   , 0.0    , 0.0      , 0.0   +joints(5)}};
        eTransform tempframe;
        for(int i=0;i<transformations;i++){
            eTransform current_trans;
            Eigen::Matrix3f rot;
            Real al=DH[i][0];Real a=DH[i][1];Real d=DH[i][2];Real t=DH[i][3];
            rot << cos(t),-sin(t)*cos(al), sin(t)*sin(al),
                    sin(t), cos(t)*cos(al),-cos(t)*sin(al),
                    0.0   ,        sin(al),        cos(al);
            tempframe.linear()=rot;
            tempframe.translation()=Eigen::Vector3f(a*cos(t),a*sin(t),d);
            tempframe=tempframe*current_trans;//eTransform(DH[i][0],DH[i][1],DH[i][2],DH[i][3]);
        }
        return tempframe;
    }
    ///Pack the values into a Transformation matrix.
    eTransform tempframe;
    Eigen::Matrix3f rot;
    rot << r11,r12,r13,
           r21,r22,r23,
           r31,r32,r33;
    tempframe.linear()=rot;
    tempframe.translation()=Eigen::Vector3f(p_x,p_y,p_z);
    return tempframe;
}



/*!
 * \brief solve just the ik for a spherical wrist following the puma 560 wrist
 *
 * \warning This function only fills up joints 4 5 and 6, the others are left to you
 *
 * \note the initial_wrist should have the same position vector as the target pose for your life to make sense
 *
 * \param target         The target pose (6DOF, given as homogenous transform)
 * \param initial_wrist  The pose of the wrist assuming all wrist joints are zero.
 * \return               The joint positions of the wrist.
 */
ePose corvus_point::ik_wrist(eTransform &target, eTransform &initial_wrist){
    const Real epsilon=0.0001; //anything less than this is zero, for all we care
    ePose joints;
    // This is where we want joint 5's axis of rotation to be, to find it we use a cross product of the two z vectors
    Eigen::Vector3f z5=target.rotation().col(2).cross(initial_wrist.rotation().col(2));
    // But the cross product doesn't conserve magnitdue, so we need to renormalize (although check first for singular wrist)
    if(std::abs(z5.norm())<epsilon){
        // Singular condition for J4 J6

        z5.normalize();
        // Joints 4 and 6 both act about the same z axis, so the sum of the two is the difference between where we want
        // where our current y is and where we want our final y to be
        Real J4J6sum=get_angle(initial_wrist.rotation().col(1),target.rotation().col(1),initial_wrist.rotation().col(2));
        // It's up to us how to set J4 and J6, the world is our oyster, the only requirement is that they sum to J4J6sum.
        // Craig suggests that we use the previous version of J4, but since we are possibly using servos with only valid
        // ranges of -90 to +90 deg, we can just split the job 50/50, which will guarantee reachability, but not necessarily smoothness
        joints(4)=J4J6sum/2.0;
        joints(6)=J4J6sum/2.0;
    }else{
        z5.normalize();
        // Joint 4 is the difference between where we want joint 5's axis of rotation to be and where it is now
        // Note that is means you need to have joint 5's axis of rotation alligned with the y vector at zero pose
        joints(4)= get_angle( initial_wrist.rotation().col(1),z5,initial_wrist.rotation().col(2));
        // Joint 6 is the difference between where we want joint 5's axis of rotation to be and where we want our final y to be
        // Note that is means you need to have joint 5's axis of rotation alligned with the y vector at zero pose
        joints(6)=-get_angle(        target.rotation().col(1),z5,target.rotation().col(2));
    }
    // In order to make sure we get the whole range of values, we check against a normal vector to give a sign
    // Joint 5 is the most straihtforward, since it's just our desired z vector compared to what we have now:
    joints(5)=-get_angle(initial_wrist.rotation().col(2),target.rotation().col(2),z5);
    return joints;
}

// The nice thing about a wrist is that you have two options for any orientation.
// This should give you the other one...
ePose corvus_point::wrist_flip(ePose old_wrist){
    old_wrist(4)= old_wrist(4)+M_PI;
    old_wrist(5)=-old_wrist(5);
    old_wrist(6)= old_wrist(6)+M_PI;
    pose_unit_circle(old_wrist);
    return old_wrist;
}


void corvus_point::pose_unit_circle(ePose &joints){
    joints(1)=unit_circle(joints(1));
    joints(2)=unit_circle(joints(2));
    joints(3)=unit_circle(joints(3));
    joints(4)=unit_circle(joints(4));
    joints(5)=unit_circle(joints(5));
    joints(6)=unit_circle(joints(6));
}

Real corvus_point::unit_circle(Real theta){
    while(theta>M_PI){
        theta-=2*M_PI;
    }
    while(theta<-M_PI){
        theta+=2*M_PI;
    }
    return theta;
}


/*!
 * \brief  angle from current vector to vect_in
 * \param  vect_in vector to be compared
 * \return angle in radians
 */
Real corvus_point::get_angle(Eigen::Vector3f vect_in_1,Eigen::Vector3f vect_in_2,Eigen::Vector3f normal){
    if((vect_in_1.cross(vect_in_2)).dot(normal)>0)
        return +acos(vect_in_1.dot(vect_in_2));
    else
        return -acos(vect_in_1.dot(vect_in_2));
}

/*!
 * \brief inverse kinematics
 *
 * This function isn't fully optimized, right now it takes ~0.3ms to run
 * with N=200 and ~3ms with N=2000 (as you may have guessed, it's big O of N).
 * Things to optimize include:
 *  - Preallocate the iterations vector
 *  - Use iterators, not for loop
 *  - Hard-code only elbow up (cuts time in half)
 *  - Split up if(basic_ik function, since we are doing
 *  - Little tweaks probably not worth your time
 *
 * \param target    Target frame
 * \param old_pose  Previous pose, used to calculate cost of movement to find cheapest solution
 * \param div       Number of divisions to iterate through
 * \param verbose   If true, spits out info to stdout
 * \return success
 */
bool corvus_point::ik(eTransform target, ePose old_pose, int div,bool verbose,bool elbow_down){
    const Real big_error=1e38;
    const Real negligable_error=0.001;
    const int num_points=div+1;
    ///Put the coordinates in variables to make life easier to read
    Real x=target.translation()(0);
    Real y=target.translation()(1);
    Real z=target.translation()(2);
    Eigen::Vector3f target_z_dir=target.rotation().col(2);
    iteration_eigen itr_current;
    iteration_eigen itr_previous;
    iteration_eigen itr_best;
    ///Initialize error crazy high
    itr_best.movement_cost=big_error;
    itr_best.vector_error=big_error;
    int flips=0;
    iteration_eigen iterations[num_points];
    ///Try S=-1,S=+1 which correspond to +/-sqrt(), aka elbow up, elbow down
    int S_start=-1;
    if(!elbow_down){
        S_start=1;
    }
    for(int S=S_start;S<=1;S+=2){
        int i=0;
        itr_previous.movement_cost=big_error;
        itr_previous.vector_error=big_error;
        itr_previous.valid=false;
        ///Run through the length of the linear J0
        for(Real J0=0.0;  J0<=J0_MAX; J0+=J0_MAX/div){
            ///Solve for X,Y,Z since they are only a function of J0,J1,J2,J3
            ///Check if the arm can even reach the point
            iterations[i].valid=basic_ik(iterations[i].joints,x,y,z,J0,S);
            if(iterations[i].valid){
                ///Check how well the vector pointing out the end of the arm is normal to
                /// the vector we want pointing out of the end effector
                ///The following shows where the code comes from:
                //iterations[i].vector_error=trans(iterations[i].joints,4).rotation().col(2).dot(target.rotation().col(2));
                ///But this should be faster:
                iterations[i].vector_error=target_z_dir.dot(Eigen::Vector3f(-sin(iterations[i].joints(2)+iterations[i].joints(3))*cos(iterations[i].joints(1)),-cos(iterations[i].joints(2)+iterations[i].joints(3)),-sin(iterations[i].joints(2)+iterations[i].joints(3))*sin(iterations[i].joints(1))));
                if(verbose){
                    std::cout.precision(3);
                    std::cout.setf( std::ios::fixed, std:: ios::floatfield );
                    std::cout << std::endl << J0 << "," << iterations[i].vector_error;
                }
            }
            i++;
        }
        /// \todo We are finding the root of an equation, and there are
        /// tons of better ways to do that.
        /// Right now we are just using one linear interpolation, but
        /// that could be extended: http://en.wikipedia.org/wiki/False_position_method
        /// There are also lots of other interesting options out there
        /// http://en.wikipedia.org/wiki/Root-finding_algorithm
        /// There also may be libraries that can do this crazy-fast.
        for(int i=1;i<num_points;i++){
            bool crossed_zero=iterations[i].vector_error*iterations[i-1].vector_error<=0;
            bool both_valid=(iterations[i].valid && iterations[i-1].valid);
            /// The following checks to see if we are at the start or end of a valid range
            ///               (beginning/end of range)or(avoid invalid index   and adjacent to invalid    )
            bool next_invalid=(i+1>=num_points        || (i+1<num_points        && !iterations[i+1].valid));
            bool last_invalid=(i-2<0                  || (i-2>=0                && !iterations[i-2].valid));
            if(iterations[i].valid && std::abs(iterations[i].vector_error)<negligable_error){
                ///We found a point that's close enough to zero that we can assume it's probably good to go
                if(cost(iterations[i].joints,old_pose)<itr_best.movement_cost){
                    itr_best=iterations[i];
                    itr_best.movement_cost=cost(iterations[i].joints,old_pose);
                }
            }else if(both_valid && (crossed_zero || next_invalid || last_invalid)){
                ///We have crossed zero
                flips++;
                Real interpolated_J0=linear_interpolation(iterations[i-1].vector_error,iterations[i].vector_error,iterations[i-1].joints(0),iterations[i].joints(0),0.0);
                ///Check where the interpolated J0 is, and if it's more than two steps extrapolated, this is probably bad data and just run away.
                Real extrapolation_distance=std::max(interpolated_J0-iterations[i].joints(0),iterations[i-1].joints(0)-interpolated_J0);
                ePose joints;
                if(basic_ik(joints,x,y,z,interpolated_J0,S) && extrapolation_distance<(J0_MAX/div*2)){
                    if(cost(joints,old_pose)<itr_best.movement_cost){
                        if(verbose)
                            std::cout << ",<<--Bingo";
                        itr_best.joints=joints;
                        itr_best.movement_cost=cost(joints,old_pose);
                        itr_best.vector_error=0.0;
                        if(verbose){
                            std::cout.precision(3);
                            std::cout.setf( std::ios::fixed, std:: ios::floatfield );
                            std::cout << interpolated_J0 <<",";
                            std::cout.precision(3);
                            std::cout.setf( std::ios::fixed, std:: ios::floatfield );
                        }
                    }
//                }else{
//                    if(extrapolation_distance>=(J0_MAX/div*2)){

//                        std::cout << "Tried to extrapolate more than two steps" << std::endl;
//                    }else{
//                        std::cout << "Yikes!  Didn't see that coming!" << std::endl;
//                        //std::cout << "i=" <<  iterations[i].vector_error << ", i-1=" << iterations[i-1].vector_error << "" << std::endl;
//                    }
                }
            }
        }
    }

    if(verbose){
        std::cout << std::endl;
        std::cout << "flips=" << flips << std::endl;
    }
    ///If no reachable area was found, just give up.
    if(itr_best.vector_error>0.1)
        return false;
    joint_pose=itr_best.joints;
    joint_pose(4)=0.0;joint_pose(5)=0.0;
    /// \todo These are assuming that the vectors lie in the plane of rotation, but they probably don't, so these
    ///       could give innacurate info (more innacurate the larger itr_best.vector_error is, so we have the
    ///       threshold above)
    eTransform tempframe1=trans(joint_pose,6);
    eTransform tempframe2=trans(joint_pose,4);
    joint_pose(4)=-get_angle(target.rotation().col(2),tempframe1.rotation().col(2),tempframe2.rotation().col(2));
    eTransform tempframe3=trans(joint_pose,6);
    joint_pose(5)=-get_angle(target.rotation().col(1),tempframe3.rotation().col(1),tempframe3.rotation().col(2));
    //std::cout << joint_pose(0) << "  " << joint_pose(1) << "  " << joint_pose(2) << "  " << joint_pose(3) << "  " << joint_pose(4) << "  " << joint_pose(5) << std::endl;
    //std::cout << trans(joint_pose).matrix() << std::endl;
    //std::cout << target.matrix() << std::endl;
    //target_error=(target.matrix()-trans(joint_pose).matrix()).lpNorm<1>();
    target_error=0.0;
    //std::cout << target_error <<std::endl;
    //if(target_error>.01)
        //return false;
    return true;
}

/*!
 * \brief Helper function for when we only use 3 DOF
 *
 * This is assuming we are using the usual arm, still
 * in the plane of the two arm segments, with offsets:
 *                _
 *             X   |
 *             |   |height_offset
 *    ______.__|  _|
 *   /
 *  /       |  |
 * /        ----length_offset
 *
 * \warning this is redundant to the next one that allows all 3 offsets.
 * \warning DONT USE
 *
 * \param target        x,y,z of 'X' above
 * \param length_offset distance beyond normal wrist point along J4 axis
 * \param height_offset distance above normal wrist point along J3XJ4
 * \param width_offset  distance 'out of the page' along J3 axis
 * \return success
 */
bool corvus_point::ik(Eigen::Vector3f target,Real length_offset,Real height_offset,Real width_offset){
    Real e=sqrt((D_A3_A5+length_offset)*(D_A3_A5+length_offset)+height_offset*height_offset);
    Real x=target(0);
    Real y=target(1);
    Real z=target(2);
    bool success=basic_ik(joint_pose,x,y,z,0.0,1,D_A2_A3,e);
    joint_pose(3)+=asin(height_offset/e);
    joint_pose(4)=0.0;
    joint_pose(5)=0.0;
    fk();
    return success;
}

/*!
 * \brief IK function for when we only use 3 DOF
 *
 * This is assuming we are using the usual arm with offsets:
 *
 *          ----offset.y
 *          |  |
 *                  (offset.x into screen)
 *     =====+__   _
 *   //        |   |
 *  //         |   |offset.z
 * //          X  _|
 *
 * \note Offsets can of course be zero
 * \warning I'm not sure of the directions of the offsets, take this diagram with a grain of salt
 * \todo This should replace the other 3DOF ik's, since it can handle it all (although maybe a bit
 * slower because of the extra asin() and a couple extra multiplies/sqrts)
 *
 * \param target        x,y,z of 'X' above
 * \param offset        distance beyond normal wrist: x along J3 axis, y along J4, z along J3XJ4 axis
 * \return success
 */
bool corvus_point::ik(Eigen::Vector3f target,Eigen::Vector3f offset,int S,Real a2,Real a3){
    //const Real a2=0.34985;//!< Distance from J2 axis to J3 axis
    //const Real a3=0.398;  //!< Distance from J3 axis to J5 axis
    Real x=target(0);
    Real y=target(1);
    Real z=target(2);
    Real offset_x=offset(0);
    Real offset_y=offset(1);
    Real offset_z=offset(2);
    ///To deal with the y and z offsets, we are imagining a basic arm with only two links:
    ///Where the length of the second link is the distance from J3 axis to the point (ignoring offset_x)
    a3=sqrt((a3+offset_y)*(a3+offset_y)+offset_z*offset_z);
    joint_pose(0)=0.0;
    //z=z-joint_pose(0);
    ///Check if the arm can even reach the point (inner circle)
    if(x*x+z*z<=offset_x*offset_x)
        return false;
    ///Note that this is essentially the same as having the offset_x on between J1 and J2, so
    ///the results from the PUMA can be used for help visualizing the problem:
    ///Lee, C. G. & Ziegler, M. Geometric approach in solving inverse kinematics of PUMA robots, IEEE, 1984
    joint_pose(1)=-atan2(z,-x)-asin(offset_x/sqrt(x*x+z*z));
    ///Now we are projecting our target point onto the plane containing the arm, so in a plane
    ///we only care about 2D.  In our case, we consider y to be 'up' (well down, to be pedantic)
    ///so the other dimension will be considered x_prime, and is the distance of the projected
    ///point from the robot base.
    ///This could also be thought of as projecting to a plane off the robot center, to make it
    ///look like the PUMA, if that sort of thing helps you (and there are lots of pictures of the
    ///PUMA out there)
    ///\todo Make pretty diagrams of this
    Real x_prime=sqrt(x*x+z*z-offset_x*offset_x);
    Real D=(x_prime*x_prime+y*y-a2*a2-a3*a3)/(2*a2*a3);
    ///Check if the arm can even reach the point (outer circle)
    if(D<=1){
        joint_pose(3)=atan2(S*sqrt(1-D*D),D);
        joint_pose(2)=M_PI_2-(atan2(-y,x_prime)-atan2(a3*sin(-joint_pose(3)),a2+a3*cos(-joint_pose(3))));
        ///Correction for J3
        joint_pose(3)-=asin(offset_z/a3);
    }else{
        return false;
    }
    joint_pose(4)=0.0;
    joint_pose(5)=0.0;
    fk();
    ///Use this to verify that things are working as expected.  Should be zero (or 10^-16)
    //std::cout << (frame*offset-target).norm() << std::endl;
    return true;
}

bool corvus_point::redundant_joint_run_away_ik(Eigen::Vector3f target,Real theta,Real height,int S,Real a2,Real a3){
    const Real offset_z=0.1;//meters
    const Real offset_r=0.5;//meters
    Real best_J5=0;
    Real best_cost=-1e30;
    Eigen::Vector3f point_joint_space(height,0,0);
    Eigen::Vector3f direction_joint_space(0,sin(theta),cos(theta));
    eTransform joint_to_cart=trans(joint_pose,4);
    Eigen::Vector3f direction_cart_space=joint_to_cart.rotation()*direction_joint_space;
    Eigen::Vector3f point_cart_space=joint_to_cart*point_joint_space;
    for(Real J5=0;J5<2*M_PI;J5+=M_PI/100){
        if(ik(target,Eigen::Vector3f(offset_r*sin(J5),offset_r*cos(J5),offset_z),S,a2,a3)){
            joint_to_cart=trans(joint_pose,4);
            Eigen::Vector3f new_point_cart_space=joint_to_cart*point_joint_space;
            Real cost=(new_point_cart_space-point_cart_space).dot(direction_cart_space);
            if(cost>best_cost){
                best_cost=cost;
                best_J5=J5;
            }
        }
    }
    return ik(target,Eigen::Vector3f(offset_r*sin(best_J5),offset_r*cos(best_J5),offset_z),S,a2,a3);
}

/*!
 * \brief IK function for when we only use 5 DOF
 *
 * \todo  This could also be used with J0, where that has an increased cost.
 *
 * \param target_point  x,y,z
 * \param target_axis   vector along which we want our tool to be
 * \param offset_r      distance beyond normal wrist: along J5
 * \param offset_y      distance beyond normal wrist: along J4
 * \return success
 */
bool corvus_point::ik(Eigen::Vector3f target_point,Eigen::Vector3f target_axis,Real offset_r,Real offset_y,int S,Real a2,Real a3){
    const Real angle_increment=0.01;
    const Real dot_product_threshold=0.1;
    Real best_J4=0;
    Real best_dot_product=2.0*dot_product_threshold;
    ///Iterate through J4 options.
    ///\warning There could be two solutions each, so for now we only scan 180 degrees and only find
    ///one of them.  This should be handled like the 6DOF case, where we use a cost function to chose.
    for(Real J4=-M_PI_2;J4<M_PI_2;J4+=angle_increment){
        ///Check if target_point is reachable
        if(ik(target_point,Eigen::Vector3f(offset_r*sin(J4),offset_y,offset_r*cos(J4)),S,a2,a3)){
            joint_pose(4)=J4;
            fk();
            ///Check if target_z_axis is closer to normal with wrist z
            if(std::abs(target_axis.dot(frame.rotation().col(2)))<best_dot_product){
                ///This is our new best
                ///\todo this should do interpolation like the 6DOF case
                best_dot_product=std::abs(target_axis.dot(frame.rotation().col(2)));
                best_J4=J4;
            }
        }
    }
    if(ik(target_point,Eigen::Vector3f(offset_r*sin(best_J4),offset_y,offset_r*cos(best_J4)),S,a2,a3) && best_dot_product<dot_product_threshold){

        joint_pose(4)=best_J4;
        fk();
        joint_pose(5)=-get_angle(target_axis,frame.rotation().col(0),frame.rotation().col(2));
        fk();
        ///\todo Here we could expand this by solving for J6, which would be an image rotation for a camera, or one of the servos
        ///on Zach's grasper.  For a needle syringe driver we don't care.
        return true;
    }else{
        ///Couldn't get any points that could reach the point and had a dot product less than dot_product_threshold.  Sad day.
        return false;
    }
}

/*!
 * \brief translates cartesian velocity to joint space velocity for simple 3DOF case
 *
 * \warning uses numerical differentiation now, could be solved analytically pretty easily
 *
 * \param velocity  velocity in cartesian
 * \param offset    distance beyond normal wrist: x along J4 axis, y along J3XJ4, z along J3 axis
 * \param S         elbow up or elbow down
 * \param a2        Distance from J2 axis to J3 axis
 * \param a3        Distance from J3 axis to J5 axis
 * \return          if false, then velocity not achievable \todo not implemented
 */
bool corvus_point::ik_vel(Eigen::Vector3f velocity,Eigen::Vector3f offset,int S,Real a2,Real a3){
    corvus_point target_point;
    const Real d_t=0.001;
    Eigen::Vector3f d_pos=velocity*d_t; //!< This is m/s*s, which should give something on the order of .1mm (assuming .1m/s and dt=1ms) and be small enough to estimate velocity from accurately, but big enough to not have machine epsilon issues
    ///Maybe we should get 'origin' as an input in stead of using frame*offset?
    bool is_reachable=target_point.ik(frame*offset+d_pos,offset,S,a2,a3);
    joint_vel(0)=0.0;
    joint_vel(1)=(target_point.joint_pose(1)-joint_pose(1))/d_t; //!< answer should be in rad/s
    joint_vel(2)=(target_point.joint_pose(2)-joint_pose(2))/d_t; //!< answer should be in rad/s
    joint_vel(3)=(target_point.joint_pose(3)-joint_pose(3))/d_t; //!< answer should be in rad/s
    joint_vel(4)=0.0;
    joint_vel(5)=0.0;
    return is_reachable;
}

/*!
 * \brief Helper function for when we want the arm to be upside down
 *
 * This is assuming we are using the usual arm, still
 * in the plane of the two arm segments, with offsets:
 *
 *          ----length_offset
 *          |  |
 *
 *     _____.__   _
 *    /        |   |height_offset
 *   /         |   |
 *  /          X  _|
 *
 * \param target        x,y,z of 'X' above
 * \param length_offset distance beyond normal wrist point along J4 axis
 * \param height_offset distance above normal wrist point alonng J3XJ4
 * \return success
 */
bool corvus_point::upside_down_ik(Eigen::Vector3f target,Real length_offset,Real height_offset){
    bool success=ik(target,length_offset,-height_offset);
    joint_pose(2)=-joint_pose(2);
    joint_pose(3)=-joint_pose(3);
    if(joint_pose(1)>0)
        joint_pose(1)=joint_pose(1)-M_PI;
    else
        joint_pose(1)=joint_pose(1)+M_PI;
    fk();
    error=success && collision_detection(joint_pose);
    return error;
}

/*!
 * \brief Basic Inverse Kinematics
 *
 * Since the target position only affects joints 0, 1, 2 and 3,
 * if we have J0 as a given, we can solve for the other 3.
 * \param joints The joints, where the solution is placed
 * \param x x
 * \param y y
 * \param z z
 * \param J0 the given J0
 * \param S  elbow up or elbow down
 * \param a2 Distance from J2 axis to J3 axis
 * \param a3 Distance from J3 axis to J5 axis
 * \return if false, then point not reachable
 */
bool corvus_point::basic_ik(ePose &joints,Real x,Real y,Real z,Real J0,int S,Real a2,Real a3){
    //const Real a2=0.34985;//!< Distance from J2 axis to J3 axis
    //const Real a3=0.398;  //!< Distance from J3 axis to J5 axis
    z=z-J0;
    Real D=(z*z+x*x+y*y-a2*a2-a3*a3)/(2*a2*a3);
    ///Check if the arm can even reach the point
    if(D<=1){
        joints(0)=J0;
        joints(1)=-atan2(z,-x);
        joints(3)=atan2(S*sqrt(1-D*D),D);
        joints(2)=M_PI_2-(atan2(-y,sqrt(z*z+x*x))-atan2(a3*sin(-joints(3)),a2+a3*cos(-joints(3))));
        return true;
    }else{
        return false;
    }
}

/*!
 * \brief Basic Inverse Kinematic Velocity
 * \param joints The joint positions
 * \param joint_vels The joint velocities, where the solution is placed
 * \param cart_vel cartesian velocity (meters per second)
 * \param J0_vel the given J0 velocity
 * \param S  elbow up or elbow down
 * \param a2 Distance from J2 axis to J3 axis
 * \param a3 Distance from J3 axis to J5 axis
 * \return if false, then point not reachable
 */
bool corvus_point::basic_ik_vel(ePose joints,ePose &joint_vels,Eigen::Vector3f cart_vel,Real J0_vel,int S,Real a2,Real a3){
    //const Real a2=0.34985;//!< Distance from J2 axis to J3 axis
    //const Real a3=0.398;  //!< Distance from J3 axis to J5 axis
    Real dx=-cart_vel(0);
    Real dy=-cart_vel(2);
    Real dz=-cart_vel(1);
    joint_vels(0)=J0_vel;
    dz=dz-joint_vels(0);
    //joint_vels(1)=(2*dx*sin(joints(1)))/(a2*cos(2*joints(1) + joints(2)) - 2*a3*cos(joints(2) + joints(3)) + a2*cos(2*joints(1) - joints(2))) + (dy*cos(joints(1)))/(a3*cos(joints(2) + joints(3)) - a2*cos(2*joints(1))*cos(joints(2)));
    //joint_vels(2)=(dz*sin(joints(2) + joints(3)))/(a2*sin(joints(3))) + (dx*cos(joints(1))*(a3/2 - (a2*cos(joints(3)))/2 - (a2*cos(2*joints(2) + joints(3)))/2 + (a3*cos(2*joints(2) + 2*joints(3)))/2))/(a2*(a2*cos(joints(2))*sin(joints(3)) - a3*sin(joints(2)) + a3*cos(joints(3))*cos(joints(3))*sin(joints(2)) - 2*a2*cos(joints(1))*cos(joints(1))*cos(joints(2))*sin(joints(3)) + a3*cos(joints(2))*cos(joints(3))*sin(joints(3)))) + (dy*sin(joints(1))*(a3/2 + (a2*cos(joints(3)))/2 + (a2*cos(2*joints(2) + joints(3)))/2 + (a3*cos(2*joints(2) + 2*joints(3)))/2))/(a2*(a2*cos(joints(2))*sin(joints(3)) - a3*sin(joints(2)) + a3*cos(joints(3))*cos(joints(3))*sin(joints(2)) - 2*a2*cos(joints(1))*cos(joints(1))*cos(joints(2))*sin(joints(3)) + a3*cos(joints(2))*cos(joints(3))*sin(joints(3))));
    //joint_vels(3)=(dx*cos(joints(1))*((a2*a2*cos(2*joints(2)))/2 - (a3*a3*cos(2*joints(2) + 2*joints(3)))/2 + a2*a2/2 - a3*a3/2))/(a2*a3*(a2*cos(joints(2))*sin(joints(3)) - a3*sin(joints(2)) + a3*cos(joints(3))*cos(joints(3))*sin(joints(2)) - 2*a2*cos(joints(1))*cos(joints(1))*cos(joints(2))*sin(joints(3)) + a3*cos(joints(2))*cos(joints(3))*sin(joints(3)))) - (dy*sin(joints(1))*((a2*a2*cos(2*joints(2)))/2 + (a3*a3*cos(2*joints(2) + 2*joints(3)))/2 + a2*a2/2 + a3*a3/2 + a2*a3*cos(joints(3)) + a2*a3*cos(2*joints(2) + joints(3))))/(a2*a3*(a2*cos(joints(2))*sin(joints(3)) - a3*sin(joints(2)) + a3*cos(joints(3))*cos(joints(3))*sin(joints(2)) - 2*a2*cos(joints(1))*cos(joints(1))*cos(joints(2))*sin(joints(3)) + a3*cos(joints(2))*cos(joints(3))*sin(joints(3)))) - (dz*(a3*sin(joints(2) + joints(3)) + a2*sin(joints(2))))/(a2*a3*sin(joints(3)));
//    joint_vels(1)=(2*dx*sin(joints(1)))/(a2*cos(2*joints(1) + (M_PI_2-joints(2))) - 2*a3*cos((M_PI_2-joints(2)) + joints(3)) + a2*cos(2*joints(1) - (M_PI_2-joints(2)))) + (dy*cos(joints(1)))/(a3*cos((M_PI_2-joints(2)) + joints(3)) - a2*cos(2*joints(1))*cos((M_PI_2-joints(2))));
//    joint_vels(2)=(dz*sin((M_PI_2-joints(2)) + joints(3)))/(a2*sin(joints(3))) + (dx*cos(joints(1))*(a3/2 - (a2*cos(joints(3)))/2 - (a2*cos(2*(M_PI_2-joints(2)) + joints(3)))/2 + (a3*cos(2*(M_PI_2-joints(2)) + 2*joints(3)))/2))/(a2*(a2*cos((M_PI_2-joints(2)))*sin(joints(3)) - a3*sin((M_PI_2-joints(2))) + a3*cos(joints(3))*cos(joints(3))*sin((M_PI_2-joints(2))) - 2*a2*cos(joints(1))*cos(joints(1))*cos((M_PI_2-joints(2)))*sin(joints(3)) + a3*cos((M_PI_2-joints(2)))*cos(joints(3))*sin(joints(3)))) + (dy*sin(joints(1))*(a3/2 + (a2*cos(joints(3)))/2 + (a2*cos(2*(M_PI_2-joints(2)) + joints(3)))/2 + (a3*cos(2*(M_PI_2-joints(2)) + 2*joints(3)))/2))/(a2*(a2*cos((M_PI_2-joints(2)))*sin(joints(3)) - a3*sin((M_PI_2-joints(2))) + a3*cos(joints(3))*cos(joints(3))*sin((M_PI_2-joints(2))) - 2*a2*cos(joints(1))*cos(joints(1))*cos((M_PI_2-joints(2)))*sin(joints(3)) + a3*cos((M_PI_2-joints(2)))*cos(joints(3))*sin(joints(3))));
//    joint_vels(3)=(dx*cos(joints(1))*((a2*a2*cos(2*(M_PI_2-joints(2))))/2 - (a3*a3*cos(2*(M_PI_2-joints(2)) + 2*joints(3)))/2 + a2*a2/2 - a3*a3/2))/(a2*a3*(a2*cos((M_PI_2-joints(2)))*sin(joints(3)) - a3*sin((M_PI_2-joints(2))) + a3*cos(joints(3))*cos(joints(3))*sin((M_PI_2-joints(2))) - 2*a2*cos(joints(1))*cos(joints(1))*cos((M_PI_2-joints(2)))*sin(joints(3)) + a3*cos((M_PI_2-joints(2)))*cos(joints(3))*sin(joints(3)))) - (dy*sin(joints(1))*((a2*a2*cos(2*(M_PI_2-joints(2))))/2 + (a3*a3*cos(2*(M_PI_2-joints(2)) + 2*joints(3)))/2 + a2*a2/2 + a3*a3/2 + a2*a3*cos(joints(3)) + a2*a3*cos(2*(M_PI_2-joints(2)) + joints(3))))/(a2*a3*(a2*cos((M_PI_2-joints(2)))*sin(joints(3)) - a3*sin((M_PI_2-joints(2))) + a3*cos(joints(3))*cos(joints(3))*sin((M_PI_2-joints(2))) - 2*a2*cos(joints(1))*cos(joints(1))*cos((M_PI_2-joints(2)))*sin(joints(3)) + a3*cos((M_PI_2-joints(2)))*cos(joints(3))*sin(joints(3)))) - (dz*(a3*sin((M_PI_2-joints(2)) + joints(3)) + a2*sin((M_PI_2-joints(2)))))/(a2*a3*sin(joints(3)));
    Real T1=       joints(1);
    Real T2=M_PI_2-joints(2);
    Real T3=       joints(3);
    joint_vels(1)=(dy*cos(T1))/(a3*cos(T2 + T3) + a2*cos(T2)) - (dx*sin(T1))/(a3*cos(T2 + T3) + a2*cos(T2));
    joint_vels(2)=(dz*sin(T2 + T3))/(a2*sin(T3)) + (dx*cos(T2 + T3)*cos(T1))/(a2*sin(T3)) + (dy*cos(T2 + T3)*sin(T1))/(a2*sin(T3));
    joint_vels(3)=-(dz*(a3*sin(T2 + T3) + a2*sin(T2)))/(a2*a3*sin(T3)) - (dx*cos(T1)*(a3*cos(T2 + T3) + a2*cos(T2)))/(a2*a3*sin(T3)) - (dy*sin(T1)*(a3*cos(T2 + T3) + a2*cos(T2)))/(a2*a3*sin(T3));
    return true;
}

/*!
 * \brief corvus_point::fk
 * \return
 */
bool corvus_point::fk(void){
    frame=trans(joint_pose);
    return true;
}

/*!
 * \brief Linear interpolation, solving for y given two points
 * \param x0 first x
 * \param x1 second x
 * \param y0 first y
 * \param y1 second y
 * \param x
 * \return
 */
Real corvus_point::linear_interpolation(Real x0,Real x1,Real y0,Real y1,Real x){
    return y0+(y1-y0)*(x-x0)/(x1-x0);
}

corvus_point corvus_point::pvt_interpolation(corvus_point point_1,corvus_point point_2, Real time_in){
    //for(Real t=0;t<1;t+=0.01){
    Real a,b,dx,dy;
    dx=(point_2.get_time()-point_1.get_time());
    ///If dx is zero, we can't do any interpolation so just return the first point, for lack of a better plan
    if(dx==0.0)
        return point_1;
    Real t=(time_in-point_1.get_time())/dx;
    corvus_point point_out;
    for(int i=0;i<6;i++){
        ///These equations from: http://en.wikipedia.org/wiki/Spline_interpolation
        dy=(point_2.get_joint_pos(i)-point_1.get_joint_pos(i));
        a= point_1.get_joint_vel(i)*dx-dy;
        b=-point_2.get_joint_vel(i)*dx+dy;
        point_out.set_joint_pos(i,(1-t)*point_1.get_joint_pos(i)+t*point_2.get_joint_pos(i)+t*(1-t)*(a*(1-t)+b*t));
        point_out.set_joint_vel(i,dy/dx+(1-2*t)*(a*(1-t)+b*t)/dx+t*(1-t)*(b-a)/dx);
    }
    point_out.set_time(time_in);
    point_out.set_error(false);
    point_out.fk();
//    a= point_1.joint_vel(2)*(point_2.get_time()-point_1.get_time())-(point_2.joint_pose(2)-point_1.joint_pose(2));
//    b=-point_2.joint_vel(2)*(point_2.get_time()-point_1.get_time())+(point_2.joint_pose(2)-point_1.joint_pose(2));
//    point_out.joint_pose(2)=(1-t)*point_1.joint_pose(2)+t*point_2.joint_pose(2)+t*(1-t)*(a*(1-t)+b*t);
    return point_out;
}

/*!
 * \brief Cost of a move
 *
 * This function finds the cost of going from old_pose to
 * new_pose.
 * \todo This could be written with time as the cost, or energy,
 * or some combination thereof.
 * \param new_pose Proposed new pose
 * \param old_pose Current or previous pose
 * \return
 */
Real corvus_point::cost(ePose new_pose,ePose old_pose){
    //if(collision_detection(new_pose))
    //    return 2e38;
    //else
        return std::abs(new_pose(0)-old_pose(0));
}

/*!
 * \brief Get a human readable string with all the values of the point
 * \note this is for debugging
 * \return string with debug info
 */
std::string corvus_point::get_str(bool oneline){
    std::stringstream ss;
    if(oneline){
        //ss << "Position:";
        //ss << ","<<joint_pose(0);
        ss << ","<<joint_pose(1);
        ss << ","<<joint_pose(2);
        ss << ","<<joint_pose(3);
        //ss << ","<<joint_pose(4);
        //ss << ","<<joint_pose(5);
        //ss << ",Velocity:";
        //ss << ","<<joint_vel(0);
        ss << ","<<joint_vel(1);
        ss << ","<<joint_vel(2);
        ss << ","<<joint_vel(3);
        //ss << ","<<joint_vel(4);
        //ss << ","<<joint_vel(5);
    }else{
        ss << "Frame:"<<std::endl<<frame.matrix()<<std::endl;
        ss << "Pose:"<<std::endl;
        ss << "  J0="<<joint_pose(0)<<std::endl;
        ss << "  J1="<<joint_pose(1)<<std::endl;
        ss << "  J2="<<joint_pose(2)<<std::endl;
        ss << "  J3="<<joint_pose(3)<<std::endl;
        ss << "  J4="<<joint_pose(4)<<std::endl;
        ss << "  J5="<<joint_pose(5)<<std::endl;
        ss << "Velocity:"<<std::endl;
        ss << "  J0="<<joint_vel(0)<<std::endl;
        ss << "  J1="<<joint_vel(1)<<std::endl;
        ss << "  J2="<<joint_vel(2)<<std::endl;
        ss << "  J3="<<joint_vel(3)<<std::endl;
        ss << "  J4="<<joint_vel(4)<<std::endl;
        ss << "  J5="<<joint_vel(5)<<std::endl;
        ss << "Target Error: "<<target_error<<std::endl;
        ss << "Error: "<<error<<std::endl;
        ss << "Time: "<<time<<std::endl;
    }
    return ss.str();
}


/*! \brief get cartesion position \param i dimension \return position */
Real corvus_point::get_cart_pos(int i){return frame.translation()(i);}
/*! \brief set cartesion position \param i dimension \param val new value */
void corvus_point::set_cart_pos(int i,Real val){Eigen::Vector3f temp=frame.translation();temp(i)=val;frame.translation()=temp;}
/*! \brief get cartesion position as a vector \return position */
Eigen::Vector3f corvus_point::get_pos(void){return frame.translation();}
/*! \brief set cartesion position \param param vector with new values for x,y,z */
void corvus_point::set_pos(Eigen::Vector3f param){frame.translation()=param;}
/*! \brief set time \param time_in set private variable time */
void corvus_point::set_time(Real time_in){time=time_in;}
/*! \brief get time \return private variable time */
Real corvus_point::get_time(void){return time;}
/*! \brief get joint position \param joint joint number 0-5 \return joint value, meters or radians */
Real corvus_point::get_joint_pos(int joint){
    if(joint==0)
        return joint_pose(0);
    if(joint==1)
        return joint_pose(1);
    if(joint==2)
        return joint_pose(2);
    if(joint==3)
        return joint_pose(3);
    if(joint==4)
        return joint_pose(4);
    if(joint==5)
        return joint_pose(5);
    return 0.0;
}
/*! \brief set joint position \param joint joint number 0-5 \param pos joint value, meters or radians */
void corvus_point::set_joint_pos(int joint,Real pos){
    if(joint==0)
        joint_pose(0)=pos;
    if(joint==1)
        joint_pose(1)=pos;
    if(joint==2)
        joint_pose(2)=pos;
    if(joint==3)
        joint_pose(3)=pos;
    if(joint==4)
        joint_pose(4)=pos;
    if(joint==5)
        joint_pose(5)=pos;
}
/*! \brief get joint velocity \param joint joint number 0-5 \return joint value, meters/s or radians/s */
Real corvus_point::get_joint_vel(int joint){
    if(joint==0)
        return joint_vel(0);
    if(joint==1)
        return joint_vel(1);
    if(joint==2)
        return joint_vel(2);
    if(joint==3)
        return joint_vel(3);
    if(joint==4)
        return joint_vel(4);
    if(joint==5)
        return joint_vel(5);
    return 0.0;
}
/*! \brief set joint velocity \param joint joint number 0-5 \param vel joint value, meters/s or radians/s */
void corvus_point::set_joint_vel(int joint,Real vel){
    if(joint==0)
        joint_vel(0)=vel;
    if(joint==1)
        joint_vel(1)=vel;
    if(joint==2)
        joint_vel(2)=vel;
    if(joint==3)
        joint_vel(3)=vel;
    if(joint==4)
        joint_vel(4)=vel;
    if(joint==5)
        joint_vel(5)=vel;
}
/*! \brief get error state of point \return whether or not there is an error (if there is, don't use this point) */
bool corvus_point::get_error(void){return error;}
/*! \brief set error state of point \return whether or not there is an error (if there is, don't use this point) */
void corvus_point::set_error(bool err){error=err;}


eTransform corvus_point::random_rotation(void){
    Eigen::Vector3f rnd = Eigen::Vector3f::Random() * M_PI; // Random values from -pi to +pi
    Eigen::Matrix3f rot;
    rot = Eigen::AngleAxisf(rnd(0), Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(rnd(1), Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(rnd(2), Eigen::Vector3f::UnitZ());
    eTransform tempframe;
    tempframe.setIdentity();
    tempframe.linear()=rot;
    return tempframe;
}
