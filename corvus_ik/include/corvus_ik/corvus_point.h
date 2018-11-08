#ifndef CORVUS_POINT_H
#define CORVUS_POINT_H

#include "sstream"
#include "eigen3/Eigen/Eigen"

#define D_A2_A3 0.34985 //!< Distance from J2 axis to J3 axis
#define D_A3_A5 0.398   //!< Distance from J3 axis to J5 axis
#define J0_MAX  2.275   //!< Distance from 0 that J0 can go
#define J0_DIV  455     //!< Default number of divisions for J0 ik

#define Real float

#define eTransform Eigen::Transform<Real,3,Eigen::Isometry> // this is just an Eigen::Isometry3f
#define ePose Eigen::Matrix<Real, 6, 1>

//! The CORVUS Arm point class.
/*!
  This class defines the position of a CORVUS arm.  This means that this contains math
  relating to the physical layout of the arm, and is not portable to a different setup.
*/
class corvus_point
{
public:
    corvus_point(void);
    corvus_point(eTransform frame_in,ePose old_ePose,Real t=0.0);
    corvus_point(ePose new_ePose,Real t=0.0);
    void relative_move(Real dx,Real dy,Real dz);
    void relative_move(Eigen::Vector3f leap_hand);
    void run_away(Real height,Real theta,Real force);
    eTransform leap_to_base(Eigen::Vector3f leap_position=Eigen::Vector3f(0.0,0.050,0.121));
    int ik(void);
    bool collision_detection(ePose joints);
    eTransform trans(ePose joints,int transformations=6);
    ePose ik_wrist(eTransform &target, eTransform &initial_wrist);
    ePose wrist_flip(ePose old_wrist);
    void pose_unit_circle(ePose &joints);
    Real unit_circle(Real theta);
    Real  get_angle(Eigen::Vector3f vect_in_1,Eigen::Vector3f vect_in_2,Eigen::Vector3f normal);
    bool ik(eTransform target, ePose old_ePose, int div=J0_DIV,bool verbose=false,bool elbow_down=false);
    bool ik(Eigen::Vector3f target,Real length_offset,Real height_offset,Real width_offset=0);
    bool ik(    Eigen::Vector3f target  ,Eigen::Vector3f offset=Eigen::Vector3f(0.0,0.0,0.0),int S=1,Real a2=D_A2_A3,Real a3=D_A3_A5);
    bool redundant_joint_run_away_ik(Eigen::Vector3f target,Real theta,Real height,int S=1,Real a2=D_A2_A3,Real a3=D_A3_A5);
    bool ik(Eigen::Vector3f target_point,Eigen::Vector3f target_axis,Real offset_r,Real offset_y,int S=1,Real a2=D_A2_A3,Real a3=D_A3_A5);
    bool ik_vel(Eigen::Vector3f velocity,Eigen::Vector3f offset=Eigen::Vector3f(0.0,0.0,0.0),int S=1,Real a2=D_A2_A3,Real a3=D_A3_A5);
    bool upside_down_ik(Eigen::Vector3f target,Real length_offset,Real height_offset);
    bool basic_ik(ePose &joints,Real x,Real y,Real z,           Real J0,    int S=1,Real a2=D_A2_A3,Real a3=D_A3_A5);
    bool basic_ik_vel(ePose joints,ePose &joint_vels,Eigen::Vector3f cart_vel,Real J0_vel,int S=1,Real a2=D_A2_A3,Real a3=D_A3_A5);
    bool fk(void);
    Real linear_interpolation(Real x0,Real x1,Real y0,Real y1,Real x);
    corvus_point pvt_interpolation(corvus_point point_1,corvus_point point_2, Real time_in);
    Real cost(ePose new_ePose,ePose old_ePose);
    std::string get_str(bool oneline=false);
    Real get_cart_pos(int i);
    void set_cart_pos(int i,Real val);
    Eigen::Vector3f get_pos(void);
    void set_pos(Eigen::Vector3f param);
    void set_time(Real time_in);
    Real get_time(void);
    Real get_joint_pos(int joint);
    void set_joint_pos(int joint,Real pos);
    Real get_joint_vel(int joint);
    void set_joint_vel(int joint,Real vel);
    bool get_error(void);
    void set_error(bool err);
    eTransform random_rotation(void);

    friend std::ostream &operator<<( std::ostream &output, corvus_point &p )
    {
        output << p.get_str();
        return output;
    }


    ////! This holds an iteration, used for convenience in the analytical ik()
    struct iteration_eigen{
        ePose joints;            //!< This holds a CORVUS arm ePose, aka a point in joint space
        Real movement_cost;   //!< This holds the cost of a new ePose, relative to the previous ePose \warning No units established yet!
        Real vector_error;    //!< This is the error in the ePose, and the function we are finding roots of
        bool valid;             //!< This is false if the target is not in the reachable workspace for this iteration
    };

    eTransform frame;       ///< Cartesian Position: X,Y,Z, (Meters) Roll,Pitch,Yaw (Radians)
    ePose    joint_pose;  ///< Joint Position: J0 (Meters),J1,J2,J3,J4,J5 (Radians)
    ePose    joint_vel;  ///< Joint Velocity: J0 (Meters/second),J1,J2,J3,J4,J5 (Radians/second)
    Real  target_error;///< The error of the ik relative to the target \warning this has bad units, of meters plus unit vectors, please use carefully!
private:
    bool   error;        ///< Whether or not there is an error
    Real time;         ///< Absolute time in seconds \warning this can provide microsecond resolution for only ~136 years; plan accordingly.

};

#endif // CORVUS_POINT_H
