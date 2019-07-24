#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/tf.h"
#include <eigen3/Eigen/Eigen>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Bool.h"
#include "math.h"
#include "limits.h"
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Eigenvalues>


/// Boost time, for tic() toc()
#include "boost/date_time/posix_time/posix_time.hpp" ///!< include all types plus i/o

/// Silly little timing functions, to get real CPU time, not ROS time
boost::posix_time::ptime start_tic_toc[10];
inline void tic(int i=0){start_tic_toc[i]=boost::posix_time::microsec_clock::universal_time();}
inline double toc(int i=0){return ((double)(boost::posix_time::microsec_clock::universal_time()-start_tic_toc[i]).total_microseconds())/1000000.0;}


/// The node is publishing the following
ros::Publisher pub_map; /// This is a point cloud
ros::Publisher pub_ellipse_robot_vis; /// This is a point cloud
ros::Publisher pub_ellipse_feature_vis; /// This is a point cloud
ros::Publisher pub_robot_state; /// This is the x- and y- of the robot
ros::Publisher pub_robot_state_w_cov; /// This is the x- and y- of the robot
ros::Publisher pub_feature_w_cov; /// This is the x- and y- of the robot
tf::TransformBroadcaster *pose_broadcaster; /// This is a tf
tf::TransformListener *listener;

/// ROS params
float update_thresh         = 1.0;    /// Mahalanobis lower thresh
float new_feature_thresh    = 10.0;   /// Mahalanobis upper thresh
int   num_consecutive_errs  = 2;      /// How many times do we have to see something where it doesn't belong before we give up

float ekf_distance_noise    = 0.500;  /// What is the noise in the distance aspect of the sensor, in meters. For VLP 16, +/- 3cm
float ekf_bearing_noise     = 0.020;  /// What is the noise in the bearing aspect of the sensor, in radians. For VLP 16, +/- 0.4 deg
float ekf_phi_noise         = 0.100;  /// What is the noise in the sensor's ability to measure the agular pose of the target, in radians. Dependant upon how you determine the angle of the target.
float ekf_propogate_noise   = 0.010;

bool publish_pointclouds    = true;
std::string world_frame     = "world";

int  consecutive_err_count  = 0;

/// enum setup
enum robot_state{
    ROBOT_X=0,
    ROBOT_Y=1,
    ROBOT_P=2,
    ROBOT_SIZE=3
};

/// enum setup
enum landmark_state{
    LANDMARK_X=0,
    LANDMARK_Y=1,
    LANDMARK_P=2,
    LANDMARK_SIZE=3
};

const int UNSURE = -1;
const int NEW_FEATURE = -2;

/// Global variables
Eigen::VectorXf state;
Eigen::MatrixXf covariance;
std::vector<std::string> landmark_ids;
sensor_msgs::PointCloud ellipse_vis_array;
ros::Time current_time, last_time;
Eigen::Matrix2f Q;      /// Q matrix for propagate
Eigen::MatrixXf Phi;    /// Phi matrix for propagate
Eigen::MatrixXf G;      /// G matrix for propagate
Eigen::Matrix2f J;      /// Fixed rotation matrix J for update
Eigen::MatrixXf R;      /// R matrix for update

bool reset_state=false;

void create_ellipse(Eigen::VectorXf xy_vector, Eigen::MatrixXf cov_matrix, sensor_msgs::PointCloud &ellipse_msg){

    /// Declare local variables for the function
    int r = 3;
    float dTheta = 0.1;                         /// Resolution of point cloud of plotted ellipse
    float ellipse_range = M_PI*2;               /// Range of the ellipse
    Eigen::Vector2f ellipse_vector;
    geometry_msgs::Point32 ellipse_pt;

    // Plotting error ellipses, Mark Addition
    Eigen::EigenSolver<Eigen::MatrixXf> es(cov_matrix);

    Eigen::MatrixXf I = r*Eigen::MatrixXf::Identity(cov_matrix.rows(),cov_matrix.rows());
    Eigen::MatrixXcf Eval = es.eigenvalues().asDiagonal();
    Eigen::MatrixXcf Evec = es.eigenvectors();
    Eigen::Vector2cf e1;
    Eigen::Vector2cf temp;
    ellipse_msg.header.frame_id = world_frame;
    ellipse_msg.header.stamp = ros::Time::now();
    for(int ii = 0; ii< ellipse_range/dTheta; ii++){
        /// Populate the "rotation" vector to get into cartesian
        e1(0) = std::cos(ii*dTheta);
        e1(1) = std::sin(ii*dTheta);
        temp = I.cast<std::complex<float> >() * Evec * Eval.cwiseSqrt() * e1;
        ellipse_vector = xy_vector + temp.real();
        ellipse_pt.x = ellipse_vector(0);
        ellipse_pt.y = ellipse_vector(1);
        ellipse_msg.points.push_back(ellipse_pt);
    }

}



/// Publish the error ellipse
void publish_ellipse(const ros::TimerEvent&){

    /// Publishing Robot state error ellipse
    /// Create a blank point cloud
    sensor_msgs::PointCloud ellipse_robot_msg;

    create_ellipse(state.head(2),covariance.block<2,2>(0,0),ellipse_robot_msg);

    pub_ellipse_robot_vis.publish(ellipse_robot_msg);

    /// -----------------------------
    ///  Publishing feature state error ellipse

    // Create a blank point cloud
    sensor_msgs::PointCloud ellipse_feature_msg;
    int num_features = (state.size()-ROBOT_SIZE)/LANDMARK_SIZE;         /// Current number of features in the state vector
    /// FOR LOOP for features
    for(int ii = 0; ii < num_features; ii++){
        int feature_index = LANDMARK_SIZE*ii + ROBOT_SIZE;
        create_ellipse(state.segment(feature_index,2),covariance.block<2,2>(feature_index,feature_index),ellipse_feature_msg);
    }
    if(num_features > 0){
        /// Only publish if we have at least one feature
        pub_ellipse_feature_vis.publish(ellipse_feature_msg);
    }
    // End Mark Addition

}

void publish_landmark_tf(const ros::TimerEvent&)
{
    int num_features = (state.size()-ROBOT_SIZE)/LANDMARK_SIZE;         /// Current number of features in the state vector

    /// Loop through all of the features in the entire state vector to figure out if you have seen the feature previously
    for(int ii = 0; ii<num_features;ii++){
        geometry_msgs::TransformStamped robot_pose;
        robot_pose.header.stamp = current_time;
        robot_pose.header.frame_id = world_frame; //!< the solid world coordinate frame that our state is in
        robot_pose.child_frame_id = landmark_ids.at(ii); //!< the name of the landmark is the QR Text
        robot_pose.transform.translation.x=state[ROBOT_SIZE+ii*LANDMARK_SIZE+LANDMARK_X];
        robot_pose.transform.translation.y=state[ROBOT_SIZE+ii*LANDMARK_SIZE+LANDMARK_Y];

        geometry_msgs::Quaternion robot_quat = tf::createQuaternionMsgFromYaw(state[ROBOT_SIZE+ii*LANDMARK_SIZE+LANDMARK_P]);
        robot_pose.transform.rotation=robot_quat;
        pose_broadcaster->sendTransform(robot_pose);
    }
}

int get_feature_index(std::string id)
{
    int feature_index = UNSURE;
    int num_features = (state.size()-ROBOT_SIZE)/LANDMARK_SIZE;         /// Current number of features in the state vector
    
    /// Loop through all of the features in the entire state vector to figure out if you have seen the feature previously
    for(int ii = 0; ii<num_features;ii++){
        if(landmark_ids.at(ii)==id){
            feature_index=ii*LANDMARK_SIZE+ROBOT_SIZE;
        }
    }
    return feature_index;
}

void resetSlam()
{

    /// eliminate all but robot x,y,phi
    state.resize(ROBOT_SIZE);
    state[ROBOT_X]=0;
    state[ROBOT_Y]=0;
    state[ROBOT_P]=0;
    covariance.resize(ROBOT_SIZE,ROBOT_SIZE);
    covariance = Eigen::MatrixXf::Zero(ROBOT_SIZE,ROBOT_SIZE);

    landmark_ids.resize(0);
}

void initializeSlam(float distance_noise,float bearing_noise,float phi_noise,float propogate_noise)
{

    /// Initialize all my variables
    J << 0,-1,
         1, 0;
    Q = propogate_noise*Eigen::MatrixXf::Identity(2,2);
    R = Eigen::MatrixXf::Identity(LANDMARK_SIZE,LANDMARK_SIZE);
    R(LANDMARK_X,LANDMARK_X) = distance_noise; // Distance noise
    R(LANDMARK_Y,LANDMARK_Y) = bearing_noise;  // Bearing noise
    R(LANDMARK_P,LANDMARK_P) = phi_noise;      // Phi noise

    /// Set robot pose to 0,0,0
    resetSlam();

}


Eigen::Matrix2f Get_C(float th)
{
    Eigen::Matrix2f C;
    
    /// Formulate a rotation matrix from angle
    C(0,0) =  std::cos(th);
    C(0,1) = -std::sin(th);
    C(1,0) =  std::sin(th);
    C(1,1) =  std::cos(th);
    
    return C;
}

Eigen::MatrixXf Get_R_mapped(geometry_msgs::Point32 pt)
{
    
    /// Declare variables
    Eigen::MatrixXf R_mapped;
    Eigen::MatrixXf G_update((int)LANDMARK_SIZE,(int)LANDMARK_SIZE);

    
    float feature_th = std::atan2(pt.y,pt.x);
    float feature_d = std::sqrt(pt.x*pt.x+pt.y*pt.y);
    /// G matrix to map the noise
    G_update(0,0) = std::cos(feature_th);
    G_update(0,1) = -feature_d*std::sin(feature_th);
    G_update(1,0) = std::sin(feature_th);
    G_update(1,1) = feature_d*std::cos(feature_th);
    G_update(0,2) = 0.0;
    G_update(1,2) = 0.0;
    G_update(2,2) = 1.0;
    G_update(2,0) = 0.0;
    G_update(2,1) = 0.0;
    R_mapped = G_update*R*G_update.transpose();
    
    return R_mapped;
}

void add_feature(geometry_msgs::Point32 pt,float phi, std::string current_landmark_id)
{
    /// We have a new feature!!
    ///
    /// Get the angle of the robot state from the state vector
    Eigen::Matrix2f C = Get_C(state[ROBOT_P]);

    Eigen::MatrixXf Rp = Get_R_mapped(pt);
    
    //float mahalanobis_current_min = std::numeric_limits<float>::max();
    Eigen::Vector2f robot_state = state.head(ROBOT_Y+1);        /// Current robot state (x and y)
    Eigen::Vector3f z_m;                                    	/// This is the measurement of the feature in robot frame
    /// Define the actual measurement in a vector
    z_m(LANDMARK_X) = pt.x;
    z_m(LANDMARK_Y) = pt.y;
    z_m(LANDMARK_P) = phi;

    /// Update the state vector by adding in the new feature (x,y,phi)
    Eigen::Vector2f feature_update_x_y;
    feature_update_x_y = C*z_m.head(LANDMARK_Y+1) + robot_state;
    state.conservativeResize(state.rows()+LANDMARK_SIZE);  /// Resize the state vector to make space for the new feature (x,y,phi)
    state(state.rows()-LANDMARK_SIZE + LANDMARK_X) = feature_update_x_y(LANDMARK_X);
    state(state.rows()-LANDMARK_SIZE + LANDMARK_Y) = feature_update_x_y(LANDMARK_Y);
    state(state.rows()-LANDMARK_SIZE + LANDMARK_P) = state(ROBOT_P) + z_m(LANDMARK_P);


    /// Update the covariance
    /// Calculate H_r and H_l
    Eigen::MatrixXf H_r((int)LANDMARK_SIZE,(int)ROBOT_SIZE);
    H_r.block(LANDMARK_X,ROBOT_X,LANDMARK_Y+1,ROBOT_Y+1) = -C.transpose();
    H_r.block(LANDMARK_X,ROBOT_P,LANDMARK_Y+1,1) = -C.transpose()*J*(C*z_m.head(LANDMARK_Y+1));
    H_r(2,0) = 0.0;
    H_r(2,1) = 0.0;
    H_r(2,2) = -1.0;
    Eigen::MatrixXf H_l((int)LANDMARK_SIZE,(int)LANDMARK_SIZE);
    H_l.block(LANDMARK_X,LANDMARK_X,LANDMARK_Y+1,LANDMARK_Y+1) = C.transpose();
    H_l(0,2) = 0.0;
    H_l(1,2) = 0.0;
    H_l(LANDMARK_P,LANDMARK_P) = 1.0;
    H_l(LANDMARK_P,0) = 0.0;
    H_l(LANDMARK_P,1) = 0.0;


    //std::cout << "Covariance Before: " << covariance << std::endl;

    /// Segment out the covariance matrix of just the robot
    Eigen::Matrix3f P_rr = covariance.block<ROBOT_SIZE,ROBOT_SIZE>(0,0);

    /// Calculate the off-diagonal block
    int cov_rows = covariance.rows();
    Eigen::MatrixXf cross_term = -covariance.block(0,0,cov_rows,LANDMARK_SIZE)*H_r.transpose()*H_l;

    /// Populate new covariance matrix
    covariance.conservativeResize(cov_rows+LANDMARK_SIZE,cov_rows+LANDMARK_SIZE); /// Resize the covariance matrix to add 3 more rows and cols
    covariance.block(0,cov_rows,cov_rows,LANDMARK_SIZE) = cross_term;
    covariance.block(cov_rows,0,LANDMARK_SIZE,cov_rows) = cross_term.transpose();
    covariance.block(cov_rows,cov_rows,LANDMARK_SIZE,LANDMARK_SIZE) = H_l.transpose()*(H_r*P_rr*H_r.transpose() + Rp)*H_l;


    //std::cout << "Covariance After: " << covariance << std::endl;


    /// Add the new id to the array
    landmark_ids.push_back(current_landmark_id);

    /// Print statement
//    std::cout << "We added a new feature to the state vector, we now have: " << covariance.cols() << " states" << std::endl;
}

/// Mahalanobis distance function
int Mahalanobis_dist(Eigen::MatrixXf Rp, geometry_msgs::Point32 pt, Eigen::Matrix2f C){

    /// Declare constants
    bool DEBUG=false;                                               /// To output degbug info
    const int DIMS=2;                                               /// We are using 2d metric, so we are ignoring rotation

    /// Declare variables
    int feature_index = UNSURE;                                         /// By default send UNSURE
    float mahalanobis_current_min = std::numeric_limits<float>::max(); /// Initialize the distance to something huge
    Eigen::Vector2f robot_state = state.head(DIMS);                 /// Current robot state
    int num_features = (state.size()-ROBOT_SIZE)/LANDMARK_SIZE;     /// Current number of features in the state vector
    Eigen::Vector2f z_feature;                                      /// This is the estimate of the feature from the state vector
    Eigen::Vector2f z_hat_feature;                                  /// This is the expected measurement from geometry in robot frame
    Eigen::Vector2f z_m;                                            /// This is the measurement of the feature in robot frame
    Eigen::Vector2f res;                                            /// This is the residual of the measurement and estimate
    Eigen::MatrixXf H(DIMS,LANDMARK_SIZE*num_features+ROBOT_SIZE);  /// This is the H matrix
    Eigen::MatrixXf S;                                              /// This is the H matrix
    H = Eigen::MatrixXf::Zero(DIMS,LANDMARK_SIZE*num_features+ROBOT_SIZE); /// Initialize the matrix to zero
    H.block(0,0,DIMS,DIMS) = -C.transpose();                        /// Set the top left corner of the matrix to -C'

    /// Loop through all of the features in the entire state vector to figure out if you have seen the feature previously
    for(int ii = 0; ii<num_features;ii++){
        /// Get estimate of the measurement from robot to feature from state vector
        z_feature(0) = state(LANDMARK_SIZE*ii+ROBOT_SIZE);
        z_feature(1) = state(LANDMARK_SIZE*ii+(ROBOT_SIZE+1));
        z_hat_feature = C.transpose()*(z_feature-robot_state);
        /// Define the actual measuremetn in a vector
        z_m(0) = pt.x;
        z_m(1) = pt.y;
        /// Calculate the residual
        res = z_m-z_hat_feature;
        /// Populate H matrix
        Eigen::MatrixXf H_r_partial(DIMS,1);
        H_r_partial = -C.transpose()*J*(z_feature-robot_state);
        H.block(0,DIMS,DIMS,1) = H_r_partial;
        H.block(0,LANDMARK_SIZE*ii+ROBOT_SIZE,DIMS,DIMS) = C.transpose();
        /// Get the x,y part of Rp
        Eigen::Matrix2f Rp_partial=Rp.block(0,0,DIMS,DIMS);
        /// Calculate the S matrix
        S = H*covariance*H.transpose() + Rp_partial;
        /// Check the condition number of a matrix
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(S);
        float cond = svd.singularValues()(0)/ svd.singularValues()(svd.singularValues().size()-1);
        /// Maybe check the condition number on S here????
        if( abs(cond) < 80){
            float gamma_mahalanobis = res.transpose()*S.inverse()*res;

//            gamma_mahalanobis = res.transpose()*res; /// This is Euclidean distance
            /// Check to see if the mahalanobis distance is less than the current minimum
            if(gamma_mahalanobis < mahalanobis_current_min){
                mahalanobis_current_min = gamma_mahalanobis;
                feature_index = LANDMARK_SIZE*ii+ROBOT_SIZE; /// The index in the state feature related to the smallest Mahalanobis distance
            }
        }
    }

    /// Figure out if the mahalanobis distance is good for the update, new feature, or nothing
    if (mahalanobis_current_min > new_feature_thresh && feature_index>UNSURE){
        /// We have a new feature!!
        ROS_INFO_COND(DEBUG,"Found new feature, closest feature mahalanobis_current_min=%f",mahalanobis_current_min);
        feature_index = NEW_FEATURE;    }
    else if((mahalanobis_current_min >= update_thresh && mahalanobis_current_min <= new_feature_thresh) || feature_index <= UNSURE){
        /// It is too hard to tell if it is a new feature or old feature, therefore do not update!
        ROS_INFO_COND(DEBUG,"Unknown if new feature, closest feature mahalanobis_current_min=%f",mahalanobis_current_min);
        feature_index = UNSURE;
    }
    else{
        /// This is probably an old feature, send the index so we know which one
        ROS_INFO_COND(DEBUG,"Found old feature, closest feature id=%d, mahalanobis_current_min=%f",(feature_index-ROBOT_SIZE)/LANDMARK_SIZE,mahalanobis_current_min);
    }

    return feature_index;
}

void update_feature(geometry_msgs::Point32 pt, float phi, int feature_index)
{
    Eigen::MatrixXf R_mapped = Get_R_mapped(pt);
    
    /// Get the angle of the robot state from the state vector
    Eigen::Matrix2f C = Get_C(state[ROBOT_P]);

    /// Define the actual measurement in a vector
    Eigen::Vector3f z_m;
    z_m(LANDMARK_X) = pt.x;
    z_m(LANDMARK_Y) = pt.y;
    z_m(LANDMARK_P) = phi;

    /// Define the best measurement
    Eigen::Vector2f z_hat_pos = C.transpose()*(state.segment(feature_index,2)-state.head(2)); /// This checks out: z_hat = C'*(p_l_hat - p_r_hat) Stergios notes 8.4 p.119

    Eigen::Vector3f z_hat;
    z_hat(LANDMARK_X) = z_hat_pos(LANDMARK_X);
    z_hat(LANDMARK_Y) = z_hat_pos(LANDMARK_Y);
    z_hat(LANDMARK_P) = state[feature_index + LANDMARK_P] - state[ROBOT_P];

    /// Only bother to check mahalanobis distance if we want to try throwing out outlier updates.
    /// If num_consecutive_errs<=0 then we would always proceed anyways, so save time by skipping the maths.
    if(num_consecutive_errs>0)
    {

        /// Check Mahalanobis distance
        int mahalanobis_feature_index = Mahalanobis_dist(R_mapped,pt,C);  /// Returns a feature_index of the feature in the state vector (-1 means unsure, -2 means new feature)

        /// For now, we can just exit if we aren't sure it's a
        if(mahalanobis_feature_index == NEW_FEATURE || mahalanobis_feature_index == UNSURE){
            /// We saw the feature not quite where we expected
            /// \todo Decide if UNSURE counts as an error for consecutive_err_count or not. If not, we could get stuck not quite seeing it but also not updating.
            consecutive_err_count++;
            float bearing_error=(std::atan2(pt.y,pt.x)-std::atan2(z_hat(LANDMARK_Y),z_hat(LANDMARK_X)))*180.0/M_PI;
            float dist_error=std::sqrt(pow(pt.x-z_hat(LANDMARK_X),2)+pow(pt.y-z_hat(LANDMARK_Y),2));
            ROS_INFO_COND(!reset_state,"Unexpected observation. Err %f deg, %f m.",bearing_error,dist_error);

            /// Check for lots of errors
            if(consecutive_err_count > num_consecutive_errs){
                /// We haven't seen the feature where we thought we would AND we've seen it elsewhere a lot.
                /// Maybe we ARE seeing the feature, we just have a bad state estimate
                //ROS_WARN("Repeated unexpected observation. Expected %f,%f but saw %f,%f.",z_hat(LANDMARK_X),z_hat(LANDMARK_Y),pt.x,pt.y);
                float bearing_error=(std::atan2(pt.y,pt.x)-std::atan2(z_hat(LANDMARK_Y),z_hat(LANDMARK_X)))*180.0/M_PI;
                float dist_error=std::sqrt(pow(pt.x-z_hat(LANDMARK_X),2)+pow(pt.y-z_hat(LANDMARK_Y),2));
                ROS_WARN_COND(!reset_state,"Repeated unexpected observation. Err %f deg, %f m.",bearing_error,dist_error);

                /// Allow the feature update to go through.
            }else{
                /// Don't update the feature, since we aren't sure what we saw anyways. This way we are robust to a few outlier updates.
                return;
            }
        }else{
            /// We saw the feature
            consecutive_err_count = 0;
        }
    }

    /// Update the covariance
    /// Calculate H matrix
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(LANDMARK_SIZE,state.rows());
    H.block(LANDMARK_X,ROBOT_X,LANDMARK_Y+1,ROBOT_Y+1) = -C.transpose();
    H.block(LANDMARK_X,ROBOT_P,LANDMARK_Y+1,1) = -C.transpose()*J*(state.segment(feature_index,2)-state.head(2));
    H.block(0,feature_index,2,2) = C.transpose();
    H(LANDMARK_P,ROBOT_P)=-1.0;
    H(LANDMARK_P,feature_index+LANDMARK_P)=1.0;

    /// Residual
    Eigen::Vector3f res;
    res = z_m - z_hat;

    /// Residual Covariance;
    Eigen::MatrixXf S;
    S = H*covariance*H.transpose() + R_mapped;

    bool check_cond_num = false;
    
    float cond=0.0;

    if(check_cond_num)
    {
        /// Check the condition number of a matrix
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(S);
        cond = svd.singularValues()(0)/ svd.singularValues()(svd.singularValues().size()-1);
    }
    /// Maybe check the condition number on S here????
    if(abs(cond) < 80){
        /// Kalman Gain
        Eigen::MatrixXf K;
        K = covariance*H.transpose()*S.inverse();
        /// State update
        state = state + K*res;

        /// Covariance update
        Eigen::MatrixXf identity_minus_kh;
        identity_minus_kh = Eigen::MatrixXf::Identity(covariance.rows(),covariance.cols()) - K*H;
        covariance = identity_minus_kh*covariance*identity_minus_kh.transpose() + K*R_mapped*K.transpose();

        //std::cout << "Covariance: "<< std::endl << covariance << std::endl;
    }
}

double get_yaw(geometry_msgs::Transform trans)
{
    tf::Quaternion q(
        trans.rotation.x,
        trans.rotation.y,
        trans.rotation.z,
        trans.rotation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

/// Callback function for receiving a point feature
void featureCallback(const geometry_msgs::TransformStamped::ConstPtr& feature){
    /// \todo This should be transformed into base_link frame if it is not already. But R_mapped expects distance/bearing measurements, so be careful with that.

    /// Get the point from each feature detected
    geometry_msgs::Point32 pt;

    pt.x = feature->transform.translation.x;
    pt.y = feature->transform.translation.y;
    /// Get the orientation of the feature
    float phi = get_yaw(feature->transform);
    std::string id = feature->child_frame_id;

    /// Check feature in map
    int feature_index = get_feature_index(id);  /// Returns a feature_index of the feature in the state vector (-1 means nothing to update)

    //std::cout << std::endl << "I saw " << id << " at " << pt.x << ",\t" << pt.y << ",\t" << phi << std::endl << std::endl;

    if (feature_index != -1){ /// This is a point I have already seen, let's do an update!
        update_feature(pt,phi,feature_index);
    }else{
        add_feature(pt,phi,id);
    }

    if(publish_pointclouds){
        /// Now output a cloud of states for viz purposes
        sensor_msgs::PointCloud map_features;
        map_features.header.frame_id = world_frame;
        map_features.header.stamp = ros::Time::now();
        int num_features_now = (state.size()-ROBOT_SIZE)/LANDMARK_SIZE;
        for(int jj = 0; jj < num_features_now; jj++){
            geometry_msgs::Point32 p1;
            p1.x=state(LANDMARK_SIZE*jj+ROBOT_SIZE+ROBOT_X);
            p1.y=state(LANDMARK_SIZE*jj+ROBOT_SIZE+ROBOT_Y);
            map_features.points.push_back(p1);
        }
        pub_map.publish(map_features);
    }
}

/*!
 * \brief featureArrayCallback
 *
 * This is a planned future implimentation, where we do a batch update.
 * This would save processing time, but that may not matter much compared to other nodes anyways.
 *
 * \param path A series of landmarks, stuffed into a path message, with landmark ids in pose header frameids. THIS IS BAD!
 * \todo should change to a custom message that's just an array of TransformStamped msgs.
 */
void featureArrayCallback(const nav_msgs::Path::ConstPtr& path)
{
    /// This is silly, since it just loops through anyways, but it proves that it works.
    for(int ii=0;ii<path->poses.size();ii++)
    {
        geometry_msgs::TransformStamped::Ptr feature(new geometry_msgs::TransformStamped);
        feature->transform.translation.x=path->poses[ii].pose.position.x;
        feature->transform.translation.y=path->poses[ii].pose.position.y;
        feature->transform.rotation.z=path->poses[ii].pose.position.z;
        /// This is a bit hacky. Probably should do this differently.
        feature->header=path->header;
        feature->child_frame_id=path->poses[ii].header.frame_id;
        featureCallback(feature);
    }
    
    /// First, loop through all features that are already in the state vector, and do an update.
    for(int ii=0;ii<path->poses.size();ii++)
    {
    
    }
    
    /// Then, loop through all features that are new, and add them to the state vector
    for(int ii=0;ii<path->poses.size();ii++)
    {
    
    }
}

/*!
 * \brief reset the ekf
 *
 * If the message is true, we reset the EKF as if rebooted.
 *
 * \param reset
 */
void resetCallback(const std_msgs::Bool::ConstPtr& reset)
{
    //Save current state
    reset_state = reset->data;
    if(reset_state){
        resetSlam();
        consecutive_err_count = 0;
    }
}

/// Callback function for receiving an odometry reading
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){

    /// Get the linear and angular velocity from odom
    float vx = odom->twist.twist.linear.x;
    float vth = odom->twist.twist.angular.z;
    float th = state[ROBOT_P];
    current_time = ros::Time::now();
    //compute odometry in a typical way given the velocities of the robot
    float dt = (current_time - last_time).toSec();
    if(dt>0.001 && dt< 1.000){

        /// ------- Propagating the states --------------///
        /// Calculate the change in states
        float delta_x = (vx * std::cos(th)) * dt;
        float delta_y = (vx * std::sin(th)) * dt;
        float delta_th = vth * dt;

        /// Update the states
        state[ROBOT_X] += delta_x;
        state[ROBOT_Y] += delta_y;
        state[ROBOT_P] += delta_th;


        /// ------- Propagating the states --------------///
        /// Calculating Phi
        Phi = Eigen::MatrixXf::Identity(covariance.rows(),covariance.cols());
        Phi(0,2) = -dt*vx*std::sin(th);
        Phi(1,2) =  dt*vx*std::cos(th);

        /// Calculating G
        G = Eigen::MatrixXf::Zero(covariance.rows(),2);
        G(0,0) = -dt*std::cos(th);
        G(1,0) = -dt*std::sin(th);
        G(2,1) = -dt;

        covariance = Phi*covariance*Phi.transpose() + G*Q*G.transpose();

    }

    /// Visualization purposes
    {
        geometry_msgs::TransformStamped robot_pose;
        robot_pose.header.stamp = current_time;
        robot_pose.header.frame_id = world_frame; //!< the solid world coordinate frame that our state is in
        robot_pose.child_frame_id = "odom"; //!< the frame of the robot that is the first 3 elements of the state vector
        double angle = state[ROBOT_P]-tf::getYaw(odom->pose.pose.orientation);
        double x=std::cos(angle)*odom->pose.pose.position.x-std::sin(angle)*odom->pose.pose.position.y;
        double y=std::sin(angle)*odom->pose.pose.position.x+std::cos(angle)*odom->pose.pose.position.y;

        robot_pose.transform.translation.x=state[ROBOT_X]-x;
        robot_pose.transform.translation.y=state[ROBOT_Y]-y;

        geometry_msgs::Quaternion robot_quat = tf::createQuaternionMsgFromYaw(angle);
        robot_pose.transform.rotation=robot_quat;
        pose_broadcaster->sendTransform(robot_pose);
    }
    {
        geometry_msgs::TransformStamped robot_pose;
        robot_pose.header.stamp = current_time;
        robot_pose.header.frame_id = world_frame; //!< the solid world coordinate frame that our state is in
        robot_pose.child_frame_id = "ekf_base_link"; //!< the frame of the robot that is the first 3 elements of the state vector
        robot_pose.transform.translation.x=state[ROBOT_X];
        robot_pose.transform.translation.y=state[ROBOT_Y];

        geometry_msgs::Quaternion robot_quat = tf::createQuaternionMsgFromYaw(state[ROBOT_P]);
        robot_pose.transform.rotation=robot_quat;
        pose_broadcaster->sendTransform(robot_pose);
    }
    last_time=current_time;

    if(publish_pointclouds){
        /// Publish the robot state
        sensor_msgs::PointCloud robot_xy;
        robot_xy.header.frame_id = world_frame;
        robot_xy.header.stamp = ros::Time::now();
        geometry_msgs::Point32 p1;
        p1.x=state(0);
        p1.y=state(1);
        robot_xy.points.push_back(p1);
        pub_robot_state.publish(robot_xy);
    }

    geometry_msgs::PoseWithCovarianceStamped robot_state_w_cov;
    robot_state_w_cov.header.frame_id = world_frame;
    robot_state_w_cov.header.stamp = ros::Time::now();
    robot_state_w_cov.pose.pose.position.x = state[ROBOT_X];
    robot_state_w_cov.pose.pose.position.y = state[ROBOT_Y];
    robot_state_w_cov.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state[ROBOT_P]);
    for(int ii=0;ii<36;ii++){
        robot_state_w_cov.pose.covariance[ii]=0.0;
    }
    robot_state_w_cov.pose.covariance[6*0+0]=covariance(ROBOT_X,ROBOT_X);
    robot_state_w_cov.pose.covariance[6*0+1]=covariance(ROBOT_X,ROBOT_Y);
    robot_state_w_cov.pose.covariance[6*0+5]=covariance(ROBOT_X,ROBOT_P);
    
    robot_state_w_cov.pose.covariance[6*1+0]=covariance(ROBOT_Y,ROBOT_X);
    robot_state_w_cov.pose.covariance[6*1+1]=covariance(ROBOT_Y,ROBOT_Y);
    robot_state_w_cov.pose.covariance[6*1+5]=covariance(ROBOT_Y,ROBOT_P);
    
    robot_state_w_cov.pose.covariance[6*5+0]=covariance(ROBOT_P,ROBOT_X);
    robot_state_w_cov.pose.covariance[6*5+1]=covariance(ROBOT_P,ROBOT_Y);
    robot_state_w_cov.pose.covariance[6*5+5]=covariance(ROBOT_P,ROBOT_P);
    pub_robot_state_w_cov.publish(robot_state_w_cov);
}

int main(int argc, char **argv){


    ros::init(argc,argv,"slam_ekf_2d_node");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");


    listener = new tf::TransformListener;
    pose_broadcaster = new tf::TransformBroadcaster;
    if(publish_pointclouds){
        pub_map = n.advertise<sensor_msgs::PointCloud>("map_features",1);
        pub_ellipse_robot_vis = n.advertise<sensor_msgs::PointCloud>("ellipse_vis_robot",1);
        pub_ellipse_feature_vis = n.advertise<sensor_msgs::PointCloud>("ellipse_vis_feature",1);
        pub_robot_state = n.advertise<sensor_msgs::PointCloud>("robot_state",1);
    }
    pub_robot_state_w_cov = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_state_w_cov",1);
    pub_feature_w_cov = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("feature_w_cov",1);

    ros::Subscriber feature_sub = n.subscribe("landmark",20,featureCallback);
    ros::Subscriber feature_array_sub = n.subscribe("landmark_array",5,featureArrayCallback);
    ros::Subscriber odom_sub = n.subscribe("odom",1,odomCallback);
    ros::Subscriber reset_sub = n.subscribe("/reset",1,resetCallback);
    ros::Timer tf_timer = n.createTimer(ros::Duration(0.1), publish_landmark_tf);

    /// Initiliaze time variables and state with covariance
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    pn.getParam("update_thresh", update_thresh);
    pn.getParam("new_feature_thresh",new_feature_thresh);
    pn.getParam("num_consecutive_errs",num_consecutive_errs);
    pn.getParam("distance_noise",ekf_distance_noise);
    pn.getParam("bearing_noise",ekf_bearing_noise);
    pn.getParam("phi_noise",ekf_phi_noise);
    pn.getParam("propogate_noise",ekf_propogate_noise);
    pn.getParam("publish_pointclouds",publish_pointclouds);
    pn.getParam("world_frame",world_frame);

    ros::Timer ellipse_timer;
    if(publish_pointclouds){
        ellipse_timer = n.createTimer(ros::Duration(0.1), publish_ellipse);
    }

    initializeSlam(ekf_distance_noise,ekf_bearing_noise,ekf_phi_noise,ekf_propogate_noise);

    ros::spin();

    /// Code Cleanup
    delete pose_broadcaster;

    return 0;
}
