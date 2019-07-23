#include <ros/ros.h>
#include <image_transport/image_transport.h>

/// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>

/// Zbar Barcode Library
#include <zbar.h>


class BarCodeFinder
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  ros::Publisher pub_point_out_;
  /// ZBar Stuff
  zbar::ImageScanner scanner_;
  std::string codeqr_;
  /// Params
  bool display_image_;
  bool publish_image_;
  bool debug_;
  bool qr_text_in_frameid_;
  std::map<std::string,float> qr_real_width_map_;

public:
  BarCodeFinder(void)
    : it_(nh_)
  {
    std::string image_topic = nh_.resolveName("image");
    sub_ = it_.subscribeCamera(image_topic, 1, &BarCodeFinder::imageCb, this);
    pub_ = it_.advertise("image_out", 1);
    pub_point_out_ = nh_.advertise<geometry_msgs::PoseStamped>("qrcode", 1);;

    /// Private node handle for params.
    ros::NodeHandle pnh("~");
    pnh.getParam("display_image", display_image_);
    pnh.getParam("publish_image", publish_image_);
    pnh.getParam("debug", debug_);
    pnh.getParam("qr_real_width_map", qr_real_width_map_);
    pnh.getParam("qr_text_in_frameid", qr_text_in_frameid_);

    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
  }

  /*!
   * \brief get_qr_size
   *
   * Gets a size from the parameter map based on the first characters of the QR code.
   * The idea is that if you want multiple sized landmarks, you might name them
   * Landmark-Large-01 Landmark-Large-02 Landmark-Small-01 Landmark-Small-02 and so on.
   * Then, your dictionary would say text that starts with "Landmark-Large-" are 10cm, and
   * text that starts with "Landmark-Small-" are 3cm, and this function would then get the
   * correct real world size.
   *
   * \param qr_text Text encoded in the QR code.
   * \return Size of the QR code. -1 if not in dictionary.
   */
  float get_qr_size(std::string qr_text)
  {
      for (std::map<std::string,float>::iterator it=qr_real_width_map_.begin(); it!=qr_real_width_map_.end(); ++it)
      {
          if(qr_text.substr(0,it->first.size())==it->first)
          {
              if(debug_)
                  std::cout << "MATCH: qr=#" << qr_text.substr(0,it->first.size()) << "#  map=#" << it->first << "#" << std::endl;
              return it->second;
          }else{
              if(debug_)
                  std::cout << "       qr=#" << qr_text.substr(0,it->first.size()) << "#  map=#" << it->first << "#" << std::endl;
          }
      }
      return -1.f;
  }

  tf::Transform make_tf(cv::Mat transmat, cv::Mat rotmat){

      //get geometry_msgs/Tranasform from 4x4 transformation matrix
      tf::Vector3 msgorigin( transmat.at<double>(0,0), transmat.at<double>(0,1), transmat.at<double>(0,2) );
      //tfScalar angle = sqrt(rotmat.at<double>(0,0)*rotmat.at<double>(0,0)+rotmat.at<double>(0,1)*rotmat.at<double>(0,1)+rotmat.at<double>(0,2)*rotmat.at<double>(0,2));
      //tf::Vector3 axis(rotmat.at<double>(0,0)/angle,rotmat.at<double>(0,1)/angle,rotmat.at<double>(0,2)/angle);
      //tf::Quaternion myQuat(axis,angle);
      tf::Matrix3x3 myMat;
      myMat.setValue(rotmat.at<double>(0,0),
                     rotmat.at<double>(0,1),
                     rotmat.at<double>(0,2),
                     rotmat.at<double>(1,0),
                     rotmat.at<double>(1,1),
                     rotmat.at<double>(1,2),
                     rotmat.at<double>(2,0),
                     rotmat.at<double>(2,1),
                     rotmat.at<double>(2,2));
      tf::Quaternion myQuat;
      myMat.getRotation(myQuat);

      tf::Transform trans(myQuat,msgorigin);

      return trans;
  }

  geometry_msgs::Pose make_pose(tf::Transform trans)
  {
      geometry_msgs::Pose pose;

      pose.position.x = trans.getOrigin().getX();
      pose.position.y = trans.getOrigin().getY();
      pose.position.z = trans.getOrigin().getZ();

      pose.orientation.x = trans.getRotation().getX();
      pose.orientation.y = trans.getRotation().getY();
      pose.orientation.z = trans.getRotation().getZ();
      pose.orientation.w = trans.getRotation().getW();

      return pose;
  }


  geometry_msgs::Pose make_pose(cv::Mat transmat, cv::Mat rotmat)
  {
      return make_pose(make_tf(transmat,rotmat));
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    /// Convert from sensor msgs type to opencv.
    /// \todo Test converting directly to MONO8 here, maybe save some time?
    cv::Mat image,image_gray;
    cv_bridge::CvImagePtr input_bridge;
    try {
        input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
        ROS_ERROR("[cv_barcode_it] Failed to convert image");
        return;
    }

    /// Get the camera model
    cam_model_.fromCameraInfo(info_msg);

    /// Convert to gray since QR codes are black and white
    cv::cvtColor(image,image_gray,CV_BGR2GRAY);
    /// If we are displaying or publishing the image for debug purposes, make a copy to draw onto
    if(display_image_ || publish_image_){
        cv::cvtColor(image_gray,image,CV_GRAY2RGB);
    }

    /// Copy the image data pointer to ZBar
    int width = image_gray.cols;
    int height = image_gray.rows;
    double centerx = double(width) / 2.0;
    double centery = double(height) / 2.0;
    uchar *raw = (uchar *)image_gray.data;
    // wrap image data
    zbar::Image zimage(width, height, "Y800", raw, width * height);
    // scan the image for barcodes
    int n = scanner_.scan(zimage);
    int symbols = 0;
    for(zbar::Image::SymbolIterator symbol = zimage.symbol_begin();  symbol != zimage.symbol_end();  ++symbol) {
        symbols++;

        /// Display the QR Code's text
        if(debug_)
            std::cout << symbol->get_data() << std::endl;

        int size=symbol->get_location_size();
        double qr_real_width = get_qr_size(symbol->get_data());
        double qr_real_half_width = qr_real_width/2.0;

        /// Only continue for nonzero sizes. This means if the prefix was not in our dictionary we ignore it.
        /// This could provide some safety, in case you pass a random barcode.
        /// \todo Could just provide a unit vector towards the qrcode if we do not know the size.
        if(qr_real_width>0.f && size==4)
        {

            //Make buffers to get 3x1 rotation vector and 3x1 translation vector
            cv::Mat_<double> rotationVector(3, 1);
            cv::Mat_<double> rotationActuallyMatrix(3, 3);
            cv::Mat_<double> translationVector(3, 1);


            //ROS_WARN("ROWS=%d,COLS=%d",rows,cols);
            std::vector<cv::Point3d> objectVerticesInObjectCoordinates;
            /// No idea what the proper order of these should be. It changes the "inherent" orientation of the QR.
            objectVerticesInObjectCoordinates.push_back(cv::Point3d(0.f, qr_real_half_width,-qr_real_half_width));
            objectVerticesInObjectCoordinates.push_back(cv::Point3d(0.f, qr_real_half_width, qr_real_half_width));
            objectVerticesInObjectCoordinates.push_back(cv::Point3d(0.f,-qr_real_half_width, qr_real_half_width));
            objectVerticesInObjectCoordinates.push_back(cv::Point3d(0.f,-qr_real_half_width,-qr_real_half_width));
            std::vector<cv::Point2d> imagePoints;
            imagePoints.resize(size);
            for(int i=0;i< size;i++)
            {
                imagePoints[i] = cv::Point2d(double(symbol->get_location_x( (i+1)%size )),
                                             double(symbol->get_location_y( (i+1)%size )));
            }

            /// Use solvePnP to get the rotation and translation vector of the QR code relative to the camera
            cv::solvePnP(objectVerticesInObjectCoordinates, cv::Mat(imagePoints), cam_model_.fullIntrinsicMatrix(), cam_model_.distortionCoeffs(), rotationVector, translationVector);
            cv::Rodrigues(rotationVector,rotationActuallyMatrix);

            /// \todo Publish TF's or at least geometry_msgs::Transforms
            //tf::Transform trans = make_tf(translationVector,rotationActuallyMatrix);

            /// \todo change to pose array message for viz, and transforms for other stuff
            /// \todo Could get fancypants and use visualization_msgs to actually put text in 3d...
            /// Publish the x,y,z and yaw of the QR code in the optical frame
            geometry_msgs::PoseStamped msg;
            if(qr_text_in_frameid_)
            {
                /// This is totally incorrect and will break RViz. But it could be useful for parsing position and text with one message.
                msg.header.frame_id=symbol->get_data();
            }else{
                /// This is correct, but now we don't have anywhere to store the QR text.
                msg.header.frame_id=image_msg->header.frame_id;
            }
            /// Copy the timestamp, since this location was valid at the time the picture was taken.
            msg.header.stamp=image_msg->header.stamp;
            msg.pose = make_pose(translationVector,rotationActuallyMatrix);
            pub_point_out_.publish(msg);

            /// Only bother drawing on the image if someone is going to see it
            if(display_image_ || publish_image_){
                /// Trace around the QR Code, to show what we are actually tracking.
                cv::line(image,imagePoints[0],imagePoints[1],cv::Scalar(255,0,0),3);
                cv::line(image,imagePoints[1],imagePoints[2],cv::Scalar(0,255,0),3);
                cv::line(image,imagePoints[2],imagePoints[3],cv::Scalar(0,0,255),3);
                cv::line(image,imagePoints[3],imagePoints[0],cv::Scalar(255,255,0),3);
            }
        }else{
            if(debug_)
                std::cout << "Failed to match symbol to map. qr_real_width = " << qr_real_width << "\t size = " << size << std::endl;
        }
    }

    if(display_image_){
        /// For debugging mostly, try to use the published image instead for long-term use.
        cv::imshow("cv_barcode_it", image);
        cv::waitKey(1);
    }
    if(publish_image_){
        /// This way you can log it with rosbag or view it in rviz or on another computer over the network if you want.
        pub_.publish(input_bridge->toImageMsg());
    }
    if(debug_)
        std::cout << "Found " << symbols << " symbols" << std::endl;

    // clean up
    zimage.set_data(NULL, 0);

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_barcode_it");
  BarCodeFinder finder;
  ros::spin();
}
