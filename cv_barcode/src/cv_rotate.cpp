#include <ros/ros.h>
#include <image_transport/image_transport.h>

/// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseArray.h>


class BarCodeFinder
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_;
  image_transport::Publisher pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  image_geometry::PinholeCameraModel cam_model_;
  ros::Publisher pub_points_out_;
  ros::Publisher pub_trans_out_;
  /// ZBar Stuff
  std::string codeqr_;
  /// Params
  bool display_image_;
  bool publish_image_;
  bool publish_tf_;
  bool debug_;
  bool qr_text_in_frameid_;
  std::map<std::string,float> qr_real_width_map_;

public:
  BarCodeFinder(void)
    : it_(nh_)
  {
    std::string image_topic = nh_.resolveName("image");
    image_transport::TransportHints hints("compressed");
    sub_ = it_.subscribe(image_topic, 1, &BarCodeFinder::imageCb, this, hints);
    pub_ = it_.advertise("image_out", 1);
    pub_points_out_ = nh_.advertise<geometry_msgs::PoseArray>("qrcodes", 1);
    pub_trans_out_ = nh_.advertise<geometry_msgs::TransformStamped>("qrcode_trans", 10);

    /// Private node handle for params.
    ros::NodeHandle pnh("~");
    pnh.getParam("display_image", display_image_);
    pnh.getParam("publish_image", publish_image_);
    pnh.getParam("publish_tf", publish_tf_);
    pnh.getParam("debug", debug_);
    pnh.getParam("qr_real_width_map", qr_real_width_map_);
    pnh.getParam("qr_text_in_frameid", qr_text_in_frameid_);

  }



  void imageCb(const sensor_msgs::ImageConstPtr& image_msg)
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
        ROS_ERROR("[cv_barcode_node] Failed to convert image");
        return;
    }

    cv::flip(image,image_gray,-1);

    /// For debugging mostly, try to use the published image instead for long-term use.
    cv::imshow("cv_rotate", image_gray);
    cv::waitKey(1);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_barcode_node");
  BarCodeFinder finder;
  ros::spin();
}
