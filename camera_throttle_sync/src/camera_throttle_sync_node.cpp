#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class FrameSync
{
    ros::NodeHandle nh_; /// Public node handle for the image transport initializer
    ros::NodeHandle pnh_;/// Private node handle for params
    image_transport::ImageTransport it_;    /// Image transport allows use of compressed
    image_transport::CameraSubscriber sub1_;/// 
    image_transport::CameraPublisher pub1_;
    image_transport::CameraSubscriber sub2_;
    image_transport::CameraPublisher pub2_;
    ros::Time last_frame;
    float target_framerate;
    bool need_new_frame;
    bool timesync;

public:
  FrameSync()
    : it_(nh_), target_framerate(1.0), need_new_frame(false)
  {
    std::string hints = "compressed";
    pnh_.getParam("hints", hints);

    std::string image_topic1 = nh_.resolveName("image1");
    image_transport::TransportHints hints1(hints);
    sub1_ = it_.subscribeCamera(image_topic1, 1, &FrameSync::imageCb1, this, hints1);
    pub1_ = it_.advertiseCamera("image_out1", 1);

    std::string image_topic2 = nh_.resolveName("image2");
    image_transport::TransportHints hints2(hints);
    sub2_ = it_.subscribeCamera(image_topic2, 1, &FrameSync::imageCb2, this, hints2);
    pub2_ = it_.advertiseCamera("image_out2", 1);

    pnh_ = ros::NodeHandle("~");

    /// Target framerate, in frames/sec.
    /// A target framerate>1000fps means keep everything.
    pnh_.getParam("target_framerate", target_framerate);
    pnh_.getParam("timesync", timesync);

    last_frame = ros::Time::now();
  }


  void imageCb2(const sensor_msgs::ImageConstPtr& image_msg,
                const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
      /// Only publish if we recently got a frame from the other camera.
      if(need_new_frame || target_framerate>1000.0){
          if(timesync)
          {
              /// Make a local copy so we can change the timestamp
              sensor_msgs::Image image_copy = *(image_msg);
              sensor_msgs::CameraInfo info_copy = *(info_msg);
              image_copy.header.stamp = last_frame;
              info_copy.header.stamp = last_frame;
              pub2_.publish(image_copy,info_copy);
          }else{
              pub2_.publish(image_msg,info_msg);
          }
          need_new_frame=false;
      }
  }

  void imageCb1(const sensor_msgs::ImageConstPtr& image_msg,
                const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
      ros::Duration dt = image_msg->header.stamp-last_frame;
      if(dt.toSec()>1.0/target_framerate || target_framerate>1000.0){
          pub1_.publish(image_msg,info_msg);
          last_frame = image_msg->header.stamp;
          /// Tell the other camera we are ready to get an image
          /// This isn't the most accurate way to sync, but should give roughly synchronized images
          need_new_frame=true;
      }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_throttle_sync_node");
  FrameSync syncer;
  ros::spin();
}
