/// ROS
#include <ros/ros.h>

/// Messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

/// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

/// Zbar Barcode Library
#include <zbar.h>  


/// Boost time, for tic() toc()
#include "boost/date_time/posix_time/posix_time.hpp" ///!< include all types plus i/o

/// Silly little timing functions, to get real CPU time, not ROS time
boost::posix_time::ptime start_tic_toc[10];
inline void tic(int i=0){start_tic_toc[i]=boost::posix_time::microsec_clock::universal_time();}
inline double toc(int i=0){return ((double)(boost::posix_time::microsec_clock::universal_time()-start_tic_toc[i]).total_microseconds())/1000000.0;}

/// ZBar Stuff
zbar::ImageScanner scanner;  
std::string codeqr;

/// CV Frames
cv::Mat frame, cropped, img, imgout;

/// ROS Node Variables
ros::NodeHandle *nh;
ros::Publisher pub_point_out;
ros::Publisher pub_img_out;

/// Change this to display a pretty picture
bool display_image=true;
bool publish_image=false;
bool publish_both_sides=false;
bool qr_text_in_frameid=false;
float Fx=554.254691191187;
float qr_real_width=0.168;//!< m
bool debug = false;/// Print debug info to std::cout

void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{
    tic();

    /// Convert from a ROS message to an OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr_rgb;
    try
    {
        cv_ptr_rgb = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    /// Convert to gray since QR codes are black and white
    cv::cvtColor(cv_ptr_rgb->image,img,CV_BGR2GRAY);
    /// If we are displaying or publishing the image for debug purposes, make a copy to draw onto
    if(display_image || publish_image){
        cv::cvtColor(img,imgout,CV_GRAY2RGB);  
    }

    /// Copy the image data pointer to ZBar
    int width = img.cols;  
    int height = img.rows;  
    uchar *raw = (uchar *)img.data;  
    // wrap image data  
    zbar::Image image(width, height, "Y800", raw, width * height);  
    // scan the image for barcodes  
    int n = scanner.scan(image);
    int symbols = 0;
    for(zbar::Image::SymbolIterator symbol = image.symbol_begin();  symbol != image.symbol_end();  ++symbol) {
        symbols++;

        /// Display the QR Code's text
        if(debug)
            std::cout << symbol->get_data();

        /// Find the average x,y and line length.
        /// Average x and y represent the middle of the QR Code (mostly)
        /// Average line length gives us a way to estimate distance, since
        /// we know how long that line is in the real world.
        /// Average is probably bad, since it won't deal with skew very well.
        /// Maybe max would be better? I dunno.
        float avg_dist_px=0.0;
        float avg_x_px=0.0;
        float avg_y_px=0.0;
        int size=symbol->get_location_size();
        for(int i=0;i< size;i++)
        {
            float dx=symbol->get_location_x(i)-symbol->get_location_x( (i+1)%size );
            float dy=symbol->get_location_y(i)-symbol->get_location_y( (i+1)%size );
            float dist_px=sqrt(dx*dx+dy*dy);
            avg_dist_px+=dist_px;
            avg_x_px+=symbol->get_location_x(i);
            avg_y_px+=symbol->get_location_y(i);
            
            if(debug){
                std::cout << "\t" << dist_px;
                std::cout << "\t" << symbol->get_location_x(i);
                std::cout << "\t" << symbol->get_location_y(i);
            }
        }
        avg_dist_px/=size;
        avg_x_px/=size;
        avg_y_px/=size;
        if(debug){
            std::cout << "\t" << avg_dist_px;
            std::cout << "\t" << avg_x_px;
            std::cout << "\t" << avg_y_px;
        }
        
        float left_dist_px;
        float right_dist_px;
        float left_bear_px;
        float right_bear_px;
        for(int i=0;i< size;i++)
        {
            if(symbol->get_location_x(i) < avg_x_px && symbol->get_location_x( (i+1)%size ) < avg_x_px)
            {
                float dx=symbol->get_location_x(i)-symbol->get_location_x( (i+1)%size );
                float dy=symbol->get_location_y(i)-symbol->get_location_y( (i+1)%size );
                left_dist_px=sqrt(dx*dx+dy*dy);
                left_bear_px=(symbol->get_location_x(i)+symbol->get_location_x( (i+1)%size ))/2.0;
            }
            if(symbol->get_location_x(i) > avg_x_px && symbol->get_location_x( (i+1)%size ) > avg_x_px)
            {
                float dx=symbol->get_location_x(i)-symbol->get_location_x( (i+1)%size );
                float dy=symbol->get_location_y(i)-symbol->get_location_y( (i+1)%size );
                right_dist_px=sqrt(dx*dx+dy*dy);
                right_bear_px=(symbol->get_location_x(i)+symbol->get_location_x( (i+1)%size ))/2.0;
            }
        }
        /// We are assuming a pinhole camera model, with even pixels. 
        /// THIS IS DEFINITELY WRONG!!!!  ESPECIALLY NEAR THE EDGES!!!
        //float fov=39.0;//!< Degrees
        //float fov=60.0;//!< Degrees
        float fov_rad=2 * atan(0.5 * width / Fx);
        //float fov_rad=fov*M_PI/180.0;
        float rad_per_px=fov_rad/width;
        
        /// Law of cosines:
        /// c^2 = a^2 + b^2 - 2ab*cos(C)
        /// c   = qr_real_width
        /// a/b = qr_real_width/tan(left_dist_px *rad_per_px/2.0)/
        ///       qr_real_width/tan(right_dist_px*rad_per_px/2.0)
        /// a/b = tan(right_dist_px*rad_per_px/2.0)/
        ///       tan(left_dist_px *rad_per_px/2.0)
        /// a/b = sin(right)/(1+cos(right))/
        ///       sin(left )/(1+cos(left ))/
        /// a/b = left_dist_px/right_dist_px   <-- ASSUMES SMALL ANGLE APPROX
        /// a/b = r
        /// b   = a/r
        /// c^2 = a^2 + a^2/r^2 - 2a^2/r*cos(C)
        /// c^2 = a^2*(1 + 1/r^2 - 2*cos(C)/r)
        /// a   = sqrt(c^2/(1 + 1/r^2 - 2*cos(C)/r))
        
        /// c^2 = b^2*r^2 + b^2 - 2b^2*r*cos(C)
        /// c^2 = b^2*(1 + r^2 - 2*r*cos(C))
        /// b   = sqrt(c^2/(1 + r^2 - 2*r*cos(C))
        
        
        float c   = qr_real_width;
        float r   = std::tan(right_dist_px*rad_per_px/2.0)/
                    std::tan(left_dist_px *rad_per_px/2.0);
        float C   = (right_bear_px-left_bear_px)*rad_per_px;
        float a   = std::sqrt(c*c/(1 + 1.0/r/r - 2/r*std::cos(C)));
        float b   = std::sqrt(c*c/(1 + r*r     - 2*r*std::cos(C)));

        float bearing_r=(right_bear_px-width/2.0)*rad_per_px;
        float beta= M_PI_2 - bearing_r;
        float A   = std::asin(a*sin(C)/c);
        float yaw = A - beta;

        /// Use some trig to calculate distance
        float distance=qr_real_width/std::tan(avg_dist_px*rad_per_px/2.0)/2.0;//!< m
        float bearing_x=(avg_x_px-width/2.0)*rad_per_px;
        float bearing_y=(avg_y_px-height/2.0)*rad_per_px;

        /// Publish the x,y,z and yaw of the QR code in the optical frame
        geometry_msgs::PoseStamped msg;
        if(qr_text_in_frameid)
        {
            /// This is totally incorrect and will break RViz. But it could be useful for parsing position and text with one message.
            msg.header.frame_id=symbol->get_data();
        }else{
            msg.header.frame_id="base_link";
        }
        msg.header.stamp=ros::Time::now();
        msg.pose.position.x=std::cos(bearing_y)*std::sin(bearing_x)*distance;
        msg.pose.position.y=std::sin(bearing_y)*distance;
        msg.pose.position.z=std::cos(bearing_y)*std::cos(bearing_x)*distance;
        /// \note this is only using the yaw, since that's all I care about, but the same could be done for pitch and roll if you care about that.
        msg.pose.orientation.x=0.0;
        msg.pose.orientation.y=std::sin(yaw/2.f); /// Dunno +/-, but should be rotating about y
        msg.pose.orientation.z=0.0;
        msg.pose.orientation.w=std::cos(yaw/2.f);
        pub_point_out.publish(msg);
        
        if(display_image || publish_image){
            /// Make a nice circle the size of the QR Code (This represents our distance measurement)
            cv::circle(imgout,cv::Point2f(avg_x_px,avg_y_px),avg_dist_px/2.0,cv::Scalar(255,0,0),3);
            /// And a little circle in the middle of the QR Code (This represents our bearing measurement) 
            cv::circle(imgout,cv::Point2f(avg_x_px,avg_y_px),10,cv::Scalar(0,0,255),3); 
            
            /// Trace around the QR Code, just to be friendly.
            cv::line(imgout,cv::Point2f(symbol->get_location_x(0),symbol->get_location_y(0)),
                            cv::Point2f(symbol->get_location_x(1),symbol->get_location_y(1)),cv::Scalar(255,0,0),3); 
            cv::line(imgout,cv::Point2f(symbol->get_location_x(1),symbol->get_location_y(1)),
                            cv::Point2f(symbol->get_location_x(2),symbol->get_location_y(2)),cv::Scalar(0,255,0),3); 
            cv::line(imgout,cv::Point2f(symbol->get_location_x(2),symbol->get_location_y(2)),
                            cv::Point2f(symbol->get_location_x(3),symbol->get_location_y(3)),cv::Scalar(0,0,255),3); 
            cv::line(imgout,cv::Point2f(symbol->get_location_x(3),symbol->get_location_y(3)),
                            cv::Point2f(symbol->get_location_x(0),symbol->get_location_y(0)),cv::Scalar(255,255,0),3);
        }
    }  
    
    if(display_image){
        /// For debugging
        cv::imshow("barcodes", imgout);
        cv::waitKey(3);
    }
    if(publish_image){
        /// This way you can log it with rosbag or view it in rviz or on another computer over the network if you want.
        cv_bridge::CvImage out_msg;
        out_msg.header   = image_msg->header; // Same timestamp and tf frame as input image
        out_msg.encoding = image_msg->encoding; // Or whatever
        out_msg.image    = imgout; // Your cv::Mat

        pub_img_out.publish(out_msg.toImageMsg());
    }
    if(debug)
        std::cout << "Found " << symbols << " symbols in " << toc() << " seconds" << std::endl;

    // clean up  
    image.set_data(NULL, 0);
}


int
main (int argc, char** argv)
{
    
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    
    /// Initialize ROS
    ros::init (argc, argv, "cv_barcode_node");
    nh = new ros::NodeHandle("~");
    nh->getParam("publish_image",publish_image);
    nh->getParam("display_image",display_image);
    nh->getParam("publish_both_sides",publish_both_sides);
    nh->getParam("qr_text_in_frameid",qr_text_in_frameid);
    nh->getParam("Fx",Fx);
    nh->getParam("debug",debug);

    // advertise
    pub_point_out = nh->advertise<geometry_msgs::PoseStamped>("/landmark", 1);
    pub_img_out = nh->advertise<sensor_msgs::Image>("/rgb/image_barcode", 1);

    // subscribe
    ros::Subscriber sub = nh->subscribe("/rgb/image_rect", 1, imageCallback);

    // timer
    //ros::Timer param_timer = nh->createTimer(ros::Duration(1.0), param_timer_callback);

    // Spin
    ros::spin ();

}
