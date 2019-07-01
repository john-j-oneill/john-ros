/// ROS
#include <ros/ros.h>
#include <ros/package.h>

//TF CRAP
#include <tf/tf.h>

/// Messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

/// OpenCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

/// Zbar Barcode Library
#include <zbar.h>  

/// Boost time, for tic() toc()
#include "boost/date_time/posix_time/posix_time.hpp" ///!< include all types plus i/o

/// Silly little timing functions, to get real CPU time, not ROS time
boost::posix_time::ptime start_tic_toc[10];
inline void tic(int i=0){start_tic_toc[i]=boost::posix_time::microsec_clock::universal_time();}
inline double toc(int i=0){return ((double)(boost::posix_time::microsec_clock::universal_time()-start_tic_toc[i]).total_microseconds())/1000000.0;}



//if you wanna hear some shit
bool verboz = false;

/// ZBar Stuff
zbar::ImageScanner scanner;  

//camera parameters (comes from calibration)
cv::Mat cameraMatrix;  //3x3 matrix
cv::Mat distortionParameters; //1x5 matrix
//TODO ACTUALLY CALIBRATE LIFECAM
//640x480
//cameraMatrix = (cv::Mat_<double>(3, 3) << 647.940890, 0.000000, 322.301665, 0.000000, 647.818508, 264.981038, 0.000000, 0.000000, 1.000000);
//distortionParameters = (cv::Mat_<double>(1, 5) << -0.014068, 0.097657, 0.001673, 0.001186, 0.000000);
//1280x720
//cameraMatrix = (cv::Mat_<double>(3, 3) << 973.825377, 0.0, 632.1498354, 0.0, 977.2157731, 364.39605619, 0.0, 0.0, 1.0 );
//distortionParameters = (cv::Mat_<double>(1, 5) << -0.063413327, 0.5331313121, -0.000502394, 0.00028850168, -0.984497690467);

//qr info HAVE TO CHANGE THIS
float QRCodeDimensionInMeters = 0.11865; //0.119; // 0.095; //9.5 cm
float QRCodeDimensionInMeters_Small = 0.060; //0.027; // 0.095; //9.5 cm

std::vector<cv::Point3d> objectVerticesInObjectCoordinates;

//qr list
std::vector<std::string> G_codeqrlist;
std::vector<std::vector<cv::Point3d> > G_codeqrtemplates;


/// CV Frames
cv::Mat frame, transmat, rotmat;
int seq = 0;

/// ROS Node Variables
ros::NodeHandle *nh;
ros::Publisher pub_transform_out;
ros::Publisher pub_img_out;

/// Change this to display a pretty picture
bool display_image=false;
bool publish_image=true;


geometry_msgs::TransformStamped make_msg(cv::Mat transmat, cv::Mat rotmat, std::string qr_text){

    //get geometry_msgs/Tranasform from 4x4 transformation matrix
    geometry_msgs::Vector3 msgorigin;
    msgorigin.x = transmat.at<double>(0,0);
    msgorigin.y = transmat.at<double>(0,1);
    msgorigin.z = transmat.at<double>(0,2);
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

    geometry_msgs::Quaternion msgquaternion;
    msgquaternion.x = myQuat.getX();
    msgquaternion.y = myQuat.getY();
    msgquaternion.z = myQuat.getZ();
    msgquaternion.w = myQuat.getW();

    if(verboz){ std::cout << "done " << toc(3) << " seconds\n";}

    if(verboz){ std::cout << "publishing transform ... ";tic(3);}

    //publish out the transformation matrix
    geometry_msgs::TransformStamped msg;
    msg.header.frame_id=qr_text; //"QRDocking";
    msg.header.stamp=ros::Time::now();
    msg.header.seq=seq++;
    msg.transform.translation = msgorigin;
    msg.transform.rotation = msgquaternion;
    return msg;
}


//takes in qr code image and gets the pnp solution
bool getTransformationFromQR(cv::Mat &frame, cv::Mat &TranslationMatrix, cv::Mat &RotationMatrix, bool drawon = false){
	//cv::Mat frame = rgb image from webcam
	//cv::Mat TransformationMatrix = 4x4 camera pose
	//std::string codeqr = string that qr code reads ie: "ronnydocking"
	//bool drawon = will draw side lines, center circle and corners
	//bool return if desired qr string was even found

	//webcam size
	int width = frame.cols;
	int height = frame.rows;
	float centerx = float(width) / 2.0;
	float centery = float(height) / 2.0;

	//some extra variables
	bool foundcode = false;
	cv::Mat img;
	cv::Point2f obj_pos;
	cv::Point2f corner1, corner2, shiftpos;

	//make it gray
	cv::cvtColor(frame, img, CV_BGR2GRAY);

	//make sure we still know the size
	width = img.cols;
	height = img.rows;
	centerx = float(width) / 2.0;
	centery = float(height) / 2.0;

	//get opencv image to zbar image
	uchar *raw = (uchar *)img.data;
	zbar::Image image(width, height, "Y800", raw, width * height);

	// scan the image for barcodes  
	int n = scanner.scan(image);

    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
        foundcode=true;
        //variables for qr code location
        std::vector<cv::Point2d> onlineqrpoints; // need this vector

        //get average location  of qr code
        float avg_x_px = 0.0;
        float avg_y_px = 0.0;

        //loop through all corners of qr code
        int size = symbol->get_location_size();
        for (int i = 0; i < size; i++){
            //put the points in vector
            shiftpos.x = double(symbol->get_location_x(i) - centerx);
            shiftpos.y = double(symbol->get_location_y(i) - centery);
            onlineqrpoints.push_back(shiftpos);

            if (drawon){
                //get center location
                avg_x_px += symbol->get_location_x(i);
                avg_y_px += symbol->get_location_y(i);
                //draw lines
                corner1.x = symbol->get_location_x(i);
                corner1.y = symbol->get_location_y(i);
                corner2.x = symbol->get_location_x((i + 1) % size);
                corner2.y = symbol->get_location_y((i + 1) % size);
                //put side bars and center circle
                cv::line(frame, corner1, corner2, cv::Scalar(255, 0, 0), 3);
                cv::circle(frame, corner1, 5, cv::Scalar(0, 50 + i * 50, 0), 2);
            }
        }
        if (drawon){
            // store qr center
            avg_x_px /= size;
            avg_y_px /= size;
            obj_pos.x = avg_x_px;
            obj_pos.y = avg_y_px;
            cv::circle(frame, obj_pos, 20, cv::Scalar(0, 0, 255), 3);
        }

        //Make buffers to get 3x1 rotation vector and 3x1 translation vector
        cv::Mat_<double> rotationVector(3, 1);
        cv::Mat_<double> rotationActuallyMatrix(3, 3);
        cv::Mat_<double> translationVector(3, 1);

        //mask corners for pnp
        double buf = QRCodeDimensionInMeters / 2.0;
        std::string smalltext = "dockingsmall";

        if(symbol->get_data().compare(smalltext))
        {
            //std::cout << "I see a Trump sized guy!\t" << buf << std::endl;
        }else{
            buf = QRCodeDimensionInMeters_Small / 2.0;
            //std::cout << "I see the small guy!\t" << buf << std::endl;
        }

        // could have put this in consturctor
        objectVerticesInObjectCoordinates.resize(4);
        objectVerticesInObjectCoordinates.at(0)=(cv::Point3d(-buf, buf, 0));
        objectVerticesInObjectCoordinates.at(1)=(cv::Point3d(-buf, -buf, 0));
        objectVerticesInObjectCoordinates.at(2)=(cv::Point3d(buf, -buf, 0));
        objectVerticesInObjectCoordinates.at(3)=(cv::Point3d(buf, buf, 0));

        if(verboz){ std::cout << "starting solvepnp\n";tic(4);}
        //Use solvePnP to get the rotation and translation vector of the QR code relative to the camera
        cv::solvePnP(objectVerticesInObjectCoordinates, onlineqrpoints, cameraMatrix, distortionParameters, rotationVector, translationVector);
        cv::Rodrigues(rotationVector,rotationActuallyMatrix);
        if(verboz){ std::cout << "done solvepnp " << toc(4) << " seconds\n";}

        //store it
        TranslationMatrix = translationVector;
        RotationMatrix = rotationActuallyMatrix;

        if(verboz){
            //std::cout << viewMatrix << "\n";
            //std::cout << rotationVector << "\n";
            //std::cout << translationVector << "\n";
        }


        geometry_msgs::TransformStamped msg = make_msg(TranslationMatrix,RotationMatrix,symbol->get_data());
        pub_transform_out.publish(msg);
    }

	// clean up  
	image.set_data(NULL, 0);

	return foundcode;
}

//callback function
void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{

    if(verboz){ std::cout << "starting callback \n"; tic(0);}

    //cv bridge crap
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

	//get it from the cv bridge
    frame =  cv_ptr_rgb->image; 

    if(frame.empty()){
        return;
    }

    if(verboz){ std::cout << "starting function \n";tic(1);}

	//rods function
    bool botherdrawing = display_image || publish_image;
    bool gotem = getTransformationFromQR(frame, transmat, rotmat, botherdrawing);
    if(verboz){ std::cout << "rotmat " << rotmat << " seconds\n";}
    if(verboz){ std::cout << "done " << toc(1) << " seconds\n";}

    if(display_image){
        cv::imshow("barcodes", frame);
        cv::waitKey(30);
    }
    if(publish_image){
        cv_bridge::CvImage out_msg;
        out_msg.header   = image_msg->header; // Same timestamp and tf frame as input image
        out_msg.encoding = "bgr8"; // Or whatever
        out_msg.image    = frame; // Your cv::Mat
        pub_img_out.publish(out_msg.toImageMsg());
    }

}


int main (int argc, char** argv)
{
    //started its own zbar
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);



    //THESE ARE ACTUAL LIFECAM PARAMS
    //DEFAULT TO HD
    //1280x720
    cameraMatrix = (cv::Mat_<double>(3, 3) << 973.825377, 0.0, 0.0, 0.0, 977.2157731, 0.0, 0.0, 0.0, 1.0);
    distortionParameters = (cv::Mat_<double>(1, 5) << -0.063413327, 0.5331313121, -0.000502394, 0.00028850168, -0.984497690467);
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-Simulation") {
            //THIS IS FOR THE SIMULATED KINECT IN GAZEBO.
            //WE COULD JUST SUBSCRIBE TO /camera/rgb/camera_info
            cameraMatrix = (cv::Mat_<double>(3, 3) << 554.254691191187, 0.0, 0.0, 0.0, 554.254691191187, 0.0, 0.0, 0.0, 1.0);
            distortionParameters = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
            //Also the QRs are different sizes, methinks.
            QRCodeDimensionInMeters = 0.168;
            QRCodeDimensionInMeters_Small = 0.084;
        }
        if (std::string(argv[i]) == "-Kinect") {
            //THIS IS FOR THE SIMULATED KINECT IN GAZEBO.
            //WE COULD JUST SUBSCRIBE TO /camera/rgb/camera_info
            cameraMatrix = (cv::Mat_<double>(3, 3) << 525.0, 0.0, 0.0, 0.0, 525.0, 0.0, 0.0, 0.0, 1.0);
            distortionParameters = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
            //Also the QRs are different sizes, methinks.
            QRCodeDimensionInMeters = 0.168;
            QRCodeDimensionInMeters_Small = 0.084;
        }
        if (std::string(argv[i]) == "-HD") {
            //THESE ARE ACTUAL LIFECAM PARAMS
            //1280x720
            cameraMatrix = (cv::Mat_<double>(3, 3) << 973.825377, 0.0, 0.0, 0.0, 977.2157731, 0.0, 0.0, 0.0, 1.0);
            distortionParameters = (cv::Mat_<double>(1, 5) << -0.063413327, 0.5331313121, -0.000502394, 0.00028850168, -0.984497690467);
        }
        if (std::string(argv[i]) == "-SD") {
            //THESE ARE ACTUAL LIFECAM PARAMS
            //640x480 centered
            cameraMatrix = (cv::Mat_<double>(3, 3) << 667.93355, 0.0, 0.0, 0.0, 667.33104262, 0.0, 0.0, 0.0, 1.0);
            distortionParameters = (cv::Mat_<double>(1, 5) << -0.0216951776, 0.477391311, -0.014749349, 0.002832143367, -0.909214);
        }
    }

    
    /// Initialize ROS
    ros::init (argc, argv, "ronny_qrdocking");
    nh = new ros::NodeHandle("~");

    // advertise
    pub_transform_out = nh->advertise<geometry_msgs::TransformStamped>("/dockinglandmark", 1);
    pub_img_out = nh->advertise<sensor_msgs::Image>("/rgb/docking_barcode", 1);

    // subscribe
    ros::Subscriber sub = nh->subscribe("/rgb/image_lifecam", 1, imageCallback);

    // Spin
    ros::spin ();

}
