/*
 * Name: Brandon Marlowe
 * ---------------------
 * TurtleBot detects and follows orange ball, always staying at least ~ 0.7
 * meters away
 */

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>

#define RGB_FOCAL_LEN_MM 138.90625  // camera focal length in mm ... 525 pixels
#define BALL_DIAM_MM 40.0          // 9" diameter ball in mm
#define CAMERA_HEIGHT_MM 300.0      // height of camera off ground in mm
#define IMG_HEIGHT_PX 480.0         // in pixels
#define IMG_WIDTH_PX 640.0          // in pixels

static const std::string OPENCV_WINDOW = "Image window";
static double currObjDist = 0.0, alignmentError = 0.0, prevVelX = 0.0,
              botVelX = 0.0;
static int x, y, radius;

class DetectorTracker {
  ros::NodeHandle nodeHandle_;
  image_transport::ImageTransport imageTransport_;
  image_transport::Subscriber imageSub_;
  image_transport::Publisher imagePub_;

 public:
  DetectorTracker() : imageTransport_(nodeHandle_) {
    // Subscribe to input video feed and publish output video feed
    imageSub_ = imageTransport_.subscribe(
        "/usb_cam/image_raw", 10, &DetectorTracker::imageCallback, this);
    imagePub_ = imageTransport_.advertise("/image_converter/output_video", 10);

    cv::namedWindow(OPENCV_WINDOW, 0);
  }

  ~DetectorTracker() { cv::destroyWindow(OPENCV_WINDOW); }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvPtr;
    cv_bridge::CvImagePtr cvGrayPtr;
    std::vector<cv::Vec3f> circleIMG;
    cv::Mat hsvIMG, redIMG_lower, redIMG_upper, redIMG, srcIMG;

    cv::Scalar black =
        (0, 255, 5);  // RGB color for circle to be drawn on image
    cv::Scalar blue = (200, 200, 250);  // RGB color for text displayed on image

    try {
      cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      srcIMG = cvPtr->image;

      // converting color to HSV
      cv::cvtColor(srcIMG, hsvIMG, CV_BGR2HSV);

      //defining upper and lower red color range
      cv::inRange(hsvIMG, cv::Scalar(0, 100, 100), cv::Scalar(20, 255, 255),
                  redIMG_lower);
      cv::inRange(hsvIMG, cv::Scalar(160, 100, 100), cv::Scalar(170, 255, 255),
                  redIMG_upper);


      // weighting image and performing blur to reduce noise in image
      cv::addWeighted(redIMG_lower, 1.0, redIMG_upper, 1.0, 0.0, redIMG);
      cv::GaussianBlur(redIMG, redIMG, cv::Size(9, 9), 2, 2);

      // Hough gradient transform to find circles
      cv::HoughCircles(redIMG, circleIMG, CV_HOUGH_GRADIENT, 1.4, hsvIMG.rows / 8,
                       100, 40, 0, 50);

    }

    catch (cv_bridge::Exception &exception) {
      ROS_ERROR("cv_bridge exception: %s", exception.what());
      return;
    }

    for (size_t i = 0; i < circleIMG.size(); i++) {
      // center coordinates of circle, and the radius
      x = static_cast<int>(round(circleIMG[i][0]));
      y = static_cast<int>(round(circleIMG[i][1]));
      radius = static_cast<int>(round(circleIMG[i][2]));
      cv::Point center(x, y);

      // draws circle around ball and cross-hair at center
      cv::circle(srcIMG, center, radius, black, 2);
      cv::line(srcIMG, center, center, black, 2);

      alignmentError = (x - (IMG_WIDTH_PX / 2.0));

      // text overlay
      std::stringstream ssDist, ssBotVelX, ssAlignError;
      ssAlignError << alignmentError;
      ssBotVelX << botVelX;

      std::string alignErrStr = "    ALN_ERR: " + ssAlignError.str() + " px";
      std::string objDistStr = "    OBJ_DST: " + ssDist.str() + " m";
      std::string botVelStr = "    BOT_VEL: " + ssBotVelX.str() + " m/s";

      cv::putText(srcIMG, alignErrStr, cv::Point(350, 375),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, blue, 1, CV_AA);
      cv::putText(srcIMG, objDistStr, cv::Point(350, 400),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, blue, 1, CV_AA);
      cv::putText(srcIMG, botVelStr, cv::Point(350, 425),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, blue, 1, CV_AA);
    }

    // image is published to screen with circle drawn around red ball
    cv::imshow(OPENCV_WINDOW, srcIMG);
    cv::waitKey(3);

    // Output modified video stream
    imagePub_.publish(cvPtr->toImageMsg());
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "ball_tracker_node");
  DetectorTracker detectorTracker;
  ros::spin();

  return 0;
}
