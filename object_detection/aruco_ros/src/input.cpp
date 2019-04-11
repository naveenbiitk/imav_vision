#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//static const std::string OPENCV_WINDOW = "Image window";

cv::Mat img,maskHSV,hsv;
int h_min=0,h_max=20,s_min=100,s_max=255,v_min=100,v_max=255;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("colour_image", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("threshold_image", 1);

  //  cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    img=cv_ptr->image;
    /*cv::createTrackbar("H minimum:",OPENCV_WINDOW,& h_min,180);
    cv::createTrackbar("H maximum:",OPENCV_WINDOW,& h_max,180);
    cv::createTrackbar("S minimum:",OPENCV_WINDOW,& s_min,255);
    cv::createTrackbar("S maximum:",OPENCV_WINDOW,& s_max,255);
    cv::createTrackbar("V minimum:",OPENCV_WINDOW,& v_min,255);
    cv::createTrackbar("V maximum:",OPENCV_WINDOW,& v_max,255);
    // Draw an example circle on the video stream
    */cv::cvtColor(img,hsv,CV_BGR2HSV);
    cv::inRange(hsv,cv::Scalar(h_min,s_min,v_min),cv::Scalar(h_max,s_max,v_max),maskHSV);
    cv::cvtColor(maskHSV,img,cv::COLOR_GRAY2BGR);
    // Update GUI Window
    /*cv::imshow(OPENCV_WINDOW,img );
    cv::waitKey(3);
*/
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
          cv_bridge::CvImage in_msg;
          in_msg.header.stamp = ros::Time::now();
          in_msg.encoding = sensor_msgs::image_encodings::BGR8;
          in_msg.image = img;
          image_pub_.publish(in_msg.toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "input");
  ImageConverter ic;
  ros::spin();
  return 0;
}

