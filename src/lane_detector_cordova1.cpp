/*
 * lane_detector.cpp
 *
 *      Author:
 *         Nicolas Acero
 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <lane_detector/cannyEdge.h>
#include <lane_detector/houghTransform.h>
#include <cv.h>
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <lane_detector/DetectorConfig.h>

static const std::string OPENCV_WINDOW = "Image window";
cv::Mat currentFrame;
cv::Mat* lastFrame;
Preprocessor* preproc;
FeatureExtractor* extractor;

void configCallback(lane_detector::DetectorConfig& config, uint32_t level)
{
  if(preproc != NULL && extractor != NULL) {
    ROS_DEBUG("Config was set");
    preproc->setConfig(config);
    extractor->setConfig(config);
  }
}

void readImg(const sensor_msgs::ImageConstPtr& img)
{

  cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    currentFrame = cv_ptr->image;

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, cv::Scalar(255,0,0));

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, currentFrame);
    //cv::waitKey(3);

}

void readImgTmp(const std::string imgName) {
  currentFrame = cv::imread("/home/n/caltech-lanes/cordova1/" + imgName);
  if(currentFrame.data == NULL) {
    std::cout << "Error: No image with name " << imgName << " received" << std::endl;
  }
}

void processImage(Preprocessor* preproc, FeatureExtractor* extractor) {
  cv::Mat copyImg = currentFrame;
  preproc->preprocess(currentFrame);
  extractor->extract(copyImg, currentFrame);
}


int main(int argc, char **argv){

    preproc = new CannyEdge();
    extractor =  new HoughTransform();

    ros::init(argc, argv, "lane_detector");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    dynamic_reconfigure::Server<lane_detector::DetectorConfig> server;
    dynamic_reconfigure::Server<lane_detector::DetectorConfig>::CallbackType f;

    f = boost::bind(&configCallback, _1, _2);
    server.setCallback(f);

		/**
	   * The advertise() function is how you tell ROS that you want to
	   * publish on a given topic name. This invokes a call to the ROS
	   * master node, which keeps a registry of who is publishing and who
	   * is subscribing. After this advertise() call is made, the master
	   * node will notify anyone who is trying to subscribe to this topic name,
	   * and they will in turn negotiate a peer-to-peer connection with this
	   * node.  advertise() returns a Publisher object which allows you to
	   * publish messages on that topic through a call to publish().  Once
	   * all copies of the returned Publisher object are destroyed, the topic
	   * will be automatically unadvertised.
	   *
	   * The second parameter to advertise() is the size of the message queue
	   * used for publishing messages.  If messages are published more quickly
	   * than we can send them, the number here specifies how many messages to
	   * buffer up before throwing some away.
	   */

     image_transport::Subscriber image_sub = it.subscribe("/wide_stereo/left/image_raw_throttle", 1, readImg);

     cv::namedWindow(OPENCV_WINDOW);


		//This node needs to publish at least at a rate of 20hz or navigational stack will complain
    ros::Rate loop_rate(30);
    std::stringstream ss;
    int i = 0;
    while(ros::ok()) {
      i++;
      // Update GUI Window
      if(i < 10) {
        ss << "f0000" << i << ".png";
      }
      else if(i < 100) {
        ss << "f000" << i << ".png";
      }
      else if(i < 220) {
        ss << "f00" << i << ".png";
      }

      /*else if(i < 1152) {
        ss << "IMG_" << i << ".png";
      }*/

      else {
        i = 1;
        ss << "f00001.png";
      }

      readImgTmp(ss.str());
      ss.str(std::string());


      //if(&currentFrame != lastFrame) {
        processImage(preproc, extractor);
      //}

      if (currentFrame.data != NULL) {
        cv::imshow(OPENCV_WINDOW, currentFrame);
        cv::waitKey(1);
      }

      //lastFrame = &currentFrame;

      ros::spinOnce();
      loop_rate.sleep();
}

ros::spin();
}
