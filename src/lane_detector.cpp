/*
 * lane_detector.cpp
 *
 *      Author:
 *         Nicolas Acero
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <lane_detector/preprocessor.h>
#include <lane_detector/featureExtractor.h>
#include <lane_detector/fitting.h>
#include <cv.h>
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <lane_detector/DetectorConfig.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <lane_detector/LaneDetector.hh>
#include <lane_detector/mcv.hh>
#include <lane_detector/utils.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>

cv_bridge::CvImagePtr currentFrame_ptr;
Preprocessor preproc;
FeatureExtractor extractor;
Fitting fitting_phase;
image_transport::Publisher resultImg_pub;
ros::Publisher lane_pub;
lane_detector::DetectorConfig dynConfig;
LaneDetector::CameraInfo cameraInfo;
LaneDetector::LaneDetectorConf lanesConf;


/**
 * readCameraInfo reads and sets the camera parameters if received on topic "camera_info".
 * Otherwise the parameters are set with some constant values related to the camera used
 * in our experiments.
 */
void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& cm, bool* done) {

  if(cm != NULL) {
    cameraInfo.focalLength.x = cm->P[0];
    cameraInfo.focalLength.y = cm->P[5];
    cameraInfo.opticalCenter.x = cm->P[2];
    cameraInfo.opticalCenter.y = cm->P[6];
    cameraInfo.imageWidth = cm->width;
    cameraInfo.imageHeight = cm->height;
    cameraInfo.cameraHeight = dynConfig.camera_height;
    cameraInfo.pitch = dynConfig.camera_pitch * CV_PI/180;
    cameraInfo.yaw = 0.0;
  }
  else {
    cameraInfo.focalLength.x = 270.076996;
    cameraInfo.focalLength.y = 300.836426;
    cameraInfo.opticalCenter.x = 325.678818;
    cameraInfo.opticalCenter.y = 250.211312;
    cameraInfo.imageWidth = 640;
    cameraInfo.imageHeight = 480;
    cameraInfo.cameraHeight = dynConfig.camera_height;
    cameraInfo.pitch = dynConfig.camera_pitch * CV_PI/180;
    cameraInfo.yaw = 0.0;
  }

  *done = true;
}

//Callback function for Dynamic Reconfigre
void configCallback(lane_detector::DetectorConfig& config, uint32_t level)
{
        preproc.setConfig(config);
        extractor.setConfig(config);
        fitting_phase.setConfig(config);
        dynConfig = config;
        ROS_DEBUG("Config was set");
}

//Callback function for topic "lane_detector/driving_orientation"
void drivingOrientationCB(const std_msgs::Int32::ConstPtr& driving_orientation)
{
  if(driving_orientation->data == 0)
      fitting_phase.setDrivingOrientation(lane_detector::on_the_right);
  else if(driving_orientation->data == 1)
      fitting_phase.setDrivingOrientation(lane_detector::on_the_left);
}

void processImage(LaneDetector::CameraInfo& cameraInfo, LaneDetector::LaneDetectorConf& lanesConf) {
  if(currentFrame_ptr) {
    //information paramameters of the IPM transform
    LaneDetector::IPMInfo ipmInfo;
    // detect bounding boxes arround the lanes
    std::vector<LaneDetector::Box> boxes;
    cv::Mat processed_bgr = currentFrame_ptr->image;
    preproc.preprocess(currentFrame_ptr->image, processed_bgr, ipmInfo, cameraInfo);
    cv::Mat preprocessed = processed_bgr.clone();
    lane_detector::utils::scaleMat(processed_bgr, processed_bgr);
    if(processed_bgr.channels() == 1) cv::cvtColor(processed_bgr, processed_bgr, CV_GRAY2BGR);
    extractor.extract(processed_bgr, preprocessed, boxes);
    lane_detector::Lane current_lane = fitting_phase.fitting(currentFrame_ptr->image, processed_bgr, preprocessed, ipmInfo, cameraInfo, boxes);
    lane_pub.publish(current_lane);

    cv::imshow("Out", processed_bgr);
    cv::waitKey(1);

    //cv::line(currentFrame_ptr->image, cv::Point((currentFrame_ptr->image.cols-1)/2, 0), cv::Point((currentFrame_ptr->image.cols-1)/2, currentFrame_ptr->image.rows), cv::Scalar(0, 255, 239), 1);
  }
}

//Callback function for a new image on topic "image".
void readImg(const sensor_msgs::ImageConstPtr& img) {

        try
        {
                currentFrame_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
                processImage(cameraInfo, lanesConf);
                resultImg_pub.publish(*currentFrame_ptr->toImageMsg());
        }
        catch (cv_bridge::Exception& e)
        {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
        }

}

void laneDetectionFromFiles(std::string& path) {

std::vector<std::string> fileNames;

if (boost::filesystem::is_directory(path))
 {
   for (boost::filesystem::directory_iterator itr(path); itr!=boost::filesystem::directory_iterator(); ++itr)
   {
     if (boost::filesystem::is_regular_file(itr->status()) && (itr->path().filename().string().find(".jpg") != std::string::npos || itr->path().filename().string().find(".png") != std::string::npos)) {
       fileNames.push_back(itr->path().filename().string());
     }
   }

   std::sort(fileNames.begin(), fileNames.end());

   std::vector<cv_bridge::CvImagePtr> frames;
   sensor_msgs::Image currentFrame;

   for(int i = 0; i < fileNames.size(); i++) {
     cv::Mat img = cv::imread(path + "/" + fileNames.at(i));
     cv_bridge::CvImage img_bridge;
     std_msgs::Header header; // empty header
     header.stamp = ros::Time::now(); // time
     img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
     currentFrame = *img_bridge.toImageMsg(); // from cv_bridge to sensor_msgs::Image
     if(currentFrame.step == 0) {
       std::cout << "Error: No image with name " << fileNames.at(i) << " received" << std::endl;
     }
     try
     {
             currentFrame_ptr = cv_bridge::toCvCopy(currentFrame, sensor_msgs::image_encodings::BGR8);
             frames.push_back(currentFrame_ptr);
     }
     catch (cv_bridge::Exception& e)
     {
             ROS_ERROR("cv_bridge exception: %s", e.what());
             return;
     }
   }
   int i = 0;
   int64 t0 = cv::getTickCount();
   while(i < frames.size()) {
     currentFrame_ptr = frames.at(i);
     processImage(cameraInfo, lanesConf);
     i++;
   }
   int64 t1 = cv::getTickCount();
   double secs = (t1-t0)/cv::getTickFrequency();
   ROS_INFO("%i images processed in %f seconds. Frequency: %fHz", i, secs, (float)i/secs);
   fitting_phase.closeFile();
 }
}


int main(int argc, char **argv){

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

        bool info_set = false;
        bool loadFiles = false;
        ros::param::get("~images_from_folder", loadFiles);
        /**
        * Read camera information
        * IMPORTANT: If images are loaded from a folder the camera parameters have to be set
        * inside the function readCameraInfo (lane_detector.cpp::57)
        */
        ros::Subscriber cameraInfo_sub = nh.subscribe<sensor_msgs::CameraInfo>("camera_info", 1, std::bind(readCameraInfo, std::placeholders::_1, &info_set));
        if(loadFiles) readCameraInfo(NULL,&info_set);
        while (!info_set) {
          ros::spinOnce();
          ROS_WARN("No information on topic camera_info received");
        }

        //Stop the Subscriber
        cameraInfo_sub.shutdown();

        //Set cameraInfo
        preproc.setCameraInfo(cameraInfo);

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

        /**
         * Subscriber on topic "lane_detector/driving_orientation".
         * This subscriber allows to change lane while driving and to select the desired
         * driving direction
         */
        ros::Subscriber driving_orientation_sub = nh.subscribe<std_msgs::Int32>("lane_detector/driving_orientation", 1, drivingOrientationCB);
        image_transport::Subscriber image_sub = it.subscribe("/image", 1, readImg);
        resultImg_pub = it.advertise("lane_detector/result", 1);
        lane_pub = nh.advertise<lane_detector::Lane>("lane_detector/lane", 1);

        std::string imagesPath = "";
        ros::param::get("~images_path", imagesPath);
        if(loadFiles) laneDetectionFromFiles(imagesPath); // Whether to load the images from a folder (data set) or from the kinect

        //ros::MultiThreadedSpinner spinner(0); // Use one thread for core
        //spinner.spin(); // spin() will not return until the node has been shutdown
        ros::spin();
        return 0;
}
