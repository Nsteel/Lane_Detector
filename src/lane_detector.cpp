/*
 * lane_detector.cpp
 *
 *      Author:
 *         Nicolas Acero
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <lane_detector/cannyEdge.h>
#include <lane_detector/houghTransform.h>
#include <lane_detector/fittingApproach.h>
#include <cv.h>
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <lane_detector/DetectorConfig.h>
#include <swri_profiler/profiler.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>

cv_bridge::CvImagePtr currentFrame_ptr;
Preprocessor* preproc;
FeatureExtractor<linesPair>* extractor;
Fitting<linesPair>* fittingPhase;
image_transport::Publisher resultImg_pub;
ros::Publisher detectedPoints_pub;
lane_detector::DetectorConfig dynConfig;

void configCallback(lane_detector::DetectorConfig& config, uint32_t level)
{
        preproc->setConfig(config);
        extractor->setConfig(config);
        fittingPhase->setConfig(config);
        dynConfig = config;
        ROS_DEBUG("Config was set");
}

template<class T>
void processImage(Preprocessor* preproc, FeatureExtractor<T>* extractor, Fitting<T>* fittingPhase) {
        cv::Mat copyImg;
        linesPair features;
        currentFrame_ptr->image.copyTo(copyImg);
        preproc->preprocess(currentFrame_ptr->image);
        extractor->extract(copyImg, currentFrame_ptr->image, features);
        std::vector<cv::Point> points = fittingPhase->fitting(copyImg, currentFrame_ptr->image, features);

        geometry_msgs::Point32 vanishingPoint;
        vanishingPoint.x = points[0].x;
        vanishingPoint.y = points[0].y;

        geometry_msgs::Point32 left_lane;
        left_lane.x = points[1].x;
        left_lane.y = points[1].y;

        geometry_msgs::Point32 right_lane;
        right_lane.x = points[2].x;
        right_lane.y = points[2].y;

        geometry_msgs::PolygonStamped detectedPoints;
        detectedPoints.header.stamp = ros::Time::now(); // time
        detectedPoints.header.frame_id = "base_footprint";
        detectedPoints.polygon.points = {vanishingPoint, left_lane, right_lane};
        detectedPoints_pub.publish(detectedPoints);
}

void readImg(const sensor_msgs::ImageConstPtr& img)
{

        try
        {
                currentFrame_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
                processImage(preproc, extractor, fittingPhase);
                if(dynConfig.image != 0) {
                        currentFrame_ptr->encoding = sensor_msgs::image_encodings::MONO8;
                }
                resultImg_pub.publish(*currentFrame_ptr->toImageMsg());
        }
        catch (cv_bridge::Exception& e)
        {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
        }

}


int main(int argc, char **argv){

        preproc = new CannyEdge();
        extractor =  new HoughTransform();
        fittingPhase = new FittingApproach();

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

        image_transport::Subscriber image_sub = it.subscribe("/kinect_mono_throttled", 1, readImg);
        resultImg_pub = it.advertise("lane_detector/result", 1);
        detectedPoints_pub = nh.advertise<geometry_msgs::PolygonStamped>("lane_detector/vanishing_point", 1);

        ros::MultiThreadedSpinner spinner(0); // Use one thread for core
        spinner.spin(); // spin() will not return until the node has been shutdown
        //ros::spin();
        return 0;
}
