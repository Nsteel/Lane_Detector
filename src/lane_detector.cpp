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
#include <cv.h>
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <lane_detector/DetectorConfig.h>
#include <swri_profiler/profiler.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <lane_detector/ipm.h>
#include <lane_detector/LaneDetector.hh>
#include <lane_detector/CameraInfoOpt.h>
#include <lane_detector/LaneDetectorOpt.h>
#include <lane_detector/mcv.hh>

cv_bridge::CvImagePtr currentFrame_ptr;
Preprocessor* preproc;
image_transport::Publisher resultImg_pub;
ros::Publisher detectedPoints_pub;
lane_detector::DetectorConfig dynConfig;
LaneDetector::CameraInfo cameraInfo;
LaneDetector::LaneDetectorConf lanesConf;


void setConfig() {

  //init the strucure
  lanesConf.ipmWidth = dynConfig.ipmWidth;
  lanesConf.ipmHeight = dynConfig.ipmHeight;
  lanesConf.ipmLeft = dynConfig.ipmLeft;
  lanesConf.ipmRight = dynConfig.ipmRight;
  lanesConf.ipmBottom = dynConfig.ipmBottom;
  lanesConf.ipmTop = dynConfig.ipmTop;
  lanesConf.ipmInterpolation = dynConfig.ipmInterpolation;

  lanesConf.lineWidth = dynConfig.lineWidth;
  lanesConf.lineHeight = dynConfig.lineHeight;
  lanesConf.kernelWidth = dynConfig.kernelWidth;
  lanesConf.kernelHeight = dynConfig.kernelHeight;
  lanesConf.lowerQuantile =
      dynConfig.lowerQuantile;
  lanesConf.localMaxima =
      dynConfig.localMaxima;
  lanesConf.groupingType = dynConfig.groupingType;
  lanesConf.binarize = dynConfig.binarize;
  lanesConf.detectionThreshold =
      dynConfig.detectionThreshold;
  lanesConf.smoothScores =
      dynConfig.smoothScores;
  lanesConf.rMin = dynConfig.rMin;
  lanesConf.rMax = dynConfig.rMax;
  lanesConf.rStep = dynConfig.rStep;
  lanesConf.thetaMin = dynConfig.thetaMin * CV_PI/180;
  lanesConf.thetaMax = dynConfig.thetaMax * CV_PI/180;
  lanesConf.thetaStep = dynConfig.thetaStep * CV_PI/180;
  lanesConf.ipmVpPortion = dynConfig.ipmVpPortion;
  lanesConf.getEndPoints = dynConfig.getEndPoints;
  lanesConf.group = dynConfig.group;
  lanesConf.groupThreshold = dynConfig.groupThreshold;
  lanesConf.ransac = dynConfig.ransac;

  lanesConf.ransacLineNumSamples = dynConfig.ransacLineNumSamples;
  lanesConf.ransacLineNumIterations = dynConfig.ransacLineNumIterations;
  lanesConf.ransacLineNumGoodFit = dynConfig.ransacLineNumGoodFit;
  lanesConf.ransacLineThreshold = dynConfig.ransacLineThreshold;
  lanesConf.ransacLineScoreThreshold = dynConfig.ransacLineScoreThreshold;
  lanesConf.ransacLineBinarize = dynConfig.ransacLineBinarize;
  lanesConf.ransacLineWindow = dynConfig.ransacLineWindow;

  lanesConf.ransacSplineNumSamples = dynConfig.ransacSplineNumSamples;
  lanesConf.ransacSplineNumIterations = dynConfig.ransacSplineNumIterations;
  lanesConf.ransacSplineNumGoodFit = dynConfig.ransacSplineNumGoodFit;
  lanesConf.ransacSplineThreshold = dynConfig.ransacSplineThreshold;
  lanesConf.ransacSplineScoreThreshold = dynConfig.ransacSplineScoreThreshold;
  lanesConf.ransacSplineBinarize = dynConfig.ransacSplineBinarize;
  lanesConf.ransacSplineWindow = dynConfig.ransacSplineWindow;

  lanesConf.ransacSplineDegree = dynConfig.ransacSplineDegree;

  lanesConf.ransacSpline = dynConfig.ransacSpline;
  lanesConf.ransacLine = dynConfig.ransacLine;
  lanesConf.ransacSplineStep = dynConfig.ransacSplineStep;

  lanesConf.overlapThreshold = dynConfig.overlapThreshold;

  lanesConf.localizeAngleThreshold = dynConfig.localizeAngleThreshold;
  lanesConf.localizeNumLinePixels = dynConfig.localizeNumLinePixels;

  lanesConf.extendAngleThreshold = dynConfig.extendAngleThreshold;
  lanesConf.extendMeanDirAngleThreshold = dynConfig.extendMeanDirAngleThreshold;
  lanesConf.extendLinePixelsTangent = dynConfig.extendLinePixelsTangent;
  lanesConf.extendLinePixelsNormal = dynConfig.extendLinePixelsNormal;
  lanesConf.extendContThreshold = dynConfig.extendContThreshold;
  lanesConf.extendDeviationThreshold = dynConfig.extendDeviationThreshold;
  lanesConf.extendRectTop = dynConfig.extendRectTop;
  lanesConf.extendRectBottom = dynConfig.extendRectBottom;

  lanesConf.extendIPMAngleThreshold = dynConfig.extendIPMAngleThreshold;
  lanesConf.extendIPMMeanDirAngleThreshold = dynConfig.extendIPMMeanDirAngleThreshold;
  lanesConf.extendIPMLinePixelsTangent = dynConfig.extendIPMLinePixelsTangent;
  lanesConf.extendIPMLinePixelsNormal = dynConfig.extendIPMLinePixelsNormal;
  lanesConf.extendIPMContThreshold = dynConfig.extendIPMContThreshold;
  lanesConf.extendIPMDeviationThreshold = dynConfig.extendIPMDeviationThreshold;
  lanesConf.extendIPMRectTop = dynConfig.extendIPMRectTop;
  lanesConf.extendIPMRectBottom = dynConfig.extendIPMRectBottom;

  lanesConf.splineScoreJitter = dynConfig.splineScoreJitter;
  lanesConf.splineScoreLengthRatio = dynConfig.splineScoreLengthRatio;
  lanesConf.splineScoreAngleRatio = dynConfig.splineScoreAngleRatio;
  lanesConf.splineScoreStep = dynConfig.splineScoreStep;

  lanesConf.splineTrackingNumAbsentFrames = dynConfig.splineTrackingNumAbsentFrames;
  lanesConf.splineTrackingNumSeenFrames = dynConfig.splineTrackingNumSeenFrames;

  lanesConf.mergeSplineThetaThreshold = dynConfig.mergeSplineThetaThreshold;
  lanesConf.mergeSplineRThreshold = dynConfig.mergeSplineRThreshold;
  lanesConf.mergeSplineMeanThetaThreshold = dynConfig.mergeSplineMeanThetaThreshold;
  lanesConf.mergeSplineMeanRThreshold = dynConfig.mergeSplineMeanRThreshold;
  lanesConf.mergeSplineCentroidThreshold = dynConfig.mergeSplineCentroidThreshold;

  lanesConf.lineTrackingNumAbsentFrames = dynConfig.lineTrackingNumAbsentFrames;
  lanesConf.lineTrackingNumSeenFrames = dynConfig.lineTrackingNumSeenFrames;

  lanesConf.mergeLineThetaThreshold = dynConfig.mergeLineThetaThreshold;
  lanesConf.mergeLineRThreshold = dynConfig.mergeLineRThreshold;

  lanesConf.numStrips = dynConfig.numStrips;


  lanesConf.checkSplines = dynConfig.checkSplines;
  lanesConf.checkSplinesCurvenessThreshold = dynConfig.checkSplinesCurvenessThreshold;
  lanesConf.checkSplinesLengthThreshold = dynConfig.checkSplinesLengthThreshold;
  lanesConf.checkSplinesThetaDiffThreshold = dynConfig.checkSplinesThetaDiffThreshold;
  lanesConf.checkSplinesThetaThreshold = dynConfig.checkSplinesThetaThreshold;

  lanesConf.checkIPMSplines = dynConfig.checkIPMSplines;
  lanesConf.checkIPMSplinesCurvenessThreshold = dynConfig.checkIPMSplinesCurvenessThreshold;
  lanesConf.checkIPMSplinesLengthThreshold = dynConfig.checkIPMSplinesLengthThreshold;
  lanesConf.checkIPMSplinesThetaDiffThreshold = dynConfig.checkIPMSplinesThetaDiffThreshold;
  lanesConf.checkIPMSplinesThetaThreshold = dynConfig.checkIPMSplinesThetaThreshold;

  lanesConf.finalSplineScoreThreshold = dynConfig.finalSplineScoreThreshold;

  lanesConf.useGroundPlane = dynConfig.useGroundPlane;

  lanesConf.checkColor = dynConfig.checkColor;
  lanesConf.checkColorNumBins = dynConfig.checkColorNumBins;
  lanesConf.checkColorWindow = dynConfig.checkColorWindow;
  lanesConf.checkColorNumYellowMin = dynConfig.checkColorNumYellowMin;
  lanesConf.checkColorRGMin = dynConfig.checkColorRGMin;
  lanesConf.checkColorRGMax = dynConfig.checkColorRGMax;
  lanesConf.checkColorGBMin = dynConfig.checkColorGBMin;
  lanesConf.checkColorRBMin = dynConfig.checkColorRBMin;
  lanesConf.checkColorRBFThreshold = dynConfig.checkColorRBFThreshold;
  lanesConf.checkColorRBF = dynConfig.checkColorRBF;

  lanesConf.ipmWindowClear = dynConfig.ipmWindowClear;
  lanesConf.ipmWindowLeft = dynConfig.ipmWindowLeft;
  lanesConf.ipmWindowRight = dynConfig.ipmWindowRight;

  lanesConf.checkLaneWidth = dynConfig.checkLaneWidth;
  lanesConf.checkLaneWidthMean = dynConfig.checkLaneWidthMean;
  lanesConf.checkLaneWidthStd = dynConfig.checkLaneWidthStd;

  mcvInitCameraInfo("/home/n/lane-detector/src/CameraInfo.conf", &cameraInfo);

}

void configCallback(lane_detector::DetectorConfig& config, uint32_t level)
{
        //preproc->setConfig(config);
        dynConfig = config;
        setConfig();
        ROS_DEBUG("Config was set");
}

void processImage(LaneDetector::CameraInfo& cameraInfo, LaneDetector::LaneDetectorConf& lanesConf) {

  if(currentFrame_ptr) {

    // detect lanes
    std::vector<float> lineScores, splineScores;
    std::vector<LaneDetector::Line> lanes;
    std::vector<LaneDetector::Spline> splines;
    std::vector<LaneDetector::Spline> splines_world;
    cv::Mat originalImg = currentFrame_ptr->image;
    //cv::Mat originalImg = cv::imread("/home/n/Desktop/curve.png");
    CvMat raw_mat = originalImg;
    CvMat* raw_ptr = &raw_mat;
    CvMat* mat_ptr;
    LaneDetector::mcvLoadImage(&raw_ptr, &mat_ptr);
    if(dynConfig.debug_lines) LaneDetector::DEBUG_LINES = 1;
    //LaneDetector::SHOW_IMAGE(&mat, "HOla", 1);
    mcvGetLanes(mat_ptr, raw_ptr, &lanes, &lineScores, &splines, &splines_world, &splineScores,
                &cameraInfo, &lanesConf, NULL);

       // print lanes
        for(int i=0; i<splines.size(); i++)
         {
           if (splines[i].color == LaneDetector::LINE_COLOR_YELLOW)
             mcvDrawSpline(raw_ptr, splines[i], CV_RGB(255,255,0), 3);
           else
             mcvDrawSpline(raw_ptr, splines[i], CV_RGB(0,255,0), 3);
         }

        std::cout << "frame#" << setw(8) << setfill('0') << 0 <<
          " has " << splines_world.size() << " splines" << endl;
        for (int i=0; i<splines_world.size(); ++i)
        {
          std::cout << "\tspline#" << i+1 << " has " <<
            splines_world[i].degree+1 << " points and score " <<
            splineScores[i] << endl;
          for (int j=0; j<=splines_world[i].degree; ++j)
            std::cout<< "\t\t" <<
              splines_world[i].points[j].x << ", " <<
              splines_world[i].points[j].y << endl;
          char str[256];
          sprintf(str, "%d", i);
          LaneDetector::mcvDrawText(raw_ptr, str,
                      cvPointFrom32f(splines[i].points[splines[i].degree]),
                                      1, CV_RGB(0, 0, 255));
        }
        cvReleaseMat(&mat_ptr);
  }
}

void readImg(const sensor_msgs::ImageConstPtr& img)
{

        try
        {
                currentFrame_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
                processImage(cameraInfo, lanesConf);
                //if(dynConfig.image != 0) {
                  //      currentFrame_ptr->encoding = sensor_msgs::image_encodings::MONO8;
                //}
                resultImg_pub.publish(*currentFrame_ptr->toImageMsg());
                currentFrame_ptr.reset();
        }
        catch (cv_bridge::Exception& e)
        {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
        }

}


int main(int argc, char **argv){

        preproc = new CannyEdge();
        //processImage(cameraInfo, lanesConf);
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

        //ros::MultiThreadedSpinner spinner(0); // Use one thread for core
        //spinner.spin(); // spin() will not return until the node has been shutdown
        ros::spin();
        delete preproc;
        return 0;
}
