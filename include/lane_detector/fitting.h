/*
 * fitting.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef FITTING_H_
#define FITTING_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <lane_detector/DetectorConfig.h>
#include <cv.h>
#include <lane_detector/LaneDetector.hh>
#include <lane_detector/utils.h>
#include <lane_detector/InversePerspectiveMapping.hh>
#include <lane_detector/fittingApproach.h>
#include <lane_detector/lane_tracker/Ctracker.h>
#include <lane_detector/Lane.h>
#include <geometry_msgs/Point32.h>
#include <lane_detector/splineCombination.h>
#include <fstream>
#include <sstream>
#include <ros/package.h>

class Fitting {
public:
        inline Fitting() {
          index = 0;
        };
        inline void setConfig(lane_detector::DetectorConfig& config) {
                this->config = config;
                lane_detector::utils::translateConfiguration(config, this->lanesConf);
                tracker.setDt(config.tracking_dt);
                tracker.setAccelNoiseMag(config.tracking_acc_noise_magnitude);
                tracker.setDistThres(config.tracking_dist_threshold);
                tracker.setMaximumAllowedSkippedFrames(config.tracking_num_absent_frames);
                tracker.setMinimumSeenFrames(config.tracking_num_seen_frames);
                tracker.setMaxTraceLength(50);
                last_lane = SplineCombination();
                driving_orientation = config.driving_orientation == 0? lane_detector::on_the_right :
                                                                       lane_detector::on_the_left;
                if(config.transform_back) {
                  save_path = ros::package::getPath("lane_detector") + "/data/new_set/results.txt";
                  outputFile.open(save_path.c_str(), ios_base::out);
                }
        };
        inline void setDrivingOrientation(lane_detector::Driving driving_orientation) {
          this->driving_orientation = driving_orientation;
          //Reinitialize Tracker
          tracker = CTracker();
          tracker.setDt(config.tracking_dt);
          tracker.setAccelNoiseMag(config.tracking_acc_noise_magnitude);
          tracker.setDistThres(config.tracking_dist_threshold);
          tracker.setMaximumAllowedSkippedFrames(config.tracking_num_absent_frames);
          tracker.setMinimumSeenFrames(config.tracking_num_seen_frames);
          tracker.setMaxTraceLength(50);
          last_lane = SplineCombination();
        }
        lane_detector::Lane fitting(cv::Mat& original, cv::Mat& preprocessed_bgr, cv::Mat& preprocessed, LaneDetector::IPMInfo& ipmInfo, LaneDetector::CameraInfo& cameraInfo, std::vector<LaneDetector::Box>& boxes);
        inline void closeFile() {
          if(config.transform_back) {
            outputFile.close();
            ROS_INFO("Results written to %s", save_path.c_str());
          }
        };
private:
        void findCurrentLane(const std::vector<cv::Point2f>& centroids, const std::vector<std::vector<cv::Point>>& splines,  SplineCombination& current_lane, cv::Mat& image);
        float calcCost(SplineCombination& combination);
        void getSpline(cv::Mat& inImage, cv::Rect& rect, std::vector<cv::Point>& spline);
        void drawSplines(cv::Mat& outImage, std::vector< std::vector<cv::Point> > & splines);
        void drawBoxes(cv::Mat& outImage, std::vector<cv::Rect>& rects);
        lane_detector::DetectorConfig config;
        LaneDetector::LaneDetectorConf lanesConf;
        LaneDetector::IPMInfo ipmInfo;
        LaneDetector::CameraInfo cameraInfo;
        CTracker tracker;
        lane_detector::Driving driving_orientation;
        SplineCombination last_lane;
        //Variables to save results
        int index;
        std::ofstream outputFile;
        string save_path;
};

#endif /* FITTING_H_ */
