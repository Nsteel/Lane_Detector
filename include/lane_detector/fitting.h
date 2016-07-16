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
#include <lane_detector/LaneDetectorOpt.h>
#include <lane_detector/utils.h>
#include <lane_detector/InversePerspectiveMapping.hh>
#include <lane_detector/fittingApproach.h>

class Fitting {
public:
        inline Fitting(){
          kalmanFilters.push_back(cv::KalmanFilter(4,2,0));
          kalmanFilters.push_back(cv::KalmanFilter(4,2,0));
          kalmanFilters.push_back(cv::KalmanFilter(4,2,0));
        };
        inline void setConfig(lane_detector::DetectorConfig& config) {
                this->config = config;
                lane_detector::utils::translateConfiguration(config, this->lanesConf);
        };
        void fitting(cv::Mat& original, cv::Mat& preprocessed, std::vector<LaneDetector::Box>& boxes);
private:
        void initTracking(const std::vector<LaneDetector::Box>& ipmBoxes);
        void trackBoxes(std::vector<LaneDetector::Box>& ipmBoxes);
        lane_detector::DetectorConfig config;
        LaneDetector::LaneDetectorConf lanesConf;
        cv::Point last_line_startPoint = cv::Point(0,0);
        cv::Point last_line_endPoint = cv::Point(0,0);
        std::vector<cv::KalmanFilter> kalmanFilters;
        uint32_t count_non_valid = 0;
        uint32_t count_non_right = 0;
        bool kf_initialized = false;
};

#endif /* FITTING_H_ */
