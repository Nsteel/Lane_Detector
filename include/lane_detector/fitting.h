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
        Fitting(){
        };
        inline void setConfig(lane_detector::DetectorConfig& config) {
                this->config = config;
                lane_detector::utils::translateConfiguration(config, this->lanesConf);
        };
        void fitting(cv::Mat& original, cv::Mat& preprocessed, std::vector<LaneDetector::Box>& boxes);
private:
        lane_detector::DetectorConfig config;
        LaneDetector::LaneDetectorConf lanesConf;
        cv::Point last_left_centroid = cv::Point(0,0);
        cv::Point last_right_centroid = cv::Point(0,0);
        uint32_t count_non_valid = 0;
        //uint32_t count_non_right = 0;
};

#endif /* FITTING_H_ */
