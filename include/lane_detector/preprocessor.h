/*
 * Preprocessor.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef PREPROCESSOR_H_
#define PREPROCESSOR_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <lane_detector/DetectorConfig.h>
#include <cv.h>
#include <lane_detector/LaneDetector.hh>
#include <lane_detector/utils.h>
#include <lane_detector/InversePerspectiveMapping.hh>

class Preprocessor {
public:
        inline Preprocessor(){
          kernel_x = (cv::Mat_<float>(5,1) << -0.252732, 0.111711, 0.427063, 0.111711, -0.252732);
          kernel_y = (cv::Mat_<float>(5,1) << 0.999903, 0.999976, 1, 0.999976, 0.999903);
        };
        inline void setCameraInfo(LaneDetector::CameraInfo& cameraInfo) {
          this->cameraInfo = cameraInfo;
        };
        inline void setConfig(lane_detector::DetectorConfig& config) {
                this->config = config;
                lane_detector::utils::translateConfiguration(config, this->lanesConf);
        };
        void preprocess(cv::Mat& originalImg, cv::Mat& img, LaneDetector::IPMInfo& ipmInfo, LaneDetector::CameraInfo& cameraInfo_);
private:
        lane_detector::DetectorConfig config;
        LaneDetector::LaneDetectorConf lanesConf;
        LaneDetector::CameraInfo cameraInfo;
        LaneDetector::IPMInfo ipmInfo;
        cv::Mat kernel_x;
        cv::Mat kernel_y;
};

#endif /* PREPROCESSOR_H */
