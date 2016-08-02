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
        Preprocessor(){
        };
        inline void setCameraInfo(LaneDetector::CameraInfo& cameraInfo) {
          this->cameraInfo = cameraInfo;
        };
        inline void setConfig(lane_detector::DetectorConfig& config) {
                this->config = config;
                lane_detector::utils::translateConfiguration(config, this->lanesConf);
        };
        void preprocess(cv::Mat& img, LaneDetector::IPMInfo& ipmInfo);
private:
        lane_detector::DetectorConfig config;
        LaneDetector::LaneDetectorConf lanesConf;
        LaneDetector::CameraInfo cameraInfo;
        LaneDetector::IPMInfo ipmInfo;
};

#endif /* PREPROCESSOR_H */
