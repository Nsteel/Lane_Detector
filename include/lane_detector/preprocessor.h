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

class Preprocessor {
public:
        Preprocessor(){
        };
        inline void setConfig(lane_detector::DetectorConfig& config) {
                this->config = config;
        };
        virtual void preprocess(cv::Mat& img) = 0;
protected:
        lane_detector::DetectorConfig config;
};

#endif /* PREPROCESSOR_H */
