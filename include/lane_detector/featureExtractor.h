/*
 * FeatureExtractor.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef FEATUREEXTRACTOR_H_
#define FEATUREEXTRACTOR_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <lane_detector/DetectorConfig.h>
#include <cv.h>

template<typename T>
class FeatureExtractor {
public:
        FeatureExtractor(){
        };
        virtual inline void setConfig(lane_detector::DetectorConfig& config) {
                this->config = config;
        };
        virtual void extract(cv::Mat& original, cv::Mat& preprocessed, T& features) = 0;
protected:
        lane_detector::DetectorConfig config;
};

#endif /* FEATUREEXTRACTOR_H_ */
