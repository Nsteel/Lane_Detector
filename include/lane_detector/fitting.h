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
#include <vector>

typedef std::vector<cv::Point> line;

template<typename T>
class Fitting {
public:
        Fitting(){
        };
        inline virtual void setConfig(lane_detector::DetectorConfig& config) {
                this->config = config;
        };
        virtual std::vector<cv::Point> fitting(cv::Mat& original, cv::Mat& preprocessed, T& features) = 0;
protected:
        lane_detector::DetectorConfig config;
};

#endif /* FITTING_H_ */
