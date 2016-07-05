/*
 * fittingApproach.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef FITTINGAPPROACH_H_
#define FITTINGAPPROACH_H_

#include <lane_detector/fitting.h>
#include <ros/ros.h>
typedef std::vector<cv::Point> line;
typedef std::vector<line> lines;

class FittingApproach : public Fitting {
public:
        FittingApproach(){
        };
        inline void setConfig(lane_detector::DetectorConfig& config) {
                Fitting::setConfig(config);
        };
        std::vector<cv::Point> fitting(cv::Mat& original, cv::Mat& preprocessed);
};

#endif /* FITTINGAPPROACH_H_ */
