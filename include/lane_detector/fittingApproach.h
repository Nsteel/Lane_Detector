/*
 * fittingApproach.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef FITTINGAPPROACH_H_
#define FITTINGAPPROACH_H_

#include <ros/ros.h>
#include <cv.h>

class FittingApproach {
public:
        FittingApproach(){
        };
        void fitting(cv::Mat& preprocessed, std::vector<cv::Point>& splinePoints);
};

#endif /* FITTINGAPPROACH_H_ */
