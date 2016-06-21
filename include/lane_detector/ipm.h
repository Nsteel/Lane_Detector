/*
 * ipm.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef IPM_H_
#define IPM_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>

class IPM {
public:
        IPM(){
        };
        void applyHomography(cv::Mat& inputImg, cv::Mat& dstImg);
private:
        const double focal_lenght = 270.076996;
        const double z_distance = 300.0;
        const double alpha = (12.0-90.)*CV_PI/180;
        const double gamma = (90-90.)*CV_PI/180; //84 was better
};

#endif /* IPM_H_ */
