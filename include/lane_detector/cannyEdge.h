/*
 * CannyEdge.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef CANNYEDGE_H_
#define CANNYEDGE_H_

#include <lane_detector/preprocessor.h>

class CannyEdge : public Preprocessor {
public:
        CannyEdge() : Preprocessor(){
        };
        void preprocess(cv::Mat& img);
};

#endif /* CANNYEDGE_H_ */
