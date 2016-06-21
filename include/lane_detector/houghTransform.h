/*
 * HoughTransform.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef HOUGHTRANSFORM_H_
#define HOUGHTRANSFORM_H_

#include <lane_detector/featureExtractor.h>
typedef std::vector<cv::Point> line;
typedef std::vector<line> lines;
typedef std::pair<lines, lines> linesPair;

class HoughTransform : public FeatureExtractor<linesPair> {
public:
        HoughTransform(){
        };
        void extract(cv::Mat& original, cv::Mat& preprocessed, linesPair& features);
        inline void setConfig(lane_detector::DetectorConfig& config) {
                FeatureExtractor::setConfig(config);
                thresholdLeft = config.hough_threshold_left_ROI;
                thresholdRight = config.hough_threshold_right_ROI;
        };
private:
        bool filterLineLeft(cv::Point pt1, cv::Point pt2);
        bool filterLineRight(cv::Point pt1, cv::Point pt2);
        int thresholdLeft;
        int thresholdRight;

};

#endif /* HOUGHTRANSFORM_H_ */
