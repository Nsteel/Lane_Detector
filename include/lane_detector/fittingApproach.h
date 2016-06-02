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
typedef std::pair<lines, lines> linesPair;

class FittingApproach : public Fitting<linesPair> {
public:
        FittingApproach() : timeOut(timeOut.fromNSec(1000000000)){
        };
        inline void setConfig(lane_detector::DetectorConfig& config) {
                Fitting::setConfig(config);
        };
        std::vector<cv::Point> fitting(cv::Mat& original, cv::Mat& preprocessed, linesPair& features);
private:
        cv::Point findVanishingPoint();
        line clusterLines(std::vector<line>& lines, int imageWidth, cv::Point vanishingPoint, bool is_right);
        void filterLines(std::vector<line>& lines, cv::Point vanishingPoint);
        void pixelVoting(lines::iterator& it, cv::Mat& img);
        void setPixel(int x, int y, cv::Mat& img);
        line lastLeftLine;
        line lastRightLine;
        lines bestLinesLeft;
        lines bestLinesRight;
        cv::Point vanishingPoint;
        cv::Mat currentFrame;
        cv::Mat preprocessed;
        ros::Time lastUpdate;
        ros::Time timeOut;
};

#endif /* FITTINGAPPROACH_H_ */
