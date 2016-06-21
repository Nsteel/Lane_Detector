#include <lane_detector/houghTransform.h>
#include <vector>
#include <iostream>
#include <swri_profiler/profiler.h>
#include <lane_detector/utils.h>

const int MAX_ITERATIONS = 2;
const int MIN_LINES = 2;
const int MAX_LINES = 6;
const int HOUGH_INCREMENT = 1;
/*const int MIN_LINES_left = 50;
const int MAX_LINES_left = 200;
const int MIN_LINES_right = 50;
const int MAX_LINES_right = 200;*/

void HoughTransform::extract(cv::Mat& original, cv::Mat& preprocessed, linesPair& features) {

        SWRI_PROFILE("Extract");


        lines bestLines;
        std::vector<cv::Vec2f> lines;
        int i = 0;
        while(thresholdLeft > 1 && i <= MAX_ITERATIONS && (lines.size() < MIN_LINES || lines.size() > MAX_LINES))
        {
                lines.clear();
                i++;
                cv::HoughLines(preprocessed, lines, 1, CV_PI/180, thresholdLeft);
                if(i > 1) ROS_DEBUG("Threshold:%i, Size:%lu, Automatic calibration for the Hough threshold is being performed on the left ROI", thresholdLeft, lines.size());
                if(lines.size() < MIN_LINES  && thresholdLeft > 1) thresholdLeft-=HOUGH_INCREMENT;
                else if(lines.size() > MAX_LINES) thresholdLeft+=HOUGH_INCREMENT;
        }
        ROS_DEBUG("Selected threshold for the Hough Transformation on the left ROI: %i. Detected HoughLines:%lu", thresholdLeft, lines.size());

        for(int i=0; i < lines.size(); i++) {
                float rho = lines[i][0], theta = lines[i][1];
                cv::Point pt1, pt2;
                double a = std::cos(theta), b = std::sin(theta);
                double x0 = a*rho, y0 = b*rho;
                pt1.x = cvRound(x0 + 1000*(-b));
                pt1.y = cvRound(original.rows*config.horizon_percentage/100 + y0 + 1000*(a));
                pt2.x = cvRound(x0 - 1000*(-b));
                pt2.y = cvRound(original.rows*config.horizon_percentage/100 + y0 - 1000*(a));

                //if(!filterLineLeft(pt1, pt2)) {
                        cv::line(original, pt1, pt2, cv::Scalar(255,0,255), 1, CV_AA);
                        line currentLine;
                        currentLine.push_back(pt1);
                        currentLine.push_back(pt2);
                        bestLines.push_back(currentLine);
                //}
        }

        if(config.image == 0) preprocessed = original;

        features.first = bestLines;
        features.second = bestLines;

}

bool HoughTransform::filterLineRight(cv::Point pt1, cv::Point pt2) {
        SWRI_PROFILE("filterLineRight");
        float slopeAngle = 180 - utils::calcSlopeAngle(pt1, pt2);
        //ROS_DEBUG("slopeAngle right:%f", slopeAngle);
        line l;
        l.push_back(pt1);
        l.push_back(pt2);
        if(slopeAngle > config.min_slopeAngle_right && slopeAngle < config.max_slopeAngle_right) {
                return false;
        }
        else {
                return true;
        }
}

bool HoughTransform::filterLineLeft(cv::Point pt1, cv::Point pt2) {
        float slopeAngle = utils::calcSlopeAngle(pt1, pt2);
        ROS_INFO("slopeAngle left:%f", slopeAngle);
        line l;
        l.push_back(pt1);
        l.push_back(pt2);
        if(slopeAngle > config.min_slopeAngle_left && slopeAngle < config.max_slopeAngle_left) {
                return false;
        }
        else {
                return true;
        }
}
