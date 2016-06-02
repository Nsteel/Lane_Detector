#include <lane_detector/houghTransform.h>
#include <vector>
#include <iostream>
#include <swri_profiler/profiler.h>
#include <lane_detector/utils.h>

const int MAX_ITERATIONS = 2;
const int MIN_LINES = 50;
const int MAX_LINES = 200;
const int HOUGH_INCREMENT = 3;
/*const int MIN_LINES_left = 50;
const int MAX_LINES_left = 200;
const int MIN_LINES_right = 50;
const int MAX_LINES_right = 200;*/

void HoughTransform::extract(cv::Mat& original, cv::Mat& preprocessed, linesPair& features) {

        SWRI_PROFILE("Extract");

        cv::Mat leftROI = preprocessed(cv::Rect(cv::Point(0, 0), cv::Point(original.cols*0.5, preprocessed.rows)));
        cv::Mat rightROI = preprocessed(cv::Rect(cv::Point(original.cols*0.5+1, 0), cv::Point(original.cols, preprocessed.rows)));

        lines bestLinesLeft;
        std::vector<cv::Vec2f> leftLines;
        int i = 0;
        while(thresholdLeft > 1 && i <= MAX_ITERATIONS && (leftLines.size() < MIN_LINES || leftLines.size() > MAX_LINES))
        {
                leftLines.clear();
                i++;
                cv::HoughLines(leftROI, leftLines, 1, CV_PI/180, thresholdLeft);
                if(i > 1) ROS_DEBUG("Threshold:%i, Size:%lu, Automatic calibration for the Hough threshold is being performed on the left ROI", thresholdLeft, leftLines.size());
                if(leftLines.size() < MIN_LINES  && thresholdLeft > 1) thresholdLeft-=HOUGH_INCREMENT;
                else if(leftLines.size() > MAX_LINES) thresholdLeft+=HOUGH_INCREMENT;
        }
        ROS_DEBUG("Selected threshold for the Hough Transformation on the left ROI: %i. Detected HoughLines:%lu", thresholdLeft, leftLines.size());

        for(int i=0; i < leftLines.size(); i++) {
                float rho = leftLines[i][0], theta = leftLines[i][1];
                cv::Point pt1, pt2;
                double a = std::cos(theta), b = std::sin(theta);
                double x0 = a*rho, y0 = b*rho;
                pt1.x = cvRound(x0 + 1000*(-b));
                pt1.y = cvRound(original.rows*config.horizon_percentage/100 + y0 + 1000*(a));
                pt2.x = cvRound(x0 - 1000*(-b));
                pt2.y = cvRound(original.rows*config.horizon_percentage/100 + y0 - 1000*(a));

                if(!filterLineLeft(pt1, pt2, original.rows, original.cols)) {
                        cv::line(original, pt1, pt2, cv::Scalar(255,0,255), 1, CV_AA);
                        line currentLine;
                        currentLine.push_back(pt1);
                        currentLine.push_back(pt2);
                        bestLinesLeft.push_back(currentLine);
                }
        }

        lines bestLinesRight;
        std::vector<cv::Vec2f> rightLines;
        i = 0;
        while(thresholdRight > 1 && i <= MAX_ITERATIONS && (rightLines.size() < MIN_LINES || rightLines.size() > MAX_LINES)) {
                rightLines.clear();
                i++;
                cv::HoughLines(rightROI, rightLines, 1, CV_PI/180, thresholdRight);
                if(i > 1) ROS_DEBUG("Threshold:%i, Size:%lu, Automatic calibration for the Hough threshold is being performed on the right ROI", thresholdRight, rightLines.size());
                if(rightLines.size() < MIN_LINES  && thresholdRight > 1) thresholdRight-=HOUGH_INCREMENT;
                else if(rightLines.size() > MAX_LINES) thresholdRight+=HOUGH_INCREMENT;
        }
        ROS_DEBUG("Selected threshold for the Hough Transformation on the right ROI: %i. Detected HoughLines:%lu", thresholdRight, rightLines.size());

        for(int i=0; i < rightLines.size(); i++) {
                float rho = rightLines[i][0], theta = rightLines[i][1];
                cv::Point pt1, pt2;
                double a = std::cos(theta), b = std::sin(theta);
                double x0 = a*rho, y0 = b*rho;
                pt1.x = cvRound(original.cols*0.5+1 + x0 + 1000*(-b));
                pt1.y = cvRound(original.rows*config.horizon_percentage/100 + y0 + 1000*(a));
                pt2.x = cvRound(original.cols*0.5+1 + x0 - 1000*(-b));
                pt2.y = cvRound(original.rows*config.horizon_percentage/100 + y0 - 1000*(a));

                if(!filterLineRight(pt1, pt2, original.rows, original.cols)) {
                        cv::line(original, pt1, pt2, cv::Scalar(255,0,255), 1, CV_AA);
                        line currentLine;
                        currentLine.push_back(pt1);
                        currentLine.push_back(pt2);
                        bestLinesRight.push_back(currentLine);
                }
        }

        switch (config.image) {
        case 0:
                preprocessed = original;
                break;
        case 2:
                preprocessed = leftROI;
                break;
        case 3:
                preprocessed = rightROI;
                break;
        }

        features.first = bestLinesLeft;
        features.second = bestLinesRight;

}

bool HoughTransform::filterLineRight(cv::Point pt1, cv::Point pt2, int imageRows, int imageCols) {
        SWRI_PROFILE("filterLineRight");
        float slopeAngle = 180 - utils::calcSlopeAngle(pt1, pt2);
        //ROS_DEBUG("slopeAngle right:%f", slopeAngle);
        line l;
        l.push_back(pt1);
        l.push_back(pt2);
        int x = cvRound(utils::extrapolateLineX(imageRows, l));
        if(slopeAngle > config.min_slopeAngle_right && slopeAngle < config.max_slopeAngle_right && x >= imageCols/2) {
                return false;
        }
        else {
                return true;
        }
}

bool HoughTransform::filterLineLeft(cv::Point pt1, cv::Point pt2, int imageRows, int imageCols) {
        float slopeAngle = utils::calcSlopeAngle(pt1, pt2);
        //ROS_DEBUG("slopeAngle left:%f", slopeAngle);
        line l;
        l.push_back(pt1);
        l.push_back(pt2);
        int x = cvRound(utils::extrapolateLineX(imageRows, l));
        if(slopeAngle > config.min_slopeAngle_left && slopeAngle < config.max_slopeAngle_left  && x <= imageCols/2) {
                return false;
        }
        else {
                return true;
        }
}
