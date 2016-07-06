#include <lane_detector/featureExtractor.h>
#include <vector>
#include <iostream>
#include <swri_profiler/profiler.h>


void FeatureExtractor::extract(cv::Mat& preprocessed, std::vector<LaneDetector::Line>& lanes) {

        SWRI_PROFILE("Extract");

        CvMat preprocessed_mat = preprocessed;
        CvMat* preprocessed_ptr = &preprocessed_mat;

        std::vector<float> lineScores;

        //get the initial lines
        LaneDetector::getLines(preprocessed_ptr, LaneDetector::LINE_VERTICAL, lanes, lineScores, &lanesConf);
        for(unsigned int ind = 0; ind < lineScores.size(); ind++)
        {
          lanes[ind].score = lineScores[ind];
        }
        ROS_DEBUG("Lines Size:%lu", lanes.size());
        if(lanes.size() > 3) {
          std::sort(lanes.begin(), lanes.end(), utils::compareLanes);
          auto last_lane = lanes.begin() + 3;
          lanes = std::vector<LaneDetector::Line>(lanes.begin(), last_lane);
        }

        LaneDetector::mcvScaleMat(preprocessed_ptr, preprocessed_ptr);
        preprocessed = cv::cvarrToMat(preprocessed_ptr, true);
        cv::cvtColor(preprocessed, preprocessed, CV_GRAY2BGR);

        if(config.draw_lines) {
          for (LaneDetector::Line line : lanes)
          {
            cv::Point startPoint(line.startPoint.x, line.startPoint.y);
            cv::Point endPoint(line.endPoint.x, line.endPoint.y);
            cv::clipLine(cv::Size(preprocessed.cols, preprocessed.rows), startPoint, endPoint);
            cv::line(preprocessed, startPoint, endPoint, cv::Scalar(0,255,0));
        }
      }
      cvReleaseMat(&preprocessed_ptr);
}
