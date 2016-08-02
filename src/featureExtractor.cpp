#include <lane_detector/featureExtractor.h>
#include <vector>
#include <iostream>


void FeatureExtractor::extract(cv::Mat& original, cv::Mat& preprocessed, std::vector<LaneDetector::Box>& boxes) {

        CvMat preprocessed_mat = preprocessed;
        CvMat* preprocessed_ptr = &preprocessed_mat;

        std::vector<float> lineScores;
        std::vector<LaneDetector::Line> lanes;

        //get the initial lines
        LaneDetector::getLines(preprocessed_ptr, LaneDetector::LINE_VERTICAL, lanes, lineScores, &lanesConf);
        for(unsigned int ind = 0; ind < lineScores.size(); ind++)
        {
          lanes[ind].score = lineScores[ind];
        }

        mcvGetLinesBoundingBoxesVec(lanes, LaneDetector::LINE_VERTICAL, cvSize(preprocessed.cols-1, preprocessed.rows-1), boxes);
        mcvGroupBoundingBoxesVec(boxes, LaneDetector::LINE_VERTICAL, lanesConf.overlapThreshold);
        ROS_DEBUG("Bounding boxes count:%lu", boxes.size());
        if(boxes.size() > config.max_num_lanes) {
          std::sort(boxes.begin(), boxes.end(), lane_detector::utils::compareBoxes);
          auto last_box = boxes.begin() + config.max_num_lanes;
          boxes = std::vector<LaneDetector::Box>(boxes.begin(), last_box);
        }

        LaneDetector::mcvScaleMat(preprocessed_ptr, preprocessed_ptr);
        preprocessed = cv::cvarrToMat(preprocessed_ptr, true);
        //cvReleaseMat(&preprocessed_ptr);

        if(config.draw_lines) {
          for (LaneDetector::Box box : boxes)
          {
            cv::Point startPoint(box.line.startPoint.x, box.line.startPoint.y);
            cv::Point endPoint(box.line.endPoint.x, box.line.endPoint.y);
            cv::clipLine(cv::Size(original.cols, original.rows), startPoint, endPoint);
            cv::line(original, startPoint, endPoint, cv::Scalar(0,255,0));
        }
      }
}
