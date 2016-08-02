#include <lane_detector/fitting.h>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <vector>


float Fitting::calcCost(SplineCombination& combination) {
  return combination.calcCost(last_lane);
}

lane_detector::Lane Fitting::fitting(cv::Mat& original, cv::Mat& preprocessed, LaneDetector::IPMInfo& ipmInfo, std::vector<LaneDetector::Box>& ipmBoxes)
{

        this->ipmInfo = ipmInfo;

        if(last_lane.num_absent_frames >= config.lane_num_absent_frames) last_lane = SplineCombination();

        cv::Scalar Colors[] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255), cv::Scalar(255, 127, 255), cv::Scalar(127, 0, 255), cv::Scalar(127, 0, 127) };
        FittingApproach fitSpline;
        std::vector<cv::Point> splinePoints;
        std::vector< std::vector<cv::Point> > splines;
        std::vector<cv::Point2f> centroids;
        std::vector<cv::Rect> rects;
        SplineCombination current_lane;
        lane_detector::Lane current_lane_msg;
        lane_detector::utils::boxes2Rects(ipmBoxes, rects);
        lane_detector::utils::getRectsCentroids(rects, centroids);
        tracker.Update(centroids, rects, CTracker::RectsDist);
        rects = tracker.getLastRects();
        //std::cout << "rects_size: " << rects.size() << std::endl;
        std::vector<CvRect> rectsAux(rects.begin(), rects.end());
        LaneDetector::mcvGroupBoundingBoxes(rectsAux, LaneDetector::LINE_VERTICAL, lanesConf.groupThreshold);
        rects = std::vector<cv::Rect>(rectsAux.begin(), rectsAux.end());
        lane_detector::utils::getRectsCentroids(rects, centroids);
        ROS_DEBUG("Lanes detected: %lu", rects.size());

        for(cv::Rect bounding_box : rects) {
          fitSpline.fitting(preprocessed, bounding_box, splinePoints);
          std::sort(splinePoints.begin(), splinePoints.end(), lane_detector::utils::sortPointsY);
          splines.push_back(splinePoints);

          // draw the spline
	        if(config.draw_splines) lane_detector::utils::drawSpline(original, splinePoints, 2, cv::Scalar(255, 0, 0));

          if(config.draw_boxes) cv::rectangle(original, cv::Point(bounding_box.x, bounding_box.y),
                                                  cv::Point(bounding_box.x + bounding_box.width-1, bounding_box.y + bounding_box.height-1),
                                                  cv::Scalar(0,0,255));

          splinePoints.clear();
        }

        //std::cout << "centroids: " << centroids.size() << std::endl;
        if(centroids.size() >= 2) {
          findCurrentLane(centroids, splines, current_lane, original);
          if(current_lane.correct) last_lane = current_lane;
          else if(!current_lane.correct && last_lane.correct && last_lane.num_absent_frames < config.lane_num_absent_frames) {
            current_lane = last_lane;
            last_lane.num_absent_frames++;
          }
        }
        else if(last_lane.correct && last_lane.num_absent_frames < config.lane_num_absent_frames) {
          current_lane = last_lane;
          last_lane.num_absent_frames++;
        }
        //std::cout << "splines size: " << splines.size() << std::endl;
        //if(splines.size() == 3) {
          //std::cout << "1: " << splines[0].size() << " 2: " << splines[1].size() << " 3: " << splines[2].size() << std::endl;
        //}

        if(current_lane.correct) {
          //std::cout << "lane_width: " << current_lane.lane_width << std::endl;
          cv::circle(original, current_lane.centroid_spline1, 5, cv::Scalar(0,255,0), 2);
          cv::circle(original, current_lane.centroid_spline2, 5, cv::Scalar(0,255,0), 2);
          if(config.draw_splines) lane_detector::utils::drawSpline(original, current_lane.spline1, 2, cv::Scalar(0, 255, 0));
          if(config.draw_splines) lane_detector::utils::drawSpline(original, current_lane.spline2, 2, cv::Scalar(0, 255, 0));
        }
        if(config.draw_tracked_centroids) {
          for(int i = 0; i < tracker.tracks.size(); i++) {
              if (tracker.tracks[i]->trace.size() >= 1)
              {
              cv::circle(original, tracker.tracks[i]->trace.back(), 2, Colors[tracker.tracks[i]->track_id % 9], 2);
              }
            }
        }
        //std::cout << "Tracks: " << tracker.tracks.size() << std::endl;

        std::vector<cv::Point> longest_spline;
        std::vector<cv::Point> left_spline;
        std::vector<cv::Point> right_spline;
        std::vector<cv::Point> guide_spline;

        if(current_lane.correct) {

          longest_spline = current_lane.longest_spline;
          right_spline = current_lane.right_spline;
          left_spline = current_lane.left_spline;

          for(int i = 0; i < longest_spline.size(); i++) {
            if(longest_spline == left_spline) {
              cv::Point p = longest_spline[i];
              p.x += current_lane.lane_width/2;
              guide_spline.push_back(p);
            }
            else { //longest_spline == right_spline
              cv::Point p = longest_spline[i];
              p.x -= current_lane.lane_width/2;
              guide_spline.push_back(p);
            }
          }

          // draw the guide spline
           if(config.draw_splines) lane_detector::utils::drawSpline(original, guide_spline, 1, cv::Scalar(0, 255, 255));


           //Convert cv::Point to cv::Point2f
           std::vector<cv::Point2f> left_spline_float = lane_detector::utils::cvtCvPoint2CvPoint2f(left_spline);
           std::vector<cv::Point2f> right_spline_float = lane_detector::utils::cvtCvPoint2CvPoint2f(right_spline);
           std::vector<cv::Point2f> guide_spline_float = lane_detector::utils::cvtCvPoint2CvPoint2f(guide_spline);

           //Transform splines to wolrd coordinates (in m)
           lane_detector::utils::ipmPoints2World(guide_spline_float, guide_spline_float, ipmInfo);
           lane_detector::utils::ipmPoints2World(right_spline_float, right_spline_float, ipmInfo);
           lane_detector::utils::ipmPoints2World(left_spline_float, left_spline_float, ipmInfo);

           std::vector<geometry_msgs::Point32> left_spline_ros;
           std::vector<geometry_msgs::Point32> right_spline_ros;
           std::vector<geometry_msgs::Point32> guide_spline_ros;

           //Convert Cv-Points to ROS-points (geometry_msgs::Point32)
           lane_detector::utils::cvtCvPoints2ROSPoints(guide_spline_float, guide_spline_ros);
           lane_detector::utils::cvtCvPoints2ROSPoints(right_spline_float, right_spline_ros);
           lane_detector::utils::cvtCvPoints2ROSPoints(left_spline_float, left_spline_ros);

           current_lane_msg.guide_line = guide_spline_ros;
           current_lane_msg.right_line = right_spline_ros;
           current_lane_msg.left_line = left_spline_ros;
        }
      //cv::line(original, cv::Point(car_position.x, 0), cv::Point(car_position.x, lanesConf.ipmHeight-1), cv::Scalar(0, 255, 239), 1);
      return current_lane_msg;
}

// -------------------------------------------------
// Solving assignment problem for the lane
// ------------------------------------------------
void Fitting::findCurrentLane(const std::vector<cv::Point2f>& centroids, const std::vector<std::vector<cv::Point>>& splines, SplineCombination& current_lane, cv::Mat& image) {

  std::vector<SplineCombination> spline_combinations;

  assignments_t assignment;

  lane_detector::utils::makeSplineCombinations(config, ipmInfo, centroids, splines, spline_combinations);

  //One lanes has to be assigned
  size_t N = 1;
  size_t M = spline_combinations.size();
  distMatrix_t cost(N * M);

  //std::cout << "Combis: " << spline_combinations.size() << std::endl;

  if(spline_combinations.size() > 1) {
      for (size_t i = 0; i < N; i++)
    {
      for (size_t j = 0; j <  spline_combinations.size(); j++)
      {
        cost[i + j * N] = calcCost(spline_combinations[j]);
        //std::cout << "Cost: " << cost[i+j*N] << std::endl;
      }
    }

    AssignmentProblemSolver APS;
    APS.Solve(cost, N, M, assignment, AssignmentProblemSolver::optimal);

    // -----------------------------------
    // clean assignment from pairs with large distance
    // -----------------------------------
    for (size_t i = 0; i < assignment.size(); i++)
    {
      if (assignment[i] != -1)
      {
        //std::cout << "assing: " << assignment[i] << " cost: " << cost[i + assignment[i] * N] << std::endl;
        if(cost[i + assignment[i] * N] < config.lane_threshold) current_lane = spline_combinations[assignment[i]];
      }
      //else std::cout << "Assign problem" << std::endl;
    }
  }

  else if(spline_combinations.size() > 0) {
      float cost = spline_combinations[0].calcCost(last_lane);
      if(cost < config.lane_threshold) current_lane = spline_combinations[0];
      //std::cout << "single cost:" << cost << std::endl;
  }
}
