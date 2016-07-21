#include <lane_detector/fitting.h>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <vector>


float Fitting::calcCost(std::vector<cv::Point2f>& combination) {

  std::array<float, 4> diff;

  if(driving_orientation == lane_detector::on_the_left)  //Driving on the left
  {
    diff[0] = combination[0].x;
    diff[1] = combination[1].x;
  }
  else //Driving on the right
  {
    diff[0] = combination[0].x - (config.ipmWidth - 1);
    diff[1] = combination[1].x - (config.ipmWidth - 1);
  }
  diff[2] = config.width_between_lines * ipmInfo.xScale - std::fabs(combination[0].x - combination[1].x);
  diff[4] = diff[2];

  //std::cout << 820 * ipmInfo.xScale << std::endl;
  //std::cout << "x1: "<< combination[0].x << " x2: " << combination[1].x << " d1: " << diff[0] << " d2: " << diff[1] << " d3: " << diff[2] << std::endl;

  float dist = 0;
  for (float i = 0; i < diff.size(); ++i)
  {
    dist += diff[i] * diff[i];
  }
  return sqrtf(dist);
}

lane_detector::Lane Fitting::fitting(cv::Mat& original, cv::Mat& preprocessed, LaneDetector::IPMInfo& ipmInfo, std::vector<LaneDetector::Box>& ipmBoxes)
{

        this->ipmInfo = ipmInfo;

        cv::Scalar Colors[] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255), cv::Scalar(255, 127, 255), cv::Scalar(127, 0, 255), cv::Scalar(127, 0, 127) };
        FittingApproach fitSpline;
        std::vector<cv::Point> splinePoints;
        std::vector< std::vector<cv::Point> > splines;
        std::vector<cv::Point2f> centroids;
        std::vector<cv::Rect> rects;
        lane_detector::Lane current_lane;
        lane_detector::utils::boxes2Rects(ipmBoxes, rects);
        lane_detector::utils::getRectsCentroids(rects, centroids);
        tracker.Update(centroids, rects, CTracker::RectsDist);
        rects = tracker.getLastRects();
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

        cv::Point closest(0,0);
        cv::Point second_closest(0,0);
        int closest_idx = 0;
        int second_closest_idx = 0;
        //std::cout << "centroids: " << centroids.size() << std::endl;
        if(centroids.size() >= 2) {
          std::vector<cv::Point2f> current_lane;
          findCurrentLane(centroids, current_lane);
          //std::cout << current_lane.size() << std::endl;
          if(current_lane.size() == 2) {
            std::vector<cv::Point2f>::iterator it1 = std::find(centroids.begin(), centroids.end(), current_lane[0]);
            std::vector<cv::Point2f>::iterator it2 = std::find(centroids.begin(), centroids.end(), current_lane[1]);
            if(it1 != centroids.end()) closest_idx = it1 - centroids.begin();
            if(it2 != centroids.end()) second_closest_idx = it2 - centroids.begin();
            //std::cout << "centroids: " << centroids.size() << std::endl;
            closest = current_lane[0];
            second_closest = current_lane[1];
            }
          }
        //std::cout << "splines size: " << splines.size() << std::endl;
        //if(splines.size() == 3) {
          //std::cout << "1: " << splines[0].size() << " 2: " << splines[1].size() << " 3: " << splines[2].size() << std::endl;
        //}

        if(closest.x != 0 && closest.y != 0 && second_closest.x != 0 && second_closest.y != 0) {
          cv::circle(original, closest, 5, cv::Scalar(0,255,0), 2);
          cv::circle(original, second_closest, 5, cv::Scalar(0,255,0), 2);
        }

        for(int i = 0; i < tracker.tracks.size(); i++) {
          if (tracker.tracks[i]->trace.size() >= 1)
          {
          cv::circle(original, tracker.tracks[i]->trace.back(), 2, Colors[tracker.tracks[i]->track_id % 9], 2);
          }
        }

        std::vector<cv::Point> longest_spline;
        std::vector<cv::Point> second_longest_spline;
        std::vector<cv::Point> left_spline;
        std::vector<cv::Point> right_spline;
        std::vector<cv::Point> guide_spline;
        uint32_t longest_spline_idx = 0;
        uint32_t second_longest_spline_idx = 0;

        //std::cout << "splines: " << splines.size() << std::endl;
        //if(splines.size() > 1) std::cout << "closest: " << splines[closest_idx].size() << " second: " << splines[second_closest_idx].size() << std::endl;

        if(splines.size() > 1 && splines[closest_idx].size() > 3 && splines[second_closest_idx].size() > 3) {
          if(splines[closest_idx][0].y < splines[second_closest_idx][0].y)  {
            longest_spline = splines[closest_idx];
            second_longest_spline = splines[second_closest_idx];
            longest_spline_idx = closest_idx;
            second_longest_spline_idx = second_closest_idx;
          }
          else {
            longest_spline = splines[second_closest_idx];
            second_longest_spline = splines[closest_idx];
            longest_spline_idx = second_closest_idx;
            second_longest_spline_idx = closest_idx;
          }

          int middle = 0;
          for(int i = 0; i < longest_spline.size(); i++) {
            if(longest_spline[i].y >= second_longest_spline[0].y) {
              middle = std::abs(cvRound((longest_spline[i].x - second_longest_spline[0].x)/2));

              //std::cout << "Middle: " << middle << std::endl;
              break;
            }
          }

          uint32_t height_spline1 = longest_spline.back().y - longest_spline.front().y;
          uint32_t height_spline2 = second_longest_spline.back().y - second_longest_spline.front().y;
          //std::cout << "height spline1: " << height_spline1 << "height spline2: " << height_spline2 << std::endl;
          if(height_spline1 < height_spline2) {
              std::vector<cv::Point> aux = longest_spline;
              uint32_t idx = longest_spline_idx;
              longest_spline = second_longest_spline;
              second_longest_spline = aux;
              longest_spline_idx = second_longest_spline_idx;
              second_longest_spline_idx = idx;
          }

          guide_spline = longest_spline;

          if(centroids[longest_spline_idx].x > centroids[second_longest_spline_idx].x) {
              right_spline = splines[longest_spline_idx];
              left_spline = splines[second_longest_spline_idx];

              for(int i = 0; i < guide_spline.size(); i++) {
                guide_spline[i].x -= middle;
              }
          }

          else {
            right_spline = splines[second_longest_spline_idx];
            left_spline = splines[longest_spline_idx];

            for(int i = 0; i < guide_spline.size(); i++) {
              guide_spline[i].x += middle;
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

           current_lane.guide_line = guide_spline_ros;
           current_lane.right_line = right_spline_ros;
           current_lane.left_line = left_spline_ros;
        }
      //cv::line(original, cv::Point(car_position.x, 0), cv::Point(car_position.x, lanesConf.ipmHeight-1), cv::Scalar(0, 255, 239), 1);
      return current_lane;
}

// -------------------------------------------------
// Solving assignment problem for the lane
// ------------------------------------------------
void Fitting::findCurrentLane(const std::vector<cv::Point2f>& centroids, std::vector<cv::Point2f>& current_lane) {

  std::vector<std::vector<cv::Point2f>> combinations;

  assignments_t assignment;

  lane_detector::utils::combineVectorInPairs(centroids, combinations);

  //One lanes has to be assigned
  size_t N = 1;
  size_t M = combinations.size();
  distMatrix_t cost(N * M);

  //std::cout << "Combis: " << combinations.size() << " combi: " << combinations[0].size() << std::endl;

  if(combinations.size() > 1) {
      for (size_t i = 0; i < N; i++)
    {
      for (size_t j = 0; j <  combinations.size(); j++)
      {
        cost[i + j * N] = calcCost(combinations[j]);
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
        current_lane = combinations[assignment[i]];
      }
      //else std::cout << "Assign problem" << std::endl;
    }
  }

  else if(combinations.size() > 0) current_lane = combinations[0];
}
