#include <lane_detector/fittingApproach.h>
#include <iostream>
#include <swri_profiler/profiler.h>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <lane_detector/utils.h>


float lastX_left = 0;
float lastX_right = 0;
cv::Point lastVp;

void FittingApproach::setPixel(int x, int y, cv::Mat& img)
{
        uchar* pixel = img.ptr<uchar>(y); // pointer to first pixel in row
        int value = pixel[x];
        value = ++value;
        pixel[x] = value;
}

void FittingApproach::pixelVoting(lines::iterator& it, cv::Mat& img)

{
        SWRI_PROFILE("Bresenham");
        int x0 = it->at(0).x;
        int x1 = it->at(1).x;
        int y0 = it->at(0).y;
        int y1 = it->at(1).y;
        int dx =  std::abs(x1-x0), sx = x0<x1 ? 1 : -1;
        int dy = -std::abs(y1-y0), sy = y0<y1 ? 1 : -1;
        int err = dx+dy, e2; /* error value e_xy */

        int x0_copy = x0;
        int y0_copy = y0;
        bool flag = false;

        while(1)
        {
                if(x0 >= 0 && x0 < img.cols && y0 >= 0 && y0 < img.rows)
                {
                        if(!flag)
                        {
                                it->at(0).x = x0;
                                it->at(0).y = y0;
                                flag = true;
                        }
                        setPixel(x0, y0, img);
                }
                if ((x0==x1 && y0==y1)) break;
                else if (x0_copy >= img.cols && x0 <=0) break;
                else if (x0_copy <= 0 && x0 >= img.cols) break;
                else if (y0_copy >= img.rows && y0 <=0) break;
                else if (y0_copy <= 0 && y0 >= img.rows) break;
                e2 = 2*err;
                if (e2 > dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
                if (e2 < dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
        }
        it->at(1).x = x0;
        it->at(1).y = y0;
}

std::vector<cv::Point> FittingApproach::fitting(cv::Mat& original, cv::Mat& preprocessed, linesPair& features)
{

        SWRI_PROFILE("Fitting");

        if(original.channels() == 1)
        {
                cv::cvtColor(original, original, CV_GRAY2BGR);
        }
        currentFrame = original;
        this->preprocessed = preprocessed;
        bestLinesLeft = features.first;
        bestLinesRight = features.second;
        ROS_DEBUG("Found %lu left lanes and %lu posible right lanes", bestLinesLeft.size(), bestLinesRight.size());

        if(bestLinesLeft.size() > 0 && bestLinesRight.size() > 0)
        {
                vanishingPoint = findVanishingPoint();
                filterLines(bestLinesLeft,vanishingPoint);
                filterLines(bestLinesRight,vanishingPoint);

                line leftLane = clusterLines(bestLinesLeft, currentFrame.cols, vanishingPoint, false);
                line rightLane = clusterLines(bestLinesRight, currentFrame.cols, vanishingPoint, true);
                bestLinesLeft.clear();
                bestLinesRight.clear();
                float currentX_left = utils::extrapolateLineX(currentFrame.rows, leftLane);
                float currentX_right = utils::extrapolateLineX(currentFrame.rows, rightLane);

                //TODO
                if((lastX_left == 0 && lastX_right == 0 && currentX_left > 0 && currentX_right > 0 &&
                    currentX_left < currentFrame.cols/2 &&
                    currentX_right > currentFrame.cols/2) || ((lastX_left != 0 && lastX_right != 0) &&
                                                              (std::fabs(lastX_left - currentX_left) < config.max_lane_diff) &&
                                                              (std::fabs(lastX_right - currentX_right) < config.max_lane_diff) &&
                                                              currentX_left > 0 && currentX_right > 0 && currentX_left < currentFrame.cols/2 &&
                                                              currentX_right > currentFrame.cols/2))
                {
                        //ROS_WARN("currentX_left:%f, currentX_right:%f", currentX_left, currentX_right);
                        lastLeftLine = leftLane;
                        lastX_left = currentX_left;
                        lastX_right = currentX_right;
                        lastRightLine = rightLane;
                        lastVp = vanishingPoint;
                        lastUpdate = ros::Time::now();
                }
        }
        else ROS_WARN("No lanes detedted, using last lanes");
        /*cv::Point points_[1][4];
           points_[0][0] = lastRightLine[1];
           points_[0][1] = lastLeftLine[1];
           int x1 = (currentFrame.cols/2)-currentFrame.cols*0.12;
           int x2 = (currentFrame.cols/2)+currentFrame.cols*0.12;
           int y1 = cvRound((lastLeftLine[0].y*(lastLeftLine[1].x-x1) + lastLeftLine[1].y*(x1 - lastLeftLine[0].x))/(lastLeftLine[1].x - lastLeftLine[0].x));
           int y2 = cvRound((lastRightLine[0].y*(lastRightLine[1].x-x2) + lastRightLine[1].y*(x2 - lastRightLine[0].x))/(lastRightLine[1].x - lastRightLine[0].x));
           points_[0][2] = cv::Point(x1, y1);
           points_[0][3] = cv::Point(x2, y2);
           const cv::Point* pts[1] = {points_[0]};
           int npts[] = {4};
           cv::Mat overlay;
           currentFrame.copyTo(overlay);
           cv::fillPoly(overlay, pts, npts, 1, cv::Scalar(0,255,0));
           double opacity = 0.4;
           cv::addWeighted(overlay, opacity, currentFrame, 1 - opacity, 0, currentFrame);*/


        //TODO
        if((ros::Time::now() - lastUpdate).sec >= timeOut.sec)
        {
                lastLeftLine.clear();
                lastRightLine.clear();
                bestLinesLeft.clear();
                bestLinesRight.clear();
                lastVp = cv::Point(0,0);
                lastX_left = 0;
                lastX_right = 0;
                lastUpdate = ros::Time::now();
                ROS_WARN("Timeout for detecting lanes...Reseting old data and trying again");
        }

        if(lastLeftLine.size() > 0 && lastRightLine.size() > 0)
        {
                //ROS_INFO("Left lane: x0:%i, y0:%i ; x1:%i, y1:%i", lastLeftLine[0].x, lastLeftLine[0].y, lastLeftLine[1].x, lastLeftLine[1].y);
                //ROS_INFO("Right lane: x0:%i, y0:%i ; x1:%i, y1:%i", lastRightLine[0].x, lastRightLine[0].y, lastRightLine[1].x, lastRightLine[1].y);
                cv::line(currentFrame, lastLeftLine[0], lastLeftLine[1], cv::Scalar(50,240,12), 3, CV_AA);
                cv::line(currentFrame, lastRightLine[0], lastRightLine[1], cv::Scalar(239,203,24), 3, CV_AA);
                cv::circle(currentFrame, lastVp, 4, cv::Scalar(0,0,255), 5);
                cv::line(currentFrame, cv::Point(currentFrame.cols/2, 0), cv::Point(currentFrame.cols/2, currentFrame.rows), cv::Scalar(0, 255, 239), 1);
        }

        std::vector<cv::Point> points;
        points.push_back(lastVp);
        return points;
}

cv::Point FittingApproach::findVanishingPoint()
{
        SWRI_PROFILE("findVanishingPoint");
        cv::Mat blueLines(cv::Size(currentFrame.cols, currentFrame.rows), CV_8UC1, cv::Scalar(0));
        cv::Mat greenLines(cv::Size(currentFrame.cols, currentFrame.rows), CV_8UC1, cv::Scalar(0));
        cv::Mat redChannel(cv::Size(currentFrame.cols, currentFrame.rows), CV_8UC1, cv::Scalar(0));
        cv::Point vp(0,0);

        for(lines::iterator it = bestLinesRight.begin(); it != bestLinesRight.end(); it++)
        {
                pixelVoting(it, greenLines);
                if(it->at(0).x < 0 || it->at(0).y < 0 || it->at(1).x < 0 || it->at(1).y < 0)
                        ROS_ERROR("Negativ");
        }
        for(lines::iterator it = bestLinesLeft.begin(); it != bestLinesLeft.end(); it++)
        {
                pixelVoting(it, blueLines);
                if(it->at(0).x < 0 || it->at(0).y < 0 || it->at(1).x < 0 || it->at(1).y < 0)
                        ROS_ERROR("Negativ");
        }

        std::vector<cv::Mat> input;
        input.push_back(blueLines);
        input.push_back(greenLines);
        input.push_back(redChannel);
        cv::Mat output;
        cv::merge(input, output);
        int max_sum_1 = 0;
        int max_sum_2 = 0;
        std::list<cv::Point> vanishingPoints;

        for(int i=0; i<output.rows; i++)
        {
                cv::Vec3b* pixel = output.ptr<cv::Vec3b>(i);
                for(int j=0; j<output.cols; j++)
                {
                        int b = pixel[j][0];
                        int g = pixel[j][1];
                        int sum = b+g;
                        if(b > 0 && g > 0 && sum > max_sum_1)
                        {
                                max_sum_1 = sum;
                        }
                }
        }

        for(int i=0; i<output.rows; i++)
        {
                cv::Vec3b* pixel = output.ptr<cv::Vec3b>(i);
                for(int j=0; j<output.cols; j++)
                {
                        int b = pixel[j][0];
                        int g = pixel[j][1];
                        int sum = b+g;
                        if(b > 0 && g > 0 && sum == max_sum_1) {
                                vanishingPoints.push_back(cv::Point(j,i));
                        }
                }
        }
        ROS_DEBUG("VPS Found:%lu", vanishingPoints.size());

        int vps_size = vanishingPoints.size();
        if(vps_size > 0)
        {
                std::list<cv::Point>::iterator it;
                int sum_y = 0;
                int sum_x = 0;
                int i = 0;
                for(it = vanishingPoints.begin(); it != vanishingPoints.end(); it++)
                {
                        sum_y += (*it).y;
                        sum_x += (*it).x;
                        i++;
                }
                int x = sum_x/i;
                int y = sum_y/i;
                vp = cv::Point(x,y);
        }

        /*cv::circle(output, vp, 2, cv::Scalar(0,255,0), 5);
           cv::imshow("vanishingPoint", output);
           cv::waitKey(1);*/
        return vp;
}

line FittingApproach::clusterLines(std::vector<line>& lines, int imageWidth, cv::Point vanishingPoint, bool is_right)
{
        SWRI_PROFILE("clusterLines");
        typedef std::pair<line, cv::Vec3f> cluster;
        std::vector<cluster> data;
        std::vector<line> selectedLines;

        for(line l : lines)
        {

                if(l[0].y < l[1].y) l[0] = vanishingPoint;
                else
                {
                        l[1] = l[0];
                        l[0] = vanishingPoint;
                }
                float slopeAngle = std::fabs(180*is_right - utils::calcSlopeAngle(l[0], l[1]));
                float slope = tan(CV_PI*(std::fabs(180*is_right - slopeAngle))/180);
                cv::Point aux;
                if(is_right) aux = cv::Point(currentFrame.cols, utils::extrapolateLineY(currentFrame.cols, l));
                else aux = cv::Point(0, currentFrame.rows - utils::extrapolateLineY(0, l));
                float b = aux.y - slope*aux.x;
                data.push_back(cluster(l, cv::Vec3f(slopeAngle, slope, b)));
        }
        int i = 1;
        while(data.size() > 1)
        {
                float currentSlope = data[0].second[1];
                float currentSlopeAngle = data[0].second[0];
                float current_b = data[0].second[2];
                if(std::fabs(data[i].second[0] - currentSlopeAngle) < config.max_cluster_angle)
                {

                        float avg_slope = (currentSlope + data[i].second[1])/2;
                        float avg_b = (current_b + data[i].second[2])/2;
                        float x;
                        float y;
                        if(is_right)
                        {
                                x = currentFrame.cols;
                                y = avg_slope*x + avg_b; //For the right lines
                        }
                        else
                        {
                                x = 0;
                                y = currentFrame.rows - avg_b; //For the left lines;
                        }
                        data[0].first[1].x = x;
                        data[0].first[1].y = y;

                        data.erase(data.begin()+i);
                        if(data.size() == 1)
                        {
                                selectedLines.push_back(data[0].first);
                        }
                }
                else if((i+1) < data.size())
                {
                        i++;
                }
                else
                {
                        selectedLines.push_back(data[0].first);
                        data.erase(data.begin());
                        i = 1;
                }
        }
        if(selectedLines.size() > 1)
        {
                line closestLine;
                float minX = 99999999;
                for(int i = 0; i < selectedLines.size(); i++)
                {
                        line l = selectedLines[i];
                        float x = utils::extrapolateLineX(currentFrame.rows, l);
                        float distFromCamera = std::fabs(x - imageWidth/2);
                        if(distFromCamera < minX)
                        {
                                minX = distFromCamera;
                                closestLine = l;
                        }
                }
                return closestLine;
        }
        else if(selectedLines.size() == 1)
        {
                return selectedLines[0];
        }
        else
        {
                ROS_ERROR("Error");
                return data[0].first;
        }
}

void FittingApproach::filterLines(std::vector<line>& lines, cv::Point vanishingPoint)
{
        SWRI_PROFILE("filterLines_End");
        for(int i = 0; i < lines.size(); i++)
        {
                line l = lines[i];
                int x = cvRound(utils::extrapolateLineX(vanishingPoint.y, l));
                int distToVp = std::abs(vanishingPoint.x - x);
                if(lines.size() > 1 && distToVp > config.max_distance_to_vanishing_point)
                {
                        //cv::line(currentFrame, l[0], l[1], cv::Scalar(0,255,255), 1, CV_AA);
                        lines.erase(lines.begin()+i);
                        i--;
                }
                else
                {
                        cv::line(currentFrame, l[0], l[1], cv::Scalar(0,255,255), 1, CV_AA);
                }
        }
}
