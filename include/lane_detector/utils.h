/*
 * utils.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef UTILS_H_
#define UTILS_H_

namespace utils {
inline float extrapolateLineX(float y, line& l) {
        float x = (y*(l[1].x-l[0].x) - l[1].x*l[0].y + l[0].x*l[1].y)/(l[1].y - l[0].y);
        return x;
}
inline float extrapolateLineY(float x, line& l) {
        float y = (l[0].y*(l[1].x-x) + l[1].y*(x-l[0].x))/(l[1].x - l[0].x);
        return y;
}
inline float calcSlopeAngle(cv::Point& pt1, cv::Point& pt2) {
        float dY = std::abs(pt2.y - pt1.y);
        float dX = std::abs(pt2.x - pt1.x);
        float slope = dY/dX;
        return 180*std::atan(slope)/CV_PI;
}
};

#endif /* UTILS_H_ */
