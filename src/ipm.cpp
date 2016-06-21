/*
 * ipm.cpp
 *
 *      Author:
 *         Nicolas Acero
 */

#include <lane_detector/ipm.h>

using namespace cv;

 void IPM::applyHomography(Mat& inputImg, Mat& dstImg) {

     Size taille = inputImg.size();
     double w = (double)taille.width, h = (double)taille.height;

   // Projection 2D -> 3D matrix
      Mat A1 = (Mat_<double>(4,3) <<
          1, 0, -w/2,
          0, 1, -h/2,
          0, 0,    0,
          0, 0,    1);

      // Rotation matrices around the X,Y,Z axis
        Mat RX = (Mat_<double>(4, 4) <<
            1,          0,           0, 0,
            0, cos(alpha), -sin(alpha), 0,
            0, sin(alpha),  cos(alpha), 0,
            0,          0,           0, 1);

        Mat RZ = (Mat_<double>(4, 4) <<
            cos(gamma), -sin(gamma), 0, 0,
            sin(gamma),  cos(gamma), 0, 0,
            0,          0,           1, 0,
            0,          0,           0, 1);

        // Composed rotation matrix with (RX,RY,RZ)
        Mat R = RX * RZ;


      // Translation matrix on the Z axis change dist will change the height
      Mat T = (Mat_<double>(4, 4) <<
      1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, z_distance,
       0, 0, 0, 1);       // Camera Intrisecs matrix 3D -> 2D

      Mat A2 = (Mat_<double>(3,4) <<
          focal_lenght, 0, w/2, 0,
          0, focal_lenght, h/2, 0,
          0, 0,   1, 0);

      // Final and overall transformation matrix
      Mat transform = A2 * (T * (R * A1));

      // Apply matrix transformation
      warpPerspective(inputImg, dstImg, transform, taille, INTER_CUBIC | WARP_INVERSE_MAP);
      waitKey(30);

 }
