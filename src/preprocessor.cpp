#include <lane_detector/preprocessor.h>
#include <swri_profiler/profiler.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ocl/ocl.hpp>

void Preprocessor::preprocess(cv::Mat& originalImg, cv::Mat& img, LaneDetector::IPMInfo& ipmInfo_, LaneDetector::CameraInfo& cameraInfo_) {
        SWRI_PROFILE("Preprocess");
        CvMat raw_mat = img;
        CvMat* raw_ptr = &raw_mat;
        CvMat* mat_ptr;

        LaneDetector::mcvLoadImage(&raw_ptr, &mat_ptr);

        //get mean of the input image
        CvScalar means = cvAvg(mat_ptr);
        double mean_image = means.val[0];

        if(initialize) {
          initialize = false;
          getIpmMap(mat_ptr, &cameraInfo, &ipmInfo, vanishing_point, &lanesConf, &ipm_outPoints, &ipmGrid, &ipm_out_of_area, uvGrid);
          cameraInfo_ = cameraInfo;
        }

        ipmInfo_ = ipmInfo;

        //Get color value for the ipm pixels
        CvMat* ipm;
        ipm = cvCreateMat(config.ipmHeight, config.ipmWidth, mat_ptr->type);
        getPixelValues(mat_ptr, ipm);
        cvReleaseMat(&mat_ptr);
        mat_ptr = cvCloneMat(ipm);
        cvReleaseMat(&ipm);
        for(auto outPixelsi = ipm_outPoints.begin(); outPixelsi != ipm_outPoints.end(); outPixelsi++)
        {
          CV_MAT_ELEM(*mat_ptr, float, (*outPixelsi).y, (*outPixelsi).x) = (float)mean_image;
        }

        if(config.use_custom_kernels) {
          CvMat* fx = cvCreateMat(lanesConf.kernelWidth, 1, FLOAT_MAT_TYPE);
          CvMat* fy = cvCreateMat(lanesConf.kernelHeight, 1, FLOAT_MAT_TYPE);
          LaneDetector::mcvGetGaussianKernel(fy, (lanesConf.kernelHeight-1)/2, lanesConf.lineHeight*ipmInfo.yScale);
          LaneDetector::mcvGet2DerivativeGaussianKernel(fx, (lanesConf.kernelWidth-1)/2, lanesConf.lineWidth*ipmInfo.xScale);
          //LaneDetector::SHOW_MAT(fx, "Kernel_x:");
          //LaneDetector::SHOW_MAT(fy, "Kernel_y:");
          kernel_x = cv::cvarrToMat(fx, true);
          kernel_y = cv::cvarrToMat(fy, true);
          cvReleaseMat(&fx);
          cvReleaseMat(&fy);
        }
        //do the filtering
        if(config.use_gpu) {
          //subtract mean of the image
          img = cv::cvarrToMat(mat_ptr, true);
          cv::ocl::oclMat ocl_img(img);
          cv::ocl::subtract(ocl_img, cv::Scalar(mean_image), ocl_img);
          cv::ocl::sepFilter2D(ocl_img, ocl_img, CV_32FC1, kernel_x, kernel_y);
          //cv::ocl::abs(ocl_img, ocl_img);
          img = ocl_img;
        }
        else {
          //subtract mean of the image
          CvScalar mean;
          mean.val[0] = mean_image;
          cvSubS(mat_ptr, mean, mat_ptr);
          img = cv::cvarrToMat(mat_ptr, true);
          cv::sepFilter2D(img, img, CV_32FC1, kernel_x, kernel_y);
        }
        cvReleaseMat(&mat_ptr);
        raw_mat = img;
        mat_ptr = &raw_mat;
        //if(!config.use_gpu) cvAbsDiffS(mat_ptr, mat_ptr, CvScalar{0.0});

        //zero out points outside the image in IPM view
        for(auto outPixelsi = ipm_out_of_area.begin(); outPixelsi != ipm_out_of_area.end(); outPixelsi++)
        {
          CV_MAT_ELEM(*mat_ptr, float, (*outPixelsi).y, (*outPixelsi).x) = 0.0;
        }
        //LaneDetector::SHOW_IMAGE(mat_ptr, "IMG", 10);
        //compute quantile: .985
        float qtileThreshold = LaneDetector::mcvGetQuantile(mat_ptr, lanesConf.lowerQuantile);
        LaneDetector::mcvThresholdLower(mat_ptr, mat_ptr, qtileThreshold);
        ROS_DEBUG("xScale: %f, yScale: %f, ymaxLim: %f",ipmInfo.xScale, ipmInfo.yScale, ipmInfo.yLimits[1]);
        img = cv::cvarrToMat(mat_ptr, true);
        //cvReleaseMat(&mat_ptr);

        if(config.draw_roi) {

          cv::Point points_[1][4];
          points_[0][0] = cv::Point(ipmInfo.ipmLeft, ipmInfo.ipmTop);
          points_[0][1] = cv::Point(ipmInfo.ipmRight, ipmInfo.ipmTop);
          points_[0][2] = cv::Point(ipmInfo.ipmRight, ipmInfo.ipmBottom);
          points_[0][3] = cv::Point(ipmInfo.ipmLeft, ipmInfo.ipmBottom);
          const cv::Point* pts[1] = {points_[0]};
          int npts[] = {4};
          cv::Mat overlay;
          originalImg.copyTo(overlay);
          cv::fillPoly(overlay, pts, npts, 1, cv::Scalar(255,0,0));
          double opacity = 0.4;
          cv::addWeighted(overlay, opacity, originalImg, 1 - opacity, 0, originalImg);
          cv::circle(originalImg, cv::Point(ipmInfo.ipmLeft, ipmInfo.ipmTop), 6, cv::Scalar(0,255,255), -1);
          cv::circle(originalImg, cv::Point(ipmInfo.ipmRight, ipmInfo.ipmTop), 6, cv::Scalar(0,255,255), -1);
          cv::circle(originalImg, cv::Point(ipmInfo.ipmRight, ipmInfo.ipmBottom), 6, cv::Scalar(0,255,255), -1);
          cv::circle(originalImg, cv::Point(ipmInfo.ipmLeft, ipmInfo.ipmBottom), 6, cv::Scalar(0,255,255), -1);
        }
}

void Preprocessor::getPixelValues(CvMat* inImage, CvMat* outImage) {

  for (auto iterator = ipmGrid.begin(); iterator != ipmGrid.end(); iterator++)

    {
          float ui = CV_MAT_ELEM(*uvGrid, float, 0, (*iterator).y*config.ipmWidth+(*iterator).x);
          float vi = CV_MAT_ELEM(*uvGrid, float, 1, (*iterator).y*config.ipmWidth+(*iterator).x);

          if (ui<ipmInfo.ipmLeft || ui>ipmInfo.ipmRight ||
                vi<ipmInfo.ipmTop || vi>ipmInfo.ipmBottom)
          {
          }
          /*not out of bounds, then get nearest neighbor*/
          else
          {
              /*Bilinear interpolation*/
              if (ipmInfo.ipmInterpolation == 0)
              {
                  int x1 = int(ui), x2 = int(ui+1);
                  int y1 = int(vi), y2 = int(vi+1);
                  float x = ui - x1, y = vi - y1;
                  float val = CV_MAT_ELEM(*inImage, float, y1, x1) * (1-x) * (1-y) +
                      CV_MAT_ELEM(*inImage, float, y1, x2) * x * (1-y) +
                      CV_MAT_ELEM(*inImage, float, y2, x1) * (1-x) * y +
                      CV_MAT_ELEM(*inImage, float, y2, x2) * x * y;
                  CV_MAT_ELEM(*outImage, float, (*iterator).y, (*iterator).x) =  (float)val;
              }
              /*nearest-neighbor interpolation*/
              else
                  CV_MAT_ELEM(*outImage, float, (*iterator).y, (*iterator).x) =
                      CV_MAT_ELEM(*inImage, float, int(vi+.5), int(ui+.5));
          }
      }
}
