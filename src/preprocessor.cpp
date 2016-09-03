#include <lane_detector/preprocessor.h>
#include <swri_profiler/profiler.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ocl/ocl.hpp>

void Preprocessor::preprocess(cv::Mat& originalImg, cv::Mat& img, LaneDetector::IPMInfo& ipmInfo_, LaneDetector::CameraInfo& cameraInfo_) {
        SWRI_PROFILE("Preprocess");
        this->ipmInfo = ipmInfo;
        list<CvPoint> outPixels;
        list<CvPoint>::iterator outPixelsi;
        CvMat raw_mat = img;
        CvMat* raw_ptr = &raw_mat;
        CvMat* mat_ptr;
        /*CvMat* fx = cvCreateMat(lanesConf.kernelWidth, 1, FLOAT_MAT_TYPE);
        CvMat* fy = cvCreateMat(lanesConf.kernelHeight, 1, FLOAT_MAT_TYPE);
        LaneDetector::mcvGetGaussianKernel(fy, (lanesConf.kernelHeight-1)/2, lanesConf.lineHeight*ipmInfo.yScale);
        LaneDetector::mcvGet2DerivativeGaussianKernel(fx, (lanesConf.kernelWidth-1)/2, lanesConf.lineWidth*ipmInfo.xScale);
        LaneDetector::SHOW_MAT(fx, "Kernel_x:");
        LaneDetector::SHOW_MAT(fy, "Kernel_y:");*/
        //TODO rewrite function
        LaneDetector::mcvLoadImage(&raw_ptr, &mat_ptr);
        getIPM(&mat_ptr, &cameraInfo, &ipmInfo, &lanesConf, &outPixels);
        cameraInfo_ = cameraInfo;
        ipmInfo_ = ipmInfo;
        //do the filtering
             /*CvMat *kernel = cvCreateMat(lanesConf.kernelHeight, lanesConf.kernelWidth, FLOAT_MAT_TYPE);
             cvGEMM(fy, fx, 1, 0, 1, kernel, CV_GEMM_B_T);
             cvFilter2D(mat_ptr, mat_ptr, kernel);
             //subtract the mean of the kernel
             CvScalar mean_kernel = cvAvg(kernel);
             cvSubS(kernel, mean_kernel, kernel);
        //cvFilter2D(mat_ptr, mat_ptr, fx); //inImage outImage
        //cvFilter2D(mat_ptr, mat_ptr, fy);
        cvReleaseMat(&kernel);*/
        if(config.use_gpu) {
          //subtract mean of the image
          img = cv::cvarrToMat(mat_ptr, true);
          cv::Scalar mean = cv::mean(img);
          cv::ocl::oclMat ocl_img(img);
          cv::ocl::subtract(ocl_img, mean, ocl_img);
          cv::ocl::sepFilter2D(ocl_img, ocl_img, CV_32FC1, kernel_x, kernel_y);
          cv::ocl::abs(ocl_img, ocl_img);
          img = ocl_img;
        }
        else {
          //subtract mean of the image
          CvScalar mean = cvAvg(mat_ptr);
          cvSubS(mat_ptr, mean, mat_ptr);
          img = cv::cvarrToMat(mat_ptr, true);
          cv::sepFilter2D(img, img, CV_32FC1, kernel_x, kernel_y);
        }
        cvReleaseMat(&mat_ptr);
        raw_mat = img;
        mat_ptr = &raw_mat;
        if(!config.use_gpu) cvAbsDiffS(mat_ptr, mat_ptr, CvScalar{0.0});
        //zero out points outside the image in IPM view
        for(outPixelsi=outPixels.begin(); outPixelsi!=outPixels.end(); outPixelsi++)
        {
          CV_MAT_ELEM(*mat_ptr, float, (*outPixelsi).y, (*outPixelsi).x) = 0;
        }
        outPixels.clear();
        //LaneDetector::SHOW_IMAGE(mat_ptr, "IMG", 10);
        //compute quantile: .985
        float qtileThreshold = LaneDetector::mcvGetQuantile(mat_ptr, lanesConf.lowerQuantile);
        LaneDetector::mcvThresholdLower(mat_ptr, mat_ptr, qtileThreshold);
        ROS_DEBUG("xScale: %f, yScale: %f, ymaxLim: %f",ipmInfo.xScale, ipmInfo.yScale, ipmInfo.yLimits[1]);
        img = cv::cvarrToMat(mat_ptr, true);
        cvReleaseMat(&mat_ptr);
        //cvReleaseMat(&fx);
        //cvReleaseMat(&fy);

        if(config.draw_roi) {
          CvPoint2D32f vpf;
          vpf = LaneDetector::mcvGetVanishingPoint(&cameraInfo);
          cv::Point vp;
          vp.x = vpf.x;
          vp.y = vpf.y;

          cv::Point points_[1][3];
          //points_[0][0] = cv::Point(vp.x, ipmInfo.ipmTop);
          points_[0][0] = cv::Point(ipmInfo.ipmRight, ipmInfo.ipmTop);
          points_[0][1] = cv::Point(ipmInfo.ipmLeft, ipmInfo.ipmTop);
          points_[0][2] = cv::Point(vp.x, ipmInfo.ipmBottom);
          const cv::Point* pts[1] = {points_[0]};
          int npts[] = {3};
          cv::Mat overlay;
          originalImg.copyTo(overlay);
          cv::fillPoly(overlay, pts, npts, 1, cv::Scalar(255,0,0));
          double opacity = 0.4;
          cv::addWeighted(overlay, opacity, originalImg, 1 - opacity, 0, originalImg);

          cv::circle(originalImg, cv::Point(vp.x, ipmInfo.ipmTop), 6, cv::Scalar(0,255,255), -1);
          cv::circle(originalImg, cv::Point(ipmInfo.ipmRight, ipmInfo.ipmTop), 6, cv::Scalar(0,255,255), -1);
          cv::circle(originalImg, cv::Point(ipmInfo.ipmLeft, ipmInfo.ipmTop), 6, cv::Scalar(0,255,255), -1);
          cv::circle(originalImg, cv::Point(vp.x, ipmInfo.ipmBottom), 6, cv::Scalar(0,255,255), -1);
        }

}
