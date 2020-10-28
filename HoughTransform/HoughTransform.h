#ifndef HOUGHTRANSFORM_H
#define HOUGHTRANSFORM_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

class HoughTransform
{
  public:
    int mainfunc()
    {
      //Read image as gray scale
      cv::Mat  img = cv::imread("circle.jpg", 0);
      //Blur to reduce noise
      cv::Mat img_blurred;
      cv::medianBlur(img, img_blurred, 5);
      //Vector containing the circles
      std::vector<cv::Vec3f> circles;
      //Applying hough transform to the image
      cv::HoughCircles(img_blurred, circles, cv::HOUGH_GRADIENT, 1, img.rows/64, 200, 10, 5, 30);
      // Draw detected circles
      for(size_t i=0; i<circles.size(); i++) {
          cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          circle(img, center, radius, cv::Scalar(255, 255, 255), 2, 8, 0);
       }

    }

};



#endif // HOUGHTRANSFORM_H
