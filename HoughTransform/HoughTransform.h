#ifndef HOUGHTRANSFORM_H
#define HOUGHTRANSFORM_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

class HoughTransform
{
  public:
    HoughTransform(){};

    static cv::Mat circleDetect(cv::Mat img)
    {
        // //Read image as gray scale
        //cv::Mat  img = cv::imread("marble.png", cv::IMREAD_COLOR);
        //cv::imshow("1", img);

        //Convert to gray and blur
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        medianBlur(gray, gray, 5);

        //cv::imshow("gray", gray);

        //Vector containing the circles
        std::vector<cv::Vec3f> circles;

        //Applying hough transform to the image
        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                      gray.rows,  // change this value to detect circles with different distances to each other
                      100, 20, 0, 0 // change the last two parameters
                 // (min_radius & max_radius) to detect larger circles, 0 for unknown radius
         );

        // Draw detected circles
        for( size_t i = 0; i < circles.size(); i++ )
        {
            cv::Vec3i c = circles[i];
            cv::Point center = cv::Point(c[0], c[1]);
            // circle center
            circle( img, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
            // circle outline
            int radius = c[2];
            float relation_constant = 4.8*30;
            double distance_to_marble = relation_constant/radius;
            std::cout << "Distance: " << distance_to_marble << std::endl;
            circle( img, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
        }

        //cv::imshow("final", img);
        //cv::waitKey();

        return img;
    }

};



#endif // HOUGHTRANSFORM_H
