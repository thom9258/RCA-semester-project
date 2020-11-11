#pragma once
#include <iostream>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define GREYSCALE 0
#define DEBUG true
#define NODEBUG false
#define GREEN                                                                  \
  { 0, 255, 0 }
#define BLUE                                                                   \
  { 255, 0, 0 }
#define RED                                                                    \
  { 0, 0, 255 }
#define WHITE                                                                  \
  { 255, 255, 255 }
#define BLACK                                                                  \
  { 0, 0, 0 }

const cv::Vec3b green_pixel = GREEN;
const cv::Vec3b blue_pixel = BLUE;
const cv::Vec3b red_pixel = RED;
const cv::Vec3b white_pixel = WHITE;
const cv::Vec3b black_pixel = BLACK;
/*
  enum LineTypes {
    FILLED  = -1,
    LINE_4  = 4, //!< 4-connected line
    LINE_8  = 8, //!< 8-connected line
    LINE_AA = 16 //!< antialiased line
};
*/
/*******************************************************************************
 * BASIC
 *******************************************************************************/

void cv_show_image(cv::Mat _input_image, std::string _input_name,
                   float _scale_size = 1) {
  if (_input_image.empty()) {
    std::cerr << "could not load image: " << _input_name << std::endl;
    return;
  }
  cv::resize(_input_image, _input_image, cv::Size(), _scale_size, _scale_size);
  cv::namedWindow(_input_name);
  cv::imshow(_input_name, _input_image);
}

void cv_print_point(Point _input_point, string _point_name = "") {
  if (_point_name != "") {
    cout << _point_name << ": ";
  }
  cout << '[' << _input_point.x << ',' << _input_point.y << ']' << endl;
}

void cv_print_image_data(Mat _input_image, string _image_name = "") {
  if (_image_name != "") {
    cout << _image_name << " ";
  }
  cout << "data: " << _input_image.size << endl;
}

cv::Vec3b cv_get_pixel_bgr(cv::Mat _input_image, int _i, int _j)
//  returns in the format cv::Vec3b [a, b, c]
//  where white = [255, 255, 255] and black = [0, 0, 0]
{
  return _input_image.at<cv::Vec3b>(_i, _j);
}

cv::Vec3b cv_get_pixel_bgr(cv::Mat _input_image, Point _point)
//  returns in the format cv::Vec3b [a, b, c]
//  where white = [255, 255, 255] and black = [0, 0, 0]
{
  return _input_image.at<cv::Vec3b>(_point);
}

cv::Mat cv_color_pixel(cv::Mat _workspace, cv::Point _pixel_location,
                       const Scalar &_color, bool _debug = NODEBUG) {
  cv::line(_workspace, _pixel_location, _pixel_location, _color);
  if (_debug) {
    cout << "pixel " << _pixel_location << " = " << _color << endl;
  }
  return _workspace;
}

/*******************************************************************************
 * ROBOTS IN CONTEXT
 *******************************************************************************/

std::vector<cv::Point> cv_get_pixel_path(cv::LineIterator _input_iterator,
                                         bool _debug = NODEBUG)
// returns the coordinates for every pixel in a lineiterator as points
{
  std::vector<cv::Point> pixel_path;
  for (int i = 0; i < _input_iterator.count; i++, ++_input_iterator) {
    pixel_path.push_back(_input_iterator.pos());
    if (_debug) {
      cout << "px " << i << ": " << _input_iterator.pos() << endl;
    }
  }
  return pixel_path;
}
std::vector<cv::Vec3b> cv_get_line_colors(cv::LineIterator _input_iterator,
                                          string _debug = "nan")
// returns the colors for every pixel in a lineiterator
{
  std::vector<cv::Vec3b> line_colors;
  for (int i = 0; i < _input_iterator.count; i++, ++_input_iterator) {
    line_colors.push_back(cv::Vec3b(*_input_iterator));
    if (_debug == "debug") {
      cout << "px " << i << ": " << line_colors.back() << endl;
    }
  }
  return line_colors;
}

cv::Mat cv_find_valid_bird_path(cv::Mat _workspace, cv::Point _start,
                                cv::Point _goal, bool _debug = NODEBUG)
// finds the valid bird path towards a goal.
// https://docs.opencv.org/master/dc/dd2/classcv_1_1LineIterator.html
{
  cv::LineIterator bird_path(_workspace, _start, _goal, 8);

  // create vectors from the line iterator
  //  std::vector<cv::Point> pixel_path = cv_get_pixel_path(bird_path, _debug);
  //  std::vector<cv::Vec3b> path_colors =
  //      cv_get_line_colors(_workspace, bird_path, _debug);
  // Debug
  if (_debug) {
    cout << "amount of pixels in line: " << bird_path.count << endl;
  }
  // iterate through the vectors and find the last valid pixel
  // (the last pixel before the first black pixel)
  for (int i = 0; i < bird_path.count; i++, ++bird_path) {
    if (cv::Vec3b(*bird_path) != black_pixel) {
      cv_color_pixel(_workspace, bird_path.pos(), blue_pixel);
      if (_debug) {
        cout << i << " " << bird_path.pos() << endl
             << cv_get_pixel_bgr(_workspace, bird_path.pos());
      }
    }
  }
  return _workspace;
}

cv::Point path_find_next_valid_position(cv::Mat _workspace,
                                        cv::Point _current_point,
                                        bool _debug = NODEBUG)
// Get a point, and determine based on a 3x3 mask what point is best to go to
// next
{
  _workspace = cv_color_pixel(_workspace, _current_point, red_pixel);
  cv::Point cp = _current_point;
  // row 1
  cv::Point cp1 = {cp.x - 1, cp.y - 1};
  cv::Point cp2 = {cp.x + 0, cp.y - 1};
  cv::Point cp3 = {cp.x + 1, cp.y - 1};
  // row 2
  cv::Point cp4 = {cp.x - 1, cp.y};
  cv::Point cp5 = {cp.x + 0, cp.y};
  cv::Point cp6 = {cp.x + 1, cp.y};
  // row 3
  cv::Point cp7 = {cp.x - 1, cp.y + 1};
  cv::Point cp8 = {cp.x + 0, cp.y + 1};
  cv::Point cp9 = {cp.x + 1, cp.y + 1};

  //  cv::Point[] mask_positions[] = {cp1, cp2, cp3, cp4, cp5, cp6, cp7, cp8,
  //  cp9};
  return _current_point;
}

cv::Mat cv_find_valid_path(cv::Mat _workspace, cv::Point _start,
                           cv::Point _goal, bool _debug = NODEBUG) {
  if (_debug) {
    cout << "start: " << _start << endl << "goal: " << _goal << endl;
  }
  cv::Point current_position = _start;
  while (true) {
    current_position =
        path_find_next_valid_position(_workspace, current_position);
  }
  return _workspace;
}
/*******************************************************************************
 * COMPUTER VISION
 *******************************************************************************/
void cv_add_gaussian_noise(cv::Mat &_input_image, double _averege = 0.0,
                           double _standard_deviation = 10.0)
// inputs a Image refrence, and adds Gaussian noise, either user defined
// or with chosen averege and standard deviation
{
  cv::Mat noise_only_image(_input_image.size(), CV_16SC3);
  cv::randn(noise_only_image, cv::Scalar::all(_averege),
            cv::Scalar::all(_standard_deviation));
  cv::Mat temporary_image;
  _input_image.convertTo(temporary_image, CV_16SC3);
  cv::addWeighted(temporary_image, 1.0, noise_only_image, 1.0, 0.0,
                  temporary_image);
  temporary_image.convertTo(_input_image, _input_image.type());
  return;
}

double cv_peak_snr(const cv::Mat &I1, const cv::Mat &I2) {
  cv::Mat s1;
  cv::absdiff(I1, I2, s1);  // |I1 - I2|
  s1.convertTo(s1, CV_32F); // cannot make a square on 8 bits
  s1 = s1.mul(s1);          // |I1 - I2|^2

  cv::Scalar s = cv::sum(s1); // sum elements per channel

  double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels

  if (sse <= 1e-10) // for small values return zero
    return 0;
  else {
    double mse = sse / (double)(I1.channels() * I1.total());
    double psnr = 10.0 * log10((255 * 255) / mse);
    return psnr;
  }
}

void draw_Histogram(Mat &img, std::vector<int> histogramData, int globalMaxY,
                    int R, int G, int B) {
  int image_size = img.cols;
  float bin_pixel_width = ((float)image_size) / histogramData.size();

  for (int i = 0; i < histogramData.size() - 1; i++) {
    line(img,
         Point(i * bin_pixel_width,
               image_size - (histogramData[i] * image_size) / globalMaxY),
         Point((i + 1) * bin_pixel_width,
               image_size - (histogramData[i + 1] * image_size) / globalMaxY),
         CV_RGB(R, G, B), 1, LINE_AA);
  }
}
