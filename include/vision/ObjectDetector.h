#ifndef OBJECTDETECTOR_H
#define OBJECTDETECTOR_H

#include <ros/ros.h>
#include "ColorObject.h"

class ObjectDetector
{
public:
  explicit ObjectDetector();
  ~ObjectDetector();

  void setText(std::shared_ptr<ColorObject>& color_object, cv::Mat& drawing_frame);
  void setCenterPoint(const std::shared_ptr<ColorObject>& color_object, std::vector<cv::Point>& contour);
  void findShape(std::shared_ptr<ColorObject>& color_object, cv::Mat& drawing_frame);
  void filterColor(const std::shared_ptr<ColorObject>& color_object, cv::Mat& filtered_frame, cv::Mat& color_mask);
  void DrawImageContours(const std::vector<std::vector<cv::Point>>& contour,
                         const std::shared_ptr<ColorObject>& color_object, const int contour_number,
                         cv::Mat& drawing_frame);
  cv::Mat BrightnessAndContrastAuto(const cv::Mat& frame, double clip_hist_percent);
  bool checkSquareAndRectangle(std::shared_ptr<ColorObject>& color_object, std::vector<cv::Point>& approx);
  bool checkCircle(std::shared_ptr<ColorObject>& color_object, std::vector<std::vector<cv::Point>>& contours,
                   int element);
  bool semiCircle(std::shared_ptr<ColorObject>& color_object, std::vector<std::vector<cv::Point>>& contours,
                  int element);
  void filterFrame(cv::Mat& filtered_frame);
};

#endif 
