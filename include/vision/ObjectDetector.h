#ifndef OBJECTDETECTOR_H
#define OBJECTDETECTOR_H

#include <ros/ros.h>
#include "ColorObject.h"

class ObjectDetector
{
public:
  /**
   * @brief Construct a new Object Detector object
   * 
   */
  explicit ObjectDetector();

  /**
   * @brief Destroy the Object Detector object
   * 
   */
  ~ObjectDetector();

  
  /**
   * @brief Set the Text
   * 
   * @param color_object The detected object
   * @param drawing_frame The window in which the text should be set
   */
  void setText(std::shared_ptr<ColorObject>& color_object, cv::Mat& drawing_frame);
  
  /**
   * @brief Set the Center Point
   * 
   * @param color_object The detected object
   * @param contour The contour data for the detected object
   */
  void setCenterPoint(const std::shared_ptr<ColorObject>& color_object, std::vector<cv::Point>& contour);
  
  /**
   * @brief 
   * 
   * @param color_object 
   * @param drawing_frame 
   */
  void findShape(std::shared_ptr<ColorObject>& color_object, cv::Mat& drawing_frame);
  
  /**
   * @brief 
   * 
   * @param color_object The detected shape
   * @param filtered_frame The window on which the filtered frame should be displayed
   * @param color_mask 
   */
  void filterColor(const std::shared_ptr<ColorObject>& color_object, cv::Mat& filtered_frame, cv::Mat& color_mask);
  
  /**
   * @brief Function to draw the contours of the detected shape
   * 
   * @param contour The contour data of the detected shape
   * @param color_object The detected shape
   * @param contour_number 
   * @param drawing_frame The window in which the contour should be drawn
   */
  void DrawImageContours(const std::vector<std::vector<cv::Point>>& contour,
                         const std::shared_ptr<ColorObject>& color_object, const int contour_number,
                         cv::Mat& drawing_frame);
  
  /**
   * @brief 
   * 
   * @param frame 
   * @param clip_hist_percent 
   * @return cv::Mat 
   */
  cv::Mat BrightnessAndContrastAuto(const cv::Mat& frame, double clip_hist_percent);
  
  /**
   * @brief Function to check whether the detected shape is a square or rectangle
   * 
   * @param color_object The detected shape
   * @param approx Vecor containing the contour data
   * @return true 
   * @return false 
   */
  bool checkSquareAndRectangle(std::shared_ptr<ColorObject>& color_object, std::vector<cv::Point>& approx);
  
  /**
   * @brief Function to check whether the detected shape is a circle
   * 
   * @param color_object The deteccted shape
   * @param contours Vector containing the contour data
   * @param element 
   * @return true 
   * @return false 
   */
  bool checkCircle(std::shared_ptr<ColorObject>& color_object, std::vector<std::vector<cv::Point>>& contours,
                   int element);
  
  /**
   * @brief Function to check whether the detected shape is a semicircle
   * 
   * @param color_object The detected shape 
   * @param contours Vector containing the contour data
   * @param element 
   * @return true 
   * @return false 
   */
  bool semiCircle(std::shared_ptr<ColorObject>& color_object, std::vector<std::vector<cv::Point>>& contours,
                  int element);
  
  /**
   * @brief 
   * 
   * @param filtered_frame The matrix containing the filtered frame
   */
  void filterFrame(cv::Mat& filtered_frame);
};

#endif 
