#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <ros/ros.h>
#include "vision/ColorScale.h"
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>

class Calibration
{
public:
  /**
   * @brief Construct a new Calibration object
   * 
   */
  Calibration();
  
  /**
   * @brief Destroy the Calibration object
   * 
   */
  ~Calibration();
  
  /**
   * @brief Starts the calibration software
   * 
   * @param cap Camera to be opened
   */
  void startCalibration(const cv::VideoCapture& cap);
  
  /**
   * @brief Get the Color Scale object
   * 
   * @param color in string format
   * @return ColorScale struct with HSV value of the color
   */
  ColorScale getColorScale(std::string color) const;

private:
  /**
   * @brief enhances the contrast color of the mat object.
   * 
   * @param frame the frame that has to be copied.
   * @param clipHistPercent percentage of the filter
   * @return cv::Mat 
   */ 
  cv::Mat BrightnessAndContrastAuto(const cv::Mat& frame, double clip_hist_percent);
  
  /**
   * @brief updates the range for the trackbar
   * 
   */
  void updateSetRange();
  
  /**
   * @brief Set the Default Color Scales object
   * 
   */
  void setDefaultColorScales();
  
  /**
   * @brief Set the Color Values object
   * 
   */
  void setColorValues();

  /**
   * @brief Set the Calibrated Color Values object
   * 
   */
  void setCalibratedColorValues();

  /**
   * @brief function used as callback by the trackbar sliders from openCV
   * 
   * @param v 
   * @param ptr this instance of the object
   */
  static void calibrate(int v, void* ptr)
  {
    v += 1;  // To prevent wExtra complains

    Calibration* calibration = static_cast<Calibration*>(ptr);
    calibration->updateSetRange();
  }

  /**
   * @brief Declaring variables for the values of the corresponding sliders
   * 
   */
  int m_iLowH, m_iHighH, m_iLowS, m_iHighS, m_iLowV, m_iHighV;
  
  /**
   * @brief Creating a slider window
   * 
   */
  std::string sliderWindow = "CALIBRATE";

  
  /**
   * @brief iterator that iterates through the color scale objects.
   * 
   */
  std::size_t m_color_scale_iterator;

  /**
   * @brief Matrix for the treshold image
   * 
   */
  cv::Mat m_treshold;
  
  /**
   * @brief Matrix for the source image
   * 
   */
  cv::Mat m_src;
  
  /**
   * @brief vector that contains color scales
   * 
   */
  std::vector<ColorScale> m_color_scales;

  /**
   * @brief The camera that is used
   * 
   */
  cv::VideoCapture m_cap;
  
  /**
   * @brief Function that creates the window with the camera feed
   * 
   */
  cv::Mat m_capture_window;
};

#endif  
