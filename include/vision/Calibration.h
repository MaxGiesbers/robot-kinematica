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
   * @param color 
   * @return ColorScale 
   */
  ColorScale getColorScale(std::string color) const;

private:
  /**
   * @brief 
   * 
   * @param frame 
   * @param clip_hist_percent 
   * @return cv::Mat 
   */
  cv::Mat BrightnessAndContrastAuto(const cv::Mat& frame, double clip_hist_percent);
  
  /**
   * @brief 
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
   * @brief 
   * 
   * @param v 
   * @param ptr 
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
   * @brief 
   * 
   */
  std::size_t m_iterator;

  /**
   * @brief 
   * 
   */
  cv::Mat m_treshold;
  
  /**
   * @brief Matrix for the source image
   * 
   */
  cv::Mat m_src;
  
  /**
   * @brief 
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
